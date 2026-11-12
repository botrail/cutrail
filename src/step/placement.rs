//! STEP instances, using SolidCore's matrices and row-vector convention throughout.

use std::collections::{BTreeMap, BTreeSet};

use solidcore::geom::step_matrix::{concat, frame_matrix, inverse, operator_matrix, Matrix4, IDENTITY};
use solidcore_record::step::product::{
    Frame, ItemKind, Link, ProductStructure, Transformation, MAX_ASSEMBLY_DEPTH, MAX_VISITS,
};
use solidcore_step::{p21::Exchange, topology};

use super::{Error, Result};

pub(super) fn identity() -> Matrix4 {
    IDENTITY
}

pub(super) struct Placed {
    pub entity: u64,
    pub transform: Matrix4,
    pub path: Vec<u64>,
}

fn invalid(message: impl std::fmt::Display) -> Error {
    Error::InvalidInput(format!("STEP placement: {message}"))
}

pub(super) fn frame(value: &Frame) -> Result<Matrix4> {
    let matrix = match value {
        Frame::Placement(p) => frame_matrix(p.origin, p.axis, p.ref_direction),
        Frame::Operator(o) => {
            // STEP's base_axis derives omitted axes and projects supplied directions into
            // an orthonormal basis. Independent global defaults can create a shear (or a
            // singular matrix) for a perfectly valid tilted operator.
            let x = solidcore::Vec3::from(o.axis1.unwrap_or([1., 0., 0.]));
            let y = solidcore::Vec3::from(o.axis2.unwrap_or([0., 1., 0.]));
            let z = o.axis3.unwrap_or_else(|| {
                let z = x.cross(y);
                [z.x, z.y, z.z]
            });
            let mut axes =
                frame_matrix(o.origin, Some(z), o.axis1).ok_or_else(|| invalid("degenerate operator basis"))?;
            if let Some(hint) = o.axis2 {
                let dot = solidcore::Vec3::from(hint).dot(solidcore::Vec3::new(axes[1][0], axes[1][1], axes[1][2]));
                if !dot.is_finite() || dot.abs() <= 1e-12 * solidcore::Vec3::from(hint).length() {
                    return Err(invalid("operator axis2 has no independent direction"));
                }
                if dot < 0.0 {
                    axes[1].iter_mut().take(3).for_each(|x| *x = -*x);
                }
            }
            operator_matrix(
                Some([axes[0][0], axes[0][1], axes[0][2]]),
                Some([axes[1][0], axes[1][1], axes[1][2]]),
                Some([axes[2][0], axes[2][1], axes[2][2]]),
                o.origin,
                o.scale,
                o.scale2,
                o.scale3,
            )
        }
    }
    .ok_or_else(|| invalid("degenerate frame or transformation operator"))?;
    mesh_scale(&matrix)?;
    Ok(matrix)
}

/// A finite regular affine map, its orientation and an upper bound on length scaling.
/// sqrt(||A||1 ||A||inf) >= ||A||2, so dividing the local chord tolerance by it
/// also covers rotations, mirrors and non-uniform scales in world coordinates.
pub(super) fn mesh_scale(matrix: &Matrix4) -> Result<(f64, bool)> {
    if matrix.iter().flatten().any(|x| !x.is_finite())
        || matrix[0][3] != 0.0
        || matrix[1][3] != 0.0
        || matrix[2][3] != 0.0
        || matrix[3][3] != 1.0
        || inverse(matrix).is_none()
    {
        return Err(invalid("non-finite, singular or non-affine matrix"));
    }
    let rows = (0..3)
        .map(|i| (0..3).map(|j| matrix[i][j].abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let cols = (0..3)
        .map(|j| (0..3).map(|i| matrix[i][j].abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let scale = rows.sqrt() * cols.sqrt();
    let a = &matrix[0];
    let b = &matrix[1];
    let c = &matrix[2];
    let det =
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    if !scale.is_finite() || scale <= 0.0 || !det.is_finite() || det == 0.0 {
        return Err(invalid("matrix has no finite non-zero scale or determinant"));
    }
    Ok((scale, det < 0.0))
}

fn inverted(matrix: &Matrix4) -> Result<Matrix4> {
    inverse(matrix).ok_or_else(|| invalid("singular placement"))
}

/// Coordinates in rep_1 mapped into rep_2. Frame origins were already converted to mm
/// by the record reader; dimensionless operator scales must not be converted again.
fn link_matrix(link: &Link) -> Result<Matrix4> {
    match &link.transformation {
        None => Ok(IDENTITY),
        Some(Transformation::Items { item_1, item_2, .. }) => Ok(concat(&inverted(&frame(item_1)?)?, &frame(item_2)?)),
        Some(Transformation::Frame(value)) => frame(value),
    }
}

fn same_matrix(a: &Matrix4, b: &Matrix4) -> bool {
    a.iter()
        .flatten()
        .zip(b.iter().flatten())
        .all(|(a, b)| (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0))
}

struct Walk<'a> {
    exchange: &'a Exchange,
    structure: &'a ProductStructure,
    occurrence_links: BTreeSet<usize>,
    reached: BTreeSet<u64>,
    visited_maps: BTreeSet<u64>,
    visits: usize,
    out: Vec<Placed>,
}

impl Walk<'_> {
    fn visit(&mut self, depth: usize) -> Result<()> {
        self.visits += 1;
        if depth > MAX_ASSEMBLY_DEPTH || self.visits > MAX_VISITS {
            return Err(invalid(
                "assembly/mapped representation expansion exceeds its depth or visit limit",
            ));
        }
        Ok(())
    }

    fn closure(&self, shapes: &[usize]) -> Result<BTreeMap<usize, Matrix4>> {
        let mut todo = shapes.iter().map(|&rep| (rep, IDENTITY)).collect::<Vec<_>>();
        let mut seen = BTreeMap::new();
        while let Some((rep, world)) = todo.pop() {
            if let Some(prior) = seen.get(&rep) {
                if !same_matrix(prior, &world) {
                    return Err(invalid("conflicting definition representation frames"));
                }
                continue;
            }
            mesh_scale(&world)?;
            seen.insert(rep, world);
            for (i, link) in self.structure.links.iter().enumerate() {
                if self.occurrence_links.contains(&i) {
                    continue;
                }
                if link.rep_1 == rep {
                    todo.push((link.rep_2, concat(&inverted(&link_matrix(link)?)?, &world)));
                } else if link.rep_2 == rep {
                    todo.push((link.rep_1, concat(&link_matrix(link)?, &world)));
                }
            }
        }
        Ok(seen)
    }

    fn occurrence(&self, index: usize) -> Result<Matrix4> {
        let o = &self.structure.occurrences[index];
        let child = self.closure(&self.structure.definitions[o.child].shapes)?;
        let parent = self.closure(&self.structure.definitions[o.parent].shapes)?;
        let mut placed = None;
        for &i in &o.placements {
            let link = self
                .structure
                .links
                .get(i)
                .ok_or_else(|| invalid("missing occurrence link"))?;
            let forward = child.contains_key(&link.rep_1) && parent.contains_key(&link.rep_2);
            let backward = child.contains_key(&link.rep_2) && parent.contains_key(&link.rep_1);
            if forward == backward {
                return Err(invalid(format!(
                    "ambiguous representation direction for occurrence #{}",
                    o.entity
                )));
            }
            let matrix = link_matrix(link)?;
            let (from, to, matrix) = if forward {
                (&child[&link.rep_1], &parent[&link.rep_2], matrix)
            } else {
                (&child[&link.rep_2], &parent[&link.rep_1], inverted(&matrix)?)
            };
            let matrix = concat(&concat(&inverted(from)?, &matrix), to);
            if placed.as_ref().is_some_and(|p| !same_matrix(p, &matrix)) {
                return Err(invalid(format!("conflicting placements for occurrence #{}", o.entity)));
            }
            placed = Some(matrix);
        }
        // Absence is not evidence that a component's local frame is the parent's frame.
        placed.ok_or_else(|| invalid(format!("occurrence #{} has no placement relationship", o.entity)))
    }

    fn representation(
        &mut self,
        rep_index: usize,
        world: Matrix4,
        path: &[u64],
        seen: &mut BTreeMap<usize, Matrix4>,
        active_maps: &mut Vec<usize>,
        depth: usize,
    ) -> Result<()> {
        self.visit(depth)?;
        mesh_scale(&world)?;
        if let Some(prior) = seen.get(&rep_index) {
            return if same_matrix(prior, &world) {
                Ok(())
            } else {
                Err(invalid("inconsistent representation placement cycle"))
            };
        }
        seen.insert(rep_index, world);
        let rep = self
            .structure
            .representations
            .get(rep_index)
            .ok_or_else(|| invalid("missing representation"))?;
        for item in &rep.items {
            match &item.kind {
                ItemKind::Body(_) => {
                    for entity in topology::items(self.exchange, item.entity) {
                        self.reached.insert(entity);
                        if self.out.len() >= MAX_VISITS {
                            return Err(invalid("too many placed bodies"));
                        }
                        self.out.push(Placed {
                            entity,
                            transform: world,
                            path: path.to_vec(),
                        });
                    }
                }
                ItemKind::Mapped { map, target } => {
                    if active_maps.contains(map) {
                        return Err(invalid("mapped representation cycle"));
                    }
                    let map_record = self
                        .structure
                        .maps
                        .get(*map)
                        .ok_or_else(|| invalid("missing representation map"))?;
                    // A point first leaves the map's origin frame, then enters the target,
                    // then the containing representation's world frame (row vectors).
                    let local = concat(&inverted(&frame(&map_record.origin)?)?, &frame(target)?);
                    let placed = concat(&local, &world);
                    let mut path = path.to_vec();
                    path.push(item.entity);
                    self.visited_maps.insert(item.entity);
                    active_maps.push(*map);
                    self.representation(
                        map_record.representation,
                        placed,
                        &path,
                        &mut BTreeMap::new(),
                        active_maps,
                        depth + 1,
                    )?;
                    active_maps.pop();
                }
                ItemKind::Other(_) => {}
            }
        }
        for (i, link) in self.structure.links.iter().enumerate() {
            if self.occurrence_links.contains(&i) {
                continue;
            }
            let (other, local) = if link.rep_1 == rep_index {
                (link.rep_2, inverted(&link_matrix(link)?)?)
            } else if link.rep_2 == rep_index {
                (link.rep_1, link_matrix(link)?)
            } else {
                continue;
            };
            self.representation(other, concat(&local, &world), path, seen, active_maps, depth + 1)?;
        }
        Ok(())
    }
}

pub(super) fn bodies(exchange: &Exchange, structure: &ProductStructure, body_ids: &[u64]) -> Result<Vec<Placed>> {
    let mut walk = Walk {
        exchange,
        structure,
        occurrence_links: structure
            .occurrences
            .iter()
            .flat_map(|o| o.placements.iter().copied())
            .collect(),
        reached: BTreeSet::new(),
        visited_maps: BTreeSet::new(),
        visits: 0,
        out: Vec::new(),
    };
    let instances = structure.instances().map_err(invalid)?;
    let mut worlds = Vec::<Matrix4>::new();
    let mut paths = Vec::<Vec<u64>>::new();
    for instance in instances {
        let definition = &structure.definitions[instance.definition];
        let (world, path) = if let (Some(parent), Some(occurrence)) = (instance.parent, instance.occurrence) {
            let mut path = paths[parent].clone();
            path.push(structure.occurrences[occurrence].entity);
            (concat(&walk.occurrence(occurrence)?, &worlds[parent]), path)
        } else {
            (identity(), vec![definition.entity])
        };
        let mut seen = BTreeMap::new();
        for &rep in &definition.shapes {
            walk.representation(rep, world, &path, &mut seen, &mut Vec::new(), path.len() - 1)?;
        }
        worlds.push(world);
        paths.push(path);
    }
    // Geometry-only STEP files need no product definitions. Start only representations
    // that are not map sources; a shared source is emitted once per actual mapped item.
    if structure.definitions.is_empty() {
        let sources = structure.maps.iter().map(|m| m.representation).collect::<BTreeSet<_>>();
        let mut seen = BTreeMap::new();
        for (i, rep) in structure.representations.iter().enumerate() {
            if !sources.contains(&i) && !seen.contains_key(&i) {
                walk.representation(i, IDENTITY, &[rep.entity], &mut seen, &mut Vec::new(), 0)?;
            }
        }
    }
    // Unreferenced mapped items cannot be interpreted as an identity placement.
    for (&id, item) in &exchange.entities {
        if item.has("MAPPED_ITEM") && !walk.visited_maps.contains(&id) {
            return Err(invalid(format!("mapped item #{id} is not reachable from a root")));
        }
    }
    let placed = !structure.occurrences.is_empty()
        || !structure.maps.is_empty()
        || structure.links.iter().any(|l| l.transformation.is_some());
    for &id in body_ids {
        if !walk.reached.contains(&id) {
            if placed {
                return Err(invalid(format!("body #{id} has no reachable placement")));
            }
            walk.out.push(Placed {
                entity: id,
                transform: IDENTITY,
                path: Vec::new(),
            });
        }
    }
    if !placed {
        walk.out.sort_by_key(|b| b.entity);
    }
    Ok(walk.out)
}
