//! Kernel implementation on manifold-rust (a pure-Rust port of Manifold C++ v3.5.0).

use super::Kernel;
use crate::error::{Error, Result};
use crate::mesh::TriMesh;
use crate::stock::StockShape;
use crate::sweep::{area_equivalent_radius, circle_segments, SweptVolume};
use manifold_rust::linalg::Vec3;
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as ManifoldStatus, MeshGL64, OpType};

/// Requests no normals (a negative value, as in the C++ API).
const NO_NORMALS: i32 = -1;

/// Kernel backed by manifold-rust.
#[derive(Clone, Copy, Debug, Default)]
pub struct ManifoldKernel;

fn hull_of(points: &[[f64; 3]]) -> Manifold {
    let pts: Vec<Vec3> = points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    Manifold::hull(&pts)
}

fn checked(m: Manifold, what: &str) -> Result<Manifold> {
    match m.status() {
        ManifoldStatus::NoError => Ok(m),
        status => Err(Error::Kernel(format!("{what}: {status:?}"))),
    }
}

impl Kernel for ManifoldKernel {
    type Solid = Manifold;

    fn name(&self) -> &'static str {
        "manifold-rust"
    }

    fn make_stock(&self, shape: &StockShape, chord_tolerance: f64) -> Result<Manifold> {
        shape.validate()?;
        let m = match shape {
            StockShape::Box { min, max } => {
                Manifold::cube(Vec3::new(max[0] - min[0], max[1] - min[1], max[2] - min[2]), false)
                    .translate(Vec3::new(min[0], min[1], min[2]))
            }
            StockShape::Cylinder { center, radius, z0, z1 } => {
                let n = circle_segments(*radius, chord_tolerance);
                let r = area_equivalent_radius(*radius, n);
                Manifold::cylinder(z1 - z0, r, r, n as i32).translate(Vec3::new(center[0], center[1], *z0))
            }
            StockShape::Mesh(mesh) => return self.make_mesh(mesh, chord_tolerance * 0.01),
        };
        checked(m, "stock construction")
    }

    fn make_mesh(&self, mesh: &TriMesh, merge_tolerance: f64) -> Result<Manifold> {
        // A mesh whose vertices already match exactly (an OBJ, a well-formed STL, our own output) must not be
        // merged at the coarse tolerance: fine features would collapse. Try the exact weld first, then the
        // requested tolerance for meshes with sloppy vertices.
        let mut last = None;
        for tolerance in [1e-7_f64.min(merge_tolerance), merge_tolerance] {
            let mut merged = mesh.clone();
            merged.merge_vertices(tolerance);
            let gl = MeshGL64 {
                num_prop: 3,
                vert_properties: merged.positions.iter().flat_map(|p| p.iter().copied()).collect(),
                tri_verts: merged
                    .triangles
                    .iter()
                    .flat_map(|t| t.iter().map(|&i| i as u64))
                    .collect(),
                ..Default::default()
            };
            let m = Manifold::from_mesh_gl64(&gl);
            if matches!(m.status(), ManifoldStatus::NoError) && !m.is_empty() {
                return Ok(m);
            }
            last = Some(m);
            if tolerance >= merge_tolerance {
                break;
            }
        }
        let m = last.expect("at least one attempt");
        match m.status() {
            ManifoldStatus::NoError => {
                if m.is_empty() {
                    Err(Error::InvalidInput("mesh is empty".into()))
                } else {
                    Ok(m)
                }
            }
            status => Err(Error::InvalidInput(format!(
                "mesh is not a closed manifold: {status:?}"
            ))),
        }
    }

    fn offset(&self, solid: &Manifold, distance: f64, chord_tolerance: f64) -> Result<Manifold> {
        if distance == 0.0 {
            return Ok(solid.clone());
        }
        let r = distance.abs();
        let n = circle_segments(r, chord_tolerance.max(r * 0.05)).clamp(8, 32) as i32;
        let ball = Manifold::sphere(r, n);
        let m = if distance > 0.0 {
            solid.minkowski_sum(&ball)
        } else {
            solid.minkowski_difference(&ball)
        };
        checked(m, "offset")
    }

    fn make_swept(&self, volume: &SweptVolume) -> Result<Manifold> {
        let m = match volume {
            SweptVolume::ConvexHull(points) => hull_of(points),
            SweptVolume::HullUnion(hulls) => {
                let solids: Vec<Manifold> = hulls.iter().map(|h| hull_of(h)).collect();
                if solids
                    .iter()
                    .any(|m| m.status() != ManifoldStatus::NoError || m.is_empty())
                {
                    return Err(Error::Kernel("swept volume sub-step hull is invalid".into()));
                }
                self.union_all(&solids)?
            }
            SweptVolume::Mesh(mesh) => {
                let gl = MeshGL64 {
                    num_prop: 3,
                    vert_properties: mesh.positions.iter().flat_map(|p| p.iter().copied()).collect(),
                    tri_verts: mesh
                        .triangles
                        .iter()
                        .flat_map(|t| t.iter().map(|&i| i as u64))
                        .collect(),
                    ..Default::default()
                };
                Manifold::from_mesh_gl64(&gl)
            }
        };
        let m = checked(m, "swept volume construction")?;
        if m.is_empty() {
            return Err(Error::Kernel("swept volume is empty".into()));
        }
        Ok(m)
    }

    fn make_swept_pieces(&self, volume: &SweptVolume) -> Result<Vec<Manifold>> {
        match volume {
            SweptVolume::HullUnion(hulls) => hulls
                .iter()
                .map(|h| {
                    let m = checked(hull_of(h), "swept volume piece")?;
                    if m.is_empty() {
                        return Err(Error::Kernel("swept volume piece is empty".into()));
                    }
                    Ok(m)
                })
                .collect(),
            other => Ok(vec![self.make_swept(other)?]),
        }
    }

    fn difference(&self, a: &Manifold, b: &Manifold) -> Result<Manifold> {
        checked(a.difference(b), "difference")
    }

    fn intersection(&self, a: &Manifold, b: &Manifold) -> Result<Manifold> {
        checked(a.intersection(b), "intersection")
    }

    fn union_all(&self, solids: &[Manifold]) -> Result<Manifold> {
        match solids {
            [] => Err(Error::InvalidInput("union of nothing".into())),
            [one] => Ok(one.clone()),
            many => checked(Manifold::batch_boolean(many, OpType::Add), "union"),
        }
    }

    fn volume(&self, solid: &Manifold) -> f64 {
        solid.volume()
    }

    fn triangle_count(&self, solid: &Manifold) -> usize {
        solid.num_tri()
    }

    fn simplify(&self, solid: &Manifold, tolerance: f64) -> Result<Manifold> {
        checked(solid.simplify(tolerance), "simplify")
    }

    fn is_empty(&self, solid: &Manifold) -> bool {
        solid.is_empty()
    }

    fn components(&self, solid: &Manifold) -> Vec<Manifold> {
        solid.decompose()
    }

    fn compose(&self, parts: &[Manifold]) -> Result<Manifold> {
        checked(Manifold::compose(parts), "compose")
    }

    fn mesh(&self, solid: &Manifold) -> TriMesh {
        let gl = solid.get_mesh_gl64(NO_NORMALS);
        let stride = (gl.num_prop as usize).max(3);
        let positions = gl.vert_properties.chunks(stride).map(|c| [c[0], c[1], c[2]]).collect();
        let triangles = gl
            .tri_verts
            .chunks(3)
            .map(|c| [c[0] as u32, c[1] as u32, c[2] as u32])
            .collect();
        TriMesh { positions, triangles }
    }

    fn validate(&self, solid: &Manifold) -> Result<()> {
        match solid.status() {
            ManifoldStatus::NoError => Ok(()),
            status => Err(Error::Validation(format!("manifold status {status:?}"))),
        }
    }

    fn with_fresh_id(&self, solid: &Manifold) -> (Manifold, u32) {
        let fresh = solid.as_original();
        let id = fresh.original_id();
        (fresh, id.max(0) as u32)
    }

    fn mesh_with_sources(&self, solid: &Manifold) -> (TriMesh, Vec<u32>) {
        let gl = solid.get_mesh_gl64(NO_NORMALS);
        let stride = (gl.num_prop as usize).max(3);
        let positions = gl.vert_properties.chunks(stride).map(|c| [c[0], c[1], c[2]]).collect();
        let triangles: Vec<[u32; 3]> = gl
            .tri_verts
            .chunks(3)
            .map(|c| [c[0] as u32, c[1] as u32, c[2] as u32])
            .collect();
        let mut sources = vec![0u32; triangles.len()];
        let runs = gl.run_index.len().saturating_sub(1);
        for r in 0..runs {
            let start = (gl.run_index[r] as usize) / 3;
            let end = (gl.run_index[r + 1] as usize) / 3;
            let id = gl.run_original_id.get(r).copied().unwrap_or(0);
            for slot in sources.iter_mut().take(end.min(triangles.len())).skip(start) {
                *slot = id;
            }
        }
        (TriMesh { positions, triangles }, sources)
    }
}
