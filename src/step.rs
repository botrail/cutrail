//! STEP import and retained CAD models (feature `stepio`).
//!
//! SolidCore reads analytic/NURBS B-reps. [`StepModel`] keeps those records independently of
//! tessellation; its meshes feed the existing Manifold machining engine. Machining does not
//! update the retained input B-reps. Instances keep local CAD and a world placement matrix.

use crate::error::{Error, Result};
use crate::kernel::{manifold::ManifoldKernel, Kernel};
use crate::mesh::TriMesh;
use crate::stock::StockShape;
use solidcore::geom::step_matrix::Matrix4;
use std::collections::BTreeMap;

mod placement;
mod trims;
use solidcore_record::step::product;
use solidcore_step::{p21, topology};
use solidcore_tess::{MeshTolerance, PlanStats};
use trims::KernelTrims;

/// SolidCore's portable CAD records, including analytic/NURBS surfaces and topology.
///
/// This API follows the SolidCore revision pinned in Cargo.toml.
pub use solidcore_record::brep as cad;

/// One instance of a STEP body's local CAD geometry (mm) and its world placement.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StepBody {
    entity: u64,
    name: String,
    product: Option<String>,
    brep: cad::Brep,
    face_entities: Vec<u64>,
    notes: Vec<String>,
    sheet: bool,
    #[cfg_attr(feature = "serde", serde(default = "placement::identity"))]
    transform: Matrix4,
    #[cfg_attr(feature = "serde", serde(default))]
    instance_path: Vec<u64>,
}

impl StepBody {
    /// The source STEP body entity id.
    pub fn entity(&self) -> u64 {
        self.entity
    }

    /// The body's name, or its product's name when the body has none.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The product name, when the file supplies one.
    pub fn product(&self) -> Option<&str> {
        self.product.as_deref()
    }

    /// The retained local analytic/NURBS geometry and topology, in millimetres.
    /// Apply [`Self::transform`] to place it in world coordinates. Tessellation leaves it unchanged.
    pub fn brep(&self) -> &cad::Brep {
        &self.brep
    }

    /// Local CAD to world coordinates, in millimetres. Rows 0–2 are the images of X/Y/Z;
    /// row 3 is translation: `world[j] = sum(local[i] * matrix[i][j]) + matrix[3][j]`.
    pub fn transform(&self) -> &Matrix4 {
        &self.transform
    }

    /// Root product-definition (or geometry-only representation) entity, followed by
    /// occurrence and mapped-item entity ids.
    /// Together with [`Self::entity`], identifies one use of shared source CAD.
    /// An unreferenced body in a flat file has an empty path.
    pub fn instance_path(&self) -> &[u64] {
        &self.instance_path
    }

    /// Source STEP face entity ids, indexed in the retained B-rep's face order.
    pub fn face_entities(&self) -> &[u64] {
        &self.face_entities
    }

    /// Repairs and tolerance adjustments reported by the STEP reader.
    pub fn notes(&self) -> &[String] {
        &self.notes
    }

    /// Whether this body contains only open shells, rather than a solid.
    pub fn is_sheet(&self) -> bool {
        self.sheet
    }

    /// Tessellates the retained geometry in world coordinates at `chord_tolerance` millimetres.
    ///
    /// Missing faces, lenient fills and unmet refinement tolerances are errors. Solid bodies
    /// must produce a closed manifold mesh; sheets may produce an open display mesh. The
    /// tolerance controls tessellation, not the uncertainty or fitted trims of the input CAD.
    pub fn mesh(&self, chord_tolerance: f64) -> Result<TriMesh> {
        validate_tolerance(chord_tolerance)?;
        let (scale, mirrored) = placement::mesh_scale(&self.transform)?;
        let local_tolerance = chord_tolerance / scale;
        validate_tolerance(local_tolerance)?;
        self.brep
            .validate()
            .map_err(|e| Error::InvalidInput(format!("STEP body #{} record: {e}", self.entity)))?;
        // A zero size disables the planner's size-dependent tolerance floor. It must either
        // meet the requested chord tolerance or report a refinement cap below.
        let (plan, stats) = solidcore_tess::plan(&self.brep, &MeshTolerance::chord(local_tolerance), 0.0)
            .map_err(|e| Error::Kernel(format!("STEP body #{} tessellation: {e}", self.entity)))?;
        check_plan(self.entity, &stats)?;
        let (mesh, refused) = plan.evaluate();
        if let Some(face) = refused.first() {
            return Err(Error::Validation(format!(
                "STEP body #{} face {} evaluation: {}",
                self.entity, face.face, face.reason
            )));
        }
        let mut seen = vec![false; self.brep.faces.len()];
        for &face in &mesh.tri_face {
            let slot = seen.get_mut(face as usize).ok_or_else(|| {
                Error::Validation(format!("STEP body #{} mesh has an invalid face index", self.entity))
            })?;
            *slot = true;
        }
        if mesh.triangles.len() != mesh.tri_face.len() || seen.is_empty() || seen.contains(&false) {
            return Err(Error::Validation(format!(
                "STEP body #{} tessellation omitted a CAD face",
                self.entity
            )));
        }
        let mesh = TriMesh {
            positions: mesh.positions.iter().map(|p| [p.x, p.y, p.z]).collect(),
            triangles: mesh.triangles,
        };
        if mesh.positions.iter().flatten().any(|p| !p.is_finite())
            || mesh
                .triangles
                .iter()
                .flatten()
                .any(|&i| i as usize >= mesh.positions.len())
        {
            return Err(Error::Validation(format!(
                "STEP body #{} tessellation has non-finite vertices or invalid triangle indices",
                self.entity
            )));
        }
        let mut mesh = if self.sheet {
            mesh
        } else {
            // Reuse the stock importer's fine-first welding and manifold validation. Return
            // the validated mesh so stock_from_step does not apply a coarser second weld.
            let solid = ManifoldKernel
                .make_mesh(&mesh, local_tolerance * 0.01)
                .map_err(|e| Error::Validation(format!("STEP body #{} mesh: {e}", self.entity)))?;
            let volume = ManifoldKernel.volume(&solid);
            if !volume.is_finite() || volume <= 0.0 {
                return Err(Error::Validation(format!(
                    "STEP body #{} mesh has non-positive or non-finite solid volume",
                    self.entity
                )));
            }
            ManifoldKernel.mesh(&solid)
        };
        for p in &mut mesh.positions {
            *p = [0, 1, 2].map(|j| {
                p[0] * self.transform[0][j]
                    + p[1] * self.transform[1][j]
                    + p[2] * self.transform[2][j]
                    + self.transform[3][j]
            });
        }
        if mirrored {
            mesh.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
        if mesh.positions.iter().flatten().any(|x| !x.is_finite()) {
            return Err(Error::Validation(format!(
                "STEP body #{} world coordinates overflow",
                self.entity
            )));
        }
        if !self.sheet && (!mesh.signed_volume().is_finite() || mesh.signed_volume() <= 0.0) {
            return Err(Error::Validation(format!(
                "STEP body #{} placement has invalid mesh volume",
                self.entity
            )));
        }
        Ok(mesh)
    }
}

/// All CAD bodies in a STEP file, retained independently of the simulation's stock meshes.
///
/// A body with inner void shells stays one body. Unsupported or unreadable bodies cause the
/// entire import to fail; an incomplete stock is never returned as a successful import.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StepModel {
    bodies: Vec<StepBody>,
}

impl StepModel {
    /// Placed body instances in product-tree order (entity order for a flat model).
    /// Shared CAD appears once per instance, with its own transform and path.
    pub fn bodies(&self) -> &[StepBody] {
        &self.bodies
    }

    /// Tessellates each body without modifying the retained CAD geometry.
    pub fn meshes(&self, chord_tolerance: f64) -> Result<Vec<TriMesh>> {
        validate_tolerance(chord_tolerance)?;
        if self.bodies.is_empty() {
            return Err(Error::InvalidInput("STEP model has no bodies".into()));
        }
        self.bodies.iter().map(|b| b.mesh(chord_tolerance)).collect()
    }

    /// Creates the simulation stock while keeping the original CAD model available.
    ///
    /// Open sheets cannot be machining stock. Each solid, including all its void shells,
    /// becomes one [`StockShape::Mesh`].
    pub fn stock_shapes(&self, chord_tolerance: f64) -> Result<Vec<StockShape>> {
        if let Some(body) = self.bodies.iter().find(|b| b.is_sheet()) {
            return Err(Error::InvalidInput(format!(
                "STEP body #{} is an open sheet, not solid stock",
                body.entity
            )));
        }
        Ok(self
            .meshes(chord_tolerance)?
            .into_iter()
            .map(StockShape::Mesh)
            .collect())
    }
}

fn validate_tolerance(tolerance: f64) -> Result<()> {
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return Err(Error::InvalidInput(
            "STEP chord tolerance must be finite and positive".into(),
        ));
    }
    Ok(())
}

fn check_plan(entity: u64, stats: &PlanStats) -> Result<()> {
    if let Some(face) = stats.unmeshed.first().or(stats.lenient.first()) {
        return Err(Error::Validation(format!(
            "STEP body #{entity} face {} tessellation: {}",
            face.face, face.reason
        )));
    }
    if let Some(edge) = stats.unsampled_edges.first() {
        return Err(Error::Validation(format!(
            "STEP body #{entity} edge {} sampling: {}",
            edge.edge, edge.reason
        )));
    }
    if let Some(face) = stats.over_tolerance.iter().find(|f| !f.within) {
        return Err(Error::Validation(format!(
            "STEP body #{entity} face {} exceeded chord tolerance {} (sag {})",
            face.face, face.tol, face.sag
        )));
    }
    if let Some(edge) = stats.edges_capped.iter().find(|e| !e.within) {
        return Err(Error::Validation(format!(
            "STEP body #{entity} edge {} exceeded chord tolerance {} (sag {})",
            edge.edge, edge.tol, edge.sag
        )));
    }
    Ok(())
}

/// Reads a STEP string into retained analytic/NURBS CAD bodies, in millimetres.
///
/// Supports flat models, bodies with voids, placed assembly occurrences, mapped instances
/// and transformed representation relationships. CAD stays local; meshes are world-placed.
/// Invalid or ambiguous placements reject the complete import.
pub fn model_from_step_str(step: &str) -> Result<StepModel> {
    let exchange = p21::parse(step).map_err(|e| Error::InvalidInput(format!("STEP parse error: {e}")))?;
    // The upstream topology reader visits its four supported body kinds only. Refuse other
    // solid representations explicitly so a mixed B-rep/CSG file cannot lose its CSG stock.
    for (id, entity) in &exchange.entities {
        let body = entity.records.iter().any(|r| topology::is_solid_item(&r.name));
        let unknown_solid = entity
            .records
            .iter()
            .any(|r| r.name.ends_with("_SOLID") && !topology::is_solid_item(&r.name));
        // The reader dispatches by the first record. A complex combination such as faceted
        // B-rep with voids must not be read as just one of its components, losing the voids.
        if unknown_solid || (body && entity.is_complex()) {
            return Err(Error::Unsupported(format!(
                "STEP body #{id} representation {} ({} records) is not supported",
                entity.name(),
                entity.records.len()
            )));
        }
    }
    let structure =
        product::read(&exchange).map_err(|e| Error::InvalidInput(format!("STEP product structure: {e}")))?;
    let mut contexts = BTreeMap::new();
    for representation in &structure.representations {
        for item in &representation.items {
            match &item.kind {
                product::ItemKind::Body(_) => {
                    if contexts
                        .insert(item.entity, representation.units)
                        .is_some_and(|prior| prior != representation.units)
                    {
                        return Err(Error::Unsupported(format!(
                            "STEP body #{} is shared by representations with conflicting units or uncertainty",
                            item.entity
                        )));
                    }
                }
                product::ItemKind::Other(kind)
                    if !matches!(
                        kind.as_str(),
                        "AXIS2_PLACEMENT_3D" | "AXIS2_PLACEMENT_2D" | "AXIS1_PLACEMENT" | "CARTESIAN_POINT"
                    ) =>
                {
                    return Err(Error::Unsupported(format!(
                        "STEP shape item #{} representation {kind} is not supported",
                        item.entity
                    )));
                }
                _ => {}
            }
        }
    }
    let trims = KernelTrims::default();
    let read = topology::read_solids_with(&exchange, &trims);
    if let Some(failure) = read.failures.first() {
        return Err(Error::InvalidInput(format!(
            "STEP body #{} conversion error: {} ({} unreadable bodies)",
            failure.item,
            failure.message,
            read.failures.len()
        )));
    }
    if read.solids.is_empty() {
        return Err(Error::InvalidInput("STEP file has no supported bodies".into()));
    }
    let sources = read
        .solids
        .into_iter()
        .map(|mut body| {
            body.brep
                .validate()
                .map_err(|e| Error::InvalidInput(format!("STEP body #{} record: {e}", body.item)))?;
            body.notes.extend(trims.notes_for(&body.brep));
            Ok((body.item, body))
        })
        .collect::<Result<BTreeMap<_, _>>>()?;
    let placed = placement::bodies(&exchange, &structure, &sources.keys().copied().collect::<Vec<_>>())?;
    let bodies = placed
        .into_iter()
        .map(|instance| {
            let body = sources
                .get(&instance.entity)
                .ok_or_else(|| Error::Validation(format!("STEP placed body #{} was not read", instance.entity)))?;
            Ok(StepBody {
                entity: body.item,
                name: body.name.clone(),
                product: body.product.clone(),
                brep: body.brep.clone(),
                face_entities: body.face_origin.clone(),
                notes: body.notes.clone(),
                sheet: body.sheet,
                transform: instance.transform,
                instance_path: instance.path,
            })
        })
        .collect::<Result<_>>()?;
    Ok(StepModel { bodies })
}

/// Reads a STEP file and retains its CAD bodies before tessellation.
pub fn model_from_step_file<P: AsRef<std::path::Path>>(path: P) -> Result<StepModel> {
    model_from_step_str(&std::fs::read_to_string(path)?)
}

/// Tessellates every STEP body. Solid bodies produce validated, welded manifold meshes.
pub fn meshes_from_step_str(step: &str, chord_tolerance: f64) -> Result<Vec<TriMesh>> {
    validate_tolerance(chord_tolerance)?;
    model_from_step_str(step)?.meshes(chord_tolerance)
}

/// Tessellates every body in a STEP file.
pub fn meshes_from_step_file<P: AsRef<std::path::Path>>(path: P, chord_tolerance: f64) -> Result<Vec<TriMesh>> {
    validate_tolerance(chord_tolerance)?;
    model_from_step_file(path)?.meshes(chord_tolerance)
}

/// Returns one stock mesh per solid STEP body, keeping void shells with their outer shell.
/// Use [`model_from_step_str`] and [`StepModel::stock_shapes`] to retain the source CAD too.
pub fn stock_from_step_str(step: &str, chord_tolerance: f64) -> Result<Vec<StockShape>> {
    validate_tolerance(chord_tolerance)?;
    model_from_step_str(step)?.stock_shapes(chord_tolerance)
}

/// Returns every solid in a STEP file as stock; open sheets are rejected.
pub fn stock_from_step_file<P: AsRef<std::path::Path>>(path: P, chord_tolerance: f64) -> Result<Vec<StockShape>> {
    validate_tolerance(chord_tolerance)?;
    model_from_step_file(path)?.stock_shapes(chord_tolerance)
}
