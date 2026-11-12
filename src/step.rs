//! STEP import and retained CAD models (feature `stepio`).
//!
//! SolidCore reads analytic/NURBS B-reps. [`StepModel`] keeps those records independently of
//! tessellation; its meshes feed the existing Manifold machining engine. Machining does not
//! update the retained input B-reps. Placed assemblies and mapped instances are rejected.

use crate::error::{Error, Result};
use crate::kernel::{manifold::ManifoldKernel, Kernel};
use crate::mesh::TriMesh;
use crate::stock::StockShape;
use solidcore_ops::pcurve::{self, EdgeUse, TrimFace, Trims};
use solidcore_record::step::product;
use solidcore_step::{p21, topology};
use solidcore_tess::{MeshTolerance, PlanStats};

/// SolidCore's portable CAD records, including analytic/NURBS surfaces and topology.
///
/// This API follows the SolidCore revision pinned in Cargo.toml.
pub use solidcore_record::brep as cad;

/// A STEP body's original CAD geometry, in millimetres, before tessellation and cutting.
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

    /// The retained analytic/NURBS geometry and topology. Tessellation leaves it unchanged.
    pub fn brep(&self) -> &cad::Brep {
        &self.brep
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

    /// Tessellates the retained geometry at `chord_tolerance` millimetres.
    ///
    /// Missing faces, lenient fills and unmet refinement tolerances are errors. Solid bodies
    /// must produce a closed manifold mesh; sheets may produce an open display mesh. The
    /// tolerance controls tessellation, not the uncertainty or fitted trims of the input CAD.
    pub fn mesh(&self, chord_tolerance: f64) -> Result<TriMesh> {
        validate_tolerance(chord_tolerance)?;
        self.brep
            .validate()
            .map_err(|e| Error::InvalidInput(format!("STEP body #{} record: {e}", self.entity)))?;
        // A zero size disables the planner's size-dependent tolerance floor. It must either
        // meet the requested chord tolerance or report a refinement cap below.
        let (plan, stats) = solidcore_tess::plan(&self.brep, &MeshTolerance::chord(chord_tolerance), 0.0)
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
        if self.sheet {
            Ok(mesh)
        } else {
            // Reuse the stock importer's fine-first welding and manifold validation. Return
            // the validated mesh so stock_from_step does not apply a coarser second weld.
            let solid = ManifoldKernel
                .make_mesh(&mesh, chord_tolerance * 0.01)
                .map_err(|e| Error::Validation(format!("STEP body #{} mesh: {e}", self.entity)))?;
            let volume = ManifoldKernel.volume(&solid);
            if !volume.is_finite() || volume <= 0.0 {
                return Err(Error::Validation(format!(
                    "STEP body #{} mesh has non-positive or non-finite solid volume",
                    self.entity
                )));
            }
            Ok(ManifoldKernel.mesh(&solid))
        }
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
    /// Bodies in source entity-id order, including their CAD records and reader diagnostics.
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

/// Supplies missing pcurves through SolidCore's evaluator, preserving the reader's given
/// trims and parameterisation. No projection or tolerance adjustment is implemented here.
struct KernelTrims;

impl Trims for KernelTrims {
    fn trims(&self, face: &TrimFace<'_>) -> std::result::Result<Vec<Vec<cad::Curve2>>, String> {
        let loops = face
            .loops
            .iter()
            .map(|lp| {
                lp.iter()
                    .map(|u| {
                        let e = face
                            .edges
                            .get(u.edge as usize)
                            .ok_or("pcurve edge index out of range")?;
                        Ok(EdgeUse {
                            edge: u.edge,
                            curve: e.nurbs.unwrap_or(e.curve),
                            interval: e.interval,
                            forward: u.forward,
                            tolerance: e.tolerance,
                            given: u.pcurve.as_ref(),
                        })
                    })
                    .collect::<std::result::Result<Vec<_>, String>>()
            })
            .collect::<std::result::Result<Vec<_>, String>>()?;
        pcurve::pcurves_within(
            face.nurbs.unwrap_or(face.surface),
            &loops,
            face.tolerance,
            Some(face.domain),
        )
        .map_err(|e| e.to_string())
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
/// Flat multi-body models and bodies with voids are supported. Assembly occurrences, mapped
/// instances and transformed representation relationships are rejected until their world
/// placements are supported, rather than silently importing parts at the wrong positions.
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
    for representation in &structure.representations {
        if !representation.kind.ends_with("SHAPE_REPRESENTATION") {
            continue;
        }
        for item in &representation.items {
            match &item.kind {
                product::ItemKind::Other(kind)
                    if !matches!(
                        kind.as_str(),
                        "AXIS2_PLACEMENT_3D" | "AXIS1_PLACEMENT" | "CARTESIAN_POINT"
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
    if !structure.occurrences.is_empty()
        || !structure.maps.is_empty()
        || structure.links.iter().any(|link| link.transformation.is_some())
        || exchange.entities.values().any(|e| e.has("MAPPED_ITEM"))
    {
        return Err(Error::Unsupported(
            "STEP assembly and mapped-instance placements are not supported; import a flattened part model".into(),
        ));
    }
    let read = topology::read_solids_with(&exchange, &KernelTrims);
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
    let bodies = read
        .solids
        .into_iter()
        .map(|body| {
            body.brep
                .validate()
                .map_err(|e| Error::InvalidInput(format!("STEP body #{} record: {e}", body.item)))?;
            Ok(StepBody {
                entity: body.item,
                name: body.name,
                product: body.product,
                brep: body.brep,
                face_entities: body.face_origin,
                notes: body.notes,
                sheet: body.sheet,
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
