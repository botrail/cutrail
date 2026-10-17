//! Abstraction boundary for the geometry kernel.

pub mod manifold;

use crate::error::Result;
use crate::geom::Aabb;
use crate::mesh::TriMesh;
use crate::stock::StockShape;
use crate::sweep::SweptVolume;

/// Summary of a solid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolidSummary {
    /// Bounding box
    pub bounds: Aabb,
    /// Signed volume (positive for outward orientation)
    pub signed_volume: f64,
    /// Triangle count
    pub triangles: usize,
}

impl SolidSummary {
    /// Absolute volume.
    pub fn volume(&self) -> f64 {
        self.signed_volume.abs()
    }
}

/// Geometry kernel: builds stock and swept volumes, subtracts, validates and meshes.
pub trait Kernel: Send + Sync {
    /// The kernel's solid type. Cloning may be a deep copy (it is done rarely).
    type Solid: Clone + Send + Sync;

    /// Kernel name.
    fn name(&self) -> &'static str;

    /// Builds a solid from a stock primitive.
    fn make_stock(&self, shape: &StockShape, chord_tolerance: f64) -> Result<Self::Solid>;

    /// Builds a solid from a closed triangle mesh, merging vertices within `merge_tolerance`.
    fn make_mesh(&self, mesh: &TriMesh, merge_tolerance: f64) -> Result<Self::Solid>;

    /// Dilates the solid by `distance` (erodes if negative). Used for toleranced comparison against a design model.
    fn offset(&self, solid: &Self::Solid, distance: f64, chord_tolerance: f64) -> Result<Self::Solid>;

    /// Builds a solid from a swept volume.
    fn make_swept(&self, volume: &SweptVolume) -> Result<Self::Solid>;

    /// Builds a swept volume as one solid per convex piece. The default builds a single solid.
    fn make_swept_pieces(&self, volume: &SweptVolume) -> Result<Vec<Self::Solid>> {
        Ok(vec![self.make_swept(volume)?])
    }

    /// `a \ b`. Returns `Err` on kernel failure (never a silent `None`).
    fn difference(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid>;

    /// `a ∩ b`. Used to cut the stock into tiles.
    fn intersection(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid>;

    /// Union of all solids. Used to merge stock primitives, to build hull unions, and to assemble cells for reconstruction and comparison.
    fn union_all(&self, solids: &[Self::Solid]) -> Result<Self::Solid>;

    /// Absolute volume.
    fn volume(&self, solid: &Self::Solid) -> f64;

    /// Triangle count.
    fn triangle_count(&self, solid: &Self::Solid) -> usize;

    /// Returns the solid simplified within the tolerance.
    fn simplify(&self, solid: &Self::Solid, tolerance: f64) -> Result<Self::Solid>;

    /// Whether the solid is empty.
    fn is_empty(&self, solid: &Self::Solid) -> bool;

    /// Splits into connected components. Internal voids come back as separate components with negative signed volume.
    fn components(&self, solid: &Self::Solid) -> Vec<Self::Solid>;

    /// Composes several solids into one (used to re-attach voids to their outer shell).
    fn compose(&self, parts: &[Self::Solid]) -> Result<Self::Solid>;

    /// Mesh for display and output.
    fn mesh(&self, solid: &Self::Solid) -> TriMesh;

    /// Summary; computed from the mesh by default.
    fn summary(&self, solid: &Self::Solid) -> SolidSummary {
        let mesh = self.mesh(solid);
        SolidSummary {
            bounds: mesh.bounds(),
            signed_volume: mesh.signed_volume(),
            triangles: mesh.triangle_count(),
        }
    }

    /// Validates the solid.
    fn validate(&self, solid: &Self::Solid) -> Result<()>;

    /// Gives the solid a fresh provenance ID. Triangles of boolean results keep their provenance IDs.
    fn with_fresh_id(&self, solid: &Self::Solid) -> (Self::Solid, u32);

    /// Display mesh together with the provenance ID of each triangle.
    fn mesh_with_sources(&self, solid: &Self::Solid) -> (TriMesh, Vec<u32>);
}
