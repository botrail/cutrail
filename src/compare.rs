//! Comparison against a design model: the difference between the machined stock and the design as volumes and region meshes.

use crate::geom::Aabb;
use crate::mesh::TriMesh;

/// Result of comparing against a design model.
///
/// `excess` is material left outside the design (undercut); `gouge` is the region removed inside the design (overcut).
/// With a `tolerance`, the differences are taken against the design offset outward and inward by that distance,
/// so deviations within the tolerance are ignored.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DesignComparison {
    /// Tolerance used (mm)
    pub tolerance: f64,
    /// Volume of excess material (mm³)
    pub excess_volume: f64,
    /// Volume of gouges (mm³)
    pub gouge_volume: f64,
    /// Region of excess material (no triangles if none)
    pub excess: TriMesh,
    /// Region of gouges (no triangles if none)
    pub gouge: TriMesh,
    /// Bounding box of the excess material
    pub excess_bounds: Option<Aabb>,
    /// Bounding box of the gouges
    pub gouge_bounds: Option<Aabb>,
}

impl DesignComparison {
    /// Whether there is neither excess material nor a gouge.
    pub fn is_within_tolerance(&self) -> bool {
        self.excess_volume <= 0.0 && self.gouge_volume <= 0.0
    }
}
