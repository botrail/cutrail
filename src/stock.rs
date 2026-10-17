//! Initial stock shapes.

use crate::error::{Error, Result};
use crate::geom::Aabb;
use crate::mesh::TriMesh;
use std::f64::consts::PI;

/// Stock primitives.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum StockShape {
    /// Axis-aligned box
    Box {
        /// Minimum corner
        min: [f64; 3],
        /// Maximum corner
        max: [f64; 3],
    },
    /// Cylinder with its axis along Z
    Cylinder {
        /// XY position of the axis
        center: [f64; 2],
        /// Radius
        radius: f64,
        /// Bottom Z
        z0: f64,
        /// Top Z
        z1: f64,
    },
    /// A closed triangle mesh (stock read from STL or STEP). Vertices are merged on import
    Mesh(TriMesh),
}

impl StockShape {
    /// Validates the shape.
    pub fn validate(&self) -> Result<()> {
        match self {
            StockShape::Box { min, max } => {
                if (0..3).any(|i| max[i] <= min[i] || !min[i].is_finite() || !max[i].is_finite()) {
                    return Err(Error::InvalidInput(format!(
                        "box must have positive extent: {min:?}..{max:?}"
                    )));
                }
            }
            StockShape::Cylinder { radius, z0, z1, .. } => {
                if *radius <= 0.0 || *z1 <= *z0 || !radius.is_finite() || !z0.is_finite() || !z1.is_finite() {
                    return Err(Error::InvalidInput(
                        "cylinder must have positive radius and height".into(),
                    ));
                }
            }
            StockShape::Mesh(mesh) => {
                if mesh.triangles.len() < 4 {
                    return Err(Error::InvalidInput("stock mesh needs at least 4 triangles".into()));
                }
                let n = mesh.positions.len() as u32;
                if mesh.triangles.iter().any(|t| t.iter().any(|&i| i >= n)) {
                    return Err(Error::InvalidInput("stock mesh has out-of-range vertex indices".into()));
                }
                if mesh.positions.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
                    return Err(Error::InvalidInput("stock mesh has non-finite vertices".into()));
                }
            }
        }
        Ok(())
    }

    /// Bounding box.
    pub fn bounds(&self) -> Aabb {
        match self {
            StockShape::Box { min, max } => Aabb { min: *min, max: *max },
            StockShape::Cylinder { center, radius, z0, z1 } => Aabb {
                min: [center[0] - radius, center[1] - radius, *z0],
                max: [center[0] + radius, center[1] + radius, *z1],
            },
            StockShape::Mesh(mesh) => mesh.bounds(),
        }
    }

    /// Analytic volume.
    pub fn volume(&self) -> f64 {
        match self {
            StockShape::Box { .. } => self.bounds().volume(),
            StockShape::Cylinder { radius, z0, z1, .. } => PI * radius * radius * (z1 - z0),
            StockShape::Mesh(mesh) => mesh.signed_volume().abs(),
        }
    }
}
