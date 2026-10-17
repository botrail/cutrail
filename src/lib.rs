//! cutrail: a material removal simulation core for machining.
//!
//! The stock is held as a triangle mesh (Manifold) and the volume swept by the tool is subtracted segment by segment.
//! The tool axis can point in any direction; segments with a rotating axis (5-axis), arc and helical moves,
//! ball and bull-nose end mills, and shank/holder collision detection are supported.
//! The public API is built from the crate's own types ([`Tool`], [`Pose`], [`MotionSegment`], [`StockShape`]);
//! the kernel's types stay behind the [`Kernel`] trait.
//!
//! ```
//! use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};
//!
//! let stock = StockShape::Box { min: [0.0, 0.0, 0.0], max: [10.0, 10.0, 10.0] };
//! let mut engine = Engine::new(ManifoldKernel, &[stock], EngineOptions::default()).unwrap();
//! let tool = Tool::flat_end_mill(4.0, 6.0);
//! let seg = MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0]));
//! let report = engine.cut(&tool, &seg).unwrap();
//! assert_eq!(report.outcome, cutrail::CutOutcome::Updated);
//! assert!((engine.total_volume() - 840.0).abs() < 0.5);
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod brep;
pub mod compare;
pub mod engine;
pub mod error;
pub mod geom;
pub mod kernel;
pub mod mesh;
pub mod motion;
#[cfg(feature = "stepio")]
pub mod step;
pub mod stock;
pub mod stream;
pub mod sweep;
pub mod tool;

pub use brep::{Brep, Source};
pub use compare::DesignComparison;
pub use engine::{BatchError, Collision, CutOutcome, CutReport, EngineOptions, MachiningEngine, PartId, PartInfo};
pub use stream::SegmentBuilder;
pub use error::{Error, Result};
pub use geom::{Aabb, Point3, Vector3};
pub use kernel::manifold::ManifoldKernel;
pub use kernel::{Kernel, SolidSummary};
pub use mesh::TriMesh;
pub use motion::{slerp, ArcGeometry, MotionSegment, Pose};
pub use stock::StockShape;
pub use sweep::SweptVolume;
pub use tool::{Cutter, Holder, Shank, Tool};

/// Engine using the default kernel (manifold-rust).
pub type Engine = MachiningEngine<ManifoldKernel>;
