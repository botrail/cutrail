//! WebAssembly bindings for cutrail, for Web Workers and Node.
//!
//! Segments are passed as a flat array of 12 f64 per segment (start tip xyz, start axis xyz, end tip xyz,
//! end axis xyz), or sampled poses as 6 f64 per pose (tip xyz, axis xyz) that the engine coalesces into
//! segments. Meshes are returned per cell (part or tile) as positions (f32), indices (u32) and the kind of
//! every triangle (stock surface or cut surface).
#![forbid(unsafe_code)]

use cutrail::{
    Engine, EngineOptions, FaceKind, ManifoldKernel, MotionSegment, PartId, Pose, SegmentBuilder, StockShape, Tool,
};
use wasm_bindgen::prelude::*;

/// Machining engine.
#[wasm_bindgen]
pub struct WasmEngine {
    inner: Engine,
    tool: Tool,
    builder: SegmentBuilder,
    segments: u64,
}

/// The mesh of one cell with the kind of every triangle: 0 for the original stock surface, 1 for a surface
/// the cutter made. Call `free()` once the arrays have been read.
#[wasm_bindgen]
pub struct CellMesh {
    positions: Vec<f32>,
    indices: Vec<u32>,
    kinds: Vec<u8>,
}

#[wasm_bindgen]
impl CellMesh {
    /// Vertex positions as flat xyz.
    #[wasm_bindgen(getter)]
    pub fn positions(&self) -> Vec<f32> {
        self.positions.clone()
    }

    /// Triangle vertex indices.
    #[wasm_bindgen(getter)]
    pub fn indices(&self) -> Vec<u32> {
        self.indices.clone()
    }

    /// One byte per triangle: 0 = stock surface, 1 = cut surface.
    #[wasm_bindgen(getter)]
    pub fn kinds(&self) -> Vec<u8> {
        self.kinds.clone()
    }
}

fn js_err(e: impl std::fmt::Display) -> JsError {
    JsError::new(&e.to_string())
}

#[wasm_bindgen]
impl WasmEngine {
    /// Creates an engine with a box stock from `[min_x, min_y, min_z]` to `[max_x, max_y, max_z]` and a flat end mill.
    /// A `tile_size` of 0 or less disables tiling.
    #[wasm_bindgen(constructor)]
    pub fn new(
        min: &[f64],
        max: &[f64],
        tool_diameter: f64,
        flute_length: f64,
        tile_size: f64,
        sweep_tolerance: f64,
    ) -> Result<WasmEngine, JsError> {
        if min.len() != 3 || max.len() != 3 {
            return Err(JsError::new("min and max must have 3 components"));
        }
        let mut options = EngineOptions {
            sweep_tolerance,
            ..Default::default()
        };
        if tile_size > 0.0 {
            options.tile_size = Some(tile_size);
        }
        let stock = StockShape::Box {
            min: [min[0], min[1], min[2]],
            max: [max[0], max[1], max[2]],
        };
        let inner = Engine::new(ManifoldKernel, &[stock], options).map_err(js_err)?;
        Ok(WasmEngine {
            inner,
            tool: Tool::flat_end_mill(tool_diameter, flute_length),
            builder: SegmentBuilder::new(5.0, 2.0).with_max_deviation(0.01),
            segments: 0,
        })
    }

    fn cut_segment(&mut self, label: usize, segment: &MotionSegment) -> Result<f64, JsError> {
        let report = self
            .inner
            .cut(&self.tool, segment)
            .map_err(|e| JsError::new(&format!("segment {label}: {e}")))?;
        self.segments += 1;
        Ok(report.removed_volume)
    }

    /// Switches the tool to a flat end mill.
    pub fn set_flat_end_mill(&mut self, diameter: f64, flute_length: f64) {
        self.tool = Tool::flat_end_mill(diameter, flute_length);
    }

    /// Switches the tool to a ball end mill.
    pub fn set_ball_end_mill(&mut self, diameter: f64, flute_length: f64) {
        self.tool = Tool::ball_end_mill(diameter, flute_length);
    }

    /// Switches the tool to a bull-nose end mill.
    pub fn set_bull_nose_end_mill(&mut self, diameter: f64, corner_radius: f64, flute_length: f64) {
        self.tool = Tool::bull_nose_end_mill(diameter, corner_radius, flute_length);
    }

    /// Thresholds for [`Self::push_poses`]: a segment is emitted after `min_distance` (mm) or
    /// `min_angle_degrees` of motion, or before a sample would stray more than `max_deviation` (mm) from
    /// the chord. Resets the stream.
    pub fn set_stream(&mut self, min_distance: f64, min_angle_degrees: f64, max_deviation: f64) {
        self.builder = SegmentBuilder::new(min_distance, min_angle_degrees).with_max_deviation(max_deviation);
    }

    /// Pushes sampled poses (6 numbers each: tip xyz, axis xyz), cutting every segment the stream emits.
    /// Returns the removed volume. Call [`Self::flush`] at the end of the stream.
    pub fn push_poses(&mut self, poses: &[f64]) -> Result<f64, JsError> {
        if !poses.len().is_multiple_of(6) {
            return Err(JsError::new("poses must be a multiple of 6 numbers"));
        }
        let mut removed = 0.0;
        for (i, p) in poses.chunks(6).enumerate() {
            let pose = Pose::new([p[0], p[1], p[2]], [p[3], p[4], p[5]]);
            if let Some(segment) = self.builder.push(pose) {
                removed += self.cut_segment(i, &segment)?;
            }
        }
        Ok(removed)
    }

    /// Cuts the motion the stream still holds below its thresholds. Returns the removed volume.
    pub fn flush(&mut self) -> Result<f64, JsError> {
        match self.builder.flush() {
            Some(segment) => self.cut_segment(0, &segment),
            None => Ok(0.0),
        }
    }

    /// Number of segments cut so far.
    pub fn segment_count(&self) -> f64 {
        self.segments as f64
    }

    /// Cuts the segments in order and returns the total removed volume. Segments before a failure are committed.
    pub fn cut_many(&mut self, segments: &[f64]) -> Result<f64, JsError> {
        if !segments.len().is_multiple_of(12) {
            return Err(JsError::new("segments must be a multiple of 12 numbers"));
        }
        let mut removed = 0.0;
        for (i, s) in segments.chunks(12).enumerate() {
            let seg = MotionSegment::linear(
                Pose::new([s[0], s[1], s[2]], [s[3], s[4], s[5]]),
                Pose::new([s[6], s[7], s[8]], [s[9], s[10], s[11]]),
            );
            removed += self.cut_segment(i, &seg)?;
        }
        Ok(removed)
    }

    /// Current revision.
    pub fn revision(&self) -> f64 {
        self.inner.revision() as f64
    }

    /// Total remaining volume (mm³).
    pub fn total_volume(&self) -> f64 {
        self.inner.total_volume()
    }

    /// Total triangle count.
    pub fn total_triangles(&self) -> f64 {
        self.inner.total_triangles() as f64
    }

    /// Identifiers of all cells.
    pub fn part_ids(&self) -> Vec<u32> {
        self.inner.parts().iter().map(|p| p.id.0).collect()
    }

    /// Identifiers of cells changed after `revision`.
    pub fn changed_since(&self, revision: f64) -> Vec<u32> {
        self.inner
            .changed_since(revision as u64)
            .iter()
            .map(|id| id.0)
            .collect()
    }

    /// Vertex positions of a cell as flat xyz f32; empty if the cell does not exist.
    pub fn mesh_positions(&self, id: u32) -> Vec<f32> {
        self.inner
            .mesh(PartId(id))
            .map(|m| m.positions.iter().flat_map(|p| p.iter().map(|&v| v as f32)).collect())
            .unwrap_or_default()
    }

    /// Triangle indices of a cell as u32; empty if the cell does not exist.
    pub fn mesh_indices(&self, id: u32) -> Vec<u32> {
        self.inner
            .mesh(PartId(id))
            .map(|m| m.triangles.iter().flat_map(|t| t.iter().copied()).collect())
            .unwrap_or_default()
    }

    /// The mesh of a cell with the kind of every triangle, or `undefined` if the cell does not exist.
    pub fn mesh_with_kinds(&self, id: u32) -> Option<CellMesh> {
        let (mesh, kinds) = self.inner.mesh_with_kinds(PartId(id))?;
        Some(CellMesh {
            positions: mesh
                .positions
                .iter()
                .flat_map(|p| p.iter().map(|&v| v as f32))
                .collect(),
            indices: mesh.triangles.iter().flat_map(|t| t.iter().copied()).collect(),
            kinds: kinds.iter().map(|k| if *k == FaceKind::Cut { 1 } else { 0 }).collect(),
        })
    }
}
