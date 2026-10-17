//! WebAssembly bindings for cutrail, for Web Workers and Node.
//!
//! Segments are passed as a flat array of 12 f64 per segment (start tip xyz, start axis xyz, end tip xyz,
//! end axis xyz). Meshes are returned per cell (part or tile) as positions (f32) and indices (u32).
#![forbid(unsafe_code)]

use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, PartId, Pose, StockShape, Tool};
use wasm_bindgen::prelude::*;

/// Machining engine.
#[wasm_bindgen]
pub struct WasmEngine {
    inner: Engine,
    tool: Tool,
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
        })
    }

    /// Switches the tool to a flat end mill.
    pub fn set_flat_end_mill(&mut self, diameter: f64, flute_length: f64) {
        self.tool = Tool::flat_end_mill(diameter, flute_length);
    }

    /// Switches the tool to a ball end mill.
    pub fn set_ball_end_mill(&mut self, diameter: f64, flute_length: f64) {
        self.tool = Tool::ball_end_mill(diameter, flute_length);
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
            let report = self
                .inner
                .cut(&self.tool, &seg)
                .map_err(|e| JsError::new(&format!("segment {i}: {e}")))?;
            removed += report.removed_volume;
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
}
