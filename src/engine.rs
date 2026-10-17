//! Machining engine: stock state (parts or tiles), per-segment subtraction, validation and commit, history.

use crate::error::{Error, Result};
use crate::geom::Aabb;
use crate::kernel::Kernel;
use crate::mesh::TriMesh;
use crate::motion::MotionSegment;
use crate::stock::StockShape;
use crate::sweep;
use crate::tool::Tool;
use std::collections::HashMap;
use std::io::{self, Write};
use std::sync::OnceLock;
use crate::brep::{self, Brep, Source};
use crate::compare::DesignComparison;
use crate::tool::Cutter;
use std::time::Duration;
use web_time::Instant;

/// Identifier of a cell. In part mode, pieces created by a split get new numbers; in tiled mode it is the tile index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PartId(pub u32);

/// Engine options.
#[derive(Clone, Debug, PartialEq)]
pub struct EngineOptions {
    /// Chord tolerance (mm) for approximating the arcs of swept volumes
    pub sweep_tolerance: f64,
    /// Chord tolerance (mm) for approximating the arcs of stock primitives
    pub stock_tolerance: f64,
    /// A removal below this fraction of the cell volume is treated as `NoChange`
    pub volume_epsilon: f64,
    /// Edge length of the cubic tiles the stock is split into; `None` keeps the stock as parts
    pub tile_size: Option<f64>,
    /// Simplify a cell after a cut once its triangle count exceeds this; `None` disables simplification
    pub simplify_threshold: Option<usize>,
    /// Simplification tolerance (mm)
    pub simplify_tolerance: f64,
    /// Check shank and holder against the stock; adds an intersection per cut
    pub collision_check: bool,
    /// Intersections above this volume (mm³) are reported as collisions
    pub collision_volume_epsilon: f64,
}

impl Default for EngineOptions {
    fn default() -> Self {
        EngineOptions {
            sweep_tolerance: 0.01,
            stock_tolerance: 0.01,
            volume_epsilon: 1e-9,
            tile_size: None,
            simplify_threshold: None,
            simplify_tolerance: 1e-3,
            collision_check: false,
            collision_volume_epsilon: 1e-6,
        }
    }
}

impl EngineOptions {
    /// Enables tiling.
    pub fn with_tile_size(mut self, size: f64) -> Self {
        self.tile_size = Some(size);
        self
    }

    /// Enables collision detection.
    pub fn with_collision_check(mut self) -> Self {
        self.collision_check = true;
        self
    }
}

/// Collision between shank/holder and the stock, checked after the cutting part has been subtracted.
#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    /// Intersecting volume (mm³)
    pub volume: f64,
    /// Bounding box of the intersection
    pub bounds: Aabb,
}

/// Outcome of one cut.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutOutcome {
    /// The tool removed nothing; the state is unchanged
    NoChange,
    /// The stock was updated
    Updated,
    /// All stock has been removed
    Exhausted,
}

/// Report of one cut (a segment or a batch of segments).
#[derive(Clone, Debug, PartialEq)]
pub struct CutReport {
    /// Outcome
    pub outcome: CutOutcome,
    /// Revision after the cut (unchanged for `NoChange`)
    pub revision: u64,
    /// Changed region (union of the intersections of the swept volume's bounds with the cells' bounds)
    pub changed_bounds: Option<Aabb>,
    /// Processing time
    pub elapsed: Duration,
    /// Removed volume (mm³), the volume difference before and after
    pub removed_volume: f64,
    /// Number of cells before the cut
    pub parts_before: usize,
    /// Number of cells after the cut
    pub parts_after: usize,
    /// Number of cells on which a boolean ran
    pub cells_touched: usize,
    /// Shank/holder collision (when `collision_check` is enabled and an intersection was found)
    pub collision: Option<Collision>,
    /// Diagnostics (splits, exhausted cells, and so on)
    pub diagnostics: Vec<String>,
}

/// Public information about a cell.
#[derive(Clone, Debug, PartialEq)]
pub struct PartInfo {
    /// Identifier
    pub id: PartId,
    /// Bounding box (the tile region in tiled mode)
    pub bounds: Aabb,
    /// Volume (mm³)
    pub volume: f64,
    /// Triangle count
    pub triangles: usize,
    /// Revision of the last change
    pub revision: u64,
}

/// Error from `cut_many`. Segments before the failing one have been committed.
#[derive(Debug)]
pub struct BatchError {
    /// Index of the failing segment
    pub index: usize,
    /// Reports of the segments committed before the failure
    pub completed: Vec<CutReport>,
    /// The error
    pub error: Error,
}

impl std::fmt::Display for BatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "segment {} failed after {} completed: {}", self.index, self.completed.len(), self.error)
    }
}

impl std::error::Error for BatchError {}

/// Swept volumes of a sequence of segments (a solid and bounds per convex piece).
struct SweepPlan<S> {
    cutting: Vec<(S, Aabb)>,
    non_cutting: Vec<(S, Aabb)>,
    bounds: Aabb,
    pieces: usize,
}

struct Cell<S> {
    id: PartId,
    bounds: Aabb,
    solid: S,
    volume: f64,
    triangles: usize,
    revision: u64,
    mesh: OnceLock<TriMesh>,
}

impl<S> Cell<S> {
    fn new(id: PartId, bounds: Aabb, solid: S, volume: f64, triangles: usize, revision: u64) -> Self {
        Cell { id, bounds, solid, volume, triangles, revision, mesh: OnceLock::new() }
    }
}

enum Layout {
    /// Part mode: splits create more parts
    Whole,
    /// Tiled mode: maps tile indices to positions in `cells`
    Tiled { origin: [f64; 3], size: f64, dims: [usize; 3], index: HashMap<u32, usize> },
}

/// Machining engine.
pub struct MachiningEngine<K: Kernel> {
    kernel: K,
    options: EngineOptions,
    layout: Layout,
    cells: Vec<Cell<K::Solid>>,
    revision: u64,
    next_id: u32,
    history: Vec<CutReport>,
    /// Provenance ID of triangles → source (used for analytic surface reconstruction)
    sources: HashMap<u32, Source>,
}

impl<K: Kernel> std::fmt::Debug for MachiningEngine<K> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MachiningEngine")
            .field("kernel", &self.kernel.name())
            .field("tiled", &self.is_tiled())
            .field("cells", &self.cells.len())
            .field("revision", &self.revision)
            .field("total_volume", &self.total_volume())
            .finish()
    }
}

impl<K: Kernel> MachiningEngine<K> {
    /// Creates an engine from stock primitives.
    ///
    /// With `tile_size` `None`, each primitive becomes one part. With `Some`, the union of all primitives
    /// is cut into cubic tiles.
    pub fn new(kernel: K, stock: &[StockShape], options: EngineOptions) -> Result<Self> {
        if stock.is_empty() {
            return Err(Error::InvalidInput("at least one stock shape is required".into()));
        }
        if !(options.sweep_tolerance > 0.0 && options.stock_tolerance > 0.0 && options.volume_epsilon >= 0.0) {
            return Err(Error::InvalidInput("tolerances must be positive".into()));
        }
        if options.simplify_tolerance <= 0.0 {
            return Err(Error::InvalidInput("simplify tolerance must be positive".into()));
        }
        if options.collision_volume_epsilon < 0.0 {
            return Err(Error::InvalidInput("collision volume epsilon must not be negative".into()));
        }
        for shape in stock {
            shape.validate()?;
        }
        let mut engine = MachiningEngine {
            kernel,
            options,
            layout: Layout::Whole,
            cells: Vec::new(),
            revision: 0,
            next_id: 0,
            history: Vec::new(),
            sources: HashMap::new(),
        };
        match engine.options.tile_size {
            None => engine.init_whole(stock)?,
            Some(size) => engine.init_tiled(stock, size)?,
        }
        Ok(engine)
    }

    fn init_whole(&mut self, stock: &[StockShape]) -> Result<()> {
        for shape in stock {
            let solid = self.kernel.make_stock(shape, self.options.stock_tolerance)?;
            self.kernel.validate(&solid)?;
            let (solid, source_id) = self.kernel.with_fresh_id(&solid);
            self.sources.insert(source_id, Source::Stock);
            let summary = self.kernel.summary(&solid);
            let id = PartId(self.next_id);
            self.next_id += 1;
            self.cells.push(Cell::new(id, summary.bounds, solid, summary.volume(), summary.triangles, 0));
        }
        Ok(())
    }

    fn init_tiled(&mut self, stock: &[StockShape], size: f64) -> Result<()> {
        if !(size > 0.0 && size.is_finite()) {
            return Err(Error::InvalidInput(format!("tile size must be positive: {size}")));
        }
        let mut solids = Vec::with_capacity(stock.len());
        let mut bounds = Aabb::empty();
        for shape in stock {
            let solid = self.kernel.make_stock(shape, self.options.stock_tolerance)?;
            self.kernel.validate(&solid)?;
            let (solid, source_id) = self.kernel.with_fresh_id(&solid);
            self.sources.insert(source_id, Source::Stock);
            bounds = bounds.union(&shape.bounds());
            solids.push(solid);
        }
        let whole = self.kernel.union_all(&solids)?;
        let origin = bounds.min;
        let extent = bounds.extent();
        let dims = [
            ((extent[0] / size).ceil() as usize).max(1),
            ((extent[1] / size).ceil() as usize).max(1),
            ((extent[2] / size).ceil() as usize).max(1),
        ];
        let mut index = HashMap::new();
        for i in 0..dims[0] {
            for j in 0..dims[1] {
                for k in 0..dims[2] {
                    let min = [origin[0] + i as f64 * size, origin[1] + j as f64 * size, origin[2] + k as f64 * size];
                    let max = [
                        (min[0] + size).min(bounds.max[0]),
                        (min[1] + size).min(bounds.max[1]),
                        (min[2] + size).min(bounds.max[2]),
                    ];
                    let tile_bounds = Aabb { min, max };
                    let tile_box = self.kernel.make_stock(&StockShape::Box { min, max }, self.options.stock_tolerance)?;
                    let solid = self.kernel.intersection(&whole, &tile_box)?;
                    if self.kernel.is_empty(&solid) {
                        continue;
                    }
                    let linear = (i * dims[1] + j) * dims[2] + k;
                    let volume = self.kernel.volume(&solid);
                    let triangles = self.kernel.triangle_count(&solid);
                    index.insert(linear as u32, self.cells.len());
                    self.cells.push(Cell::new(PartId(linear as u32), tile_bounds, solid, volume, triangles, 0));
                }
            }
        }
        self.next_id = (dims[0] * dims[1] * dims[2]) as u32;
        self.layout = Layout::Tiled { origin, size, dims, index };
        Ok(())
    }

    /// The kernel.
    pub fn kernel(&self) -> &K {
        &self.kernel
    }

    /// The options.
    pub fn options(&self) -> &EngineOptions {
        &self.options
    }

    /// Current revision; incremented on every `Updated` or `Exhausted` cut.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Whether the stock is tiled.
    pub fn is_tiled(&self) -> bool {
        matches!(self.layout, Layout::Tiled { .. })
    }

    /// Information about the cells (parts or tiles).
    pub fn parts(&self) -> Vec<PartInfo> {
        self.cells
            .iter()
            .map(|c| PartInfo { id: c.id, bounds: c.bounds, volume: c.volume, triangles: c.triangles, revision: c.revision })
            .collect()
    }

    /// Total remaining volume (mm³).
    pub fn total_volume(&self) -> f64 {
        self.cells.iter().map(|c| c.volume).sum()
    }

    /// Total triangle count.
    pub fn total_triangles(&self) -> usize {
        self.cells.iter().map(|c| c.triangles).sum()
    }

    /// Reports of all cuts so far.
    pub fn history(&self) -> &[CutReport] {
        &self.history
    }

    /// Cells changed or created after `revision`. Cells that disappeared can be found by diffing against `parts()`.
    pub fn changed_since(&self, revision: u64) -> Vec<PartId> {
        self.cells.iter().filter(|c| c.revision > revision).map(|c| c.id).collect()
    }

    /// Display mesh of a cell, cached until the cell changes.
    pub fn mesh(&self, id: PartId) -> Option<&TriMesh> {
        self.cells.iter().find(|c| c.id == id).map(|c| c.mesh.get_or_init(|| self.kernel.mesh(&c.solid)))
    }

    /// Display meshes of all cells.
    pub fn meshes(&self) -> Vec<(PartId, &TriMesh)> {
        self.cells.iter().map(|c| (c.id, c.mesh.get_or_init(|| self.kernel.mesh(&c.solid)))).collect()
    }

    /// All cells concatenated into one mesh.
    pub fn combined_mesh(&self) -> TriMesh {
        let mut out = TriMesh::default();
        for (_, m) in self.meshes() {
            out.append(m);
        }
        out
    }

    /// Writes all cells as binary STL.
    pub fn write_stl<W: Write>(&self, w: W) -> io::Result<()> {
        self.combined_mesh().write_stl(w)
    }

    /// Writes all cells to a file in the format given by its extension (stl / obj / glb / 3mf).
    pub fn write_mesh_file<P: AsRef<std::path::Path>>(&self, path: P) -> io::Result<()> {
        self.combined_mesh().write_file(path)
    }

    /// Reconstructs a B-rep of the machined shape (planes and cylinders with axis +Z).
    ///
    /// Surfaces produced by horizontal moves and plunges of flat end mills in 3-axis machining become analytic;
    /// everything else becomes planar facets of merged coplanar triangles. In tiled mode all tiles are united first.
    pub fn reconstruct_brep(&self) -> Result<Brep> {
        let machined: K::Solid = match self.cells.len() {
            0 => return Err(Error::InvalidInput("no material left to reconstruct".into())),
            1 => self.cells[0].solid.clone(),
            _ => self.kernel.union_all(&self.cells.iter().map(|c| c.solid.clone()).collect::<Vec<_>>())?,
        };
        let (mesh, ids) = self.kernel.mesh_with_sources(&machined);
        brep::reconstruct(&mesh, &ids, &self.sources, self.options.sweep_tolerance * 2.0)
    }

    /// Writes the machined shape as a B-rep in STEP (AP214).
    pub fn write_step<W: Write>(&self, w: W, name: &str) -> Result<()> {
        let brep = self.reconstruct_brep()?;
        brep.write_step(w, name)?;
        Ok(())
    }

    /// Compares against a design model (a closed mesh) and returns the volumes and regions of excess material and gouges.
    ///
    /// With a positive `tolerance`, excess material is what remains outside the design dilated by that distance,
    /// and a gouge is what is missing inside the design eroded by that distance.
    pub fn compare_with_design(&self, design: &TriMesh, tolerance: f64) -> Result<DesignComparison> {
        if tolerance < 0.0 || !tolerance.is_finite() {
            return Err(Error::InvalidInput("tolerance must be non-negative".into()));
        }
        let design_solid = self.kernel.make_mesh(design, self.options.stock_tolerance * 0.01)?;
        let (outer, inner) = if tolerance > 0.0 {
            (
                self.kernel.offset(&design_solid, tolerance, self.options.sweep_tolerance)?,
                self.kernel.offset(&design_solid, -tolerance, self.options.sweep_tolerance)?,
            )
        } else {
            (design_solid.clone(), design_solid)
        };
        let machined: Option<K::Solid> = match self.cells.len() {
            0 => None,
            1 => Some(self.cells[0].solid.clone()),
            _ => Some(self.kernel.union_all(&self.cells.iter().map(|c| c.solid.clone()).collect::<Vec<_>>())?),
        };
        let (excess, gouge) = match &machined {
            Some(m) => (self.kernel.difference(m, &outer)?, self.kernel.difference(&inner, m)?),
            None => (self.kernel.difference(&inner, &inner)?, inner.clone()),
        };
        let summarize = |solid: &K::Solid| -> (f64, TriMesh, Option<Aabb>) {
            if self.kernel.is_empty(solid) {
                (0.0, TriMesh::default(), None)
            } else {
                let s = self.kernel.summary(solid);
                (s.volume(), self.kernel.mesh(solid), Some(s.bounds))
            }
        };
        let (excess_volume, excess_mesh, excess_bounds) = summarize(&excess);
        let (gouge_volume, gouge_mesh, gouge_bounds) = summarize(&gouge);
        Ok(DesignComparison {
            tolerance,
            excess_volume,
            gouge_volume,
            excess: excess_mesh,
            gouge: gouge_mesh,
            excess_bounds,
            gouge_bounds,
        })
    }

    /// Builds the swept volumes (per convex piece) of the cutting and non-cutting parts for a sequence of segments.
    fn plan(&mut self, tool: &Tool, segments: &[MotionSegment]) -> Result<SweepPlan<K::Solid>> {
        let mut plan = SweepPlan { cutting: Vec::new(), non_cutting: Vec::new(), bounds: Aabb::empty(), pieces: 0 };
        let radius = tool.radius();
        let mesh_radius = {
            let n = sweep::circle_segments(radius, self.options.sweep_tolerance);
            sweep::mean_width_radius(radius, n)
        };
        let flat = matches!(tool.cutter, Cutter::FlatEndMill { .. });
        for segment in segments {
            segment.validate()?;
            let pieces = sweep::swept_pieces(tool, segment, self.options.sweep_tolerance)?;
            plan.pieces += pieces.len();
            for (from, to, points) in pieces {
                let volume = sweep::SweptVolume::ConvexHull(points);
                let piece_bounds = volume.bounds();
                plan.bounds = plan.bounds.union(&piece_bounds);
                let solid = self.kernel.make_swept(&volume)?;
                let (solid, source_id) = self.kernel.with_fresh_id(&solid);
                let source = if flat && from.is_vertical(1e-9) && to.is_vertical(1e-9) {
                    Source::FlatSweep { radius, mesh_radius, from, to }
                } else {
                    Source::Other
                };
                self.sources.insert(source_id, source);
                plan.cutting.push((solid, piece_bounds));
            }
            if self.options.collision_check {
                for volume in sweep::non_cutting_volumes(tool, segment, self.options.sweep_tolerance)? {
                    plan.bounds = plan.bounds.union(&volume.bounds());
                    let piece_bounds = volume.piece_bounds();
                    let solids = self.kernel.make_swept_pieces(&volume)?;
                    plan.non_cutting.extend(solids.into_iter().zip(piece_bounds));
                }
            }
        }
        Ok(plan)
    }

    /// Cuts one segment. On failure the state is not modified.
    pub fn cut(&mut self, tool: &Tool, segment: &MotionSegment) -> Result<CutReport> {
        let started = Instant::now();
        tool.validate()?;
        segment.validate()?;
        let plan = self.plan(tool, std::slice::from_ref(segment))?;
        let mut diagnostics = Vec::new();
        if plan.pieces > 1 {
            diagnostics.push(format!("segment split into {} sub-steps", plan.pieces));
        }
        self.apply(&plan, started, diagnostics)
    }

    /// Cuts several segments as one update: each cell has the segments' sweeps subtracted in order, with a single report.
    pub fn cut_batch(&mut self, tool: &Tool, segments: &[MotionSegment]) -> Result<CutReport> {
        let started = Instant::now();
        if segments.is_empty() {
            return Err(Error::InvalidInput("cut_batch needs at least one segment".into()));
        }
        tool.validate()?;
        let plan = self.plan(tool, segments)?;
        self.apply(&plan, started, vec![format!("batch of {} segments in {} pieces", segments.len(), plan.pieces)])
    }

    /// Cuts several segments in order. Segments before a failure are committed.
    pub fn cut_many(&mut self, tool: &Tool, segments: &[MotionSegment]) -> std::result::Result<Vec<CutReport>, BatchError> {
        let mut completed = Vec::with_capacity(segments.len());
        for (index, segment) in segments.iter().enumerate() {
            match self.cut(tool, segment) {
                Ok(report) => completed.push(report),
                Err(error) => return Err(BatchError { index, completed, error }),
            }
        }
        Ok(completed)
    }

    /// Positions of the cells overlapping a bounding box.
    fn candidates(&self, bounds: &Aabb) -> Vec<usize> {
        match &self.layout {
            Layout::Whole => self
                .cells
                .iter()
                .enumerate()
                .filter(|(_, c)| c.bounds.intersects(bounds))
                .map(|(i, _)| i)
                .collect(),
            Layout::Tiled { origin, size, dims, index } => {
                let mut lo = [0usize; 3];
                let mut hi = [0usize; 3];
                for a in 0..3 {
                    let l = ((bounds.min[a] - origin[a]) / size).floor();
                    let h = ((bounds.max[a] - origin[a]) / size).floor();
                    if h < 0.0 || l >= dims[a] as f64 {
                        return Vec::new();
                    }
                    lo[a] = l.max(0.0) as usize;
                    hi[a] = (h.max(0.0) as usize).min(dims[a] - 1);
                }
                let mut out = Vec::new();
                for i in lo[0]..=hi[0] {
                    for j in lo[1]..=hi[1] {
                        for k in lo[2]..=hi[2] {
                            let linear = ((i * dims[1] + j) * dims[2] + k) as u32;
                            if let Some(&pos) = index.get(&linear) {
                                if self.cells[pos].bounds.intersects(bounds) {
                                    out.push(pos);
                                }
                            }
                        }
                    }
                }
                out
            }
        }
    }

    /// Subtracts each piece of the swept volume from the overlapping cells in order, checks collisions, validates and commits.
    fn apply(&mut self, plan: &SweepPlan<K::Solid>, started: Instant, mut diagnostics: Vec<String>) -> Result<CutReport> {
        let parts_before = self.cells.len();
        let bounds = plan.bounds;
        let candidates = self.candidates(&bounds);
        if candidates.is_empty() {
            return Ok(self.finish(
                CutOutcome::NoChange,
                None,
                0.0,
                started,
                parts_before,
                0,
                None,
                vec!["swept volume does not overlap any part".into()],
            ));
        }
        let next_revision = self.revision + 1;
        let mut next_id = self.next_id;
        let mut updates: Vec<(usize, Vec<Cell<K::Solid>>)> = Vec::new();
        let mut removed = 0.0;
        let mut changed: Option<Aabb> = None;
        let mut touched = 0;
        let mut collision_volume = 0.0;
        let mut collision_bounds = Aabb::empty();

        for pos in candidates {
            let cell = &self.cells[pos];
            let local = cell.bounds.intersection(&bounds).unwrap_or(cell.bounds);
            // Subtract the cutting pieces in order; skip the cell if no piece overlaps it
            let mut current: Option<K::Solid> = None;
            let mut exhausted = false;
            for (piece, piece_bounds) in &plan.cutting {
                if !piece_bounds.intersects(&cell.bounds) {
                    continue;
                }
                let base = current.as_ref().unwrap_or(&cell.solid);
                let r = self.kernel.difference(base, piece)?;
                if self.kernel.is_empty(&r) {
                    exhausted = true;
                    current = Some(r);
                    break;
                }
                current = Some(r);
            }
            // Intersection of the non-cutting parts with the stock after removal
            if !exhausted {
                let base = current.as_ref().unwrap_or(&cell.solid);
                for (piece, piece_bounds) in &plan.non_cutting {
                    if !piece_bounds.intersects(&cell.bounds) {
                        continue;
                    }
                    let inter = self.kernel.intersection(base, piece)?;
                    if self.kernel.is_empty(&inter) {
                        continue;
                    }
                    let v = self.kernel.volume(&inter);
                    if v > self.options.collision_volume_epsilon {
                        collision_volume += v;
                        collision_bounds = collision_bounds.union(&self.kernel.summary(&inter).bounds);
                    }
                }
            }
            let Some(result) = current else {
                continue;
            };
            touched += 1;
            if self.kernel.is_empty(&result) {
                removed += cell.volume;
                changed = Some(changed.map_or(local, |c| c.union(&local)));
                diagnostics.push(format!("part {} removed entirely", cell.id.0));
                updates.push((pos, Vec::new()));
                continue;
            }
            let new_volume = match self.layout {
                Layout::Whole => self.kernel.summary(&result).volume(),
                Layout::Tiled { .. } => self.kernel.volume(&result),
            };
            let delta = cell.volume - new_volume;
            let eps = self.options.volume_epsilon * cell.volume.max(1.0);
            if delta < -eps {
                return Err(Error::Validation(format!(
                    "volume increased on part {}: {} -> {}",
                    cell.id.0, cell.volume, new_volume
                )));
            }
            if delta <= eps {
                continue;
            }
            removed += delta;
            changed = Some(changed.map_or(local, |c| c.union(&local)));
            let new_cells = match self.layout {
                Layout::Whole => {
                    let pieces = self.pieces_of(result)?;
                    let largest = pieces
                        .iter()
                        .enumerate()
                        .max_by(|a, b| a.1 .1.volume().total_cmp(&b.1 .1.volume()))
                        .map(|(k, _)| k)
                        .unwrap_or(0);
                    if pieces.len() > 1 {
                        diagnostics.push(format!("part {} split into {} parts", cell.id.0, pieces.len()));
                    }
                    pieces
                        .into_iter()
                        .enumerate()
                        .map(|(k, (solid, s))| {
                            let id = if k == largest {
                                cell.id
                            } else {
                                let id = PartId(next_id);
                                next_id += 1;
                                id
                            };
                            Cell::new(id, s.bounds, solid, s.volume(), s.triangles, next_revision)
                        })
                        .collect()
                }
                Layout::Tiled { .. } => {
                    let mut solid = result;
                    let mut triangles = self.kernel.triangle_count(&solid);
                    let mut volume = new_volume;
                    if let Some(threshold) = self.options.simplify_threshold {
                        if triangles > threshold {
                            let simplified = self.kernel.simplify(&solid, self.options.simplify_tolerance)?;
                            let t = self.kernel.triangle_count(&simplified);
                            if t < triangles && !self.kernel.is_empty(&simplified) {
                                diagnostics.push(format!("part {} simplified {} -> {} triangles", cell.id.0, triangles, t));
                                volume = self.kernel.volume(&simplified);
                                solid = simplified;
                                triangles = t;
                            }
                        }
                    }
                    vec![Cell::new(cell.id, cell.bounds, solid, volume, triangles, next_revision)]
                }
            };
            updates.push((pos, new_cells));
        }

        let collision = if collision_volume > 0.0 {
            diagnostics.push(format!("collision: non-cutting part intersects {collision_volume:.4} mm3 of stock"));
            Some(Collision { volume: collision_volume, bounds: collision_bounds })
        } else {
            None
        };

        if updates.is_empty() {
            return Ok(self.finish(
                CutOutcome::NoChange,
                None,
                0.0,
                started,
                parts_before,
                touched,
                collision,
                vec!["swept volume overlaps bounds but removes no material".into()],
            ));
        }

        let mut removed_any = false;
        let mut cells = Vec::with_capacity(self.cells.len() + updates.len());
        let mut pending = updates.into_iter().peekable();
        for (pos, cell) in std::mem::take(&mut self.cells).into_iter().enumerate() {
            if pending.peek().map(|(p, _)| *p == pos).unwrap_or(false) {
                let (_, new_cells) = pending.next().expect("peeked");
                if new_cells.is_empty() {
                    removed_any = true;
                }
                cells.extend(new_cells);
            } else {
                cells.push(cell);
            }
        }
        self.cells = cells;
        if removed_any {
            if let Layout::Tiled { index, .. } = &mut self.layout {
                index.clear();
                for (pos, cell) in self.cells.iter().enumerate() {
                    index.insert(cell.id.0, pos);
                }
            }
        }
        self.next_id = next_id;
        self.revision = next_revision;
        let outcome = if self.cells.is_empty() { CutOutcome::Exhausted } else { CutOutcome::Updated };
        Ok(self.finish(outcome, changed, removed, started, parts_before, touched, collision, diagnostics))
    }

    #[allow(clippy::too_many_arguments)]
    fn finish(
        &mut self,
        outcome: CutOutcome,
        changed_bounds: Option<Aabb>,
        removed_volume: f64,
        started: Instant,
        parts_before: usize,
        cells_touched: usize,
        collision: Option<Collision>,
        diagnostics: Vec<String>,
    ) -> CutReport {
        let report = CutReport {
            outcome,
            revision: self.revision,
            changed_bounds,
            elapsed: started.elapsed(),
            removed_volume,
            parts_before,
            parts_after: self.cells.len(),
            cells_touched,
            collision,
            diagnostics,
        };
        self.history.push(report.clone());
        report
    }

    /// Splits a boolean result into parts (part mode only). Internal voids (components with negative signed volume) are re-attached to the smallest enclosing shell.
    fn pieces_of(&self, solid: K::Solid) -> Result<Vec<(K::Solid, crate::kernel::SolidSummary)>> {
        let components = self.kernel.components(&solid);
        if components.len() <= 1 {
            let summary = self.kernel.summary(&solid);
            return Ok(vec![(solid, summary)]);
        }
        let mut outers: Vec<(K::Solid, crate::kernel::SolidSummary, Vec<K::Solid>)> = Vec::new();
        let mut voids: Vec<(K::Solid, crate::kernel::SolidSummary)> = Vec::new();
        for c in components {
            let s = self.kernel.summary(&c);
            if s.signed_volume < 0.0 {
                voids.push((c, s));
            } else {
                outers.push((c, s, Vec::new()));
            }
        }
        for (v, vs) in voids {
            let host = outers
                .iter()
                .enumerate()
                .filter(|(_, (_, s, _))| s.bounds.contains(&vs.bounds))
                .min_by(|a, b| a.1 .1.volume().total_cmp(&b.1 .1.volume()))
                .map(|(k, _)| k);
            match host {
                Some(k) => outers[k].2.push(v),
                None => return Err(Error::Validation("internal void has no containing body".into())),
            }
        }
        let mut out = Vec::with_capacity(outers.len());
        for (solid, summary, holes) in outers {
            if holes.is_empty() {
                out.push((solid, summary));
            } else {
                let mut all = Vec::with_capacity(holes.len() + 1);
                all.push(solid);
                all.extend(holes);
                let composed = self.kernel.compose(&all)?;
                let summary = self.kernel.summary(&composed);
                out.push((composed, summary));
            }
        }
        Ok(out)
    }
}
