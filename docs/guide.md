# User guide

English | [日本語](ja/guide.md)

## Creating an engine

```rust
use cutrail::{Engine, EngineOptions, ManifoldKernel, StockShape};

let stock = StockShape::Box { min: [0.0, 0.0, 0.0], max: [200.0, 100.0, 20.0] };
let options = EngineOptions { sweep_tolerance: 0.01, ..Default::default() }.with_tile_size(10.0);
let mut engine = Engine::new(ManifoldKernel, &[stock], options)?;
```

| `EngineOptions` | Default | Meaning |
|---|---|---|
| `sweep_tolerance` | 0.01 mm | Chord tolerance of swept volumes; sets the circle subdivision and the sub-steps of rotations and arcs |
| `stock_tolerance` | 0.01 mm | Chord tolerance of cylindrical stock; mesh stock vertices are merged at 1/100 of this |
| `volume_epsilon` | 1e-9 | A removal below this fraction of the cell volume is reported as `NoChange` |
| `tile_size` | `None` | Edge length of cubic tiles; `None` keeps the stock as parts |
| `simplify_threshold` / `simplify_tolerance` | `None` / 1e-3 | Simplify a tile once its triangle count exceeds the threshold (normally unnecessary) |
| `collision_check` / `collision_volume_epsilon` | `false` / 1e-6 mm³ | Shank and holder collision detection |

Several `StockShape`s can be passed: `Box`, `Cylinder` (axis +Z) and `Mesh` (a closed triangle mesh, e.g. read from STL or converted from STEP).

A tile edge of one to two tool diameters works well. Smaller tiles make each boolean faster and reduce the total triangle count; even 3,200 tiles take about 0.2 s to set up.

## Tools

```rust
use cutrail::{Holder, Shank, Tool};

let flat = Tool::flat_end_mill(6.0, 20.0);                  // diameter, flute length
let ball = Tool::ball_end_mill(6.0, 20.0);                  // hemispherical tip of radius 3
let bull = Tool::bull_nose_end_mill(6.0, 1.0, 20.0);        // corner radius 1
let with_holder = flat.clone()
    .with_shank(Shank { diameter: 6.0, length: 30.0 })
    .with_holder(Holder { diameter: 20.0, length: 40.0 });
```

The flute length is the height from the tip (lowest point) to the top of the cutting part. Shank and holder do not cut; they are only intersected with the stock when `collision_check` is enabled.

## Poses and segments

```rust
use cutrail::{MotionSegment, Pose};

let a = Pose::vertical([10.0, 10.0, 15.0]);              // axis +Z
let b = Pose::tilted_about_y([50.0, 10.0, 15.0], 20.0);  // axis tilted 20° about Y
let c = Pose::new([50.0, 10.0, 15.0], [0.3, 0.0, 0.95]); // any axis (normalized)

let line = MotionSegment::linear(a, b);
let arc = MotionSegment::arc(
    Pose::vertical([30.0, 20.0, 15.0]),  // start
    Pose::vertical([20.0, 30.0, 15.0]),  // end
    [20.0, 20.0, 15.0],                  // center
    [0.0, 0.0, 1.0],                     // normal (counterclockwise, right-handed)
);
let mid = arc.pose_at(0.5);
```

- The tip follows the line or arc; the axis rotates from start to end by spherical linear interpolation. A segment whose axis turns by 150° or more is rejected; split it.
- An arc is a full circle when start and end coincide in the arc plane. The height along the normal changes linearly with the angle (helical). Flip the normal for a clockwise arc.
- Coordinates are in the stock frame, in millimeters. Coordinate and unit conversion from the upstream system is the caller's job.

## Cutting

```rust
let report = engine.cut(&tool, &line)?;             // one segment
let reports = engine.cut_many(&tool, &segments)?;   // in order; segments before a failure are committed
let report = engine.cut_batch(&tool, &segments)?;   // one update and one report for all segments
```

`CutReport`:

| Field | Meaning |
|---|---|
| `outcome` | `NoChange` (nothing removed, state unchanged), `Updated`, `Exhausted` (no material left) |
| `revision` | Revision after the cut; unchanged for `NoChange` |
| `removed_volume` | Removed volume in mm³ |
| `changed_bounds` | Bounding box of the change |
| `cells_touched` | Number of cells on which a boolean ran |
| `collision` | Shank/holder collision (volume and bounds) |
| `elapsed` | Processing time |
| `diagnostics` | Notes: splits, exhausted cells, sub-step counts, collisions |

On `Err` the state is not modified. `Error::Unsupported` can be resolved by splitting segments; `Error::Kernel` and `Error::Validation` report a failed boolean or a result that failed validation.

## State and display

```rust
for part in engine.parts() { /* id, bounds, volume, triangles, revision */ }
let mesh = engine.mesh(part_id);                    // cached display mesh
let changed = engine.changed_since(last_revision);  // resend only what changed
let total = engine.total_volume();
```

In tiled mode `PartId` is the tile index and exhausted tiles disappear from `parts()`. In part mode, pieces created by a split get new ids and the largest piece keeps the original id.

## Output

```rust
engine.write_stl(std::fs::File::create("out.stl")?)?;
engine.write_mesh_file("out.obj")?;   // stl / obj / glb / 3mf by extension
engine.write_step(std::fs::File::create("out.step")?, "machined")?;   // analytic B-rep (brep-export.md)
let mesh = engine.combined_mesh();    // TriMesh
```

## Loading stock

```rust
use cutrail::TriMesh;
let mut mesh = TriMesh::read_stl_file("stock.stl")?;   // binary or ASCII
mesh.merge_vertices(1e-4);                              // also done on import; useful to check closedness
let engine = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], options)?;
```

STEP files are read with feature `stepio` (truck-stepio reads the B-rep, which is tessellated and vertex-merged):

```rust
let shapes = cutrail::step::stock_from_step_file("stock.step", 0.01)?;   // one StockShape::Mesh per shell
let design = cutrail::step::meshes_from_step_file("design.step", 0.01)?; // meshes for comparison
```

## Comparing against a design model

```rust
let design = TriMesh::read_stl_file("design.stl")?;
let cmp = engine.compare_with_design(&design, 0.05)?;   // tolerance 0.05 mm (0 for exact)
println!("excess {:.3} mm3, gouge {:.3} mm3", cmp.excess_volume, cmp.gouge_volume);
cmp.excess.write_file("excess.glb")?;                   // region of excess material
```

`excess` is material left outside the design; `gouge` is design material that was removed. With a tolerance the design is dilated and eroded by that distance first. The offsets use a polyhedral sphere, so very small tolerances carry a rounding error at corners. The comparison is meant for verification, not for every cut.

## Collision detection

```rust
let options = EngineOptions::default().with_tile_size(10.0).with_collision_check();
let report = engine.cut(&tool, &seg)?;
if let Some(c) = &report.collision {
    eprintln!("non-cutting part intersects {:.3} mm3 of stock within {:?}", c.volume, c.bounds);
}
```

The swept volumes of shank and holder are intersected with the stock after the cutting part has been subtracted. The cut itself is still applied; what to do about the collision is up to the caller.

## Pose stream

Turns poses sampled at a physics or render rate into segments:

```rust
use cutrail::SegmentBuilder;
let mut builder = SegmentBuilder::new(0.5, 2.0);   // emit after 0.5 mm or 2° of motion
for pose in poses {
    if let Some(seg) = builder.push(pose) {
        engine.cut(&tool, &seg)?;
    }
}
if let Some(seg) = builder.flush() {
    engine.cut(&tool, &seg)?;
}
```

## WebAssembly

`cutrail-wasm` exposes `WasmEngine`. Segments are passed as a `Float64Array` with 12 numbers per segment (start tip xyz, start axis xyz, end tip xyz, end axis xyz); meshes come back per cell as a `Float32Array` of positions and a `Uint32Array` of indices.

```sh
wasm-pack build cutrail-wasm --target nodejs --release --out-dir pkg-node
node cutrail-wasm/test/node_test.cjs 1000 10 20        # segments tile tilt
wasm-pack build cutrail-wasm --target web --release --out-dir www/pkg   # Web Worker example in cutrail-wasm/www/
```

WebAssembly is single-threaded and runs at about 1.5x the native time.
