# cutrail

[![crates.io](https://img.shields.io/crates/v/cutrail.svg)](https://crates.io/crates/cutrail)
[![docs.rs](https://docs.rs/cutrail/badge.svg)](https://docs.rs/cutrail)
[![CI](https://github.com/botrail/cutrail/actions/workflows/ci.yml/badge.svg)](https://github.com/botrail/cutrail/actions/workflows/ci.yml)

English | [日本語](README.ja.md)

A material removal simulation core for machining, written in Rust. The stock is kept as a triangle mesh and the volume swept by the tool is subtracted with robust mesh booleans. cutrail supports 3- and 5-axis milling, reads and writes STEP, STL, OBJ, glTF and 3MF, compares the result against a design model, and runs in WebAssembly.

## Features

- **Robust material removal.** The stock is a mesh held by [manifold-rust](https://crates.io/crates/manifold-rust) (a pure-Rust port of Manifold). Coincident floors and walls, which every real toolpath produces, never make a cut fail. Failures are typed errors and leave the state untouched.
- **Tools and motions.** Flat, ball and bull-nose end mills. Linear, arc and helical moves. The tool axis can point anywhere, and the axis may rotate within a segment (5-axis). Sweeps are built as convex hulls subdivided by chord tolerance.
- **Bounded cost per cut.** The stock is tiled; only tiles overlapping the sweep are updated. After 100,000 segments the time per segment is still a few milliseconds.
- **I/O.** Stock from boxes, cylinders, meshes (STL) or STEP. Output as STL, OBJ, glTF or 3MF. Results of 3-axis flat-end-mill machining (including arc moves) can be exported as an analytic B-rep (planes and cylinders) in STEP, verified as valid solids in FreeCAD and OpenCASCADE.
- **Verification.** Comparison against a design model (volumes and regions of excess material and gouges), shank/holder collision detection, removed volume and changed bounds per cut.
- **Integration.** A pose-stream adapter that turns sampled tool poses into segments, incremental mesh transfer of changed tiles, and WebAssembly bindings for Web Workers and Node.

## Quick start

```toml
[dependencies]
cutrail = "0.1"
```

| Cargo feature | Adds |
|---|---|
| `stepio` | Reading stock and design models from STEP (via truck) |
| `serde` | `Serialize` / `Deserialize` for poses, segments, tools, options, reports, meshes and B-reps |
| `parallel` | Parallel booleans in manifold-rust (native only) |

```rust
use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

fn main() -> cutrail::Result<()> {
    // A 60x40x20 mm block, stored as 10 mm tiles
    let stock = StockShape::Box { min: [0.0, 0.0, 0.0], max: [60.0, 40.0, 20.0] };
    let options = EngineOptions::default().with_tile_size(10.0);
    let mut engine = Engine::new(ManifoldKernel, &[stock], options)?;

    // Cut a slot and drill a hole with a 6 mm flat end mill
    let tool = Tool::flat_end_mill(6.0, 20.0);
    let slot = MotionSegment::linear(Pose::vertical([10.0, 10.0, 15.0]), Pose::vertical([50.0, 10.0, 15.0]));
    let plunge = MotionSegment::linear(Pose::vertical([30.0, 30.0, 20.0]), Pose::vertical([30.0, 30.0, 12.0]));
    for seg in [slot, plunge] {
        let report = engine.cut(&tool, &seg)?;
        println!("{:?}: removed {:.2} mm3 in {:?}", report.outcome, report.removed_volume, report.elapsed);
    }

    // Write the result
    engine.write_mesh_file("machined.glb")?;
    engine.write_step(std::fs::File::create("machined.step")?, "machined")?;
    Ok(())
}
```

A 5-axis move only needs the tool pose (tip and axis) at both ends:

```rust
let seg = MotionSegment::linear(
    Pose::tilted_about_y([2.0, 5.0, 7.0], -25.0),
    Pose::tilted_about_y([8.0, 5.0, 7.0], 25.0),
);
```

## Main API

| Type / function | Role |
|---|---|
| `Engine` / `MachiningEngine<K>` | Stock state, cutting, validation, history: `new`, `cut`, `cut_many`, `cut_batch`, `parts`, `mesh`, `changed_since`, `write_mesh_file`, `write_step`, `compare_with_design` |
| `EngineOptions` | Chord tolerance, tile size, collision check and other settings |
| `Tool`, `Cutter`, `Shank`, `Holder` | Tools; the cutting part is distinguished from shank and holder |
| `Pose`, `MotionSegment` | Tip position and axis direction, linear and arc segments, `pose_at(t)` |
| `CutReport`, `CutOutcome`, `Collision` | Result of one cut (`NoChange` / `Updated` / `Exhausted`, removed volume, changed bounds, collision) |
| `TriMesh` | Mesh I/O (STL and OBJ in; STL, OBJ, glTF, 3MF out) and vertex merging |
| `StockShape` | Box, cylinder or mesh stock |
| `step` (feature `stepio`) | Read stock and design models from STEP |
| `Brep` | Reconstructed analytic B-rep and STEP export |
| `DesignComparison` | Difference against a design model (excess material and gouges) |
| `SegmentBuilder` | Coalesces sampled poses into segments by distance and angle thresholds and a chord-deviation bound |
| `Kernel` | Abstraction over the geometry kernel; `ManifoldKernel` is the default |

## Accuracy and performance

- Cylindrical surfaces are polygonized to the chord tolerance `sweep_tolerance` (default 0.01 mm) using a radius whose mean width matches the diameter. The removed-volume error is about 0.3% for a 4 mm tool.
- Rotating segments are subdivided so that the chord error of the farthest tool point stays within the tolerance.
- Pocketing 100,000 one-millimeter segments with 10 mm tiles: the median time per segment grows from 2 ms (first 100) to 6 ms (last 100) and memory stays at 92 MB. 5-axis sweeps cost about 1.7x a 3-axis sweep; WebAssembly (Node) runs at about 1.5x the native time.
- A 99 s robot machining cycle recorded at 100 Hz (botrail) replays in 1.8 s as 710 segments with the chord-deviation bound at 0.01 mm, within 0.04% of the removed volume of the one-segment-per-sample reference.
- Details in [docs/accuracy-and-performance.md](docs/accuracy-and-performance.md).

## Limitations

- Tools are bodies of revolution (flat, ball, bull-nose). Turning, threading and cutter compensation are out of scope.
- Analytic B-rep export covers horizontal moves, plunges and horizontal arcs of flat end mills in 3-axis machining; other surfaces are exported as facets.
- Tolerance offsets in the design comparison use a polyhedral sphere, so very small tolerances need finer settings.
- `SegmentBuilder` only emits linear segments; pass arcs explicitly as `MotionSegment::arc`.
- The API is 0.x. `Kernel` and `ManifoldKernel` expose manifold-rust types, so a minor release may change them; `Engine` and the geometry types are the stable surface.

## Documentation

- [docs/architecture.md](docs/architecture.md): overall design and why the stock is a mesh
- [docs/guide.md](docs/guide.md): usage (options, tools, motions, output, STEP, comparison, collisions, WebAssembly)
- [docs/accuracy-and-performance.md](docs/accuracy-and-performance.md): how accuracy is controlled and measured performance
- [docs/brep-export.md](docs/brep-export.md): analytic surface reconstruction and STEP export

## Development

```sh
cargo test --release                            # unit and integration tests
cargo test --release --features serde,stepio    # including serialization and STEP import
cargo fmt --all --check && cargo clippy --release --workspace --all-targets --all-features -- -D warnings
cargo run --release --example slot -- out.stl
cargo run --release --example bench -- 10000 10 1 0 0 flat   # segments tile batch simplify tilt tool
cargo run --release --example step_export -- out/            # sample shapes as STEP and STL
cargo run --release --example pose_stream -- poses.csv --max-deviation 0.01 --tile 20   # replay a recorded pose stream
cargo run --release --example compare_meshes -- a.stl b.stl  # excess and gouge volume between two meshes
wasm-pack build cutrail-wasm --target nodejs --release --out-dir pkg-node && node cutrail-wasm/test/node_test.cjs 1000 10 20
```

## Dependencies

- [manifold-rust](https://github.com/larsbrubaker/manifold-rust) (Apache-2.0): mesh booleans
- [truck](https://github.com/ricosjp/truck) (Apache-2.0, feature `stepio`): STEP reading and tessellation
- [zip](https://crates.io/crates/zip) for 3MF, [web-time](https://crates.io/crates/web-time) for timing in WebAssembly, [thiserror](https://crates.io/crates/thiserror)

## License

Apache-2.0 ([LICENSE](LICENSE))
