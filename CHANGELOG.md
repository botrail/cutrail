# Changelog

## 0.1.1 (unreleased)

- `MachiningEngine::mesh_with_kinds`: the mesh of one cell with the stock / cut kind of every triangle, for incremental display of tiled stock.
- `cutrail-wasm`: `mesh_with_kinds`, pose streaming (`set_stream`, `push_poses`, `flush`), `set_bull_nose_end_mill`, `segment_count`; `www/` is now a three.js demo that draws the cut tile by tile, with 3-axis and 5-axis presets (zigzag, rim chamfer, cone swarf, drafted pocket) and pose-stream CSV replay.

## 0.1.0 (2026-10-18)

First release.

- Stock as a closed mesh (manifold-rust booleans); boxes, cylinders, meshes and STEP (feature `stepio`) as stock.
- Flat, ball and bull-nose end mills with optional shank and holder; linear, arc and helical moves; arbitrary and rotating tool axes (5-axis).
- Tiled stock with bounded cost per cut, batching, incremental mesh transfer (`changed_since`).
- Shank/holder collision detection, comparison against a design model (excess material and gouges).
- Output as STL, OBJ, glTF and 3MF; analytic B-rep (planes and cylinders) in STEP for 3-axis flat-end-mill results, verified in FreeCAD and OpenCASCADE.
- `SegmentBuilder` for sampled pose streams with distance, angle and chord-deviation thresholds.
- WebAssembly bindings (`cutrail-wasm`, not published).
- Optional `serde` and `parallel` features.
