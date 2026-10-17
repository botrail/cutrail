# Changelog

## 0.1.0 (unreleased)

First release.

- Stock as a closed mesh (manifold-rust booleans); boxes, cylinders, meshes and STEP (feature `stepio`) as stock.
- Flat, ball and bull-nose end mills with optional shank and holder; linear, arc and helical moves; arbitrary and rotating tool axes (5-axis).
- Tiled stock with bounded cost per cut, batching, incremental mesh transfer (`changed_since`).
- Shank/holder collision detection, comparison against a design model (excess material and gouges).
- Output as STL, OBJ, glTF and 3MF; analytic B-rep (planes and cylinders) in STEP for 3-axis flat-end-mill results, verified in FreeCAD and OpenCASCADE.
- `SegmentBuilder` for sampled pose streams with distance, angle and chord-deviation thresholds.
- WebAssembly bindings (`cutrail-wasm`, not published).
- Optional `serde` and `parallel` features.
