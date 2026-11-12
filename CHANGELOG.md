# Changelog

## 0.1.1 (unreleased)

- High-degree and interior NURBS trims without an exact inverse can use adaptive cubic UV fitting. Outward-rounded Bernstein bounds verify the full 3D deviation against the input geometry tolerance; original surfaces and edges are retained, and fit diagnostics are saved with the CAD records.

- STEP import expands nested assembly and mapped instances, retaining local CAD, world transforms and instance paths. Meshes and stock apply placements, mirrors and scale-aware chord tolerances. Exact rational boundary pcurves also support bilinear NURBS patches that previously failed to import.

- `stepio` now uses SolidCore instead of truck. `StepModel` retains imported analytic/NURBS CAD records, names, face IDs and repair notes, and can be tessellated at different tolerances or serialized with `serde`. Stock imports preserve inner voids and reject partial results and unsupported placements.
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
