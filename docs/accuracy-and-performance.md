# Accuracy and performance

English | [日本語](ja/accuracy-and-performance.md)

## How accuracy is controlled

| Element | Method | Typical error |
|---|---|---|
| Cylindrical surfaces | Polygonized with the number of segments (8 to 1024) that keeps the chord error within `sweep_tolerance`; the circumradius is chosen so that the mean width (perimeter/π) equals the diameter | The width of a straight slot matches on average regardless of direction; the end arcs are 0.3% smaller in area at 32 segments |
| Hemispheres and tori (ball and bull-nose) | Rings subdivided with the same chord tolerance (2 to 64 rings per quarter turn) | Removed volume within 2% of the analytic value |
| Axis rotation | Sub-steps such that the chord error of the farthest tool point stays within the tolerance; each sub-step is the hull of its end instances | 0.01 mm tolerance with a 20 mm flute gives 3.6° per sub-step |
| Arc moves | Subdivided by the chord error at the farthest tool point from the arc center | 16 pieces for a quarter circle of radius 3 mm with a 1 mm tool radius and 6 mm flute |
| Cylindrical stock | Circumradius chosen so that the polygon area equals the circle | Volume matches the analytic value |
| No-change detection | A removal below `volume_epsilon` times the cell volume is `NoChange` and the mesh is not replaced | Retracing a path adds no triangles |

Validation compares against analytic values (slots, holes, through holes, annuli) and against grid sampling of interpolated tool instances (0.05 to 0.25 mm spacing); all agree within 1 to 2%.

## Benchmark setup

A 200×100×20 mm block is pocketed with a 6 mm flat end mill in an x-direction zigzag (2 mm row pitch, 190 mm rows, 1 mm segments), stepping down 1.5 mm per level. Reproducible with `examples/bench.rs`. Single thread, Linux. Several runs were executed concurrently, so totals vary by 20 to 30%; use the numbers for trends.

## Effect of tiling (one segment per call)

| Setup | Segments | Total [s] | First 100 p50 [ms] | Last 100 p50 [ms] | Triangles | RSS [MB] |
|---|---|---|---|---|---|---|
| no tiling | 4,000 | 159 | 3.1 | 75.3 | 104k | 76 |
| 40 mm tiles | 10,000 | 124 | 2.9 | 14.5 | 204k | 74 |
| 20 mm tiles | 10,000 | 61 | 1.8 | 10.9 | 195k | 62 |
| 10 mm tiles | 10,000 | 48 | 2.8 | 6.2 | 169k | 50 |
| 5 mm tiles | 10,000 | 38 | 3.2 | 4.1 | 110k | 46 |
| 10 mm tiles | 100,000 | 624 | 2.0 | 6.0 | 172k | 92 |

Without tiling the time per cut grows with the number of segments. With tiling it depends only on the contents of the tile, staying below 10 ms (p90 8.6 ms, max 44 ms) through 100,000 segments. Smaller tiles are faster and produce fewer triangles.

## Tools and 5-axis (1,000 segments, 10 mm tiles)

| Tool | Tilt | Total [s] | p50 first/last [ms] | Triangles |
|---|---|---|---|---|
| flat | 0 | 2.5 | 1.9 / 2.0 | 27k |
| ball | 0 | 4.3 | 4.0 / 3.9 | 34k |
| flat | ±20° (axis change ≤ 3.1° per segment) | 4.0 | 3.4 / 3.8 | 41k |
| flat | ±60° (≤ 9.4°, up to 3 sub-steps) | 14.1 | 11.6 / 12.6 | 71k |
| ball | ±20° | 5.9 | 4.7 / 5.6 | 70k |

With 10,000 ±20° segments the last-100 median is 6.6 ms.

## Batching and simplification

- `cut_batch` with 8 segments per call reduces the total for 100,000 segments on 20 mm tiles from 944 s to 666 s, but triangles grow from 196k to 325k and RSS from 118 to 176 MB. Smaller tiles are both faster and lighter; use batching to reduce call counts (for example across the WebAssembly boundary).
- Simplifying after every cut barely reduces triangles and slows the booleans, so it is off by default.

## WebAssembly (Node, 1,000 segments, 10 mm tiles)

| Tilt | Per segment [ms] | Ratio to native |
|---|---|---|
| 0 | 3.7 | about 1.5 |
| ±20° | 6.5 | about 1.6 |

Removed and remaining volumes are identical to native. The wasm binary is 1.1 MB.

## Triangle growth

Every cut adds the geometry of a new wall (the scallop of one step), so the triangle count grows with the actual machined surface, roughly 84 triangles per cylinder. Tiling bounds the size of each boolean, and tiles that end up entirely inside a pocket disappear.
