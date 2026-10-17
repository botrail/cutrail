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

## Recorded robot path (pose stream)

The machining demo of [botrail](https://github.com/botrail/botrail), a 6-axis robot carrying a spindle over a 180×120×12 mm plate (a rounded-rectangle trim, a zigzag pocket and a 5-axis rim chamfer, 99 s cycle), was baked and its tool-tip poses sampled in the plate frame at 100 Hz and 1 kHz (9,916 and 99,151 poses, Ø8 flat end mill). `examples/pose_stream.rs` replays the stream through `SegmentBuilder` and the engine. The reference is the stream cut one segment per sample (9,565 segments, 167 s, 682k triangles); excess and gouge are the volumes by which a run differs from it (material left, material removed beyond it), out of 10,825 mm³ removed. botrail's own voxel carve of the same bake removes 10,827 mm³ at 0.25 mm voxels.

| Setting (20 mm tiles) | Segments | Total [s] | p50 / p90 [ms] | Triangles | Excess / gouge [mm³] |
|---|---|---|---|---|---|
| distance 0.25 mm, 2° | 4,323 | 33.7 | 6.8 / 15.4 | 310k | 1.2 / 1.2 |
| distance 0.5 mm, 2° | 2,338 | 10.3 | 4.1 / 8.4 | 160k | 5.7 / 2.4 |
| distance 1 mm, 2° | 1,422 | 4.1 | 2.8 / 5.4 | 87k | 19.8 / 3.8 |
| distance 2 mm, 2° | 772 | 1.8 | 2.2 / 4.1 | 48k | 56.9 / 9.6 |
| distance 4 mm, 2° | 427 | 1.0 | 2.1 / 4.4 | 31k | 167.0 / 21.8 |
| deviation 0.005 mm (cap 5 mm, 2°) | 938 | 2.5 | 2.7 / 4.6 | 44k | 1.2 / 2.6 |
| deviation 0.01 mm | 710 | 1.8 | 2.4 / 4.5 | 38k | 2.6 / 3.9 |
| deviation 0.02 mm | 582 | 1.5 | 2.5 / 4.6 | 35k | 5.4 / 5.8 |
| deviation 0.05 mm | 474 | 1.2 | 2.2 / 4.4 | 31k | 11.4 / 11.5 |
| deviation 0.01 mm, angle 0.5° | 848 | 3.6 | 2.8 / 13.5 | 47k | 2.5 / 2.2 |
| deviation 0.01 mm, angle 5° | 684 | 1.8 | 2.4 / 4.0 | 37k | 2.6 / 4.3 |
| deviation 0.01 mm, from the 1 kHz stream | 699 | 1.7 | 2.2 / 4.1 | 38k | 3.0 / 3.9 |

- A distance threshold alone trades accuracy for segments badly: the chords cut corners, and 1 mm already leaves 20 mm³ of excess. The deviation bound reaches the accuracy of the 0.25 mm threshold with a quarter of the segments and a thirteenth of the time.
- The angle threshold only matters for the 5-axis chamfer; 2° is enough, and 0.5° adds segments without accuracy.
- The pose rate does not matter: the 1 kHz stream gives the same segments at the same cost.
- Tile size (deviation 0.01 mm): 10 mm 2.1 s (432 tiles, 55k triangles), 15 mm 2.4 s, 20 mm 1.8 s, 30 mm 1.9 s, 40 mm 2.0 s, no tiling 9.0 s (p50 15.5 ms). The result is identical for every tile size; for an 8 mm tool, 2 to 4 tool diameters is the flat region.
- `cut_batch` with 4 or 16 segments per call saves under 10% here.

Recommended for a robot pose stream: `SegmentBuilder::new(5.0, 2.0).with_max_deviation(0.01)` and tiles of 2 to 4 tool diameters. The 99 s cycle then replays in 1.8 s (2.5 ms per segment). The recorded path itself deviates from the programmed one (the plunge overshoots by 0.5 mm and the trim floor drifts by 0.1 mm); the replay reproduces the recorded motion, not the program.
