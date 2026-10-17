# Architecture

English | [日本語](ja/architecture.md)

## Data flow

```
Upstream (toolpath generation, robot or machine-tool motion)
   │  tool poses (tip and axis), or linear/arc segments
   ▼
SegmentBuilder (optional) ────► MotionSegment
   ▼
sweep: build the swept volume from the segment and tool (a sequence of convex hulls subdivided by chord tolerance)
   ▼
engine: pick the overlapping tiles, subtract through the kernel, validate, commit
   ▼
kernel::manifold: mesh booleans with Manifold
   ▼
Output: per-tile display meshes, STL / OBJ / glTF / 3MF, B-rep STEP, comparison against a design model
```

## Stock representation

The stock is a closed triangle mesh owned by the kernel (by default [manifold-rust](https://crates.io/crates/manifold-rust)). A cut is the boolean "stock minus swept volume", and the kernel guarantees that the result is a manifold mesh.

A B-rep stock updated by one boolean per segment was deliberately not chosen. Machining produces floors at the same depth and walls along the same path on every pass, and B-rep booleans tend to fail on exactly these coincident and touching faces. Trials with several Rust B-rep kernels failed almost every coincident or touching case, and in sequences of cuts the time per boolean grew with the face count. Mesh booleans handled every case and ran 1,000 consecutive cuts without a failure. B-rep is therefore used only at the boundaries: reading STEP, and writing STEP after reconstructing analytic surfaces.

## Swept volumes

Tools are convex bodies of revolution, so the volume swept by a pure translation is exactly the convex hull of the tool instances at both ends. A tool instance is a point cloud sampled from the revolution profile (radius versus height from the tip) around the axis; the kernel builds the hull.

Segments with a rotating axis and arc moves are sampled into a sequence of poses such that the chord error of the farthest tool point stays within `sweep_tolerance`, and the hulls of consecutive pose pairs are subtracted one after another. Forming their union first was measured to add triangles without saving time, so it is not done.

## Tiling

The stock is split into cubic tiles held as separate meshes; a cut touches only tiles whose bounds intersect the swept volume. The cost of one boolean then depends only on the contents of that tile, not on the machining history. Coplanar faces at tile boundaries are harmless for mesh booleans. Display meshes are cached per tile, and only tiles changed since a given revision need to be fetched.

Without tiling (part mode), parts separated by a cut are tracked individually and internal voids are attached to their enclosing shell.

## Provenance and B-rep reconstruction

Manifold keeps, for every output triangle, the identity of the input it came from. The engine assigns a unique ID to the stock and to every convex piece of a swept volume and records the tool geometry (radius, start and end poses). When a B-rep is requested, triangles are classified into planes and cylinders from this information and faces and edges are rebuilt (see [brep-export.md](brep-export.md)).

## Modules

| Module | Role |
|---|---|
| `engine` | Stock state (parts or tiles), applying cuts, validation, history, design comparison, B-rep reconstruction entry points |
| `kernel` | The `Kernel` abstraction and the `kernel::manifold` implementation |
| `sweep` | Swept volumes (sequences of hulls), circle subdivision, rotation sub-steps |
| `tool` | Tools (revolution profile of the cutting part, shank and holder) |
| `motion` | Poses, linear and arc segments, interpolation |
| `stock` | Initial stock shapes |
| `mesh` | Triangle mesh I/O and vertex merging |
| `brep` | Analytic surface reconstruction and STEP writing |
| `compare` | Result of comparing against a design model |
| `stream` | Adapter that coalesces sampled poses into segments |
| `step` (feature `stepio`) | STEP reading via truck-stepio |

The `Kernel` trait defines solid construction (stock, mesh, swept volume), difference, intersection and union, volume, meshing, and provenance access. Another mesh boolean library or a volumetric representation (such as dexels) can be placed behind the same boundary.
