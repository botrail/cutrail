# Analytic surface reconstruction and STEP export

English | [日本語](ja/brep-export.md)

The machined stock is a mesh, but the surfaces produced by horizontal moves and plunges of a flat end mill in 3-axis machining can be reconstructed as planes and cylinders (axis +Z) and written as a B-rep.

```rust
let brep = engine.reconstruct_brep()?;
println!("faces (planes, cylinders) = {:?}", brep.surface_counts());
println!("edges (lines, arcs, polylines) = {:?}", brep.curve_counts());
brep.check(1e-6)?;                      // loops are closed and edge endpoints lie on their faces
engine.write_step(std::fs::File::create("machined.step")?, "machined")?;
```

## How it works

1. **Provenance.** The mesh boolean keeps, for every output triangle, the input it came from. The engine assigns a unique ID to the stock and to every convex piece of a swept volume and records the tool radius and the start and end poses.
2. **Classification.** If a triangle comes from a flat-end-mill sweep with a vertical axis (horizontal move or plunge), its surface follows from the known geometry: the floor (a horizontal plane at the tip height), the straight walls (vertical planes at radius r from the path), and the end cylinders (axis through the tip, radius r). The radius is the true tool radius, not the polygon radius of the mesh. Stock triangles use their own plane. Everything else (ball and bull-nose tools, tilted axes, ramps, the sides of arc moves) is treated as facets, i.e. the plane of each triangle.
3. **Faces.** Triangles on the same surface that share an edge are merged; overlapping passes at one depth become a single floor. Cylindrical faces spanning more than 180° are split in two.
4. **Edges.** Face boundaries are traced into loops and split at vertices where three or more faces meet. Plane–plane edges become lines; horizontal plane–cylinder edges become circular arcs; vertical plane–cylinder and cylinder–cylinder edges become vertical lines; anything else becomes a polyline. Vertices are projected onto all incident surfaces.
5. **STEP.** AP214 `MANIFOLD_SOLID_BREP` with `PLANE` / `CYLINDRICAL_SURFACE` faces, `LINE` / `CIRCLE` / `POLYLINE` edges, `FACE_OUTER_BOUND` for outer loops and `FACE_BOUND` for holes.

## Examples

| Shape | Faces (planes, cylinders) | Edges (lines, arcs, polylines) |
|---|---|---|
| Through slot (Ø4) | (10, 0) | (24, 0, 0) |
| Blind hole (Ø4) | (7, 2) | (14, 4, 0) |
| Stadium slot (Ø3) | (9, 2) | (20, 4, 0) |
| Pocket from three overlapping passes (Ø3) | (9, 6) | (24, 12, 0) |

Reading the STEP back with truck-stepio gives a volume within 1% of the engine's and a closed mesh.

## Limitations

- Only horizontal moves and plunges of flat end mills in 3-axis machining become analytic surfaces; other surfaces are included in the same STEP as facets.
- Tangent configurations (a cylinder tangent to a wall, a hole tangent to a stock face) can make vertex snapping unstable.
- Validation is done by tessellating the STEP and reading it back; checking with commercial CAD systems is left to the user.
