# Analytic surface reconstruction and STEP export

English | [日本語](ja/brep-export.md)

The machined stock is a mesh, but the surfaces produced by horizontal moves, plunges and horizontal arcs of a flat end mill in 3-axis machining can be reconstructed as planes and cylinders (axis +Z) and written as a B-rep.

```rust
let brep = engine.reconstruct_brep()?;
println!("faces (planes, cylinders) = {:?}", brep.surface_counts());
println!("edges (lines, arcs, polylines) = {:?}", brep.curve_counts());
brep.check(1e-6)?;                      // loops are closed, edge endpoints lie on their faces, no coincident vertices
engine.write_step(std::fs::File::create("machined.step")?, "machined")?;
```

## How it works

1. **Provenance.** The mesh boolean keeps, for every output triangle, the input it came from. The engine assigns a unique ID to the stock and to every convex piece of a swept volume and records the tool radius and the start and end poses; for a horizontal arc it also records the arc center and radius.
2. **Classification.** If a triangle comes from a flat-end-mill sweep with a vertical axis (horizontal move or plunge), its surface follows from the known geometry: the floor (a horizontal plane at the tip height), the straight walls (vertical planes at radius r from the path), and the end cylinders (axis through the tip, radius r). For a horizontal arc the walls are cylinders about the arc center with radii R ± r. A triangle is assigned to a wall or a cylinder only if all three of its vertices lie on the polygon of that surface, so the triangulation of the mesh (for example along tile seams) does not change the result. The radius is the true tool radius, not the polygon radius of the mesh. Stock triangles use their own plane. Everything else (ball and bull-nose tools, tilted axes, ramps, helical moves) is treated as facets, i.e. the plane of each triangle.
3. **Faces.** Triangles on the same surface that share an edge are merged; overlapping passes at one depth become a single floor. Cylindrical faces spanning more than 180° are split in two. In tiled mode the tiles are united first, and the zero-thickness walls that the union can leave along seams are removed.
4. **Edges.** Face boundaries are traced into loops and split at vertices where three or more faces meet. Plane–plane edges become lines; horizontal plane–cylinder edges become circular arcs; vertical plane–cylinder and cylinder–cylinder edges become vertical lines; anything else becomes a polyline. Vertices are projected onto all incident surfaces. Where a wall is tangent to an end cylinder (every slot has this), the contact line is added as a constraint so that the vertex lands exactly on it; vertices that end up on the same point are merged and the edges between them dropped. The seam between the two halves of a split cylinder is straightened into a vertical line.
5. **STEP.** AP214 `MANIFOLD_SOLID_BREP` with `PLANE` / `CYLINDRICAL_SURFACE` faces, `LINE` / `CIRCLE` / `POLYLINE` edges, `FACE_OUTER_BOUND` for outer loops and `FACE_BOUND` for holes.

## Examples

`examples/step_export.rs` writes these shapes as STEP and STL:

| Shape | Faces (planes, cylinders) | Edges (lines, arcs, polylines) |
|---|---|---|
| Blind hole (Ø4) in a 10 mm cube | (7, 2) | (14, 4, 0) |
| Stadium slot (Ø3) and a separate hole (Ø2) | (10, 4) | (22, 8, 0) |
| Zigzag pocket of five rows (Ø6), machined on 10 mm tiles | (13, 10) | (40, 20, 0) |
| Half-circle arc move (Ø4, arc radius 5) | (7, 5) | (17, 10, 0) |
| Ball end mill plunge (Ø4) | (294, 0), facets | (588, 0, 0) |
| Pass with the axis tilted 20° (Ø4) | (57, 0), facets | (145, 0, 0) |

Opened in FreeCAD 1.1.1 (File → Open and `Part.read`), each file is a single closed solid that passes `Shape.check(True)`. OpenCASCADE 7.8.1 (`BRepCheck_Analyzer`) reports every file valid with a maximum shape tolerance of 1e-7 mm, and the volumes equal the analytic values: 949.73 mm³ for the cube with the blind hole, where the polygonal mesh has 949.90 mm³. The tiled pocket reconstructs to the same 23 faces as the untiled one. Reading the STEP back with truck-stepio gives a closed mesh with the same volume.

## Limitations

- Only horizontal moves, plunges and horizontal arcs of flat end mills in 3-axis machining become analytic surfaces; other surfaces are included in the same STEP as facets. A recorded robot path whose height drifts from sample to sample produces facets as well, since consecutive poses are rarely at exactly the same height.
- Facet-only results (ball tools, 5-axis, recorded paths) carry one face per planar facet, so the files are large. The B-rep of a recorded 99 s robot cycle (710 segments, 15,900 faces, 20 MB) opens in FreeCAD and OpenCASCADE as a closed solid with the right volume, but the strict shape check flags self-intersecting wires on a few dozen sliver facets that the mesh booleans leave; such results are better exchanged as meshes, or healed in the CAD system.
- Validation has been done with FreeCAD / OpenCASCADE and by tessellating the STEP and reading it back; other CAD systems have not been tried.
