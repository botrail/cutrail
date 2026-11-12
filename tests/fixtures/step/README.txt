Controlled STEP fixtures for Cutrail import tests.

box.step: [0,10]^3, six planes, volume 1000 mm^3.
nurbs-cylinder.step: radius 5, height 10, axis +Z, volume 250*pi mm^3.
void.step: [0,10]^3 less the enclosed [3,7]^3 void, volume 936 mm^3.
These three files were generated from the analytic primitives with truck-modeling
0.6 and truck-stepio 0.3. Their timestamps were normalized. No truck dependency
is needed to run the tests.

spline-box.step: box.step with its z=0 plane replaced by a bilinear B-spline
patch spanning the same face, added control points #9001-#9004. Geometry and
expected volume remain unchanged. Exercises retained spline geometry and trims.

rational-box.step: spline-box.step with the bilinear surface made rational;
weights ((1,2),(1,2)) change the parameterization but keep the same planar face.
Expected volume remains 1000 mm^3.
Cutrail supplies exact rational inverse pcurves for its straight sides; the import
regression checks the nonlinear UV mapping and unchanged 1000 mm^3 volume.

cylinder.step: the same radius-5, height-10 cylinder written directly with
CYLINDRICAL_SURFACE, two PLANE caps, CIRCLE edges, and a seam used twice.
Uses the box fixture's millimetre context; expected volume 250*pi mm^3.

assembly.step: box.step's source part used twice, once through a subassembly.
The subassembly rotates +90 degrees about Z then translates (30,40,5) mm.
Its part is locally translated (3,4,0) mm: world bounds (16,43,5)-(26,53,15).
The direct instance translates (-20,0,0): bounds (-20,0,0)-(-10,10,10).
Both source bodies are #16; their occurrence paths distinguish them. Total
stock volume is 2000 mm^3; the world-space slot-cut test compares against
independent analytic box stock (about 1840 mm^3 remains).

mapped.step: box.step with two mapped instances, including a nested map. The
shared map origin is rotated +90 degrees and translated (1,2,0) mm.
First target rotates -90 degrees, scales by 2, then translates (20,30,5):
world bounds (2,14,5)-(22,34,25), volume 8000 mm^3.
The nested target places the second instance at (-31,0,0)-(-21,10,10).
Assembly and mapped fixtures were authored from these analytic placements;
they exercise noncommuting transforms, not just additive translations.

curved-hole.step: independently authored [0,10]^2 plate, bottom z=0, top
z=2+(x*x+y*y)/100, excluding the through hole [3,7]^2. Its top is a degree
(4,2) B-spline with parameterisation x=10*u*u, y=10*v and z=2+u^4+v^2.
All top 3D edges are quadratic curves in physical x/y, so their UV inverses
need sqrt and cannot be presumed exact rational pcurves. Planar side faces
and shared edges close one solid. Analytic volume: 226.24 mm^3. Tests check
volume, machining within the void, retained high-degree CAD and fit diagnostics.
A derived open-sheet test replaces the rectangular inner loop with a closed
spatial conic over the projected circle (x-5)^2+(y-5)^2=4. It checks the hole
in tessellation and projected area 100-4*pi; it is display-mesh evidence.
