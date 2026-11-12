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
The pinned SolidCore evaluator cannot construct the required exact pcurve of a
straight boundary on this patch. The import test checks an explicit refusal;
supported non-unit NURBS weights are tested with nurbs-cylinder.step instead.

cylinder.step: the same radius-5, height-10 cylinder written directly with
CYLINDRICAL_SURFACE, two PLANE caps, CIRCLE edges, and a seam used twice.
Uses the box fixture's millimetre context; expected volume 250*pi mm^3.
