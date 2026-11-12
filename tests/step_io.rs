//! SolidCore STEP import, retained CAD records, and connection to mesh machining.
#![cfg(feature = "stepio")]
mod common;
use common::assert_close;
use cutrail::step::{self, cad};
use cutrail::{CutOutcome, Engine, EngineOptions, Error, ManifoldKernel, MotionSegment, Pose, Tool};

const BOX: &str = include_str!("fixtures/step/box.step");
const CYLINDER: &str = include_str!("fixtures/step/cylinder.step");
const NURBS_CYLINDER: &str = include_str!("fixtures/step/nurbs-cylinder.step");
const VOID: &str = include_str!("fixtures/step/void.step");
const SPLINE_BOX: &str = include_str!("fixtures/step/spline-box.step");
const RATIONAL_BOX: &str = include_str!("fixtures/step/rational-box.step");
const ASSEMBLY: &str = include_str!("fixtures/step/assembly.step");
const MAPPED: &str = include_str!("fixtures/step/mapped.step");
const CURVED_HOLE: &str = include_str!("fixtures/step/curved-hole.step");

fn extra_entities(step: &str, extra: &str) -> String {
    step.replace(
        "ENDSEC;\nEND-ISO-10303-21;",
        &format!("{extra}\nENDSEC;\nEND-ISO-10303-21;"),
    )
}

#[test]
fn box_import_can_be_machined_and_keeps_its_original_cad() {
    let model = step::model_from_step_str(BOX).unwrap();
    assert_eq!(model.bodies().len(), 1);
    let body = &model.bodies()[0];
    assert_eq!(body.entity(), 16);
    assert_eq!(body.brep().faces.len(), 6);
    assert_eq!(body.face_entities(), [18, 25, 32, 39, 46, 53]);
    assert!(!body.is_sheet());
    assert!(body
        .brep()
        .faces
        .iter()
        .all(|f| matches!(f.surface, cad::Surface::Plane { .. })));
    let original = model.clone();
    let shapes = model.stock_shapes(0.01).unwrap();
    let mut engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(engine.total_volume(), 1000.0, 1e-6, "STEP box volume");
    let r = engine
        .cut(
            &Tool::flat_end_mill(4.0, 6.0),
            &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0])),
        )
        .unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_close(engine.total_volume(), 840.0, 1e-3, "slot in STEP stock");
    assert_eq!(model, original, "meshing and cutting must leave input CAD unchanged");
    assert_close(
        model.bodies()[0].mesh(0.01).unwrap().signed_volume(),
        1000.0,
        1e-9,
        "source CAD",
    );
}

#[test]
fn cylinder_retains_analytic_faces_and_can_be_retessellated() {
    let model = step::model_from_step_str(CYLINDER).unwrap();
    let body = &model.bodies()[0];
    assert!(body
        .brep()
        .faces
        .iter()
        .any(|f| matches!(f.surface, cad::Surface::Cylinder { radius, .. } if radius == 5.0)));
    let original = body.brep().clone();
    let coarse = body.mesh(0.1).unwrap();
    let fine = body.mesh(0.005).unwrap();
    assert!(fine.triangle_count() > coarse.triangle_count());
    let expected = std::f64::consts::PI * 25.0 * 10.0;
    assert!((fine.signed_volume() - expected).abs() < (coarse.signed_volume() - expected).abs());
    assert_eq!(body.brep(), &original);
    let shapes = step::stock_from_step_str(CYLINDER, 0.01).unwrap();
    let engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(engine.total_volume(), expected, 1e-2, "STEP cylinder volume");
}

#[test]
fn inner_void_stays_attached_to_its_solid() {
    let model = step::model_from_step_str(VOID).unwrap();
    assert_eq!(model.bodies().len(), 1, "void shells are not separate stock bodies");
    assert_eq!(model.bodies()[0].brep().shells.len(), 2);
    let shapes = model.stock_shapes(0.01).unwrap();
    assert_eq!(shapes.len(), 1);
    let mut engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_eq!(engine.parts().len(), 1);
    assert_close(engine.total_volume(), 936.0, 1e-9, "STEP solid with a void");
    let r = engine
        .cut(
            &Tool::flat_end_mill(1.0, 1.0),
            &MotionSegment::linear(Pose::vertical([5.0, 5.0, 4.0]), Pose::vertical([5.0, 5.0, 5.0])),
        )
        .unwrap();
    assert_eq!(
        r.outcome,
        CutOutcome::NoChange,
        "cutting in the void must not remove stock"
    );
}

#[test]
fn cylinder_written_as_nurbs_remains_curved_cad() {
    let model = step::model_from_step_str(NURBS_CYLINDER).unwrap();
    assert!(model.bodies()[0]
        .brep()
        .faces
        .iter()
        .any(|f| matches!(&f.surface, cad::Surface::RationalSpline { homogeneous, .. } if homogeneous.iter().flatten().any(|p| p[3] != 1.0))));
    let original = model.clone();
    let shapes = model.stock_shapes(0.01).unwrap();
    let engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(
        engine.total_volume(),
        250.0 * std::f64::consts::PI,
        1e-2,
        "NURBS cylinder volume",
    );
    assert_eq!(model, original, "rational weights must survive tessellation");
}

#[test]
fn spline_geometry_is_retained_instead_of_replaced_by_facets() {
    let model = step::model_from_step_str(SPLINE_BOX).unwrap();
    assert!(model.bodies()[0]
        .brep()
        .faces
        .iter()
        .any(|f| matches!(f.surface, cad::Surface::Spline { .. })));
    let shapes = model.stock_shapes(0.01).unwrap();
    let engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(engine.total_volume(), 1000.0, 1e-8, "bilinear spline face box");
}

#[test]
fn file_length_units_are_converted_to_millimetres() {
    let metres = BOX.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)");
    let shapes = step::stock_from_step_str(&metres, 0.1).unwrap();
    assert_eq!(shapes[0].bounds().max, [10_000.0; 3]);
    assert_close(shapes[0].volume(), 1e12, 1e-9, "metres to cubic millimetres");
    let inches = extra_entities(
        &BOX.replace(
            "#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );",
            "#12=(CONVERSION_BASED_UNIT('inch',#9000)LENGTH_UNIT()NAMED_UNIT(#9002));",
        ),
        "#9000=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9001);\n#9001=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n#9002=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);",
    );
    let shapes = step::stock_from_step_str(&inches, 0.01).unwrap();
    assert_eq!(shapes[0].bounds().max, [254.0; 3]);
    assert_close(
        shapes[0].volume(),
        254.0_f64.powi(3),
        1e-9,
        "inches to cubic millimetres",
    );
}

#[test]
fn rational_boundary_trims_keep_the_exact_nonlinear_parameterisation() {
    let model = step::model_from_step_str(RATIONAL_BOX).unwrap();
    let body = &model.bodies()[0];
    let face = &body.brep().faces[0];
    assert!(matches!(face.surface, cad::Surface::RationalSpline { .. }));
    let cad::Loop::Edges { edges } = &face.loops[0] else {
        panic!("edge loop")
    };
    let edge = edges
        .iter()
        .find(|e| matches!(e.pcurve, cad::Curve2::RationalSpline { .. }))
        .unwrap();
    let [a, b] = edge.pcurve.interval().unwrap();
    let uv = solidcore_ops::pcurve::eval(&edge.pcurve, (a + b) * 0.5).unwrap();
    assert_close(uv[1], 1.0 / 3.0, 1e-12, "inverse rational boundary at half the 3D edge");
    let original = model.clone();
    assert_close(
        body.mesh(0.01).unwrap().signed_volume(),
        1000.0,
        1e-9,
        "rational face box",
    );
    assert_eq!(model, original);
    // Non-unit knot domains and weights varying in both directions keep the same box.
    let text = RATIONAL_BOX
        .replace("(0.,1.),(0.,1.)", "(2.,5.),(-3.,7.)")
        .replace("(((1.,2.),(1.,2.)))", "(((1.,2.),(3.,6.)))");
    let shapes = step::stock_from_step_str(&text, 0.01).unwrap();
    assert_close(shapes[0].volume(), 1000.0, 1e-9, "non-unit rational patch domain");
}

#[test]
fn higher_degree_nurbs_trims_are_fitted_without_replacing_the_surface() {
    // Same planar geometry with x=10*u^2: the inverse for the horizontal edges is
    // sqrt(x/10), outside the added rational bilinear boundary support.
    let text = RATIONAL_BOX
        .replace(
            "B_SPLINE_SURFACE(1,1,((#9001,#9002),(#9003,#9004))",
            "B_SPLINE_SURFACE(2,1,((#9001,#9002),(#9001,#9002),(#9003,#9004))",
        )
        .replace(
            "B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2)",
            "B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2)",
        )
        .replace("(((1.,2.),(1.,2.)))", "(((1.,2.),(1.,2.),(1.,2.)))");
    let model = step::model_from_step_str(&text).unwrap();
    assert!(matches!(
        model.bodies()[0].brep().faces[0].surface,
        cad::Surface::RationalSpline { .. }
    ));
    assert!(
        model.bodies()[0]
            .notes()
            .iter()
            .any(|n| n.contains("certified 3D deviation")),
        "{:?}",
        model.bodies()[0].notes()
    );
    assert_close(
        model.meshes(0.01).unwrap()[0].signed_volume(),
        1000.,
        1e-6,
        "fitted high-degree face box",
    );
}

#[test]
fn a_curved_high_degree_face_keeps_its_internal_hole_and_solid_stock() {
    let model = step::model_from_step_str(CURVED_HOLE).unwrap();
    let body = &model.bodies()[0];
    assert!(
        body.notes().iter().any(|n| n.contains("certified 3D deviation")),
        "{:?}",
        body.notes()
    );
    let top = body
        .brep()
        .faces
        .iter()
        .find(|f| matches!(&f.surface, cad::Surface::Spline { u_knots, .. } if u_knots.len()==10))
        .unwrap();
    assert_eq!(top.loops.len(), 2);
    let original = model.clone();
    let shapes = model.stock_shapes(0.001).unwrap();
    let mut engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    // Integral of 2+(x^2+y^2)/100 over [0,10]^2 excluding [3,7]^2.
    assert_close(
        engine.total_volume(),
        226.24,
        2e-4,
        "curved plate volume with through hole",
    );
    let result = engine
        .cut(
            &Tool::flat_end_mill(1., 1.),
            &MotionSegment::linear(Pose::vertical([5., 5., 0.5]), Pose::vertical([5., 5., 1.])),
        )
        .unwrap();
    assert_eq!(result.outcome, CutOutcome::NoChange);
    assert_eq!(model, original);
}

#[test]
fn a_closed_curved_internal_trim_keeps_the_hole_in_the_display_mesh() {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let xy = [
        [7., 5.],
        [7., 7.],
        [5., 7.],
        [3., 7.],
        [3., 5.],
        [3., 3.],
        [5., 3.],
        [7., 3.],
        [7., 5.],
    ];
    let mut extra = String::new();
    for (i, p) in xy.iter().enumerate() {
        extra.push_str(&format!(
            "#{}=CARTESIAN_POINT('',({:.17},{:.17},{:.17}));\n",
            9101 + i,
            p[0],
            p[1],
            1.54 + 0.1 * p[0] + 0.1 * p[1]
        ));
    }
    extra.push_str(&format!("#9200=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#9101,#9102,#9103,#9104,#9105,#9106,#9107,#9108,#9109),.UNSPECIFIED.,.T.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,{w:.17},1.,{w:.17},1.,{w:.17},1.,{w:.17},1.))REPRESENTATION_ITEM('circle on paraboloid'));\n"));
    extra.push_str("#9201=VERTEX_POINT('',#9101);\n#9202=EDGE_CURVE('',#9201,#9201,#9200,.T.);\n#9203=ORIENTED_EDGE('',*,*,#9202,.F.);\n#9204=EDGE_LOOP('',(#9203));\n#9205=FACE_BOUND('',#9204,.T.);\n#9206=OPEN_SHELL('',(#389));");
    let text = extra_entities(CURVED_HOLE, &extra)
        .replace(
            "#16=MANIFOLD_SOLID_BREP('curved plate with through hole',#396);",
            "#16=SHELL_BASED_SURFACE_MODEL('curved sheet with circular hole',(#9206));",
        )
        .replace("'curved top',(#370,#382)", "'curved top',(#370,#9205)");
    let model = step::model_from_step_str(&text).unwrap();
    let body = &model.bodies()[0];
    assert!(body.is_sheet());
    assert_eq!(body.brep().faces[0].loops.len(), 2);
    assert!(body.notes().iter().any(|n| n.contains("certified 3D deviation")));
    let mesh = body.mesh(0.001).unwrap();
    let mut area = 0.;
    for tri in &mesh.triangles {
        let [a, b, c] = tri.map(|i| mesh.positions[i as usize]);
        area += ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) * 0.5;
        let centroid = [0, 1].map(|i| (a[i] + b[i] + c[i]) / 3.);
        assert!((centroid[0] - 5.).hypot(centroid[1] - 5.) > 1.98, "hole was filled");
    }
    assert!(
        (area - (100. - 4. * std::f64::consts::PI)).abs() < 0.02,
        "projected area {area}"
    );
}

#[cfg(feature = "serde")]
#[test]
fn fitted_trim_records_and_their_diagnostics_survive_json() {
    let model = step::model_from_step_str(CURVED_HOLE).unwrap();
    let restored: step::StepModel = serde_json::from_str(&serde_json::to_string(&model).unwrap()).unwrap();
    assert_eq!(restored.bodies()[0].notes(), model.bodies()[0].notes());
    assert_eq!(
        restored.bodies()[0].brep().faces.len(),
        model.bodies()[0].brep().faces.len()
    );
    assert_close(
        restored.meshes(0.005).unwrap()[0].signed_volume(),
        model.meshes(0.005).unwrap()[0].signed_volume(),
        1e-9,
        "fitted CAD JSON",
    );
}

#[test]
fn flat_multi_body_models_keep_every_body_in_entity_order() {
    let text = extra_entities(BOX, "#9000=MANIFOLD_SOLID_BREP('second',#17);");
    let model = step::model_from_step_str(&text).unwrap();
    assert_eq!(
        model.bodies().iter().map(|b| b.entity()).collect::<Vec<_>>(),
        [16, 9000]
    );
    assert_eq!(model.bodies()[1].name(), "second");
    assert_eq!(model.meshes(0.01).unwrap().len(), 2);
    assert_eq!(model.stock_shapes(0.01).unwrap().len(), 2);
}

#[test]
fn an_unreadable_body_rejects_the_whole_import() {
    let text = extra_entities(BOX, "#9000=MANIFOLD_SOLID_BREP('broken',#999999);");
    let error = step::stock_from_step_str(&text, 0.01).unwrap_err().to_string();
    assert!(error.contains("#9000"), "{error}");
    assert!(step::model_from_step_str(&text).is_err());
}

#[test]
fn an_unsupported_face_does_not_silently_remove_a_body() {
    let text = BOX.replace("#80 = PLANE('', #81);", "#80=OFFSET_SURFACE('',#85,1.,.F.);");
    let error = step::meshes_from_step_str(&text, 0.01).unwrap_err().to_string();
    assert!(error.contains("#16"), "{error}");
}

#[test]
fn unreachable_mapped_items_are_rejected() {
    let text = extra_entities(BOX, "#9000=MAPPED_ITEM('placed',#9001,#9002);");
    assert!(step::model_from_step_str(&text)
        .unwrap_err()
        .to_string()
        .contains("not reachable"));
}

fn assert_bounds(mesh: &cutrail::TriMesh, min: [f64; 3], max: [f64; 3]) {
    let bounds = mesh.bounds();
    for i in 0..3 {
        assert!(
            (bounds.min[i] - min[i]).abs() < 1e-9,
            "world minimum: {bounds:?} != {min:?}"
        );
        assert!(
            (bounds.max[i] - max[i]).abs() < 1e-9,
            "world maximum: {bounds:?} != {max:?}"
        );
    }
}

#[test]
fn assembly_instances_compose_local_placements_before_the_parent_world_transform() {
    let model = step::model_from_step_str(ASSEMBLY).unwrap();
    assert_eq!(model.bodies().len(), 2);
    let nested = &model.bodies()[0];
    let direct = &model.bodies()[1];
    assert_eq!(nested.instance_path(), [9002, 9200, 9400]);
    assert_eq!(direct.instance_path(), [9002, 9600]);
    assert_eq!(nested.entity(), direct.entity());
    assert_eq!(
        nested.brep(),
        step::model_from_step_str(BOX).unwrap().bodies()[0].brep()
    );
    assert_eq!(nested.brep(), direct.brep());
    // (3,4,0) in the subassembly, then +90 degrees and (30,40,5) in world.
    assert_bounds(&nested.mesh(0.01).unwrap(), [16., 43., 5.], [26., 53., 15.]);
    assert_bounds(&direct.mesh(0.01).unwrap(), [-20., 0., 0.], [-10., 10., 10.]);
    let original = model.clone();
    let mut engine = Engine::new(
        ManifoldKernel,
        &model.stock_shapes(0.01).unwrap(),
        EngineOptions::default(),
    )
    .unwrap();
    assert_close(engine.total_volume(), 2000.0, 1e-9, "both placed stocks");
    let tool = Tool::flat_end_mill(4., 6.);
    let segment = MotionSegment::linear(Pose::vertical([21., 40., 11.]), Pose::vertical([21., 56., 11.]));
    engine.cut(&tool, &segment).unwrap();
    let mut baseline = Engine::new(
        ManifoldKernel,
        &[
            cutrail::StockShape::Box {
                min: [16., 43., 5.],
                max: [26., 53., 15.],
            },
            cutrail::StockShape::Box {
                min: [-20., 0., 0.],
                max: [-10., 10., 10.],
            },
        ],
        EngineOptions::default(),
    )
    .unwrap();
    baseline.cut(&tool, &segment).unwrap();
    // Compare against independent world-space primitives. The swept tool uses an
    // area-equivalent polygon, whose width differs slightly from the ideal cylinder.
    assert_close(
        engine.total_volume(),
        baseline.total_volume(),
        1e-10,
        "slot cut in the world-placed nested part",
    );
    assert!(engine.total_volume() < 1900. && engine.total_volume() > 1800.);
    assert_eq!(model, original);
}

#[test]
fn reversed_representation_relationships_preserve_the_same_assembly_placement() {
    let reversed = ASSEMBLY.replace("#10,#9305,#9404", "#9305,#10,#9404").replace(
        "#9404=ITEM_DEFINED_TRANSFORMATION('','',#9100,#9405)",
        "#9404=ITEM_DEFINED_TRANSFORMATION('','',#9405,#9100)",
    );
    assert_eq!(
        step::model_from_step_str(&reversed).unwrap(),
        step::model_from_step_str(ASSEMBLY).unwrap()
    );
}

#[test]
fn occurrence_links_can_use_a_transformed_alias_of_the_definition_shape() {
    let text = extra_entities(&ASSEMBLY.replace("SHAPE_DEFINITION_REPRESENTATION(#9303,#9305)", "SHAPE_DEFINITION_REPRESENTATION(#9303,#9800)"),
        "#9800=SHAPE_REPRESENTATION('alias',(#9100),#11);\n#9801=REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION('','',#9305,#9800,#9802);\n#9802=ITEM_DEFINED_TRANSFORMATION('','',#9100,#9803);\n#9803=AXIS2_PLACEMENT_3D('',#9804,#83,#84);\n#9804=CARTESIAN_POINT('',(10.,0.,0.));");
    let model = step::model_from_step_str(&text).unwrap();
    assert_eq!(
        model.meshes(0.01).unwrap(),
        step::model_from_step_str(ASSEMBLY).unwrap().meshes(0.01).unwrap()
    );
}

#[test]
fn nested_mapped_items_place_rotated_map_origins_onto_their_targets() {
    let model = step::model_from_step_str(MAPPED).unwrap();
    assert_eq!(model.bodies().len(), 2);
    assert_eq!(model.bodies()[0].instance_path(), [5, 9000]);
    assert_eq!(model.bodies()[1].instance_path(), [5, 9010, 9311]);
    assert_bounds(&model.bodies()[0].mesh(0.01).unwrap(), [2., 14., 5.], [22., 34., 25.]);
    assert_bounds(&model.bodies()[1].mesh(0.01).unwrap(), [-31., 0., 0.], [-21., 10., 10.]);
    assert_close(
        model.bodies()[0].mesh(0.01).unwrap().signed_volume(),
        8000.,
        1e-10,
        "scaled mapped instance",
    );
    assert_eq!(model.bodies()[0].brep(), model.bodies()[1].brep());
}

#[test]
fn omitted_operator_axes_are_derived_instead_of_independently_defaulting_to_global_axes() {
    let original = "#9001=CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',#9002,#9003,#9004,2.,#83)";
    // X maps to Y, Y maps to Z; the omitted Z must be X, not global Z.
    let missing_z = MAPPED.replace(
        original,
        "#9001=CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',#9203,#83,#9004,2.,$)",
    );
    let model = step::model_from_step_str(&missing_z).unwrap();
    assert_bounds(&model.bodies()[0].mesh(0.01).unwrap(), [20., 26., -13.], [40., 46., 7.]);
    // X is Y and Z is Z; the omitted Y must be -X, not global Y.
    let missing_y = MAPPED.replace(
        original,
        "#9001=CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',#9203,$,#9004,2.,#83)",
    );
    let model = step::model_from_step_str(&missing_y).unwrap();
    assert_bounds(&model.bodies()[0].mesh(0.01).unwrap(), [18., 26., 5.], [38., 46., 25.]);
}

#[test]
fn a_shared_body_with_conflicting_source_units_is_rejected() {
    let text = extra_entities(BOX,
        "#9800=ADVANCED_BREP_SHAPE_REPRESENTATION('metres',(#16),#9801);\n#9801=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#9802,#13,#14))REPRESENTATION_CONTEXT('metres','3D'));\n#9802=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));");
    let error = step::model_from_step_str(&text).unwrap_err().to_string();
    assert!(error.contains("conflicting units"), "{error}");
}

#[test]
fn scaled_curved_instances_use_a_world_coordinate_chord_tolerance() {
    let additions = MAPPED
        .split("#9000=MAPPED_ITEM")
        .nth(1)
        .unwrap()
        .split("ENDSEC;")
        .next()
        .unwrap();
    let additions = format!("#9000=MAPPED_ITEM{additions}")
        .replace(",#83", ",#92")
        .replace("REPRESENTATION_MAP(#81,#9310)", "REPRESENTATION_MAP(#60,#9310)");
    let text = extra_entities(CYLINDER, &additions)
        .replace(
            "#10 = ADVANCED_BREP_SHAPE_REPRESENTATION('', (#16), #11);",
            "#10=SHAPE_REPRESENTATION('mapped root',(#9000,#9010),#11);",
        )
        .replace("#9004,2.,#92", "#9004,100.,#92");
    let model = step::model_from_step_str(&text).unwrap();
    let body = &model.bodies()[0];
    let mesh = body.mesh(0.01).unwrap();
    let center = body.transform()[3];
    let mut wall_edges = 0;
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = mesh.positions[tri[i] as usize];
            let b = mesh.positions[tri[(i + 1) % 3] as usize];
            if (a[2] - b[2]).abs() < 1. {
                continue;
            }
            let radius = ((a[0] + b[0]) * 0.5 - center[0]).hypot((a[1] + b[1]) * 0.5 - center[1]);
            assert!(
                (500. - radius).abs() <= 0.010_001,
                "world cylinder sag {} exceeds 0.01 mm",
                500. - radius
            );
            wall_edges += 1;
        }
    }
    assert!(wall_edges > 0);
}

#[test]
fn mirrored_nonuniform_instances_keep_outward_winding_and_positive_stock_volume() {
    let text = MAPPED.replace(
        "#9001=CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',#9002,#9003,#9004,2.,#83)",
        "#9001=CARTESIAN_TRANSFORMATION_OPERATOR_3D_NON_UNIFORM('','',#9012,#9203,#9004,2.,#83,3.,4.)",
    );
    let model = step::model_from_step_str(&text).unwrap();
    let mesh = model.bodies()[0].mesh(0.01).unwrap();
    assert_bounds(&mesh, [4., 3., 5.], [24., 33., 45.]);
    assert_close(
        mesh.signed_volume(),
        24000.,
        1e-10,
        "mirror reverses orientation, not volume sign",
    );
    let engine = Engine::new(
        ManifoldKernel,
        &model.stock_shapes(0.01).unwrap(),
        EngineOptions::default(),
    )
    .unwrap();
    assert_close(
        engine.total_volume(),
        25000.,
        1e-9,
        "mirrored stock plus second instance",
    );
}

#[test]
fn assembly_placement_origins_use_their_own_representation_units() {
    // Only the root representation is in metres; source CAD and subassembly remain mm.
    let text = extra_entities(&ASSEMBLY.replace("'root',(#9100),#11", "'root',(#9100),#9800"),
        "#9800=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#9801,#13,#14))REPRESENTATION_CONTEXT('metres','3D'));\n#9801=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));")
        .replace("(30.,40.,5.)", "(0.030,0.040,0.005)")
        .replace("(-20.,0.,0.)", "(-0.020,0.,0.)");
    let model = step::model_from_step_str(&text).unwrap();
    assert_bounds(&model.bodies()[0].mesh(0.01).unwrap(), [16., 43., 5.], [26., 53., 15.]);
    assert_bounds(&model.bodies()[1].mesh(0.01).unwrap(), [-20., 0., 0.], [-10., 10., 10.]);
}

#[test]
fn missing_conflicting_singular_and_cyclic_placements_reject_the_whole_model() {
    let missing = ASSEMBLY.replace("#9402=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#9403,#9401);", "");
    assert!(step::model_from_step_str(&missing)
        .unwrap_err()
        .to_string()
        .contains("no placement"));
    let conflicting = extra_entities(ASSEMBLY,
        "#9800=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#9801,#9401);\n#9801=REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION('','',#10,#9305,#9604);");
    assert!(step::model_from_step_str(&conflicting)
        .unwrap_err()
        .to_string()
        .contains("conflicting"));
    let singular = MAPPED.replace("#9003=DIRECTION('',(1.,0.,0.))", "#9003=DIRECTION('',(0.,-1.,0.))");
    assert!(step::model_from_step_str(&singular).is_err());
    let cyclic = ASSEMBLY.replace("#9302,#5,$", "#9302,#9002,$");
    assert!(step::model_from_step_str(&cyclic).is_err());
    let mapped_cycle = MAPPED.replace("REPRESENTATION_MAP(#81,#9310)", "REPRESENTATION_MAP(#81,#10)");
    assert!(step::model_from_step_str(&mapped_cycle).is_err());
}

#[test]
fn transformed_representation_links_without_occurrences_place_geometry() {
    let text = extra_entities(&BOX.replace("SHAPE_DEFINITION_REPRESENTATION(#4, #10)", "SHAPE_DEFINITION_REPRESENTATION(#4, #9000)"),
        "#9000=SHAPE_REPRESENTATION('root',(#81),#11);\n#9001=REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION('','',#10,#9000,#9002);\n#9002=ITEM_DEFINED_TRANSFORMATION('','',#81,#9003);\n#9003=AXIS2_PLACEMENT_3D('',#9004,#83,#84);\n#9004=CARTESIAN_POINT('',(3.,4.,5.));");
    let model = step::model_from_step_str(&text).unwrap();
    assert_eq!(model.bodies().len(), 1);
    assert_bounds(&model.bodies()[0].mesh(0.01).unwrap(), [3., 4., 5.], [13., 14., 15.]);
}

#[cfg(feature = "serde")]
#[test]
fn assembly_json_preserves_placements_and_rejects_invalid_matrices() {
    let model = step::model_from_step_str(ASSEMBLY).unwrap();
    let json = serde_json::to_string(&model).unwrap();
    let restored: step::StepModel = serde_json::from_str(&json).unwrap();
    assert_eq!(restored, model);
    assert_eq!(restored.meshes(0.01).unwrap(), model.meshes(0.01).unwrap());
    let mut malformed = serde_json::to_value(&model).unwrap();
    malformed["bodies"][0]["transform"][0][0] = 0.into();
    malformed["bodies"][0]["transform"][0][1] = 0.into();
    let malformed: step::StepModel = serde_json::from_value(malformed).unwrap();
    assert!(malformed.meshes(0.01).is_err());
    // Saved flat models from before placement support still mean identity.
    let mut old = serde_json::to_value(step::model_from_step_str(BOX).unwrap()).unwrap();
    old["bodies"][0].as_object_mut().unwrap().remove("transform");
    old["bodies"][0].as_object_mut().unwrap().remove("instance_path");
    let old: step::StepModel = serde_json::from_value(old).unwrap();
    assert_bounds(&old.meshes(0.01).unwrap()[0], [0.; 3], [10.; 3]);
}

#[test]
fn mixed_csg_and_brep_stock_is_rejected_instead_of_dropping_the_csg_body() {
    let text = extra_entities(BOX, "#9000=CSG_SOLID('unsupported',#999999);");
    let error = step::model_from_step_str(&text).unwrap_err();
    assert!(matches!(error, Error::Unsupported(_)));
    assert!(error.to_string().contains("#9000"));
}

#[test]
fn unknown_shape_items_cannot_be_silently_skipped() {
    let text = extra_entities(BOX, "#9000=TRIANGULATED_FACE_SET('unsupported',#999999);")
        .replace("('', (#16), #11)", "('', (#16,#9000), #11)");
    let error = step::model_from_step_str(&text).unwrap_err();
    assert!(matches!(error, Error::Unsupported(_)));
    assert!(error.to_string().contains("#9000"));
}

#[test]
fn complex_body_subtypes_cannot_hide_their_void_shells() {
    let text = VOID.replace(
        "#16 = BREP_WITH_VOIDS('', #17, (#166));",
        "#16=(BREP_WITH_VOIDS((#166))FACETED_BREP()GEOMETRIC_REPRESENTATION_ITEM()MANIFOLD_SOLID_BREP(#17)REPRESENTATION_ITEM('complex body')SOLID_MODEL());",
    );
    let error = step::stock_from_step_str(&text, 0.01).unwrap_err();
    assert!(matches!(error, Error::Unsupported(_)));
    assert!(error.to_string().contains("#16"));
}

#[test]
fn open_sheets_are_available_as_cad_but_cannot_be_stock() {
    let text = BOX
        .replace(
            "#16 = MANIFOLD_SOLID_BREP('', #17);",
            "#16=SHELL_BASED_SURFACE_MODEL('sheet',(#17));",
        )
        .replace(
            "#17 = CLOSED_SHELL('', (#18, #25, #32, #39, #46, #53));",
            "#17=OPEN_SHELL('',(#18));",
        );
    let model = step::model_from_step_str(&text).unwrap();
    assert!(model.bodies()[0].is_sheet());
    assert_eq!(model.bodies()[0].brep().faces.len(), 1);
    assert!(!model.meshes(0.01).unwrap()[0].triangles.is_empty());
    assert!(model.stock_shapes(0.01).is_err());
    assert!(step::stock_from_step_str(&text, 0.01).is_err());
}

#[test]
fn invalid_tolerances_and_non_step_input_are_rejected() {
    for tolerance in [0.0, -0.01, f64::NAN, f64::INFINITY] {
        assert!(step::stock_from_step_str(BOX, tolerance).is_err());
        assert!(step::meshes_from_step_str(BOX, tolerance).is_err());
        assert!(step::model_from_step_str(BOX).unwrap().bodies()[0]
            .mesh(tolerance)
            .is_err());
    }
    assert!(step::stock_from_step_str("not a step file", 0.01).is_err());
    assert!(step::model_from_step_str("ISO-10303-21;HEADER;ENDSEC;DATA;ENDSEC;END-ISO-10303-21;").is_err());
}

#[test]
fn file_helpers_use_the_same_retained_model_and_mesh_path() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/step/box.step");
    let model = step::model_from_step_file(path).unwrap();
    assert_eq!(model, step::model_from_step_str(BOX).unwrap());
    assert_eq!(
        step::meshes_from_step_file(path, 0.01).unwrap(),
        model.meshes(0.01).unwrap()
    );
    assert_eq!(
        step::stock_from_step_file(path, 0.01).unwrap(),
        model.stock_shapes(0.01).unwrap()
    );
}

#[cfg(feature = "serde")]
#[test]
fn retained_cad_round_trips_through_json_and_revalidates_records() {
    let model = step::model_from_step_str(CYLINDER).unwrap();
    let json = serde_json::to_string(&model).unwrap();
    let restored: step::StepModel = serde_json::from_str(&json).unwrap();
    assert_eq!(restored, model);
    assert_eq!(restored.meshes(0.01).unwrap(), model.meshes(0.01).unwrap());
    let mut malformed = serde_json::to_value(&model).unwrap();
    malformed["bodies"][0]["brep"]["version"] = 0.into();
    let malformed: step::StepModel = serde_json::from_value(malformed).unwrap();
    assert!(malformed.stock_shapes(0.01).is_err());
}
