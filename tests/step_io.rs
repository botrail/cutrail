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
fn unsupported_rational_trims_are_reported_instead_of_using_a_lossy_import() {
    // This valid planar NURBS patch reparameterizes a straight boundary nonlinearly.
    // The pinned kernel has no exact pcurve for it. Keep that capability boundary explicit.
    let error = step::model_from_step_str(RATIONAL_BOX).unwrap_err().to_string();
    assert!(error.contains("STEP body #16"), "{error}");
    assert!(error.contains("pcurve"), "{error}");
    assert!(step::stock_from_step_str(RATIONAL_BOX, 0.01).is_err());
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
fn mapped_instances_are_rejected_until_placements_are_supported() {
    let text = extra_entities(BOX, "#9000=MAPPED_ITEM('placed',#9001,#9002);");
    assert!(matches!(step::model_from_step_str(&text), Err(Error::Unsupported(_))));
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
