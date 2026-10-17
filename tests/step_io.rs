//! STEP 素材の読込（feature `stepio`）。truck で書いた STEP を読み戻して体積を確認する。
#![cfg(feature = "stepio")]
use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, Tool};
use truck_modeling::*;
use truck_stepio::out::{CompleteStepDisplay, StepHeaderDescriptor, StepModel};

fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

fn step_of(solid: &Solid) -> String {
    let compressed = solid.compress();
    CompleteStepDisplay::new(StepModel::from(&compressed), StepHeaderDescriptor::default()).to_string()
}

#[test]
fn box_round_trip_through_step() {
    let v = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let e = builder::tsweep(&v, Vector3::unit_x() * 10.0);
    let f = builder::tsweep(&e, Vector3::unit_y() * 10.0);
    let cube: Solid = builder::tsweep(&f, Vector3::unit_z() * 10.0);
    let step = step_of(&cube);
    assert!(step.starts_with("ISO-10303-21;"));
    let shapes = cutrail::step::stock_from_step_str(&step, 0.01).unwrap();
    assert_eq!(shapes.len(), 1);
    let mut engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(engine.total_volume(), 1000.0, 1e-6, "STEP box volume");
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = engine.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_close(engine.total_volume(), 840.0, 1e-3, "slot in STEP stock");
}

#[test]
fn cylinder_round_trip_through_step() {
    let v = builder::vertex(Point3::new(5.0, 0.0, 0.0));
    let w = builder::rsweep(&v, Point3::origin(), Vector3::unit_z(), Rad(7.0));
    let f = builder::try_attach_plane(&[w]).unwrap();
    let cylinder: Solid = builder::tsweep(&f, Vector3::unit_z() * 10.0);
    let step = step_of(&cylinder);
    let shapes = cutrail::step::stock_from_step_str(&step, 0.01).unwrap();
    let engine = Engine::new(ManifoldKernel, &shapes, EngineOptions::default()).unwrap();
    assert_close(engine.total_volume(), std::f64::consts::PI * 25.0 * 10.0, 1e-2, "STEP cylinder volume");
}

#[test]
fn garbage_is_rejected() {
    assert!(cutrail::step::stock_from_step_str("not a step file", 0.01).is_err());
}
