//! Ball and bull-nose end mills: removed volumes against analytic values and grid sampling.
mod common;
use common::{assert_close, cube10, grid_removed};
use cutrail::{CutOutcome, EngineOptions, MotionSegment, Pose, Tool};
use std::f64::consts::PI;

#[test]
fn ball_plunge_matches_analytic() {
    // A ball of radius 2 plunged to a tip height of z=6: hemisphere (2/3)π8 + cylinder π4·2
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 6.0);
    let r = e
        .cut(
            &tool,
            &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0])),
        )
        .unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let expected = 2.0 / 3.0 * PI * 8.0 + PI * 4.0 * 2.0;
    assert_close(r.removed_volume, expected, 2e-2, "ball plunge removed volume");
}

#[test]
fn ball_slot_matches_analytic() {
    // Cross-section: a half disk (π·4/2) plus a rectangle (4×2), over a length of 10
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 6.0);
    let r = e
        .cut(
            &tool,
            &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0])),
        )
        .unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let expected = (PI * 4.0 / 2.0 + 4.0 * 2.0) * 10.0;
    assert_close(r.removed_volume, expected, 2e-2, "ball slot removed volume");
}

#[test]
fn bull_nose_plunge_matches_grid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::bull_nose_end_mill(6.0, 1.0, 6.0);
    let seg = MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]));
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    // Analytic value: the corner ∫0^1 π r(h)² dh with r(h) = 2 + sqrt(1-(1-h)²), plus the cylinder π·9·3
    let steps = 20000;
    let mut corner = 0.0;
    for k in 0..steps {
        let h = (k as f64 + 0.5) / steps as f64;
        let rr = 2.0 + (1.0 - (1.0 - h).powi(2)).sqrt();
        corner += PI * rr * rr / steps as f64;
    }
    let expected = corner + PI * 9.0 * 3.0;
    assert_close(r.removed_volume, expected, 2e-2, "bull nose plunge removed volume");
    let grid = grid_removed(&tool, &[seg], 1, 0.05);
    assert_close(grid, expected, 1e-2, "grid estimator sanity");
}

#[test]
fn tilted_ball_sweep_matches_grid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 8.0);
    let seg = MotionSegment::linear(
        Pose::tilted_about_y([2.0, 5.0, 7.0], -20.0),
        Pose::tilted_about_y([8.0, 5.0, 7.0], 20.0),
    );
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 64, 0.1);
    assert_close(r.removed_volume, expected, 2e-2, "tilted ball sweep");
}

#[test]
fn invalid_tools_are_rejected() {
    assert!(Tool::ball_end_mill(4.0, 1.0).validate().is_err());
    assert!(Tool::bull_nose_end_mill(6.0, 4.0, 6.0).validate().is_err());
    assert!(Tool::bull_nose_end_mill(6.0, 1.0, 6.0).validate().is_ok());
}
