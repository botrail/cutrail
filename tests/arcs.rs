//! Arc and helical moves: chord subdivision, direction, and removed volumes.
mod common;
use common::{assert_close, cube10, grid_removed};
use cutrail::sweep::swept_volume;
use cutrail::{CutOutcome, EngineOptions, MotionSegment, Pose, Tool};
use std::f64::consts::PI;

#[test]
fn quarter_arc_matches_grid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let seg = MotionSegment::arc(
        Pose::vertical([8.0, 5.0, 6.0]),
        Pose::vertical([5.0, 8.0, 6.0]),
        [5.0, 5.0, 6.0],
        [0.0, 0.0, 1.0],
    );
    let v = swept_volume(&tool, &seg, 0.01).unwrap();
    assert!(v.piece_count() > 4, "arc must be chord-subdivided: {}", v.piece_count());
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 256, 0.05);
    assert_close(r.removed_volume, expected, 1e-2, "quarter arc removed volume");
}

#[test]
fn full_circle_matches_analytic() {
    // A tool of radius 1 once around a circle of radius 3: an annulus π(4²-2²), 4 deep
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let p = Pose::vertical([8.0, 5.0, 6.0]);
    let seg = MotionSegment::arc(p, p, [5.0, 5.0, 6.0], [0.0, 0.0, 1.0]);
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    assert_close(
        r.removed_volume,
        PI * (16.0 - 4.0) * 4.0,
        1e-2,
        "full circle removed volume",
    );
}

#[test]
fn clockwise_arc_via_flipped_normal() {
    // Flipping the normal reverses the direction; a quarter turn removes the same volume either way
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let ccw = MotionSegment::arc(
        Pose::vertical([8.0, 5.0, 6.0]),
        Pose::vertical([5.0, 8.0, 6.0]),
        [5.0, 5.0, 6.0],
        [0.0, 0.0, 1.0],
    );
    let cw = MotionSegment::arc(
        Pose::vertical([5.0, 8.0, 6.0]),
        Pose::vertical([8.0, 5.0, 6.0]),
        [5.0, 5.0, 6.0],
        [0.0, 0.0, -1.0],
    );
    let mut a = cube10(EngineOptions::default());
    let mut b = cube10(EngineOptions::default());
    let ra = a.cut(&tool, &ccw).unwrap();
    let rb = b.cut(&tool, &cw).unwrap();
    assert_close(ra.removed_volume, rb.removed_volume, 1e-6, "cw vs ccw");
    let mid = cw.pose_at(0.5);
    assert_close(mid.tip[0], 5.0 + 3.0 * (PI / 4.0).cos(), 1e-9, "midpoint x");
}

#[test]
fn helical_descent_matches_grid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let seg = MotionSegment::arc(
        Pose::vertical([8.0, 5.0, 10.0]),
        Pose::vertical([5.0, 8.0, 8.0]),
        [5.0, 5.0, 9.0],
        [0.0, 0.0, 1.0],
    );
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 256, 0.05);
    assert_close(r.removed_volume, expected, 1e-2, "helical removed volume");
}

#[test]
fn arc_with_mismatched_radius_is_rejected() {
    let seg = MotionSegment::arc(
        Pose::vertical([8.0, 5.0, 6.0]),
        Pose::vertical([5.0, 9.0, 6.0]),
        [5.0, 5.0, 6.0],
        [0.0, 0.0, 1.0],
    );
    assert!(seg.validate().is_err());
}
