//! Shank and holder collision detection.
mod common;
use common::{assert_close, cube10};
use cutrail::{CutOutcome, EngineOptions, Holder, MotionSegment, Pose, Shank, Tool};
use std::f64::consts::PI;

#[test]
fn shank_collision_is_reported_after_cutting() {
    // Flute Ø4 of length 6, shank Ø8 of length 10. Plunging to z=2 puts the shank (z 8..18) into the stock over z 8..10
    let tool = Tool::flat_end_mill(4.0, 6.0).with_shank(Shank { diameter: 8.0, length: 10.0 });
    let mut e = cube10(EngineOptions::default().with_collision_check());
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 2.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_close(r.removed_volume, PI * 4.0 * 8.0, 1e-2, "flute removal");
    let c = r.collision.expect("collision expected");
    assert_close(c.volume, PI * (16.0 - 4.0) * 2.0, 2e-2, "collision volume");
    assert!(c.bounds.min[2] >= 7.9 && c.bounds.max[2] <= 10.1, "{:?}", c.bounds);
}

#[test]
fn no_collision_when_shank_stays_outside_or_check_is_off() {
    let tool = Tool::flat_end_mill(4.0, 6.0).with_shank(Shank { diameter: 8.0, length: 10.0 });
    let mut e = cube10(EngineOptions::default().with_collision_check());
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]))).unwrap();
    assert!(r.collision.is_none(), "{:?}", r.collision);
    let mut off = cube10(EngineOptions::default());
    let r = off.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 2.0]))).unwrap();
    assert!(r.collision.is_none());
}

#[test]
fn holder_collision_in_tiled_mode() {
    // Flute Ø4 of length 3, shank Ø4 of length 2 (same diameter as the hole, so no contact), holder Ø10 of length 5 (hits over z 6..11)
    let tool = Tool::flat_end_mill(4.0, 3.0)
        .with_shank(Shank { diameter: 4.0, length: 2.0 })
        .with_holder(Holder { diameter: 10.0, length: 5.0 });
    let mut e = cube10(EngineOptions::default().with_tile_size(4.0).with_collision_check());
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 1.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let c = r.collision.expect("holder collision expected");
    assert_close(c.volume, PI * (25.0 - 4.0) * 4.0, 2e-2, "holder collision volume");
}
