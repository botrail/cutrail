//! Arbitrary tool axes and segments whose axis rotates (5-axis); expected volumes come from grid sampling.
mod common;
use common::{assert_close};
use cutrail::sweep::{rotation_steps, swept_volume};
use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};
use std::f64::consts::PI;


/// Whether point `p` lies inside the tool (a flat-bottomed cylinder) at `pose`.
fn inside(tool: &Tool, pose: &Pose, p: [f64; 3]) -> bool {
    let a = pose.axis;
    let d = [p[0] - pose.tip[0], p[1] - pose.tip[1], p[2] - pose.tip[2]];
    let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
    if h < 0.0 || h > tool.flute_length() {
        return false;
    }
    let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - h * h;
    r2 <= tool.radius() * tool.radius()
}

/// Grid estimate of the volume inside the box [min,max] covered by the sweep of `segments`, interpolating `sub` sub-steps per segment.
fn grid_removed(min: [f64; 3], max: [f64; 3], tool: &Tool, segments: &[MotionSegment], sub: usize, spacing: f64) -> f64 {
    let n = [
        ((max[0] - min[0]) / spacing).ceil() as usize,
        ((max[1] - min[1]) / spacing).ceil() as usize,
        ((max[2] - min[2]) / spacing).ceil() as usize,
    ];
    // Precompute the interpolated poses and the AABB of each segment
    let reach = (tool.flute_length().powi(2) + tool.radius().powi(2)).sqrt();
    let items: Vec<(Vec<Pose>, [f64; 3], [f64; 3])> = segments
        .iter()
        .map(|seg| {
            let (from, to) = seg.endpoints();
            let poses: Vec<Pose> = (0..=sub).map(|k| Pose::lerp(from, to, k as f64 / sub as f64)).collect();
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for p in &poses {
                for i in 0..3 {
                    lo[i] = lo[i].min(p.tip[i] - reach);
                    hi[i] = hi[i].max(p.tip[i] + reach);
                }
            }
            (poses, lo, hi)
        })
        .collect();
    let mut count = 0usize;
    for ix in 0..n[0] {
        let x = min[0] + (ix as f64 + 0.5) * spacing;
        for iy in 0..n[1] {
            let y = min[1] + (iy as f64 + 0.5) * spacing;
            for iz in 0..n[2] {
                let z = min[2] + (iz as f64 + 0.5) * spacing;
                let p = [x, y, z];
                let covered = items.iter().any(|(poses, lo, hi)| {
                    (0..3).all(|i| p[i] >= lo[i] && p[i] <= hi[i]) && poses.iter().any(|pose| inside(tool, pose, p))
                });
                if covered {
                    count += 1;
                }
            }
        }
    }
    count as f64 * spacing * spacing * spacing
}

fn cube10(tile: Option<f64>) -> Engine {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    let options = EngineOptions { tile_size: tile, ..Default::default() };
    Engine::new(ManifoldKernel, &[stock], options).unwrap()
}

#[test]
fn rotation_steps_follow_chord_tolerance() {
    assert_eq!(rotation_steps(0.0, 10.0, 0.01), 1);
    // A 20° rotation with reach 10.2 and chord tolerance 0.01: 5.08° per step, so 4 steps
    assert_eq!(rotation_steps(20f64.to_radians(), 10.2, 0.01), 4);
    assert!(rotation_steps(PI / 2.0, 100.0, 1e-6) <= 256);
}

#[test]
fn fixed_tilt_translation_is_a_single_hull() {
    let tool = Tool::flat_end_mill(3.0, 20.0);
    let a = Pose::tilted_about_y([5.0, 5.0, -5.0], 30.0);
    let b = Pose::tilted_about_y([5.0, 5.0, 15.0], 30.0);
    let v = swept_volume(&tool, &MotionSegment::linear(a, b), 0.01).unwrap();
    assert_eq!(v.piece_count(), 1);
}

#[test]
fn tilted_through_hole_matches_analytic() {
    // A radius-1.5 tool tilted 30° fed along its axis through the stock: removes πr²·10/cos30°
    let mut e = cube10(None);
    let tool = Tool::flat_end_mill(3.0, 10.0);
    let a = Pose::tilted_about_y([5.0 - 10.0 * 30f64.to_radians().sin(), 5.0, 5.0 - 10.0 * 30f64.to_radians().cos()], 30.0);
    let b = Pose::tilted_about_y([5.0 + 2.0 * 30f64.to_radians().sin(), 5.0, 5.0 + 2.0 * 30f64.to_radians().cos()], 30.0);
    let r = e.cut(&tool, &MotionSegment::linear(a, b)).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let removed = PI * 1.5 * 1.5 * 10.0 / 30f64.to_radians().cos();
    assert_close(e.total_volume(), 1000.0 - removed, 5e-3, "remaining volume");
}

#[test]
fn rotation_about_tip_matches_grid() {
    // Tip fixed at (5,5,6) while the axis rotates from +Z to 40°; the swept volume is compared with grid sampling
    let mut e = cube10(None);
    let tool = Tool::flat_end_mill(4.0, 8.0);
    let a = Pose::vertical([5.0, 5.0, 6.0]);
    let b = Pose::tilted_about_y([5.0, 5.0, 6.0], 40.0);
    let seg = MotionSegment::linear(a, b);
    let v = swept_volume(&tool, &seg, 0.01).unwrap();
    assert!(v.piece_count() > 1, "rotation must be sub-stepped");
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed([0.0; 3], [10.0; 3], &tool, &[seg], 64, 0.1);
    assert_close(r.removed_volume, expected, 2e-2, "removed volume");
}

#[test]
fn rotation_with_translation_matches_grid_in_whole_and_tiled_modes() {
    let tool = Tool::flat_end_mill(4.0, 8.0);
    let a = Pose::tilted_about_y([2.0, 5.0, 7.0], -25.0);
    let b = Pose::tilted_about_y([8.0, 5.0, 7.0], 25.0);
    let seg = MotionSegment::linear(a, b);
    let expected = grid_removed([0.0; 3], [10.0; 3], &tool, std::slice::from_ref(&seg), 64, 0.1);
    for tile in [None, Some(4.0)] {
        let mut e = cube10(tile);
        let r = e.cut(&tool, &seg).unwrap();
        assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
        assert_close(r.removed_volume, expected, 2e-2, &format!("removed volume (tile {tile:?})"));
    }
}

#[test]
fn tilted_sweep_sequence_like_p0b() {
    // Feed the tip 1 mm at a time while the tilt follows a ±20° sine (200 segments, each with a different pose)
    let n = 200;
    let len = n as f64 + 10.0; // the tip travels x = 3..203, so the stock must be longer
    let stock = StockShape::Box { min: [0.0; 3], max: [len, 10.0, 10.0] };
    let tool = Tool::flat_end_mill(4.0, 10.0);
    let pose = |i: usize| Pose::tilted_about_y([3.0 + i as f64, 5.0, 7.0], 20.0 * (2.0 * PI * i as f64 / 40.0).sin());
    let segments: Vec<_> = (0..n).map(|i| MotionSegment::linear(pose(i), pose(i + 1))).collect();
    let expected = grid_removed([0.0; 3], [len, 10.0, 10.0], &tool, &segments, 8, 0.25);
    for tile in [None, Some(10.0)] {
        let options = EngineOptions { tile_size: tile, ..Default::default() };
        let mut e = Engine::new(ManifoldKernel, std::slice::from_ref(&stock), options).unwrap();
        let reports = e.cut_many(&tool, &segments).unwrap();
        assert!(reports.iter().all(|r| r.outcome == CutOutcome::Updated));
        let removed: f64 = reports.iter().map(|r| r.removed_volume).sum();
        assert_close(removed, expected, 1.5e-2, &format!("removed volume (tile {tile:?})"));
        assert_close(len * 100.0 - e.total_volume(), removed, 1e-6, "report sums match state");
    }
}
