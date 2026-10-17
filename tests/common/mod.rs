//! Helpers shared by the integration tests.
#![allow(dead_code)]

use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

/// Asserts that `actual` is within the relative error `rel` of `expected`.
pub fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

/// A 10 mm cube of stock.
pub fn cube10(options: EngineOptions) -> Engine {
    Engine::new(ManifoldKernel, &[StockShape::Box { min: [0.0; 3], max: [10.0; 3] }], options).unwrap()
}

/// A 10 mm cube with a slot through it along y, cut by a flat end mill of `tool_diameter` with its tip at `tip_z`.
pub fn slotted_cube(tool_diameter: f64, tip_z: f64) -> Engine {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(tool_diameter, 10.0);
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -5.0, tip_z]), Pose::vertical([5.0, 15.0, tip_z]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    e
}

/// Whether point `p` lies inside the cutting part of `tool` at `pose`.
pub fn inside(tool: &Tool, pose: &Pose, p: [f64; 3]) -> bool {
    let a = pose.axis;
    let d = [p[0] - pose.tip[0], p[1] - pose.tip[1], p[2] - pose.tip[2]];
    let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
    let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - h * h;
    tool.contains(r2.max(0.0).sqrt(), h)
}

/// Grid estimate of the volume inside the 10 mm cube covered by the sweep of `segments`, with the poses of each
/// segment interpolated in `sub` steps.
pub fn grid_removed(tool: &Tool, segments: &[MotionSegment], sub: usize, spacing: f64) -> f64 {
    let reach = tool.reach();
    let items: Vec<(Vec<Pose>, [f64; 3], [f64; 3])> = segments
        .iter()
        .map(|seg| {
            let poses: Vec<Pose> = (0..=sub).map(|k| seg.pose_at(k as f64 / sub as f64)).collect();
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
    let n = (10.0 / spacing).ceil() as usize;
    let mut count = 0usize;
    for ix in 0..n {
        let x = (ix as f64 + 0.5) * spacing;
        for iy in 0..n {
            let y = (iy as f64 + 0.5) * spacing;
            for iz in 0..n {
                let z = (iz as f64 + 0.5) * spacing;
                let p = [x, y, z];
                if items.iter().any(|(poses, lo, hi)| {
                    (0..3).all(|i| p[i] >= lo[i] && p[i] <= hi[i]) && poses.iter().any(|pose| inside(tool, pose, p))
                }) {
                    count += 1;
                }
            }
        }
    }
    count as f64 * spacing * spacing * spacing
}
