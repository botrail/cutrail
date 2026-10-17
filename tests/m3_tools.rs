//! ボール・ブルノーズエンドミル、円弧・ヘリカル移動、シャンク・ホルダーの干渉検出。
use cutrail::sweep::swept_volume;
use cutrail::{CutOutcome, Engine, EngineOptions, Holder, ManifoldKernel, MotionSegment, Pose, Shank, StockShape, Tool};
use std::f64::consts::PI;

fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

fn cube10(options: EngineOptions) -> Engine {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    Engine::new(ManifoldKernel, &[stock], options).unwrap()
}

/// 点が姿勢 `pose` の工具の切削部の内部にあるか。
fn inside(tool: &Tool, pose: &Pose, p: [f64; 3]) -> bool {
    let a = pose.axis;
    let d = [p[0] - pose.tip[0], p[1] - pose.tip[1], p[2] - pose.tip[2]];
    let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
    let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - h * h;
    tool.contains(r2.max(0.0).sqrt(), h)
}

/// 直方体 [0,10]³ の中で、区間列の掃引（`sub` 分割で姿勢を補間）に覆われる体積の格子推定。
fn grid_removed(tool: &Tool, segments: &[MotionSegment], sub: usize, spacing: f64) -> f64 {
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

#[test]
fn ball_plunge_matches_analytic() {
    // 半径 2 のボールを先端 z=6 まで突込む：半球 (2/3)π8 + 円柱 π4·2
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let expected = 2.0 / 3.0 * PI * 8.0 + PI * 4.0 * 2.0;
    assert_close(r.removed_volume, expected, 2e-2, "ball plunge removed volume");
}

#[test]
fn ball_slot_matches_analytic() {
    // 断面は半円 (π·4/2) + 長方形 (4×2)、長さ 10
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0]))).unwrap();
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
    // 解析値：角部 ∫0^1 π r(h)² dh（r(h) = 2 + sqrt(1-(1-h)²)）+ 円柱 π·9·3
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
    let seg = MotionSegment::linear(Pose::tilted_about_y([2.0, 5.0, 7.0], -20.0), Pose::tilted_about_y([8.0, 5.0, 7.0], 20.0));
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 64, 0.1);
    assert_close(r.removed_volume, expected, 2e-2, "tilted ball sweep");
}

#[test]
fn quarter_arc_matches_grid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let seg = MotionSegment::arc(Pose::vertical([8.0, 5.0, 6.0]), Pose::vertical([5.0, 8.0, 6.0]), [5.0, 5.0, 6.0], [0.0, 0.0, 1.0]);
    let v = swept_volume(&tool, &seg, 0.01).unwrap();
    assert!(v.piece_count() > 4, "arc must be chord-subdivided: {}", v.piece_count());
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 256, 0.05);
    assert_close(r.removed_volume, expected, 1e-2, "quarter arc removed volume");
}

#[test]
fn full_circle_matches_analytic() {
    // 半径 1 の工具を半径 3 の円に沿って一周：環 π(4²-2²)、深さ 4
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let p = Pose::vertical([8.0, 5.0, 6.0]);
    let seg = MotionSegment::arc(p, p, [5.0, 5.0, 6.0], [0.0, 0.0, 1.0]);
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    assert_close(r.removed_volume, PI * (16.0 - 4.0) * 4.0, 1e-2, "full circle removed volume");
}

#[test]
fn clockwise_arc_via_flipped_normal() {
    // 法線を反転すると逆回り。1/4 周の除去体積は向きによらず同じ
    let tool = Tool::flat_end_mill(2.0, 6.0);
    let ccw = MotionSegment::arc(Pose::vertical([8.0, 5.0, 6.0]), Pose::vertical([5.0, 8.0, 6.0]), [5.0, 5.0, 6.0], [0.0, 0.0, 1.0]);
    let cw = MotionSegment::arc(Pose::vertical([5.0, 8.0, 6.0]), Pose::vertical([8.0, 5.0, 6.0]), [5.0, 5.0, 6.0], [0.0, 0.0, -1.0]);
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
    let seg = MotionSegment::arc(Pose::vertical([8.0, 5.0, 10.0]), Pose::vertical([5.0, 8.0, 8.0]), [5.0, 5.0, 9.0], [0.0, 0.0, 1.0]);
    let r = e.cut(&tool, &seg).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    let expected = grid_removed(&tool, std::slice::from_ref(&seg), 256, 0.05);
    assert_close(r.removed_volume, expected, 1e-2, "helical removed volume");
}

#[test]
fn arc_with_mismatched_radius_is_rejected() {
    let seg = MotionSegment::arc(Pose::vertical([8.0, 5.0, 6.0]), Pose::vertical([5.0, 9.0, 6.0]), [5.0, 5.0, 6.0], [0.0, 0.0, 1.0]);
    assert!(seg.validate().is_err());
}

#[test]
fn shank_collision_is_reported_after_cutting() {
    // 刃 Ø4 刃長 6、シャンク Ø8 長 10。先端 z=2 まで突込むとシャンク（z 8..18）が z 8..10 で素材に当たる
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
    // 刃 Ø4 刃長 3、シャンク Ø4 長 2（穴と同径で当たらない）、ホルダー Ø10 長 5（z 6..11 で当たる）
    let tool = Tool::flat_end_mill(4.0, 3.0)
        .with_shank(Shank { diameter: 4.0, length: 2.0 })
        .with_holder(Holder { diameter: 10.0, length: 5.0 });
    let mut e = cube10(EngineOptions::default().with_tile_size(4.0).with_collision_check());
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 1.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    let c = r.collision.expect("holder collision expected");
    assert_close(c.volume, PI * (25.0 - 4.0) * 4.0, 2e-2, "holder collision volume");
}

#[test]
fn invalid_tools_are_rejected() {
    assert!(Tool::ball_end_mill(4.0, 1.0).validate().is_err());
    assert!(Tool::bull_nose_end_mill(6.0, 4.0, 6.0).validate().is_err());
    assert!(Tool::bull_nose_end_mill(6.0, 1.0, 6.0).validate().is_ok());
}
