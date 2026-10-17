//! 基本的な切削ケース：非交差、溝、穴、分割、全量除去、共面、内部空洞、連続切削。
use cutrail::{CutOutcome, Engine, EngineOptions, Error, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};
use std::f64::consts::PI;

fn cube10() -> Engine {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    Engine::new(ManifoldKernel, &[stock], EngineOptions::default()).unwrap()
}

fn v(p: [f64; 3]) -> Pose {
    Pose::vertical(p)
}

fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

#[test]
fn disjoint_is_no_change() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([20.0, 20.0, 20.0]), v([24.0, 20.0, 20.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::NoChange);
    assert_eq!(e.revision(), 0);
    assert_eq!(e.total_volume(), 1000.0);
}

#[test]
fn slot_matches_analytic() {
    // 幅 4、深さ 4 の溝を y 方向に貫通させる：除去 4*10*4 = 160
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, -3.0, 6.0]), v([5.0, 13.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_eq!(r.revision, 1);
    assert_close(e.total_volume(), 840.0, 1e-3, "remaining volume");
    assert_close(r.removed_volume, 160.0, 5e-3, "removed volume");
    assert_eq!(e.parts().len(), 1);
}

#[test]
fn plunge_hole_matches_analytic() {
    // 半径 2、深さ 4 の止まり穴：除去 16π
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 10.0]), v([5.0, 5.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_close(e.total_volume(), 1000.0 - 16.0 * PI, 1e-3, "remaining volume");
}

#[test]
fn through_cut_splits_into_two_parts() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(2.0, 12.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, -3.0, -1.0]), v([5.0, 13.0, -1.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_eq!(e.parts().len(), 2, "{:?}", r.diagnostics);
    assert_close(e.total_volume(), 800.0, 1e-3, "remaining volume");
    let ids: Vec<_> = e.parts().iter().map(|p| p.id).collect();
    assert_ne!(ids[0], ids[1]);
}

#[test]
fn exhausted_when_tool_covers_stock() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(40.0, 20.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, -1.0]), v([5.0, 5.0, -2.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Exhausted);
    assert_eq!(e.parts().len(), 0);
    assert_close(r.removed_volume, 1000.0, 1e-9, "removed volume");
}

#[test]
fn same_segment_twice_is_no_change() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let seg = MotionSegment::linear(v([5.0, -3.0, 6.0]), v([5.0, 13.0, 6.0]));
    let r1 = e.cut(&tool, &seg).unwrap();
    let tris = e.parts()[0].triangles;
    let r2 = e.cut(&tool, &seg).unwrap();
    assert_eq!(r1.outcome, CutOutcome::Updated);
    assert_eq!(r2.outcome, CutOutcome::NoChange);
    assert_eq!(e.revision(), 1);
    assert_eq!(e.parts()[0].triangles, tris, "no-change must keep the previous mesh");
}

#[test]
fn adjacent_same_depth_slots_succeed() {
    // 溝 A は x∈[2,6]、溝 B は x∈[5,9]。底面 z=6 が共面で重なる。和の幅は 7
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let a = e.cut(&tool, &MotionSegment::linear(v([4.0, -3.0, 6.0]), v([4.0, 13.0, 6.0]))).unwrap();
    let b = e.cut(&tool, &MotionSegment::linear(v([7.0, -3.0, 6.0]), v([7.0, 13.0, 6.0]))).unwrap();
    assert_eq!(a.outcome, CutOutcome::Updated);
    assert_eq!(b.outcome, CutOutcome::Updated);
    assert_close(e.total_volume(), 1000.0 - 7.0 * 10.0 * 4.0, 1e-3, "remaining volume");
    assert_eq!(e.parts().len(), 1);
}

#[test]
fn internal_cut_keeps_single_part_with_void() {
    // 刃長 2 の工具を素材内部に置く（物理的にはあり得ないが空洞の扱いを確認する）
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 2.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 4.0]), v([5.0, 5.0, 4.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated, "{:?}", r.diagnostics);
    assert_eq!(e.parts().len(), 1, "void must stay attached: {:?}", r.diagnostics);
    assert_close(e.total_volume(), 1000.0 - 8.0 * PI, 1e-3, "remaining volume");
}

#[test]
fn hundred_step_slot_sequence() {
    // 1 mm ずつ 100 回送る（x=3 から x=103）。和は直線部 100 mm の長円、深さ 3
    let stock = StockShape::Box { min: [0.0; 3], max: [110.0, 10.0, 10.0] };
    let mut e = Engine::new(ManifoldKernel, &[stock], EngineOptions::default()).unwrap();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let segments: Vec<_> = (0..100)
        .map(|i| MotionSegment::linear(v([3.0 + i as f64, 5.0, 7.0]), v([4.0 + i as f64, 5.0, 7.0])))
        .collect();
    let reports = e.cut_many(&tool, &segments).unwrap();
    assert!(reports.iter().all(|r| r.outcome == CutOutcome::Updated));
    assert_eq!(e.revision(), 100);
    assert_eq!(e.parts().len(), 1);
    let removed = (4.0 * 100.0 + PI * 4.0) * 3.0;
    assert_close(e.total_volume(), 11000.0 - removed, 1e-3, "remaining volume");
}

#[test]
fn ramp_move_is_supported() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, -3.0, 10.0]), v([5.0, 13.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert!(e.total_volume() < 1000.0 && e.total_volume() > 840.0);
    assert_eq!(e.parts().len(), 1);
}

#[test]
fn axis_flip_within_segment_is_unsupported() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let up = Pose::new([5.0, 5.0, 5.0], [0.0, 0.0, 1.0]);
    let down = Pose::new([5.0, 5.0, 5.0], [0.0, 0.0, -1.0]);
    let err = e.cut(&tool, &MotionSegment::linear(up, down)).unwrap_err();
    assert!(matches!(err, Error::Unsupported(_)), "{err}");
    assert_eq!(e.revision(), 0);
}

#[test]
fn invalid_tool_is_rejected() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(0.0, 6.0);
    let err = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 10.0]), v([5.0, 5.0, 6.0]))).unwrap_err();
    assert!(matches!(err, Error::InvalidInput(_)));
}

#[test]
fn stl_output_has_expected_size() {
    let mut e = cube10();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 10.0]), v([5.0, 5.0, 6.0]))).unwrap();
    let mut buf = Vec::new();
    e.write_stl(&mut buf).unwrap();
    let tris = e.combined_mesh().triangle_count();
    assert_eq!(buf.len(), 84 + 50 * tris);
    assert_eq!(u32::from_le_bytes(buf[80..84].try_into().unwrap()) as usize, tris);
}

#[test]
fn cylinder_stock_volume_matches() {
    let stock = StockShape::Cylinder { center: [5.0, 5.0], radius: 5.0, z0: 0.0, z1: 10.0 };
    let e = Engine::new(ManifoldKernel, &[stock], EngineOptions::default()).unwrap();
    assert_close(e.total_volume(), PI * 25.0 * 10.0, 1e-6, "cylinder stock volume");
}
