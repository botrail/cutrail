//! タイル分割モードと区間のまとめ処理の検証。部品単位モードと同じ体積になること。
use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};
use std::f64::consts::PI;

fn v(p: [f64; 3]) -> Pose {
    Pose::vertical(p)
}

fn engine(tile: Option<f64>) -> Engine {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    let options = EngineOptions { tile_size: tile, ..Default::default() };
    Engine::new(ManifoldKernel, &[stock], options).unwrap()
}

fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

#[test]
fn tiled_stock_has_expected_tiles_and_volume() {
    let e = engine(Some(4.0));
    assert!(e.is_tiled());
    assert_eq!(e.parts().len(), 27); // 3x3x3（最後の列は 2 mm 幅）
    assert_close(e.total_volume(), 1000.0, 1e-9, "tiled stock volume");
}

#[test]
fn tiled_slot_matches_whole() {
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let seg = MotionSegment::linear(v([5.0, -3.0, 6.0]), v([5.0, 13.0, 6.0]));
    let mut whole = engine(None);
    let mut tiled = engine(Some(4.0));
    let rw = whole.cut(&tool, &seg).unwrap();
    let rt = tiled.cut(&tool, &seg).unwrap();
    assert_eq!(rt.outcome, CutOutcome::Updated);
    assert_close(tiled.total_volume(), whole.total_volume(), 1e-9, "remaining volume");
    assert_close(rt.removed_volume, rw.removed_volume, 1e-9, "removed volume");
    // x: タイル [0,4] と [4,8]、y: 3 列、z: [4,8] と [8,10] の 2 段 → 2*3*2
    assert_eq!(rt.cells_touched, 12);
    assert_eq!(tiled.parts().len(), 27, "tiles are kept even when partly cut");
}

#[test]
fn tiled_plunge_only_touches_overlapping_tiles() {
    let mut e = engine(Some(4.0));
    let tool = Tool::flat_end_mill(2.0, 6.0);
    // 穴 r=1 を (2,2) に。x,y とも最初のタイル内に収まる
    let r = e.cut(&tool, &MotionSegment::linear(v([2.0, 2.0, 10.0]), v([2.0, 2.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_eq!(r.cells_touched, 2, "z 方向に 2 タイル（z 4..8 と 8..10）");
    assert_close(e.total_volume(), 1000.0 - PI * 4.0, 5e-3, "remaining volume");
    let changed = e.changed_since(0);
    assert_eq!(changed.len(), 2);
    assert!(e.changed_since(e.revision()).is_empty());
}

#[test]
fn tiled_exhausted_when_everything_is_cut() {
    let mut e = engine(Some(5.0));
    let tool = Tool::flat_end_mill(40.0, 20.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, -1.0]), v([5.0, 5.0, -2.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Exhausted);
    assert_eq!(e.parts().len(), 0);
    assert_close(r.removed_volume, 1000.0, 1e-9, "removed volume");
}

#[test]
fn tiled_partial_exhaustion_removes_tiles() {
    let mut e = engine(Some(5.0));
    // z∈[5,10] の上半分を丸ごと除去する大きな工具
    let tool = Tool::flat_end_mill(40.0, 10.0);
    let r = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 5.0]), v([5.0, 5.0, 5.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_eq!(e.parts().len(), 4, "upper 4 tiles removed, lower 4 remain");
    assert_close(e.total_volume(), 500.0, 1e-9, "remaining volume");
    // 消えたタイルに再度切削しても NoChange
    let r2 = e.cut(&tool, &MotionSegment::linear(v([5.0, 5.0, 6.0]), v([5.0, 5.0, 6.0]))).unwrap();
    assert_eq!(r2.outcome, CutOutcome::NoChange);
}

#[test]
fn batch_equals_sequential() {
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let segments: Vec<_> = (0..6)
        .map(|i| MotionSegment::linear(v([2.0 + i as f64, 5.0, 7.0]), v([3.0 + i as f64, 5.0, 7.0])))
        .collect();
    let mut seq = engine(Some(4.0));
    let mut batch = engine(Some(4.0));
    let reports = seq.cut_many(&tool, &segments).unwrap();
    let total_removed: f64 = reports.iter().map(|r| r.removed_volume).sum();
    let rb = batch.cut_batch(&tool, &segments).unwrap();
    assert_eq!(rb.outcome, CutOutcome::Updated);
    assert_eq!(batch.revision(), 1);
    assert_eq!(seq.revision(), 6);
    assert_close(batch.total_volume(), seq.total_volume(), 1e-6, "remaining volume");
    assert_close(rb.removed_volume, total_removed, 1e-6, "removed volume");
    // まとめ処理は三角形数を減らすとは限らない（和集合の交線が増える）。体積の一致だけを確認する
}

#[test]
fn batch_in_whole_mode_matches_too() {
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let segments: Vec<_> = (0..4)
        .map(|i| MotionSegment::linear(v([2.0 + i as f64, 5.0, 7.0]), v([3.0 + i as f64, 5.0, 7.0])))
        .collect();
    let mut seq = engine(None);
    let mut batch = engine(None);
    seq.cut_many(&tool, &segments).unwrap();
    batch.cut_batch(&tool, &segments).unwrap();
    assert_close(batch.total_volume(), seq.total_volume(), 1e-6, "remaining volume");
}

#[test]
fn mesh_cache_is_reused_until_change() {
    let mut e = engine(Some(5.0));
    let id = e.parts()[0].id;
    let before = e.mesh(id).unwrap().clone();
    let again = e.mesh(id).unwrap();
    assert_eq!(&before, again);
    let tool = Tool::flat_end_mill(2.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(v([2.0, 2.0, 10.0]), v([2.0, 2.0, 6.0]))).unwrap();
    let upper = e.parts().iter().find(|p| p.bounds.min[2] >= 5.0 && p.bounds.min[0] < 5.0 && p.bounds.min[1] < 5.0).unwrap().id;
    assert!(e.mesh(upper).unwrap().triangle_count() > 12);
}

#[test]
fn simplify_option_keeps_volume() {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    let mut options = EngineOptions::default().with_tile_size(10.0);
    options.simplify_threshold = Some(50);
    options.simplify_tolerance = 1e-3;
    let mut e = Engine::new(ManifoldKernel, &[stock], options).unwrap();
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let segments: Vec<_> = (0..5)
        .map(|i| MotionSegment::linear(v([2.0 + i as f64, 5.0, 7.0]), v([3.0 + i as f64, 5.0, 7.0])))
        .collect();
    e.cut_many(&tool, &segments).unwrap();
    let removed = (4.0 * 5.0 + PI * 4.0) * 3.0;
    assert_close(e.total_volume(), 1000.0 - removed, 1e-2, "remaining volume with simplify");
}

#[test]
fn tiled_cylinder_stock_volume_matches() {
    let stock = StockShape::Cylinder { center: [5.0, 5.0], radius: 5.0, z0: 0.0, z1: 10.0 };
    let e = Engine::new(ManifoldKernel, &[stock], EngineOptions::default().with_tile_size(4.0)).unwrap();
    // タイル境界での交差でカーネルが微小な辺を併合するため、1e-5 程度の差が出る
    assert_close(e.total_volume(), PI * 25.0 * 10.0, 1e-4, "tiled cylinder volume");
    // 半径 5 の円は 10x10 の角をわずかに残すだけなので、4 mm タイルはすべて円柱と交差する
    assert_eq!(e.parts().len(), 27);
}
