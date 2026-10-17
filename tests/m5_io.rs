//! メッシュ入出力、メッシュ素材、設計モデルとの比較、姿勢ストリーム。
use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, SegmentBuilder, StockShape, Tool, TriMesh};
use std::io::Cursor;

fn assert_close(actual: f64, expected: f64, rel: f64, what: &str) {
    let err = ((actual - expected) / expected).abs();
    assert!(err <= rel, "{what}: actual {actual} expected {expected} rel.err {err}");
}

fn slotted_cube(tool_diameter: f64, tip_z: f64) -> Engine {
    let stock = StockShape::Box { min: [0.0; 3], max: [10.0; 3] };
    let mut e = Engine::new(ManifoldKernel, &[stock], EngineOptions::default()).unwrap();
    let tool = Tool::flat_end_mill(tool_diameter, 10.0);
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -5.0, tip_z]), Pose::vertical([5.0, 15.0, tip_z]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    e
}

/// 頂点を面ごとに持つ（STL 風の）立方体メッシュ。
fn cube_mesh_with_duplicates(size: f64) -> TriMesh {
    let faces: [[[f64; 3]; 4]; 6] = [
        [[0.0, 0.0, 0.0], [0.0, size, 0.0], [size, size, 0.0], [size, 0.0, 0.0]], // bottom (-z)
        [[0.0, 0.0, size], [size, 0.0, size], [size, size, size], [0.0, size, size]], // top (+z)
        [[0.0, 0.0, 0.0], [size, 0.0, 0.0], [size, 0.0, size], [0.0, 0.0, size]], // -y
        [[0.0, size, 0.0], [0.0, size, size], [size, size, size], [size, size, 0.0]], // +y
        [[0.0, 0.0, 0.0], [0.0, 0.0, size], [0.0, size, size], [0.0, size, 0.0]], // -x
        [[size, 0.0, 0.0], [size, size, 0.0], [size, size, size], [size, 0.0, size]], // +x
    ];
    let mut mesh = TriMesh::default();
    for f in faces {
        let base = mesh.positions.len() as u32;
        mesh.positions.extend_from_slice(&f);
        mesh.triangles.push([base, base + 1, base + 2]);
        mesh.triangles.push([base, base + 2, base + 3]);
    }
    mesh
}

#[test]
fn stl_round_trip_and_merge() {
    let e = slotted_cube(4.0, 6.0);
    let mut buf = Vec::new();
    e.write_stl(&mut buf).unwrap();
    let mut mesh = TriMesh::read_stl(Cursor::new(buf)).unwrap();
    assert_eq!(mesh.triangle_count(), e.combined_mesh().triangle_count());
    let before = mesh.positions.len();
    let merged = mesh.merge_vertices(1e-6);
    assert!(merged > 0 && mesh.positions.len() < before);
    assert_close(mesh.signed_volume(), 840.0, 1e-3, "volume of merged STL mesh");
    // 併合したメッシュを素材として取り込む
    let e2 = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], EngineOptions::default()).unwrap();
    assert_close(e2.total_volume(), 840.0, 1e-3, "mesh stock volume");
}

#[test]
fn ascii_stl_is_read() {
    let text = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
    let mesh = TriMesh::read_stl(Cursor::new(text.as_bytes())).unwrap();
    assert_eq!(mesh.triangle_count(), 1);
    assert_eq!(mesh.positions[1], [1.0, 0.0, 0.0]);
}

#[test]
fn obj_glb_3mf_outputs_are_well_formed() {
    let e = slotted_cube(4.0, 6.0);
    let mesh = e.combined_mesh();

    let mut obj = Vec::new();
    mesh.write_obj(&mut obj).unwrap();
    let text = String::from_utf8(obj).unwrap();
    assert_eq!(text.lines().filter(|l| l.starts_with("v ")).count(), mesh.positions.len());
    assert_eq!(text.lines().filter(|l| l.starts_with("f ")).count(), mesh.triangle_count());

    let mut glb = Vec::new();
    mesh.write_glb(&mut glb).unwrap();
    assert_eq!(&glb[0..4], b"glTF");
    assert_eq!(u32::from_le_bytes(glb[8..12].try_into().unwrap()) as usize, glb.len());
    let json_len = u32::from_le_bytes(glb[12..16].try_into().unwrap()) as usize;
    let json = std::str::from_utf8(&glb[20..20 + json_len]).unwrap();
    assert!(json.contains("\"POSITION\":1"));
    assert!(json.contains(&format!("\"count\":{}", mesh.triangle_count() * 3)));

    let mut m3 = Cursor::new(Vec::new());
    mesh.write_3mf(&mut m3).unwrap();
    let mut archive = zip::ZipArchive::new(Cursor::new(m3.into_inner())).unwrap();
    assert_eq!(archive.len(), 3);
    let mut model = String::new();
    std::io::Read::read_to_string(&mut archive.by_name("3D/3dmodel.model").unwrap(), &mut model).unwrap();
    assert_eq!(model.matches("<vertex ").count(), mesh.positions.len());
    assert_eq!(model.matches("<triangle ").count(), mesh.triangle_count());
}

#[test]
fn write_mesh_file_dispatches_on_extension() {
    let e = slotted_cube(4.0, 6.0);
    let dir = std::env::temp_dir().join(format!("cutrail-m5-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for ext in ["stl", "obj", "glb", "3mf"] {
        let path = dir.join(format!("out.{ext}"));
        e.write_mesh_file(&path).unwrap();
        assert!(std::fs::metadata(&path).unwrap().len() > 100, "{ext}");
    }
    assert!(e.write_mesh_file(dir.join("out.xyz")).is_err());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn mesh_stock_with_duplicate_vertices_is_merged_on_import() {
    let mesh = cube_mesh_with_duplicates(10.0);
    assert_eq!(mesh.positions.len(), 24);
    let mut e = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], EngineOptions::default()).unwrap();
    assert_close(e.total_volume(), 1000.0, 1e-9, "cube volume");
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let r = e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0]))).unwrap();
    assert_eq!(r.outcome, CutOutcome::Updated);
    assert_close(e.total_volume(), 840.0, 1e-3, "slot in mesh stock");
}

#[test]
fn open_mesh_stock_is_rejected() {
    let mut mesh = cube_mesh_with_duplicates(10.0);
    mesh.triangles.pop();
    let err = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], EngineOptions::default()).unwrap_err();
    assert!(matches!(err, cutrail::Error::InvalidInput(_)), "{err}");
}

#[test]
fn compare_with_design_reports_excess_and_gouge() {
    // 設計：幅 4、深さ 4 の貫通溝（体積 840）
    let design = slotted_cube(4.0, 6.0).combined_mesh();
    // 加工 A：深さ 2 しか削っていない → 削り残し 4*10*2 = 80
    let a = slotted_cube(4.0, 8.0);
    let ca = a.compare_with_design(&design, 0.0).unwrap();
    assert_close(ca.excess_volume, 80.0, 2e-2, "excess of shallow cut");
    assert!(ca.gouge_volume < 0.5, "gouge {}", ca.gouge_volume);
    assert!(ca.excess_bounds.is_some() && ca.excess.triangle_count() > 0);
    // 加工 B：幅 6 で削った → 削りすぎ 2*10*4 = 80
    let b = slotted_cube(6.0, 6.0);
    let cb = b.compare_with_design(&design, 0.0).unwrap();
    assert_close(cb.gouge_volume, 80.0, 2e-2, "gouge of wide cut");
    assert!(cb.excess_volume < 0.5, "excess {}", cb.excess_volume);
    // 公差 0.5：設計を 0.5 膨らませると溝は x 3.5..6.5、底 z 6.5 になるので、A の削り残しは 3*1.5*10 = 45 に減る
    let ca_tol = a.compare_with_design(&design, 0.5).unwrap();
    assert!(ca_tol.excess_volume < ca.excess_volume, "{} vs {}", ca_tol.excess_volume, ca.excess_volume);
    assert_close(ca_tol.excess_volume, 45.0, 5e-2, "excess beyond tolerance");
    // 一致していれば両方ゼロ
    let same = slotted_cube(4.0, 6.0).compare_with_design(&design, 0.0).unwrap();
    assert!(same.is_within_tolerance(), "{:?}", (same.excess_volume, same.gouge_volume));
}

#[test]
fn segment_builder_coalesces_small_moves() {
    let mut b = SegmentBuilder::new(1.0, 5.0);
    let mut emitted = Vec::new();
    for i in 0..=55 {
        let x = 0.1 * i as f64; // 0.0 〜 5.5 mm
        if let Some(seg) = b.push(Pose::vertical([x, 0.0, 0.0])) {
            emitted.push(seg);
        }
    }
    assert_eq!(emitted.len(), 5, "1 mm ごとに区間が出る");
    let tail = b.flush().expect("remaining 0.5 mm");
    let (from, to) = tail.endpoints();
    assert_close(to.tip[0] - from.tip[0], 0.5, 1e-9, "flushed length");
    assert!(b.flush().is_none());
    // 回転だけでも角度のしきい値で区間が出る
    let mut r = SegmentBuilder::new(1.0, 5.0);
    assert!(r.push(Pose::vertical([0.0; 3])).is_none());
    assert!(r.push(Pose::tilted_about_y([0.0; 3], 3.0)).is_none());
    assert!(r.push(Pose::tilted_about_y([0.0; 3], 6.0)).is_some());
}
