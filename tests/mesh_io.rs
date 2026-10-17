//! Mesh formats (STL, OBJ, glTF, 3MF) and mesh stock.
mod common;
use common::{assert_close, cube10, slotted_cube};
use cutrail::{
    CutOutcome, Engine, EngineOptions, FaceKind, ManifoldKernel, MotionSegment, Pose, StockShape, Tool, TriMesh,
};
use std::io::Cursor;

/// A cube mesh with one set of vertices per face, as an STL file would store it.
fn cube_mesh_with_duplicates(size: f64) -> TriMesh {
    let faces: [[[f64; 3]; 4]; 6] = [
        [[0.0, 0.0, 0.0], [0.0, size, 0.0], [size, size, 0.0], [size, 0.0, 0.0]], // bottom (-z)
        [
            [0.0, 0.0, size],
            [size, 0.0, size],
            [size, size, size],
            [0.0, size, size],
        ], // top (+z)
        [[0.0, 0.0, 0.0], [size, 0.0, 0.0], [size, 0.0, size], [0.0, 0.0, size]], // -y
        [
            [0.0, size, 0.0],
            [0.0, size, size],
            [size, size, size],
            [size, size, 0.0],
        ], // +y
        [[0.0, 0.0, 0.0], [0.0, 0.0, size], [0.0, size, size], [0.0, size, 0.0]], // -x
        [
            [size, 0.0, 0.0],
            [size, size, 0.0],
            [size, size, size],
            [size, 0.0, size],
        ], // +x
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
    // The merged mesh can be used as stock
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
    assert_eq!(
        text.lines().filter(|l| l.starts_with("v ")).count(),
        mesh.positions.len()
    );
    assert_eq!(
        text.lines().filter(|l| l.starts_with("f ")).count(),
        mesh.triangle_count()
    );

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
    let dir = std::env::temp_dir().join(format!("cutrail-mesh-io-{}", std::process::id()));
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
    let r = e
        .cut(
            &tool,
            &MotionSegment::linear(Pose::vertical([5.0, -3.0, 6.0]), Pose::vertical([5.0, 13.0, 6.0])),
        )
        .unwrap();
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
fn obj_round_trip_keeps_vertices_and_precision() {
    let mut e = cube10(EngineOptions::default());
    e.cut(
        &Tool::flat_end_mill(3.0, 6.0),
        &MotionSegment::linear(Pose::vertical([3.0, 5.0, 6.0]), Pose::vertical([7.0, 5.0, 6.0])),
    )
    .unwrap();
    let mesh = e.combined_mesh();
    let mut buf = Vec::new();
    mesh.write_obj(&mut buf).unwrap();
    let back = TriMesh::read_obj(Cursor::new(buf)).unwrap();
    assert_eq!(back.positions, mesh.positions);
    assert_eq!(back.triangles, mesh.triangles);
    let quad = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1/1/1 2/2/2 3/3/3 4/4/4\nf -4 -1 -2\n";
    let m = TriMesh::read_obj(Cursor::new(quad.as_bytes())).unwrap();
    assert_eq!(m.triangles, vec![[0, 1, 2], [0, 2, 3], [0, 3, 2]]);
}

fn triangle_area(mesh: &TriMesh, t: [u32; 3]) -> f64 {
    let [a, b, c] = [
        mesh.positions[t[0] as usize],
        mesh.positions[t[1] as usize],
        mesh.positions[t[2] as usize],
    ];
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() / 2.0
}

#[test]
fn merged_mesh_labels_cut_faces() {
    for tile in [None, Some(4.0)] {
        let mut options = EngineOptions::default();
        if let Some(t) = tile {
            options = options.with_tile_size(t);
        }
        let mut e = Engine::new(
            ManifoldKernel,
            &[StockShape::Box {
                min: [0.0; 3],
                max: [10.0; 3],
            }],
            options,
        )
        .unwrap();
        e.cut(
            &Tool::flat_end_mill(4.0, 10.0),
            &MotionSegment::linear(Pose::vertical([5.0, -5.0, 6.0]), Pose::vertical([5.0, 15.0, 6.0])),
        )
        .unwrap();
        let (mesh, kinds) = e.merged_mesh_with_kinds().unwrap();
        assert_eq!(kinds.len(), mesh.triangle_count());
        let area = |kind: FaceKind| -> f64 {
            mesh.triangles
                .iter()
                .zip(&kinds)
                .filter(|(_, k)| **k == kind)
                .map(|(t, _)| triangle_area(&mesh, *t))
                .sum()
        };
        // The slot adds a floor (4x10) and two walls (4x10); the stock loses the top opening (4x10) and two ends (4x4)
        assert_close(area(FaceKind::Cut), 120.0, 2e-3, "cut area"); // the polygonal slot is 0.16% wider than the tool
        assert_close(area(FaceKind::Stock), 528.0, 2e-3, "stock area");
    }
}
