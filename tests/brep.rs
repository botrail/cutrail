//! Analytic surface reconstruction and STEP export.
mod common;
#[allow(unused_imports)]
use common::{assert_close, cube10};
use cutrail::brep::{Curve, Surface};
use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

#[test]
fn through_slot_is_all_planes_and_lines() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(4.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, -5.0, 6.0]), Pose::vertical([5.0, 15.0, 6.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    assert_eq!(b.surface_counts(), (10, 0), "faces: {:?}", b.faces.iter().map(|f| f.triangles).collect::<Vec<_>>());
    let (lines, circles, polylines) = b.curve_counts();
    assert_eq!((circles, polylines), (0, 0));
    assert_eq!(lines, b.edges.len());
    assert!(b.faces.iter().all(|f| f.loops.len() == 1), "no inner loops expected");
    // The wall planes sit exactly at radius 2 from the path
    let walls: Vec<_> = b
        .faces
        .iter()
        .filter_map(|f| match &f.surface {
            Surface::Plane { origin, normal } if normal[0].abs() > 0.999 && (origin[0] - 3.0).abs() < 1e-9 || normal[0].abs() > 0.999 && (origin[0] - 7.0).abs() < 1e-9 => Some(origin[0]),
            _ => None,
        })
        .collect();
    assert_eq!(walls.len(), 2, "{walls:?}");
}

#[test]
fn plunge_hole_reconstructs_cylinder_halves_and_arcs() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(4.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    assert_eq!(b.surface_counts(), (7, 2));
    let (_, circles, polylines) = b.curve_counts();
    assert_eq!(circles, 4, "two arcs at the top and two at the floor");
    assert_eq!(polylines, 0);
    for f in &b.faces {
        if let Surface::Cylinder { center, radius } = &f.surface {
            assert_eq!((center[0], center[1]), (5.0, 5.0));
            assert_eq!(*radius, 2.0, "exact tool radius, not the mesh radius");
            assert!(!f.same_sense, "hole wall normal points toward the axis");
        }
    }
    let top = b.faces.iter().find(|f| matches!(&f.surface, Surface::Plane { origin, normal } if normal[2] > 0.999 && (origin[2] - 10.0).abs() < 1e-9)).unwrap();
    assert_eq!(top.loops.len(), 2, "top face has the hole as an inner loop");
    assert_eq!(top.loops[top.outer].len(), 4);
    for edge in &b.edges {
        if let Curve::Circle { radius, z, .. } = &edge.curve {
            assert_eq!(*radius, 2.0);
            assert!((*z - 10.0).abs() < 1e-9 || (*z - 6.0).abs() < 1e-9);
        }
    }
}

#[test]
fn stadium_slot_has_two_half_cylinders() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(3.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(Pose::vertical([3.0, 5.0, 6.0]), Pose::vertical([7.0, 5.0, 6.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    assert_eq!(b.surface_counts(), (9, 2));
    let (_, circles, polylines) = b.curve_counts();
    assert_eq!(circles, 4);
    assert_eq!(polylines, 0);
    let floor = b.faces.iter().find(|f| matches!(&f.surface, Surface::Plane { origin, normal } if normal[2] > 0.999 && (origin[2] - 6.0).abs() < 1e-9)).unwrap();
    assert_eq!(floor.loops.len(), 1);
    assert_eq!(floor.loops[0].len(), 4, "floor outline: line, arc, line, arc");
    let walls: Vec<f64> = b
        .faces
        .iter()
        .filter_map(|f| match &f.surface {
            Surface::Plane { origin, normal } if normal[1].abs() > 0.999 && (origin[1] - 5.0).abs() < 2.0 => Some(origin[1]),
            _ => None,
        })
        .collect();
    assert_eq!(walls.len(), 2);
    assert!(walls.iter().any(|&y| (y - 3.5).abs() < 1e-9) && walls.iter().any(|&y| (y - 6.5).abs() < 1e-9), "{walls:?}");
}

#[test]
fn pocket_from_overlapping_passes_merges_the_floor() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(3.0, 6.0);
    for (i, y) in [3.0, 5.0, 7.0].iter().enumerate() {
        let (a, b) = if i % 2 == 0 { (3.0, 7.0) } else { (7.0, 3.0) };
        e.cut(&tool, &MotionSegment::linear(Pose::vertical([a, *y, 7.0]), Pose::vertical([b, *y, 7.0]))).unwrap();
    }
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    let floors = b.faces.iter().filter(|f| matches!(&f.surface, Surface::Plane { origin, normal } if normal[2] > 0.999 && (origin[2] - 7.0).abs() < 1e-9)).count();
    assert_eq!(floors, 1, "overlapping passes share one floor face");
    let (planes, cylinders) = b.surface_counts();
    assert!(cylinders >= 4 && planes >= 8, "planes {planes} cylinders {cylinders}");
    assert_eq!(b.curve_counts().2, 0, "no polyline fallback expected");
}

#[test]
fn tiled_engine_reconstructs_the_same_topology() {
    let tool = Tool::flat_end_mill(4.0, 6.0);
    let seg = MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]));
    let mut whole = cube10(EngineOptions::default());
    let mut tiled = cube10(EngineOptions::default().with_tile_size(4.0));
    whole.cut(&tool, &seg).unwrap();
    tiled.cut(&tool, &seg).unwrap();
    let bw = whole.reconstruct_brep().unwrap();
    let bt = tiled.reconstruct_brep().unwrap();
    bt.check(1e-6).unwrap();
    assert_eq!(bw.surface_counts(), bt.surface_counts());
    assert_eq!(bw.curve_counts(), bt.curve_counts());
}

#[test]
fn ball_end_mill_falls_back_to_facets_but_stays_valid() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::ball_end_mill(4.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    assert_eq!(b.surface_counts().1, 0);
    assert!(b.faces.len() > 50);
    let mut buf = Vec::new();
    e.write_step(&mut buf, "ball").unwrap();
    assert!(buf.len() > 10_000);
}

#[test]
fn step_text_has_expected_entities() {
    let mut e = cube10(EngineOptions::default());
    let tool = Tool::flat_end_mill(4.0, 6.0);
    e.cut(&tool, &MotionSegment::linear(Pose::vertical([5.0, 5.0, 10.0]), Pose::vertical([5.0, 5.0, 6.0]))).unwrap();
    let mut buf = Vec::new();
    e.write_step(&mut buf, "hole").unwrap();
    let text = String::from_utf8(buf).unwrap();
    assert!(text.starts_with("ISO-10303-21;"));
    assert!(text.contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN"));
    assert_eq!(text.matches("CYLINDRICAL_SURFACE(").count(), 2);
    assert_eq!(text.matches("= CIRCLE(").count(), 4);
    assert_eq!(text.matches("ADVANCED_FACE(").count(), 9);
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 1);
    assert!(text.contains("FACE_BOUND('") && text.contains("FACE_OUTER_BOUND('"));
    assert!(text.trim_end().ends_with("END-ISO-10303-21;"));
}

#[cfg(feature = "stepio")]
#[test]
fn step_reads_back_with_matching_volume() {
    // A stadium slot (Ø3, its ends inside the stock) and a blind hole (Ø2) that does not touch it
    let mut e = cube10(EngineOptions::default());
    let slot_tool = Tool::flat_end_mill(3.0, 6.0);
    let hole_tool = Tool::flat_end_mill(2.0, 6.0);
    e.cut(&slot_tool, &MotionSegment::linear(Pose::vertical([3.0, 5.0, 6.0]), Pose::vertical([7.0, 5.0, 6.0]))).unwrap();
    e.cut(&hole_tool, &MotionSegment::linear(Pose::vertical([5.0, 2.0, 10.0]), Pose::vertical([5.0, 2.0, 8.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-6).unwrap();
    assert_eq!(b.surface_counts(), (10, 4));
    let mut buf = Vec::new();
    e.write_step(&mut buf, "stadium").unwrap();
    let text = String::from_utf8(buf).unwrap();
    let meshes = cutrail::step::meshes_from_step_str(&text, 0.01).unwrap();
    assert_eq!(meshes.len(), 1);
    let mut mesh = meshes.into_iter().next().unwrap();
    mesh.merge_vertices(1e-3);
    assert_close(mesh.signed_volume().abs(), e.total_volume(), 1e-2, "volume of STEP read back");
    let stock = Engine::new(ManifoldKernel, &[StockShape::Mesh(mesh)], EngineOptions::default()).unwrap();
    assert_close(stock.total_volume(), e.total_volume(), 1e-2, "STEP mesh is a closed manifold");
}

#[test]
fn tangent_walls_meet_end_cylinders_on_a_line() {
    // The walls of a slot are tangent to its end cylinders: the shared edges must come out as exact vertical lines.
    let mut e = cube10(EngineOptions::default());
    e.cut(&Tool::flat_end_mill(3.0, 6.0), &MotionSegment::linear(Pose::vertical([3.0, 5.0, 6.0]), Pose::vertical([7.0, 5.0, 6.0]))).unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-9).unwrap();
    assert_eq!(b.curve_counts().2, 0, "no polyline fallback on tangent edges");
}

#[test]
fn tiled_zigzag_pocket_is_consistent() {
    // Rows that overlap and connecting moves whose walls are tangent to the row ends, across tile seams.
    let mut e = Engine::new(
        ManifoldKernel,
        &[StockShape::Box { min: [0.0; 3], max: [30.0, 30.0, 10.0] }],
        EngineOptions::default().with_tile_size(10.0),
    )
    .unwrap();
    let tool = Tool::flat_end_mill(6.0, 10.0);
    let v = Pose::vertical;
    e.cut(&tool, &MotionSegment::linear(v([6.0, 6.0, 10.0]), v([6.0, 6.0, 7.0]))).unwrap();
    let rows = [6.0, 10.0, 14.0, 18.0, 22.0];
    let mut x = 6.0;
    for (i, y) in rows.iter().enumerate() {
        let to = if x == 6.0 { 24.0 } else { 6.0 };
        e.cut(&tool, &MotionSegment::linear(v([x, *y, 7.0]), v([to, *y, 7.0]))).unwrap();
        x = to;
        if let Some(next) = rows.get(i + 1) {
            e.cut(&tool, &MotionSegment::linear(v([x, *y, 7.0]), v([x, *next, 7.0]))).unwrap();
        }
    }
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-9).unwrap();
    assert_eq!(b.curve_counts().2, 0, "no polyline fallback");
}

#[test]
fn horizontal_arc_walls_are_cylinders_about_the_center() {
    let mut e = Engine::new(ManifoldKernel, &[StockShape::Box { min: [0.0; 3], max: [20.0; 3] }], EngineOptions::default()).unwrap();
    e.cut(
        &Tool::flat_end_mill(4.0, 6.0),
        &MotionSegment::arc(Pose::vertical([5.0, 10.0, 17.0]), Pose::vertical([15.0, 10.0, 17.0]), [10.0, 10.0, 17.0], [0.0, 0.0, 1.0]),
    )
    .unwrap();
    let b = e.reconstruct_brep().unwrap();
    b.check(1e-9).unwrap();
    let (planes, cylinders) = b.surface_counts();
    assert_eq!(planes, 7, "6 stock faces and one floor");
    let outer = b.faces.iter().filter(|f| matches!(&f.surface, Surface::Cylinder { center, radius } if (center[0] - 10.0).abs() < 1e-9 && (radius - 7.0).abs() < 1e-9)).count();
    let inner = b.faces.iter().filter(|f| matches!(&f.surface, Surface::Cylinder { center, radius } if (center[0] - 10.0).abs() < 1e-9 && (radius - 3.0).abs() < 1e-9)).count();
    assert!(outer >= 1 && inner >= 1, "walls about the arc center: outer {outer} inner {inner} of {cylinders}");
    assert_eq!(b.curve_counts().2, 0, "no polyline fallback");
    // The analytic volume: a half annulus (7^2 - 3^2) * pi / 2 plus two half-disks of radius 2, 3 mm deep.
    let removed = (49.0 - 9.0) * std::f64::consts::PI / 2.0 + std::f64::consts::PI * 4.0;
    let expected = 8000.0 - removed * 3.0;
    assert!((e.total_volume() - expected).abs() / expected < 5e-3, "mesh volume {} vs analytic {expected}", e.total_volume());
}
