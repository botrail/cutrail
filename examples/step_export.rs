//! Writes a few machined shapes as STEP and STL files, for checking in a CAD system.
//!
//! ```sh
//! cargo run --release --example step_export -- <output_dir>
//! ```

use std::error::Error;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

fn cube(size: f64, options: EngineOptions) -> cutrail::Result<Engine> {
    Engine::new(
        ManifoldKernel,
        &[StockShape::Box {
            min: [0.0; 3],
            max: [size; 3],
        }],
        options,
    )
}

fn export(engine: &Engine, dir: &Path, name: &str) -> Result<(), Box<dyn Error>> {
    let brep = engine.reconstruct_brep()?;
    let (planes, cylinders) = brep.surface_counts();
    let (lines, circles, polylines) = brep.curve_counts();
    let check = match brep.check(1e-6) {
        Ok(_) => "ok".to_string(),
        Err(e) => e.to_string(),
    };
    println!(
        "{name}: volume {:.4} mm3, {} faces ({planes} planes, {cylinders} cylinders), {} edges ({lines} lines, {circles} circles, {polylines} polylines), check {check}",
        engine.total_volume(),
        brep.faces.len(),
        brep.edges.len()
    );
    brep.write_step(BufWriter::new(File::create(dir.join(format!("{name}.step")))?), name)?;
    engine.write_mesh_file(dir.join(format!("{name}.stl")))?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let dir = std::env::args().nth(1).ok_or("usage: step_export <output_dir>")?;
    let dir = Path::new(&dir);
    fs::create_dir_all(dir)?;
    let v = Pose::vertical;

    // A blind hole: one cylinder wall and a circular floor.
    let mut hole = cube(10.0, EngineOptions::default())?;
    hole.cut(
        &Tool::flat_end_mill(4.0, 6.0),
        &MotionSegment::linear(v([5.0, 5.0, 10.0]), v([5.0, 5.0, 6.0])),
    )?;
    export(&hole, dir, "hole")?;

    // A slot with round ends plus a hole that does not touch it.
    let mut stadium = cube(10.0, EngineOptions::default())?;
    stadium.cut(
        &Tool::flat_end_mill(3.0, 6.0),
        &MotionSegment::linear(v([3.0, 5.0, 6.0]), v([7.0, 5.0, 6.0])),
    )?;
    stadium.cut(
        &Tool::flat_end_mill(2.0, 6.0),
        &MotionSegment::linear(v([5.0, 2.0, 10.0]), v([5.0, 2.0, 8.0])),
    )?;
    export(&stadium, dir, "stadium")?;

    // A zigzag pocket machined on 10 mm tiles: the tile seams must not appear in the B-rep.
    let mut pocket = Engine::new(
        ManifoldKernel,
        &[StockShape::Box {
            min: [0.0; 3],
            max: [30.0, 30.0, 10.0],
        }],
        EngineOptions::default().with_tile_size(10.0),
    )?;
    let tool = Tool::flat_end_mill(6.0, 10.0);
    pocket.cut(&tool, &MotionSegment::linear(v([6.0, 6.0, 10.0]), v([6.0, 6.0, 7.0])))?;
    let mut x = 6.0;
    let rows = [6.0, 10.0, 14.0, 18.0, 22.0];
    for (i, y) in rows.iter().enumerate() {
        let to = if x == 6.0 { 24.0 } else { 6.0 };
        pocket.cut(&tool, &MotionSegment::linear(v([x, *y, 7.0]), v([to, *y, 7.0])))?;
        x = to;
        if let Some(next) = rows.get(i + 1) {
            pocket.cut(&tool, &MotionSegment::linear(v([x, *y, 7.0]), v([x, *next, 7.0])))?;
        }
    }
    export(&pocket, dir, "pocket_tiled")?;

    // An arc move: the sweep is a union of hulls, so its walls become facets.
    let mut arc = cube(20.0, EngineOptions::default())?;
    arc.cut(
        &Tool::flat_end_mill(4.0, 6.0),
        &MotionSegment::arc(
            v([5.0, 10.0, 17.0]),
            v([15.0, 10.0, 17.0]),
            [10.0, 10.0, 17.0],
            [0.0, 0.0, 1.0],
        ),
    )?;
    export(&arc, dir, "arc")?;

    // A ball end mill plunge: facets only.
    let mut ball = cube(10.0, EngineOptions::default())?;
    ball.cut(
        &Tool::ball_end_mill(4.0, 6.0),
        &MotionSegment::linear(v([5.0, 5.0, 10.0]), v([5.0, 5.0, 6.0])),
    )?;
    export(&ball, dir, "ball")?;

    // A tilted (5-axis) pass: facets only.
    let mut tilted = cube(20.0, EngineOptions::default())?;
    tilted.cut(
        &Tool::flat_end_mill(4.0, 10.0),
        &MotionSegment::linear(
            Pose::tilted_about_y([4.0, 10.0, 18.0], 20.0),
            Pose::tilted_about_y([16.0, 10.0, 18.0], 20.0),
        ),
    )?;
    export(&tilted, dir, "tilted")?;
    Ok(())
}
