//! Replays a recorded tool-pose stream through [`SegmentBuilder`] and the engine.
//!
//! The input is a CSV file with one pose per row, `t,x,y,z,ax,ay,az[,...]`: the tip position and
//! the unit tool axis in the stock frame, in millimeters. Lines starting with `#` are comments; two
//! of them configure the run:
//!
//! ```text
//! # stock_box xmin ymin zmin xmax ymax zmax
//! # tool flat|ball <diameter> <flute_length>
//! ```
//!
//! ```sh
//! cargo run --release --example pose_stream -- poses.csv [--min-distance 0.5] [--min-angle 2] \
//!     [--max-deviation 0] [--tile 0] [--batch 1] [--out out.obj] [--step out.step] \
//!     [--reference ref.stl] [--tolerance 0]
//! ```
//!
//! `--out` writes the result as one closed solid (STL, OBJ, glTF or 3MF by extension; OBJ keeps full
//! precision). `--reference` compares the result with another machined mesh (STL or OBJ), for example the
//! same stream replayed at the finest settings: `excess` is material left relative to it, `gouge` is
//! material removed beyond it.

use std::error::Error;
use std::fs::{self, File};
use std::io::BufWriter;
use std::time::Instant;

use cutrail::{CutOutcome, Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, SegmentBuilder, StockShape, Tool, TriMesh};

struct Args {
    csv: String,
    min_distance: f64,
    min_angle: f64,
    max_deviation: f64,
    tile: f64,
    batch: usize,
    out: Option<String>,
    step: Option<String>,
    reference: Option<String>,
    tolerance: f64,
}

fn value(it: &mut impl Iterator<Item = String>, name: &str) -> Result<String, String> {
    it.next().ok_or_else(|| format!("{name} needs a value"))
}

fn number<T: std::str::FromStr>(it: &mut impl Iterator<Item = String>, name: &str) -> Result<T, String> {
    value(it, name)?.parse().map_err(|_| format!("{name}: not a number"))
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        csv: String::new(),
        min_distance: 0.5,
        min_angle: 2.0,
        max_deviation: 0.0,
        tile: 0.0,
        batch: 1,
        out: None,
        step: None,
        reference: None,
        tolerance: 0.0,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--min-distance" => args.min_distance = number(&mut it, &a)?,
            "--min-angle" => args.min_angle = number(&mut it, &a)?,
            "--max-deviation" => args.max_deviation = number(&mut it, &a)?,
            "--tile" => args.tile = number(&mut it, &a)?,
            "--batch" => args.batch = number(&mut it, &a)?,
            "--tolerance" => args.tolerance = number(&mut it, &a)?,
            "--out" => args.out = Some(value(&mut it, &a)?),
            "--step" => args.step = Some(value(&mut it, &a)?),
            "--reference" => args.reference = Some(value(&mut it, &a)?),
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            s => args.csv = s.to_string(),
        }
    }
    if args.csv.is_empty() {
        return Err("usage: pose_stream <csv> [--min-distance D] [--min-angle A] [--max-deviation E] [--tile T] [--batch N] [--out F] [--step F] [--reference F] [--tolerance X]".into());
    }
    Ok(args)
}

#[derive(Default)]
struct Stats {
    times_ms: Vec<f64>,
    updated: usize,
    no_change: usize,
    exhausted: usize,
    cells: usize,
}

fn apply(engine: &mut Engine, tool: &Tool, segments: &[MotionSegment], stats: &mut Stats) -> cutrail::Result<()> {
    let report = if segments.len() == 1 { engine.cut(tool, &segments[0])? } else { engine.cut_batch(tool, segments)? };
    stats.times_ms.push(report.elapsed.as_secs_f64() * 1e3);
    if matches!(report.outcome, CutOutcome::NoChange) {
        stats.no_change += 1;
    } else if matches!(report.outcome, CutOutcome::Exhausted) {
        stats.exhausted += 1;
    } else {
        stats.updated += 1;
    }
    stats.cells += report.cells_touched;
    Ok(())
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[(((sorted.len() - 1) as f64) * p).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args()?;
    let text = fs::read_to_string(&args.csv)?;
    let mut stock: Option<([f64; 3], [f64; 3])> = None;
    let mut tool = Tool::flat_end_mill(8.0, 30.0);
    let mut poses: Vec<Pose> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(rest) = line.strip_prefix('#') {
            let f: Vec<&str> = rest.split_whitespace().collect();
            match f.first().copied() {
                Some("stock_box") if f.len() >= 7 => {
                    let v: Vec<f64> = f[1..7].iter().map(|s| s.parse()).collect::<Result<_, _>>()?;
                    stock = Some(([v[0], v[1], v[2]], [v[3], v[4], v[5]]));
                }
                Some("tool") if f.len() >= 4 => {
                    let d: f64 = f[2].parse()?;
                    let l: f64 = f[3].parse()?;
                    tool = if f[1] == "ball" { Tool::ball_end_mill(d, l) } else { Tool::flat_end_mill(d, l) };
                }
                _ => {}
            }
            continue;
        }
        let v: Vec<&str> = line.split(',').collect();
        if v.len() < 7 {
            continue;
        }
        let n: Result<Vec<f64>, _> = v[1..7].iter().map(|s| s.trim().parse::<f64>()).collect();
        if let Ok(n) = n {
            poses.push(Pose::new([n[0], n[1], n[2]], [n[3], n[4], n[5]]));
        }
    }
    let (min, max) = stock.ok_or("the CSV has no `# stock_box` line")?;
    let mut options = EngineOptions::default();
    if args.tile > 0.0 {
        options = options.with_tile_size(args.tile);
    }
    let mut engine = Engine::new(ManifoldKernel, &[StockShape::Box { min, max }], options)?;
    let initial = engine.total_volume();
    let mut builder = SegmentBuilder::new(args.min_distance, args.min_angle).with_max_deviation(args.max_deviation);
    let batch = args.batch.max(1);
    let mut pending: Vec<MotionSegment> = Vec::new();
    let mut stats = Stats::default();
    let mut segments = 0usize;

    let start = Instant::now();
    for pose in &poses {
        if let Some(seg) = builder.push(*pose) {
            pending.push(seg);
        }
        if pending.len() >= batch {
            segments += pending.len();
            apply(&mut engine, &tool, &pending, &mut stats)?;
            pending.clear();
        }
    }
    if let Some(seg) = builder.flush() {
        pending.push(seg);
    }
    if !pending.is_empty() {
        segments += pending.len();
        apply(&mut engine, &tool, &pending, &mut stats)?;
    }
    let total = start.elapsed().as_secs_f64();

    let mut sorted = stats.times_ms.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let sum_ms: f64 = sorted.iter().sum();
    let mean = if sorted.is_empty() { 0.0 } else { sum_ms / sorted.len() as f64 };
    let removed = initial - engine.total_volume();
    println!(
        "{} samples -> {} segments in {} calls ({} updated, {} unchanged, {} exhausted), batch {}",
        poses.len(),
        segments,
        sorted.len(),
        stats.updated,
        stats.no_change,
        stats.exhausted,
        batch
    );
    println!(
        "time: total {:.2} s, in cuts {:.2} s (mean {:.2} ms, p50 {:.2}, p90 {:.2}, max {:.2}), {} cell booleans",
        total,
        sum_ms / 1e3,
        mean,
        percentile(&sorted, 0.5),
        percentile(&sorted, 0.9),
        percentile(&sorted, 1.0),
        stats.cells
    );
    println!(
        "stock: {} triangles in {} parts, removed {:.3} mm3 of {:.3}",
        engine.total_triangles(),
        engine.parts().len(),
        removed,
        initial
    );

    let mut excess = f64::NAN;
    let mut gouge = f64::NAN;
    if let Some(path) = &args.reference {
        let mut mesh = TriMesh::read_file(path)?;
        mesh.merge_vertices(1e-6);
        let cmp = engine.compare_with_design(&mesh, args.tolerance)?;
        excess = cmp.excess_volume;
        gouge = cmp.gouge_volume;
        println!(
            "vs reference (tolerance {}): excess {:.4} mm3 {:?}, gouge {:.4} mm3 {:?}",
            args.tolerance, excess, cmp.excess_bounds, gouge, cmp.gouge_bounds
        );
    }
    if let Some(path) = &args.out {
        // One closed solid, so the file can serve as a reference for another run
        let t = Instant::now();
        engine.merged_mesh()?.write_file(path)?;
        println!("wrote {path} ({:.2} s)", t.elapsed().as_secs_f64());
    }
    if let Some(path) = &args.step {
        let t = Instant::now();
        let brep = engine.reconstruct_brep()?;
        let (planes, cylinders) = brep.surface_counts();
        let (lines, circles, polylines) = brep.curve_counts();
        let check = match brep.check(1e-6) {
            Ok(_) => "ok".to_string(),
            Err(e) => e.to_string(),
        };
        println!(
            "brep: {} faces ({planes} planes, {cylinders} cylinders), {} edges ({lines} lines, {circles} circles, {polylines} polylines), check {check}, {:.2} s",
            brep.faces.len(),
            brep.edges.len(),
            t.elapsed().as_secs_f64()
        );
        brep.write_step(BufWriter::new(File::create(path)?), "machined")?;
        println!("wrote {path}");
    }
    println!(
        "RESULT min_distance={} min_angle={} max_deviation={} tile={} batch={} samples={} segments={} calls={} updated={} total_s={:.3} cut_s={:.3} mean_ms={:.3} p50_ms={:.3} p90_ms={:.3} max_ms={:.3} triangles={} parts={} removed_mm3={:.3} excess_mm3={:.4} gouge_mm3={:.4}",
        args.min_distance,
        args.min_angle,
        args.max_deviation,
        args.tile,
        batch,
        poses.len(),
        segments,
        sorted.len(),
        stats.updated,
        total,
        sum_ms / 1e3,
        mean,
        percentile(&sorted, 0.5),
        percentile(&sorted, 0.9),
        percentile(&sorted, 1.0),
        engine.total_triangles(),
        engine.parts().len(),
        removed,
        excess,
        gouge
    );
    Ok(())
}
