//! Minimal example: three pocket passes and a hole, written to STL.
//!
//! ```sh
//! cargo run --release --example slot -- out.stl
//! ```
use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out = std::env::args().nth(1).unwrap_or_else(|| "slot.stl".to_string());
    let stock = StockShape::Box {
        min: [0.0, 0.0, 0.0],
        max: [60.0, 40.0, 20.0],
    };
    let mut engine = Engine::new(ManifoldKernel, &[stock], EngineOptions::default())?;
    let tool = Tool::flat_end_mill(6.0, 20.0);

    let v = Pose::vertical;
    let segments = vec![
        MotionSegment::linear(v([10.0, 10.0, 15.0]), v([50.0, 10.0, 15.0])),
        MotionSegment::linear(v([50.0, 14.0, 15.0]), v([10.0, 14.0, 15.0])),
        MotionSegment::linear(v([10.0, 18.0, 15.0]), v([50.0, 18.0, 15.0])),
        MotionSegment::linear(v([30.0, 30.0, 20.0]), v([30.0, 30.0, 12.0])),
    ];
    let start = std::time::Instant::now();
    let reports = engine.cut_many(&tool, &segments)?;
    for (i, r) in reports.iter().enumerate() {
        println!(
            "segment {i}: {:?} removed {:.3} mm3 in {:.2} ms (rev {})",
            r.outcome,
            r.removed_volume,
            r.elapsed.as_secs_f64() * 1000.0,
            r.revision
        );
    }
    println!(
        "total {:.2} ms, remaining volume {:.3} mm3, parts {}, triangles {}",
        start.elapsed().as_secs_f64() * 1000.0,
        engine.total_volume(),
        engine.parts().len(),
        engine.combined_mesh().triangle_count()
    );
    engine.write_stl(std::io::BufWriter::new(std::fs::File::create(&out)?))?;
    println!("wrote {out}");
    Ok(())
}
