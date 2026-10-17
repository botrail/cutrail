//! ベンチマーク：ジグザグのポケット加工を多数の短い区間で実行し、1 回あたりの時間が履歴で増えないかを見る。
//!
//! ```sh
//! cargo run --release --example bench -- <segments> <tile_size|0> <batch> <simplify_threshold|0> <tilt_deg|0> <flat|ball>
//! ```
//!
//! `tilt_deg` を指定すると、工具軸を Y 軸回りに ±tilt_deg の正弦（周期 40 区間）で変えながら送る（5 軸相当）。
use cutrail::{Engine, EngineOptions, ManifoldKernel, MotionSegment, Pose, StockShape, Tool};
use std::time::{Duration, Instant};

fn arg<T: std::str::FromStr>(i: usize, default: T) -> T {
    std::env::args().nth(i).and_then(|s| s.parse().ok()).unwrap_or(default)
}

/// 素材 200x100x20 に、x 方向のジグザグを 1 mm 区間で刻む。行ピッチ 2 mm、深さは 1.5 mm ずつ下げる。
/// `tilt` が 0 でなければ、区間の通し番号に応じて軸を Y 軸回りに傾ける。
fn zigzag(n: usize, tilt: f64) -> Vec<MotionSegment> {
    let mut out = Vec::with_capacity(n);
    let pose = |p: [f64; 3], k: usize| {
        if tilt == 0.0 {
            Pose::vertical(p)
        } else {
            Pose::tilted_about_y(p, tilt * (2.0 * std::f64::consts::PI * k as f64 / 40.0).sin())
        }
    };
    let (x0, x1) = (5.0, 195.0);
    let rows: Vec<f64> = (0..46).map(|r| 5.0 + 2.0 * r as f64).collect();
    let mut level = 0;
    'outer: loop {
        let z = 18.5 - 1.5 * level as f64;
        if z < 1.0 {
            break;
        }
        for (ri, &y) in rows.iter().enumerate() {
            let forward = (ri + level) % 2 == 0;
            for i in 0..190 {
                let (a, b) = if forward { (x0 + i as f64, x0 + i as f64 + 1.0) } else { (x1 - i as f64, x1 - i as f64 - 1.0) };
                let k = out.len();
                out.push(MotionSegment::linear(pose([a, y, z], k), pose([b, y, z], k + 1)));
                if out.len() == n {
                    break 'outer;
                }
            }
        }
        level += 1;
    }
    out
}

fn percentile(sorted: &[Duration], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx].as_secs_f64() * 1000.0
}

fn stats(times: &[Duration]) -> (f64, f64, f64) {
    let mut s = times.to_vec();
    s.sort();
    (percentile(&s, 0.5), percentile(&s, 0.9), percentile(&s, 1.0))
}

fn rss_mb() -> f64 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines().find(|l| l.starts_with("VmRSS:")).and_then(|l| l.split_whitespace().nth(1)?.parse::<f64>().ok())
        })
        .map(|kb| kb / 1024.0)
        .unwrap_or(f64::NAN)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n: usize = arg(1, 10_000);
    let tile: f64 = arg(2, 0.0);
    let batch: usize = arg(3, 1).max(1);
    let simplify: usize = arg(4, 0);
    let tilt: f64 = arg(5, 0.0);
    let tool_kind: String = arg(6, "flat".to_string());

    let mut options = EngineOptions::default();
    if tile > 0.0 {
        options.tile_size = Some(tile);
    }
    if simplify > 0 {
        options.simplify_threshold = Some(simplify);
    }
    let stock = StockShape::Box { min: [0.0; 3], max: [200.0, 100.0, 20.0] };
    let t_setup = Instant::now();
    let mut engine = Engine::new(ManifoldKernel, &[stock], options)?;
    let setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
    let tool = if tool_kind == "ball" { Tool::ball_end_mill(6.0, 20.0) } else { Tool::flat_end_mill(6.0, 20.0) };
    let segments = zigzag(n, tilt);

    let mut times: Vec<Duration> = Vec::with_capacity(n / batch + 1);
    let started = Instant::now();
    if batch == 1 {
        for seg in &segments {
            let r = engine.cut(&tool, seg)?;
            times.push(r.elapsed);
        }
    } else {
        for chunk in segments.chunks(batch) {
            let r = engine.cut_batch(&tool, chunk)?;
            times.push(r.elapsed);
        }
    }
    let total = started.elapsed();
    let head = &times[..times.len().min(100)];
    let tail = &times[times.len().saturating_sub(100)..];
    let (h50, h90, hmax) = stats(head);
    let (t50, t90, tmax) = stats(tail);
    let (a50, a90, amax) = stats(&times);
    println!(
        "| {} | {} | {} | {} | {} | {} | {} | {:.1} | {:.1} | {:.2} / {:.2} / {:.2} | {:.2} / {:.2} / {:.2} | {:.2} / {:.2} / {:.2} | {} | {} | {:.3} | {:.0} |",
        n,
        tool_kind,
        if tilt != 0.0 { format!("±{tilt}°") } else { "0".into() },
        if tile > 0.0 { format!("{tile}") } else { "なし".into() },
        batch,
        if simplify > 0 { format!("{simplify}") } else { "なし".into() },
        times.len(),
        setup_ms,
        total.as_secs_f64(),
        h50,
        h90,
        hmax,
        t50,
        t90,
        tmax,
        a50,
        a90,
        amax,
        engine.parts().len(),
        engine.total_triangles(),
        engine.total_volume(),
        rss_mb()
    );
    Ok(())
}
