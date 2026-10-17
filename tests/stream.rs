//! Coalescing sampled poses into segments.
mod common;
use common::assert_close;
use cutrail::{MotionSegment, Pose, SegmentBuilder};

fn tips(segments: &[MotionSegment]) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    for s in segments {
        let (from, to) = s.endpoints();
        if out.is_empty() {
            out.push(from.tip);
        }
        out.push(to.tip);
    }
    out
}

fn distance_to_polyline(p: [f64; 3], poly: &[[f64; 3]]) -> f64 {
    poly.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
            let len2 = ab.iter().map(|v| v * v).sum::<f64>();
            let t = if len2 > 0.0 {
                (ap.iter().zip(&ab).map(|(x, y)| x * y).sum::<f64>() / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
            ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn max_deviation_breaks_segments_at_corners() {
    // 10 mm along +x, then 10 mm along +y, sampled every 0.1 mm. The distance threshold alone would
    // chord across the corner; the deviation bound breaks the path at the corner sample.
    let mut samples = Vec::new();
    for i in 0..=100 {
        samples.push(Pose::vertical([i as f64 * 0.1, 0.0, 5.0]));
    }
    for i in 1..=100 {
        samples.push(Pose::vertical([10.0, i as f64 * 0.1, 5.0]));
    }
    let mut builder = SegmentBuilder::new(100.0, 0.0).with_max_deviation(0.02);
    let mut segments: Vec<MotionSegment> = samples.iter().filter_map(|p| builder.push(*p)).collect();
    segments.extend(builder.flush());
    assert_eq!(segments.len(), 2, "one segment per straight stretch: {segments:?}");
    let poly = tips(&segments);
    assert!(
        (poly[1][0] - 10.0).abs() < 1e-9 && poly[1][1].abs() < 1e-9,
        "the break lands on the corner: {poly:?}"
    );
    assert_eq!(poly[2], [10.0, 10.0, 5.0]);

    // Without the bound the same thresholds chord across the corner.
    let mut plain = SegmentBuilder::new(100.0, 0.0);
    let mut chords: Vec<MotionSegment> = samples.iter().filter_map(|p| plain.push(*p)).collect();
    chords.extend(plain.flush());
    assert_eq!(chords.len(), 1);
}

#[test]
fn max_deviation_bounds_the_chord_error_on_arcs() {
    // A quarter circle of radius 10 sampled every 0.5 degrees.
    let r = 10.0;
    let samples: Vec<Pose> = (0..=180)
        .map(|i| {
            let a = (i as f64 * 0.5f64).to_radians();
            Pose::vertical([r * a.cos(), r * a.sin(), 5.0])
        })
        .collect();
    let tol = 0.05;
    let mut builder = SegmentBuilder::new(100.0, 0.0).with_max_deviation(tol);
    let mut segments: Vec<MotionSegment> = samples.iter().filter_map(|p| builder.push(*p)).collect();
    segments.extend(builder.flush());
    let poly = tips(&segments);
    // Sagitta s = c^2 / (8 r): chords of up to 2 mm keep s <= 0.05, so about 8 segments fit the arc.
    assert!((6..=12).contains(&segments.len()), "{} segments", segments.len());
    for p in &samples {
        let d = distance_to_polyline(p.tip, &poly);
        assert!(d <= tol + 1e-9, "sample {:?} is {d} from the polyline", p.tip);
    }
    assert_eq!(poly.last().unwrap(), &samples.last().unwrap().tip);
}

#[test]
fn distance_threshold_still_caps_segment_length() {
    let samples: Vec<Pose> = (0..=200).map(|i| Pose::vertical([i as f64 * 0.1, 0.0, 5.0])).collect();
    let mut builder = SegmentBuilder::new(5.0, 0.0).with_max_deviation(0.01);
    let mut segments: Vec<MotionSegment> = samples.iter().filter_map(|p| builder.push(*p)).collect();
    segments.extend(builder.flush());
    assert_eq!(segments.len(), 4, "{segments:?}");
    for s in &segments {
        let (a, b) = s.endpoints();
        assert!((b.tip[0] - a.tip[0] - 5.0).abs() < 1e-9);
    }
}

#[test]
fn distance_and_angle_thresholds_emit_segments() {
    let mut b = SegmentBuilder::new(1.0, 5.0);
    let mut emitted = Vec::new();
    for i in 0..=55 {
        let x = 0.1 * i as f64; // 0.0 to 5.5 mm
        if let Some(seg) = b.push(Pose::vertical([x, 0.0, 0.0])) {
            emitted.push(seg);
        }
    }
    assert_eq!(emitted.len(), 5, "one segment per millimeter");
    let tail = b.flush().expect("remaining 0.5 mm");
    let (from, to) = tail.endpoints();
    assert_close(to.tip[0] - from.tip[0], 0.5, 1e-9, "flushed length");
    assert!(b.flush().is_none());
    // Rotation alone emits a segment once the angle threshold is exceeded
    let mut r = SegmentBuilder::new(1.0, 5.0);
    assert!(r.push(Pose::vertical([0.0; 3])).is_none());
    assert!(r.push(Pose::tilted_about_y([0.0; 3], 3.0)).is_none());
    assert!(r.push(Pose::tilted_about_y([0.0; 3], 6.0)).is_some());
}
