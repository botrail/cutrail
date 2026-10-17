//! Adapter that turns a stream of poses into path segments; the connection point for upstream systems.
//!
//! Tool poses sampled at a physics or render rate are usually too fine. `SegmentBuilder` emits a
//! segment only when the motion since the last committed pose exceeds a distance or angle threshold,
//! coalescing jitter and poses sampled while stationary. With [`SegmentBuilder::with_max_deviation`]
//! it also keeps every skipped sample within a distance of the emitted chord, so corners and arcs are
//! segmented by their shape rather than by the distance threshold alone.

use crate::geom::{dot, norm, sub, Point3};
use crate::motion::{MotionSegment, Pose};

/// Builds linear segments from a sequence of poses.
#[derive(Clone, Debug)]
pub struct SegmentBuilder {
    min_distance: f64,
    min_angle: f64,
    max_deviation: f64,
    anchor: Option<Pose>,
    latest: Option<Pose>,
    skipped: Vec<Pose>,
}

/// Distance from `p` to the segment `a`–`b`.
fn distance_to_segment(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = sub(b, a);
    let len2 = dot(ab, ab);
    let t = if len2 > 0.0 {
        (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    norm(sub(p, q))
}

impl SegmentBuilder {
    /// A builder that emits a segment after `min_distance` (mm) or `min_angle_degrees` of motion.
    pub fn new(min_distance: f64, min_angle_degrees: f64) -> SegmentBuilder {
        SegmentBuilder {
            min_distance: min_distance.max(0.0),
            min_angle: min_angle_degrees.max(0.0).to_radians(),
            max_deviation: 0.0,
            anchor: None,
            latest: None,
            skipped: Vec::new(),
        }
    }

    /// Also emits a segment before a skipped sample would end up farther than `tolerance` (mm) from
    /// the chord between the committed pose and the newest one. With this, `min_distance` only caps
    /// the segment length and the shape of the path decides where segments break. `0` disables it.
    pub fn with_max_deviation(mut self, tolerance: f64) -> SegmentBuilder {
        self.max_deviation = tolerance.max(0.0);
        self
    }

    /// Adds a pose. Returns a linear segment from the last committed pose once a threshold is exceeded.
    pub fn push(&mut self, pose: Pose) -> Option<MotionSegment> {
        let Some(anchor) = self.anchor else {
            self.anchor = Some(pose);
            return None;
        };
        if self.max_deviation > 0.0 {
            if let Some(latest) = self.latest {
                let misses = self
                    .skipped
                    .iter()
                    .any(|s| distance_to_segment(s.tip, anchor.tip, pose.tip) > self.max_deviation);
                if misses {
                    // The chord to `pose` would leave an earlier sample outside the tolerance:
                    // commit the path up to the last pose that still fit, and hold `pose` after it.
                    self.anchor = Some(latest);
                    self.skipped.clear();
                    self.skipped.push(pose);
                    self.latest = Some(pose);
                    return Some(MotionSegment::linear(anchor, latest));
                }
            }
        }
        let moved = norm(sub(pose.tip, anchor.tip)) >= self.min_distance && self.min_distance > 0.0
            || (self.min_distance <= 0.0 && norm(sub(pose.tip, anchor.tip)) > 0.0);
        let turned = self.min_angle > 0.0 && anchor.axis_angle_to(&pose) >= self.min_angle;
        if moved || turned {
            self.anchor = Some(pose);
            self.latest = None;
            self.skipped.clear();
            Some(MotionSegment::linear(anchor, pose))
        } else {
            self.latest = Some(pose);
            if self.max_deviation > 0.0 {
                self.skipped.push(pose);
            }
            None
        }
    }

    /// Emits the motion accumulated below the thresholds as a segment.
    pub fn flush(&mut self) -> Option<MotionSegment> {
        self.skipped.clear();
        let (Some(anchor), Some(latest)) = (self.anchor, self.latest.take()) else {
            return None;
        };
        if norm(sub(latest.tip, anchor.tip)) <= 0.0 && anchor.axis_angle_to(&latest) <= 0.0 {
            return None;
        }
        self.anchor = Some(latest);
        Some(MotionSegment::linear(anchor, latest))
    }

    /// The last committed pose.
    pub fn anchor(&self) -> Option<Pose> {
        self.anchor
    }

    /// Discards the state and starts over at the next pose.
    pub fn reset(&mut self) {
        self.anchor = None;
        self.latest = None;
        self.skipped.clear();
    }
}
