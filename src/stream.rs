//! Adapter that turns a stream of poses into path segments; the connection point for upstream systems.
//!
//! Tool poses sampled at a physics or render rate are usually too fine. `SegmentBuilder` emits a
//! segment only when the motion since the last committed pose exceeds a distance or angle threshold,
//! coalescing jitter and poses sampled while stationary.

use crate::geom::{norm, sub};
use crate::motion::{MotionSegment, Pose};

/// Builds linear segments from a sequence of poses.
#[derive(Clone, Debug)]
pub struct SegmentBuilder {
    min_distance: f64,
    min_angle: f64,
    anchor: Option<Pose>,
    latest: Option<Pose>,
}

impl SegmentBuilder {
    /// A builder that emits a segment after `min_distance` (mm) or `min_angle_degrees` of motion.
    pub fn new(min_distance: f64, min_angle_degrees: f64) -> SegmentBuilder {
        SegmentBuilder {
            min_distance: min_distance.max(0.0),
            min_angle: min_angle_degrees.max(0.0).to_radians(),
            anchor: None,
            latest: None,
        }
    }

    /// Adds a pose. Returns a linear segment from the last committed pose once a threshold is exceeded.
    pub fn push(&mut self, pose: Pose) -> Option<MotionSegment> {
        let Some(anchor) = self.anchor else {
            self.anchor = Some(pose);
            return None;
        };
        let moved = norm(sub(pose.tip, anchor.tip)) >= self.min_distance && self.min_distance > 0.0
            || (self.min_distance <= 0.0 && norm(sub(pose.tip, anchor.tip)) > 0.0);
        let turned = self.min_angle > 0.0 && anchor.axis_angle_to(&pose) >= self.min_angle;
        if moved || turned {
            self.anchor = Some(pose);
            self.latest = None;
            Some(MotionSegment::linear(anchor, pose))
        } else {
            self.latest = Some(pose);
            None
        }
    }

    /// Emits the motion accumulated below the thresholds as a segment.
    pub fn flush(&mut self) -> Option<MotionSegment> {
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
    }
}
