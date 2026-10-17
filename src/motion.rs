//! Tool poses and path segments. Coordinates are in the stock frame, in millimeters.

use crate::error::{Error, Result};
use crate::geom::{cross, dot, norm, normalize, sub, Point3, Vector3};
use std::f64::consts::TAU;

/// Spherical linear interpolation between unit vectors; `t` runs from 0 to 1.
pub fn slerp(a: Vector3, b: Vector3, t: f64) -> Vector3 {
    let a = normalize(a).unwrap_or([0.0, 0.0, 1.0]);
    let b = normalize(b).unwrap_or(a);
    let cos = dot(a, b).clamp(-1.0, 1.0);
    let angle = cos.acos();
    if angle < 1e-12 {
        return a;
    }
    let s = angle.sin();
    let wa = ((1.0 - t) * angle).sin() / s;
    let wb = (t * angle).sin() / s;
    normalize([wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2]]).unwrap_or(a)
}

/// Tool pose. Tools are bodies of revolution, so the tip position and axis direction define it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Pose {
    /// Position of the tip (center of the bottom of the cutting part)
    pub tip: Point3,
    /// Tool axis direction (unit vector from the tip toward the shank)
    pub axis: Vector3,
}

impl Pose {
    /// A pose with the axis along +Z (3-axis machining).
    pub fn vertical(tip: Point3) -> Pose {
        Pose {
            tip,
            axis: [0.0, 0.0, 1.0],
        }
    }

    /// A pose from a tip position and an axis direction; the axis is normalized.
    pub fn new(tip: Point3, axis: Vector3) -> Pose {
        Pose {
            tip,
            axis: normalize(axis).unwrap_or(axis),
        }
    }

    /// A pose whose axis is +Z tilted about the Y axis by `degrees`; positive angles lean toward +X.
    pub fn tilted_about_y(tip: Point3, degrees: f64) -> Pose {
        let (s, c) = degrees.to_radians().sin_cos();
        Pose { tip, axis: [s, 0.0, c] }
    }

    /// Interpolates between two poses: the tip linearly, the axis spherically.
    pub fn lerp(a: &Pose, b: &Pose, t: f64) -> Pose {
        Pose {
            tip: [
                a.tip[0] + (b.tip[0] - a.tip[0]) * t,
                a.tip[1] + (b.tip[1] - a.tip[1]) * t,
                a.tip[2] + (b.tip[2] - a.tip[2]) * t,
            ],
            axis: slerp(a.axis, b.axis, t),
        }
    }

    /// Angle between the two axes (rad).
    pub fn axis_angle_to(&self, other: &Pose) -> f64 {
        let a = normalize(self.axis).unwrap_or([0.0, 0.0, 1.0]);
        let b = normalize(other.axis).unwrap_or(a);
        dot(a, b).clamp(-1.0, 1.0).acos()
    }

    /// Whether the axis coincides with +Z.
    pub fn is_vertical(&self, tolerance: f64) -> bool {
        match normalize(self.axis) {
            Some(a) => dot(a, [0.0, 0.0, 1.0]) >= 1.0 - tolerance,
            None => false,
        }
    }

    /// Checks that the pose is finite and the axis is non-zero.
    pub fn validate(&self) -> Result<()> {
        if self.tip.iter().any(|v| !v.is_finite()) {
            return Err(Error::InvalidInput(format!("pose tip is not finite: {:?}", self.tip)));
        }
        if normalize(self.axis).is_none() || self.axis.iter().any(|v| !v.is_finite()) {
            return Err(Error::InvalidInput(format!(
                "pose axis is not a valid direction: {:?}",
                self.axis
            )));
        }
        Ok(())
    }
}

/// A path segment.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum MotionSegment {
    /// Linear move: the tip travels in a straight line and the axis rotates spherically from start to end
    Linear {
        /// Start pose
        from: Pose,
        /// End pose
        to: Pose,
    },
    /// Arc move (including helical). The tip turns counterclockwise about `normal` around `center`
    /// from `from.tip` to `to.tip`; a full circle when start and end coincide in the arc plane.
    /// The height along `normal` changes linearly with the angle. The axis rotates spherically
    Arc {
        /// Start pose
        from: Pose,
        /// End pose
        to: Pose,
        /// Arc center
        center: Point3,
        /// Normal of the arc plane (right-handed rotation)
        normal: Vector3,
    },
}

/// Arc geometry (internal).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ArcGeometry {
    /// In-plane unit vector from the center to the start point
    pub u: Vector3,
    /// `normal × u`
    pub v: Vector3,
    /// Normalized normal
    pub n: Vector3,
    /// Radius
    pub radius: f64,
    /// Sweep angle (rad), in (0, 2π]
    pub angle: f64,
    /// Height of the start point along the normal
    pub h0: f64,
    /// Height of the end point along the normal
    pub h1: f64,
}

impl MotionSegment {
    /// Creates a linear move.
    pub fn linear(from: Pose, to: Pose) -> MotionSegment {
        MotionSegment::Linear { from, to }
    }

    /// Creates an arc move.
    pub fn arc(from: Pose, to: Pose, center: Point3, normal: Vector3) -> MotionSegment {
        MotionSegment::Arc {
            from,
            to,
            center,
            normal,
        }
    }

    /// Start and end poses.
    pub fn endpoints(&self) -> (&Pose, &Pose) {
        match self {
            MotionSegment::Linear { from, to } | MotionSegment::Arc { from, to, .. } => (from, to),
        }
    }

    /// Arc geometry, or `None` for a linear segment.
    pub fn arc_geometry(&self) -> Result<Option<ArcGeometry>> {
        let MotionSegment::Arc {
            from,
            to,
            center,
            normal,
        } = self
        else {
            return Ok(None);
        };
        let n = normalize(*normal)
            .ok_or_else(|| Error::InvalidInput(format!("arc normal is not a valid direction: {normal:?}")))?;
        let d0 = sub(from.tip, *center);
        let d1 = sub(to.tip, *center);
        let h0 = dot(d0, n);
        let h1 = dot(d1, n);
        let p0 = [d0[0] - n[0] * h0, d0[1] - n[1] * h0, d0[2] - n[2] * h0];
        let p1 = [d1[0] - n[0] * h1, d1[1] - n[1] * h1, d1[2] - n[2] * h1];
        let r0 = norm(p0);
        let r1 = norm(p1);
        if r0 <= 1e-9 || !r0.is_finite() {
            return Err(Error::InvalidInput("arc start point coincides with the center".into()));
        }
        if (r1 - r0).abs() > 1e-6 * r0.max(1.0) {
            return Err(Error::InvalidInput(format!(
                "arc end point radius {r1} differs from start radius {r0}"
            )));
        }
        let u = [p0[0] / r0, p0[1] / r0, p0[2] / r0];
        let v = cross(n, u);
        let mut angle = dot(p1, v).atan2(dot(p1, u));
        if angle <= 1e-9 {
            angle += TAU;
        }
        Ok(Some(ArcGeometry {
            u,
            v,
            n,
            radius: r0,
            angle,
            h0,
            h1,
        }))
    }

    /// Pose at parameter `t` (0 to 1) along the segment: the tip on the path, the axis by spherical interpolation.
    pub fn pose_at(&self, t: f64) -> Pose {
        match self {
            MotionSegment::Linear { from, to } => Pose::lerp(from, to, t),
            MotionSegment::Arc { from, to, center, .. } => match self.arc_geometry() {
                Ok(Some(g)) => {
                    let theta = g.angle * t;
                    let (s, c) = theta.sin_cos();
                    let h = g.h0 + (g.h1 - g.h0) * t;
                    let tip = [
                        center[0] + g.radius * (c * g.u[0] + s * g.v[0]) + g.n[0] * h,
                        center[1] + g.radius * (c * g.u[1] + s * g.v[1]) + g.n[1] * h,
                        center[2] + g.radius * (c * g.u[2] + s * g.v[2]) + g.n[2] * h,
                    ];
                    Pose {
                        tip,
                        axis: slerp(from.axis, to.axis, t),
                    }
                }
                _ => Pose::lerp(from, to, t),
            },
        }
    }

    /// Validates the segment.
    pub fn validate(&self) -> Result<()> {
        let (from, to) = self.endpoints();
        from.validate()?;
        to.validate()?;
        self.arc_geometry().map(|_| ())
    }
}
