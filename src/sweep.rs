//! Swept volume generation. Returns kernel-independent geometric descriptions.
//!
//! Tools are convex bodies of revolution, so the volume swept by a translation equals the convex hull of the end instances.
//! Segments with a rotating axis and arcs are sampled into poses such that the chord error of the farthest
//! tool point stays within `chord_tolerance`, and approximated by the sequence (union) of hulls of consecutive pose pairs.

use crate::error::{Error, Result};
use crate::geom::{cross, normalize, Aabb, Point3, Vector3};
use crate::mesh::TriMesh;
use crate::motion::{MotionSegment, Pose};
use crate::tool::Tool;
use std::f64::consts::{PI, TAU};

/// Axis change (rad) at or beyond which a segment is rejected as an axis flip.
const MAX_AXIS_CHANGE: f64 = 150.0 * PI / 180.0;
/// Upper bound on rotation sub-steps per segment.
const MAX_SUB_STEPS: usize = 256;
/// Upper bound on arc subdivisions per segment.
const MAX_ARC_STEPS: usize = 4096;

/// Geometric description of a swept volume.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SweptVolume {
    /// Convex hull of a point set: the sweep of a translation
    ConvexHull(Vec<Point3>),
    /// Union of several hulls: sub-steps of rotations and arcs
    HullUnion(Vec<Vec<Point3>>),
    /// A closed triangle mesh
    Mesh(TriMesh),
}

impl SweptVolume {
    /// Bounding box.
    pub fn bounds(&self) -> Aabb {
        match self {
            SweptVolume::ConvexHull(points) => Aabb::from_points(points.iter().copied()),
            SweptVolume::HullUnion(hulls) => Aabb::from_points(hulls.iter().flatten().copied()),
            SweptVolume::Mesh(mesh) => mesh.bounds(),
        }
    }

    /// Number of convex pieces.
    pub fn piece_count(&self) -> usize {
        match self {
            SweptVolume::ConvexHull(_) | SweptVolume::Mesh(_) => 1,
            SweptVolume::HullUnion(hulls) => hulls.len(),
        }
    }

    /// Bounding box of each convex piece.
    pub fn piece_bounds(&self) -> Vec<Aabb> {
        match self {
            SweptVolume::HullUnion(hulls) => hulls.iter().map(|h| Aabb::from_points(h.iter().copied())).collect(),
            _ => vec![self.bounds()],
        }
    }
}

/// Number of segments (8 to 1024) that approximate a circle within `chord_tolerance`.
pub fn circle_segments(radius: f64, chord_tolerance: f64) -> usize {
    let tol = chord_tolerance.max(1e-9).min(radius * 0.5);
    let step = 2.0 * (1.0 - tol / radius).acos();
    let n = (TAU / step).ceil();
    if n.is_finite() {
        (n as usize).clamp(8, 1024)
    } else {
        1024
    }
}

/// Circumradius of a regular n-gon whose area equals that of a circle of radius `radius`. Used for cylindrical stock.
pub fn area_equivalent_radius(radius: f64, segments: usize) -> f64 {
    let n = segments as f64;
    let a = TAU / n;
    radius * (TAU / (n * a.sin())).sqrt()
}

/// Circumradius of a regular n-gon whose mean width (perimeter / π) equals the diameter `2 * radius`. Used for swept volumes.
///
/// The volume removed by a straight move is dominated by width × length, so the radius is chosen to match the width on average regardless of direction.
/// The end arcs are about 0.3% smaller in area at n = 32.
pub fn mean_width_radius(radius: f64, segments: usize) -> f64 {
    let n = segments as f64;
    radius * PI / (n * (PI / n).sin())
}

/// Number of pieces (1 to `max_steps`) that split a rotation by `angle` (rad) so that the chord error of the
/// farthest point (at distance `reach` from the center) stays within `chord_tolerance`.
fn chord_steps(angle: f64, reach: f64, chord_tolerance: f64, max_steps: usize) -> usize {
    if angle <= 1e-12 || reach <= 0.0 {
        return 1;
    }
    let ratio = (chord_tolerance.max(1e-12) / reach).min(1.0);
    let step = 2.0 * (1.0 - ratio).acos();
    if step <= 0.0 || !step.is_finite() {
        return max_steps;
    }
    ((angle / step).ceil().max(1.0) as usize).min(max_steps)
}

/// Number of sub-steps (1 to 256) for a tool-axis rotation by `angle` (rad).
pub fn rotation_steps(angle: f64, reach: f64, chord_tolerance: f64) -> usize {
    chord_steps(angle, reach, chord_tolerance, MAX_SUB_STEPS)
}

/// An orthonormal basis perpendicular to the axis.
fn frame(axis: Vector3) -> Option<(Vector3, Vector3)> {
    let axis = normalize(axis)?;
    let helper = if axis[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let u = normalize(cross(axis, helper))?;
    let v = cross(axis, u);
    Some((u, v))
}

/// Point cloud of a revolution profile `(radius, height)` placed at `pose`; radii are scaled by `radial_scale`.
fn revolve_points(
    profile: &[(f64, f64)],
    pose: &Pose,
    segments: usize,
    radial_scale: f64,
) -> Result<Vec<Point3>> {
    let axis = normalize(pose.axis)
        .ok_or_else(|| Error::InvalidInput(format!("pose axis is not a valid direction: {:?}", pose.axis)))?;
    let (u, v) = frame(axis).ok_or_else(|| Error::InvalidInput("degenerate tool axis".into()))?;
    let mut points = Vec::with_capacity(profile.len() * segments);
    for &(radial, height) in profile {
        let c = [pose.tip[0] + axis[0] * height, pose.tip[1] + axis[1] * height, pose.tip[2] + axis[2] * height];
        let r = radial * radial_scale;
        if r <= 1e-12 {
            points.push(c);
            continue;
        }
        for k in 0..segments {
            let (s, cs) = (TAU * k as f64 / segments as f64).sin_cos();
            points.push([
                c[0] + r * (cs * u[0] + s * v[0]),
                c[1] + r * (cs * u[1] + s * v[1]),
                c[2] + r * (cs * u[2] + s * v[2]),
            ]);
        }
    }
    Ok(points)
}

/// Outline points of the cutting part of the tool at `pose`.
pub fn tool_instance_points(tool: &Tool, pose: &Pose, chord_tolerance: f64) -> Result<Vec<Point3>> {
    let n = circle_segments(tool.radius(), chord_tolerance);
    let scale = mean_width_radius(tool.radius(), n) / tool.radius();
    revolve_points(&tool.cutting_profile(chord_tolerance), pose, n, scale)
}

/// Outline points of the non-cutting parts (shank, holder) of the tool at `pose`, one set per cylinder.
pub fn non_cutting_instance_points(tool: &Tool, pose: &Pose, chord_tolerance: f64) -> Result<Vec<Vec<Point3>>> {
    tool.non_cutting_cylinders()
        .iter()
        .map(|&(radius, h0, h1)| {
            let n = circle_segments(radius, chord_tolerance);
            let scale = mean_width_radius(radius, n) / radius;
            revolve_points(&[(radius, h0), (radius, h1)], pose, n, scale)
        })
        .collect()
}

/// Poses along the segment (start and end included), spaced so that the chord error of the farthest tool point stays within `chord_tolerance`.
pub fn sample_poses(tool: &Tool, segment: &MotionSegment, chord_tolerance: f64) -> Result<Vec<Pose>> {
    let (from, to) = segment.endpoints();
    let axis_angle = from.axis_angle_to(to);
    if axis_angle > MAX_AXIS_CHANGE {
        return Err(Error::Unsupported(format!(
            "tool axis changes by {:.1} degrees within one segment; split the segment",
            axis_angle.to_degrees()
        )));
    }
    let reach = tool.reach();
    let mut steps = rotation_steps(axis_angle, reach, chord_tolerance);
    if let Some(arc) = segment.arc_geometry()? {
        let arc_steps = chord_steps(arc.angle, arc.radius + reach, chord_tolerance, MAX_ARC_STEPS);
        steps = steps.max(arc_steps);
    }
    Ok((0..=steps).map(|k| segment.pose_at(k as f64 / steps as f64)).collect())
}

/// Builds, for each pair of consecutive poses, the hull of both instance point sets.
fn hulls_between(
    poses: &[Pose],
    mut points_at: impl FnMut(&Pose) -> Result<Vec<Point3>>,
) -> Result<SweptVolume> {
    let mut prev = points_at(&poses[0])?;
    if poses.len() == 2 {
        let mut hull = prev;
        hull.extend(points_at(&poses[1])?);
        return Ok(SweptVolume::ConvexHull(hull));
    }
    let mut hulls = Vec::with_capacity(poses.len() - 1);
    for pose in &poses[1..] {
        let next = points_at(pose)?;
        let mut hull = prev.clone();
        hull.extend(next.iter().copied());
        hulls.push(hull);
        prev = next;
    }
    Ok(SweptVolume::HullUnion(hulls))
}

/// Volume swept by the cutting part of the tool along the segment.
pub fn swept_volume(tool: &Tool, segment: &MotionSegment, chord_tolerance: f64) -> Result<SweptVolume> {
    tool.validate()?;
    segment.validate()?;
    let poses = sample_poses(tool, segment, chord_tolerance)?;
    hulls_between(&poses, |pose| tool_instance_points(tool, pose, chord_tolerance))
}

/// The swept volume of the cutting part as convex pieces, each with the poses at its ends (for provenance records).
pub fn swept_pieces(tool: &Tool, segment: &MotionSegment, chord_tolerance: f64) -> Result<Vec<(Pose, Pose, Vec<Point3>)>> {
    tool.validate()?;
    segment.validate()?;
    let poses = sample_poses(tool, segment, chord_tolerance)?;
    let mut prev = tool_instance_points(tool, &poses[0], chord_tolerance)?;
    let mut out = Vec::with_capacity(poses.len() - 1);
    for k in 1..poses.len() {
        let next = tool_instance_points(tool, &poses[k], chord_tolerance)?;
        let mut hull = prev.clone();
        hull.extend(next.iter().copied());
        out.push((poses[k - 1], poses[k], hull));
        prev = next;
    }
    Ok(out)
}

/// Volumes swept by the non-cutting parts (shank, holder) along the segment, one per cylinder.
pub fn non_cutting_volumes(tool: &Tool, segment: &MotionSegment, chord_tolerance: f64) -> Result<Vec<SweptVolume>> {
    tool.validate()?;
    segment.validate()?;
    let count = tool.non_cutting_cylinders().len();
    if count == 0 {
        return Ok(Vec::new());
    }
    let poses = sample_poses(tool, segment, chord_tolerance)?;
    (0..count)
        .map(|i| hulls_between(&poses, |pose| Ok(non_cutting_instance_points(tool, pose, chord_tolerance)?.swap_remove(i))))
        .collect()
}
