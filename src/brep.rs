//! Analytic surface reconstruction and B-rep STEP export.
//!
//! After the booleans, every triangle still knows which input (stock or swept volume) it came from.
//! Since the sweep geometry (tool radius, poses) is known, triangles are classified into planes and cylinders, merged into faces,
//! and the face boundaries are rebuilt as line and arc edges. This covers horizontal moves and plunges of flat end mills in 3-axis machining.
//! Triangles that cannot be classified are output as planar faces of merged coplanar triangles (facets).

use crate::error::{Error, Result};
use crate::geom::{cross, dot, norm, normalize, sub, Point3, Vector3};
use crate::mesh::TriMesh;
use crate::motion::Pose;
use std::collections::{HashMap, HashSet};
use std::io::{self, Write};

/// Where a triangle came from.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Source {
    /// The stock
    Stock,
    /// A flat end mill sweep with axis +Z at constant height (a plunge has identical XY for from and to)
    FlatSweep {
        /// Tool radius
        radius: f64,
        /// Circumradius of the polygon used in the mesh
        mesh_radius: f64,
        /// Start pose
        from: Pose,
        /// End pose
        to: Pose,
    },
    /// Anything else (treated as facets)
    Other,
}

/// An analytic surface.
#[derive(Clone, Debug, PartialEq)]
pub enum Surface {
    /// A plane; `normal` points out of the material
    Plane {
        /// A point on the plane
        origin: Point3,
        /// Unit normal
        normal: Vector3,
    },
    /// A cylinder with axis +Z
    Cylinder {
        /// XY position of the axis
        center: [f64; 2],
        /// Radius
        radius: f64,
    },
}

/// Curve of an edge.
#[derive(Clone, Debug, PartialEq)]
pub enum Curve {
    /// Straight line from start to end
    Line,
    /// Arc in a horizontal plane; `ccw` tells whether it runs counterclockwise about +Z from start to end
    Circle {
        /// XY of the center
        center: [f64; 2],
        /// Height
        z: f64,
        /// Radius
        radius: f64,
        /// Counterclockwise
        ccw: bool,
    },
    /// Polyline (start and end included)
    Polyline(Vec<Point3>),
}

/// An edge.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    /// Index of the start vertex
    pub start: usize,
    /// Index of the end vertex
    pub end: usize,
    /// Curve
    pub curve: Curve,
}

/// A face. `loops[outer]` is the outer boundary, the rest are holes. Each loop is a sequence of (edge index, forward).
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    /// Analytic surface
    pub surface: Surface,
    /// Whether the face normal (out of the material) agrees with the surface's natural normal
    pub same_sense: bool,
    /// Boundary loops
    pub loops: Vec<Vec<(usize, bool)>>,
    /// Index of the outer loop
    pub outer: usize,
    /// Number of source triangles
    pub triangles: usize,
}

/// A reconstructed B-rep.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Brep {
    /// Vertices
    pub vertices: Vec<Point3>,
    /// Edges
    pub edges: Vec<Edge>,
    /// Faces
    pub faces: Vec<Face>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Key {
    Plane { n: [i64; 3], d: i64 },
    Cylinder { c: [i64; 2], r: i64 },
}

/// Quantization (1e-7 mm) for surface identity. Triangles from the same plane or cylinder agree within floating-point error.
const Q: f64 = 1e7;

fn q(x: f64) -> i64 {
    (x * Q).round() as i64
}

fn plane_key(normal: Vector3, point: Point3) -> (Key, Surface) {
    let n = normalize(normal).unwrap_or([0.0, 0.0, 1.0]);
    let d = dot(n, point);
    let key = Key::Plane { n: [q(n[0]), q(n[1]), q(n[2])], d: q(d) };
    let origin = [n[0] * d, n[1] * d, n[2] * d];
    (key, Surface::Plane { origin, normal: n })
}

fn cylinder_key(center: [f64; 2], radius: f64) -> (Key, Surface) {
    (Key::Cylinder { c: [q(center[0]), q(center[1])], r: q(radius) }, Surface::Cylinder { center, radius })
}

fn triangle_normal(mesh: &TriMesh, t: [u32; 3]) -> Vector3 {
    let a = mesh.positions[t[0] as usize];
    let b = mesh.positions[t[1] as usize];
    let c = mesh.positions[t[2] as usize];
    normalize(cross(sub(b, a), sub(c, a))).unwrap_or([0.0, 0.0, 1.0])
}

fn centroid(mesh: &TriMesh, t: [u32; 3]) -> Point3 {
    let a = mesh.positions[t[0] as usize];
    let b = mesh.positions[t[1] as usize];
    let c = mesh.positions[t[2] as usize];
    [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0]
}

/// Classifies a triangle from its provenance and geometry.
fn classify(mesh: &TriMesh, t: [u32; 3], source: &Source, tol: f64) -> (Key, Surface) {
    let n = triangle_normal(mesh, t);
    let c = centroid(mesh, t);
    if let Source::FlatSweep { radius, mesh_radius, from, to } = source {
        let vertical = n[2].abs() > 0.999;
        let same_level = (from.tip[2] - to.tip[2]).abs() < 1e-9;
        let d = [to.tip[0] - from.tip[0], to.tip[1] - from.tip[1]];
        let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
        let plunge = len < 1e-9;
        if same_level || plunge {
            if vertical {
                // The floor (height of the lowest tip); any other horizontal plane keeps the triangle's own height
                let floor = from.tip[2].min(to.tip[2]);
                let z = if (c[2] - floor).abs() < tol { floor } else { c[2] };
                return plane_key([0.0, 0.0, if n[2] > 0.0 { 1.0 } else { -1.0 }], [c[0], c[1], z]);
            }
            if plunge {
                return cylinder_key([from.tip[0], from.tip[1]], *radius);
            }
            let u = [d[0] / len, d[1] / len];
            let left = [-u[1], u[0]];
            let rel = [c[0] - from.tip[0], c[1] - from.tip[1]];
            let t_along = rel[0] * u[0] + rel[1] * u[1];
            let side = rel[0] * left[0] + rel[1] * left[1];
            let cap_tol = tol + (mesh_radius - radius).abs();
            if t_along <= 1e-6 {
                let dist = norm([rel[0], rel[1], 0.0]);
                if (dist - mesh_radius).abs() < cap_tol {
                    return cylinder_key([from.tip[0], from.tip[1]], *radius);
                }
            } else if t_along >= len - 1e-6 {
                let rel_b = [c[0] - to.tip[0], c[1] - to.tip[1]];
                let dist = norm([rel_b[0], rel_b[1], 0.0]);
                if (dist - mesh_radius).abs() < cap_tol {
                    return cylinder_key([to.tip[0], to.tip[1]], *radius);
                }
            } else if (side.abs() - mesh_radius).abs() < cap_tol && (n[0] * u[0] + n[1] * u[1]).abs() < 1e-3 {
                // Straight side wall; the true wall is the plane at radius r from the path
                let sign = if side > 0.0 { 1.0 } else { -1.0 };
                let normal = [left[0] * sign, left[1] * sign, 0.0];
                let point = [from.tip[0] + left[0] * sign * radius, from.tip[1] + left[1] * sign * radius, c[2]];
                return plane_key(normal, point);
            }
        }
    }
    // Stock and everything else: the triangle's own plane
    plane_key(n, c)
}

struct UnionFind(Vec<usize>);

impl UnionFind {
    fn new(n: usize) -> Self {
        UnionFind((0..n).collect())
    }
    fn find(&mut self, i: usize) -> usize {
        let mut r = i;
        while self.0[r] != r {
            r = self.0[r];
        }
        let mut x = i;
        while self.0[x] != r {
            let next = self.0[x];
            self.0[x] = r;
            x = next;
        }
        r
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra] = rb;
        }
    }
}

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn angle_about(center: [f64; 2], p: Point3) -> f64 {
    (p[1] - center[1]).atan2(p[0] - center[0])
}

/// Snaps a vertex by projecting it onto every analytic surface it touches.
fn snap(mut p: Point3, surfaces: &[&Surface]) -> Point3 {
    for _ in 0..64 {
        let before = p;
        for s in surfaces {
            match s {
                Surface::Plane { origin, normal } => {
                    let d = dot(*normal, sub(p, *origin));
                    p = [p[0] - normal[0] * d, p[1] - normal[1] * d, p[2] - normal[2] * d];
                }
                Surface::Cylinder { center, radius } => {
                    let dx = p[0] - center[0];
                    let dy = p[1] - center[1];
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist > 1e-12 {
                        p[0] = center[0] + dx / dist * radius;
                        p[1] = center[1] + dy / dist * radius;
                    }
                }
            }
        }
        if norm(sub(p, before)) < 1e-13 {
            break;
        }
    }
    p
}

fn on_surface(p: Point3, s: &Surface, tol: f64) -> bool {
    match s {
        Surface::Plane { origin, normal } => dot(*normal, sub(p, *origin)).abs() <= tol,
        Surface::Cylinder { center, radius } => {
            let dx = p[0] - center[0];
            let dy = p[1] - center[1];
            ((dx * dx + dy * dy).sqrt() - radius).abs() <= tol
        }
    }
}

fn is_horizontal(s: &Surface) -> Option<f64> {
    match s {
        Surface::Plane { origin, normal } if normal[2].abs() > 0.999 => Some(origin[2]),
        _ => None,
    }
}

/// Chooses the curve of an edge from the two adjacent surfaces.
fn fit_curve(chain: &[Point3], a: &Surface, b: &Surface, start: Point3, end: Point3, tol: f64) -> Curve {
    match (a, b) {
        (Surface::Plane { .. }, Surface::Plane { .. }) => Curve::Line,
        (Surface::Plane { .. }, Surface::Cylinder { center, radius })
        | (Surface::Cylinder { center, radius }, Surface::Plane { .. }) => {
            let plane = if matches!(a, Surface::Plane { .. }) { a } else { b };
            if let Some(z) = is_horizontal(plane) {
                // The direction follows the angular increments of the original vertex sequence
                let mut total = 0.0;
                for w in chain.windows(2) {
                    let mut d = angle_about(*center, w[1]) - angle_about(*center, w[0]);
                    if d > std::f64::consts::PI {
                        d -= 2.0 * std::f64::consts::PI;
                    } else if d < -std::f64::consts::PI {
                        d += 2.0 * std::f64::consts::PI;
                    }
                    total += d;
                }
                Curve::Circle { center: *center, z, radius: *radius, ccw: total >= 0.0 }
            } else if (start[0] - end[0]).abs() < tol && (start[1] - end[1]).abs() < tol {
                Curve::Line
            } else {
                Curve::Polyline(chain.to_vec())
            }
        }
        (Surface::Cylinder { .. }, Surface::Cylinder { .. }) => {
            if (start[0] - end[0]).abs() < tol && (start[1] - end[1]).abs() < tol {
                Curve::Line
            } else {
                Curve::Polyline(chain.to_vec())
            }
        }
    }
}

/// Reconstructs a B-rep from a mesh and its provenance.
///
/// `sources` maps provenance IDs to [`Source`]. `tolerance` is the distance (mm) used for classification and vertex matching.
pub fn reconstruct(mesh: &TriMesh, source_ids: &[u32], sources: &HashMap<u32, Source>, tolerance: f64) -> Result<Brep> {
    let n_tri = mesh.triangles.len();
    if n_tri == 0 {
        return Err(Error::InvalidInput("mesh has no triangles".into()));
    }
    if source_ids.len() != n_tri {
        return Err(Error::InvalidInput("source ids must match the triangle count".into()));
    }
    let tol = tolerance.max(1e-9);
    let other = Source::Other;

    // 1. Classification
    let mut keys = Vec::with_capacity(n_tri);
    let mut surfaces: HashMap<Key, Surface> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        let source = sources.get(&source_ids[i]).unwrap_or(&other);
        let (key, surface) = classify(mesh, *t, source, tol);
        surfaces.entry(key).or_insert(surface);
        keys.push(key);
    }

    // 2. Merge triangles that share an edge and lie on the same surface
    let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            edge_to_tris.entry(edge_key(t[k], t[(k + 1) % 3])).or_default().push(i);
        }
    }
    let mut uf = UnionFind::new(n_tri);
    for tris in edge_to_tris.values() {
        for w in tris.windows(2) {
            if keys[w[0]] == keys[w[1]] {
                uf.union(w[0], w[1]);
            }
        }
    }
    let mut patch_count = 0;
    let mut root_to_patch: HashMap<usize, usize> = HashMap::new();
    let mut patch_of: Vec<usize> = (0..n_tri)
        .map(|i| {
            let r = uf.find(i);
            *root_to_patch.entry(r).or_insert_with(|| {
                patch_count += 1;
                patch_count - 1
            })
        })
        .collect();
    let mut patch_key: Vec<Key> = vec![keys[0]; patch_count];
    for i in 0..n_tri {
        patch_key[patch_of[i]] = keys[i];
    }

    // 3. Split cylindrical faces spanning more than 180° in two by angle (faces added here are not revisited)
    let original_patch_count = patch_count;
    for p in 0..original_patch_count {
        let Key::Cylinder { .. } = patch_key[p] else { continue };
        let Surface::Cylinder { center, .. } = surfaces[&patch_key[p]].clone() else { continue };
        let members: Vec<usize> = (0..n_tri).filter(|&i| patch_of[i] == p).collect();
        if members.len() < 2 {
            continue;
        }
        let mut angles: Vec<(f64, usize)> = members.iter().map(|&i| (angle_about(center, centroid(mesh, mesh.triangles[i])), i)).collect();
        angles.sort_by(|a, b| a.0.total_cmp(&b.0));
        let m = angles.len();
        let mut gap_max = 0.0;
        let mut gap_at = 0;
        for k in 0..m {
            let a0 = angles[k].0;
            let a1 = if k + 1 < m { angles[k + 1].0 } else { angles[0].0 + std::f64::consts::TAU };
            if a1 - a0 > gap_max {
                gap_max = a1 - a0;
                gap_at = k;
            }
        }
        let extent = std::f64::consts::TAU - gap_max;
        if extent <= std::f64::consts::PI + 1e-6 {
            continue;
        }
        let start = angles[(gap_at + 1) % m].0;
        let new_patch = patch_count;
        patch_count += 1;
        patch_key.push(patch_key[p]);
        for &(a, i) in &angles {
            let mut rel = a - start;
            if rel < -1e-9 {
                rel += std::f64::consts::TAU;
            }
            if rel >= extent / 2.0 - 1e-9 {
                patch_of[i] = new_patch;
            }
        }
    }

    // 4. Boundary half-edges and corner vertices
    let mut half: HashMap<(u32, u32), usize> = HashMap::with_capacity(n_tri * 3);
    for (i, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            half.insert((t[k], t[(k + 1) % 3]), patch_of[i]);
        }
    }
    let mut vertex_patches: HashMap<u32, HashSet<usize>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        for &v in t {
            vertex_patches.entry(v).or_default().insert(patch_of[i]);
        }
    }
    let boundary = |a: u32, b: u32| -> bool { half.get(&(a, b)) != half.get(&(b, a)) };
    // Boundary loops per patch (following the triangle winding, i.e. counterclockwise seen from outside)
    let mut loops_of_patch: Vec<Vec<Vec<u32>>> = vec![Vec::new(); patch_count];
    {
        let mut outgoing: HashMap<(usize, u32), Vec<u32>> = HashMap::new();
        for (&(a, b), &p) in &half {
            if boundary(a, b) {
                outgoing.entry((p, a)).or_default().push(b);
            }
        }
        let mut used: HashSet<(u32, u32)> = HashSet::new();
        let mut starts: Vec<((usize, u32), u32)> = Vec::new();
        for (&(p, a), bs) in &outgoing {
            for &b in bs {
                starts.push(((p, a), b));
            }
        }
        starts.sort();
        for ((p, a), b) in starts {
            if used.contains(&(a, b)) {
                continue;
            }
            let mut lp = vec![a];
            let (mut ca, mut cb) = (a, b);
            loop {
                used.insert((ca, cb));
                lp.push(cb);
                if cb == a {
                    break;
                }
                let Some(next) = outgoing.get(&(p, cb)).and_then(|bs| bs.iter().copied().find(|&n| !used.contains(&(cb, n)))) else {
                    break;
                };
                ca = cb;
                cb = next;
                if lp.len() > n_tri * 3 + 3 {
                    return Err(Error::Validation("boundary loop walk did not terminate".into()));
                }
            }
            if lp.last() == Some(&a) {
                lp.pop();
                if lp.len() >= 2 {
                    loops_of_patch[p].push(lp);
                }
            }
        }
    }
    let mut is_corner: HashSet<u32> = vertex_patches.iter().filter(|(_, ps)| ps.len() >= 3).map(|(&v, _)| v).collect();
    for lps in &loops_of_patch {
        for lp in lps {
            if lp.iter().filter(|v| is_corner.contains(v)).count() < 2 {
                is_corner.insert(lp[0]);
                is_corner.insert(lp[lp.len() / 2]);
            }
        }
    }

    // 5. Vertex snapping
    let patch_surface = |p: usize| -> &Surface { &surfaces[&patch_key[p]] };
    let mut vertex_index: HashMap<u32, usize> = HashMap::new();
    let mut brep = Brep::default();
    let mut vid = |v: u32, brep: &mut Brep| -> usize {
        if let Some(&i) = vertex_index.get(&v) {
            return i;
        }
        let surfs: Vec<&Surface> = vertex_patches.get(&v).map(|ps| ps.iter().map(|&p| patch_surface(p)).collect()).unwrap_or_default();
        let p = snap(mesh.positions[v as usize], &surfs);
        brep.vertices.push(p);
        vertex_index.insert(v, brep.vertices.len() - 1);
        brep.vertices.len() - 1
    };

    // 6. Split loops at corners into edges and assemble faces
    let mut edge_index: HashMap<Vec<u32>, usize> = HashMap::new();
    let mut patch_tri_count = vec![0usize; patch_count];
    for &p in &patch_of {
        patch_tri_count[p] += 1;
    }
    let mut face_normals: Vec<Option<Vector3>> = vec![None; patch_count];
    for (i, t) in mesh.triangles.iter().enumerate() {
        if face_normals[patch_of[i]].is_none() {
            face_normals[patch_of[i]] = Some(triangle_normal(mesh, *t));
        }
    }
    for p in 0..patch_count {
        if loops_of_patch[p].is_empty() {
            continue;
        }
        let surface = patch_surface(p).clone();
        let mesh_normal = face_normals[p].unwrap_or([0.0, 0.0, 1.0]);
        let natural = match &surface {
            Surface::Plane { normal, .. } => *normal,
            Surface::Cylinder { center, .. } => {
                let c = centroid(mesh, mesh.triangles[(0..n_tri).find(|&i| patch_of[i] == p).unwrap()]);
                normalize([c[0] - center[0], c[1] - center[1], 0.0]).unwrap_or([1.0, 0.0, 0.0])
            }
        };
        let same_sense = dot(mesh_normal, natural) >= 0.0;
        let mut face_loops: Vec<Vec<(usize, bool)>> = Vec::new();
        let mut loop_areas: Vec<f64> = Vec::new();
        for lp in &loops_of_patch[p] {
            let m = lp.len();
            let Some(first_corner) = (0..m).find(|&k| is_corner.contains(&lp[k])) else { continue };
            let rotated: Vec<u32> = (0..m).map(|k| lp[(first_corner + k) % m]).collect();
            let mut chains: Vec<Vec<u32>> = Vec::new();
            let mut current = vec![rotated[0]];
            for &v in rotated.iter().skip(1).chain(std::iter::once(&rotated[0])) {
                current.push(v);
                if is_corner.contains(&v) {
                    chains.push(std::mem::take(&mut current));
                    current.push(v);
                }
            }
            let mut loop_edges = Vec::with_capacity(chains.len());
            for chain in &chains {
                let canonical = {
                    let mut rev = chain.clone();
                    rev.reverse();
                    if rev < *chain { rev } else { chain.clone() }
                };
                let forward = canonical == *chain;
                let idx = if let Some(&e) = edge_index.get(&canonical) {
                    e
                } else {
                    let start = vid(canonical[0], &mut brep);
                    let end = vid(*canonical.last().unwrap(), &mut brep);
                    let other_patch = {
                        let (a, b) = (canonical[0], canonical[1]);
                        // Neighboring patch: whichever of the two half-edge owners is not p
                        let pa = half.get(&(a, b)).copied();
                        let pb = half.get(&(b, a)).copied();
                        match (pa, pb) {
                            (Some(x), Some(y)) if x == p => Some(y),
                            (Some(x), Some(_)) => Some(x),
                            (Some(x), None) | (None, Some(x)) => Some(x),
                            _ => None,
                        }
                    };
                    let other_surface = other_patch.map(patch_surface).unwrap_or(&surface).clone();
                    let pts: Vec<Point3> = canonical.iter().map(|&v| snap(mesh.positions[v as usize], &[&surface, &other_surface])).collect();
                    let curve = fit_curve(&pts, &surface, &other_surface, brep.vertices[start], brep.vertices[end], tol);
                    brep.edges.push(Edge { start, end, curve });
                    edge_index.insert(canonical, brep.edges.len() - 1);
                    brep.edges.len() - 1
                };
                loop_edges.push((idx, forward));
            }
            // Signed area (positive when counterclockwise about the face normal)
            let area = match &surface {
                Surface::Plane { .. } => {
                    let mut a = [0.0; 3];
                    for k in 0..m {
                        let v0 = mesh.positions[lp[k] as usize];
                        let v1 = mesh.positions[lp[(k + 1) % m] as usize];
                        let c = cross(v0, v1);
                        a = [a[0] + c[0], a[1] + c[1], a[2] + c[2]];
                    }
                    dot(a, mesh_normal) * 0.5
                }
                Surface::Cylinder { center, radius } => {
                    let base = angle_about(*center, mesh.positions[lp[0] as usize]);
                    let uv: Vec<[f64; 2]> = lp
                        .iter()
                        .map(|&v| {
                            let pt = mesh.positions[v as usize];
                            let mut th = angle_about(*center, pt) - base;
                            if th > std::f64::consts::PI {
                                th -= std::f64::consts::TAU;
                            } else if th < -std::f64::consts::PI {
                                th += std::f64::consts::TAU;
                            }
                            [th * radius, pt[2]]
                        })
                        .collect();
                    let mut a = 0.0;
                    for k in 0..m {
                        let (p0, p1) = (uv[k], uv[(k + 1) % m]);
                        a += p0[0] * p1[1] - p1[0] * p0[1];
                    }
                    a * 0.5 * if same_sense { 1.0 } else { -1.0 }
                }
            };
            face_loops.push(loop_edges);
            loop_areas.push(area);
        }
        if face_loops.is_empty() {
            continue;
        }
        let outer = loop_areas.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).map(|(k, _)| k).unwrap_or(0);
        brep.faces.push(Face { surface, same_sense, loops: face_loops, outer, triangles: patch_tri_count[p] });
    }
    if brep.faces.is_empty() {
        return Err(Error::Validation("no faces reconstructed".into()));
    }
    Ok(brep)
}

impl Brep {
    /// Number of edges per curve type (lines, arcs, polylines).
    pub fn curve_counts(&self) -> (usize, usize, usize) {
        let mut c = (0, 0, 0);
        for e in &self.edges {
            match e.curve {
                Curve::Line => c.0 += 1,
                Curve::Circle { .. } => c.1 += 1,
                Curve::Polyline(_) => c.2 += 1,
            }
        }
        c
    }

    /// Number of faces per surface type (planes, cylinders).
    pub fn surface_counts(&self) -> (usize, usize) {
        let mut c = (0, 0);
        for f in &self.faces {
            match f.surface {
                Surface::Plane { .. } => c.0 += 1,
                Surface::Cylinder { .. } => c.1 += 1,
            }
        }
        c
    }

    /// Consistency check: loops are closed and edge endpoints lie on both adjacent faces.
    pub fn check(&self, tolerance: f64) -> Result<()> {
        let mut edge_faces: HashMap<usize, Vec<usize>> = HashMap::new();
        for (fi, face) in self.faces.iter().enumerate() {
            for lp in &face.loops {
                if lp.is_empty() {
                    return Err(Error::Validation(format!("face {fi} has an empty loop")));
                }
                for k in 0..lp.len() {
                    let (e0, f0) = lp[k];
                    let (e1, f1) = lp[(k + 1) % lp.len()];
                    let end0 = if f0 { self.edges[e0].end } else { self.edges[e0].start };
                    let start1 = if f1 { self.edges[e1].start } else { self.edges[e1].end };
                    if end0 != start1 {
                        return Err(Error::Validation(format!("face {fi}: loop is not closed between edges {e0} and {e1}")));
                    }
                    edge_faces.entry(e0).or_default().push(fi);
                }
            }
        }
        for (ei, edge) in self.edges.iter().enumerate() {
            let faces = edge_faces.get(&ei).cloned().unwrap_or_default();
            if faces.len() != 2 {
                return Err(Error::Validation(format!("edge {ei} is used by {} faces", faces.len())));
            }
            for &fi in &faces {
                for v in [edge.start, edge.end] {
                    if !on_surface(self.vertices[v], &self.faces[fi].surface, tolerance) {
                        return Err(Error::Validation(format!("edge {ei}: vertex {v} is off face {fi} by more than {tolerance}")));
                    }
                }
                if let Curve::Circle { center, z, radius, .. } = &edge.curve {
                    for v in [edge.start, edge.end] {
                        let p = self.vertices[v];
                        let d = ((p[0] - center[0]).powi(2) + (p[1] - center[1]).powi(2)).sqrt();
                        if (d - radius).abs() > tolerance || (p[2] - z).abs() > tolerance {
                            return Err(Error::Validation(format!("edge {ei}: circle endpoint off the circle")));
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Writes AP214 STEP (MANIFOLD_SOLID_BREP with planes and cylinders, lines, circles and polylines).
    pub fn write_step<W: Write>(&self, mut w: W, name: &str) -> io::Result<()> {
        let mut out = String::new();
        let mut next = 1usize;
        let mut line = |out: &mut String, body: String| -> usize {
            let id = next;
            next += 1;
            out.push_str(&format!("#{id} = {body};\n"));
            id
        };
        let real = |x: f64| -> String {
            let v = if x.abs() < 1e-15 { 0.0 } else { x };
            let mut s = format!("{v:.9}");
            while s.ends_with('0') {
                s.pop();
            }
            if s.ends_with('.') {
                s.push('0');
            }
            s
        };
        let point = |out: &mut String, line: &mut dyn FnMut(&mut String, String) -> usize, p: Point3| -> usize {
            line(out, format!("CARTESIAN_POINT('', ({}, {}, {}))", real(p[0]), real(p[1]), real(p[2])))
        };
        let direction = |out: &mut String, line: &mut dyn FnMut(&mut String, String) -> usize, d: Vector3| -> usize {
            line(out, format!("DIRECTION('', ({}, {}, {}))", real(d[0]), real(d[1]), real(d[2])))
        };

        // Context
        let app = line(&mut out, "APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into());
        line(&mut out, format!("APPLICATION_PROTOCOL_DEFINITION('international standard', 'automotive_design', 2000, #{app})"));
        let pctx = line(&mut out, format!("PRODUCT_CONTEXT('', #{app}, 'mechanical')"));
        let product = line(&mut out, format!("PRODUCT('{name}', '{name}', '', (#{pctx}))"));
        line(&mut out, format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part', $, (#{product}))"));
        let formation = line(&mut out, format!("PRODUCT_DEFINITION_FORMATION('', '', #{product})"));
        let dctx = line(&mut out, format!("PRODUCT_DEFINITION_CONTEXT('part definition', #{app}, 'design')"));
        let definition = line(&mut out, format!("PRODUCT_DEFINITION('design', '', #{formation}, #{dctx})"));
        let shape = line(&mut out, format!("PRODUCT_DEFINITION_SHAPE('', '', #{definition})"));
        let lu = line(&mut out, "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI., .METRE.) )".into());
        let au = line(&mut out, "( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($, .RADIAN.) )".into());
        let su = line(&mut out, "( NAMED_UNIT(*) SI_UNIT($, .STERADIAN.) SOLID_ANGLE_UNIT() )".into());
        let unc = line(&mut out, format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-6), #{lu}, 'distance_accuracy_value', 'confusion accuracy')"));
        let ctx = line(&mut out, format!("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu}, #{au}, #{su})) REPRESENTATION_CONTEXT('Context #1', '3D Context with UNIT and UNCERTAINTY') )"));

        // Vertices
        let mut vertex_ids = Vec::with_capacity(self.vertices.len());
        for &p in &self.vertices {
            let pid = point(&mut out, &mut line, p);
            vertex_ids.push(line(&mut out, format!("VERTEX_POINT('', #{pid})")));
        }
        // Edges
        let mut edge_ids = Vec::with_capacity(self.edges.len());
        for e in &self.edges {
            let (s, t) = (self.vertices[e.start], self.vertices[e.end]);
            let curve = match &e.curve {
                Curve::Line => {
                    let d = sub(t, s);
                    let len = norm(d).max(1e-12);
                    let p = point(&mut out, &mut line, s);
                    let dir = direction(&mut out, &mut line, [d[0] / len, d[1] / len, d[2] / len]);
                    let vec = line(&mut out, format!("VECTOR('', #{dir}, {})", real(len)));
                    line(&mut out, format!("LINE('', #{p}, #{vec})"))
                }
                Curve::Circle { center, z, radius, ccw } => {
                    let c = point(&mut out, &mut line, [center[0], center[1], *z]);
                    let axis = direction(&mut out, &mut line, [0.0, 0.0, if *ccw { 1.0 } else { -1.0 }]);
                    let refd = direction(&mut out, &mut line, [1.0, 0.0, 0.0]);
                    let place = line(&mut out, format!("AXIS2_PLACEMENT_3D('', #{c}, #{axis}, #{refd})"));
                    line(&mut out, format!("CIRCLE('', #{place}, {})", real(*radius)))
                }
                Curve::Polyline(pts) => {
                    let ids: Vec<String> = pts.iter().map(|&p| format!("#{}", point(&mut out, &mut line, p))).collect();
                    line(&mut out, format!("POLYLINE('', ({}))", ids.join(", ")))
                }
            };
            edge_ids.push(line(&mut out, format!("EDGE_CURVE('', #{}, #{}, #{curve}, .T.)", vertex_ids[e.start], vertex_ids[e.end])));
        }
        // Faces
        let mut face_ids = Vec::with_capacity(self.faces.len());
        for face in &self.faces {
            let surface = match &face.surface {
                Surface::Plane { origin, normal } => {
                    let helper = if normal[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                    let x = normalize(cross(*normal, helper)).unwrap_or([1.0, 0.0, 0.0]);
                    let x = normalize(cross(x, *normal)).unwrap_or(x);
                    let o = point(&mut out, &mut line, *origin);
                    let n = direction(&mut out, &mut line, *normal);
                    let xd = direction(&mut out, &mut line, x);
                    let place = line(&mut out, format!("AXIS2_PLACEMENT_3D('', #{o}, #{n}, #{xd})"));
                    line(&mut out, format!("PLANE('', #{place})"))
                }
                Surface::Cylinder { center, radius } => {
                    let o = point(&mut out, &mut line, [center[0], center[1], 0.0]);
                    let n = direction(&mut out, &mut line, [0.0, 0.0, 1.0]);
                    let xd = direction(&mut out, &mut line, [1.0, 0.0, 0.0]);
                    let place = line(&mut out, format!("AXIS2_PLACEMENT_3D('', #{o}, #{n}, #{xd})"));
                    line(&mut out, format!("CYLINDRICAL_SURFACE('', #{place}, {})", real(*radius)))
                }
            };
            let mut bound_ids = Vec::with_capacity(face.loops.len());
            for (k, lp) in face.loops.iter().enumerate() {
                let oriented: Vec<String> = lp
                    .iter()
                    .map(|&(e, forward)| format!("#{}", line(&mut out, format!("ORIENTED_EDGE('', *, *, #{}, {})", edge_ids[e], if forward { ".T." } else { ".F." }))))
                    .collect();
                let loop_id = line(&mut out, format!("EDGE_LOOP('', ({}))", oriented.join(", ")));
                let kind = if k == face.outer { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bound_ids.push(format!("#{}", line(&mut out, format!("{kind}('', #{loop_id}, .T.)"))));
            }
            face_ids.push(format!(
                "#{}",
                line(&mut out, format!("ADVANCED_FACE('', ({}), #{surface}, {})", bound_ids.join(", "), if face.same_sense { ".T." } else { ".F." }))
            ));
        }
        let shell = line(&mut out, format!("CLOSED_SHELL('', ({}))", face_ids.join(", ")));
        let brep = line(&mut out, format!("MANIFOLD_SOLID_BREP('', #{shell})"));
        let o = point(&mut out, &mut line, [0.0, 0.0, 0.0]);
        let z = direction(&mut out, &mut line, [0.0, 0.0, 1.0]);
        let x = direction(&mut out, &mut line, [1.0, 0.0, 0.0]);
        let origin = line(&mut out, format!("AXIS2_PLACEMENT_3D('', #{o}, #{z}, #{x})"));
        let repr = line(&mut out, format!("ADVANCED_BREP_SHAPE_REPRESENTATION('', (#{origin}, #{brep}), #{ctx})"));
        line(&mut out, format!("SHAPE_DEFINITION_REPRESENTATION(#{shape}, #{repr})"));

        writeln!(w, "ISO-10303-21;")?;
        writeln!(w, "HEADER;")?;
        writeln!(w, "FILE_DESCRIPTION(('cutrail machined stock'), '2;1');")?;
        writeln!(w, "FILE_NAME('{name}.step', '2026-10-17T00:00:00', ('cutrail'), (''), 'cutrail', 'cutrail', '');")?;
        writeln!(w, "FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));")?;
        writeln!(w, "ENDSEC;")?;
        writeln!(w, "DATA;")?;
        w.write_all(out.as_bytes())?;
        writeln!(w, "ENDSEC;")?;
        writeln!(w, "END-ISO-10303-21;")?;
        Ok(())
    }

    /// Writes a STEP file.
    pub fn write_step_file<P: AsRef<std::path::Path>>(&self, path: P, name: &str) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        self.write_step(io::BufWriter::new(file), name)
    }
}
