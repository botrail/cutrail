//! Basic geometry types. The public API uses `[f64; 3]` instead of exposing an external math crate.

/// A point in 3D (mm).
pub type Point3 = [f64; 3];
/// A vector in 3D.
pub type Vector3 = [f64; 3];

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum corner
    pub min: Point3,
    /// Maximum corner
    pub max: Point3,
}

impl Aabb {
    /// An empty box containing nothing.
    pub fn empty() -> Self {
        Aabb { min: [f64::INFINITY; 3], max: [f64::NEG_INFINITY; 3] }
    }

    /// The box enclosing a set of points.
    pub fn from_points<I: IntoIterator<Item = Point3>>(points: I) -> Self {
        let mut b = Aabb::empty();
        for p in points {
            b.insert(p);
        }
        b
    }

    /// Grows the box to include a point.
    pub fn insert(&mut self, p: Point3) {
        for (i, v) in p.iter().enumerate() {
            self.min[i] = self.min[i].min(*v);
            self.max[i] = self.max[i].max(*v);
        }
    }

    /// Whether the box is empty.
    pub fn is_empty(&self) -> bool {
        (0..3).any(|i| self.min[i] > self.max[i])
    }

    /// Whether the boxes intersect (boundaries included).
    pub fn intersects(&self, other: &Aabb) -> bool {
        !self.is_empty()
            && !other.is_empty()
            && (0..3).all(|i| self.min[i] <= other.max[i] && other.min[i] <= self.max[i])
    }

    /// The intersection, or `None` if the boxes do not intersect.
    pub fn intersection(&self, other: &Aabb) -> Option<Aabb> {
        if !self.intersects(other) {
            return None;
        }
        let mut r = *self;
        for i in 0..3 {
            r.min[i] = self.min[i].max(other.min[i]);
            r.max[i] = self.max[i].min(other.max[i]);
        }
        Some(r)
    }

    /// The smallest box containing both.
    pub fn union(&self, other: &Aabb) -> Aabb {
        let mut r = *self;
        for i in 0..3 {
            r.min[i] = self.min[i].min(other.min[i]);
            r.max[i] = self.max[i].max(other.max[i]);
        }
        r
    }

    /// Whether `other` lies entirely inside this box (boundaries included).
    pub fn contains(&self, other: &Aabb) -> bool {
        (0..3).all(|i| self.min[i] <= other.min[i] && other.max[i] <= self.max[i])
    }

    /// Extent along each axis.
    pub fn extent(&self) -> Vector3 {
        [self.max[0] - self.min[0], self.max[1] - self.min[1], self.max[2] - self.min[2]]
    }

    /// Volume.
    pub fn volume(&self) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let e = self.extent();
        e[0] * e[1] * e[2]
    }
}

/// Dot product.
pub fn dot(a: Vector3, b: Vector3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product.
pub fn cross(a: Vector3, b: Vector3) -> Vector3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Difference `a - b`.
pub fn sub(a: Point3, b: Point3) -> Vector3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Length.
pub fn norm(a: Vector3) -> f64 {
    dot(a, a).sqrt()
}

/// Normalization; `None` if the length is zero.
pub fn normalize(a: Vector3) -> Option<Vector3> {
    let n = norm(a);
    if n > 0.0 && n.is_finite() {
        Some([a[0] / n, a[1] / n, a[2] / n])
    } else {
        None
    }
}
