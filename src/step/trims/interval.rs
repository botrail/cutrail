//! Outward-rounded intervals and Bernstein polynomials used only to accept fitted trims.
//! A non-finite or unresolved bound is never a successful fit.

use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct I {
    pub lo: f64,
    pub hi: f64,
}

impl I {
    pub const ZERO: Self = Self { lo: 0., hi: 0. };
    pub const ONE: Self = Self { lo: 1., hi: 1. };

    pub fn point(x: f64) -> Self {
        if x.is_finite() {
            Self { lo: x, hi: x }
        } else {
            Self::whole()
        }
    }

    fn whole() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }

    fn rounded(lo: f64, hi: f64) -> Self {
        if lo.is_nan() || hi.is_nan() {
            Self::whole()
        } else {
            Self {
                lo: lo.next_down(),
                hi: hi.next_up(),
            }
        }
    }

    pub fn abs_upper(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }

    pub fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }
}

impl Add for I {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        if self == Self::ZERO {
            return rhs;
        }
        if rhs == Self::ZERO {
            return self;
        }
        Self::rounded(self.lo + rhs.lo, self.hi + rhs.hi)
    }
}

impl Sub for I {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        if rhs == Self::ZERO {
            return self;
        }
        Self::rounded(self.lo - rhs.hi, self.hi - rhs.lo)
    }
}

impl Mul for I {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        if self == Self::ZERO || rhs == Self::ZERO {
            return Self::ZERO;
        }
        if self == Self::ONE {
            return rhs;
        }
        if rhs == Self::ONE {
            return self;
        }
        let values = [self.lo * rhs.lo, self.lo * rhs.hi, self.hi * rhs.lo, self.hi * rhs.hi];
        if values.iter().any(|x| x.is_nan()) {
            return Self::whole();
        }
        Self::rounded(
            values.into_iter().fold(f64::INFINITY, f64::min),
            values.into_iter().fold(f64::NEG_INFINITY, f64::max),
        )
    }
}

impl Div for I {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        if rhs.lo <= 0. && rhs.hi >= 0. {
            return Self::whole();
        }
        if rhs == Self::ONE {
            return self;
        }
        self * Self::rounded(1. / rhs.hi, 1. / rhs.lo)
    }
}

pub(super) type Poly = Vec<I>;
pub(super) type H = [I; 4];

pub(super) fn add(a: &[I], b: &[I]) -> Poly {
    debug_assert_eq!(a.len(), b.len());
    a.iter().zip(b).map(|(&a, &b)| a + b).collect()
}

pub(super) fn one_minus(a: &[I]) -> Poly {
    a.iter().map(|&a| I::ONE - a).collect()
}

/// Bernstein product: (a*b)_k = sum_{i+j=k} a_i b_j C(m,i) C(n,j)/C(m+n,k).
pub(super) fn product(a: &[I], b: &[I]) -> Poly {
    let m = a.len() - 1;
    let n = b.len() - 1;
    let choose = binomials();
    let mut out = vec![I::ZERO; m + n + 1];
    for (i, &a) in a.iter().enumerate() {
        for (j, &b) in b.iter().enumerate() {
            let weight = choose[m][i] * choose[n][j] / choose[m + n][i + j];
            out[i + j] = out[i + j] + a * b * weight;
        }
    }
    out
}

fn binomials() -> &'static Vec<Vec<I>> {
    static TABLE: std::sync::OnceLock<Vec<Vec<I>>> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        // Surface degree <=16 in each direction, cubic pcurve, edge degree <=16.
        let mut rows = vec![vec![I::ONE]];
        for n in 1..=112 {
            let mut row = vec![I::ONE; n + 1];
            for i in 1..n {
                row[i] = rows[n - 1][i - 1] + rows[n - 1][i];
            }
            rows.push(row);
        }
        rows
    })
}

/// B_i^degree(u(t)), as Bernstein polynomials on the same t interval.
pub(super) fn basis(u: &[I], degree: usize) -> Vec<Poly> {
    let v = one_minus(u);
    let mut out = vec![vec![I::ONE]];
    for d in 1..=degree {
        let mut next = Vec::with_capacity(d + 1);
        for i in 0..=d {
            let left = if i > 0 {
                product(&out[i - 1], u)
            } else {
                vec![I::ZERO; d * (u.len() - 1) + 1]
            };
            let right = if i < d {
                product(&out[i], &v)
            } else {
                vec![I::ZERO; d * (u.len() - 1) + 1]
            };
            next.push(add(&left, &right));
        }
        out = next;
    }
    out
}

/// De Boor on one nonempty knot span, with t traversing [a,b]. Each coefficient
/// remains an interval, so span extraction includes the rounding of all operations.
pub(super) fn spline_piece(knots: &[f64], ctrl: &[H], span: usize, [a, b]: [f64; 2]) -> Option<Vec<H>> {
    let p = knots.len().checked_sub(ctrl.len() + 1)?;
    if !(1..=16).contains(&p) || span < p || span >= ctrl.len() || a < knots[span] || b > knots[span + 1] || a >= b {
        return None;
    }
    let mut d = ctrl
        .get(span - p..=span)?
        .iter()
        .map(|h| h.map(|x| vec![x]))
        .collect::<Vec<_>>();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = span - p + j;
            let denominator = I::point(knots[i + p + 1 - r]) - I::point(knots[i]);
            let alpha = [a, b].map(|t| (I::point(t) - I::point(knots[i])) / denominator);
            d[j] = std::array::from_fn(|k| add(&product(&d[j - 1][k], &one_minus(&alpha)), &product(&d[j][k], &alpha)));
        }
    }
    let out = (0..=p).map(|i| std::array::from_fn(|k| d[p][k][i])).collect::<Vec<H>>();
    out.iter().flatten().all(|x| x.finite()).then_some(out)
}

/// Split a Bézier polygon at its midpoint, carrying the exact input polynomial's
/// rounding enclosure into both halves. Constant coordinates stay constant.
pub(super) fn split<const N: usize>(points: &[[I; N]]) -> (Vec<[I; N]>, Vec<[I; N]>) {
    let mut work = points.to_vec();
    let mut left = vec![work[0]];
    let mut right = vec![*work.last().expect("nonempty polygon")];
    for len in (1..work.len()).rev() {
        for i in 0..len {
            work[i] = std::array::from_fn(|k| {
                if work[i][k] == work[i + 1][k] {
                    work[i][k]
                } else {
                    (work[i][k] + work[i + 1][k]) * I::point(0.5)
                }
            });
        }
        left.push(work[0]);
        right.push(work[len - 1]);
    }
    right.reverse();
    (left, right)
}

/// Rational Bézier difference, without subtracting independently bounded positions.
/// D = (A*V - B*W)/(W*V). A positive Bernstein denominator makes D a rational
/// Bézier curve; its control-point convex hull bounds the complete 3D deviation.
pub(super) fn deviation(a: &[H], b: &[H]) -> f64 {
    let aw = a.iter().map(|h| h[3]).collect::<Vec<_>>();
    let bw = b.iter().map(|h| h[3]).collect::<Vec<_>>();
    let w = product(&aw, &bw);
    if w.iter().any(|w| !w.finite() || w.lo <= 0.) {
        return f64::INFINITY;
    }
    let numerator = [0, 1, 2].map(|k| {
        let av = a.iter().map(|h| h[k]).collect::<Vec<_>>();
        let bv = b.iter().map(|h| h[k]).collect::<Vec<_>>();
        product(&av, &bw)
            .into_iter()
            .zip(product(&bv, &aw))
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>()
    });
    let mut largest: f64 = 0.;
    for (i, w) in w.iter().enumerate() {
        let mut norm = I::ZERO;
        for n in &numerator {
            let q = I::point(n[i].abs_upper()) / I::point(w.lo);
            norm = norm + q * q;
        }
        let bound = norm.hi.sqrt().next_up();
        if !bound.is_finite() {
            return f64::INFINITY;
        }
        largest = largest.max(bound);
    }
    largest
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounding_encloses_tiny_values_and_refuses_zero_weight() {
        let tiny = I::point(f64::from_bits(1));
        let underflow = tiny * I::point(0.5);
        assert!(underflow.lo <= 0. && underflow.hi >= f64::from_bits(1));
        let p = vec![[I::ZERO, I::ZERO, I::ZERO, I::ONE]; 2];
        let mut bad = p.clone();
        bad[0][3] = I::ZERO;
        assert!(!deviation(&p, &bad).is_finite());
    }

    #[test]
    fn rational_difference_bounds_between_sample_point_excursions() {
        let flat = vec![[I::ZERO, I::ZERO, I::ZERO, I::ONE]; 4];
        let mut wave = flat.clone();
        wave[1][2] = I::point(2.);
        wave[2][2] = I::point(-2.);
        let bound = deviation(&flat, &wave);
        assert!(bound > 0.5 && bound.is_finite());
        for i in 0..=100 {
            let t = f64::from(i) / 100.;
            let z = 6. * t * (1. - t) * (1. - 2. * t);
            assert!(z.abs() <= bound);
        }
        let (a, b) = split(&wave);
        assert!(deviation(&flat, &a) <= bound && deviation(&flat, &b) <= bound);
    }
}
