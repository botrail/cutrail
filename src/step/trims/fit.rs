//! Cubic UV candidates from the kernel's nearest-point evaluator, accepted only by
//! an outward-rounded Bernstein bound on the complete 3D edge/surface discrepancy.

use std::collections::BTreeMap;

use solidcore::eval::{CurveEval, Starts, SurfaceEval};
use solidcore_record::brep::{Curve, Curve2, Surface};

use super::interval::{self as bez, H, I};

const MAX_SPANS: usize = 1024; // 3*spans+1 <= the record's 4096 pcurve control points
const MAX_DEPTH: usize = 40;
const MAX_WORK: usize = 32_768;
const MAX_PATCHES: usize = 256;

type R<T> = Result<T, String>;

pub(super) struct Report {
    pub curve: Curve2,
    pub bound: f64,
    pub spans: usize,
}

struct Spline {
    knots: Vec<f64>,
    ctrl: Vec<H>,
}

impl Spline {
    fn pieces(&self, [a, b]: [f64; 2]) -> R<Vec<(usize, [f64; 2])>> {
        let degree = self
            .knots
            .len()
            .checked_sub(self.ctrl.len() + 1)
            .ok_or("fit: spline counts")?;
        if !(1..=16).contains(&degree) || a >= b || a < self.knots[degree] || b > self.knots[self.ctrl.len()] {
            return Err("fit: spline domain/degree".into());
        }
        let mut out = Vec::new();
        for i in degree..self.ctrl.len() {
            let lo = a.max(self.knots[i]);
            let hi = b.min(self.knots[i + 1]);
            if lo < hi {
                out.push((i, [lo, hi]));
            }
        }
        Ok(out)
    }
}

/// Coordinates are translated to one common origin before any bounding operations.
/// The translation is also interval arithmetic; it is not a rounded replacement shape.
fn homogeneous(points: &[[f64; 4]], origin: [f64; 3]) -> R<Vec<H>> {
    if points.is_empty() || points.iter().flatten().any(|x| !x.is_finite()) || points.iter().any(|h| h[3] <= 0.) {
        return Err("fit requires finite, positive NURBS weights".into());
    }
    let scale = points.iter().map(|h| h[3]).fold(0., f64::max);
    Ok(points
        .iter()
        .map(|h| {
            let w = I::point(h[3]);
            let s = I::point(scale);
            [
                (I::point(h[0]) - I::point(origin[0]) * w) / s,
                (I::point(h[1]) - I::point(origin[1]) * w) / s,
                (I::point(h[2]) - I::point(origin[2]) * w) / s,
                w / s,
            ]
        })
        .collect())
}

struct SurfaceBounds {
    knots: [Vec<f64>; 2],
    net: Vec<Vec<H>>, // u, then v
    degree: [usize; 2],
    origin: [f64; 3],
    patches: BTreeMap<(usize, usize), Vec<Vec<H>>>,
}

impl SurfaceBounds {
    fn new(surface: &Surface) -> R<Self> {
        let (knots, raw) = match surface {
            Surface::Spline {
                u_knots,
                v_knots,
                points,
            } => (
                [u_knots.clone(), v_knots.clone()],
                points
                    .iter()
                    .map(|r| r.iter().map(|p| [p[0], p[1], p[2], 1.]).collect())
                    .collect::<Vec<Vec<_>>>(),
            ),
            Surface::RationalSpline {
                u_knots,
                v_knots,
                homogeneous,
            } => ([u_knots.clone(), v_knots.clone()], homogeneous.clone()),
            _ => return Err("bounded trim fitting requires a spline/NURBS surface".into()),
        };
        let nu = raw.len();
        let nv = raw.first().map_or(0, Vec::len);
        if nu == 0 || nv == 0 || nu.checked_mul(nv).is_none_or(|n| n > 262_144) || raw.iter().any(|r| r.len() != nv) {
            return Err("fit: surface size".into());
        }
        let degree = [knots[0].len().checked_sub(nu + 1), knots[1].len().checked_sub(nv + 1)];
        let [Some(p), Some(q)] = degree else {
            return Err("fit: surface counts".into());
        };
        if !(1..=16).contains(&p)
            || !(1..=16).contains(&q)
            || nu <= p
            || nv <= q
            || knots
                .iter()
                .any(|k| k.len() > 4096 || k.iter().any(|x| !x.is_finite()) || k.windows(2).any(|w| w[0] > w[1]))
        {
            return Err("fit: surface knots/degree".into());
        }
        let first = raw[0][0];
        if first[3] <= 0. {
            return Err("fit: non-positive surface weight".into());
        }
        let origin = [0, 1, 2].map(|i| first[i] / first[3]);
        if origin.iter().any(|x| !x.is_finite()) {
            return Err("fit: surface origin".into());
        }
        let flat = raw.iter().flatten().copied().collect::<Vec<_>>();
        let ctrl = homogeneous(&flat, origin)?;
        let net = ctrl.chunks(nv).map(<[H]>::to_vec).collect();
        Ok(Self {
            knots,
            net,
            degree: [p, q],
            origin,
            patches: BTreeMap::new(),
        })
    }

    fn domain(&self) -> [[f64; 2]; 2] {
        [
            [self.knots[0][self.degree[0]], self.knots[0][self.net.len()]],
            [self.knots[1][self.degree[1]], self.knots[1][self.net[0].len()]],
        ]
    }

    fn spans(&self, d: usize, lo: f64, hi: f64) -> Vec<usize> {
        let count = if d == 0 { self.net.len() } else { self.net[0].len() };
        (self.degree[d]..count)
            .filter(|&i| {
                self.knots[d][i] < self.knots[d][i + 1] && self.knots[d][i] <= hi && self.knots[d][i + 1] >= lo
            })
            .collect()
    }

    fn patch(&mut self, ui: usize, vi: usize) -> R<&Vec<Vec<H>>> {
        if !self.patches.contains_key(&(ui, vi)) {
            if self.patches.len() >= MAX_PATCHES {
                return Err("NURBS trim fit patch budget exceeded".into());
            }
            let ur = [self.knots[0][ui], self.knots[0][ui + 1]];
            let vr = [self.knots[1][vi], self.knots[1][vi + 1]];
            let mut columns = Vec::new();
            for j in vi - self.degree[1]..=vi {
                let c = self.net.iter().map(|r| r[j]).collect::<Vec<_>>();
                columns.push(bez::spline_piece(&self.knots[0], &c, ui, ur).ok_or("fit: u span extraction")?);
            }
            let mut patch = Vec::new();
            for i in 0..=self.degree[0] {
                let mut c = vec![[I::ZERO; 4]; self.net[0].len()];
                for (j, col) in columns.iter().enumerate() {
                    c[vi - self.degree[1] + j] = col[i];
                }
                patch.push(bez::spline_piece(&self.knots[1], &c, vi, vr).ok_or("fit: v span extraction")?);
            }
            self.patches.insert((ui, vi), patch);
        }
        Ok(&self.patches[&(ui, vi)])
    }

    /// Composition of a tensor Bézier patch with a polynomial UV curve. It is a
    /// rational Bézier curve of degree uv_degree*(u_degree+v_degree).
    fn compose(&mut self, ui: usize, vi: usize, uv: &[[I; 2]]) -> R<Vec<H>> {
        let u = uv
            .iter()
            .map(|p| {
                (p[0] - I::point(self.knots[0][ui])) / (I::point(self.knots[0][ui + 1]) - I::point(self.knots[0][ui]))
            })
            .collect::<Vec<_>>();
        let v = uv
            .iter()
            .map(|p| {
                (p[1] - I::point(self.knots[1][vi])) / (I::point(self.knots[1][vi + 1]) - I::point(self.knots[1][vi]))
            })
            .collect::<Vec<_>>();
        let ub = bez::basis(&u, self.degree[0]);
        let vb = bez::basis(&v, self.degree[1]);
        let count = (uv.len() - 1) * (self.degree[0] + self.degree[1]) + 1;
        let patch = self.patch(ui, vi)?;
        let mut out = vec![[I::ZERO; 4]; count];
        for (i, u) in ub.iter().enumerate() {
            for (j, v) in vb.iter().enumerate() {
                for (k, w) in bez::product(u, v).into_iter().enumerate() {
                    for d in 0..4 {
                        out[k][d] = out[k][d] + w * patch[i][j][d];
                    }
                }
            }
        }
        Ok(out)
    }
}

#[derive(Clone, Copy)]
struct Anchor {
    uv: [f64; 2],
    tangent: Option<[f64; 2]>,
}

pub(super) struct Fitter {
    on: SurfaceEval,
    starts: Starts,
    surface: SurfaceBounds,
    work: usize,
}

impl Fitter {
    pub fn new(surface: &Surface) -> R<Self> {
        let value = solidcore_convert::surface(surface).map_err(|e| e.to_string())?;
        let on = SurfaceEval::new(&value).map_err(|e| e.to_string())?;
        Ok(Self {
            on,
            starts: Starts::default(),
            surface: SurfaceBounds::new(surface)?,
            work: 0,
        })
    }

    fn work(&mut self) -> R<()> {
        self.work += 1;
        if self.work > MAX_WORK {
            Err("NURBS trim fit work budget exceeded".into())
        } else {
            Ok(())
        }
    }

    fn anchor(&self, edge: &CurveEval, t: f64, near: Option<[f64; 2]>, tol: f64) -> R<Anchor> {
        let (p, dt) = edge.derivs(t).ok_or("NURBS trim: edge cannot be evaluated")?;
        let uv = self
            .on
            .project_with(&self.starts, p, near, tol * 0.03125)
            .ok_or("NURBS trim: edge is not projectable within its declared tolerance")?;
        let (_, du, dv) = self
            .on
            .derivs(uv[0], uv[1])
            .ok_or("NURBS trim: surface cannot be evaluated")?;
        let (a, b, c) = (du.dot(du), du.dot(dv), dv.dot(dv));
        let det = a * c - b * b;
        let tangent = if det.is_finite() && det > 1e-20 * a * c {
            let (x, y) = (du.dot(dt), dv.dot(dt));
            let q = [(c * x - b * y) / det, (a * y - b * x) / det];
            q.iter().all(|x| x.is_finite()).then_some(q)
        } else if c.is_finite() && c > 0. && c >= a {
            // A patch may collapse in u on its boundary while the v trim is regular.
            // This rank-one tangent is a candidate only; the interval proof still decides.
            Some([0., dv.dot(dt) / c])
        } else if a.is_finite() && a > 0. {
            Some([du.dot(dt) / a, 0.])
        } else {
            None
        };
        let tangent = tangent.filter(|q| q.iter().all(|x| x.is_finite()));
        Ok(Anchor { uv, tangent })
    }

    pub fn fit(&mut self, curve: &Curve, interval: [f64; 2], tolerance: f64) -> R<Report> {
        if !tolerance.is_finite() || tolerance <= 0. {
            return Err("NURBS trim: invalid fitting tolerance".into());
        }
        let value = solidcore_convert::curve(curve).map_err(|e| e.to_string())?;
        let eval = CurveEval::new(&value).map_err(|e| e.to_string())?;
        let (knots, ctrl) = match curve {
            Curve::Line { start, end } => (
                vec![interval[0], interval[0], interval[1], interval[1]],
                [interval[0], interval[1]]
                    .map(|t| {
                        let p = [0, 1, 2].map(|i| {
                            I::point(start[i]) + (I::point(end[i]) - I::point(start[i])) * I::point(t)
                                - I::point(self.surface.origin[i])
                        });
                        [p[0], p[1], p[2], I::ONE]
                    })
                    .to_vec(),
            ),
            Curve::Spline { knots, points } => (
                knots.clone(),
                homogeneous(
                    &points.iter().map(|p| [p[0], p[1], p[2], 1.]).collect::<Vec<_>>(),
                    self.surface.origin,
                )?,
            ),
            Curve::RationalSpline {
                knots,
                homogeneous: points,
            } => (knots.clone(), homogeneous(points, self.surface.origin)?),
            _ => return Err("NURBS trim fitting requires an edge's spline form".into()),
        };
        let source = Spline { knots, ctrl };
        let mut segments = Vec::new();
        let mut bound: f64 = 0.;
        let mut previous = None;
        for (span, range) in source.pieces(interval)? {
            let a = match previous {
                Some(a) => a,
                None => self.anchor(&eval, range[0], None, tolerance)?,
            };
            let b = self.anchor(&eval, range[1], Some(a.uv), tolerance)?;
            bound = bound.max(self.refine(&source, &eval, span, range, [a, b], tolerance, &mut segments, 0)?);
            previous = Some(b);
        }
        if segments.is_empty() {
            return Err("NURBS trim: no fitted spans".into());
        }
        let mut knots = vec![interval[0]; 4];
        let mut points = vec![segments[0].1[0]];
        for (i, (range, pc)) in segments.iter().enumerate() {
            points.extend_from_slice(&pc[1..]);
            if i + 1 < segments.len() {
                knots.extend(std::iter::repeat_n(range[1], 3));
            }
        }
        knots.extend(std::iter::repeat_n(interval[1], 4));
        Ok(Report {
            curve: Curve2::Spline { knots, points },
            bound,
            spans: segments.len(),
        })
    }

    #[allow(clippy::too_many_arguments)] // one recursion carries one edge's source interval and its anchors
    fn refine(
        &mut self,
        source: &Spline,
        eval: &CurveEval,
        span: usize,
        range: [f64; 2],
        ends: [Anchor; 2],
        tolerance: f64,
        segments: &mut Vec<([f64; 2], [[f64; 2]; 4])>,
        depth: usize,
    ) -> R<f64> {
        self.work()?;
        let [a, b] = ends;
        let h = range[1] - range[0];
        let secant = [0, 1].map(|i| (b.uv[i] - a.uv[i]) / h);
        let da = a.tangent.unwrap_or(secant);
        let db = b.tangent.unwrap_or(secant);
        let domain = self.surface.domain();
        let pc = [
            a.uv,
            [0, 1].map(|i| (a.uv[i] + h * da[i] / 3.).clamp(domain[i][0], domain[i][1])),
            [0, 1].map(|i| (b.uv[i] - h * db[i] / 3.).clamp(domain[i][0], domain[i][1])),
            b.uv,
        ];
        let edge = bez::spline_piece(&source.knots, &source.ctrl, span, range)
            .ok_or("NURBS trim: edge span extraction failed")?;
        let uv = pc.map(|p| p.map(I::point));
        let target = tolerance * 0.25;
        // Samples can cheaply reject a poor candidate. Only the interval proof below
        // can accept one; a hidden excursion between these samples cannot pass it.
        let promising = [0.25, 0.5, 0.75].iter().all(|&s| {
            let t = range[0] + h * s;
            let q = [0, 1].map(|i| {
                pc[0][i] * (1. - s).powi(3)
                    + 3. * pc[1][i] * (1. - s).powi(2) * s
                    + 3. * pc[2][i] * (1. - s) * s * s
                    + pc[3][i] * s.powi(3)
            });
            match (self.on.point(q), eval.point(t)) {
                (Some(a), Some(b)) => (a - b).length() <= target * 2.,
                _ => false,
            }
        });
        if promising {
            if let Some(bound) = self.certify(&edge, &uv, target, 0)? {
                if segments.len() >= MAX_SPANS {
                    return Err("NURBS trim fit span budget exceeded".into());
                }
                segments.push((range, pc));
                return Ok(bound);
            }
        }
        let mid = range[0] + h * 0.5;
        if depth >= MAX_DEPTH || mid <= range[0] || mid >= range[1] || segments.len() >= MAX_SPANS {
            return Err(format!("NURBS trim cannot be certified within its tolerance and refinement limits: interval {range:?}, UV {:?} to {:?}, depth {depth}, {} spans",a.uv,b.uv,segments.len()));
        }
        let anchor = self.anchor(eval, mid, Some([0, 1].map(|i| (a.uv[i] + b.uv[i]) * 0.5)), tolerance)?;
        let left = self.refine(
            source,
            eval,
            span,
            [range[0], mid],
            [a, anchor],
            tolerance,
            segments,
            depth + 1,
        )?;
        let right = self.refine(
            source,
            eval,
            span,
            [mid, range[1]],
            [anchor, b],
            tolerance,
            segments,
            depth + 1,
        )?;
        Ok(left.max(right))
    }

    fn certify(&mut self, edge: &[H], uv: &[[I; 2]], target: f64, depth: usize) -> R<Option<f64>> {
        self.work()?;
        let ranges = [0, 1].map(|d| {
            [
                uv.iter().map(|p| p[d].lo).fold(f64::INFINITY, f64::min),
                uv.iter().map(|p| p[d].hi).fold(f64::NEG_INFINITY, f64::max),
            ]
        });
        let domain = self.surface.domain();
        if (0..2).any(|d| ranges[d][0] < domain[d][0] || ranges[d][1] > domain[d][1]) {
            return Ok(None);
        }
        let us = self.surface.spans(0, ranges[0][0], ranges[0][1]);
        let vs = self.surface.spans(1, ranges[1][0], ranges[1][1]);
        let mut largest: f64 = 0.;
        let mut accepted = !us.is_empty() && !vs.is_empty() && us.len() * vs.len() <= 16;
        if accepted {
            for &u in &us {
                for &v in &vs {
                    // Bound each possible knot cell on the full t interval. Its polynomial
                    // extension may overestimate, but is exact wherever UV lies in that cell.
                    let bound = bez::deviation(edge, &self.surface.compose(u, v, uv)?);
                    largest = largest.max(bound);
                    if !bound.is_finite() || bound > target {
                        accepted = false;
                        break;
                    }
                }
                if !accepted {
                    break;
                }
            }
        }
        if accepted {
            return Ok(Some(largest));
        }
        if depth >= 5 {
            return Ok(None);
        }
        let (ea, eb) = bez::split(edge);
        let (ua, ub) = bez::split(uv);
        let Some(a) = self.certify(&ea, &ua, target, depth + 1)? else {
            return Ok(None);
        };
        Ok(self.certify(&eb, &ub, target, depth + 1)?.map(|b| a.max(b)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn paraboloid() -> Surface {
        Surface::Spline {
            u_knots: vec![0., 0., 0., 0., 0., 1., 1., 1., 1., 1.],
            v_knots: vec![0., 0., 0., 1., 1., 1.],
            points: (0..5)
                .map(|i| {
                    (0..3)
                        .map(|j| {
                            [
                                10. * f64::from(i * (i - 1)) / 12.,
                                5. * f64::from(j),
                                2. + if i == 4 { 1. } else { 0. } + if j == 2 { 1. } else { 0. },
                            ]
                        })
                        .collect()
                })
                .collect(),
        }
    }

    fn check_dense(surface: &Surface, source: &Curve, report: &Report) {
        let se = SurfaceEval::new(&solidcore_convert::surface(surface).unwrap()).unwrap();
        let ce = CurveEval::new(&solidcore_convert::curve(source).unwrap()).unwrap();
        let [a, b] = report.curve.interval().unwrap();
        assert!(report.bound <= 2.5e-7, "{}", report.bound);
        assert!(report.spans > 0 && report.spans <= MAX_SPANS);
        for i in 0..=1000 {
            let t = a + (b - a) * f64::from(i) / 1000.;
            let uv = solidcore_ops::pcurve::eval(&report.curve, t).unwrap();
            let gap = (se.point(uv).unwrap() - ce.point(t).unwrap()).length();
            assert!(gap <= report.bound + 1e-11, "t={t}, gap={gap}, bound={}", report.bound);
        }
    }

    #[test]
    fn tensor_composition_bound_includes_a_known_exact_curve() {
        let s = paraboloid();
        let mut surface = SurfaceBounds::new(&s).unwrap();
        let uv = [[I::ZERO, I::point(0.5)], [I::ONE, I::point(0.5)]];
        let controls = (0..5)
            .map(|i| {
                [
                    10. * f64::from(i * (i - 1)) / 12.,
                    5.,
                    2.25 + if i == 4 { 1. } else { 0. },
                    1.,
                ]
            })
            .collect::<Vec<_>>();
        let source = homogeneous(&controls, surface.origin).unwrap();
        let bound = bez::deviation(&source, &surface.compose(4, 2, &uv).unwrap());
        assert!(bound < 1e-10, "{bound}");
    }

    #[test]
    fn a_curved_closed_internal_boundary_has_a_complete_error_bound() {
        let s = paraboloid();
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let xy = [
            [7., 5.],
            [7., 7.],
            [5., 7.],
            [3., 7.],
            [3., 5.],
            [3., 3.],
            [5., 3.],
            [7., 3.],
            [7., 5.],
        ];
        // This spatial conic is on z=2+(x^2+y^2)/100. Its UV image has
        // u=sqrt(x/10), so no rational pcurve is presumed or substituted.
        let c = Curve::RationalSpline {
            knots: vec![0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.],
            homogeneous: xy
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let w = if i % 2 == 0 { 1. } else { w };
                    [p[0] * w, p[1] * w, (1.54 + 0.1 * p[0] + 0.1 * p[1]) * w, w]
                })
                .collect(),
        };
        let report = Fitter::new(&s).unwrap().fit(&c, [0., 1.], 1e-6).unwrap();
        check_dense(&s, &c, &report);
        let a = solidcore_ops::pcurve::eval(&report.curve, 0.).unwrap();
        let b = solidcore_ops::pcurve::eval(&report.curve, 1.).unwrap();
        assert!((a[0] - b[0]).abs() < 1e-8 && (a[1] - b[1]).abs() < 1e-8);
    }

    #[test]
    fn a_reversed_nonunit_trim_crosses_surface_knot_cells() {
        let s = Surface::Spline {
            u_knots: vec![0., 0., 0., 0.5, 1., 1., 1.],
            v_knots: vec![0., 0., 1., 1.],
            points: [0., 1., 7., 10.]
                .into_iter()
                .map(|x| vec![[x, 0., 0.], [x, 10., 0.]])
                .collect(),
        };
        let c = Curve::Spline {
            knots: vec![2., 2., 5., 5.],
            points: vec![[9., 5., 0.], [1., 5., 0.]],
        };
        let report = Fitter::new(&s).unwrap().fit(&c, [2., 5.], 1e-6).unwrap();
        check_dense(&s, &c, &report);
        let a = solidcore_ops::pcurve::eval(&report.curve, 2.).unwrap();
        let b = solidcore_ops::pcurve::eval(&report.curve, 5.).unwrap();
        assert!(a[0] > 0.5 && b[0] < 0.5);
    }

    #[test]
    fn between_sample_excursions_are_rejected_by_the_certificate() {
        let s = Surface::Spline {
            u_knots: vec![0., 0., 1., 1.],
            v_knots: vec![0., 0., 1., 1.],
            points: vec![vec![[0., 0., 0.], [0., 10., 0.]], vec![[10., 0., 0.], [10., 10., 0.]]],
        };
        // z=100*t^2*(1-t)^2*(t-1/4)*(t-1/2)*(t-3/4): the ends,
        // midpoint and both quarter points all lie on the plane.
        let z = [0., 0., -25. / 56., 65. / 56., -65. / 56., 25. / 56., 0., 0.];
        let c = Curve::Spline {
            knots: [vec![0.; 8], vec![1.; 8]].concat(),
            points: z
                .iter()
                .enumerate()
                .map(|(i, &z)| [10. * i as f64 / 7., 5., z])
                .collect(),
        };
        let eval = CurveEval::new(&solidcore_convert::curve(&c).unwrap()).unwrap();
        for t in [0., 0.25, 0.5, 0.75, 1.] {
            assert!(eval.point(t).unwrap().z.abs() < 1e-12);
        }
        assert!(Fitter::new(&s).unwrap().fit(&c, [0., 1.], 1e-6).is_err());
    }
}
