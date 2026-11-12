//! Exact inverse pcurves first, then tolerance-bounded fitting for other NURBS trims.
//!
//! On one side, S(s) = (A wa (1-s) + B wb s) / (wa (1-s) + wb s).
//! Its inverse is s = wa lambda / (wb (1-lambda) + wa lambda), where lambda is
//! distance along AB. Applying that homogeneous linear map to an edge's spline
//! control points preserves its knots, parameterisation and rational weights.
//! No surface is replaced and no sampled projection is used as an exact trim.

use super::cad::{self, Curve, Curve2, Surface};
use std::cell::RefCell;

mod fit;
mod interval;
use solidcore::eval::pcurve::segment;
use solidcore_ops::pcurve::{self, EdgeUse, TrimFace, Trims};

#[derive(Default)]
pub(super) struct KernelTrims {
    fits: RefCell<Vec<FittedFace>>,
}

struct FittedFace {
    surface: Surface,
    uses: Vec<FittedUse>,
}

struct FittedUse {
    source: Curve,
    interval: [f64; 2],
    report: fit::Report,
}

impl KernelTrims {
    pub fn notes_for(&self, brep: &cad::Brep) -> Vec<String> {
        let fits = self.fits.borrow();
        let mut count = 0;
        let mut largest: f64 = 0.;
        let mut spans = 0;
        for face in &brep.faces {
            let surface = face.nurbs.as_ref().unwrap_or(&face.surface);
            for lp in &face.loops {
                if let cad::Loop::Edges { edges } = lp {
                    for use_ in edges {
                        let e = &brep.edges[use_.edge as usize];
                        let source = e.nurbs.as_ref().unwrap_or(&e.curve);
                        if let Some(fit) = fits
                            .iter()
                            .filter(|f| &f.surface == surface)
                            .flat_map(|f| &f.uses)
                            .find(|f| &f.source == source && f.interval == e.interval && f.report.curve == use_.pcurve)
                        {
                            count += 1;
                            largest = largest.max(fit.report.bound);
                            spans += fit.report.spans;
                        }
                    }
                }
            }
        }
        if count == 0 {
            Vec::new()
        } else {
            // Keep the printed upper bound conservative too, instead of rounding it
            // down to a short diagnostic value.
            let bound = largest.next_up();
            vec![format!("{count} fitted NURBS pcurves ({spans} cubic spans); certified 3D deviation bound <= {bound:.17e} mm; source surfaces and 3D edges retained")]
        }
    }
}

impl Trims for KernelTrims {
    fn trims(&self, face: &TrimFace<'_>) -> Result<Vec<Vec<Curve2>>, String> {
        let surface = face.nurbs.unwrap_or(face.surface);
        let mut fitter = None;
        let mut fitted = Vec::new();
        let mut made = Vec::new();
        for (li, lp) in face.loops.iter().enumerate() {
            let mut pcs = Vec::new();
            for (ui, u) in lp.iter().enumerate() {
                let edge = face
                    .edges
                    .get(u.edge as usize)
                    .ok_or("pcurve edge index out of range")?;
                let curve = edge.nurbs.unwrap_or(edge.curve);
                let pc = if let Some(pc) = &u.pcurve {
                    pc.clone()
                } else if let Some(pc) =
                    boundary_pcurve(surface, curve, edge.interval, face.tolerance.min(edge.tolerance))
                {
                    pc
                } else {
                    if fitter.is_none() {
                        fitter = Some(fit::Fitter::new(surface)?);
                    }
                    let report = fitter
                        .as_mut()
                        .expect("initialized fitter")
                        .fit(curve, edge.interval, face.tolerance.min(edge.tolerance))
                        .map_err(|e| format!("NURBS trim loop {li} use {ui}: {e}"))?;
                    let pc = report.curve.clone();
                    fitted.push((
                        li,
                        ui,
                        FittedUse {
                            source: curve.clone(),
                            interval: edge.interval,
                            report,
                        },
                    ));
                    pc
                };
                pcs.push(pc);
            }
            made.push(pcs);
        }
        // SolidCore still owns periodic unwrapping and loop closure. The reader subsequently
        // checks the record and its 3D edge/surface agreement.
        let loops = face
            .loops
            .iter()
            .zip(&made)
            .map(|(lp, pcs)| {
                lp.iter()
                    .zip(pcs)
                    .map(|(u, pc)| {
                        let e = &face.edges[u.edge as usize];
                        EdgeUse {
                            edge: u.edge,
                            curve: e.nurbs.unwrap_or(e.curve),
                            interval: e.interval,
                            forward: u.forward,
                            tolerance: e.tolerance,
                            given: Some(pc),
                        }
                    })
                    .collect()
            })
            .collect::<Vec<Vec<_>>>();
        let closed =
            pcurve::pcurves_within(surface, &loops, face.tolerance, Some(face.domain)).map_err(|e| e.to_string())?;
        if !fitted.is_empty() {
            let uses = fitted
                .into_iter()
                .map(|(li, ui, mut fit)| {
                    fit.report.curve = closed[li][ui].clone();
                    fit
                })
                .collect();
            self.fits.borrow_mut().push(FittedFace {
                surface: surface.clone(),
                uses,
            });
        }
        Ok(closed)
    }
}

fn clamped_linear(knots: &[f64]) -> Option<[f64; 2]> {
    match knots {
        &[a, a1, b, b1] if a == a1 && b == b1 && a < b => Some([a, b]),
        _ => None,
    }
}

/// An edge as a clamped homogeneous spline, on the edge's own interval.
fn edge_spline(curve: &Curve, [a, b]: [f64; 2]) -> Option<(Vec<f64>, Vec<[f64; 4]>)> {
    match curve {
        Curve::Line { start, end } => {
            let at = |t: f64| {
                let p = [0, 1, 2].map(|i| start[i] + t * (end[i] - start[i]));
                [p[0], p[1], p[2], 1.0]
            };
            Some((vec![a, a, b, b], vec![at(a), at(b)]))
        }
        Curve::Spline { knots, points } => {
            let points = points.iter().map(|p| [p[0], p[1], p[2], 1.0]).collect::<Vec<_>>();
            let degree = knots.len().checked_sub(points.len() + 1)?;
            segment(degree, knots, &points, a, b).ok()
        }
        Curve::RationalSpline { knots, homogeneous } => {
            let degree = knots.len().checked_sub(homogeneous.len() + 1)?;
            segment(degree, knots, homogeneous, a, b).ok()
        }
        _ => None,
    }
}

fn boundary_pcurve(surface: &Surface, curve: &Curve, interval: [f64; 2], tolerance: f64) -> Option<Curve2> {
    let Surface::RationalSpline {
        u_knots,
        v_knots,
        homogeneous,
    } = surface
    else {
        return None;
    };
    let ud = clamped_linear(u_knots)?;
    let vd = clamped_linear(v_knots)?;
    let [row0, row1] = homogeneous.as_slice() else {
        return None;
    };
    let (&[p00, p01], &[p10, p11]) = (row0.as_slice(), row1.as_slice()) else {
        return None;
    };
    if [p00, p01, p10, p11]
        .iter()
        .any(|p| !p.iter().all(|x| x.is_finite()) || p[3] <= 0.0)
    {
        return None;
    }
    let (knots, points) = edge_spline(curve, interval)?;
    for (a, b, varying, domain, fixed) in [
        (p00, p01, 1, vd, ud[0]),
        (p10, p11, 1, vd, ud[1]),
        (p00, p10, 0, ud, vd[0]),
        (p01, p11, 0, ud, vd[1]),
    ] {
        let Some(homogeneous) = inverse_side(&points, a, b, varying, domain, fixed, tolerance) else {
            continue;
        };
        return Some(Curve2::RationalSpline { knots, homogeneous });
    }
    None
}

fn inverse_side(
    points: &[[f64; 4]],
    a: [f64; 4],
    b: [f64; 4],
    varying: usize,
    domain: [f64; 2],
    fixed: f64,
    tolerance: f64,
) -> Option<Vec<[f64; 3]>> {
    let origin = [0, 1, 2].map(|i| a[i] / a[3]);
    let end = [0, 1, 2].map(|i| b[i] / b[3]);
    let d = [0, 1, 2].map(|i| end[i] - origin[i]);
    let length2: f64 = d.iter().map(|v| v * v).sum();
    if !length2.is_finite() || length2 <= 0.0 {
        return None;
    }
    // Only rounding-sized collinearity errors are admitted, and never more than a small
    // fraction of the declared geometry tolerance. Convex positive weights bound the whole
    // spline by its control points; endpoint/midpoint samples would not do that.
    let size = origin
        .iter()
        .chain(&end)
        .fold(length2.sqrt().max(1.0), |s, x| s.max(x.abs()));
    let slack = (128.0 * f64::EPSILON * size).min(tolerance * 0.125);
    if !slack.is_finite() || slack < 0.0 {
        return None;
    }
    let mut out = Vec::with_capacity(points.len());
    for p in points {
        if !p.iter().all(|x| x.is_finite()) || p[3] <= 0.0 {
            return None;
        }
        let offset = [0, 1, 2].map(|i| p[i] / p[3] - origin[i]);
        let lambda = offset.iter().zip(d).map(|(x, y)| x * y).sum::<f64>() / length2;
        let residual2 = offset.iter().zip(d).map(|(x, y)| (x - lambda * y).powi(2)).sum::<f64>();
        if !residual2.is_finite()
            || residual2 > slack * slack
            || lambda < -slack / length2.sqrt()
            || lambda > 1.0 + slack / length2.sqrt()
        {
            return None;
        }
        let lambda = lambda.clamp(0.0, 1.0);
        let weight = b[3] * (1.0 - lambda) + a[3] * lambda;
        let s = domain[0] * weight + (domain[1] - domain[0]) * a[3] * lambda;
        let mut q = [fixed * weight, fixed * weight, weight];
        q[varying] = s;
        // Preserve the original edge's parameterisation, including its rational weights.
        q.iter_mut().for_each(|x| *x *= p[3]);
        if !q.iter().all(|x| x.is_finite()) || q[2] <= 0.0 {
            return None;
        }
        out.push(q);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch() -> Surface {
        Surface::RationalSpline {
            u_knots: vec![0., 0., 1., 1.],
            v_knots: vec![0., 0., 1., 1.],
            // The far corner is lifted: this is not a plane that could be substituted.
            homogeneous: vec![
                vec![[0., 0., 0., 1.], [0., 20., 0., 2.]],
                vec![[10., 0., 0., 1.], [20., 20., 4., 2.]],
            ],
        }
    }

    #[test]
    fn rational_spline_edge_is_inverted_on_its_trimmed_nonunit_interval() {
        let edge = Curve::RationalSpline {
            knots: vec![1., 1., 1., 7., 7., 7.],
            homogeneous: vec![[10., 0., 0., 1.], [5., 2.5, 0.5, 0.5], [20., 20., 4., 2.]],
        };
        let pc = boundary_pcurve(&patch(), &edge, [2., 5.], 1e-6).unwrap();
        assert_eq!(pc.interval(), Some([2., 5.]));
        for i in 0..=100 {
            let t = 2. + 3. * f64::from(i) / 100.;
            let s = (t - 1.) / 6.;
            let w = (1. - s).powi(2) + (1. - s) * s + 2. * s * s;
            let expected_y = (5. * (1. - s) * s + 20. * s * s) / w;
            let expected_z = ((1. - s) * s + 4. * s * s) / w;
            let uv = pcurve::eval(&pc, t).unwrap();
            assert!((uv[0] - 1.).abs() < 1e-12);
            assert!((20. * uv[1] / (1. + uv[1]) - expected_y).abs() < 1e-11);
            assert!((4. * uv[1] / (1. + uv[1]) - expected_z).abs() < 1e-11);
        }
    }

    #[test]
    fn an_edge_that_leaves_the_surface_between_end_and_midpoint_checks_is_refused() {
        let edge = Curve::Spline {
            knots: vec![0., 0., 0., 0., 1., 1., 1., 1.],
            points: vec![[0., 0., 0.], [0., 3., 1.], [0., 7., -1.], [0., 10., 0.]],
        };
        assert!(boundary_pcurve(&patch(), &edge, [0., 1.], 1e-6).is_none());
    }

    #[test]
    fn surface_extensions_and_non_bilinear_patches_are_refused() {
        let edge = Curve::Line {
            start: [0., -2., 0.],
            end: [0., 12., 0.],
        };
        assert!(boundary_pcurve(&patch(), &edge, [0., 1.], 1e-6).is_none());
        let mut surface = patch();
        if let Surface::RationalSpline { u_knots, .. } = &mut surface {
            u_knots.push(1.);
        }
        let edge = Curve::Line {
            start: [0., 0., 0.],
            end: [0., 10., 0.],
        };
        assert!(boundary_pcurve(&surface, &edge, [0., 1.], 1e-6).is_none());
    }
}
