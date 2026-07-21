//! A 3D B-Spline/NURBS surface (OCCT `Geom_BSplineSurface`).

use openrcad_foundation::{BndBox, Interval, Interval3, Pnt, Trsf, Vec as GeomVec};
use serde::{Deserialize, Serialize};

use crate::bspline_derivatives::{
    evaluate_active_curve, find_span, homogenize, project_surface, HomogeneousSurfaceDerivatives,
};
use crate::surface::Surface;

/// A 3D B-Spline/NURBS surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BSplineSurface {
    u_degree: usize,
    v_degree: usize,
    poles: Vec<Vec<Pnt>>, // Grid: u_len x v_len
    weights: Option<Vec<Vec<f64>>>,
    u_knots: Vec<f64>,
    u_mults: Vec<usize>,
    u_flat_knots: Vec<f64>,
    v_knots: Vec<f64>,
    v_mults: Vec<usize>,
    v_flat_knots: Vec<f64>,
}

/// Point and first/second partial derivatives of a B-spline/NURBS surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BSplineSurfaceDerivatives {
    /// Evaluated spatial point.
    pub point: Pnt,
    /// First partial derivative in U.
    pub du: GeomVec,
    /// First partial derivative in V.
    pub dv: GeomVec,
    /// Second partial derivative in U.
    pub duu: GeomVec,
    /// Mixed second partial derivative.
    pub duv: GeomVec,
    /// Second partial derivative in V.
    pub dvv: GeomVec,
}

impl BSplineSurface {
    /// Create a new B-Spline surface.
    ///
    /// # Panics
    /// Panics if input sizes are inconsistent.
    #[allow(clippy::too_many_arguments)] // a NURBS surface is defined by exactly these fields
    pub fn new(
        u_degree: usize,
        v_degree: usize,
        poles: Vec<Vec<Pnt>>,
        weights: Option<Vec<Vec<f64>>>,
        u_knots: Vec<f64>,
        u_mults: Vec<usize>,
        v_knots: Vec<f64>,
        v_mults: Vec<usize>,
    ) -> Self {
        assert!(u_degree >= 1, "u_degree must be >= 1");
        assert!(v_degree >= 1, "v_degree must be >= 1");
        let u_len = poles.len();
        assert!(u_len > 0, "poles must not be empty in U");
        let v_len = poles[0].len();
        for row in &poles {
            assert_eq!(
                row.len(),
                v_len,
                "all rows in poles grid must have same length"
            );
        }

        if let Some(ref w) = weights {
            assert_eq!(w.len(), u_len, "weights grid U length must match poles");
            for row in w {
                assert_eq!(row.len(), v_len, "weights grid V length must match poles");
                for &weight in row {
                    assert!(weight > 0.0, "weights must be positive");
                }
            }
        }

        // Flat knots construction
        let mut u_flat_knots = Vec::new();
        for (&k, &m) in u_knots.iter().zip(u_mults.iter()) {
            for _ in 0..m {
                u_flat_knots.push(k);
            }
        }
        assert_eq!(
            u_flat_knots.len(),
            u_len + u_degree + 1,
            "sum of U multiplicities must equal u_len + u_degree + 1"
        );

        let mut v_flat_knots = Vec::new();
        for (&k, &m) in v_knots.iter().zip(v_mults.iter()) {
            for _ in 0..m {
                v_flat_knots.push(k);
            }
        }
        assert_eq!(
            v_flat_knots.len(),
            v_len + v_degree + 1,
            "sum of V multiplicities must equal v_len + v_degree + 1"
        );

        Self {
            u_degree,
            v_degree,
            poles,
            weights,
            u_knots,
            u_mults,
            u_flat_knots,
            v_knots,
            v_mults,
            v_flat_knots,
        }
    }

    /// Get the U degree.
    #[inline]
    pub const fn u_degree(&self) -> usize {
        self.u_degree
    }

    /// Get the V degree.
    #[inline]
    pub const fn v_degree(&self) -> usize {
        self.v_degree
    }

    /// Get the grid of poles (control points).
    #[inline]
    pub fn poles(&self) -> &[Vec<Pnt>] {
        &self.poles
    }

    /// Get the weights of the control points (if rational).
    #[inline]
    pub fn weights(&self) -> Option<&[Vec<f64>]> {
        self.weights.as_deref()
    }

    /// Get the distinct U knots.
    #[inline]
    pub fn u_knots(&self) -> &[f64] {
        &self.u_knots
    }

    /// Get the multiplicities of the distinct U knots.
    #[inline]
    pub fn u_multiplicities(&self) -> &[usize] {
        &self.u_mults
    }

    /// Get the distinct V knots.
    #[inline]
    pub fn v_knots(&self) -> &[f64] {
        &self.v_knots
    }

    /// Get the multiplicities of the distinct V knots.
    #[inline]
    pub fn v_multiplicities(&self) -> &[usize] {
        &self.v_mults
    }

    /// A guaranteed-enclosing axis-aligned box of the surface over the parameter
    /// rectangle `[u0, u1] × [v0, v1]`, from the convex-hull property of the
    /// active control poles (valid for rational surfaces — see
    /// [`BSplineCurve::interval_bbox`](crate::BSplineCurve::interval_bbox)).
    pub fn interval_bbox(&self, u0: f64, u1: f64, v0: f64, v1: f64) -> Interval3 {
        let (u0, u1) = if u0 <= u1 { (u0, u1) } else { (u1, u0) };
        let (v0, v1) = if v0 <= v1 { (v0, v1) } else { (v1, v0) };
        let ud = self.u_degree;
        let vd = self.v_degree;
        let ut = &self.u_flat_knots;
        let vt = &self.v_flat_knots;
        let mut b = BndBox::new();
        for i in 0..self.poles.len() {
            if !(ut[i] <= u1 && ut[i + ud + 1] >= u0) {
                continue;
            }
            for j in 0..self.poles[i].len() {
                if vt[j] <= v1 && vt[j + vd + 1] >= v0 {
                    b.add(&self.poles[i][j]);
                }
            }
        }
        match (b.corner_min(), b.corner_max()) {
            (Some(lo), Some(hi)) => Interval3::new(
                Interval::new(lo.x(), hi.x()),
                Interval::new(lo.y(), hi.y()),
                Interval::new(lo.z(), hi.z()),
            ),
            _ => Interval3::new(Interval::whole(), Interval::whole(), Interval::whole()),
        }
    }

    /// Evaluates the surface normal at $(u, v)$.
    pub fn normal(&self, u: f64, v: f64) -> GeomVec {
        let (_, su, sv) = self.d1(u, v);
        su.cross(&sv)
            .normalized()
            .map(GeomVec::from_dir)
            .unwrap_or(GeomVec::DZ)
    }

    /// Evaluates both the point and the first partial derivatives at $(u, v)$.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec) {
        let derivatives = self.evaluate_derivatives(u, v, 1);
        (derivatives.point, derivatives.du, derivatives.dv)
    }

    /// Evaluate the point and all first/second partial derivatives in one pass.
    pub fn derivatives(&self, u: f64, v: f64) -> BSplineSurfaceDerivatives {
        self.evaluate_derivatives(u, v, 2)
    }

    /// Evaluate the point and all first/second partial derivatives.
    ///
    /// The tuple order is `(point, du, dv, duu, duv, dvv)`. This mirrors a
    /// conventional CAD-kernel `D2` operation without widening [`Surface`].
    #[allow(clippy::type_complexity)]
    pub fn d2(&self, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec, GeomVec, GeomVec, GeomVec) {
        let derivatives = self.derivatives(u, v);
        (
            derivatives.point,
            derivatives.du,
            derivatives.dv,
            derivatives.duu,
            derivatives.duv,
            derivatives.dvv,
        )
    }

    fn evaluate_derivatives(&self, u: f64, v: f64, max_order: usize) -> BSplineSurfaceDerivatives {
        let max_order = max_order.min(2);
        let u_span = find_span(self.u_degree, &self.u_flat_knots, self.poles.len(), u);
        let v_span = find_span(self.v_degree, &self.v_flat_knots, self.poles[0].len(), v);
        let first_u = u_span - self.u_degree;
        let first_v = v_span - self.v_degree;
        let origin = self.poles[first_u][first_v];

        let mut values_in_v = Vec::with_capacity(self.u_degree + 1);
        let mut first_in_v = Vec::with_capacity(self.u_degree + 1);
        let mut second_in_v = Vec::with_capacity(self.u_degree + 1);
        for local_u in 0..=self.u_degree {
            let u_index = first_u + local_u;
            let mut active_v = Vec::with_capacity(self.v_degree + 1);
            for local_v in 0..=self.v_degree {
                let v_index = first_v + local_v;
                let weight = self
                    .weights
                    .as_ref()
                    .map(|weights| weights[u_index][v_index])
                    .unwrap_or(1.0);
                active_v.push(homogenize(self.poles[u_index][v_index], weight, origin));
            }
            let derivatives_in_v = evaluate_active_curve(
                self.v_degree,
                &self.v_flat_knots,
                v_span,
                &active_v,
                v,
                max_order,
            );
            values_in_v.push(derivatives_in_v[0]);
            first_in_v.push(derivatives_in_v[1]);
            second_in_v.push(derivatives_in_v[2]);
        }

        let values_in_u = evaluate_active_curve(
            self.u_degree,
            &self.u_flat_knots,
            u_span,
            &values_in_v,
            u,
            max_order,
        );
        let mut homogeneous = HomogeneousSurfaceDerivatives {
            value: values_in_u[0],
            du: values_in_u[1],
            duu: values_in_u[2],
            ..HomogeneousSurfaceDerivatives::default()
        };

        if max_order >= 1 {
            let first_v_in_u = evaluate_active_curve(
                self.u_degree,
                &self.u_flat_knots,
                u_span,
                &first_in_v,
                u,
                max_order - 1,
            );
            homogeneous.dv = first_v_in_u[0];
            homogeneous.duv = first_v_in_u[1];
        }
        if max_order >= 2 {
            homogeneous.dvv = evaluate_active_curve(
                self.u_degree,
                &self.u_flat_knots,
                u_span,
                &second_in_v,
                u,
                0,
            )[0];
        }

        let (point, du, dv, duu, duv, dvv) = project_surface(origin, homogeneous);
        BSplineSurfaceDerivatives {
            point,
            du,
            dv,
            duu,
            duv,
            dvv,
        }
    }
}

impl Surface for BSplineSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        self.evaluate_derivatives(u, v, 0).point
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (
            self.u_flat_knots[self.u_degree],
            self.u_flat_knots[self.poles.len()],
            self.v_flat_knots[self.v_degree],
            self.v_flat_knots[self.poles[0].len()],
        )
    }

    fn transformed(&self, t: &Trsf) -> Self {
        let new_poles: Vec<Vec<Pnt>> = self
            .poles
            .iter()
            .map(|row| {
                row.iter()
                    .map(|p| t.transform_point(p))
                    .collect::<Vec<Pnt>>()
            })
            .collect::<Vec<Vec<Pnt>>>();
        Self::new(
            self.u_degree,
            self.v_degree,
            new_poles,
            self.weights.clone(),
            self.u_knots.clone(),
            self.u_mults.clone(),
            self.v_knots.clone(),
            self.v_mults.clone(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quarter_cylinder(origin: Pnt, radius: f64, height: f64) -> BSplineSurface {
        let quarter_weight = std::f64::consts::FRAC_1_SQRT_2;
        BSplineSurface::new(
            2,
            1,
            vec![
                vec![
                    Pnt::new(origin.x() + radius, origin.y(), origin.z()),
                    Pnt::new(origin.x() + radius, origin.y(), origin.z() + height),
                ],
                vec![
                    Pnt::new(origin.x() + radius, origin.y() + radius, origin.z()),
                    Pnt::new(
                        origin.x() + radius,
                        origin.y() + radius,
                        origin.z() + height,
                    ),
                ],
                vec![
                    Pnt::new(origin.x(), origin.y() + radius, origin.z()),
                    Pnt::new(origin.x(), origin.y() + radius, origin.z() + height),
                ],
            ],
            Some(vec![
                vec![1.0, 1.0],
                vec![quarter_weight, quarter_weight],
                vec![1.0, 1.0],
            ]),
            vec![0.0, 1.0],
            vec![3, 3],
            vec![0.0, 1.0],
            vec![2, 2],
        )
    }

    fn assert_vector_near(actual: GeomVec, expected: GeomVec, tolerance: f64) {
        for (label, actual, expected) in [
            ("x", actual.x(), expected.x()),
            ("y", actual.y(), expected.y()),
            ("z", actual.z(), expected.z()),
        ] {
            assert!(
                (actual - expected).abs() <= tolerance,
                "{label}: actual={actual} expected={expected} tolerance={tolerance}"
            );
        }
    }

    fn finite_difference_vector(positive: Pnt, negative: Pnt, denominator: f64) -> GeomVec {
        GeomVec::new(
            (positive.x() - negative.x()) / denominator,
            (positive.y() - negative.y()) / denominator,
            (positive.z() - negative.z()) / denominator,
        )
    }

    #[test]
    fn planar_bspline_surface_eval() {
        // Flat 2x2 grid of poles on Z = 5 plane
        let poles = vec![
            vec![Pnt::new(0.0, 0.0, 5.0), Pnt::new(0.0, 2.0, 5.0)],
            vec![Pnt::new(2.0, 0.0, 5.0), Pnt::new(2.0, 2.0, 5.0)],
        ];
        let u_knots = vec![0.0, 2.0];
        let u_mults = vec![2, 2];
        let v_knots = vec![0.0, 2.0];
        let v_mults = vec![2, 2];

        let surf = BSplineSurface::new(1, 1, poles, None, u_knots, u_mults, v_knots, v_mults);

        let (p, su, sv) = surf.d1(1.0, 1.0);
        // Center of the flat bilinear surface patch
        assert_eq!(p, Pnt::new(1.0, 1.0, 5.0));

        // su = (1, 0, 0), sv = (0, 1, 0)
        assert!((su.x() - 1.0).abs() < 1e-12);
        assert!((su.y() - 0.0).abs() < 1e-12);
        assert!((su.z() - 0.0).abs() < 1e-12);

        assert!((sv.x() - 0.0).abs() < 1e-12);
        assert!((sv.y() - 1.0).abs() < 1e-12);
        assert!((sv.z() - 0.0).abs() < 1e-12);

        let n = surf.normal(1.0, 1.0);
        assert_eq!(n, GeomVec::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn rational_cylinder_partials_match_conic_reference() {
        let surface = quarter_cylinder(Pnt::ORIGIN, 1.0, 2.0);
        let derivatives = surface.derivatives(0.5, 0.25);
        let radial = std::f64::consts::FRAC_1_SQRT_2;

        assert!(derivatives
            .point
            .is_equal(&Pnt::new(radial, radial, 0.5), 1.0e-14));
        assert_vector_near(
            derivatives.du,
            GeomVec::new(-1.171_572_875_253_81, 1.171_572_875_253_81, 0.0),
            1.0e-13,
        );
        assert_vector_near(derivatives.dv, GeomVec::new(0.0, 0.0, 2.0), 1.0e-14);
        assert_vector_near(
            derivatives.duu,
            GeomVec::new(-1.941_125_496_954_28, -1.941_125_496_954_28, 0.0),
            1.0e-12,
        );
        assert_vector_near(derivatives.duv, GeomVec::ZERO, 1.0e-13);
        assert_vector_near(derivatives.dvv, GeomVec::ZERO, 1.0e-13);
    }

    #[test]
    fn surface_second_partials_agree_with_finite_difference() {
        let surface = BSplineSurface::new(
            2,
            2,
            vec![
                vec![
                    Pnt::new(0.0, 0.0, 0.0),
                    Pnt::new(0.0, 1.0, 0.5),
                    Pnt::new(0.0, 2.0, -0.25),
                ],
                vec![
                    Pnt::new(1.0, 0.0, 0.75),
                    Pnt::new(1.0, 1.0, 2.0),
                    Pnt::new(1.0, 2.0, 0.5),
                ],
                vec![
                    Pnt::new(2.0, 0.0, -0.5),
                    Pnt::new(2.0, 1.0, 0.25),
                    Pnt::new(2.0, 2.0, 1.0),
                ],
            ],
            Some(vec![
                vec![1.0, 0.8, 1.2],
                vec![1.3, 0.7, 1.1],
                vec![0.9, 1.4, 1.0],
            ]),
            vec![0.0, 1.0],
            vec![3, 3],
            vec![0.0, 1.0],
            vec![3, 3],
        );
        let u = 0.37;
        let v = 0.41;
        let step: f64 = 1.0e-5;
        let derivatives = surface.derivatives(u, v);
        let center = surface.point(u, v);
        let u_before = surface.point(u - step, v);
        let u_after = surface.point(u + step, v);
        let v_before = surface.point(u, v - step);
        let v_after = surface.point(u, v + step);

        assert_vector_near(
            derivatives.du,
            finite_difference_vector(u_after, u_before, 2.0 * step),
            2.0e-9,
        );
        assert_vector_near(
            derivatives.dv,
            finite_difference_vector(v_after, v_before, 2.0 * step),
            2.0e-9,
        );
        assert_vector_near(
            derivatives.duu,
            GeomVec::new(
                (u_after.x() - 2.0 * center.x() + u_before.x()) / step.powi(2),
                (u_after.y() - 2.0 * center.y() + u_before.y()) / step.powi(2),
                (u_after.z() - 2.0 * center.z() + u_before.z()) / step.powi(2),
            ),
            3.0e-5,
        );
        assert_vector_near(
            derivatives.dvv,
            GeomVec::new(
                (v_after.x() - 2.0 * center.x() + v_before.x()) / step.powi(2),
                (v_after.y() - 2.0 * center.y() + v_before.y()) / step.powi(2),
                (v_after.z() - 2.0 * center.z() + v_before.z()) / step.powi(2),
            ),
            3.0e-5,
        );

        let plus_plus = surface.point(u + step, v + step);
        let plus_minus = surface.point(u + step, v - step);
        let minus_plus = surface.point(u - step, v + step);
        let minus_minus = surface.point(u - step, v - step);
        let mixed_denominator = 4.0 * step.powi(2);
        assert_vector_near(
            derivatives.duv,
            GeomVec::new(
                (plus_plus.x() - plus_minus.x() - minus_plus.x() + minus_minus.x())
                    / mixed_denominator,
                (plus_plus.y() - plus_minus.y() - minus_plus.y() + minus_minus.y())
                    / mixed_denominator,
                (plus_plus.z() - plus_minus.z() - minus_plus.z() + minus_minus.z())
                    / mixed_denominator,
            ),
            3.0e-5,
        );
    }

    #[test]
    fn surface_derivatives_hold_at_endpoints_across_scale_and_far_origin() {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for origin in [Pnt::ORIGIN, Pnt::new(1.0e9, -1.0e9, 5.0e8)] {
                let surface = quarter_cylinder(origin, scale, 2.0 * scale);
                let input_resolution = f64::EPSILON
                    * origin
                        .x()
                        .abs()
                        .max(origin.y().abs())
                        .max(origin.z().abs())
                        .max(1.0);
                let tolerance = (scale * 2.0e-12).max(input_resolution * 4.0);
                for (u, v) in [(0.0, 0.0), (0.5, 0.25), (1.0, 1.0)] {
                    let derivatives = surface.derivatives(u, v);
                    for value in [
                        derivatives.point.x(),
                        derivatives.point.y(),
                        derivatives.point.z(),
                        derivatives.du.x(),
                        derivatives.du.y(),
                        derivatives.du.z(),
                        derivatives.dv.x(),
                        derivatives.dv.y(),
                        derivatives.dv.z(),
                        derivatives.duu.x(),
                        derivatives.duu.y(),
                        derivatives.duu.z(),
                        derivatives.duv.x(),
                        derivatives.duv.y(),
                        derivatives.duv.z(),
                        derivatives.dvv.x(),
                        derivatives.dvv.y(),
                        derivatives.dvv.z(),
                    ] {
                        assert!(value.is_finite(), "non-finite partial at ({u}, {v})");
                    }
                    assert_vector_near(
                        derivatives.dv,
                        GeomVec::new(0.0, 0.0, 2.0 * scale),
                        tolerance,
                    );
                }
            }
        }
    }
}
