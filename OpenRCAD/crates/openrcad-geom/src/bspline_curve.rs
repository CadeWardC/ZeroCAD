//! A 3D B-Spline/NURBS curve (OCCT `Geom_BSplineCurve`).

use openrcad_foundation::{BndBox, Interval, Interval3, Pnt, Trsf, Vec as GeomVec};
use serde::{Deserialize, Serialize};

use crate::bspline_derivatives::{evaluate_active_curve, find_span, homogenize, project_curve};
use crate::curve::Curve;

/// A 3D B-Spline/NURBS curve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BSplineCurve {
    degree: usize,
    poles: Vec<Pnt>,
    weights: Option<Vec<f64>>,
    knots: Vec<f64>,
    mults: Vec<usize>,
    flat_knots: Vec<f64>,
}

/// Point and first two derivatives of a B-spline/NURBS curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BSplineCurveDerivatives {
    /// Evaluated spatial point.
    pub point: Pnt,
    /// First derivative with respect to the curve parameter.
    pub d1: GeomVec,
    /// Second derivative with respect to the curve parameter.
    pub d2: GeomVec,
}

impl BSplineCurve {
    /// Create a B-Spline curve.
    ///
    /// # Panics
    /// Panics if input sizes are inconsistent:
    /// - `degree` must be >= 1.
    /// - `poles` length must match `weights` (if present).
    /// - `knots` and `mults` must have the same length.
    /// - The sum of multiplicities must equal `poles.len() + degree + 1`.
    pub fn new(
        degree: usize,
        poles: Vec<Pnt>,
        weights: Option<Vec<f64>>,
        knots: Vec<f64>,
        mults: Vec<usize>,
    ) -> Self {
        assert!(degree >= 1, "degree must be at least 1");
        assert!(!poles.is_empty(), "poles must not be empty");
        if let Some(ref w) = weights {
            assert_eq!(poles.len(), w.len(), "weights length must match poles");
            for &weight in w {
                assert!(weight > 0.0, "weights must be positive");
            }
        }
        assert_eq!(
            knots.len(),
            mults.len(),
            "knots and mults lengths must match"
        );

        // Construct flat knot vector
        let mut flat_knots = Vec::new();
        for (&k, &m) in knots.iter().zip(mults.iter()) {
            for _ in 0..m {
                flat_knots.push(k);
            }
        }

        assert_eq!(
            flat_knots.len(),
            poles.len() + degree + 1,
            "sum of multiplicities must equal poles.len() + degree + 1"
        );

        Self {
            degree,
            poles,
            weights,
            knots,
            mults,
            flat_knots,
        }
    }

    /// The degree of the B-spline.
    #[inline]
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// The control points (poles).
    #[inline]
    pub fn poles(&self) -> &[Pnt] {
        &self.poles
    }

    /// The weights of the control points (if rational).
    #[inline]
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    /// The distinct knots.
    #[inline]
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// The multiplicities of the distinct knots.
    #[inline]
    pub fn multiplicities(&self) -> &[usize] {
        &self.mults
    }

    /// A guaranteed-enclosing axis-aligned box of the curve over `[t0, t1]`,
    /// from the convex-hull property of the active control poles. Valid for
    /// rational (NURBS) curves too: with positive weights the evaluated point is
    /// a convex combination of the poles, so it lies in their convex hull.
    pub fn interval_bbox(&self, t0: f64, t1: f64) -> Interval3 {
        let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let d = self.degree;
        let t = &self.flat_knots;
        let mut b = BndBox::new();
        for i in 0..self.poles.len() {
            // Pole `i` has support `[t[i], t[i + d + 1]]`. Include it when that
            // touches the query interval; over-inclusion at exact knots is safe
            // (it only widens the bound, never under-estimates).
            if t[i] <= t1 && t[i + d + 1] >= t0 {
                b.add(&self.poles[i]);
            }
        }
        match (b.corner_min(), b.corner_max()) {
            (Some(lo), Some(hi)) => Interval3::new(
                Interval::new(lo.x(), hi.x()),
                Interval::new(lo.y(), hi.y()),
                Interval::new(lo.z(), hi.z()),
            ),
            // No active poles (degenerate query): return the whole box so the
            // caller never prunes on an empty bound.
            _ => Interval3::new(Interval::whole(), Interval::whole(), Interval::whole()),
        }
    }

    /// Find the knot span index $i$ such that $flat\_knots[i] \leq u < flat\_knots[i+1]$.
    fn find_span(&self, u: f64) -> usize {
        find_span(self.degree, &self.flat_knots, self.poles.len(), u)
    }

    /// Insert a knot `x` with multiplicity `num` times using Boehm's algorithm.
    pub fn insert_knot(&mut self, x: f64, num: usize) {
        if num == 0 {
            return;
        }
        let t = &self.flat_knots;
        let degree = self.degree;
        let n = self.poles.len();

        // Check range
        if x < t[degree] || x > t[n] {
            return;
        }

        for _ in 0..num {
            let span = self.find_span(x);
            let t = &self.flat_knots;

            // Build homogeneous control points
            let mut h_poles = Vec::with_capacity(n);
            for i in 0..n {
                let p = self.poles[i];
                let w = self.weights.as_ref().map(|w| w[i]).unwrap_or(1.0);
                h_poles.push([p.x() * w, p.y() * w, p.z() * w, w]);
            }

            // Create new control points
            let mut new_h_poles = Vec::with_capacity(n + 1);

            // 1. Poles before the affected span
            for i in 0..=(span - degree) {
                new_h_poles.push(h_poles[i]);
            }

            // 2. Affected poles (interpolated)
            for i in (span - degree + 1)..=span {
                let denom = t[i + degree] - t[i];
                let alpha = if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (x - t[i]) / denom
                };
                let p_prev = h_poles[i - 1];
                let p_curr = h_poles[i];
                let mut p_new = [0.0; 4];
                for coord in 0..4 {
                    p_new[coord] = (1.0 - alpha) * p_prev[coord] + alpha * p_curr[coord];
                }
                new_h_poles.push(p_new);
            }

            // 3. Poles after the affected span
            for i in span..n {
                new_h_poles.push(h_poles[i]);
            }

            // Project back to 3D and separate weights
            let mut new_poles = Vec::with_capacity(n + 1);
            let mut new_weights = self.weights.as_ref().map(|_| Vec::with_capacity(n + 1));

            for hp in new_h_poles {
                let w = hp[3];
                new_poles.push(Pnt::new(hp[0] / w, hp[1] / w, hp[2] / w));
                if let Some(ref mut nw) = new_weights {
                    nw.push(w);
                }
            }

            // Insert knot into flat_knots
            let mut new_flat_knots = self.flat_knots.clone();
            new_flat_knots.insert(span + 1, x);

            // Update state
            self.poles = new_poles;
            self.weights = new_weights;
            self.flat_knots = new_flat_knots;
        }

        // Reconstruct distinct knots and multiplicities
        let mut distinct_knots = Vec::new();
        let mut distinct_mults = Vec::new();
        if !self.flat_knots.is_empty() {
            let mut last_k = self.flat_knots[0];
            let mut count = 1;
            for &k in self.flat_knots.iter().skip(1) {
                if (k - last_k).abs() < 1e-12 {
                    count += 1;
                } else {
                    distinct_knots.push(last_k);
                    distinct_mults.push(count);
                    last_k = k;
                    count = 1;
                }
            }
            distinct_knots.push(last_k);
            distinct_mults.push(count);
        }
        self.knots = distinct_knots;
        self.mults = distinct_mults;
    }

    /// Evaluate the point and first two parameter derivatives in one pass.
    pub fn derivatives(&self, u: f64) -> BSplineCurveDerivatives {
        self.evaluate_derivatives(u, 2)
    }

    /// Evaluate the point, first derivative, and second derivative.
    ///
    /// This mirrors the conventional CAD-kernel `D2` operation without adding
    /// a second-derivative requirement to the common [`Curve`] trait.
    pub fn d2(&self, u: f64) -> (Pnt, GeomVec, GeomVec) {
        let derivatives = self.derivatives(u);
        (derivatives.point, derivatives.d1, derivatives.d2)
    }

    fn evaluate_derivatives(&self, u: f64, max_order: usize) -> BSplineCurveDerivatives {
        let span = self.find_span(u);
        let first_control = span - self.degree;
        let origin = self.poles[first_control];
        let mut active = Vec::with_capacity(self.degree + 1);
        for local_index in 0..=self.degree {
            let control_index = first_control + local_index;
            let weight = self
                .weights
                .as_ref()
                .map(|weights| weights[control_index])
                .unwrap_or(1.0);
            active.push(homogenize(self.poles[control_index], weight, origin));
        }
        let homogeneous =
            evaluate_active_curve(self.degree, &self.flat_knots, span, &active, u, max_order);
        let (point, d1, d2) = project_curve(origin, homogeneous);
        BSplineCurveDerivatives { point, d1, d2 }
    }
}

impl Curve for BSplineCurve {
    fn point(&self, u: f64) -> Pnt {
        self.evaluate_derivatives(u, 0).point
    }

    fn d1(&self, u: f64) -> (Pnt, GeomVec) {
        let derivatives = self.evaluate_derivatives(u, 1);
        (derivatives.point, derivatives.d1)
    }

    fn bounds(&self) -> (f64, f64) {
        let t = &self.flat_knots;
        (t[self.degree], t[self.poles.len()])
    }

    fn is_closed(&self) -> bool {
        let (t0, t1) = self.bounds();
        self.point(t0)
            .is_equal(&self.point(t1), openrcad_foundation::tolerance::CONFUSION)
    }

    fn transformed(&self, t: &Trsf) -> Self {
        let new_poles = self.poles.iter().map(|p| t.transform_point(p)).collect();
        Self::new(
            self.degree,
            new_poles,
            self.weights.clone(),
            self.knots.clone(),
            self.mults.clone(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quarter_circle(origin: Pnt, radius: f64) -> BSplineCurve {
        BSplineCurve::new(
            2,
            vec![
                Pnt::new(origin.x() + radius, origin.y(), origin.z()),
                Pnt::new(origin.x() + radius, origin.y() + radius, origin.z()),
                Pnt::new(origin.x(), origin.y() + radius, origin.z()),
            ],
            Some(vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0]),
            vec![0.0, 1.0],
            vec![3, 3],
        )
    }

    fn assert_vector_near(actual: GeomVec, expected: GeomVec, tolerance: f64) {
        assert!(
            (actual.x() - expected.x()).abs() <= tolerance,
            "x: actual={} expected={} tolerance={tolerance}",
            actual.x(),
            expected.x()
        );
        assert!(
            (actual.y() - expected.y()).abs() <= tolerance,
            "y: actual={} expected={} tolerance={tolerance}",
            actual.y(),
            expected.y()
        );
        assert!(
            (actual.z() - expected.z()).abs() <= tolerance,
            "z: actual={} expected={} tolerance={tolerance}",
            actual.z(),
            expected.z()
        );
    }

    #[test]
    fn linear_bspline_eval_3d() {
        let poles = vec![
            Pnt::new(0.0, 0.0, 0.0),
            Pnt::new(1.0, 2.0, 3.0),
            Pnt::new(2.0, 0.0, 0.0),
        ];
        let knots = vec![0.0, 1.0, 2.0];
        let mults = vec![2, 1, 2];
        let mut curve = BSplineCurve::new(1, poles, None, knots, mults);

        assert_eq!(curve.point(0.0), Pnt::new(0.0, 0.0, 0.0));
        assert_eq!(curve.point(1.0), Pnt::new(1.0, 2.0, 3.0));
        assert_eq!(curve.point(2.0), Pnt::new(2.0, 0.0, 0.0));
        assert_eq!(curve.point(0.5), Pnt::new(0.5, 1.0, 1.5));

        let (_, v) = curve.d1(0.5);
        assert!((v.x() - 1.0).abs() < 1e-12);
        assert!((v.y() - 2.0).abs() < 1e-12);
        assert!((v.z() - 3.0).abs() < 1e-12);

        // Test knot insertion
        curve.insert_knot(0.5, 1);
        // Degree is 1, so poles increases to 4
        assert_eq!(curve.poles().len(), 4);
        assert_eq!(curve.point(0.5), Pnt::new(0.5, 1.0, 1.5));
    }

    #[test]
    fn rational_conic_second_derivatives_match_reference_values() {
        let curve = quarter_circle(Pnt::ORIGIN, 1.0);
        let derivatives = curve.derivatives(0.5);
        let radial = std::f64::consts::FRAC_1_SQRT_2;

        assert!(derivatives
            .point
            .is_equal(&Pnt::new(radial, radial, 0.0), 1.0e-14));
        // Standard quadratic rational quarter-circle values, also used as the
        // pinned OCCT-compatible conic reference for this evaluator.
        assert_vector_near(
            derivatives.d1,
            GeomVec::new(-1.171_572_875_253_81, 1.171_572_875_253_81, 0.0),
            1.0e-13,
        );
        assert_vector_near(
            derivatives.d2,
            GeomVec::new(-1.941_125_496_954_28, -1.941_125_496_954_28, 0.0),
            1.0e-12,
        );
        assert!(derivatives.d1.dot(&(derivatives.point - Pnt::ORIGIN)).abs() < 1.0e-13);
    }

    #[test]
    fn derivatives_are_finite_at_endpoints_and_repeated_knots() {
        let curve = BSplineCurve::new(
            3,
            vec![
                Pnt::new(0.0, 0.0, 0.0),
                Pnt::new(1.0, 2.0, 0.5),
                Pnt::new(2.0, -1.0, 1.0),
                Pnt::new(3.0, 1.0, -0.5),
                Pnt::new(4.0, 2.0, 0.0),
                Pnt::new(5.0, 0.0, 1.0),
            ],
            Some(vec![1.0, 0.75, 1.5, 0.8, 1.25, 1.0]),
            vec![0.0, 0.5, 1.0],
            vec![4, 2, 4],
        );

        for parameter in [0.0, 0.5, 1.0] {
            let derivatives = curve.derivatives(parameter);
            for value in [
                derivatives.point.x(),
                derivatives.point.y(),
                derivatives.point.z(),
                derivatives.d1.x(),
                derivatives.d1.y(),
                derivatives.d1.z(),
                derivatives.d2.x(),
                derivatives.d2.y(),
                derivatives.d2.z(),
            ] {
                assert!(value.is_finite(), "non-finite derivative at {parameter}");
            }
        }
        assert!(curve.point(0.0).is_equal(&curve.poles()[0], 1.0e-14));
        assert!(curve.point(1.0).is_equal(&curve.poles()[5], 1.0e-14));
    }

    #[test]
    fn second_derivative_agrees_with_finite_difference_away_from_knots() {
        let curve = BSplineCurve::new(
            3,
            vec![
                Pnt::new(0.0, 0.0, 0.0),
                Pnt::new(1.0, 3.0, -1.0),
                Pnt::new(2.0, -2.0, 2.0),
                Pnt::new(4.0, 1.0, 3.0),
            ],
            Some(vec![1.0, 0.7, 1.4, 1.0]),
            vec![0.0, 1.0],
            vec![4, 4],
        );
        let parameter = 0.37;
        let step = 1.0e-5;
        let derivatives = curve.derivatives(parameter);
        let before = curve.point(parameter - step);
        let center = curve.point(parameter);
        let after = curve.point(parameter + step);
        let finite_d1 = GeomVec::new(
            (after.x() - before.x()) / (2.0 * step),
            (after.y() - before.y()) / (2.0 * step),
            (after.z() - before.z()) / (2.0 * step),
        );
        let finite_d2 = GeomVec::new(
            (after.x() - 2.0 * center.x() + before.x()) / step.powi(2),
            (after.y() - 2.0 * center.y() + before.y()) / step.powi(2),
            (after.z() - 2.0 * center.z() + before.z()) / step.powi(2),
        );

        assert_vector_near(derivatives.d1, finite_d1, 2.0e-9);
        assert_vector_near(derivatives.d2, finite_d2, 2.0e-5);
    }

    #[test]
    fn rational_derivatives_hold_across_scale_and_far_origin() {
        let tangent_factor = 1.171_572_875_253_81;
        let curvature_factor = 1.941_125_496_954_28;
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for origin in [Pnt::ORIGIN, Pnt::new(1.0e9, -1.0e9, 5.0e8)] {
                let derivatives = quarter_circle(origin, scale).derivatives(0.5);
                let input_resolution = f64::EPSILON
                    * origin
                        .x()
                        .abs()
                        .max(origin.y().abs())
                        .max(origin.z().abs())
                        .max(1.0);
                let tolerance = (scale * 2.0e-12).max(input_resolution * 4.0);
                assert_vector_near(
                    derivatives.d1,
                    GeomVec::new(-tangent_factor * scale, tangent_factor * scale, 0.0),
                    tolerance,
                );
                assert_vector_near(
                    derivatives.d2,
                    GeomVec::new(-curvature_factor * scale, -curvature_factor * scale, 0.0),
                    tolerance * 2.0,
                );
                let expected = Pnt::new(
                    origin.x() + std::f64::consts::FRAC_1_SQRT_2 * scale,
                    origin.y() + std::f64::consts::FRAC_1_SQRT_2 * scale,
                    origin.z(),
                );
                assert!(derivatives.point.is_equal(&expected, tolerance));
            }
        }
    }
}
