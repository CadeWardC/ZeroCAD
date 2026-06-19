//! A 3D B-Spline/NURBS curve (OCCT `Geom_BSplineCurve`).

use openrcad_foundation::{BndBox, Interval, Interval3, Pnt, Trsf, Vec as GeomVec};
use serde::{Deserialize, Serialize};

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
        let n = self.poles.len();
        let degree = self.degree;
        let t = &self.flat_knots;

        // Clamp parameter to active range
        let u = u.clamp(t[degree], t[n]);

        if u >= t[n] {
            return n - 1;
        }

        let mut low = degree;
        let mut high = n;
        while low + 1 < high {
            let mid = (low + high) / 2;
            if u < t[mid] {
                high = mid;
            } else {
                low = mid;
            }
        }
        low
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
}

impl Curve for BSplineCurve {
    fn point(&self, u: f64) -> Pnt {
        let span = self.find_span(u);
        let t = &self.flat_knots;
        let degree = self.degree;

        // Initialize active homogeneous control points
        let mut d = Vec::with_capacity(degree + 1);
        for j in 0..=degree {
            let idx = span - degree + j;
            let p = self.poles[idx];
            let w = self.weights.as_ref().map(|w| w[idx]).unwrap_or(1.0);
            d.push([p.x() * w, p.y() * w, p.z() * w, w]);
        }

        // Run de Boor's algorithm
        for r in 1..=degree {
            for j in (r..=degree).rev() {
                let idx = span - degree + j;
                let denom = t[idx + degree + 1 - r] - t[idx];
                let alpha = if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (u - t[idx]) / denom
                };
                for coord in 0..4 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        let res_h = d[degree];
        let w = res_h[3];
        Pnt::new(res_h[0] / w, res_h[1] / w, res_h[2] / w)
    }

    fn d1(&self, u: f64) -> (Pnt, GeomVec) {
        let span = self.find_span(u);
        let t = &self.flat_knots;
        let degree = self.degree;

        // Initialize active homogeneous control points
        let mut d = Vec::with_capacity(degree + 1);
        for j in 0..=degree {
            let idx = span - degree + j;
            let p = self.poles[idx];
            let w = self.weights.as_ref().map(|w| w[idx]).unwrap_or(1.0);
            d.push([p.x() * w, p.y() * w, p.z() * w, w]);
        }

        // Run de Boor's algorithm up to step degree - 1
        for r in 1..degree {
            for j in (r..=degree).rev() {
                let idx = span - degree + j;
                let denom = t[idx + degree + 1 - r] - t[idx];
                let alpha = if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (u - t[idx]) / denom
                };
                for coord in 0..4 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        // Now compute the final step point and derivative simultaneously
        let idx = span;
        let denom = t[idx + 1] - t[idx];
        let alpha = if denom.abs() < 1e-15 {
            0.0
        } else {
            (u - t[idx]) / denom
        };

        let mut p_h = [0.0; 4];
        for coord in 0..4 {
            p_h[coord] = (1.0 - alpha) * d[degree - 1][coord] + alpha * d[degree][coord];
        }

        let mut dp_h = [0.0; 4];
        if denom.abs() > 1e-15 {
            let factor = degree as f64 / denom;
            for coord in 0..4 {
                dp_h[coord] = factor * (d[degree][coord] - d[degree - 1][coord]);
            }
        }

        let w = p_h[3];
        let dw = dp_h[3];
        let pt = Pnt::new(p_h[0] / w, p_h[1] / w, p_h[2] / w);

        let vx = (dp_h[0] - pt.x() * dw) / w;
        let vy = (dp_h[1] - pt.y() * dw) / w;
        let vz = (dp_h[2] - pt.z() * dw) / w;
        let vec = GeomVec::new(vx, vy, vz);

        (pt, vec)
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
}
