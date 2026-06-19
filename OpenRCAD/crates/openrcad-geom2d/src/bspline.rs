//! A 2D B-Spline/NURBS curve (OCCT `Geom2d_BSplineCurve`).

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::curve::Curve2d;

/// A 2D B-Spline/NURBS curve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BSplineCurve2d {
    degree: usize,
    poles: Vec<Pnt2d>,
    weights: Option<Vec<f64>>,
    knots: Vec<f64>,
    mults: Vec<usize>,
    flat_knots: Vec<f64>,
}

impl BSplineCurve2d {
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
        poles: Vec<Pnt2d>,
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
    pub fn poles(&self) -> &[Pnt2d] {
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
}

impl Curve2d for BSplineCurve2d {
    fn point(&self, u: f64) -> Pnt2d {
        let span = self.find_span(u);
        let t = &self.flat_knots;
        let degree = self.degree;

        // Initialize active homogeneous control points
        let mut d = Vec::with_capacity(degree + 1);
        for j in 0..=degree {
            let idx = span - degree + j;
            let p = self.poles[idx];
            let w = self.weights.as_ref().map(|w| w[idx]).unwrap_or(1.0);
            d.push([p.x() * w, p.y() * w, w]);
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
                for coord in 0..3 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        let res_h = d[degree];
        let w = res_h[2];
        Pnt2d::new(res_h[0] / w, res_h[1] / w)
    }

    fn d1(&self, u: f64) -> (Pnt2d, Vec2d) {
        let span = self.find_span(u);
        let t = &self.flat_knots;
        let degree = self.degree;

        // Initialize active homogeneous control points
        let mut d = Vec::with_capacity(degree + 1);
        for j in 0..=degree {
            let idx = span - degree + j;
            let p = self.poles[idx];
            let w = self.weights.as_ref().map(|w| w[idx]).unwrap_or(1.0);
            d.push([p.x() * w, p.y() * w, w]);
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
                for coord in 0..3 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        // Now we compute the final step point and derivative simultaneously
        let idx = span;
        let denom = t[idx + 1] - t[idx];
        let alpha = if denom.abs() < 1e-15 {
            0.0
        } else {
            (u - t[idx]) / denom
        };

        let mut p_h = [0.0; 3];
        for coord in 0..3 {
            p_h[coord] = (1.0 - alpha) * d[degree - 1][coord] + alpha * d[degree][coord];
        }

        let mut dp_h = [0.0; 3];
        if denom.abs() > 1e-15 {
            let factor = degree as f64 / denom;
            for coord in 0..3 {
                dp_h[coord] = factor * (d[degree][coord] - d[degree - 1][coord]);
            }
        }

        let w = p_h[2];
        let dw = dp_h[2];
        let pt = Pnt2d::new(p_h[0] / w, p_h[1] / w);

        let vx = (dp_h[0] - pt.x() * dw) / w;
        let vy = (dp_h[1] - pt.y() * dw) / w;
        let vec = Vec2d::new(vx, vy);

        (pt, vec)
    }

    fn bounds(&self) -> (f64, f64) {
        let t = &self.flat_knots;
        (t[self.degree], t[self.poles.len()])
    }

    fn is_closed(&self) -> bool {
        let (t0, t1) = self.bounds();
        self.point(t0).distance(&self.point(t1)) <= openrcad_foundation::tolerance::CONFUSION
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_bspline_eval() {
        // Linear B-spline from (0,0) to (1,2) to (2,0)
        let poles = vec![
            Pnt2d::new(0.0, 0.0),
            Pnt2d::new(1.0, 2.0),
            Pnt2d::new(2.0, 0.0),
        ];
        let knots = vec![0.0, 1.0, 2.0];
        let mults = vec![2, 1, 2]; // clamped knot vector: [0.0, 0.0, 1.0, 2.0, 2.0]
        let curve = BSplineCurve2d::new(1, poles, None, knots, mults);

        assert_eq!(curve.point(0.0), Pnt2d::new(0.0, 0.0));
        assert_eq!(curve.point(1.0), Pnt2d::new(1.0, 2.0));
        assert_eq!(curve.point(2.0), Pnt2d::new(2.0, 0.0));
        assert_eq!(curve.point(0.5), Pnt2d::new(0.5, 1.0));

        let (_, v) = curve.d1(0.5);
        assert!((v.x() - 1.0).abs() < 1e-12);
        assert!((v.y() - 2.0).abs() < 1e-12);
    }
}
