//! A 3D B-Spline/NURBS surface (OCCT `Geom_BSplineSurface`).

use openrcad_foundation::{BndBox, Interval, Interval3, Pnt, Trsf, Vec as GeomVec};
use serde::{Deserialize, Serialize};

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

    /// Find the knot span index in a flat knot vector.
    fn find_span(degree: usize, flat_knots: &[f64], poles_len: usize, u: f64) -> usize {
        let u = u.clamp(flat_knots[degree], flat_knots[poles_len]);
        if u >= flat_knots[poles_len] {
            return poles_len - 1;
        }
        let mut low = degree;
        let mut high = poles_len;
        while low + 1 < high {
            let mid = (low + high) / 2;
            if u < flat_knots[mid] {
                high = mid;
            } else {
                low = mid;
            }
        }
        low
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
        let u_span = Self::find_span(self.u_degree, &self.u_flat_knots, self.poles.len(), u);
        let v_span = Self::find_span(self.v_degree, &self.v_flat_knots, self.poles[0].len(), v);

        let p_u = self.u_degree;
        let p_v = self.v_degree;

        // We will evaluate the active grid of (p_u + 1) x (p_v + 1) control points.
        // First, for each active row in U:
        // Evaluate the 1D B-spline curve in V to get:
        // - Homogeneous point V_pts[i]
        // - Homogeneous derivative V_ders[i]
        let mut v_pts = Vec::with_capacity(p_u + 1);
        let mut v_ders = Vec::with_capacity(p_u + 1);

        for i in 0..=p_u {
            let u_idx = u_span - p_u + i;

            // Extract the active row of poles/weights
            let mut d = Vec::with_capacity(p_v + 1);
            for j in 0..=p_v {
                let v_idx = v_span - p_v + j;
                let p = self.poles[u_idx][v_idx];
                let w = self
                    .weights
                    .as_ref()
                    .map(|w| w[u_idx][v_idx])
                    .unwrap_or(1.0);
                d.push([p.x() * w, p.y() * w, p.z() * w, w]);
            }

            // de Boor in V up to degree - 1
            for r in 1..p_v {
                for j in (r..=p_v).rev() {
                    let idx = v_span - p_v + j;
                    let denom = self.v_flat_knots[idx + p_v + 1 - r] - self.v_flat_knots[idx];
                    let alpha = if denom.abs() < 1e-15 {
                        0.0
                    } else {
                        (v - self.v_flat_knots[idx]) / denom
                    };
                    for coord in 0..4 {
                        d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                    }
                }
            }

            // Final step in V
            let idx = v_span;
            let denom = self.v_flat_knots[idx + 1] - self.v_flat_knots[idx];
            let alpha = if denom.abs() < 1e-15 {
                0.0
            } else {
                (v - self.v_flat_knots[idx]) / denom
            };

            let mut p_h = [0.0; 4];
            for coord in 0..4 {
                p_h[coord] = (1.0 - alpha) * d[p_v - 1][coord] + alpha * d[p_v][coord];
            }

            let mut dp_h = [0.0; 4];
            if denom.abs() > 1e-15 {
                let factor = p_v as f64 / denom;
                for coord in 0..4 {
                    dp_h[coord] = factor * (d[p_v][coord] - d[p_v - 1][coord]);
                }
            }

            v_pts.push(p_h);
            v_ders.push(dp_h);
        }

        // Now, we have v_pts (point in V) and v_ders (derivative in V) for each active U row.
        // 1. Evaluate the point and U-derivative using v_pts
        let (pt_h, du_h) = self.de_boor_1d_h(u_span, u, &v_pts);

        // 2. Evaluate the V-derivative by evaluating v_ders in U
        let dv_h = self.de_boor_1d_val_h(u_span, u, &v_ders);

        // Project homogeneous results to spatial 3D
        let w = pt_h[3];
        let dw_u = du_h[3];
        let dw_v = dv_h[3];

        let pt = Pnt::new(pt_h[0] / w, pt_h[1] / w, pt_h[2] / w);

        let su = GeomVec::new(
            (du_h[0] - pt.x() * dw_u) / w,
            (du_h[1] - pt.y() * dw_u) / w,
            (du_h[2] - pt.z() * dw_u) / w,
        );

        let sv = GeomVec::new(
            (dv_h[0] - pt.x() * dw_v) / w,
            (dv_h[1] - pt.y() * dw_v) / w,
            (dv_h[2] - pt.z() * dw_v) / w,
        );

        (pt, su, sv)
    }

    /// Helper: Run 1D de Boor on homogeneous coordinates to get both value and derivative.
    fn de_boor_1d_h(&self, u_span: usize, u: f64, active_h: &[[f64; 4]]) -> ([f64; 4], [f64; 4]) {
        let p = self.u_degree;
        let t = &self.u_flat_knots;

        let mut d = active_h.to_vec();

        // de Boor up to degree - 1
        for r in 1..p {
            for j in (r..=p).rev() {
                let idx = u_span - p + j;
                let denom = t[idx + p + 1 - r] - t[idx];
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

        // Final step
        let idx = u_span;
        let denom = t[idx + 1] - t[idx];
        let alpha = if denom.abs() < 1e-15 {
            0.0
        } else {
            (u - t[idx]) / denom
        };

        let mut p_h = [0.0; 4];
        for coord in 0..4 {
            p_h[coord] = (1.0 - alpha) * d[p - 1][coord] + alpha * d[p][coord];
        }

        let mut dp_h = [0.0; 4];
        if denom.abs() > 1e-15 {
            let factor = p as f64 / denom;
            for coord in 0..4 {
                dp_h[coord] = factor * (d[p][coord] - d[p - 1][coord]);
            }
        }

        (p_h, dp_h)
    }

    /// Helper: Run 1D de Boor on homogeneous coordinates to get only the value.
    fn de_boor_1d_val_h(&self, u_span: usize, u: f64, active_h: &[[f64; 4]]) -> [f64; 4] {
        let p = self.u_degree;
        let t = &self.u_flat_knots;

        let mut d = active_h.to_vec();

        for r in 1..=p {
            for j in (r..=p).rev() {
                let idx = u_span - p + j;
                let denom = t[idx + p + 1 - r] - t[idx];
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

        d[p]
    }
}

impl Surface for BSplineSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        let (p, _, _) = self.d1(u, v);
        p
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
}
