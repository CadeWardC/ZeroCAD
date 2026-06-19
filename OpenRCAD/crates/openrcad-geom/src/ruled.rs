use crate::{Curve, GeomCurve, Surface};
use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

/// A ruled surface formed by sweeping a straight line between two curves.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuledSurface {
    pub curve1: GeomCurve,
    pub curve2: GeomCurve,
}

impl RuledSurface {
    /// Create a new ruled surface.
    pub fn new(curve1: GeomCurve, curve2: GeomCurve) -> Self {
        Self { curve1, curve2 }
    }
}

impl Surface for RuledSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        let p1 = self.curve1.point(u);
        let p2 = self.curve2.point(u);
        p1 + (p2 - p1) * v
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        let (u_min, u_max) = self.curve1.bounds();
        (u_min, u_max, 0.0, 1.0)
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self {
            curve1: self.curve1.transformed(t),
            curve2: self.curve2.transformed(t),
        }
    }
}

impl RuledSurface {
    /// Analytical derivatives.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let (p1, dp1) = self.curve1.d1(u);
        let (p2, dp2) = self.curve2.d1(u);

        let pt = p1 + (p2 - p1) * v;
        let du = dp1 + (dp2 - dp1) * v;
        let dv = p2 - p1;

        (pt, du, dv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GeomCurve, Line};
    use openrcad_foundation::{Ax1, Dir, Pnt};

    #[test]
    fn test_ruled_surface_evaluation() {
        let c1 = GeomCurve::line(Line::new(Ax1::new(Pnt::origin(), Dir::dx())));
        let c2 = GeomCurve::line(Line::new(Ax1::new(Pnt::new(0.0, 10.0, 0.0), Dir::dx())));
        let surf = RuledSurface::new(c1, c2);

        let p_mid = surf.point(5.0, 0.5);
        assert!((p_mid.x() - 5.0).abs() < 1e-9);
        assert!((p_mid.y() - 5.0).abs() < 1e-9);
        assert!(p_mid.z().abs() < 1e-9);

        let (pt, du, dv) = surf.d1(5.0, 0.5);
        assert!((pt.x() - 5.0).abs() < 1e-9);
        assert!((pt.y() - 5.0).abs() < 1e-9);
        assert!(du.x() > 0.9); // direction is +X
        assert!(dv.y() > 9.9); // direction is +Y, magnitude 10
    }
}
