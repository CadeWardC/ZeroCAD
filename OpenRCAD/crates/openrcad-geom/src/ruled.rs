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
    /// Analytic (u, v) of a point on a *helical* ruled surface — one whose
    /// rails are a [`crate::Helix`] paired with another helix or a circle (a
    /// thread band or its run-out wedge). Returns `None` when neither rail is
    /// a helix (callers fall back to their generic Newton projection).
    ///
    /// The angle comes from `atan2` in the helix frame; the turn count (a
    /// multi-turn band is NOT `2π`-periodic — every turn is at a different
    /// height) is disambiguated by the caller's `hint`, else by the point's
    /// axial height against the rails' mean lead, else (pure-circle pairing /
    /// zero lead) normalized into `[0, 2π)`.
    pub fn helical_uv_hinted(&self, p: Pnt, hint: Option<(f64, f64)>) -> Option<(f64, f64)> {
        use crate::{GeomCurve, Helix};
        const TAU: f64 = core::f64::consts::TAU;
        let helix: &Helix = match (&self.curve1, &self.curve2) {
            (GeomCurve::Helix(h), _) => h,
            (_, GeomCurve::Helix(h)) => h,
            _ => return None,
        };
        let lead_of = |c: &GeomCurve| match c {
            GeomCurve::Helix(h) => h.lead(),
            _ => 0.0,
        };
        let pos = helix.position();
        let d = p - pos.location();
        let dx = d.dot(&Vec::from_dir(pos.x_direction()));
        let dy = d.dot(&Vec::from_dir(pos.y_direction()));
        let ang = dy.atan2(dx);
        let mean_lead = 0.5 * (lead_of(&self.curve1) + lead_of(&self.curve2));
        let u_est = if let Some((hu, _)) = hint {
            hu
        } else if mean_lead.abs() > 1e-9 {
            d.dot(&Vec::from_dir(pos.direction())) * TAU / mean_lead
        } else {
            ang.rem_euclid(TAU)
        };
        let k = ((u_est - ang) / TAU).round();
        let u = ang + TAU * k;
        let p1 = self.curve1.point(u);
        let p2 = self.curve2.point(u);
        let ruling = p2 - p1;
        let len2 = ruling.dot(&ruling);
        let v = if len2 > 1e-18 {
            (p - p1).dot(&ruling) / len2
        } else {
            0.5
        };
        Some((u, v))
    }

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
