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
        let helices: std::vec::Vec<&Helix> = [&self.curve1, &self.curve2]
            .into_iter()
            .filter_map(|curve| match curve {
                GeomCurve::Helix(helix) => Some(helix),
                _ => None,
            })
            .collect();
        if helices.is_empty() {
            return None;
        }

        // Generate absolute-turn candidates from every available invariant.
        // Lead resolves a multi-turn helix from axial height; taper resolves a
        // zero-lead spiral from radius; a prior UV remains useful for ordinary
        // circle-like rails. Each estimate is snapped onto the point's angular
        // branch in that helix's frame.
        let mut candidates = std::vec::Vec::new();
        for helix in helices {
            let pos = helix.position();
            let offset = p - pos.location();
            let x = offset.dot(&Vec::from_dir(pos.x_direction()));
            let y = offset.dot(&Vec::from_dir(pos.y_direction()));
            let angle = y.atan2(x);
            let radius = x.hypot(y);
            let axial = offset.dot(&Vec::from_dir(pos.direction()));
            let mut estimates = std::vec::Vec::new();
            if helix.taper().abs() > 1e-12 {
                estimates.push((radius - helix.radius()) / helix.taper());
            }
            if helix.lead().abs() > 1e-12 {
                estimates.push(axial * TAU / helix.lead());
            }
            if let Some((u, _)) = hint {
                estimates.push(u);
            }
            if estimates.is_empty() {
                estimates.push(angle);
            }
            for estimate in estimates {
                let u = angle + TAU * ((estimate - angle) / TAU).round();
                if u.is_finite()
                    && candidates
                        .iter()
                        .all(|candidate: &f64| (*candidate - u).abs() > 1e-10)
                {
                    candidates.push(u);
                }
            }
        }

        // Select the branch whose finite ruling segment actually contains the
        // point. Clamping v for the score prevents an aliased whole-turn branch
        // from appearing exact only by extrapolating far outside the face.
        candidates
            .into_iter()
            .filter_map(|u| {
                let p1 = self.curve1.point(u);
                let p2 = self.curve2.point(u);
                let ruling = p2 - p1;
                let len2 = ruling.dot(&ruling);
                let v = if len2 > 1e-18 {
                    (p - p1).dot(&ruling) / len2
                } else {
                    0.5
                };
                let bounded_v = v.clamp(0.0, 1.0);
                let projected = p1 + ruling * bounded_v;
                let error = projected.distance(&p);
                error.is_finite().then_some((error, u, bounded_v))
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal))
            .map(|(_, u, v)| (u, v))
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

    #[test]
    fn helical_uv_resolves_long_leaded_and_tapered_rails() {
        use crate::Helix;

        let frame = openrcad_foundation::Ax3::new(Pnt::origin(), Dir::dz());
        let leaded = GeomCurve::helix(Helix::new(frame, 3.0, 0.0, 1.0));
        let shifted = GeomCurve::helix(Helix::new(
            openrcad_foundation::Ax3::new(Pnt::new(0.0, 0.0, 0.1), Dir::dz()),
            3.0,
            0.0,
            1.0,
        ));
        let surface = RuledSurface::new(leaded.clone(), shifted);
        let point = leaded.point(60.0);
        let (u, v) = surface.helical_uv_hinted(point, None).unwrap();
        assert!((u - 60.0).abs() < 1e-9);
        assert!(v.abs() < 1e-9);

        let tapered = GeomCurve::helix(Helix::new(frame, 4.25, 0.4, 0.0));
        let circle = GeomCurve::helix(Helix::new(
            openrcad_foundation::Ax3::new(Pnt::new(0.0, 0.0, -0.75), Dir::dz()),
            4.0,
            0.0,
            0.0,
        ));
        let surface = RuledSurface::new(tapered.clone(), circle);
        let point = tapered.point(-2.0);
        let (u, v) = surface.helical_uv_hinted(point, None).unwrap();
        assert!((u + 2.0).abs() < 1e-9);
        assert!(v.abs() < 1e-9);
    }
}
