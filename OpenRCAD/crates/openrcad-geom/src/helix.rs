//! A 3D helix (no OCCT analogue — OpenRCAD native, for modeled threads).
//!
//! A (possibly tapered) helix about the main direction of an [`Ax3`] frame.
//! Parameter `u` is the angle in radians from the frame's +X axis:
//!
//! `point(u) = location + r(u)·(cos u · X + sin u · Y) + (lead·u / 2π) · Z`
//!
//! with `r(u) = radius + taper·u`. `lead` is the axial advance per full turn,
//! *signed*: a negative lead makes the helix left-handed. `taper` (radius
//! change per radian) lets one type also represent conical spirals, which the
//! thread builder uses for run-out ramps.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// A helix (or conical spiral when tapered) in 3D.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Helix {
    pos: Ax3,
    radius: f64,
    taper: f64,
    lead: f64,
}

impl Helix {
    /// A helix about the frame's main direction. `radius` is the radius at
    /// `u = 0`, `taper` the radius change per radian, and `lead` the signed
    /// axial advance per full turn.
    #[inline]
    pub const fn new(pos: Ax3, radius: f64, taper: f64, lead: f64) -> Self {
        Self {
            pos,
            radius,
            taper,
            lead,
        }
    }

    /// The defining frame (location = axis point at `u = 0`).
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The radius at `u = 0`.
    #[inline]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// The radius change per radian.
    #[inline]
    pub const fn taper(&self) -> f64 {
        self.taper
    }

    /// The signed axial advance per full turn.
    #[inline]
    pub const fn lead(&self) -> f64 {
        self.lead
    }

    /// The helix axis direction.
    #[inline]
    pub const fn axis(&self) -> Dir {
        self.pos.direction()
    }

    /// The radius at parameter `u`.
    #[inline]
    pub fn radius_at(&self, u: f64) -> f64 {
        self.radius + self.taper * u
    }
}

impl Curve for Helix {
    fn point(&self, u: f64) -> Pnt {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let z = self.pos.direction();
        let (cu, su) = (u.cos(), u.sin());
        let r = self.radius + self.taper * u;
        let h = self.lead * u / (2.0 * PI);
        Pnt::new(
            c.x() + r * (cu * x.x() + su * y.x()) + h * z.x(),
            c.y() + r * (cu * x.y() + su * y.y()) + h * z.y(),
            c.z() + r * (cu * x.z() + su * y.z()) + h * z.z(),
        )
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        let p = self.point(u);
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let z = self.pos.direction();
        let (cu, su) = (u.cos(), u.sin());
        let r = self.radius + self.taper * u;
        let k = self.lead / (2.0 * PI);
        // d/du [r(u)·(cos u·X + sin u·Y)] = taper·(cos u·X + sin u·Y)
        //                                 + r·(−sin u·X + cos u·Y)
        let v = Vec::new(
            self.taper * (cu * x.x() + su * y.x()) + r * (-su * x.x() + cu * y.x()) + k * z.x(),
            self.taper * (cu * x.y() + su * y.y()) + r * (-su * x.y() + cu * y.y()) + k * z.y(),
            self.taper * (cu * x.z() + su * y.z()) + r * (-su * x.z() + cu * y.z()) + k * z.z(),
        );
        (p, v)
    }

    fn bounds(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_closed(&self) -> bool {
        false
    }

    fn transformed(&self, t: &Trsf) -> Self {
        let scale = t.scale_factor().abs();
        Self::new(
            self.pos.transformed(t),
            self.radius * scale,
            self.taper * scale,
            self.lead * scale,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helix_advances_and_rotates() {
        // Radius 2, lead 1 per turn, about +Z from origin.
        let h = Helix::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0, 0.0, 1.0);
        assert_eq!(h.point(0.0), Pnt::new(2.0, 0.0, 0.0));
        let p = h.point(2.0 * PI);
        assert!((p.x() - 2.0).abs() < 1e-12);
        assert!(p.y().abs() < 1e-12);
        assert!((p.z() - 1.0).abs() < 1e-12);
        // Quarter turn: on +Y, a quarter of the lead up.
        let q = h.point(PI / 2.0);
        assert!(q.x().abs() < 1e-12);
        assert!((q.y() - 2.0).abs() < 1e-12);
        assert!((q.z() - 0.25).abs() < 1e-12);
    }

    #[test]
    fn tapered_helix_radius_and_d1() {
        let h = Helix::new(Ax3::new(Pnt::origin(), Dir::dz()), 1.0, 0.1, 2.0);
        assert!((h.radius_at(3.0) - 1.3).abs() < 1e-12);
        // Finite-difference check of d1.
        let u = 1.234;
        let eps = 1e-6;
        let (p, d) = h.d1(u);
        let p2 = h.point(u + eps);
        let fd = (p2 - p) / eps;
        assert!((d.x() - fd.x()).abs() < 1e-4);
        assert!((d.y() - fd.y()).abs() < 1e-4);
        assert!((d.z() - fd.z()).abs() < 1e-4);
    }

    #[test]
    fn left_handed_helix_descends() {
        let h = Helix::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0, 0.0, -1.0);
        assert!(h.point(PI).z() < 0.0);
    }
}
