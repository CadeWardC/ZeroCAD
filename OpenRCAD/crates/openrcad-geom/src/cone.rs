//! A conical surface (OCCT `Geom_ConicalSurface`).
//!
//! Defined by an [`Ax3`] frame (location on the axis at the reference circle,
//! main direction = the axis), a reference `radius` (at `v = 0`) and a
//! `semi_angle` α — the signed angle between the slant and the axis, positive
//! when the cone *widens* in `+Z`:
//!
//! ```text
//! r(v)   = radius + v · tan α
//! P(u,v) = location + r(v)·(cos u · X + sin u · Y) + v · Z
//! ```
//!
//! `u` is the angle about the axis (closed over `[0, 2π]`); `v` is the signed
//! distance along the axis.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::surface::Surface;

/// An infinite conical surface.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConicalSurface {
    pos: Ax3,
    radius: f64,
    semi_angle: f64,
}

impl ConicalSurface {
    /// A cone with reference `radius` at `v = 0` and half-angle `semi_angle`
    /// about the main direction of `pos`.
    #[inline]
    pub const fn new(pos: Ax3, radius: f64, semi_angle: f64) -> Self {
        Self {
            pos,
            radius,
            semi_angle,
        }
    }

    /// The defining frame.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The reference radius (at `v = 0`).
    #[inline]
    pub const fn ref_radius(&self) -> f64 {
        self.radius
    }

    /// The half-angle between slant and axis.
    #[inline]
    pub const fn semi_angle(&self) -> f64 {
        self.semi_angle
    }

    /// The radius of the cone at axial parameter `v`.
    #[inline]
    pub fn radius_at(&self, v: f64) -> f64 {
        self.radius + v * self.semi_angle.tan()
    }

    /// The point and both first partial derivatives at `(u, v)`.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let x = Vec::from_dir(self.pos.x_direction());
        let y = Vec::from_dir(self.pos.y_direction());
        let z = Vec::from_dir(self.pos.direction());
        let (cu, su) = (u.cos(), u.sin());
        let r = self.radius_at(v);
        let slope = self.semi_angle.tan();
        let pt = self.pos.location() + x * (r * cu) + y * (r * su) + z * v;
        let du = x * (-r * su) + y * (r * cu);
        let dv = x * (slope * cu) + y * (slope * su) + z;
        (pt, du, dv)
    }
}

impl Surface for ConicalSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        self.d1(u, v).0
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (0.0, 2.0 * PI, f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_uclosed(&self) -> bool {
        true
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self::new(self.pos.transformed(t), self.radius, self.semi_angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Dir;

    #[test]
    fn cone_radius_shrinks_with_height() {
        // Reference radius 4 at base; narrows so radius is 0 at v = 4 (45°-ish).
        let alpha = (-1.0_f64).atan(); // slope = tan α = -1
        let s = ConicalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 4.0, alpha);
        assert!((s.radius_at(0.0) - 4.0).abs() < 1e-12);
        assert!((s.radius_at(4.0) - 0.0).abs() < 1e-12);
        let p = s.point(0.0, 0.0);
        assert!(p.distance(&Pnt::new(4.0, 0.0, 0.0)) < 1e-12);
        let apex = s.point(0.0, 4.0);
        assert!(apex.distance(&Pnt::new(0.0, 0.0, 4.0)) < 1e-12);
    }
}
