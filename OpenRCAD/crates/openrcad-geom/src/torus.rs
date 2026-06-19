//! A toroidal surface (OCCT `Geom_ToroidalSurface`).
//!
//! Defined by an [`Ax3`] frame (the torus centre, main direction = the axis of
//! revolution), a `major_radius` `R` (centre → tube-centre circle) and a
//! `minor_radius` `r` (the tube):
//!
//! ```text
//! P(u, v) = centre + (R + r·cos v)·(cos u · X + sin u · Y) + r·sin v · Z
//! ```
//!
//! `u` revolves about the axis (`Z`), closed over `[0, 2π]`; `v` goes around the
//! tube, closed over `[0, 2π]` with `v = 0` on the outer equator. A torus is the
//! sweep surface a rolling-ball fillet traces along a circular edge, which is why
//! the cylinder/cone blends need it.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::surface::Surface;

/// A toroidal surface of revolution.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToroidalSurface {
    pos: Ax3,
    major_radius: f64,
    minor_radius: f64,
}

impl ToroidalSurface {
    /// A torus whose tube-centre circle has radius `major_radius` and whose tube
    /// has radius `minor_radius`, centred on the main direction of `pos`.
    #[inline]
    pub const fn new(pos: Ax3, major_radius: f64, minor_radius: f64) -> Self {
        Self {
            pos,
            major_radius,
            minor_radius,
        }
    }

    /// The defining frame.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The major radius (centre → tube-centre circle).
    #[inline]
    pub const fn major_radius(&self) -> f64 {
        self.major_radius
    }

    /// The minor radius (the tube).
    #[inline]
    pub const fn minor_radius(&self) -> f64 {
        self.minor_radius
    }

    /// The point and both first partial derivatives at `(u, v)`.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let x = Vec::from_dir(self.pos.x_direction());
        let y = Vec::from_dir(self.pos.y_direction());
        let z = Vec::from_dir(self.pos.direction());
        let (cu, su) = (u.cos(), u.sin());
        let (cv, sv) = (v.cos(), v.sin());
        let (rr, r) = (self.major_radius, self.minor_radius);

        let e_r = x * cu + y * su;
        let tube = rr + r * cv;
        let pt = self.pos.location() + e_r * tube + z * (r * sv);
        let du = (x * (-su) + y * cu) * tube;
        let dv = e_r * (-r * sv) + z * (r * cv);
        (pt, du, dv)
    }
}

impl Surface for ToroidalSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        self.d1(u, v).0
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (0.0, 2.0 * PI, 0.0, 2.0 * PI)
    }

    fn is_uclosed(&self) -> bool {
        true
    }

    fn is_vclosed(&self) -> bool {
        true
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self::new(
            self.pos.transformed(t),
            self.major_radius,
            self.minor_radius,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Dir;

    #[test]
    fn torus_outer_and_inner_equator() {
        let s = ToroidalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 3.0, 1.0);
        // v = 0 -> outer equator at R + r = 4 along +X.
        let outer = s.point(0.0, 0.0);
        assert!(outer.distance(&Pnt::new(4.0, 0.0, 0.0)) < 1e-12);
        // v = π -> inner equator at R - r = 2.
        let inner = s.point(0.0, PI);
        assert!(inner.distance(&Pnt::new(2.0, 0.0, 0.0)) < 1e-12);
        // v = π/2 -> top of the tube, lifted by r along +Z at radius R.
        let top = s.point(0.0, PI / 2.0);
        assert!(top.distance(&Pnt::new(3.0, 0.0, 1.0)) < 1e-12);
    }

    #[test]
    fn torus_every_point_is_minor_radius_from_the_tube_circle() {
        let s = ToroidalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 5.0, 2.0);
        for u in [0.2, 1.3, 3.0, 5.5] {
            for v in [0.0, 1.1, 2.5, 4.0] {
                let p = s.point(u, v);
                // Nearest point on the tube-centre circle (radius R in the XY plane).
                let radial = (p.x() * p.x() + p.y() * p.y()).sqrt();
                let tube_centre = Pnt::new(5.0 * p.x() / radial, 5.0 * p.y() / radial, 0.0);
                assert!((p.distance(&tube_centre) - 2.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn torus_derivatives_are_orthogonal() {
        let s = ToroidalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 4.0, 1.5);
        let (_, du, dv) = s.d1(0.7, 1.2);
        // u sweeps about the axis, v about the tube — the two are orthogonal.
        assert!(du.dot(&dv).abs() < 1e-9);
    }
}
