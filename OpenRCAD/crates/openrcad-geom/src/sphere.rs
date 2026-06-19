//! A spherical surface (OCCT `Geom_SphericalSurface`).
//!
//! Defined by an [`Ax3`] frame (location = center) and a `radius`:
//!
//! ```text
//! P(u, v) = center + radius·cos v·(cos u · X + sin u · Y) + radius·sin v · Z
//! ```
//!
//! `u` is the longitude (closed over `[0, 2π]`); `v` is the latitude in
//! `[-π/2, +π/2]` (south pole to north pole).

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::surface::Surface;

/// A spherical surface of fixed radius about a center.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SphericalSurface {
    pos: Ax3,
    radius: f64,
}

impl SphericalSurface {
    /// A sphere of `radius` centered at the location of `pos`.
    #[inline]
    pub const fn new(pos: Ax3, radius: f64) -> Self {
        Self { pos, radius }
    }

    /// The defining frame (location = center).
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The center.
    #[inline]
    pub const fn center(&self) -> Pnt {
        self.pos.location()
    }

    /// The radius.
    #[inline]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// The point and both first partial derivatives at `(u, v)`.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let x = Vec::from_dir(self.pos.x_direction());
        let y = Vec::from_dir(self.pos.y_direction());
        let z = Vec::from_dir(self.pos.direction());
        let (cu, su) = (u.cos(), u.sin());
        let (cv, sv) = (v.cos(), v.sin());
        let r = self.radius;
        let pt = self.pos.location() + x * (r * cv * cu) + y * (r * cv * su) + z * (r * sv);
        let du = x * (-r * cv * su) + y * (r * cv * cu);
        let dv = x * (-r * sv * cu) + y * (-r * sv * su) + z * (r * cv);
        (pt, du, dv)
    }
}

impl Surface for SphericalSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        self.d1(u, v).0
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (0.0, 2.0 * PI, -PI / 2.0, PI / 2.0)
    }

    fn is_uclosed(&self) -> bool {
        true
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self::new(self.pos.transformed(t), self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Dir;

    #[test]
    fn sphere_points_are_on_the_radius() {
        let s = SphericalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 3.0);
        // North pole at v = +π/2, south pole at v = -π/2.
        assert!(s.point(0.0, PI / 2.0).distance(&Pnt::new(0.0, 0.0, 3.0)) < 1e-12);
        assert!(s.point(1.0, -PI / 2.0).distance(&Pnt::new(0.0, 0.0, -3.0)) < 1e-12);
        // Equator at v = 0.
        assert!(s.point(0.0, 0.0).distance(&Pnt::new(3.0, 0.0, 0.0)) < 1e-12);
        for u in [0.2, 1.5, 3.0, 5.0] {
            for v in [-1.0, -0.3, 0.4, 1.2] {
                assert!((s.point(u, v).distance(&s.center()) - 3.0).abs() < 1e-12);
            }
        }
    }
}
