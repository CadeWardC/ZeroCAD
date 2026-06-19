//! A cylindrical surface (OCCT `Geom_CylindricalSurface`).
//!
//! Defined by an [`Ax3`] frame (location on the axis, main direction = the axis)
//! and a `radius`:
//!
//! ```text
//! P(u, v) = location + radius·(cos u · X + sin u · Y) + v · Z
//! ```
//!
//! `u` is the angle about the axis (closed over `[0, 2π]`); `v` is the signed
//! distance along the axis.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::surface::Surface;

/// An infinite cylindrical surface of fixed radius about an axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CylindricalSurface {
    pos: Ax3,
    radius: f64,
}

impl CylindricalSurface {
    /// A cylinder of `radius` whose axis is the main direction of `pos`.
    #[inline]
    pub const fn new(pos: Ax3, radius: f64) -> Self {
        Self { pos, radius }
    }

    /// The defining frame.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
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
        let r = self.radius;
        let pt = self.pos.location() + x * (r * cu) + y * (r * su) + z * v;
        let du = x * (-r * su) + y * (r * cu);
        let dv = z;
        (pt, du, dv)
    }
}

impl Surface for CylindricalSurface {
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
        Self::new(self.pos.transformed(t), self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Dir;

    #[test]
    fn cylinder_evaluates_on_axis() {
        let s = CylindricalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        // u = 0 -> along +X at radius 2; v lifts along +Z.
        let p = s.point(0.0, 5.0);
        assert!(p.distance(&Pnt::new(2.0, 0.0, 5.0)) < 1e-12);
        // Every surface point is `radius` from the axis.
        for u in [0.3, 1.1, 2.7, 4.5] {
            let q = s.point(u, 3.0);
            let radial = (q.x() * q.x() + q.y() * q.y()).sqrt();
            assert!((radial - 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn cylinder_derivatives_are_orthogonal_to_radius() {
        let s = CylindricalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 1.5);
        let (_, du, dv) = s.d1(1.0, 2.0);
        // dv is the axis; du is tangent to the circle (both ⊥ to each other).
        assert!(du.dot(&dv).abs() < 1e-12);
    }
}
