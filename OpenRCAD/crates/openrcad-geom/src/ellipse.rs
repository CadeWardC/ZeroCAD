//! A 3D ellipse (OCCT `Geom_Ellipse`).
//!
//! Lies in the plane of an [`Ax3`] frame (location = center), with a major
//! radius along X and a minor radius along Y:
//! `point(u) = center + major·cos u · X + minor·sin u · Y`.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// An ellipse in 3D.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ellipse {
    pos: Ax3,
    major_radius: f64,
    minor_radius: f64,
}

impl Ellipse {
    /// An ellipse in `pos` with major (X) and minor (Y) radii.
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

    /// The center.
    #[inline]
    pub const fn center(&self) -> Pnt {
        self.pos.location()
    }

    /// The major radius.
    #[inline]
    pub const fn major_radius(&self) -> f64 {
        self.major_radius
    }

    /// The minor radius.
    #[inline]
    pub const fn minor_radius(&self) -> f64 {
        self.minor_radius
    }
}

impl Curve for Ellipse {
    fn point(&self, u: f64) -> Pnt {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (cu, su) = (u.cos(), u.sin());
        Pnt::new(
            c.x() + self.major_radius * cu * x.x() + self.minor_radius * su * y.x(),
            c.y() + self.major_radius * cu * x.y() + self.minor_radius * su * y.y(),
            c.z() + self.major_radius * cu * x.z() + self.minor_radius * su * y.z(),
        )
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        let p = self.point(u);
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (su, cu) = (u.sin(), u.cos());
        let v = Vec::new(
            -self.major_radius * su * x.x() + self.minor_radius * cu * y.x(),
            -self.major_radius * su * x.y() + self.minor_radius * cu * y.y(),
            -self.major_radius * su * x.z() + self.minor_radius * cu * y.z(),
        );
        (p, v)
    }

    fn bounds(&self) -> (f64, f64) {
        (0.0, 2.0 * PI)
    }

    fn is_closed(&self) -> bool {
        true
    }

    fn is_periodic(&self) -> bool {
        true
    }

    fn period(&self) -> f64 {
        2.0 * PI
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
    fn ellipse_extremes_in_xy_plane() {
        let e = Ellipse::new(Ax3::new(Pnt::origin(), Dir::dz()), 4.0, 2.0);
        assert_eq!(e.point(0.0), Pnt::new(4.0, 0.0, 0.0));
        assert!((e.point(PI / 2.0).y() - 2.0).abs() < 1e-12);
        assert!((e.point(PI).x() + 4.0).abs() < 1e-12);
    }
}
