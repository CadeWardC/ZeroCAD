//! A 2D ellipse (OCCT `Geom2d_Ellipse`).
//!
//! Defined by a frame [`Ax22d`] (location = center) and a major and minor
//! radius along the frame's X and Y axes:
//! `point(u) = center + major·cos u · X + minor·sin u · Y`.

use core::f64::consts::PI;

use openrcad_foundation::{Ax22d, Dir2d, Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::curve::Curve2d;

/// An ellipse in the plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ellipse2d {
    pos: Ax22d,
    major_radius: f64,
    minor_radius: f64,
}

impl Ellipse2d {
    /// An ellipse in `pos` with the given major (X) and minor (Y) radii.
    #[inline]
    pub const fn new(pos: Ax22d, major_radius: f64, minor_radius: f64) -> Self {
        Self {
            pos,
            major_radius,
            minor_radius,
        }
    }

    /// An axis-aligned ellipse centered at `c`.
    #[inline]
    pub fn from_center(c: Pnt2d, major_radius: f64, minor_radius: f64) -> Self {
        Self::new(Ax22d::new(c, Dir2d::dx()), major_radius, minor_radius)
    }

    /// The defining frame.
    #[inline]
    pub const fn position(&self) -> Ax22d {
        self.pos
    }

    /// The center.
    #[inline]
    pub const fn center(&self) -> Pnt2d {
        self.pos.location()
    }

    /// The major radius (along X).
    #[inline]
    pub const fn major_radius(&self) -> f64 {
        self.major_radius
    }

    /// The minor radius (along Y).
    #[inline]
    pub const fn minor_radius(&self) -> f64 {
        self.minor_radius
    }
}

impl Curve2d for Ellipse2d {
    fn point(&self, u: f64) -> Pnt2d {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (cu, su) = (u.cos(), u.sin());
        Pnt2d::new(
            c.x() + self.major_radius * cu * x.x() + self.minor_radius * su * y.x(),
            c.y() + self.major_radius * cu * x.y() + self.minor_radius * su * y.y(),
        )
    }

    fn d1(&self, u: f64) -> (Pnt2d, Vec2d) {
        let p = self.point(u);
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (su, cu) = (u.sin(), u.cos());
        let v = Vec2d::new(
            -self.major_radius * su * x.x() + self.minor_radius * cu * y.x(),
            -self.major_radius * su * x.y() + self.minor_radius * cu * y.y(),
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ellipse_hits_axis_extremes() {
        let e = Ellipse2d::from_center(Pnt2d::origin(), 4.0, 2.0);
        assert_eq!(e.point(0.0), Pnt2d::new(4.0, 0.0));
        // cos/sin of multiples of PI/2 carry ~1e-16 error — compare with tolerance.
        let p1 = e.point(PI / 2.0);
        assert!(p1.x().abs() < 1e-12);
        assert!((p1.y() - 2.0).abs() < 1e-12);
        let p2 = e.point(PI);
        assert!((p2.x() + 4.0).abs() < 1e-12);
        assert!(p2.y().abs() < 1e-12);
    }
}
