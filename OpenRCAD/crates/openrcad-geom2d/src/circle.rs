//! A 2D circle (OCCT `Geom2d_Circle`).
//!
//! Defined by a frame [`Ax22d`] (location = center, X/Y reference directions)
//! and a radius. Parameter `u` is the standard angle from the +X axis:
//! `point(u) = center + radius·(cos u · X + sin u · Y)`.

use core::f64::consts::PI;

use openrcad_foundation::{Ax22d, Dir2d, Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::curve::Curve2d;

/// A circle in the plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Circle2d {
    pos: Ax22d,
    radius: f64,
}

impl Circle2d {
    /// A circle of `radius` in the frame `pos` (whose location is the center).
    #[inline]
    pub const fn new(pos: Ax22d, radius: f64) -> Self {
        Self { pos, radius }
    }

    /// A circle of `radius` centered at `c` in the global frame.
    #[inline]
    pub fn from_center(c: Pnt2d, radius: f64) -> Self {
        Self::new(Ax22d::new(c, Dir2d::dx()), radius)
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

    /// The radius.
    #[inline]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// The reference X axis (the direction of parameter 0).
    #[inline]
    pub const fn x_axis(&self) -> Dir2d {
        self.pos.x_direction()
    }
}

impl Curve2d for Circle2d {
    fn point(&self, u: f64) -> Pnt2d {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (cu, su) = (u.cos(), u.sin());
        let r = self.radius;
        Pnt2d::new(
            c.x() + r * (cu * x.x() + su * y.x()),
            c.y() + r * (cu * x.y() + su * y.y()),
        )
    }

    fn d1(&self, u: f64) -> (Pnt2d, Vec2d) {
        let p = self.point(u);
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (su, cu) = (u.sin(), u.cos());
        let r = self.radius;
        let v = Vec2d::new(
            r * (-su * x.x() + cu * y.x()),
            r * (-su * x.y() + cu * y.y()),
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
    fn circle_radius_and_origin() {
        let c = Circle2d::from_center(Pnt2d::origin(), 2.0);
        assert_eq!(c.point(0.0), Pnt2d::new(2.0, 0.0));
        // cos(PI/2) is ~6e-17, not exactly 0 — compare with tolerance.
        let q = c.point(PI / 2.0);
        assert!(q.x().abs() < 1e-12);
        assert!((q.y() - 2.0).abs() < 1e-12);
        // Every point is at distance `radius` from the center.
        for u in [0.7, 1.3, 2.9, 4.0, 5.5] {
            assert!((c.point(u).distance(&c.center()) - 2.0).abs() < 1e-12);
        }
        assert!(c.is_closed() && c.is_periodic());
    }
}
