//! A 3D circle (OCCT `Geom_Circle`).
//!
//! A circle lying in the plane of an [`Ax3`] frame (location = center) with a
//! radius. Parameter `u` is the angle from the frame's +X axis:
//! `point(u) = center + radius·(cos u · X + sin u · Y)`.

use core::f64::consts::PI;

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// A circle in 3D.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    pos: Ax3,
    radius: f64,
}

impl Circle {
    /// A circle of `radius` in the frame `pos` (location = center).
    #[inline]
    pub const fn new(pos: Ax3, radius: f64) -> Self {
        Self { pos, radius }
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

    /// The radius.
    #[inline]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// The plane normal (the frame's main direction).
    #[inline]
    pub const fn axis(&self) -> Dir {
        self.pos.direction()
    }
}

impl Curve for Circle {
    fn point(&self, u: f64) -> Pnt {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (cu, su) = (u.cos(), u.sin());
        let r = self.radius;
        Pnt::new(
            c.x() + r * (cu * x.x() + su * y.x()),
            c.y() + r * (cu * x.y() + su * y.y()),
            c.z() + r * (cu * x.z() + su * y.z()),
        )
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        let p = self.point(u);
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        let (su, cu) = (u.sin(), u.cos());
        let r = self.radius;
        let v = Vec::new(
            r * (-su * x.x() + cu * y.x()),
            r * (-su * x.y() + cu * y.y()),
            r * (-su * x.z() + cu * y.z()),
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
        Self::new(self.pos.transformed(t), self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_in_xy_plane() {
        let c = Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        assert_eq!(c.point(0.0), Pnt::new(2.0, 0.0, 0.0));
        assert!((c.point(PI / 2.0).y() - 2.0).abs() < 1e-12);
        // Every point is at `radius` from the center.
        for u in [0.7, 1.3, 2.9, 4.0, 5.5] {
            assert!((c.point(u).distance(&c.center()) - 2.0).abs() < 1e-12);
        }
    }
}
