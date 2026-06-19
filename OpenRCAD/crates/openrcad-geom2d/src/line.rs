//! A 2D line (OCCT `Geom2d_Line`).
//!
//! A line is carried by a 2D axis ([`Ax2d`]); `point(u)` is `location + u·dir`.

use openrcad_foundation::{Ax2d, Dir2d, Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::curve::Curve2d;

/// An unbounded 2D line.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Line2d {
    pos: Ax2d,
}

impl Line2d {
    /// A line along the axis `pos`.
    #[inline]
    pub const fn new(pos: Ax2d) -> Self {
        Self { pos }
    }

    /// A line through `p` in direction `d`.
    #[inline]
    pub fn from_point_dir(p: Pnt2d, d: Dir2d) -> Self {
        Self::new(Ax2d::new(p, d))
    }

    /// The carrying axis.
    #[inline]
    pub const fn position(&self) -> Ax2d {
        self.pos
    }

    /// A point on the line (the parameter origin).
    #[inline]
    pub const fn location(&self) -> Pnt2d {
        self.pos.location()
    }

    /// The line direction.
    #[inline]
    pub const fn direction(&self) -> Dir2d {
        self.pos.direction()
    }
}

impl Curve2d for Line2d {
    fn point(&self, u: f64) -> Pnt2d {
        let p = self.pos.location();
        let d = self.pos.direction();
        Pnt2d::new(p.x() + d.x() * u, p.y() + d.y() * u)
    }

    fn d1(&self, u: f64) -> (Pnt2d, Vec2d) {
        let d = self.pos.direction();
        (self.point(u), Vec2d::new(d.x(), d.y()))
    }

    fn bounds(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_closed(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_evaluates_along_direction() {
        let l = Line2d::from_point_dir(Pnt2d::new(1.0, 2.0), Dir2d::dx());
        assert_eq!(l.point(0.0), Pnt2d::new(1.0, 2.0));
        assert_eq!(l.point(5.0), Pnt2d::new(6.0, 2.0));
        let (_, d) = l.d1(3.0);
        assert_eq!(d, Vec2d::new(1.0, 0.0));
    }
}
