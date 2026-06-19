//! A 3D line (OCCT `Geom_Line`).
//!
//! Carried by an [`Ax1`]; `point(u)` is `location + u · direction`.

use openrcad_foundation::{Ax1, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// An unbounded 3D line.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Line {
    pos: Ax1,
}

impl Line {
    /// A line along `pos`.
    #[inline]
    pub const fn new(pos: Ax1) -> Self {
        Self { pos }
    }

    /// A line through `p` in direction `d`.
    #[inline]
    pub fn from_point_dir(p: Pnt, d: Dir) -> Self {
        Self::new(Ax1::new(p, d))
    }

    /// The carrying axis.
    #[inline]
    pub const fn position(&self) -> Ax1 {
        self.pos
    }

    /// A point on the line.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.pos.location()
    }

    /// The line direction.
    #[inline]
    pub const fn direction(&self) -> Dir {
        self.pos.direction()
    }
}

impl Curve for Line {
    fn point(&self, u: f64) -> Pnt {
        self.pos.point(u)
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        (self.pos.point(u), Vec::from_dir(self.pos.direction()))
    }

    fn bounds(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_closed(&self) -> bool {
        false
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self::new(Ax1::new(
            t.transform_point(&self.pos.location()),
            t.transform_dir(&self.pos.direction()),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_along_axis() {
        let l = Line::from_point_dir(Pnt::origin(), Dir::dz());
        assert_eq!(l.point(0.0), Pnt::origin());
        assert_eq!(l.point(7.0), Pnt::new(0.0, 0.0, 7.0));
    }
}
