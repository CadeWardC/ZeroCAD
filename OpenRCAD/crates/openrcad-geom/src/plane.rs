//! A plane (OCCT `Geom_Plane`).
//!
//! Defined by an [`Ax3`] frame: `point(u, v) = location + u · X + v · Y`. The
//! frame's main direction is the plane normal.

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf};
use serde::{Deserialize, Serialize};

use crate::surface::Surface;

/// A plane in 3D.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pos: Ax3,
}

impl Plane {
    /// The plane of frame `pos`.
    #[inline]
    pub const fn new(pos: Ax3) -> Self {
        Self { pos }
    }

    /// A plane through `p` with normal `n` (an X axis is chosen automatically).
    #[inline]
    pub fn from_point_normal(p: Pnt, n: Dir) -> Self {
        Self::new(Ax3::new(p, n))
    }

    /// The defining frame.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The plane normal.
    #[inline]
    pub const fn normal(&self) -> Dir {
        self.pos.direction()
    }

    /// The origin.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.pos.location()
    }

    /// Evaluates the point and first partial derivatives at (u, v).
    #[inline]
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, openrcad_foundation::Vec, openrcad_foundation::Vec) {
        let pt = self.point(u, v);
        let su = openrcad_foundation::Vec::from_dir(self.pos.x_direction());
        let sv = openrcad_foundation::Vec::from_dir(self.pos.y_direction());
        (pt, su, sv)
    }
}

impl Surface for Plane {
    fn point(&self, u: f64, v: f64) -> Pnt {
        let c = self.pos.location();
        let x = self.pos.x_direction();
        let y = self.pos.y_direction();
        Pnt::new(
            c.x() + u * x.x() + v * y.x(),
            c.y() + u * x.y() + v * y.y(),
            c.z() + u * x.z() + v * y.z(),
        )
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
        )
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self::new(self.pos.transformed(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_evaluates_in_uv() {
        let p = Plane::from_point_normal(Pnt::origin(), Dir::dz());
        assert_eq!(p.point(0.0, 0.0), Pnt::origin());
        // X axis is auto-chosen; for normal = +Z it is +X.
        assert_eq!(p.point(3.0, 0.0), Pnt::new(3.0, 0.0, 0.0));
        assert_eq!(p.normal(), Dir::dz());
    }
}
