//! A 2D parabola (OCCT `Geom2d_Parabola`).
//!
//! A parabola is defined by its focal length (distance from focus to apex)
//! and a local coordinate system ([`Ax22d`]) where:
//! - The origin is the apex of the parabola.
//! - The X axis is the axis of symmetry (pointing to the positive side).
//! - The Y axis is parallel to the directrix.
//!
//! The equation in local coordinates is: $Y^2 = 4 \cdot f \cdot X$.
//! Parametrization: $point(u) = apex + \frac{u^2}{4f} \cdot XDir + u \cdot YDir$.

use openrcad_foundation::{Ax22d, Dir2d, Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::curve::Curve2d;

/// A parabola in the plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Parabola2d {
    pos: Ax22d,
    focal: f64,
}

impl Parabola2d {
    /// Create a parabola with local coordinate system `pos` and `focal` length.
    ///
    /// # Panics
    /// Panics if `focal` is negative.
    pub fn new(pos: Ax22d, focal: f64) -> Self {
        assert!(focal >= 0.0, "focal length must be non-negative");
        Self { pos, focal }
    }

    /// The local coordinate system.
    #[inline]
    pub const fn position(&self) -> Ax22d {
        self.pos
    }

    /// The focal length (distance between vertex and focus).
    #[inline]
    pub const fn focal(&self) -> f64 {
        self.focal
    }

    /// The apex (vertex) of the parabola.
    #[inline]
    pub const fn apex(&self) -> Pnt2d {
        self.pos.location()
    }

    /// The symmetry axis (X axis).
    #[inline]
    pub const fn x_axis(&self) -> Dir2d {
        self.pos.x_direction()
    }

    /// The directrix-parallel axis (Y axis).
    #[inline]
    pub const fn y_axis(&self) -> Dir2d {
        self.pos.y_direction()
    }
}

impl Curve2d for Parabola2d {
    fn point(&self, u: f64) -> Pnt2d {
        let apex = self.pos.location();
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        if self.focal.abs() <= openrcad_foundation::tolerance::CONFUSION {
            // Degenerate parabola (focal = 0): becomes a line along the X direction
            Pnt2d::new(apex.x() + u * x_dir.x(), apex.y() + u * x_dir.y())
        } else {
            let a = (u * u) / (4.0 * self.focal);
            Pnt2d::new(
                apex.x() + a * x_dir.x() + u * y_dir.x(),
                apex.y() + a * x_dir.y() + u * y_dir.y(),
            )
        }
    }

    fn d1(&self, u: f64) -> (Pnt2d, Vec2d) {
        let p = self.point(u);
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        if self.focal.abs() <= openrcad_foundation::tolerance::CONFUSION {
            (p, Vec2d::new(x_dir.x(), x_dir.y()))
        } else {
            let factor = u / (2.0 * self.focal);
            let v = Vec2d::new(
                factor * x_dir.x() + y_dir.x(),
                factor * x_dir.y() + y_dir.y(),
            );
            (p, v)
        }
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
    fn parabola_evaluation() {
        let pos = Ax22d::new(Pnt2d::new(1.0, 2.0), Dir2d::dx());
        let p = Parabola2d::new(pos, 2.0);

        // Apex at u = 0
        assert_eq!(p.point(0.0), Pnt2d::new(1.0, 2.0));

        // u = 4.0: X = 4^2 / (4 * 2) = 2.0, Y = 4.0 -> point should be (1+2, 2+4) = (3, 6)
        assert!((p.point(4.0).x() - 3.0).abs() < 1e-12);
        assert!((p.point(4.0).y() - 6.0).abs() < 1e-12);

        // Derivative at u = 4.0: X' = 4 / (2 * 2) = 1.0, Y' = 1.0 -> d1 is (1, 1)
        let (_, v) = p.d1(4.0);
        assert!((v.x() - 1.0).abs() < 1e-12);
        assert!((v.y() - 1.0).abs() < 1e-12);
    }
}
