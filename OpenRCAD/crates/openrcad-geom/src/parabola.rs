//! A 3D parabola (OCCT `Geom_Parabola`).
//!
//! A parabola is defined by its focal length (distance from focus to apex)
//! and a local coordinate system ([`Ax3`]) where:
//! - The origin is the apex of the parabola.
//! - The main direction of the frame is normal to the plane of the parabola.
//! - The X axis is the axis of symmetry (pointing to the positive side).
//! - The Y axis is parallel to the directrix.
//!
//! Parametrization: $point(u) = apex + \frac{u^2}{4f} \cdot XDir + u \cdot YDir$.

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// A parabola in 3D space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Parabola {
    pos: Ax3,
    focal: f64,
}

impl Parabola {
    /// Create a parabola with local coordinate system `pos` and `focal` length.
    ///
    /// # Panics
    /// Panics if `focal` is negative.
    pub fn new(pos: Ax3, focal: f64) -> Self {
        assert!(focal >= 0.0, "focal length must be non-negative");
        Self { pos, focal }
    }

    /// The local coordinate system.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The focal length (distance between vertex and focus).
    #[inline]
    pub const fn focal(&self) -> f64 {
        self.focal
    }

    /// The apex (vertex) of the parabola.
    #[inline]
    pub const fn apex(&self) -> Pnt {
        self.pos.location()
    }

    /// The symmetry axis (X axis).
    #[inline]
    pub const fn x_axis(&self) -> Dir {
        self.pos.x_direction()
    }

    /// The directrix-parallel axis (Y axis).
    #[inline]
    pub const fn y_axis(&self) -> Dir {
        self.pos.y_direction()
    }
}

impl Curve for Parabola {
    fn point(&self, u: f64) -> Pnt {
        let apex = self.pos.location();
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        if self.focal.abs() <= openrcad_foundation::tolerance::CONFUSION {
            // Degenerate parabola (focal = 0): becomes a line along the X direction
            Pnt::new(
                apex.x() + u * x_dir.x(),
                apex.y() + u * x_dir.y(),
                apex.z() + u * x_dir.z(),
            )
        } else {
            let a = (u * u) / (4.0 * self.focal);
            Pnt::new(
                apex.x() + a * x_dir.x() + u * y_dir.x(),
                apex.y() + a * x_dir.y() + u * y_dir.y(),
                apex.z() + a * x_dir.z() + u * y_dir.z(),
            )
        }
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        let p = self.point(u);
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        if self.focal.abs() <= openrcad_foundation::tolerance::CONFUSION {
            (p, Vec::new(x_dir.x(), x_dir.y(), x_dir.z()))
        } else {
            let factor = u / (2.0 * self.focal);
            let v = Vec::new(
                factor * x_dir.x() + y_dir.x(),
                factor * x_dir.y() + y_dir.y(),
                factor * x_dir.z() + y_dir.z(),
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

    fn transformed(&self, t: &Trsf) -> Self {
        let new_focal = self.focal * t.scale_factor().abs();
        Self::new(self.pos.transformed(t), new_focal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parabola_evaluation_3d() {
        let pos = Ax3::new(Pnt::new(1.0, 2.0, 3.0), Dir::dz());
        let p = Parabola::new(pos, 2.0);

        // Apex at u = 0
        assert_eq!(p.point(0.0), Pnt::new(1.0, 2.0, 3.0));

        // u = 4.0: X = 4^2 / (4 * 2) = 2.0, Y = 4.0, Z = 0 -> point should be (1+2, 2+4, 3) = (3, 6, 3)
        assert!((p.point(4.0).x() - 3.0).abs() < 1e-12);
        assert!((p.point(4.0).y() - 6.0).abs() < 1e-12);
        assert!((p.point(4.0).z() - 3.0).abs() < 1e-12);

        // Derivative at u = 4.0: X' = 4 / (2 * 2) = 1.0, Y' = 1.0, Z' = 0 -> d1 is (1, 1, 0)
        let (_, v) = p.d1(4.0);
        assert!((v.x() - 1.0).abs() < 1e-12);
        assert!((v.y() - 1.0).abs() < 1e-12);
        assert!((v.z() - 0.0).abs() < 1e-12);
    }
}
