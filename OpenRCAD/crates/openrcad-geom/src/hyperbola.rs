//! A 3D hyperbola (OCCT `Geom_Hyperbola`).
//!
//! A hyperbola is defined by a local coordinate system ([`Ax3`]) and two
//! radii: `major_radius` ($a$) and `minor_radius` ($b$).
//! The equation in local coordinates is: $\frac{X^2}{a^2} - \frac{Y^2}{b^2} = 1$.
//! Parametrization: $point(u) = center + (a \cosh u) \cdot XDir + (b \sinh u) \cdot YDir$.

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::curve::Curve;

/// A hyperbola in 3D space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hyperbola {
    pos: Ax3,
    major_radius: f64,
    minor_radius: f64,
}

impl Hyperbola {
    /// Create a hyperbola with local coordinate system `pos`, `major_radius`, and `minor_radius`.
    ///
    /// # Panics
    /// Panics if `major_radius` or `minor_radius` is negative.
    pub fn new(pos: Ax3, major_radius: f64, minor_radius: f64) -> Self {
        assert!(major_radius >= 0.0, "major radius must be non-negative");
        assert!(minor_radius >= 0.0, "minor radius must be non-negative");
        Self {
            pos,
            major_radius,
            minor_radius,
        }
    }

    /// The local coordinate system.
    #[inline]
    pub const fn position(&self) -> Ax3 {
        self.pos
    }

    /// The center of the hyperbola.
    #[inline]
    pub const fn center(&self) -> Pnt {
        self.pos.location()
    }

    /// The major radius ($a$).
    #[inline]
    pub const fn major_radius(&self) -> f64 {
        self.major_radius
    }

    /// The minor radius ($b$).
    #[inline]
    pub const fn minor_radius(&self) -> f64 {
        self.minor_radius
    }

    /// The reference X axis.
    #[inline]
    pub const fn x_axis(&self) -> Dir {
        self.pos.x_direction()
    }

    /// The reference Y axis.
    #[inline]
    pub const fn y_axis(&self) -> Dir {
        self.pos.y_direction()
    }
}

impl Curve for Hyperbola {
    fn point(&self, u: f64) -> Pnt {
        let center = self.pos.location();
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        let a = self.major_radius * u.cosh();
        let b = self.minor_radius * u.sinh();

        Pnt::new(
            center.x() + a * x_dir.x() + b * y_dir.x(),
            center.y() + a * x_dir.y() + b * y_dir.y(),
            center.z() + a * x_dir.z() + b * y_dir.z(),
        )
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
        let p = self.point(u);
        let x_dir = self.pos.x_direction();
        let y_dir = self.pos.y_direction();

        let a = self.major_radius * u.sinh();
        let b = self.minor_radius * u.cosh();

        let v = Vec::new(
            a * x_dir.x() + b * y_dir.x(),
            a * x_dir.y() + b * y_dir.y(),
            a * x_dir.z() + b * y_dir.z(),
        );
        (p, v)
    }

    fn bounds(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_closed(&self) -> bool {
        false
    }

    fn transformed(&self, t: &Trsf) -> Self {
        let s = t.scale_factor().abs();
        Self::new(
            self.pos.transformed(t),
            self.major_radius * s,
            self.minor_radius * s,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hyperbola_evaluation_3d() {
        let pos = Ax3::new(Pnt::new(1.0, 2.0, 3.0), Dir::dz());
        let h = Hyperbola::new(pos, 3.0, 4.0);

        // At u = 0: cosh(0)=1, sinh(0)=0 -> X = 3, Y = 0, Z = 0 -> point should be (1+3, 2+0, 3) = (4, 2, 3)
        assert_eq!(h.point(0.0), Pnt::new(4.0, 2.0, 3.0));

        // Derivative at u = 0: sinh(0)=0, cosh(0)=1 -> X' = 0, Y' = 4, Z' = 0 -> d1 is (0, 4, 0)
        let (_, v) = h.d1(0.0);
        assert!((v.x() - 0.0).abs() < 1e-12);
        assert!((v.y() - 4.0).abs() < 1e-12);
        assert!((v.z() - 0.0).abs() < 1e-12);
    }
}
