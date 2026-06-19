//! The [`Curve`] trait — the abstraction over every 3D parametric curve
//! (OCCT `Geom_Curve`).

use openrcad_foundation::{Pnt, Trsf, Vec};

/// A parametric curve in 3D.
pub trait Curve {
    /// The point at parameter `u`.
    fn point(&self, u: f64) -> Pnt;

    /// The point and first derivative at parameter `u`.
    fn d1(&self, u: f64) -> (Pnt, Vec);

    /// The parameter range `(u_min, u_max)`.
    fn bounds(&self) -> (f64, f64);

    /// True if the curve joins up at its parameter ends.
    fn is_closed(&self) -> bool;

    /// True if the curve is periodic (default: not).
    fn is_periodic(&self) -> bool {
        false
    }

    /// The period, if periodic (default: 0).
    fn period(&self) -> f64 {
        0.0
    }

    /// Apply a transform, returning a new curve.
    fn transformed(&self, t: &Trsf) -> Self
    where
        Self: Sized;
}
