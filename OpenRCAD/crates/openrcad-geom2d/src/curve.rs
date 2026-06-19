//! The [`Curve2d`] trait — the abstraction over every 2D parametric curve.
//!
//! Mirrors OCCT's abstract `Geom2d_Curve`: a curve is a map from a parameter
//! `u` to a point on the plane, with first derivative, a parameter range, and
//! closed/periodic flags.

use openrcad_foundation::{Pnt2d, Vec2d};

/// A parametric curve in the plane.
pub trait Curve2d {
    /// The point at parameter `u`.
    fn point(&self, u: f64) -> Pnt2d;

    /// The point and first derivative at parameter `u`.
    fn d1(&self, u: f64) -> (Pnt2d, Vec2d);

    /// The parameter range `(u_min, u_max)`.
    fn bounds(&self) -> (f64, f64);

    /// True if the curve joins up at its parameter ends.
    fn is_closed(&self) -> bool;

    /// True if the curve is periodic (default: not).
    fn is_periodic(&self) -> bool {
        false
    }

    /// The period, if [`is_periodic`](Self::is_periodic) (default: 0).
    fn period(&self) -> f64 {
        0.0
    }
}
