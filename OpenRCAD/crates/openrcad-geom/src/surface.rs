//! The [`Surface`] trait — the abstraction over every parametric surface
//! (OCCT `Geom_Surface`).

use openrcad_foundation::{Pnt, Trsf};

/// A parametric surface in 3D.
pub trait Surface {
    /// The point at parameters `(u, v)`.
    fn point(&self, u: f64, v: f64) -> Pnt;

    /// The parameter rectangle `(u_min, u_max, v_min, v_max)`.
    fn bounds(&self) -> (f64, f64, f64, f64);

    /// True if the surface closes in U (default: not).
    fn is_uclosed(&self) -> bool {
        false
    }

    /// True if the surface closes in V (default: not).
    fn is_vclosed(&self) -> bool {
        false
    }

    /// Apply a transform, returning a new surface.
    fn transformed(&self, t: &Trsf) -> Self
    where
        Self: Sized;
}
