#![forbid(unsafe_code)]
// de Boor evaluation indexes parallel pole/knot arrays; index loops mirror the
// recurrence more clearly than iterator chains.
#![allow(clippy::needless_range_loop)]
//! 2D geometry for OpenRCAD — curves on the plane (OCCT `TKG2d` / `Geom2d`).
//!
//! Concrete 2D curves implementing [`Curve2d`]:
//! - [`Line2d`] (OCCT `Geom2d_Line`),
//! - [`Circle2d`] (OCCT `Geom2d_Circle`),
//! - [`Ellipse2d`] (OCCT `Geom2d_Ellipse`).
//!
//! [`GeomCurve2d`] is the owned sum type — used when a curve must be stored by
//! value (for example as the 2D parametric curve of a topology edge). It is
//! `Clone` + `Serialize`, never a `Box<dyn>` or a lifetime.

use serde::{Deserialize, Serialize};

pub mod bspline;
pub mod circle;
pub mod curve;
pub mod ellipse;
pub mod hyperbola;
pub mod line;
pub mod parabola;

pub use bspline::BSplineCurve2d;
pub use circle::Circle2d;
pub use curve::Curve2d;
pub use ellipse::Ellipse2d;
pub use hyperbola::Hyperbola2d;
pub use line::Line2d;
pub use parabola::Parabola2d;

/// An owned 2D curve: one of the concrete [`Curve2d`]s, storable by value.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GeomCurve2d {
    /// A line.
    Line(Line2d),
    /// A circle.
    Circle(Circle2d),
    /// An ellipse.
    Ellipse(Ellipse2d),
    /// A parabola.
    Parabola(Parabola2d),
    /// A hyperbola.
    Hyperbola(Hyperbola2d),
    /// A B-spline or NURBS curve.
    BSpline(BSplineCurve2d),
}

impl GeomCurve2d {
    /// Construct a [`Line2d`].
    #[inline]
    pub fn line(l: Line2d) -> Self {
        Self::Line(l)
    }

    /// Construct a [`Circle2d`].
    #[inline]
    pub fn circle(c: Circle2d) -> Self {
        Self::Circle(c)
    }

    /// Construct an [`Ellipse2d`].
    #[inline]
    pub fn ellipse(e: Ellipse2d) -> Self {
        Self::Ellipse(e)
    }

    /// Construct a [`Parabola2d`].
    #[inline]
    pub fn parabola(p: Parabola2d) -> Self {
        Self::Parabola(p)
    }

    /// Construct a [`Hyperbola2d`].
    #[inline]
    pub fn hyperbola(h: Hyperbola2d) -> Self {
        Self::Hyperbola(h)
    }

    /// Construct a [`BSplineCurve2d`].
    #[inline]
    pub fn bspline(b: BSplineCurve2d) -> Self {
        Self::BSpline(b)
    }
}

impl Curve2d for GeomCurve2d {
    fn point(&self, u: f64) -> openrcad_foundation::Pnt2d {
        match self {
            Self::Line(l) => l.point(u),
            Self::Circle(c) => c.point(u),
            Self::Ellipse(e) => e.point(u),
            Self::Parabola(p) => p.point(u),
            Self::Hyperbola(h) => h.point(u),
            Self::BSpline(b) => b.point(u),
        }
    }

    fn d1(&self, u: f64) -> (openrcad_foundation::Pnt2d, openrcad_foundation::Vec2d) {
        match self {
            Self::Line(l) => l.d1(u),
            Self::Circle(c) => c.d1(u),
            Self::Ellipse(e) => e.d1(u),
            Self::Parabola(p) => p.d1(u),
            Self::Hyperbola(h) => h.d1(u),
            Self::BSpline(b) => b.d1(u),
        }
    }

    fn bounds(&self) -> (f64, f64) {
        match self {
            Self::Line(l) => l.bounds(),
            Self::Circle(c) => c.bounds(),
            Self::Ellipse(e) => e.bounds(),
            Self::Parabola(p) => p.bounds(),
            Self::Hyperbola(h) => h.bounds(),
            Self::BSpline(b) => b.bounds(),
        }
    }

    fn is_closed(&self) -> bool {
        match self {
            Self::Line(l) => l.is_closed(),
            Self::Circle(c) => c.is_closed(),
            Self::Ellipse(e) => e.is_closed(),
            Self::Parabola(p) => p.is_closed(),
            Self::Hyperbola(h) => h.is_closed(),
            Self::BSpline(b) => b.is_closed(),
        }
    }

    fn is_periodic(&self) -> bool {
        match self {
            Self::Line(l) => l.is_periodic(),
            Self::Circle(c) => c.is_periodic(),
            Self::Ellipse(e) => e.is_periodic(),
            Self::Parabola(p) => p.is_periodic(),
            Self::Hyperbola(h) => h.is_periodic(),
            Self::BSpline(b) => b.is_periodic(),
        }
    }

    fn period(&self) -> f64 {
        match self {
            Self::Line(l) => l.period(),
            Self::Circle(c) => c.period(),
            Self::Ellipse(e) => e.period(),
            Self::Parabola(p) => p.period(),
            Self::Hyperbola(h) => h.period(),
            Self::BSpline(b) => b.period(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enum_delegates_to_variants() {
        let geom: GeomCurve2d = GeomCurve2d::circle(Circle2d::from_center(
            openrcad_foundation::Pnt2d::origin(),
            1.0,
        ));
        assert_eq!(geom.point(0.0), openrcad_foundation::Pnt2d::new(1.0, 0.0));
        assert!(geom.is_closed());
    }
}
