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
pub mod cylinder_plane_section;
pub mod ellipse;
pub mod endpoint_corrected;
pub mod hyperbola;
pub mod line;
pub mod parabola;
pub mod plane_torus_section;
pub mod span;
pub mod sphere_great_circle;
pub mod torus_plane_section;

pub use bspline::BSplineCurve2d;
pub use circle::Circle2d;
pub use curve::Curve2d;
pub use cylinder_plane_section::CylinderPlaneSection2d;
pub use ellipse::Ellipse2d;
pub use endpoint_corrected::EndpointCorrectedCurve2d;
pub use hyperbola::Hyperbola2d;
pub use line::Line2d;
pub use parabola::Parabola2d;
pub use plane_torus_section::PlaneTorusSection2d;
pub use span::{CurveKind2d, CurveSpan};
pub use sphere_great_circle::SphereGreatCircle2d;
pub use torus_plane_section::TorusPlaneSection2d;

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
    /// Exact pcurve of an axis-parallel plane section on a torus. Appended to
    /// preserve every existing serialized discriminant.
    TorusPlaneSection(TorusPlaneSection2d),
    /// Exact regular-torus section in the cutting plane's parameter space.
    PlaneTorusSection(PlaneTorusSection2d),
    /// Exact oblique planar section in a cylinder's parameter space. Appended
    /// to preserve every existing serialized discriminant.
    CylinderPlaneSection(CylinderPlaneSection2d),
    /// Exact great circle in a sphere's parameter space. Appended to preserve
    /// every existing serialized discriminant.
    SphereGreatCircle(SphereGreatCircle2d),
    /// Analytic pcurve with topology-exact endpoint corrections. Appended to
    /// preserve every existing serialized discriminant.
    EndpointCorrected(EndpointCorrectedCurve2d),
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

    /// Construct an exact torus/plane-section pcurve.
    #[inline]
    pub fn torus_plane_section(section: TorusPlaneSection2d) -> Self {
        Self::TorusPlaneSection(section)
    }

    /// Construct an exact torus section in its cutting plane's coordinates.
    #[inline]
    pub fn plane_torus_section(section: PlaneTorusSection2d) -> Self {
        Self::PlaneTorusSection(section)
    }

    /// Construct an exact oblique cylinder/plane-section pcurve.
    #[inline]
    pub fn cylinder_plane_section(section: CylinderPlaneSection2d) -> Self {
        Self::CylinderPlaneSection(section)
    }

    /// Construct an exact spherical great-circle pcurve.
    #[inline]
    pub fn sphere_great_circle(circle: SphereGreatCircle2d) -> Self {
        Self::SphereGreatCircle(circle)
    }

    /// Construct a topology-endpoint-corrected pcurve.
    #[inline]
    pub fn endpoint_corrected(curve: EndpointCorrectedCurve2d) -> Self {
        Self::EndpointCorrected(curve)
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
            Self::TorusPlaneSection(section) => section.point(u),
            Self::PlaneTorusSection(section) => section.point(u),
            Self::CylinderPlaneSection(section) => section.point(u),
            Self::SphereGreatCircle(circle) => circle.point(u),
            Self::EndpointCorrected(curve) => curve.point(u),
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
            Self::TorusPlaneSection(section) => section.d1(u),
            Self::PlaneTorusSection(section) => section.d1(u),
            Self::CylinderPlaneSection(section) => section.d1(u),
            Self::SphereGreatCircle(circle) => circle.d1(u),
            Self::EndpointCorrected(curve) => curve.d1(u),
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
            Self::TorusPlaneSection(section) => section.bounds(),
            Self::PlaneTorusSection(section) => section.bounds(),
            Self::CylinderPlaneSection(section) => section.bounds(),
            Self::SphereGreatCircle(circle) => circle.bounds(),
            Self::EndpointCorrected(curve) => curve.bounds(),
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
            Self::TorusPlaneSection(section) => section.is_closed(),
            Self::PlaneTorusSection(section) => section.is_closed(),
            Self::CylinderPlaneSection(section) => section.is_closed(),
            Self::SphereGreatCircle(circle) => circle.is_closed(),
            Self::EndpointCorrected(curve) => curve.is_closed(),
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
            Self::TorusPlaneSection(section) => section.is_periodic(),
            Self::PlaneTorusSection(section) => section.is_periodic(),
            Self::CylinderPlaneSection(section) => section.is_periodic(),
            Self::SphereGreatCircle(circle) => circle.is_periodic(),
            Self::EndpointCorrected(curve) => curve.is_periodic(),
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
            Self::TorusPlaneSection(section) => section.period(),
            Self::PlaneTorusSection(section) => section.period(),
            Self::CylinderPlaneSection(section) => section.period(),
            Self::SphereGreatCircle(circle) => circle.period(),
            Self::EndpointCorrected(curve) => curve.period(),
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
