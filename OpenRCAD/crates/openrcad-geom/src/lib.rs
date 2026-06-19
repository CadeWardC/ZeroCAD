#![forbid(unsafe_code)]
// NURBS/Bézier evaluators index parallel pole/knot arrays and 2-D pole grids;
// index loops track the de Boor / Bernstein recurrences more clearly than
// iterator chains, so the range-loop lint is allowed crate-wide.
#![allow(clippy::needless_range_loop)]
//! 3D geometry for OpenRCAD — curves and surfaces (OCCT `TKG3d` / `Geom`).
//!
//! Concrete geometry implementing [`Curve`] and [`Surface`]:
//! - [`Line`] (`Geom_Line`), [`Circle`] (`Geom_Circle`), [`Ellipse`] (`Geom_Ellipse`);
//! - [`Plane`] (`Geom_Plane`).
//!
//! [`GeomCurve`] and [`GeomSurface`] are the owned sum types topology stores
//! *by value* — `Clone` + `Serialize`, never `Box<dyn>` or a lifetime.

use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

pub mod bspline_curve;
pub mod bspline_surface;
pub mod circle;
pub mod cone;
pub mod curve;
pub mod cylinder;
pub mod ellipse;
pub mod gregory;
pub mod hyperbola;
pub mod interval_bounds;
pub mod line;
pub mod offset;
pub mod parabola;
pub mod plane;
pub mod ruled;
pub mod sphere;
pub mod surface;
pub mod torus;

pub use bspline_curve::BSplineCurve;
pub use bspline_surface::BSplineSurface;
pub use circle::Circle;
pub use cone::ConicalSurface;
pub use curve::Curve;
pub use cylinder::CylindricalSurface;
pub use ellipse::Ellipse;
pub use gregory::GregorySurface;
pub use hyperbola::Hyperbola;
pub use line::Line;
pub use offset::OffsetSurface;
pub use parabola::Parabola;
pub use plane::Plane;
pub use ruled::RuledSurface;
pub use sphere::SphericalSurface;
pub use surface::Surface;
pub use torus::ToroidalSurface;

/// An owned 3D curve: one of the concrete [`Curve`]s, storable by value.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GeomCurve {
    /// A line.
    Line(Line),
    /// A circle.
    Circle(Circle),
    /// An ellipse.
    Ellipse(Ellipse),
    /// A parabola.
    Parabola(Parabola),
    /// A hyperbola.
    Hyperbola(Hyperbola),
    /// A B-spline/NURBS curve.
    BSpline(BSplineCurve),
}

impl GeomCurve {
    /// Wrap a [`Line`].
    #[inline]
    pub fn line(l: Line) -> Self {
        Self::Line(l)
    }

    /// Wrap a [`Circle`].
    #[inline]
    pub fn circle(c: Circle) -> Self {
        Self::Circle(c)
    }

    /// Wrap an [`Ellipse`].
    #[inline]
    pub fn ellipse(e: Ellipse) -> Self {
        Self::Ellipse(e)
    }

    /// Wrap a [`Parabola`].
    #[inline]
    pub fn parabola(p: Parabola) -> Self {
        Self::Parabola(p)
    }

    /// Wrap a [`Hyperbola`].
    #[inline]
    pub fn hyperbola(h: Hyperbola) -> Self {
        Self::Hyperbola(h)
    }

    /// Wrap a [`BSplineCurve`].
    #[inline]
    pub fn bspline(b: BSplineCurve) -> Self {
        Self::BSpline(b)
    }

    /// Convert/approximate this curve to a B-Spline curve.
    pub fn to_bspline(&self) -> BSplineCurve {
        match self {
            Self::BSpline(b) => b.clone(),
            _ => {
                let (t0, t1) = self.bounds();
                let n = 50;
                let mut poles = std::vec::Vec::new();
                let mut knots = std::vec::Vec::new();
                let mut mults = std::vec::Vec::new();
                for i in 0..n {
                    let t = t0 + (t1 - t0) * (i as f64) / ((n - 1) as f64);
                    poles.push(self.point(t));
                    knots.push(t);
                    if i == 0 || i == n - 1 {
                        mults.push(2);
                    } else {
                        mults.push(1);
                    }
                }
                BSplineCurve::new(1, poles, None, knots, mults)
            }
        }
    }
}

impl Curve for GeomCurve {
    fn point(&self, u: f64) -> Pnt {
        match self {
            Self::Line(l) => l.point(u),
            Self::Circle(c) => c.point(u),
            Self::Ellipse(e) => e.point(u),
            Self::Parabola(p) => p.point(u),
            Self::Hyperbola(h) => h.point(u),
            Self::BSpline(b) => b.point(u),
        }
    }

    fn d1(&self, u: f64) -> (Pnt, Vec) {
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

    fn transformed(&self, t: &Trsf) -> Self {
        match self {
            Self::Line(l) => Self::Line(l.transformed(t)),
            Self::Circle(c) => Self::Circle(c.transformed(t)),
            Self::Ellipse(e) => Self::Ellipse(e.transformed(t)),
            Self::Parabola(p) => Self::Parabola(p.transformed(t)),
            Self::Hyperbola(h) => Self::Hyperbola(h.transformed(t)),
            Self::BSpline(b) => Self::BSpline(b.transformed(t)),
        }
    }
}

/// An owned 3D surface: one of the concrete [`Surface`]s, storable by value.
// `BSplineSurface` and `GregorySurface` are larger than the analytic variants;
// topology stores `GeomSurface` by value (the kernel's no-`Box<dyn>` rule), so
// the size spread is accepted rather than boxing the big variants.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GeomSurface {
    /// A plane.
    Plane(Plane),
    /// A cylindrical surface.
    Cylinder(CylindricalSurface),
    /// A conical surface.
    Cone(ConicalSurface),
    /// A spherical surface.
    Sphere(SphericalSurface),
    /// A toroidal surface.
    Torus(ToroidalSurface),
    /// A B-spline/NURBS surface.
    BSpline(BSplineSurface),
    /// A Gregory patch.
    Gregory(GregorySurface),
    /// An offset surface.
    Offset(OffsetSurface),
    /// A ruled surface.
    Ruled(RuledSurface),
}

impl GeomSurface {
    /// Wrap a [`Plane`].
    #[inline]
    pub fn plane(p: Plane) -> Self {
        Self::Plane(p)
    }

    /// Wrap a [`CylindricalSurface`].
    #[inline]
    pub fn cylinder(c: CylindricalSurface) -> Self {
        Self::Cylinder(c)
    }

    /// Wrap a [`ConicalSurface`].
    #[inline]
    pub fn cone(c: ConicalSurface) -> Self {
        Self::Cone(c)
    }

    /// Wrap a [`SphericalSurface`].
    #[inline]
    pub fn sphere(s: SphericalSurface) -> Self {
        Self::Sphere(s)
    }

    /// Wrap a [`ToroidalSurface`].
    #[inline]
    pub fn torus(t: ToroidalSurface) -> Self {
        Self::Torus(t)
    }

    /// Wrap a [`BSplineSurface`].
    #[inline]
    pub fn bspline(b: BSplineSurface) -> Self {
        Self::BSpline(b)
    }

    /// Wrap a [`GregorySurface`].
    #[inline]
    pub fn gregory(g: GregorySurface) -> Self {
        Self::Gregory(g)
    }

    /// Wrap an [`OffsetSurface`].
    #[inline]
    pub fn offset(o: OffsetSurface) -> Self {
        Self::Offset(o)
    }

    /// Wrap a [`RuledSurface`].
    #[inline]
    pub fn ruled(r: RuledSurface) -> Self {
        Self::Ruled(r)
    }

    /// The point and both first partial derivatives at `(u, v)`.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        match self {
            Self::Plane(p) => p.d1(u, v),
            Self::Cylinder(c) => c.d1(u, v),
            Self::Cone(c) => c.d1(u, v),
            Self::Sphere(s) => s.d1(u, v),
            Self::Torus(t) => t.d1(u, v),
            Self::BSpline(b) => b.d1(u, v),
            Self::Gregory(g) => g.d1(u, v),
            Self::Offset(o) => o.d1(u, v),
            Self::Ruled(r) => r.d1(u, v),
        }
    }

    /// Convert/approximate this surface to a B-Spline surface.
    pub fn to_bspline(&self) -> Option<BSplineSurface> {
        match self {
            Self::BSpline(b) => Some(b.clone()),
            _ => {
                let (u0, u1, v0, v1) = self.bounds();
                let u0 = if u0.is_infinite() { -100.0 } else { u0 };
                let u1 = if u1.is_infinite() { 100.0 } else { u1 };
                let v0 = if v0.is_infinite() { -100.0 } else { v0 };
                let v1 = if v1.is_infinite() { 100.0 } else { v1 };

                let nu = 30;
                let nv = 30;
                let mut poles = std::vec::Vec::new();
                let mut u_knots = std::vec::Vec::new();
                let mut u_mults = std::vec::Vec::new();
                let mut v_knots = std::vec::Vec::new();
                let mut v_mults = std::vec::Vec::new();

                for i in 0..nu {
                    let u = u0 + (u1 - u0) * (i as f64) / ((nu - 1) as f64);
                    u_knots.push(u);
                    if i == 0 || i == nu - 1 {
                        u_mults.push(2);
                    } else {
                        u_mults.push(1);
                    }

                    let mut row = std::vec::Vec::new();
                    for j in 0..nv {
                        let v = v0 + (v1 - v0) * (j as f64) / ((nv - 1) as f64);
                        row.push(self.point(u, v));
                    }
                    poles.push(row);
                }

                for j in 0..nv {
                    let v = v0 + (v1 - v0) * (j as f64) / ((nv - 1) as f64);
                    v_knots.push(v);
                    if j == 0 || j == nv - 1 {
                        v_mults.push(2);
                    } else {
                        v_mults.push(1);
                    }
                }

                Some(BSplineSurface::new(
                    1, 1, poles, None, u_knots, u_mults, v_knots, v_mults,
                ))
            }
        }
    }
}

impl Surface for GeomSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        match self {
            Self::Plane(p) => p.point(u, v),
            Self::Cylinder(c) => c.point(u, v),
            Self::Cone(c) => c.point(u, v),
            Self::Sphere(s) => s.point(u, v),
            Self::Torus(t) => t.point(u, v),
            Self::BSpline(b) => b.point(u, v),
            Self::Gregory(g) => g.point(u, v),
            Self::Offset(o) => o.point(u, v),
            Self::Ruled(r) => r.point(u, v),
        }
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        match self {
            Self::Plane(p) => p.bounds(),
            Self::Cylinder(c) => c.bounds(),
            Self::Cone(c) => c.bounds(),
            Self::Sphere(s) => s.bounds(),
            Self::Torus(t) => t.bounds(),
            Self::BSpline(b) => b.bounds(),
            Self::Gregory(g) => g.bounds(),
            Self::Offset(o) => o.bounds(),
            Self::Ruled(r) => r.bounds(),
        }
    }

    fn is_uclosed(&self) -> bool {
        match self {
            Self::Plane(p) => p.is_uclosed(),
            Self::Cylinder(c) => c.is_uclosed(),
            Self::Cone(c) => c.is_uclosed(),
            Self::Sphere(s) => s.is_uclosed(),
            Self::Torus(t) => t.is_uclosed(),
            Self::BSpline(b) => b.is_uclosed(),
            Self::Gregory(_) | Self::Offset(_) | Self::Ruled(_) => false,
        }
    }

    fn is_vclosed(&self) -> bool {
        match self {
            Self::Plane(p) => p.is_vclosed(),
            Self::Cylinder(c) => c.is_vclosed(),
            Self::Cone(c) => c.is_vclosed(),
            Self::Sphere(s) => s.is_vclosed(),
            Self::Torus(t) => t.is_vclosed(),
            Self::BSpline(b) => b.is_vclosed(),
            Self::Gregory(_) | Self::Offset(_) | Self::Ruled(_) => false,
        }
    }

    fn transformed(&self, t: &Trsf) -> Self {
        match self {
            Self::Plane(p) => Self::Plane(p.transformed(t)),
            Self::Cylinder(c) => Self::Cylinder(c.transformed(t)),
            Self::Cone(c) => Self::Cone(c.transformed(t)),
            Self::Sphere(s) => Self::Sphere(s.transformed(t)),
            Self::Torus(tor) => Self::Torus(tor.transformed(t)),
            Self::BSpline(b) => Self::BSpline(b.transformed(t)),
            Self::Gregory(g) => Self::Gregory(g.transformed(t)),
            Self::Offset(o) => Self::Offset(o.transformed(t)),
            Self::Ruled(r) => Self::Ruled(r.transformed(t)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax1, Dir};

    #[test]
    fn curve_enum_transforms() {
        let c = GeomCurve::circle(Circle::new(
            openrcad_foundation::Ax3::new(Pnt::origin(), Dir::dz()),
            1.0,
        ));
        // Rotate the circle 90° about X: its plane normal Z -> Y.
        let r = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dx()),
            core::f64::consts::FRAC_PI_2,
        );
        let c2 = c.transformed(&r);
        match c2 {
            GeomCurve::Circle(cc) => assert!(cc.axis().is_parallel(&Dir::dy(), 1e-9)),
            _ => panic!("expected a circle"),
        }
    }

    #[test]
    fn surface_enum_evaluates() {
        let s = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        assert_eq!(s.point(2.0, 3.0), Pnt::new(2.0, 3.0, 0.0));
    }
}
