//! Bounded 2D curve spans with caller-owned provenance.

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::{Curve2d, GeomCurve2d};

/// Stable curve-family classification used by arrangement diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CurveKind2d {
    Line,
    Circle,
    Ellipse,
    Parabola,
    Hyperbola,
    BSpline,
    TorusPlaneSection,
    PlaneTorusSection,
    CylinderPlaneSection,
}

impl From<&GeomCurve2d> for CurveKind2d {
    fn from(curve: &GeomCurve2d) -> Self {
        match curve {
            GeomCurve2d::Line(_) => Self::Line,
            GeomCurve2d::Circle(_) => Self::Circle,
            GeomCurve2d::Ellipse(_) => Self::Ellipse,
            GeomCurve2d::Parabola(_) => Self::Parabola,
            GeomCurve2d::Hyperbola(_) => Self::Hyperbola,
            GeomCurve2d::BSpline(_) => Self::BSpline,
            GeomCurve2d::TorusPlaneSection(_) => Self::TorusPlaneSection,
            GeomCurve2d::PlaneTorusSection(_) => Self::PlaneTorusSection,
            GeomCurve2d::CylinderPlaneSection(_) => Self::CylinderPlaneSection,
        }
    }
}

/// A bounded interval of an owned analytic curve. `provenance` is deliberately
/// generic: sketch/entity ownership belongs to the calling document layer,
/// while the geometry kernel preserves it through splitting and DCEL walks.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CurveSpan<P> {
    pub curve: GeomCurve2d,
    pub first: f64,
    pub last: f64,
    pub provenance: P,
}

impl<P> CurveSpan<P> {
    pub fn new(curve: GeomCurve2d, first: f64, last: f64, provenance: P) -> Self {
        Self {
            curve,
            first,
            last,
            provenance,
        }
    }

    pub fn kind(&self) -> CurveKind2d {
        (&self.curve).into()
    }

    pub fn start(&self) -> Pnt2d {
        self.curve.point(self.first)
    }

    pub fn end(&self) -> Pnt2d {
        self.curve.point(self.last)
    }

    pub fn start_tangent(&self) -> Vec2d {
        let tangent = self.curve.d1(self.first).1;
        if self.last >= self.first {
            tangent
        } else {
            Vec2d::new(-tangent.x(), -tangent.y())
        }
    }

    pub fn end_tangent(&self) -> Vec2d {
        let tangent = self.curve.d1(self.last).1;
        if self.last >= self.first {
            tangent
        } else {
            Vec2d::new(-tangent.x(), -tangent.y())
        }
    }

    pub fn is_finite(&self) -> bool {
        self.first.is_finite()
            && self.last.is_finite()
            && [self.start(), self.end()]
                .iter()
                .all(|point| point.x().is_finite() && point.y().is_finite())
    }

    pub fn parameter_length(&self) -> f64 {
        (self.last - self.first).abs()
    }

    pub fn with_bounds<Q>(&self, first: f64, last: f64, provenance: Q) -> CurveSpan<Q> {
        CurveSpan::new(self.curve.clone(), first, last, provenance)
    }

    pub fn map_provenance<Q>(self, map: impl FnOnce(P) -> Q) -> CurveSpan<Q> {
        CurveSpan::new(self.curve, self.first, self.last, map(self.provenance))
    }

    pub fn reversed(mut self) -> Self {
        core::mem::swap(&mut self.first, &mut self.last);
        self
    }
}

impl<P: Clone> CurveSpan<P> {
    pub fn subspan(&self, first: f64, last: f64) -> Self {
        self.with_bounds(first, last, self.provenance.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Line2d;
    use openrcad_foundation::{Dir2d, Pnt2d};

    #[test]
    fn split_and_reverse_preserve_provenance() {
        let span = CurveSpan::new(
            GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::origin(), Dir2d::dx())),
            0.0,
            10.0,
            42_u32,
        );
        let split = span.subspan(2.0, 7.0);
        assert_eq!(split.start(), Pnt2d::new(2.0, 0.0));
        assert_eq!(split.end(), Pnt2d::new(7.0, 0.0));
        assert_eq!(split.provenance, 42);
        let reversed = split.reversed();
        assert_eq!(reversed.start(), Pnt2d::new(7.0, 0.0));
        assert_eq!(reversed.end(), Pnt2d::new(2.0, 0.0));
        assert_eq!(reversed.provenance, 42);
    }
}
