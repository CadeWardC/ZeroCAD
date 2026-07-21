//! Exact affine reparameterization of a bounded 3D curve.

use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::{Curve, GeomCurve};

/// A curve evaluated through an affine parameter map.
///
/// This is useful when two exact rail curves describe the right geometry but
/// use different parameter intervals. `domain_first` and `domain_last` become
/// the public bounds while the corresponding points are read from
/// `target_first` and `target_last` on the wrapped curve. No sampling or
/// tessellated approximation is introduced.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReparametrizedCurve {
    curve: Box<GeomCurve>,
    domain_first: f64,
    domain_last: f64,
    target_first: f64,
    target_last: f64,
}

impl ReparametrizedCurve {
    /// Construct an exact affine parameter mapping.
    ///
    /// Returns `None` for non-finite or collapsed intervals so invalid geometry
    /// can be rejected before candidate topology is built.
    pub fn new(
        curve: GeomCurve,
        domain_first: f64,
        domain_last: f64,
        target_first: f64,
        target_last: f64,
    ) -> Option<Self> {
        let parameters = [domain_first, domain_last, target_first, target_last];
        if parameters.iter().any(|parameter| !parameter.is_finite())
            || domain_first == domain_last
            || target_first == target_last
        {
            return None;
        }
        Some(Self {
            curve: Box::new(curve),
            domain_first,
            domain_last,
            target_first,
            target_last,
        })
    }

    /// The exact wrapped curve.
    #[inline]
    pub fn curve(&self) -> &GeomCurve {
        &self.curve
    }

    /// Map a public parameter into the wrapped curve's parameter space.
    #[inline]
    pub fn target_parameter(&self, parameter: f64) -> f64 {
        self.target_first + (parameter - self.domain_first) * self.parameter_scale()
    }

    #[inline]
    fn parameter_scale(&self) -> f64 {
        (self.target_last - self.target_first) / (self.domain_last - self.domain_first)
    }
}

impl Curve for ReparametrizedCurve {
    fn point(&self, parameter: f64) -> Pnt {
        self.curve.point(self.target_parameter(parameter))
    }

    fn d1(&self, parameter: f64) -> (Pnt, Vec) {
        let (point, derivative) = self.curve.d1(self.target_parameter(parameter));
        (point, derivative * self.parameter_scale())
    }

    fn bounds(&self) -> (f64, f64) {
        (self.domain_first, self.domain_last)
    }

    fn is_closed(&self) -> bool {
        let (first, last) = self.curve.bounds();
        self.curve.is_closed()
            && ((self.target_first == first && self.target_last == last)
                || (self.target_first == last && self.target_last == first))
    }

    // A bounded affine wrapper is intentionally not periodic. This prevents a
    // trimmed rail from being extended onto another periodic branch by callers.
    fn is_periodic(&self) -> bool {
        false
    }

    fn transformed(&self, transform: &Trsf) -> Self {
        Self {
            curve: Box::new(self.curve.transformed(transform)),
            domain_first: self.domain_first,
            domain_last: self.domain_last,
            target_first: self.target_first,
            target_last: self.target_last,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Circle, Line};
    use openrcad_foundation::{Ax3, Dir};

    #[test]
    fn affine_mapping_preserves_points_and_scales_derivatives() {
        let base = GeomCurve::line(Line::from_point_dir(Pnt::new(2.0, 3.0, 5.0), Dir::dx()));
        let curve = ReparametrizedCurve::new(base, 10.0, 14.0, -3.0, 5.0).unwrap();
        let (point, derivative) = curve.d1(11.0);
        assert_eq!(point, Pnt::new(1.0, 3.0, 5.0));
        assert_eq!(derivative, Vec::new(2.0, 0.0, 0.0));
    }

    #[test]
    fn reversed_full_closed_curve_stays_closed_but_not_periodic() {
        let circle = GeomCurve::circle(Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0));
        let (first, last) = circle.bounds();
        let curve = ReparametrizedCurve::new(circle, 4.0, 9.0, last, first).unwrap();
        assert!(curve.is_closed());
        assert!(!curve.is_periodic());
        let arithmetic_tolerance = 64.0 * f64::EPSILON * 2.0;
        assert!(curve.point(4.0).distance(&curve.point(9.0)) <= arithmetic_tolerance);
    }

    #[test]
    fn rejects_collapsed_or_non_finite_intervals() {
        let line = GeomCurve::line(Line::from_point_dir(Pnt::origin(), Dir::dx()));
        assert!(ReparametrizedCurve::new(line.clone(), 1.0, 1.0, 0.0, 2.0).is_none());
        assert!(ReparametrizedCurve::new(line, 0.0, 1.0, 2.0, f64::NAN).is_none());
    }
}
