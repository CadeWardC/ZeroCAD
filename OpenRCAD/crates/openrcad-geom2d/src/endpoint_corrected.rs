//! Topology-endpoint correction for an otherwise analytic pcurve.

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::{Curve2d, GeomCurve2d};

/// A bounded pcurve whose analytic interior receives a linear, endpoint-exact
/// UV correction.
///
/// This is used only after sewing chooses shared 3D topology vertices. At a
/// far origin, evaluating an analytic 3D conic at its parameter can differ by
/// a few floating-point ulps from that shared vertex. The tiny UV correction
/// keeps adjacent coedges exactly contiguous while preserving the analytic
/// parameterization throughout the span.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EndpointCorrectedCurve2d {
    base: Box<GeomCurve2d>,
    first: f64,
    last: f64,
    first_correction: Vec2d,
    last_correction: Vec2d,
}

impl EndpointCorrectedCurve2d {
    pub fn new(
        base: GeomCurve2d,
        first: f64,
        last: f64,
        first_correction: Vec2d,
        last_correction: Vec2d,
    ) -> Option<Self> {
        let values = [
            first,
            last,
            first_correction.x(),
            first_correction.y(),
            last_correction.x(),
            last_correction.y(),
        ];
        (values.iter().all(|value| value.is_finite()) && first != last).then_some(Self {
            base: Box::new(base),
            first,
            last,
            first_correction,
            last_correction,
        })
    }

    #[inline]
    pub fn base(&self) -> &GeomCurve2d {
        &self.base
    }

    fn correction(&self, parameter: f64) -> (Vec2d, Vec2d) {
        let inverse_span = 1.0 / (self.last - self.first);
        let fraction = (parameter - self.first) * inverse_span;
        let delta = self
            .first_correction
            .multiplied(1.0 - fraction)
            .added(&self.last_correction.multiplied(fraction));
        let derivative = self
            .last_correction
            .subtracted(&self.first_correction)
            .multiplied(inverse_span);
        (delta, derivative)
    }
}

impl Curve2d for EndpointCorrectedCurve2d {
    fn point(&self, parameter: f64) -> Pnt2d {
        let point = self.base.point(parameter);
        let (correction, _) = self.correction(parameter);
        correction + point
    }

    fn d1(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        let (point, derivative) = self.base.d1(parameter);
        let (correction, correction_derivative) = self.correction(parameter);
        (correction + point, derivative.added(&correction_derivative))
    }

    fn bounds(&self) -> (f64, f64) {
        (self.first, self.last)
    }

    fn is_closed(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Line2d;
    use openrcad_foundation::{Dir2d, Pnt2d};

    #[test]
    fn preserves_the_interior_shape_and_hits_both_corrected_endpoints() {
        let base = GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::ORIGIN, Dir2d::DX));
        let corrected = EndpointCorrectedCurve2d::new(
            base,
            0.0,
            2.0,
            Vec2d::new(0.0, 1.0),
            Vec2d::new(0.0, 3.0),
        )
        .unwrap();
        assert_eq!(corrected.point(0.0), Pnt2d::new(0.0, 1.0));
        assert_eq!(corrected.point(1.0), Pnt2d::new(1.0, 2.0));
        assert_eq!(corrected.point(2.0), Pnt2d::new(2.0, 3.0));
    }
}
