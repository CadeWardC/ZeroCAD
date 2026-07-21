//! Exact sphere-parameter curve for a three-dimensional great circle.

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::Curve2d;

/// A great circle expressed directly in a sphere's unwrapped `(u, v)` space.
///
/// The corresponding unit radial vector is
/// `cosine * cos(t) + sine * sin(t)` in the sphere frame. Storing those local
/// coefficients avoids subtracting far-origin world coordinates when building
/// or validating spherical corner pcurves.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SphereGreatCircle2d {
    cosine: [f64; 3],
    sine: [f64; 3],
    reference_u: f64,
}

impl SphereGreatCircle2d {
    /// Construct the exact UV map, using `reference_parameter` to select one
    /// deterministic unwrapped longitude branch.
    pub fn new(cosine: [f64; 3], sine: [f64; 3], reference_parameter: f64) -> Option<Self> {
        if !cosine
            .iter()
            .chain(&sine)
            .chain(std::iter::once(&reference_parameter))
            .all(|value| value.is_finite())
        {
            return None;
        }
        let (sin_t, cos_t) = reference_parameter.sin_cos();
        let x = cosine[0] * cos_t + sine[0] * sin_t;
        let y = cosine[1] * cos_t + sine[1] * sin_t;
        (x.hypot(y) > f64::EPSILON).then_some(Self {
            cosine,
            sine,
            reference_u: y.atan2(x),
        })
    }

    fn local(&self, parameter: f64) -> ([f64; 3], [f64; 3]) {
        let (sin_t, cos_t) = parameter.sin_cos();
        let mut point = [0.0; 3];
        let mut derivative = [0.0; 3];
        for coordinate in 0..3 {
            point[coordinate] = self.cosine[coordinate] * cos_t + self.sine[coordinate] * sin_t;
            derivative[coordinate] =
                -self.cosine[coordinate] * sin_t + self.sine[coordinate] * cos_t;
        }
        (point, derivative)
    }

    fn point_and_derivative(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        let (point, derivative) = self.local(parameter);
        let raw_u = point[1].atan2(point[0]);
        let u = raw_u
            + ((self.reference_u - raw_u) / (2.0 * core::f64::consts::PI)).round()
                * (2.0 * core::f64::consts::PI);
        let z = point[2].clamp(-1.0, 1.0);
        let radial_squared = (point[0] * point[0] + point[1] * point[1]).max(f64::MIN_POSITIVE);
        let du = (point[0] * derivative[1] - point[1] * derivative[0]) / radial_squared;
        let dv = derivative[2] / (1.0 - z * z).max(f64::MIN_POSITIVE).sqrt();
        (Pnt2d::new(u, z.asin()), Vec2d::new(du, dv))
    }
}

impl Curve2d for SphereGreatCircle2d {
    fn point(&self, parameter: f64) -> Pnt2d {
        self.point_and_derivative(parameter).0
    }

    fn d1(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        self.point_and_derivative(parameter)
    }

    fn bounds(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    fn is_closed(&self) -> bool {
        true
    }

    fn is_periodic(&self) -> bool {
        true
    }

    fn period(&self) -> f64 {
        2.0 * core::f64::consts::PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_an_oblique_great_circle_without_world_coordinates() {
        let inv_sqrt_two = core::f64::consts::FRAC_1_SQRT_2;
        let curve =
            SphereGreatCircle2d::new([inv_sqrt_two, 0.0, inv_sqrt_two], [0.0, 1.0, 0.0], 0.0)
                .unwrap();
        let quarter = curve.point(core::f64::consts::FRAC_PI_2);
        assert!((quarter.x() - core::f64::consts::FRAC_PI_2).abs() <= 8.0 * f64::EPSILON);
        assert!(quarter.y().abs() <= 8.0 * f64::EPSILON);
        let start = curve.point(0.0);
        assert!(start.x().abs() <= 8.0 * f64::EPSILON);
        assert!((start.y() - core::f64::consts::FRAC_PI_4).abs() <= 8.0 * f64::EPSILON);
    }
}
