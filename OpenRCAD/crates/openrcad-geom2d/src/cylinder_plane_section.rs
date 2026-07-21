//! Exact cylinder-parameter curve for an oblique planar section.

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::Curve2d;

/// An ellipse lying on a cylinder, expressed directly in the cylinder's
/// unwrapped `(u, v)` parameter space.
///
/// For the ellipse parameter `t`, its radial projection is a circle, so
/// `u(t) = u_offset +/- t`. Its axial coordinate is the affine sinusoid
/// `v(t) = v_offset + v_cosine*cos(t) + v_sine*sin(t)`. Keeping this analytic
/// map avoids subtracting far-origin world coordinates during pcurve fitting.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CylinderPlaneSection2d {
    u_offset: f64,
    reverse_u: bool,
    v_offset: f64,
    v_cosine: f64,
    v_sine: f64,
}

impl CylinderPlaneSection2d {
    pub fn new(
        u_offset: f64,
        reverse_u: bool,
        v_offset: f64,
        v_cosine: f64,
        v_sine: f64,
    ) -> Option<Self> {
        let values = [u_offset, v_offset, v_cosine, v_sine];
        values
            .iter()
            .all(|value| value.is_finite())
            .then_some(Self {
                u_offset,
                reverse_u,
                v_offset,
                v_cosine,
                v_sine,
            })
    }

    /// Apply the exact distance-coordinate scaling induced by a uniform 3D
    /// model scale. Cylinder `u` stays angular while `v` scales linearly.
    pub fn scaled_v(self, scale: f64) -> Option<Self> {
        (scale.is_finite() && scale > 0.0).then_some(Self {
            u_offset: self.u_offset,
            reverse_u: self.reverse_u,
            v_offset: self.v_offset * scale,
            v_cosine: self.v_cosine * scale,
            v_sine: self.v_sine * scale,
        })
    }

    fn point_and_derivative(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        let direction = if self.reverse_u { -1.0 } else { 1.0 };
        let (sine, cosine) = parameter.sin_cos();
        (
            Pnt2d::new(
                self.u_offset + direction * parameter,
                self.v_offset + self.v_cosine * cosine + self.v_sine * sine,
            ),
            Vec2d::new(direction, -self.v_cosine * sine + self.v_sine * cosine),
        )
    }
}

impl Curve2d for CylinderPlaneSection2d {
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
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluates_unwrapped_angle_and_axial_sinusoid_exactly() {
        let section = CylinderPlaneSection2d::new(0.3, false, 7.0, 2.0, -3.0).unwrap();
        let parameter = 0.8_f64;
        let (point, derivative) = section.d1(parameter);
        assert_eq!(point.x(), 1.1);
        assert_eq!(
            point.y(),
            7.0 + 2.0 * parameter.cos() - 3.0 * parameter.sin()
        );
        assert_eq!(derivative.x(), 1.0);
        assert_eq!(
            derivative.y(),
            -2.0 * parameter.sin() - 3.0 * parameter.cos()
        );

        let reversed = CylinderPlaneSection2d::new(0.3, true, 7.0, 2.0, -3.0).unwrap();
        assert_eq!(reversed.point(parameter).x(), -0.5);
    }

    #[test]
    fn uniform_model_scale_changes_only_the_axial_coordinate() {
        let section = CylinderPlaneSection2d::new(-0.4, false, 3.0, 2.0, 1.0).unwrap();
        let scaled = section.scaled_v(1.0e3).unwrap();
        let parameter = 1.7;
        assert_eq!(scaled.point(parameter).x(), section.point(parameter).x());
        let expected_v = section.point(parameter).y() * 1.0e3;
        assert!(
            (scaled.point(parameter).y() - expected_v).abs()
                <= 8.0 * f64::EPSILON * expected_v.abs()
        );
    }
}
