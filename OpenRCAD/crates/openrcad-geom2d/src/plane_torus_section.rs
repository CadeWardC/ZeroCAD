//! Exact planar projection of a regular torus/axis-parallel-plane section.

use core::f64::consts::TAU;

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::Curve2d;

/// One closed torus-section branch expressed in a plane's `(u, v)` frame.
///
/// The curve parameter is the torus tube angle. `lateral` and `axis` are the
/// two unit 3D directions projected into the carrying plane coordinates, so
/// evaluation stays analytic and does not depend on iterative 3D projection.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaneTorusSection2d {
    center: Pnt2d,
    lateral: Vec2d,
    axis: Vec2d,
    major_radius: f64,
    minor_radius: f64,
    signed_offset: f64,
    positive_branch: bool,
}

impl PlaneTorusSection2d {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        center: Pnt2d,
        lateral: Vec2d,
        axis: Vec2d,
        major_radius: f64,
        minor_radius: f64,
        signed_offset: f64,
        positive_branch: bool,
    ) -> Option<Self> {
        let values = [
            center.x(),
            center.y(),
            lateral.x(),
            lateral.y(),
            axis.x(),
            axis.y(),
            major_radius,
            minor_radius,
            signed_offset,
        ];
        if values.iter().any(|value| !value.is_finite())
            || major_radius <= 0.0
            || minor_radius <= 0.0
            || signed_offset.abs() >= major_radius - minor_radius
        {
            return None;
        }
        Some(Self {
            center,
            lateral,
            axis,
            major_radius,
            minor_radius,
            signed_offset,
            positive_branch,
        })
    }

    fn point_and_derivative(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        let branch = if self.positive_branch { 1.0 } else { -1.0 };
        let (cosine, sine) = (parameter.cos(), parameter.sin());
        let radial = self.major_radius + self.minor_radius * cosine;
        let lateral_squared = radial * radial - self.signed_offset * self.signed_offset;
        let lateral_distance = lateral_squared.max(0.0).sqrt();
        let radial_derivative = -self.minor_radius * sine;
        let lateral_derivative = if lateral_distance > f64::MIN_POSITIVE {
            radial * radial_derivative / lateral_distance
        } else {
            0.0
        };
        let lateral_coordinate = branch * lateral_distance;
        let axis_coordinate = self.minor_radius * sine;
        let point = Pnt2d::new(
            self.center.x()
                + self.lateral.x() * lateral_coordinate
                + self.axis.x() * axis_coordinate,
            self.center.y()
                + self.lateral.y() * lateral_coordinate
                + self.axis.y() * axis_coordinate,
        );
        let derivative = Vec2d::new(
            self.lateral.x() * branch * lateral_derivative
                + self.axis.x() * self.minor_radius * cosine,
            self.lateral.y() * branch * lateral_derivative
                + self.axis.y() * self.minor_radius * cosine,
        );
        (point, derivative)
    }
}

impl Curve2d for PlaneTorusSection2d {
    fn point(&self, parameter: f64) -> Pnt2d {
        self.point_and_derivative(parameter).0
    }

    fn d1(&self, parameter: f64) -> (Pnt2d, Vec2d) {
        self.point_and_derivative(parameter)
    }

    fn bounds(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn is_closed(&self) -> bool {
        true
    }

    fn is_periodic(&self) -> bool {
        true
    }

    fn period(&self) -> f64 {
        TAU
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projected_section_is_closed_and_finite() {
        let curve = PlaneTorusSection2d::new(
            Pnt2d::new(3.0, -2.0),
            Vec2d::new(1.0, 0.0),
            Vec2d::new(0.0, 1.0),
            4.0,
            1.0,
            0.5,
            true,
        )
        .unwrap();
        assert!(curve.point(0.0).distance(&curve.point(TAU)) <= 64.0 * f64::EPSILON * 5.0);
        for parameter in [0.0, 0.9, 2.7, 5.8, TAU] {
            let (point, derivative) = curve.d1(parameter);
            assert!(point.x().is_finite() && point.y().is_finite());
            assert!(derivative.x().is_finite() && derivative.y().is_finite());
        }
    }
}
