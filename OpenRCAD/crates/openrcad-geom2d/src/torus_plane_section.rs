//! Exact torus-parameter pcurve for an axis-parallel plane section.

use core::f64::consts::TAU;

use openrcad_foundation::{Pnt2d, Vec2d};
use serde::{Deserialize, Serialize};

use crate::Curve2d;

/// Exact `(u, v)` coordinates of one regular-torus/plane section branch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TorusPlaneSection2d {
    normal_angle: f64,
    major_radius: f64,
    minor_radius: f64,
    signed_offset: f64,
    positive_branch: bool,
}

impl TorusPlaneSection2d {
    pub fn new(
        normal_angle: f64,
        major_radius: f64,
        minor_radius: f64,
        signed_offset: f64,
        positive_branch: bool,
    ) -> Option<Self> {
        let values = [normal_angle, major_radius, minor_radius, signed_offset];
        if values.iter().any(|value| !value.is_finite())
            || major_radius <= 0.0
            || minor_radius <= 0.0
            || signed_offset.abs() >= major_radius - minor_radius
        {
            return None;
        }
        Some(Self {
            normal_angle,
            major_radius,
            minor_radius,
            signed_offset,
            positive_branch,
        })
    }

    fn point_and_derivative(&self, v: f64) -> (Pnt2d, Vec2d) {
        let branch = if self.positive_branch { 1.0 } else { -1.0 };
        let (cosine, sine) = (v.cos(), v.sin());
        let radial = self.major_radius + self.minor_radius * cosine;
        let lateral_squared = radial * radial - self.signed_offset * self.signed_offset;
        let lateral = lateral_squared.max(0.0).sqrt();
        let relative_angle = (branch * lateral).atan2(self.signed_offset);
        let lateral_derivative = if lateral > f64::MIN_POSITIVE {
            radial * (-self.minor_radius * sine) / lateral
        } else {
            0.0
        };
        let u_derivative = if radial.abs() > f64::MIN_POSITIVE {
            self.signed_offset * branch * lateral_derivative / (radial * radial)
        } else {
            0.0
        };
        (
            Pnt2d::new(self.normal_angle + relative_angle, v),
            Vec2d::new(u_derivative, 1.0),
        )
    }
}

impl Curve2d for TorusPlaneSection2d {
    fn point(&self, v: f64) -> Pnt2d {
        self.point_and_derivative(v).0
    }

    fn d1(&self, v: f64) -> (Pnt2d, Vec2d) {
        self.point_and_derivative(v)
    }

    fn bounds(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn is_closed(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn section_pcurve_is_finite_and_uses_v_as_its_parameter() {
        let curve = TorusPlaneSection2d::new(0.7, 4.0, 1.0, 0.5, true).unwrap();
        for v in [0.0, 0.9, 2.7, 5.8, TAU] {
            let (point, derivative) = curve.d1(v);
            assert_eq!(point.y(), v);
            assert!(point.x().is_finite());
            assert!(derivative.x().is_finite());
            assert_eq!(derivative.y(), 1.0);
        }
    }
}
