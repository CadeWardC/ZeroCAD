//! Exact section of a regular torus by an axis-parallel plane.

use core::f64::consts::TAU;

use openrcad_foundation::{Ax3, Dir, Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::Curve;

/// One of the two closed branches produced when a plane parallel to a regular
/// torus axis lies strictly inside the torus' inner radius.
///
/// Parameter `v` is the torus tube angle.  If `n` is the plane normal and
/// `q = axis × n`, the section is
/// `C + d n ± sqrt((R + r cos(v))² - d²) q + r sin(v) axis`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TorusPlaneSection {
    position: Ax3,
    major_radius: f64,
    minor_radius: f64,
    plane_normal: Dir,
    signed_offset: f64,
    positive_branch: bool,
}

impl TorusPlaneSection {
    pub const fn new(
        position: Ax3,
        major_radius: f64,
        minor_radius: f64,
        plane_normal: Dir,
        signed_offset: f64,
        positive_branch: bool,
    ) -> Self {
        Self {
            position,
            major_radius,
            minor_radius,
            plane_normal,
            signed_offset,
            positive_branch,
        }
    }

    pub const fn position(&self) -> Ax3 {
        self.position
    }

    pub const fn major_radius(&self) -> f64 {
        self.major_radius
    }

    pub const fn minor_radius(&self) -> f64 {
        self.minor_radius
    }

    pub const fn plane_normal(&self) -> Dir {
        self.plane_normal
    }

    pub const fn signed_offset(&self) -> f64 {
        self.signed_offset
    }

    pub const fn positive_branch(&self) -> bool {
        self.positive_branch
    }

    fn point_and_derivative(&self, v: f64) -> (Pnt, Vec) {
        let axis = Vec::from_dir(self.position.direction());
        let normal = Vec::from_dir(self.plane_normal);
        let lateral = Vec::from_dir(self.position.direction().cross(&self.plane_normal));
        let branch = if self.positive_branch { 1.0 } else { -1.0 };
        let (cosine, sine) = (v.cos(), v.sin());
        let radial = self.major_radius + self.minor_radius * cosine;
        let lateral_squared = radial * radial - self.signed_offset * self.signed_offset;
        // Construction rejects offsets that can touch the inner radius, so the
        // radicand remains strictly positive throughout the full period.
        let lateral_distance = lateral_squared.max(0.0).sqrt();
        let point = self.position.location()
            + normal * self.signed_offset
            + lateral * (branch * lateral_distance)
            + axis * (self.minor_radius * sine);
        let radial_derivative = -self.minor_radius * sine;
        let lateral_derivative = if lateral_distance > f64::MIN_POSITIVE {
            radial * radial_derivative / lateral_distance
        } else {
            0.0
        };
        let derivative =
            lateral * (branch * lateral_derivative) + axis * (self.minor_radius * cosine);
        (point, derivative)
    }
}

impl Curve for TorusPlaneSection {
    fn point(&self, v: f64) -> Pnt {
        self.point_and_derivative(v).0
    }

    fn d1(&self, v: f64) -> (Pnt, Vec) {
        self.point_and_derivative(v)
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

    fn transformed(&self, transform: &Trsf) -> Self {
        let scale = transform.scale_factor().abs();
        Self::new(
            self.position.transformed(transform),
            self.major_radius * scale,
            self.minor_radius * scale,
            transform.transform_dir(&self.plane_normal),
            self.signed_offset * scale,
            self.positive_branch,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Surface, ToroidalSurface};
    use openrcad_foundation::TolerancePolicy;

    #[test]
    fn section_lies_exactly_on_plane_and_torus_across_scale() {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            let frame = Ax3::new(Pnt::new(7.0 * scale, -9.0 * scale, 4.0 * scale), Dir::dz());
            let offset = 0.5 * scale;
            let curve = TorusPlaneSection::new(frame, 4.0 * scale, scale, Dir::dx(), offset, true);
            let torus = ToroidalSurface::new(frame, 4.0 * scale, scale);
            for v in [0.0, 0.7, 1.9, 3.4, 5.8] {
                let point = curve.point(v);
                assert!(
                    ((point.x() - frame.location().x()) - offset).abs()
                        <= TolerancePolicy::STANDARD.intersection
                );
                let torus_point = torus.point(
                    (point.y() - frame.location().y()).atan2(point.x() - frame.location().x()),
                    v,
                );
                assert!(point.distance(&torus_point) <= TolerancePolicy::STANDARD.intersection);
            }
        }
    }
}
