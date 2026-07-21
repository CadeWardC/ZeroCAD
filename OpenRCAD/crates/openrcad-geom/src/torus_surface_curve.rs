//! Exact affine curve in a torus' two-dimensional parameter space.

use core::f64::consts::TAU;

use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

use crate::{Curve, Surface, ToroidalSurface};

/// An exact curve on a torus with affine `(u, v)` coordinates.
///
/// Split seams on a toroidal support can have their endpoints moved by
/// neighboring offset faces. They are then no longer circles even though both
/// incident faces still share the same torus. This representation keeps that
/// seam exact without promoting a tessellated approximation to durable B-Rep
/// topology.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TorusSurfaceCurve {
    torus: ToroidalSurface,
    domain_first: f64,
    domain_last: f64,
    u_first: f64,
    u_last: f64,
    v_first: f64,
    v_last: f64,
}

impl TorusSurfaceCurve {
    /// Construct an exact affine curve, rejecting invalid parameter intervals.
    pub fn new(
        torus: ToroidalSurface,
        domain: (f64, f64),
        u: (f64, f64),
        v: (f64, f64),
    ) -> Option<Self> {
        let parameters = [domain.0, domain.1, u.0, u.1, v.0, v.1];
        if parameters.iter().any(|parameter| !parameter.is_finite()) || domain.0 == domain.1 {
            return None;
        }
        Some(Self {
            torus,
            domain_first: domain.0,
            domain_last: domain.1,
            u_first: u.0,
            u_last: u.1,
            v_first: v.0,
            v_last: v.1,
        })
    }

    #[inline]
    pub const fn torus(&self) -> ToroidalSurface {
        self.torus
    }

    #[inline]
    pub const fn u_bounds(&self) -> (f64, f64) {
        (self.u_first, self.u_last)
    }

    #[inline]
    pub const fn v_bounds(&self) -> (f64, f64) {
        (self.v_first, self.v_last)
    }

    #[inline]
    fn fraction(&self, parameter: f64) -> f64 {
        (parameter - self.domain_first) / (self.domain_last - self.domain_first)
    }

    #[inline]
    pub fn uv(&self, parameter: f64) -> (f64, f64) {
        let fraction = self.fraction(parameter);
        (
            self.u_first + (self.u_last - self.u_first) * fraction,
            self.v_first + (self.v_last - self.v_first) * fraction,
        )
    }
}

impl Curve for TorusSurfaceCurve {
    fn point(&self, parameter: f64) -> Pnt {
        let (u, v) = self.uv(parameter);
        self.torus.d1(u, v).0
    }

    fn d1(&self, parameter: f64) -> (Pnt, Vec) {
        let (u, v) = self.uv(parameter);
        let (point, du, dv) = self.torus.d1(u, v);
        let inverse_domain = 1.0 / (self.domain_last - self.domain_first);
        (
            point,
            du * ((self.u_last - self.u_first) * inverse_domain)
                + dv * ((self.v_last - self.v_first) * inverse_domain),
        )
    }

    fn bounds(&self) -> (f64, f64) {
        (self.domain_first, self.domain_last)
    }

    fn is_closed(&self) -> bool {
        let periodic_delta = |delta: f64| {
            let turns = (delta / TAU).round();
            let arithmetic_tolerance = 64.0 * f64::EPSILON * delta.abs().max(TAU);
            (delta - turns * TAU).abs() <= arithmetic_tolerance
        };
        periodic_delta(self.u_last - self.u_first) && periodic_delta(self.v_last - self.v_first)
    }

    fn is_periodic(&self) -> bool {
        false
    }

    fn transformed(&self, transform: &Trsf) -> Self {
        Self {
            torus: self.torus.transformed(transform),
            ..*self
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Surface;
    use openrcad_foundation::{Ax1, Ax3, Dir};

    #[test]
    fn affine_parameter_curve_remains_exact_across_scale_rotation_and_origin() {
        let base = TorusSurfaceCurve::new(
            ToroidalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 7.0, 2.0),
            (2.0, 5.0),
            (0.3, 2.7),
            (1.1, 1.3),
        )
        .unwrap();
        for transform in [
            Trsf::scale(&Pnt::origin(), 1.0e-3),
            Trsf::rotation(
                &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
                37.0_f64.to_radians(),
            ),
            Trsf::translation(Vec::new(1.0e9, -2.0e9, 3.0e9))
                .multiply(&Trsf::scale(&Pnt::origin(), 1.0e3)),
        ] {
            let curve = base.transformed(&transform);
            for parameter in [2.0, 2.75, 3.5, 4.25, 5.0] {
                let fraction = (parameter - 2.0) / 3.0;
                let u = 0.3 + (2.7 - 0.3) * fraction;
                let v = 1.1 + (1.3 - 1.1) * fraction;
                assert_eq!(curve.point(parameter), curve.torus().point(u, v));
                assert!(curve.d1(parameter).1.magnitude().is_finite());
            }
        }
    }
}
