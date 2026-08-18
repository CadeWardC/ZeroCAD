//! Parametric curves attached to individual coedges.
//!
//! A 3D edge may be shared by several faces, but its representation in each
//! face's `(u, v)` parameter space is face-specific. [`PcurveData`] therefore
//! belongs to an oriented edge-use (a coedge), not to [`EdgeData`](crate::arena::EdgeData).

use openrcad_foundation::Pnt2d;
use openrcad_geom2d::{Curve2d, GeomCurve2d};
use serde::{Deserialize, Serialize};

/// Periods of the carrying surface's parametric directions.
///
/// Pcurve coordinates are stored unwrapped. A curve crossing a cylindrical U
/// seam may therefore run from `u = 5.8` to `u = 6.7`; wrapping is performed
/// only when a consumer explicitly requests it. This avoids artificial jumps
/// at periodic seams during interpolation, splitting, and tessellation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SurfacePeriodicity {
    /// U period, or `None` when U is not periodic.
    pub u_period: Option<f64>,
    /// V period, or `None` when V is not periodic.
    pub v_period: Option<f64>,
}

impl SurfacePeriodicity {
    /// A non-periodic parameter space.
    pub const NONE: Self = Self {
        u_period: None,
        v_period: None,
    };

    /// Construct periodicity metadata for a U-periodic surface.
    #[inline]
    pub const fn u_periodic(period: f64) -> Self {
        Self {
            u_period: Some(period),
            v_period: None,
        }
    }

    /// True when every declared period is finite and strictly positive.
    #[inline]
    pub fn is_valid(self) -> bool {
        [self.u_period, self.v_period]
            .into_iter()
            .flatten()
            .all(|period| period.is_finite() && period > 0.0)
    }
}

/// A bounded 2D curve representing one coedge on its carrying face.
///
/// The pcurve has its own parameter interval; it does not need to match the 3D
/// edge's curve parameters. Both intervals are related by normalized progress.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PcurveData {
    /// Supporting curve in the face's parameter space.
    pub curve: GeomCurve2d,
    /// First pcurve parameter, corresponding to the 3D edge's natural start.
    pub first: f64,
    /// Last pcurve parameter, corresponding to the 3D edge's natural end.
    pub last: f64,
    /// Periodic directions of the carrying surface.
    #[serde(default)]
    pub periodicity: SurfacePeriodicity,
}

impl PcurveData {
    /// Construct a bounded pcurve in a non-periodic parameter space.
    #[inline]
    pub fn new(curve: GeomCurve2d, first: f64, last: f64) -> Self {
        Self {
            curve,
            first,
            last,
            periodicity: SurfacePeriodicity::NONE,
        }
    }

    /// Attach surface-periodicity metadata.
    #[inline]
    pub fn with_periodicity(mut self, periodicity: SurfacePeriodicity) -> Self {
        self.periodicity = periodicity;
        self
    }

    /// True when the parameter interval and periodicity metadata are usable.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.first.is_finite()
            && self.last.is_finite()
            && self.first != self.last
            && self.periodicity.is_valid()
    }

    /// Pcurve parameter at normalized edge progress `fraction`.
    #[inline]
    pub fn parameter_at_fraction(&self, fraction: f64) -> f64 {
        self.first + (self.last - self.first) * fraction
    }

    /// Evaluate the pcurve without wrapping periodic coordinates.
    #[inline]
    pub fn point_at_fraction(&self, fraction: f64) -> Pnt2d {
        self.curve.point(self.parameter_at_fraction(fraction))
    }

    /// Evaluate and wrap coordinates into each declared base period.
    #[inline]
    pub fn wrapped_point_at_fraction(&self, fraction: f64) -> Pnt2d {
        let point = self.point_at_fraction(fraction);
        Pnt2d::new(
            wrap_if_periodic(point.x(), self.periodicity.u_period),
            wrap_if_periodic(point.y(), self.periodicity.v_period),
        )
    }

    /// Split this pcurve at normalized edge progress `fraction`.
    ///
    /// The curve and seam metadata are preserved while each result receives
    /// the appropriate independent parameter subrange.
    pub fn split_at_fraction(&self, fraction: f64) -> Option<(Self, Self)> {
        if !self.is_valid() || !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
            return None;
        }
        let middle = self.parameter_at_fraction(fraction);
        if middle == self.first || middle == self.last {
            return None;
        }
        let mut first = self.clone();
        first.last = middle;
        let mut second = self.clone();
        second.first = middle;
        Some((first, second))
    }

    /// Return the same face-local curve with its bounded traversal reversed.
    /// This is required when sewing chooses an oppositely-oriented 3D edge as
    /// the representative for a coedge: the pcurve's normalized progress must
    /// continue to match the representative edge's natural start and end.
    #[inline]
    pub fn reversed(&self) -> Self {
        let mut reversed = self.clone();
        std::mem::swap(&mut reversed.first, &mut reversed.last);
        reversed
    }

    /// Map this pcurve through an exact diagonal change of surface coordinates.
    ///
    /// Uniform 3D scaling changes analytic surface parameters differently: a
    /// plane scales both `(u, v)`, while cylinders and cones keep angular `u`
    /// and scale distance-valued `v`. Lines and B-splines are closed under this
    /// mapping, so their stored pcurves can be preserved exactly. Other conics
    /// return `None` for anisotropic UV changes and are handled by the validated
    /// pcurve reconstruction fallback.
    pub(crate) fn scaled_surface_coordinates(&self, u_scale: f64, v_scale: f64) -> Option<Self> {
        if !u_scale.is_finite() || !v_scale.is_finite() || u_scale <= 0.0 || v_scale <= 0.0 {
            return None;
        }
        if u_scale == 1.0 && v_scale == 1.0 {
            return Some(self.clone());
        }

        let map_point = |point: Pnt2d| Pnt2d::new(point.x() * u_scale, point.y() * v_scale);
        let (curve, parameter_scale) = match &self.curve {
            GeomCurve2d::Line(line) => {
                let direction = line.direction();
                let dx = direction.x() * u_scale;
                let dy = direction.y() * v_scale;
                let parameter_scale = dx.hypot(dy);
                if !parameter_scale.is_finite() || parameter_scale <= f64::EPSILON {
                    return None;
                }
                (
                    GeomCurve2d::line(openrcad_geom2d::Line2d::from_point_dir(
                        map_point(line.location()),
                        openrcad_foundation::Dir2d::try_new(dx, dy)?,
                    )),
                    parameter_scale,
                )
            }
            GeomCurve2d::BSpline(curve) => (
                GeomCurve2d::bspline(openrcad_geom2d::BSplineCurve2d::new(
                    curve.degree(),
                    curve.poles().iter().copied().map(map_point).collect(),
                    curve.weights().map(<[f64]>::to_vec),
                    curve.knots().to_vec(),
                    curve.multiplicities().to_vec(),
                )),
                1.0,
            ),
            GeomCurve2d::Circle(curve) if u_scale == v_scale => (
                GeomCurve2d::circle(openrcad_geom2d::Circle2d::new(
                    scaled_frame(curve.position(), map_point),
                    curve.radius() * u_scale,
                )),
                1.0,
            ),
            GeomCurve2d::Ellipse(curve) if u_scale == v_scale => (
                GeomCurve2d::ellipse(openrcad_geom2d::Ellipse2d::new(
                    scaled_frame(curve.position(), map_point),
                    curve.major_radius() * u_scale,
                    curve.minor_radius() * u_scale,
                )),
                1.0,
            ),
            GeomCurve2d::Parabola(curve) if u_scale == v_scale => (
                GeomCurve2d::parabola(openrcad_geom2d::Parabola2d::new(
                    scaled_frame(curve.position(), map_point),
                    curve.focal() * u_scale,
                )),
                u_scale,
            ),
            GeomCurve2d::Hyperbola(curve) if u_scale == v_scale => (
                GeomCurve2d::hyperbola(openrcad_geom2d::Hyperbola2d::new(
                    scaled_frame(curve.position(), map_point),
                    curve.major_radius() * u_scale,
                    curve.minor_radius() * u_scale,
                )),
                1.0,
            ),
            // This curve's parameters are torus angles, so it is meaningful
            // only under the identity UV map handled above. Any anisotropic
            // coordinate change must rebuild and revalidate the pcurve from
            // its exact 3D section instead.
            GeomCurve2d::TorusPlaneSection(_) => return None,
            GeomCurve2d::PlaneTorusSection(_) => return None,
            GeomCurve2d::CylinderPlaneSection(curve) if u_scale == 1.0 => (
                GeomCurve2d::cylinder_plane_section((*curve).scaled_v(v_scale)?),
                1.0,
            ),
            GeomCurve2d::SphereGreatCircle(_) => return None,
            GeomCurve2d::EndpointCorrected(_) => return None,
            _ => return None,
        };

        Some(Self {
            curve,
            first: self.first * parameter_scale,
            last: self.last * parameter_scale,
            periodicity: SurfacePeriodicity {
                u_period: self.periodicity.u_period.map(|period| period * u_scale),
                v_period: self.periodicity.v_period.map(|period| period * v_scale),
            },
        })
    }

    /// True when the unwrapped pcurve crosses a U seam.
    pub fn crosses_u_seam(&self) -> bool {
        crosses_periodic_seam(
            self.point_at_fraction(0.0).x(),
            self.point_at_fraction(1.0).x(),
            self.periodicity.u_period,
        )
    }

    /// True when the unwrapped pcurve crosses a V seam.
    pub fn crosses_v_seam(&self) -> bool {
        crosses_periodic_seam(
            self.point_at_fraction(0.0).y(),
            self.point_at_fraction(1.0).y(),
            self.periodicity.v_period,
        )
    }
}

fn scaled_frame(
    frame: openrcad_foundation::Ax22d,
    map_point: impl FnOnce(Pnt2d) -> Pnt2d,
) -> openrcad_foundation::Ax22d {
    openrcad_foundation::Ax22d::new_axes(
        map_point(frame.location()),
        frame.x_direction(),
        frame.y_direction(),
    )
}

fn wrap_if_periodic(value: f64, period: Option<f64>) -> f64 {
    period.map_or(value, |period| value.rem_euclid(period))
}

fn crosses_periodic_seam(first: f64, last: f64, period: Option<f64>) -> bool {
    let Some(period) = period.filter(|period| period.is_finite() && *period > 0.0) else {
        return false;
    };
    (first / period).floor() != (last / period).floor()
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir2d, Pnt2d};
    use openrcad_geom2d::{CylinderPlaneSection2d, Line2d};

    fn u_line(origin: f64) -> PcurveData {
        PcurveData::new(
            GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::new(origin, 2.0), Dir2d::dx())),
            0.0,
            1.0,
        )
    }

    #[test]
    fn periodic_seam_keeps_an_unwrapped_curve_continuous() {
        let pcurve =
            u_line(5.8).with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU));
        assert!(pcurve.crosses_u_seam());
        assert_eq!(pcurve.point_at_fraction(1.0), Pnt2d::new(6.8, 2.0));
        let wrapped = pcurve.wrapped_point_at_fraction(1.0);
        assert!((wrapped.x() - (6.8 - core::f64::consts::TAU)).abs() < 1e-12);
        assert_eq!(wrapped.y(), 2.0);
    }

    #[test]
    fn splitting_preserves_independent_ranges_and_periodicity() {
        let pcurve =
            u_line(5.8).with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU));
        let (first, second) = pcurve.split_at_fraction(0.25).unwrap();
        assert_eq!((first.first, first.last), (0.0, 0.25));
        assert_eq!((second.first, second.last), (0.25, 1.0));
        assert_eq!(first.periodicity, pcurve.periodicity);
        assert_eq!(second.periodicity, pcurve.periodicity);
    }

    #[test]
    fn diagonal_surface_scale_maps_line_pcurves_exactly() {
        let source = PcurveData::new(
            GeomCurve2d::line(Line2d::from_point_dir(
                Pnt2d::new(2.0, 3.0),
                Dir2d::try_new(1.0, 1.0).expect("constant direction is non-zero"),
            )),
            -1.0,
            2.0,
        );
        let scaled = source
            .scaled_surface_coordinates(4.0, 2.0)
            .expect("line mapping is exact");
        for fraction in [0.0, 0.25, 0.5, 1.0] {
            let before = source.point_at_fraction(fraction);
            let after = scaled.point_at_fraction(fraction);
            assert!((after.x() - before.x() * 4.0).abs() < 1e-12);
            assert!((after.y() - before.y() * 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn cylinder_section_scale_preserves_angle_and_scales_axial_distance() {
        let source = PcurveData::new(
            GeomCurve2d::cylinder_plane_section(
                CylinderPlaneSection2d::new(0.4, false, 3.0, 2.0, -1.0).unwrap(),
            ),
            0.2,
            1.4,
        )
        .with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU));
        let scaled = source
            .scaled_surface_coordinates(1.0, 1.0e3)
            .expect("cylinder section mapping is exact");
        for fraction in [0.0, 0.25, 0.5, 1.0] {
            let before = source.point_at_fraction(fraction);
            let after = scaled.point_at_fraction(fraction);
            assert_eq!(after.x(), before.x());
            assert!(
                (after.y() - before.y() * 1.0e3).abs()
                    <= 8.0 * f64::EPSILON * after.y().abs().max(f64::MIN_POSITIVE)
            );
        }
    }
}
