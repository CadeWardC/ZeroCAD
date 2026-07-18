//! Geometric tolerance and precision — OCCT's `Precision` package.
//!
//! Every coordinate comparison in OpenRCAD should go through these constants
//! rather than an ad-hoc `1e-9` literal, so the tolerance policy lives in
//! exactly one place and can be tuned globally.

use crate::bnd::BndBox;
use crate::xyz::{Xy, Xyz};
use serde::{Deserialize, Serialize};

/// Kernel-wide geometric tolerances for one modeling document.
///
/// The individual values name the operation stage they govern instead of
/// relying on unrelated local epsilon literals. Algorithms should accept a
/// policy at their public boundary and carry it through intersection,
/// reconstruction, sewing, validation, and recovery.
///
/// [`TolerancePolicy::STANDARD`] preserves OpenRCAD's pre-policy behavior and
/// is used by compatibility entry points that do not yet take a policy.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TolerancePolicy {
    /// Smallest meaningful linear distance and default entity uncertainty.
    pub linear: f64,
    /// Smallest meaningful angular difference, in radians.
    pub angular: f64,
    /// Chordal/sampling accuracy used by approximation and topology matching.
    pub approximation: f64,
    /// Distance used when intersecting curves and surfaces.
    pub intersection: f64,
    /// Distance used when classifying points and face regions.
    pub classification: f64,
    /// Maximum gap that sewing may close.
    pub sewing: f64,
    /// Maximum allowed deviation between a pcurve lifted through its surface
    /// and the corresponding 3D edge curve.
    pub pcurve_consistency: f64,
    /// Lower numerical bound for scale-derived recovery tolerances.
    pub resolution: f64,
    /// Model-size fraction eligible for near-coincident snapping.
    pub snap_relative: f64,
    /// Absolute upper bound for near-coincident snapping.
    pub snap_max: f64,
    /// Model-size fraction below which a wall is considered a sliver.
    pub sliver_relative: f64,
    /// Absolute upper bound for sliver rejection.
    pub sliver_max: f64,
}

impl TolerancePolicy {
    /// Standard millimetre-scale OpenRCAD policy.
    pub const STANDARD: Self = Self {
        linear: 1e-9,
        angular: 1e-9,
        approximation: 1e-6,
        intersection: 1e-5,
        classification: 1e-5,
        sewing: 1e-5,
        pcurve_consistency: 1e-6,
        resolution: 1e-12,
        snap_relative: 1e-7,
        snap_max: 1e-4,
        sliver_relative: 1e-6,
        sliver_max: 1e-3,
    };

    /// Verify that every tolerance is finite, positive, and internally
    /// consistent before an operation consumes the policy.
    pub fn validate(&self) -> Result<(), TolerancePolicyError> {
        let values = [
            (ToleranceField::Linear, self.linear),
            (ToleranceField::Angular, self.angular),
            (ToleranceField::Approximation, self.approximation),
            (ToleranceField::Intersection, self.intersection),
            (ToleranceField::Classification, self.classification),
            (ToleranceField::Sewing, self.sewing),
            (ToleranceField::PcurveConsistency, self.pcurve_consistency),
            (ToleranceField::Resolution, self.resolution),
            (ToleranceField::SnapRelative, self.snap_relative),
            (ToleranceField::SnapMax, self.snap_max),
            (ToleranceField::SliverRelative, self.sliver_relative),
            (ToleranceField::SliverMax, self.sliver_max),
        ];
        for (field, value) in values {
            if !value.is_finite() || value <= 0.0 {
                return Err(TolerancePolicyError::InvalidValue { field, value });
            }
        }
        if self.angular > core::f64::consts::PI {
            return Err(TolerancePolicyError::InvalidValue {
                field: ToleranceField::Angular,
                value: self.angular,
            });
        }
        for (field, value) in [
            (ToleranceField::Approximation, self.approximation),
            (ToleranceField::Intersection, self.intersection),
            (ToleranceField::Classification, self.classification),
            (ToleranceField::Sewing, self.sewing),
            (ToleranceField::PcurveConsistency, self.pcurve_consistency),
        ] {
            if value < self.linear {
                return Err(TolerancePolicyError::BelowLinear { field, value });
            }
        }
        if self.resolution > self.linear {
            return Err(TolerancePolicyError::ResolutionAboveLinear {
                resolution: self.resolution,
                linear: self.linear,
            });
        }
        if self.snap_max < self.resolution {
            return Err(TolerancePolicyError::BelowResolution {
                field: ToleranceField::SnapMax,
                value: self.snap_max,
            });
        }
        if self.sliver_max < self.resolution {
            return Err(TolerancePolicyError::BelowResolution {
                field: ToleranceField::SliverMax,
                value: self.sliver_max,
            });
        }
        Ok(())
    }

    /// Scale-aware tolerance used by near-coincident snapping.
    #[inline]
    pub fn snap_tolerance(&self, model_scale: f64) -> f64 {
        (model_scale.abs() * self.snap_relative).clamp(self.resolution, self.snap_max)
    }

    /// Scale-aware wall thickness rejected as a degenerate sliver.
    #[inline]
    pub fn sliver_tolerance(&self, model_scale: f64) -> f64 {
        (model_scale.abs() * self.sliver_relative).clamp(self.resolution, self.sliver_max)
    }
}

impl Default for TolerancePolicy {
    fn default() -> Self {
        Self::STANDARD
    }
}

/// Operation-local tolerances derived from document policy and numerical
/// conditioning. This is runtime context, never persisted as document intent.
///
/// `model_scale` is the actual bounds diagonal; it is deliberately not floored
/// to one model unit. Very small parts therefore retain their real scale while
/// the arithmetic floor still accounts for large world translations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToleranceContext {
    pub policy: TolerancePolicy,
    pub model_scale: f64,
    pub local_feature_size: f64,
    pub translation_magnitude: f64,
    pub conditioning: f64,
    pub arithmetic_floor: f64,
    pub convergence: f64,
    pub welding: f64,
    pub quantization: f64,
}

impl ToleranceContext {
    /// Derive context from all operation bounds and the smallest feature that
    /// the operation must preserve. Invalid/absent optional hints fall back to
    /// the real non-zero bounds scale, never to an artificial unit length.
    pub fn derive(
        policy: &TolerancePolicy,
        bounds: &[BndBox],
        local_feature_size: Option<f64>,
        conditioning: f64,
    ) -> Result<Self, TolerancePolicyError> {
        policy.validate()?;

        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for bounds in bounds {
            let Some((min, max)) = bounds.corners() else {
                continue;
            };
            for (axis, (min, max)) in [(min.x(), max.x()), (min.y(), max.y()), (min.z(), max.z())]
                .into_iter()
                .enumerate()
            {
                lo[axis] = lo[axis].min(min);
                hi[axis] = hi[axis].max(max);
            }
        }
        let valid_bounds = lo.iter().chain(&hi).all(|value| value.is_finite())
            && (0..3).all(|axis| hi[axis] >= lo[axis]);
        let model_scale = if valid_bounds {
            ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
        } else {
            0.0
        };
        let translation_magnitude = if valid_bounds {
            lo.iter()
                .chain(&hi)
                .map(|value| value.abs())
                .fold(0.0, f64::max)
        } else {
            0.0
        };
        let conditioning = if conditioning.is_finite() && conditioning > 0.0 {
            conditioning.max(1.0)
        } else {
            1.0
        };
        let local_feature_size = local_feature_size
            .filter(|value| value.is_finite() && *value > 0.0)
            .or_else(|| (model_scale > 0.0).then_some(model_scale))
            .unwrap_or(policy.resolution);
        let arithmetic_floor = policy.resolution.max(
            translation_magnitude
                .max(model_scale)
                .mul_add(f64::EPSILON * conditioning, 0.0),
        );
        let stage = |value: f64| value.max(arithmetic_floor);
        let mut effective = *policy;
        effective.linear = stage(effective.linear);
        effective.approximation = stage(effective.approximation);
        effective.intersection = stage(effective.intersection);
        effective.classification = stage(effective.classification);
        effective.sewing = stage(effective.sewing);
        effective.pcurve_consistency = stage(effective.pcurve_consistency);
        effective.resolution = arithmetic_floor.min(effective.linear);

        let convergence = stage(
            (local_feature_size * f64::EPSILON.sqrt() * conditioning).min(policy.approximation),
        );
        let welding = effective.sewing;
        let quantization = (local_feature_size * policy.snap_relative)
            .clamp(arithmetic_floor, policy.snap_max.max(arithmetic_floor));

        Ok(Self {
            policy: effective,
            model_scale,
            local_feature_size,
            translation_magnitude,
            conditioning,
            arithmetic_floor,
            convergence,
            welding,
            quantization,
        })
    }
}

/// Field names used by structured policy-validation errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ToleranceField {
    Linear,
    Angular,
    Approximation,
    Intersection,
    Classification,
    Sewing,
    PcurveConsistency,
    Resolution,
    SnapRelative,
    SnapMax,
    SliverRelative,
    SliverMax,
}

/// Why a [`TolerancePolicy`] cannot be used by a modeling operation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TolerancePolicyError {
    /// A field is non-finite, zero, negative, or otherwise outside its domain.
    InvalidValue { field: ToleranceField, value: f64 },
    /// A stage tolerance is smaller than the policy's linear confusion.
    BelowLinear { field: ToleranceField, value: f64 },
    /// Numerical resolution must not exceed linear confusion.
    ResolutionAboveLinear { resolution: f64, linear: f64 },
    /// A scale-derived tolerance cap is below numerical resolution.
    BelowResolution { field: ToleranceField, value: f64 },
}

impl core::fmt::Display for TolerancePolicyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidValue { field, value } => {
                write!(f, "invalid {field:?} tolerance: {value}")
            }
            Self::BelowLinear { field, value } => {
                write!(f, "{field:?} tolerance {value} is below linear confusion")
            }
            Self::ResolutionAboveLinear { resolution, linear } => write!(
                f,
                "numerical resolution {resolution} exceeds linear confusion {linear}"
            ),
            Self::BelowResolution { field, value } => {
                write!(
                    f,
                    "{field:?} tolerance {value} is below numerical resolution"
                )
            }
        }
    }
}

impl std::error::Error for TolerancePolicyError {}

/// Linear confusion: two points within this distance are coincident.
///
/// Matches OCCT `Precision::Confusion()` (1e-9). Below this, distances are
/// treated as zero.
pub const CONFUSION: f64 = TolerancePolicy::STANDARD.linear;

/// Angular confusion in radians — two directions within this angle are parallel.
///
/// Matches OCCT `Precision::Angular()` (1e-9).
pub const ANGULAR: f64 = TolerancePolicy::STANDARD.angular;

/// Approximation tolerance for sampling/fitting (OCCT `Precision::Approximation()`).
pub const APPROXIMATION: f64 = TolerancePolicy::STANDARD.approximation;

/// A "very large but finite" coordinate bound (OCCT `Precision::Infinite()`).
pub const INFINITE: f64 = 1e100;

/// True when `a` and `b` agree to within [`CONFUSION`].
#[inline]
pub fn is_equal_scalar(a: f64, b: f64) -> bool {
    (a - b).abs() <= CONFUSION
}

/// True when the two 3D coordinates are within `tol` of each other.
#[inline]
pub fn is_xyz_equal(a: &Xyz, b: &Xyz, tol: f64) -> bool {
    (a.x() - b.x()).abs() <= tol && (a.y() - b.y()).abs() <= tol && (a.z() - b.z()).abs() <= tol
}

/// True when the two 2D coordinates are within `tol` of each other.
#[inline]
pub fn is_xy_equal(a: &Xy, b: &Xy, tol: f64) -> bool {
    (a.x() - b.x()).abs() <= tol && (a.y() - b.y()).abs() <= tol
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_equality_uses_confusion() {
        assert!(is_equal_scalar(1.0, 1.0));
        assert!(is_equal_scalar(1.0, 1.0 + CONFUSION / 2.0));
        assert!(!is_equal_scalar(1.0, 1.0 + CONFUSION * 10.0));
    }

    #[test]
    fn xyz_equality_is_per_component() {
        let a = Xyz::new(0.0, 0.0, 0.0);
        let b = Xyz::new(CONFUSION / 2.0, 0.0, 0.0);
        assert!(is_xyz_equal(&a, &b, CONFUSION));
        let c = Xyz::new(CONFUSION * 2.0, 0.0, 0.0);
        assert!(!is_xyz_equal(&a, &c, CONFUSION));
    }

    #[test]
    fn standard_policy_is_valid_and_preserves_legacy_constants() {
        let policy = TolerancePolicy::STANDARD;
        assert!(policy.validate().is_ok());
        assert_eq!(policy.linear, CONFUSION);
        assert_eq!(policy.angular, ANGULAR);
        assert_eq!(policy.approximation, APPROXIMATION);
        assert!((policy.snap_tolerance(1_000.0) - 1e-4).abs() < f64::EPSILON);
        assert!((policy.sliver_tolerance(1_000.0) - 1e-3).abs() < f64::EPSILON);
    }

    #[test]
    fn invalid_policy_is_rejected_structurally() {
        let invalid = TolerancePolicy {
            intersection: f64::NAN,
            ..TolerancePolicy::STANDARD
        };
        assert!(matches!(
            invalid.validate(),
            Err(TolerancePolicyError::InvalidValue {
                field: ToleranceField::Intersection,
                ..
            })
        ));

        let invalid = TolerancePolicy {
            sewing: CONFUSION / 2.0,
            ..TolerancePolicy::STANDARD
        };
        assert!(matches!(
            invalid.validate(),
            Err(TolerancePolicyError::BelowLinear {
                field: ToleranceField::Sewing,
                ..
            })
        ));
    }

    #[test]
    fn context_preserves_sub_unit_scale_and_accounts_for_translation() {
        let mut local = BndBox::new();
        local.add(&crate::Pnt::new(1.0e9, 0.0, 0.0));
        local.add(&crate::Pnt::new(1.0e9 + 1.0e-3, 2.0e-3, 3.0e-3));
        let context =
            ToleranceContext::derive(&TolerancePolicy::STANDARD, &[local], Some(2.0e-4), 8.0)
                .unwrap();
        assert!(context.model_scale < 1.0, "there is no unit-scale floor");
        assert_eq!(context.local_feature_size, 2.0e-4);
        assert!(context.translation_magnitude >= 1.0e9);
        assert!(context.arithmetic_floor > TolerancePolicy::STANDARD.resolution);
        assert!(context.policy.intersection >= context.arithmetic_floor);
        assert!(context.quantization < 1.0e-4);
    }

    #[test]
    fn context_property_sweep_has_no_unit_floor() {
        let scales = [1.0e-3, 1.0e-1, 1.0, 10.0, 1.0e3];
        let aspects = [1.0, 10.0, 100.0];
        let origins = [[0.0, 0.0, 0.0], [1.0e9, -1.0e9, 5.0e8]];

        for scale in scales {
            for aspect in aspects {
                for origin in origins {
                    let mut bounds = BndBox::new();
                    bounds.add(&crate::Pnt::new(origin[0], origin[1], origin[2]));
                    bounds.add(&crate::Pnt::new(
                        origin[0] + 8.0 * scale * aspect,
                        origin[1] + 4.0 * scale,
                        origin[2] + 2.0 * scale,
                    ));
                    let context = ToleranceContext::derive(
                        &TolerancePolicy::STANDARD,
                        &[bounds],
                        Some(2.0 * scale),
                        aspect,
                    )
                    .unwrap();

                    assert!(context.model_scale.is_finite() && context.model_scale > 0.0);
                    assert_eq!(context.local_feature_size, 2.0 * scale);
                    assert!(context.arithmetic_floor >= TolerancePolicy::STANDARD.resolution);
                    assert!(context.policy.intersection >= context.arithmetic_floor);
                    assert!(context.convergence >= context.arithmetic_floor);
                    assert!(context.quantization >= context.arithmetic_floor);
                    assert!(
                        context.quantization
                            <= TolerancePolicy::STANDARD
                                .snap_max
                                .max(context.arithmetic_floor)
                    );
                    if origin == [0.0, 0.0, 0.0] && scale == 1.0e-3 && aspect == 1.0 {
                        assert!(context.model_scale < 1.0, "unit-scale floor returned");
                    }
                }
            }
        }
    }
}
