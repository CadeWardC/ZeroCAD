//! Geometric tolerance and precision — OCCT's `Precision` package.
//!
//! Every coordinate comparison in OpenRCAD should go through these constants
//! rather than an ad-hoc `1e-9` literal, so the tolerance policy lives in
//! exactly one place and can be tuned globally.

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
}
