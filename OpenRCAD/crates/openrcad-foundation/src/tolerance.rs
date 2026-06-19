//! Geometric tolerance and precision — OCCT's `Precision` package.
//!
//! Every coordinate comparison in OpenRCAD should go through these constants
//! rather than an ad-hoc `1e-9` literal, so the tolerance policy lives in
//! exactly one place and can be tuned globally.

use crate::xyz::{Xy, Xyz};

/// Linear confusion: two points within this distance are coincident.
///
/// Matches OCCT `Precision::Confusion()` (1e-9). Below this, distances are
/// treated as zero.
pub const CONFUSION: f64 = 1e-9;

/// Angular confusion in radians — two directions within this angle are parallel.
///
/// Matches OCCT `Precision::Angular()` (1e-9).
pub const ANGULAR: f64 = 1e-9;

/// Approximation tolerance for sampling/fitting (OCCT `Precision::Approximation()`).
pub const APPROXIMATION: f64 = 1e-6;

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
}
