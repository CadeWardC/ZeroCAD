//! Unit directions — [`Dir`] (3D) and [`Dir2d`] (2D).
//!
//! A [`Dir`] is a vector of length one (OCCT `gp_Dir`). Constructing one from a
//! non-unit triple normalizes it; constructing one from (near) zero is a logic
//! error and panics, so use [`Dir::try_new`] when the input may be degenerate.

use core::ops::{Add, Neg};

use serde::{Deserialize, Serialize};

use crate::pnt::Pnt;
use crate::vec::{Vec, Vec2d};
use crate::xyz::{Xy, Xyz};

/// A unit 3D direction (OCCT `gp_Dir`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dir {
    coord: Xyz,
}

impl Dir {
    /// `(1, 0, 0)` — the default direction.
    pub const DX: Self = Self {
        coord: Xyz::new(1.0, 0.0, 0.0),
    };
    /// `(0, 1, 0)`.
    pub const DY: Self = Self {
        coord: Xyz::new(0.0, 1.0, 0.0),
    };
    /// `(0, 0, 1)`.
    pub const DZ: Self = Self {
        coord: Xyz::new(0.0, 0.0, 1.0),
    };

    /// The `+X` direction `(1, 0, 0)`.
    #[inline]
    pub const fn dx() -> Self {
        Self::DX
    }
    /// The `+Y` direction `(0, 1, 0)`.
    #[inline]
    pub const fn dy() -> Self {
        Self::DY
    }
    /// The `+Z` direction `(0, 0, 1)`.
    #[inline]
    pub const fn dz() -> Self {
        Self::DZ
    }

    /// Construct a unit direction from a (possibly non-unit) triple.
    ///
    /// Panics if the triple is (near) zero — a direction with no heading is a
    /// caller bug. See [`Dir::try_new`] for the fallible form.
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self::try_new(x, y, z).expect("Dir::new from a near-zero vector")
    }

    /// Fallible construction: `None` if `(x, y, z)` is (near) zero.
    #[inline]
    pub fn try_new(x: f64, y: f64, z: f64) -> Option<Self> {
        Xyz::new(x, y, z).normalized().map(Self::from_xyz)
    }

    /// Wrap an already-unit coordinate triple without renormalizing.
    #[inline]
    pub const fn from_xyz(coord: Xyz) -> Self {
        Self { coord }
    }

    /// From a free vector — same direction, normalized.
    #[inline]
    pub fn from_vec(v: &Vec) -> Option<Self> {
        v.normalized()
    }

    /// The x component.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.coord.x()
    }
    /// The y component.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.coord.y()
    }
    /// The z component.
    #[inline]
    pub const fn z(&self) -> f64 {
        self.coord.z()
    }

    /// The underlying coordinate triple.
    #[inline]
    pub const fn coord(&self) -> Xyz {
        self.coord
    }

    /// Cosine of the angle to `other` (the dot product of two unit vectors).
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.coord.dot(&other.coord)
    }

    /// Cross product, renormalized to a unit direction.
    ///
    /// Panics if `self` and `other` are (near) parallel (the cross is then
    /// zero). See [`Dir::try_cross`].
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        self.coord
            .cross(&other.coord)
            .normalized()
            .map(Self::from_xyz)
            .expect("Dir::cross of parallel directions")
    }

    /// Fallible cross product: `None` if `self` and `other` are parallel.
    #[inline]
    pub fn try_cross(&self, other: &Self) -> Option<Self> {
        self.coord
            .cross(&other.coord)
            .normalized()
            .map(Self::from_xyz)
    }

    /// Angle to `other` in `[0, π]`.
    #[inline]
    pub fn angle(&self, other: &Self) -> f64 {
        self.dot(other).clamp(-1.0, 1.0).acos()
    }

    /// True when within `ang_tol` radians of `other`.
    #[inline]
    pub fn is_equal(&self, other: &Self, ang_tol: f64) -> bool {
        self.angle(other) <= ang_tol
    }

    /// True when parallel or anti-parallel within `ang_tol`.
    #[inline]
    pub fn is_parallel(&self, other: &Self, ang_tol: f64) -> bool {
        let a = self.angle(other);
        a <= ang_tol || (core::f64::consts::PI - a) <= ang_tol
    }

    /// True when `self` is perpendicular to `other` within `ang_tol`.
    #[inline]
    pub fn is_normal(&self, other: &Self, ang_tol: f64) -> bool {
        (self.dot(other).abs() - 0.0).abs() <= ang_tol.sin().max(ang_tol)
    }

    /// The opposite direction.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::from_xyz(self.coord.reversed())
    }
}

impl Default for Dir {
    /// The default direction is `+X` `(1, 0, 0)` (matching OCCT `gp_Dir`).
    #[inline]
    fn default() -> Self {
        Self::DX
    }
}

impl Neg for Dir {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        self.reversed()
    }
}

impl Add<&Pnt> for &Dir {
    /// Translating a point by a unit direction (scaled by 1).
    type Output = Pnt;
    #[inline]
    fn add(self, rhs: &Pnt) -> Pnt {
        Pnt::from_xyz(rhs.coord() + self.coord)
    }
}

/// A unit 2D direction (OCCT `gp_Dir2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dir2d {
    coord: Xy,
}

impl Dir2d {
    /// `(1, 0)` — the default 2D direction.
    pub const DX: Self = Self {
        coord: Xy::new(1.0, 0.0),
    };
    /// `(0, 1)`.
    pub const DY: Self = Self {
        coord: Xy::new(0.0, 1.0),
    };

    /// The `+X` direction `(1, 0)`.
    #[inline]
    pub const fn dx() -> Self {
        Self::DX
    }
    /// The `+Y` direction `(0, 1)`.
    #[inline]
    pub const fn dy() -> Self {
        Self::DY
    }

    /// Construct a unit direction from a (possibly non-unit) pair.
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self::try_new(x, y).expect("Dir2d::new from a near-zero vector")
    }

    /// Fallible construction.
    #[inline]
    pub fn try_new(x: f64, y: f64) -> Option<Self> {
        let m = (x * x + y * y).sqrt();
        if m < crate::tolerance::CONFUSION {
            None
        } else {
            Some(Self {
                coord: Xy::new(x / m, y / m),
            })
        }
    }

    /// Wrap an already-unit coordinate pair.
    #[inline]
    pub const fn from_xy(coord: Xy) -> Self {
        Self { coord }
    }

    /// From a free vector.
    #[inline]
    pub fn from_vec(v: &Vec2d) -> Option<Self> {
        let m = v.coord().modulus();
        if m < crate::tolerance::CONFUSION {
            None
        } else {
            Some(Self::from_xy(v.coord() / m))
        }
    }

    /// The x component.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.coord.x()
    }
    /// The y component.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.coord.y()
    }

    /// The underlying coordinate pair.
    #[inline]
    pub const fn coord(&self) -> Xy {
        self.coord
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.coord.dot(&other.coord)
    }

    /// Angle to `other` in `[0, π]`.
    #[inline]
    pub fn angle(&self, other: &Self) -> f64 {
        self.dot(other).clamp(-1.0, 1.0).acos()
    }

    /// The perpendicular direction `(y, -x)` rotated +90° (OCCT `gp_Dir2d::Rotated`).
    #[inline]
    pub fn rotated_90(&self) -> Self {
        Self::from_xy(Xy::new(-self.coord.y(), self.coord.x()))
    }

    /// The opposite direction.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::from_xy(-self.coord)
    }
}

impl Default for Dir2d {
    #[inline]
    fn default() -> Self {
        Self::DX
    }
}

impl Neg for Dir2d {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        self.reversed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_normalizes() {
        let d = Dir::new(0.0, 0.0, 5.0);
        assert!((d.x() - 0.0).abs() < 1e-12);
        assert!((d.z() - 1.0).abs() < 1e-12);
        assert!(Dir::try_new(0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn cross_and_angle() {
        let i = Dir::DX;
        let j = Dir::DY;
        assert_eq!(i.cross(&j), Dir::DZ);
        assert!((i.angle(&j) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!(i.is_parallel(&Dir::new(2.0, 0.0, 0.0), 1e-9));
        assert!(i.is_normal(&j, 1e-9));
    }

    #[test]
    fn dir2d_rotates_90() {
        let d = Dir2d::DX;
        let r = d.rotated_90();
        assert!((r.x() - 0.0).abs() < 1e-12);
        assert!((r.y() - 1.0).abs() < 1e-12);
    }
}
