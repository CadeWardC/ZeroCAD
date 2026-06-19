//! Points — [`Pnt`] (3D) and [`Pnt2d`] (2D).
//!
//! A [`Pnt`] is a location (OCCT `gp_Pnt`). It is deliberately a *different
//! type* from [`Vec`](crate::vec::Vec): adding two points is meaningless, so it
//! is a compile error. The allowed mixed arithmetic is `Pnt + Vec -> Pnt`
//! (translate) and `Pnt - Pnt -> Vec` (displacement).

use core::ops::{Add, Sub};

use serde::{Deserialize, Serialize};

use crate::vec::{Vec, Vec2d};
use crate::xyz::{Xy, Xyz};

/// A 3D point / location (OCCT `gp_Pnt`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pnt {
    coord: Xyz,
}

impl Default for Pnt {
    #[inline]
    fn default() -> Self {
        Self::ORIGIN
    }
}

impl Pnt {
    /// The origin `(0, 0, 0)`.
    pub const ORIGIN: Self = Self { coord: Xyz::ZERO };

    /// Create a point from three coordinates.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            coord: Xyz::new(x, y, z),
        }
    }

    /// The origin `(0, 0, 0)`.
    #[inline]
    pub const fn origin() -> Self {
        Self::ORIGIN
    }

    /// Wrap a coordinate triple.
    #[inline]
    pub const fn from_xyz(coord: Xyz) -> Self {
        Self { coord }
    }

    /// The x coordinate.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.coord.x()
    }
    /// The y coordinate.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.coord.y()
    }
    /// The z coordinate.
    #[inline]
    pub const fn z(&self) -> f64 {
        self.coord.z()
    }

    /// The underlying coordinate triple.
    #[inline]
    pub const fn coord(&self) -> Xyz {
        self.coord
    }

    /// Set all three coordinates.
    #[inline]
    pub fn set_coord(&mut self, x: f64, y: f64, z: f64) {
        self.coord.set_coord(x, y, z);
    }

    /// Euclidean distance to `other`.
    #[inline]
    pub fn distance(&self, other: &Self) -> f64 {
        self.coord.subtracted(&other.coord).modulus()
    }

    /// Squared distance to `other` (cheaper than [`distance`](Self::distance)).
    #[inline]
    pub fn distance_squared(&self, other: &Self) -> f64 {
        self.coord.subtracted(&other.coord).modulus_squared()
    }

    /// True when within `tol` of `other`.
    #[inline]
    pub fn is_equal(&self, other: &Self, tol: f64) -> bool {
        self.distance(other) <= tol
    }

    /// Translate by `v`.
    #[inline]
    pub fn translated(&self, v: &Vec) -> Self {
        Self::from_xyz(self.coord.added(&v.coord()))
    }

    /// The point `b` such that `self` is the barycenter of `self` and `other`
    /// with weight — equivalently, the midpoint.
    #[inline]
    pub fn midpoint(&self, other: &Self) -> Self {
        Self::from_xyz(self.coord.added(&other.coord).multiplied(0.5))
    }

    /// Barycenter of `self` and `other` with weights `a` and `b`
    /// (the point `(a*self + b*other) / (a + b)`).
    #[inline]
    pub fn barycentric(&self, other: &Self, a: f64, b: f64) -> Self {
        let s = a + b;
        debug_assert!(s.abs() > 0.0, "barycentric weights sum to zero");
        Self::from_xyz(
            self.coord
                .multiplied(a / s)
                .added(&other.coord.multiplied(b / s)),
        )
    }
}

impl From<Xyz> for Pnt {
    #[inline]
    fn from(coord: Xyz) -> Self {
        Self::from_xyz(coord)
    }
}

impl Add<Vec> for Pnt {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Vec) -> Self {
        Self::from_xyz(self.coord.added(&rhs.coord()))
    }
}

impl Sub<Vec> for Pnt {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Vec) -> Self {
        Self::from_xyz(self.coord.subtracted(&rhs.coord()))
    }
}

impl Sub for Pnt {
    type Output = Vec;
    #[inline]
    fn sub(self, rhs: Self) -> Vec {
        Vec::from_xyz(self.coord.subtracted(&rhs.coord))
    }
}

/// A 2D point / location (OCCT `gp_Pnt2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pnt2d {
    coord: Xy,
}

impl Default for Pnt2d {
    #[inline]
    fn default() -> Self {
        Self::ORIGIN
    }
}

impl Pnt2d {
    /// The origin `(0, 0)`.
    pub const ORIGIN: Self = Self { coord: Xy::ZERO };

    /// Create a point from two coordinates.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self {
            coord: Xy::new(x, y),
        }
    }

    /// The origin.
    #[inline]
    pub const fn origin() -> Self {
        Self::ORIGIN
    }

    /// Wrap a coordinate pair.
    #[inline]
    pub const fn from_xy(coord: Xy) -> Self {
        Self { coord }
    }

    /// The x coordinate.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.coord.x()
    }
    /// The y coordinate.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.coord.y()
    }

    /// The underlying coordinate pair.
    #[inline]
    pub const fn coord(&self) -> Xy {
        self.coord
    }

    /// Euclidean distance to `other`.
    #[inline]
    pub fn distance(&self, other: &Self) -> f64 {
        self.coord.subtracted(&other.coord).modulus()
    }

    /// Translate by `v`.
    #[inline]
    pub fn translated(&self, v: &Vec2d) -> Self {
        Self::from_xy(self.coord.added(&v.coord()))
    }
}

impl Add<Vec2d> for Pnt2d {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Vec2d) -> Self {
        Self::from_xy(self.coord.added(&rhs.coord()))
    }
}

impl Sub for Pnt2d {
    type Output = Vec2d;
    #[inline]
    fn sub(self, rhs: Self) -> Vec2d {
        Vec2d::from_xy(self.coord.subtracted(&rhs.coord))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_and_displacement() {
        let a = Pnt::new(0.0, 0.0, 0.0);
        let b = Pnt::new(3.0, 4.0, 0.0);
        assert!((a.distance(&b) - 5.0).abs() < 1e-12);
        // Pnt - Pnt -> Vec
        let v = b - a;
        assert_eq!(v, Vec::new(3.0, 4.0, 0.0));
    }

    #[test]
    fn translate_round_trip() {
        let p = Pnt::new(1.0, 1.0, 1.0);
        let v = Vec::new(1.0, 0.0, 0.0);
        assert_eq!(p + v, Pnt::new(2.0, 1.0, 1.0));
        assert_eq!((p + v) - v, p);
    }

    #[test]
    fn midpoint_is_average() {
        let a = Pnt::new(0.0, 0.0, 0.0);
        let b = Pnt::new(10.0, 20.0, 30.0);
        assert_eq!(a.midpoint(&b), Pnt::new(5.0, 10.0, 15.0));
    }
}
