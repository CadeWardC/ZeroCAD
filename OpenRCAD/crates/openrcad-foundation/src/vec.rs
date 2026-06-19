//! Free vectors — [`Vec`] (3D) and [`Vec2d`] (2D).
//!
//! A [`Vec`] is a displacement: it has magnitude and direction but no fixed
//! location (OCCT `gp_Vec`). Contrast [`Dir`](crate::dir::Dir), which is the
//! unit-heading part only. Adding a vector to a [`Pnt`](crate::pnt::Pnt)
//! translates the point; subtracting two points yields a vector.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub};

use serde::{Deserialize, Serialize};

use crate::dir::{Dir, Dir2d};
use crate::pnt::{Pnt, Pnt2d};
use crate::xyz::{Xy, Xyz};

/// A free 3D vector (OCCT `gp_Vec`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec {
    coord: Xyz,
}

impl Vec {
    /// The zero vector.
    pub const ZERO: Self = Self {
        coord: Xyz::new(0.0, 0.0, 0.0),
    };
    /// `(1, 0, 0)`.
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

    /// Create from three components.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            coord: Xyz::new(x, y, z),
        }
    }

    /// Wrap a coordinate triple.
    #[inline]
    pub const fn from_xyz(coord: Xyz) -> Self {
        Self { coord }
    }

    /// From a unit direction (magnitude 1).
    #[inline]
    pub const fn from_dir(d: Dir) -> Self {
        Self::from_xyz(d.coord())
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

    /// Squared magnitude.
    #[inline]
    pub fn magnitude_squared(&self) -> f64 {
        self.coord.modulus_squared()
    }

    /// Magnitude (length).
    #[inline]
    pub fn magnitude(&self) -> f64 {
        self.coord.modulus()
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.coord.dot(&other.coord)
    }

    /// Cross product (`self × other`).
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self::from_xyz(self.coord.cross(&other.coord))
    }

    /// Unit direction, or `None` if this is (near) zero.
    #[inline]
    pub fn normalized(&self) -> Option<Dir> {
        self.coord.normalized().map(Dir::from_xyz)
    }

    /// Normalized to a unit vector, or `None` if (near) zero.
    #[inline]
    pub fn normalized_vec(&self) -> Option<Self> {
        self.coord.normalized().map(Self::from_xyz)
    }

    /// Angle to `other` in `[0, π]`.
    #[inline]
    pub fn angle(&self, other: &Self) -> f64 {
        (self.dot(other) / (self.magnitude() * other.magnitude()))
            .clamp(-1.0, 1.0)
            .acos()
    }

    /// True when within `tol` of `other`.
    #[inline]
    pub fn is_equal(&self, other: &Self, tol: f64) -> bool {
        self.coord.subtracted(&other.coord).modulus() <= tol
    }

    /// `self + other`.
    #[inline]
    pub fn added(&self, other: &Self) -> Self {
        Self::from_xyz(self.coord.added(&other.coord))
    }

    /// `self - other`.
    #[inline]
    pub fn subtracted(&self, other: &Self) -> Self {
        Self::from_xyz(self.coord.subtracted(&other.coord))
    }

    /// Scaled by `s`.
    #[inline]
    pub fn multiplied(&self, s: f64) -> Self {
        Self::from_xyz(self.coord.multiplied(s))
    }

    /// Divided by `s`.
    #[inline]
    pub fn divided(&self, s: f64) -> Self {
        Self::from_xyz(self.coord.divided(s))
    }

    /// Componentwise negation.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::from_xyz(self.coord.reversed())
    }
}

impl Add for Vec {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self.added(&rhs)
    }
}

impl Sub for Vec {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self.subtracted(&rhs)
    }
}

impl Mul<f64> for Vec {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        self.multiplied(rhs)
    }
}

impl Div<f64> for Vec {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self {
        self.divided(rhs)
    }
}

impl Neg for Vec {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        self.reversed()
    }
}

impl AddAssign for Vec {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.coord = self.coord.added(&rhs.coord);
    }
}

impl Add<Pnt> for Vec {
    type Output = Pnt;
    #[inline]
    fn add(self, rhs: Pnt) -> Pnt {
        Pnt::from_xyz(self.coord.added(&rhs.coord()))
    }
}

/// A free 2D vector (OCCT `gp_Vec2d`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec2d {
    coord: Xy,
}

impl Vec2d {
    /// The zero vector.
    pub const ZERO: Self = Self {
        coord: Xy::new(0.0, 0.0),
    };

    /// Create from two components.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self {
            coord: Xy::new(x, y),
        }
    }

    /// Wrap a coordinate pair.
    #[inline]
    pub const fn from_xy(coord: Xy) -> Self {
        Self { coord }
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

    /// Squared magnitude.
    #[inline]
    pub fn magnitude_squared(&self) -> f64 {
        self.coord.modulus_squared()
    }

    /// Magnitude (length).
    #[inline]
    pub fn magnitude(&self) -> f64 {
        self.coord.modulus()
    }

    /// Unit direction, or `None` if (near) zero.
    #[inline]
    pub fn normalized(&self) -> Option<Dir2d> {
        Dir2d::from_vec(self)
    }

    /// `self + other`.
    #[inline]
    pub fn added(&self, other: &Self) -> Self {
        Self::from_xy(self.coord.added(&other.coord))
    }

    /// `self - other`.
    #[inline]
    pub fn subtracted(&self, other: &Self) -> Self {
        Self::from_xy(self.coord.subtracted(&other.coord))
    }

    /// Scaled by `s`.
    #[inline]
    pub fn multiplied(&self, s: f64) -> Self {
        Self::from_xy(self.coord.multiplied(s))
    }
}

impl Add<Pnt2d> for Vec2d {
    type Output = Pnt2d;
    #[inline]
    fn add(self, rhs: Pnt2d) -> Pnt2d {
        Pnt2d::from_xy(self.coord.added(&rhs.coord()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn magnitude_and_normalize() {
        let v = Vec::new(0.0, 0.0, 4.0);
        assert!((v.magnitude() - 4.0).abs() < 1e-12);
        let d = v.normalized().unwrap();
        assert!((d.z() - 1.0).abs() < 1e-12);
        assert!(Vec::ZERO.normalized().is_none());
    }

    #[test]
    fn point_plus_vector() {
        let p = Pnt::new(1.0, 2.0, 3.0);
        let v = Vec::new(10.0, 0.0, 0.0);
        let q = v + p;
        assert_eq!(q, Pnt::new(11.0, 2.0, 3.0));
    }
}
