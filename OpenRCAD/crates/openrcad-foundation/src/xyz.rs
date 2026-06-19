//! Raw coordinate tuples — [`Xy`] (2D) and [`Xyz`] (3D).
//!
//! These mirror OCCT's `gp_XY` / `gp_XYZ`: the mutable coordinate storage that
//! [`Pnt`](crate::pnt::Pnt), [`Vec`](crate::vec::Vec) and
//! [`Dir`](crate::dir::Dir) are built from. A coordinate carries *no* geometric
//! meaning of its own — it is just two or three numbers.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub};

use serde::{Deserialize, Serialize};

/// A 2D coordinate pair (OCCT `gp_XY`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Xy {
    x: f64,
    y: f64,
}

impl Xy {
    /// The origin `(0, 0)`.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    /// Create from two components.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// The x component.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.x
    }

    /// The y component.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.y
    }

    /// Set the x component.
    #[inline]
    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    /// Set the y component.
    #[inline]
    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    /// Both components as a tuple.
    #[inline]
    pub const fn coord(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    /// Set both components.
    #[inline]
    pub fn set_coord(&mut self, x: f64, y: f64) {
        self.x = x;
        self.y = y;
    }

    /// Squared length.
    #[inline]
    pub fn modulus_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// Length (Euclidean norm).
    #[inline]
    pub fn modulus(&self) -> f64 {
        self.modulus_squared().sqrt()
    }

    /// 2D dot product.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// 2D "cross" product — the scalar `x1*y2 - y1*x2` (the signed area).
    #[inline]
    pub fn cross(&self, other: &Self) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// `self + other`.
    #[inline]
    pub fn added(&self, other: &Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }

    /// `self - other`.
    #[inline]
    pub fn subtracted(&self, other: &Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }

    /// Scaled by `s`.
    #[inline]
    pub fn multiplied(&self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s)
    }

    /// Divided by `s`.
    #[inline]
    pub fn divided(&self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s)
    }

    /// Componentwise negation.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Add for Xy {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self.added(&rhs)
    }
}

impl Sub for Xy {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self.subtracted(&rhs)
    }
}

impl Mul<f64> for Xy {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        self.multiplied(rhs)
    }
}

impl Div<f64> for Xy {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self {
        self.divided(rhs)
    }
}

impl Neg for Xy {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        self.reversed()
    }
}

impl AddAssign for Xy {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

/// A 3D coordinate triple (OCCT `gp_XYZ`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Xyz {
    x: f64,
    y: f64,
    z: f64,
}

impl Xyz {
    /// The origin `(0, 0, 0)`.
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Create from three components.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// The x component.
    #[inline]
    pub const fn x(&self) -> f64 {
        self.x
    }

    /// The y component.
    #[inline]
    pub const fn y(&self) -> f64 {
        self.y
    }

    /// The z component.
    #[inline]
    pub const fn z(&self) -> f64 {
        self.z
    }

    /// Set the x component.
    #[inline]
    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    /// Set the y component.
    #[inline]
    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    /// Set the z component.
    #[inline]
    pub fn set_z(&mut self, z: f64) {
        self.z = z;
    }

    /// All three components as a tuple.
    #[inline]
    pub const fn coord(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }

    /// Set all three components.
    #[inline]
    pub fn set_coord(&mut self, x: f64, y: f64, z: f64) {
        self.x = x;
        self.y = y;
        self.z = z;
    }

    /// Squared length.
    #[inline]
    pub fn modulus_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Length (Euclidean norm).
    #[inline]
    pub fn modulus(&self) -> f64 {
        self.modulus_squared().sqrt()
    }

    /// 3D dot product.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// 3D cross product (`self × other`).
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Unit-length copy, or `None` if this is (near) zero.
    #[inline]
    pub fn normalized(&self) -> Option<Self> {
        let m = self.modulus();
        if m < crate::tolerance::CONFUSION {
            None
        } else {
            Some(self.multiplied(1.0 / m))
        }
    }

    /// `self + other`.
    #[inline]
    pub fn added(&self, other: &Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    /// `self - other`.
    #[inline]
    pub fn subtracted(&self, other: &Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    /// Scaled by `s`.
    #[inline]
    pub fn multiplied(&self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Divided by `s`.
    #[inline]
    pub fn divided(&self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }

    /// Componentwise negation.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Add for Xyz {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self.added(&rhs)
    }
}

impl Sub for Xyz {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self.subtracted(&rhs)
    }
}

impl Mul<f64> for Xyz {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        self.multiplied(rhs)
    }
}

impl Div<f64> for Xyz {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self {
        self.divided(rhs)
    }
}

impl Neg for Xyz {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        self.reversed()
    }
}

impl AddAssign for Xyz {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xyz_arithmetic() {
        let a = Xyz::new(1.0, 2.0, 3.0);
        let b = Xyz::new(4.0, 5.0, 6.0);
        assert_eq!(a.added(&b), Xyz::new(5.0, 7.0, 9.0));
        assert_eq!(b.subtracted(&a), Xyz::new(3.0, 3.0, 3.0));
        assert_eq!(a.multiplied(2.0), Xyz::new(2.0, 4.0, 6.0));
    }

    #[test]
    fn xyz_dot_and_cross() {
        let i = Xyz::new(1.0, 0.0, 0.0);
        let j = Xyz::new(0.0, 1.0, 0.0);
        let k = Xyz::new(0.0, 0.0, 1.0);
        assert!((i.dot(&j)).abs() < 1e-12);
        assert_eq!(i.cross(&j), k);
        // right-handed: j × k = i, k × i = j
        assert_eq!(j.cross(&k), i);
        assert_eq!(k.cross(&i), j);
    }

    #[test]
    fn xyz_normalize() {
        let v = Xyz::new(0.0, 0.0, 3.0);
        let n = v.normalized().expect("nonzero normalizes");
        assert!((n.modulus() - 1.0).abs() < 1e-12);
        assert_eq!(n, Xyz::new(0.0, 0.0, 1.0));
        assert!(Xyz::ZERO.normalized().is_none());
    }

    #[test]
    fn xy_cross_is_signed_area() {
        assert!((Xy::new(1.0, 0.0).cross(&Xy::new(0.0, 1.0)) - 1.0).abs() < 1e-12);
        assert!((Xy::new(0.0, 1.0).cross(&Xy::new(1.0, 0.0)) + 1.0).abs() < 1e-12);
    }
}
