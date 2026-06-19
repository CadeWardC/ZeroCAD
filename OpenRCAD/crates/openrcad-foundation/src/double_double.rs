//! Double-double arithmetic for 106-bit precision float operations.
//! Useful for robust geometric predicates where standard f64 rounding fails.

use core::ops::{Add, Mul, Neg, Sub};

/// A double-double precision floating point number.
///
/// Represents the value `head + tail` where `head` is the most significant
/// part and `tail` is the least significant part.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct DoubleDouble {
    pub head: f64,
    pub tail: f64,
}

impl DoubleDouble {
    pub const ZERO: Self = Self {
        head: 0.0,
        tail: 0.0,
    };
    pub const ONE: Self = Self {
        head: 1.0,
        tail: 0.0,
    };
    pub const NEG_ONE: Self = Self {
        head: -1.0,
        tail: 0.0,
    };

    /// Create a new `DoubleDouble` from head and tail components.
    #[inline]
    pub const fn new(head: f64, tail: f64) -> Self {
        Self { head, tail }
    }

    /// Create a `DoubleDouble` from a standard `f64`.
    #[inline]
    pub const fn from_f64(val: f64) -> Self {
        Self {
            head: val,
            tail: 0.0,
        }
    }

    /// Convert the `DoubleDouble` back to an approximate `f64`.
    #[inline]
    pub fn to_f64(self) -> f64 {
        self.head + self.tail
    }
}

impl From<f64> for DoubleDouble {
    #[inline]
    fn from(val: f64) -> Self {
        Self::from_f64(val)
    }
}

/// Compute the exact sum of two floats `a` and `b`.
/// Returns `(sum, error)` where `sum + error` is the exact mathematical sum.
#[inline]
pub fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let v = s - a;
    let e = (a - (s - v)) + (b - v);
    (s, e)
}

const SPLIT: f64 = 134217729.0; // 2^27 + 1

/// Split an `f64` into two non-overlapping halves.
/// Returns `(high, low)` such that `a = high + low`.
#[inline]
pub fn split(a: f64) -> (f64, f64) {
    let c = SPLIT * a;
    let ab = c - (c - a);
    let al = a - ab;
    (ab, al)
}

/// Compute the exact product of two floats `a` and `b`.
/// Returns `(prod, error)` where `prod + error` is the exact mathematical product.
#[inline]
pub fn two_prod(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    let (ah, al) = split(a);
    let (bh, bl) = split(b);
    let err = ((ah * bh - p) + ah * bl + al * bh) + al * bl;
    (p, err)
}

impl Add for DoubleDouble {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        let (s, e) = two_sum(self.head, other.head);
        let (t, std) = two_sum(self.tail, other.tail);
        let e = e + t;
        let (s, e) = two_sum(s, e);
        let e = e + std;
        let (s, e) = two_sum(s, e);
        DoubleDouble { head: s, tail: e }
    }
}

impl Add<f64> for DoubleDouble {
    type Output = Self;
    #[inline]
    fn add(self, other: f64) -> Self {
        let (s, e) = two_sum(self.head, other);
        let e = e + self.tail;
        let (s, e) = two_sum(s, e);
        DoubleDouble { head: s, tail: e }
    }
}

impl Add<DoubleDouble> for f64 {
    type Output = DoubleDouble;
    #[inline]
    fn add(self, other: DoubleDouble) -> DoubleDouble {
        other + self
    }
}

impl Neg for DoubleDouble {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        DoubleDouble {
            head: -self.head,
            tail: -self.tail,
        }
    }
}

impl Sub for DoubleDouble {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        self + (-other)
    }
}

impl Sub<f64> for DoubleDouble {
    type Output = Self;
    #[inline]
    fn sub(self, other: f64) -> Self {
        self + (-other)
    }
}

impl Sub<DoubleDouble> for f64 {
    type Output = DoubleDouble;
    #[inline]
    fn sub(self, other: DoubleDouble) -> DoubleDouble {
        DoubleDouble::from_f64(self) - other
    }
}

impl Mul for DoubleDouble {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        let (p1, p2) = two_prod(self.head, other.head);
        let err = p2 + (self.head * other.tail + self.tail * other.head);
        let (s, e) = two_sum(p1, err);
        DoubleDouble { head: s, tail: e }
    }
}

impl Mul<f64> for DoubleDouble {
    type Output = Self;
    #[inline]
    fn mul(self, other: f64) -> Self {
        let (p1, p2) = two_prod(self.head, other);
        let err = p2 + self.tail * other;
        let (s, e) = two_sum(p1, err);
        DoubleDouble { head: s, tail: e }
    }
}

impl Mul<DoubleDouble> for f64 {
    type Output = DoubleDouble;
    #[inline]
    fn mul(self, other: DoubleDouble) -> DoubleDouble {
        other * self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_two_sum() {
        let (s, e) = two_sum(1.0, 1e-16);
        assert_eq!(s, 1.0);
        assert_eq!(e, 1e-16);
    }

    #[test]
    fn test_two_prod() {
        let x = 1.0 + 2.0f64.powi(-27);
        let y = 1.0 + 2.0f64.powi(-27);
        let (p, e) = two_prod(x, y);
        assert_eq!(p, 1.0 + 2.0f64.powi(-26));
        assert_eq!(e, 2.0f64.powi(-54));
    }

    #[test]
    fn test_double_double_arithmetic() {
        let a = DoubleDouble::from_f64(1.0);
        let b = DoubleDouble::from_f64(1e-16);
        let c = a + b;
        assert_eq!(c.head, 1.0);
        assert_eq!(c.tail, 1e-16);

        let d = c - b;
        assert_eq!(d, a);

        let e = c * DoubleDouble::from_f64(2.0);
        assert_eq!(e.head, 2.0);
        assert_eq!(e.tail, 2e-16);
    }
}
