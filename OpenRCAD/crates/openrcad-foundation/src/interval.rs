//! Interval arithmetic: closed intervals `[lo, hi]` with outward-rounded lattice
//! operations (OCCT `Bnd`-adjacent, used by the intersection solvers).
//!
//! The intersection engine consumes these to compute *guaranteed* bounds of a
//! geometric expression over a parameter range, so its adaptive subdivision
//! prunes rigorously — unlike the f64 sample bounds used previously, an interval
//! bound can neither miss a real intersection nor invent a false one.
//!
//! ## Rigor vs. tightness
//!
//! Bounds are **conservative** (never narrower than the true real-number result):
//! each arithmetic op rounds its result *outward* by one unit roundoff so the
//! exact value is always contained. Over-widening therefore only ever costs
//! pruning tightness (a slower search), never correctness. For the analytic
//! surfaces (plane/cylinder/cone/sphere) the bounds are near-exact because the
//! expression chains are short. [`Affine`](crate::Affine) further tightens the
//! rotational-surface cases by preserving `cos`/`sin` correlation.

use serde::{Deserialize, Serialize};

use crate::bnd::BndBox;
use crate::pnt::Pnt;
use crate::xyz::Xyz;

/// 2^-53, the f64 unit roundoff upper bound.
const EPS: f64 = 1.1102230246251565e-16;

/// A closed interval `[lo, hi]` with `lo <= hi`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interval {
    pub lo: f64,
    pub hi: f64,
}

impl Interval {
    /// `[lo, hi]`, swapping if given reversed.
    pub fn new(lo: f64, hi: f64) -> Self {
        if lo <= hi {
            Self { lo, hi }
        } else {
            Self { lo: hi, hi: lo }
        }
    }

    /// Degenerate `[x, x]`.
    #[inline]
    pub const fn point(x: f64) -> Self {
        Self { lo: x, hi: x }
    }

    /// `hi - lo`.
    #[inline]
    pub fn width(&self) -> f64 {
        self.hi - self.lo
    }

    /// `0.5 (lo + hi)`.
    #[inline]
    pub fn midpoint(&self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    /// Split at the midpoint into two halves.
    #[inline]
    pub fn split(&self) -> [Self; 2] {
        let m = self.midpoint();
        [Self { lo: self.lo, hi: m }, Self { lo: m, hi: self.hi }]
    }

    /// True if `x ∈ [lo, hi]`.
    #[inline]
    pub fn contains(&self, x: f64) -> bool {
        x >= self.lo && x <= self.hi
    }

    /// True if the intervals overlap (touching counts).
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    /// Outward-round `[lo, hi]` by one unit roundoff of the larger endpoint, so
    /// the exact real result of an op is always contained.
    #[inline]
    fn widen(lo: f64, hi: f64) -> Self {
        let mag = lo.abs().max(hi.abs());
        let e = EPS * mag;
        Self {
            lo: lo - e,
            hi: hi + e,
        }
    }

    /// `[lo+rhs.lo, hi+rhs.hi]`.
    pub fn add(&self, rhs: &Self) -> Self {
        Self::widen(self.lo + rhs.lo, self.hi + rhs.hi)
    }

    /// `[lo-rhs.hi, hi-rhs.lo]`.
    pub fn sub(&self, rhs: &Self) -> Self {
        Self::widen(self.lo - rhs.hi, self.hi - rhs.lo)
    }

    /// `[-hi, -lo]`.
    pub fn neg(&self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }

    /// Multiply by a scalar.
    pub fn scale(&self, s: f64) -> Self {
        if s >= 0.0 {
            Self::widen(self.lo * s, self.hi * s)
        } else {
            Self::widen(self.hi * s, self.lo * s)
        }
    }

    /// Interval product `[min(ac,ad,bc,bd), max(…)]`.
    pub fn mul(&self, rhs: &Self) -> Self {
        let (a, b) = (self.lo, self.hi);
        let (c, d) = (rhs.lo, rhs.hi);
        let ac = a * c;
        let ad = a * d;
        let bc = b * c;
        let bd = b * d;
        Self::widen(ac.min(ad).min(bc).min(bd), ac.max(ad).max(bc).max(bd))
    }

    /// `1 / self`. Returns the whole line if `0` lies in the interval.
    pub fn recip(&self) -> Self {
        if self.lo <= 0.0 && self.hi >= 0.0 {
            return Self::whole();
        }
        let p = 1.0 / self.hi;
        let q = 1.0 / self.lo;
        Self::widen(p.min(q), p.max(q))
    }

    /// `self / rhs`.
    pub fn div(&self, rhs: &Self) -> Self {
        self.mul(&rhs.recip())
    }

    /// `[a,b]²`, exploiting sign for a tighter bound than `self.mul(self)`.
    pub fn sqr(&self) -> Self {
        let (a, b) = (self.lo, self.hi);
        if a >= 0.0 {
            Self::widen(a * a, b * b)
        } else if b <= 0.0 {
            Self::widen(b * b, a * a)
        } else {
            let m = a.abs().max(b.abs());
            Self::widen(0.0, m * m)
        }
    }

    /// `sqrt([a,b])` for `lo >= 0` (negative `lo` clamped to 0).
    pub fn sqrt(&self) -> Self {
        if self.hi < 0.0 {
            return Self::empty();
        }
        let lo = if self.lo < 0.0 { 0.0 } else { self.lo };
        Self::widen(lo.sqrt(), self.hi.sqrt())
    }

    /// `|self|`.
    pub fn abs(&self) -> Self {
        let (a, b) = (self.lo, self.hi);
        if a >= 0.0 {
            Self::widen(a, b)
        } else if b <= 0.0 {
            Self::widen(-b, -a)
        } else {
            Self::widen(0.0, a.abs().max(b.abs()))
        }
    }

    /// Componentwise min (lattice meet).
    pub fn min(&self, rhs: &Self) -> Self {
        Self::new(self.lo.min(rhs.lo), (self.hi).min(rhs.hi))
    }

    /// Componentwise max (lattice join).
    pub fn max(&self, rhs: &Self) -> Self {
        Self::new(self.lo.max(rhs.lo), (self.hi).max(rhs.hi))
    }

    /// The empty interval (contains nothing).
    #[inline]
    pub const fn empty() -> Self {
        Self {
            lo: f64::INFINITY,
            hi: f64::NEG_INFINITY,
        }
    }

    /// `(-∞, ∞)`.
    #[inline]
    pub const fn whole() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }
}

/// A 2D interval box: the Cartesian product of two [`Interval`]s.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interval2 {
    pub x: Interval,
    pub y: Interval,
}

impl Interval2 {
    #[inline]
    pub const fn new(x: Interval, y: Interval) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.x.overlaps(&other.x) && self.y.overlaps(&other.y)
    }
}

/// A 3D interval box: the Cartesian product of three [`Interval`]s — the
/// guaranteed-enclosing bound of a curve or surface patch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interval3 {
    pub x: Interval,
    pub y: Interval,
    pub z: Interval,
}

impl Interval3 {
    #[inline]
    pub const fn new(x: Interval, y: Interval, z: Interval) -> Self {
        Self { x, y, z }
    }

    /// A degenerate box around a single point.
    pub fn from_point(p: &Pnt) -> Self {
        Self {
            x: Interval::point(p.x()),
            y: Interval::point(p.y()),
            z: Interval::point(p.z()),
        }
    }

    /// True if the two boxes overlap in all axes (touching counts).
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.x.overlaps(&other.x) && self.y.overlaps(&other.y) && self.z.overlaps(&other.z)
    }

    /// Convert to an axis-aligned [`BndBox`]. No gap is applied — callers
    /// `enlarge(tol)` themselves, so adding one here would double-count.
    pub fn to_bndbox(&self) -> BndBox {
        let mut b = BndBox::new();
        b.add_xyz(&Xyz::new(self.x.lo, self.y.lo, self.z.lo));
        b.add_xyz(&Xyz::new(self.x.hi, self.y.hi, self.z.hi));
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_sub_neg() {
        let a = Interval::new(1.0, 3.0);
        let b = Interval::new(-2.0, 5.0);
        let s = a.add(&b);
        assert!(s.lo <= -1.0 && s.hi >= 8.0); // [-1, 8] widened
        let d = a.sub(&b);
        assert!(d.lo <= -4.0 && d.hi >= 5.0); // [-4, 5] widened
        let n = a.neg();
        assert_eq!((n.lo, n.hi), (-3.0, -1.0));
    }

    #[test]
    fn mul_and_scale() {
        let a = Interval::new(-1.0, 2.0);
        let b = Interval::new(3.0, 4.0);
        let m = a.mul(&b);
        assert!(m.lo <= -4.0 && m.hi >= 8.0); // [-4, 8]
        let s = a.scale(-2.0);
        assert!(s.lo <= -4.0 && s.hi >= 2.0); // [-4, 2]
    }

    #[test]
    fn sqr_sqrt_abs() {
        let a = Interval::new(-2.0, 3.0);
        let sq = a.sqr();
        assert!(sq.lo <= 0.0 && sq.hi >= 9.0); // [0, 9]
        let ab = a.abs();
        assert!(ab.lo <= 0.0 && ab.hi >= 3.0); // [0, 3]
        let sqr = Interval::new(4.0, 9.0).sqrt();
        assert!(sqr.lo <= 2.0 && sqr.hi >= 3.0); // [2, 3]
    }

    #[test]
    fn recip_div_zero() {
        assert_eq!(Interval::new(-1.0, 1.0).recip(), Interval::whole());
        let r = Interval::new(2.0, 4.0).recip();
        assert!(r.lo <= 0.25 && r.hi >= 0.5); // [0.25, 0.5]
    }

    #[test]
    fn encloses_exact_values() {
        // A real-number result must always lie in the widened interval.
        let a = Interval::point(0.1);
        let b = Interval::point(0.2);
        let s = a.add(&b);
        assert!(s.contains(0.30000000000000004)); // exact f64 sum
    }

    #[test]
    fn overlaps_and_split() {
        assert!(Interval::new(0.0, 1.0).overlaps(&Interval::new(1.0, 2.0)));
        assert!(!Interval::new(0.0, 1.0).overlaps(&Interval::new(2.0, 3.0)));
        let [lo, hi] = Interval::new(0.0, 4.0).split();
        assert_eq!((lo.lo, lo.hi), (0.0, 2.0));
        assert_eq!((hi.lo, hi.hi), (2.0, 4.0));
    }

    #[test]
    fn interval3_box_and_overlap() {
        let a = Interval3::new(
            Interval::new(0.0, 1.0),
            Interval::new(0.0, 1.0),
            Interval::new(0.0, 1.0),
        );
        let b = Interval3::new(
            Interval::new(0.5, 2.0),
            Interval::new(0.5, 2.0),
            Interval::new(0.5, 2.0),
        );
        assert!(a.overlaps(&b));
        let box_a = a.to_bndbox();
        let (lo, hi) = box_a.corners().unwrap();
        assert!((lo.x()) <= 0.0 && (hi.x()) >= 1.0);
    }
}
