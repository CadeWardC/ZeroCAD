//! Bounding boxes — [`BndBox`] (3D) and [`BndBox2d`] (2D) (OCCT `Bnd`).
//!
//! An axis-aligned box around a set of points, with a small `gap` so adjacency
//! tests are robust. A void box (nothing added yet) is the neutral element for
//! [`BndBox::add_box`].

use serde::{Deserialize, Serialize};

use crate::pnt::{Pnt, Pnt2d};
use crate::trsf::Trsf;
use crate::xyz::{Xy, Xyz};

/// A 3D axis-aligned bounding box (OCCT `BndBox`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BndBox {
    min: Option<Xyz>,
    max: Option<Xyz>,
    gap: f64,
}

impl Default for BndBox {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl BndBox {
    /// A void box (contains nothing).
    #[inline]
    pub const fn new() -> Self {
        Self {
            min: None,
            max: None,
            gap: 0.0,
        }
    }

    /// Reset to void.
    pub fn clear(&mut self) {
        self.min = None;
        self.max = None;
        self.gap = 0.0;
    }

    /// True if nothing has been added.
    #[inline]
    pub const fn is_void(&self) -> bool {
        self.min.is_none()
    }

    fn grow_to(&mut self, p: &Xyz) {
        match (self.min, self.max) {
            (Some(mut lo), Some(mut hi)) => {
                lo.set_x(lo.x().min(p.x()));
                lo.set_y(lo.y().min(p.y()));
                lo.set_z(lo.z().min(p.z()));
                hi.set_x(hi.x().max(p.x()));
                hi.set_y(hi.y().max(p.y()));
                hi.set_z(hi.z().max(p.z()));
                self.min = Some(lo);
                self.max = Some(hi);
            }
            _ => {
                self.min = Some(*p);
                self.max = Some(*p);
            }
        }
    }

    /// Enlarge to include `p`.
    #[inline]
    pub fn add(&mut self, p: &Pnt) {
        self.grow_to(&p.coord());
    }

    /// Enlarge to include the raw coordinate `p`.
    #[inline]
    pub fn add_xyz(&mut self, p: &Xyz) {
        self.grow_to(p);
    }

    /// Enlarge to include all of `other` (no-op if `other` is void).
    pub fn add_box(&mut self, other: &Self) {
        if let Some(lo) = other.corner_min() {
            self.grow_to(&lo);
        }
        if let Some(hi) = other.corner_max() {
            self.grow_to(&hi);
        }
        self.gap = self.gap.max(other.gap);
    }

    /// Add a symmetric tolerance `tol` around the box (OCCT `Enlarge`).
    pub fn enlarge(&mut self, tol: f64) {
        self.gap = self.gap.max(tol);
    }

    /// The lower corner (without the gap).
    #[inline]
    pub const fn corner_min(&self) -> Option<Xyz> {
        self.min
    }

    /// The upper corner (without the gap).
    #[inline]
    pub const fn corner_max(&self) -> Option<Xyz> {
        self.max
    }

    /// The corners as points, lower and upper (without the gap). `None` if void.
    pub fn corners(&self) -> Option<(Pnt, Pnt)> {
        match (self.min, self.max) {
            (Some(lo), Some(hi)) => Some((Pnt::from_xyz(lo), Pnt::from_xyz(hi))),
            _ => None,
        }
    }

    /// The corners with the [`gap`](Self::enlarge) applied. `None` if void.
    pub fn get(&self) -> Option<(Xyz, Xyz)> {
        match (self.min, self.max) {
            (Some(lo), Some(hi)) => Some((
                Xyz::new(lo.x() - self.gap, lo.y() - self.gap, lo.z() - self.gap),
                Xyz::new(hi.x() + self.gap, hi.y() + self.gap, hi.z() + self.gap),
            )),
            _ => None,
        }
    }

    /// True when `p` is outside the box (with the gap).
    pub fn is_out_point(&self, p: &Pnt) -> bool {
        match self.get() {
            Some((lo, hi)) => {
                p.x() < lo.x()
                    || p.x() > hi.x()
                    || p.y() < lo.y()
                    || p.y() > hi.y()
                    || p.z() < lo.z()
                    || p.z() > hi.z()
            }
            None => true,
        }
    }

    /// True when `other` is entirely outside this box.
    pub fn is_out_box(&self, other: &Self) -> bool {
        match (self.get(), other.get()) {
            (Some((a_lo, a_hi)), Some((b_lo, b_hi))) => {
                b_lo.x() > a_hi.x()
                    || b_hi.x() < a_lo.x()
                    || b_lo.y() > a_hi.y()
                    || b_hi.y() < a_lo.y()
                    || b_lo.z() > a_hi.z()
                    || b_hi.z() < a_lo.z()
            }
            _ => true,
        }
    }

    /// The axis-aligned box of this box's 8 corners after `t` (so a rotated box
    /// yields a correctly re-fitted AABB).
    pub fn transformed(&self, t: &Trsf) -> Self {
        let mut out = Self::new();
        if let Some((lo, hi)) = self.get() {
            for sx in [lo.x(), hi.x()] {
                for sy in [lo.y(), hi.y()] {
                    for sz in [lo.z(), hi.z()] {
                        out.add_xyz(&t.transform_xyz(&Xyz::new(sx, sy, sz)));
                    }
                }
            }
        }
        out
    }
}

/// A 2D axis-aligned bounding box (OCCT `BndBox2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BndBox2d {
    min: Option<Xy>,
    max: Option<Xy>,
    gap: f64,
}

impl Default for BndBox2d {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl BndBox2d {
    /// A void box.
    #[inline]
    pub const fn new() -> Self {
        Self {
            min: None,
            max: None,
            gap: 0.0,
        }
    }

    /// True if nothing has been added.
    #[inline]
    pub const fn is_void(&self) -> bool {
        self.min.is_none()
    }

    /// Enlarge to include `p`.
    pub fn add(&mut self, p: &Pnt2d) {
        let c = p.coord();
        match (self.min, self.max) {
            (Some(mut lo), Some(mut hi)) => {
                lo.set_x(lo.x().min(c.x()));
                lo.set_y(lo.y().min(c.y()));
                hi.set_x(hi.x().max(c.x()));
                hi.set_y(hi.y().max(c.y()));
                self.min = Some(lo);
                self.max = Some(hi);
            }
            _ => {
                self.min = Some(c);
                self.max = Some(c);
            }
        }
    }

    /// The corners as points, lower and upper. `None` if void.
    pub fn corners(&self) -> Option<(Pnt2d, Pnt2d)> {
        match (self.min, self.max) {
            (Some(lo), Some(hi)) => Some((Pnt2d::from_xy(lo), Pnt2d::from_xy(hi))),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::axis::Ax1;
    use crate::dir::Dir;

    #[test]
    fn add_and_corners() {
        let mut b = BndBox::new();
        b.add(&Pnt::new(0.0, 0.0, 0.0));
        b.add(&Pnt::new(2.0, 4.0, 6.0));
        b.add(&Pnt::new(1.0, 1.0, 1.0));
        let (lo, hi) = b.corners().unwrap();
        assert_eq!(lo, Pnt::new(0.0, 0.0, 0.0));
        assert_eq!(hi, Pnt::new(2.0, 4.0, 6.0));
    }

    #[test]
    fn is_out_point_uses_gap() {
        let mut b = BndBox::new();
        b.add(&Pnt::origin());
        b.add(&Pnt::new(1.0, 1.0, 1.0));
        b.enlarge(0.5);
        // Just outside the gap.
        assert!(b.is_out_point(&Pnt::new(2.0, 0.5, 0.5)));
        // Inside.
        assert!(!b.is_out_point(&Pnt::new(0.5, 0.5, 0.5)));
    }

    #[test]
    fn transformed_box_refits_aabb() {
        let mut b = BndBox::new();
        b.add(&Pnt::new(1.0, 0.0, 0.0));
        b.add(&Pnt::new(2.0, 3.0, 0.0));
        // Rotate 90° about Z: (x, y) -> (-y, x), so the box [1,2]x[0,3] becomes
        // x in [-3, 0], y in [1, 2].
        let r = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            core::f64::consts::FRAC_PI_2,
        );
        let t = b.transformed(&r);
        let (lo, hi) = t.corners().unwrap();
        assert!((lo.x() - (-3.0)).abs() < 1e-9);
        assert!((hi.x() - 0.0).abs() < 1e-9);
        assert!((lo.y() - 1.0).abs() < 1e-9);
        assert!((hi.y() - 2.0).abs() < 1e-9);
    }
}
