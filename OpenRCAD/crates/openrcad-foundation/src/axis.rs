//! Axes and lines — [`Ax1`] (3D axis) and [`Lin`]/[`Lin2d`] (geometric lines).
//!
//! [`Ax1`] is an axis: an origin plus a direction (OCCT `gp_Ax1`) — the spine of
//! a rotation or the carrier of a line. [`Lin`] is the geometric line carried by
//! an axis; [`Lin2d`] is its 2D twin.

use serde::{Deserialize, Serialize};

use crate::dir::{Dir, Dir2d};
use crate::frame::Ax2d;
use crate::pnt::{Pnt, Pnt2d};
use crate::trsf::Trsf;
use crate::vec::Vec;

/// A 3D axis: an origin and a direction (OCCT `gp_Ax1`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ax1 {
    location: Pnt,
    direction: Dir,
}

impl Ax1 {
    /// Create an axis from a location and a direction.
    #[inline]
    pub const fn new(location: Pnt, direction: Dir) -> Self {
        Self {
            location,
            direction,
        }
    }

    /// A point on the axis.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.location
    }

    /// The axis direction.
    #[inline]
    pub const fn direction(&self) -> Dir {
        self.direction
    }

    /// Set the origin.
    #[inline]
    pub fn set_location(&mut self, location: Pnt) {
        self.location = location;
    }

    /// Set the direction.
    #[inline]
    pub fn set_direction(&mut self, direction: Dir) {
        self.direction = direction;
    }

    /// The point at parameter `t` along the axis: `location + t * direction`.
    #[inline]
    pub fn point(&self, t: f64) -> Pnt {
        self.location
            .translated(&Vec::from_dir(self.direction).multiplied(t))
    }

    /// True if this axis and `other` are the same line — parallel (or
    /// anti-parallel) within `ang_tol`, and the offset between their origins is
    /// perpendicular to the direction within `lin_tol`.
    pub fn is_coaxial(&self, other: &Self, ang_tol: f64, lin_tol: f64) -> bool {
        if !self.direction.is_parallel(&other.direction, ang_tol) {
            return false;
        }
        let offset = other.location - self.location;
        // Component perpendicular to the direction must be small.
        let along = offset.dot(&Vec::from_dir(self.direction));
        let perp = offset.magnitude_squared() - along * along;
        perp <= lin_tol * lin_tol
    }

    /// True when parallel or anti-parallel within `ang_tol`.
    #[inline]
    pub fn is_parallel(&self, other: &Self, ang_tol: f64) -> bool {
        self.direction.is_parallel(&other.direction, ang_tol)
    }

    /// True when perpendicular within `ang_tol`.
    #[inline]
    pub fn is_normal(&self, other: &Self, ang_tol: f64) -> bool {
        self.direction.is_normal(&other.direction, ang_tol)
    }

    /// Angle to `other`'s direction.
    #[inline]
    pub fn angle(&self, other: &Self) -> f64 {
        self.direction.angle(&other.direction)
    }

    /// Translate the origin by `v`.
    #[inline]
    pub fn translated(&self, v: &Vec) -> Self {
        Self::new(self.location.translated(v), self.direction)
    }

    /// Reverse the direction.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self::new(self.location, self.direction.reversed())
    }

    /// Apply a transform.
    pub fn transformed(&self, t: &Trsf) -> Self {
        Self::new(
            t.transform_point(&self.location),
            t.transform_dir(&self.direction),
        )
    }
}

impl Default for Ax1 {
    #[inline]
    fn default() -> Self {
        Self::new(Pnt::origin(), Dir::DX)
    }
}

/// A 3D line — an [`Ax1`] viewed as a geometric curve (OCCT `gp_Lin`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Lin {
    pos: Ax1,
}

impl Lin {
    /// Wrap an axis as a line.
    #[inline]
    pub const fn new(pos: Ax1) -> Self {
        Self { pos }
    }

    /// The underlying axis.
    #[inline]
    pub const fn position(&self) -> Ax1 {
        self.pos
    }

    /// A point on the line.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.pos.location()
    }

    /// The line direction.
    #[inline]
    pub const fn direction(&self) -> Dir {
        self.pos.direction()
    }

    /// The point at parameter `t`.
    #[inline]
    pub fn point(&self, t: f64) -> Pnt {
        self.pos.point(t)
    }

    /// True when `p` lies on the line within `tol`.
    pub fn contains(&self, p: &Pnt, tol: f64) -> bool {
        let d = *p - self.location();
        let proj = d.dot(&Vec::from_dir(self.direction()));
        let perp2 = d.magnitude_squared() - proj * proj;
        perp2 <= tol * tol
    }
}

impl From<Ax1> for Lin {
    #[inline]
    fn from(pos: Ax1) -> Self {
        Self::new(pos)
    }
}

/// A 2D line — an [`Ax2d`] viewed as a geometric curve (OCCT `gp_Lin2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Lin2d {
    pos: Ax2d,
}

impl Lin2d {
    /// Wrap a 2D axis as a line.
    #[inline]
    pub const fn new(pos: Ax2d) -> Self {
        Self { pos }
    }

    /// The underlying axis.
    #[inline]
    pub const fn position(&self) -> Ax2d {
        self.pos
    }

    /// A point on the line.
    #[inline]
    pub const fn location(&self) -> Pnt2d {
        self.pos.location()
    }

    /// The line direction.
    #[inline]
    pub const fn direction(&self) -> Dir2d {
        self.pos.direction()
    }

    /// The point at parameter `t`.
    #[inline]
    pub fn point(&self, t: f64) -> Pnt2d {
        let d = self.direction();
        self.location()
            .translated(&crate::vec::Vec2d::new(d.x() * t, d.y() * t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_point_along_direction() {
        let a = Ax1::new(Pnt::origin(), Dir::dz());
        assert_eq!(a.point(5.0), Pnt::new(0.0, 0.0, 5.0));
    }

    #[test]
    fn coaxial_detection() {
        let a = Ax1::new(Pnt::origin(), Dir::dz());
        let b = Ax1::new(Pnt::new(0.0, 0.0, 7.0), Dir::dz()); // same line
        let c = Ax1::new(Pnt::new(3.0, 0.0, 0.0), Dir::dz()); // parallel, offset
        assert!(a.is_coaxial(&b, 1e-9, 1e-9));
        assert!(!a.is_coaxial(&c, 1e-9, 1e-9));
    }

    #[test]
    fn transformed_axis_rotates() {
        let a = Ax1::new(Pnt::origin(), Dir::dx());
        let rot = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            core::f64::consts::FRAC_PI_2,
        );
        let b = a.transformed(&rot);
        assert!(b.direction().is_parallel(&Dir::dy(), 1e-9));
    }
}
