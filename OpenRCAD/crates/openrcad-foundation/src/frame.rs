//! Coordinate systems (frames) — [`Ax2`]/[`Ax3`] (3D) and [`Ax2d`]/[`Ax22d`] (2D).
//!
//! A frame is an origin plus an orientation: a place to stand and a set of
//! axes. [`Ax2`] is a right-handed 3D system (origin + main/"Z" direction + an
//! "X" direction); [`Ax3`] is the same but may be left-handed (used by surfaces
//! and 3D conics). The 2D twins are [`Ax2d`] and [`Ax22d`].

use serde::{Deserialize, Serialize};

use crate::dir::{Dir, Dir2d};
use crate::pnt::{Pnt, Pnt2d};
use crate::trsf::Trsf;

/// Pick a reference X-direction perpendicular to `main`, using OCCT's
/// `gp_Ax2(P, V)` heuristic: choose the coordinate axis *least aligned* with
/// `main` and cross it in, so the result is robustly non-degenerate.
fn pick_xdir(main: &Dir) -> Dir {
    let (a, b, c) = (main.x(), main.y(), main.z());
    let aabs = a.abs();
    let babs = b.abs();
    let cabs = c.abs();
    let (x, y, z) = if babs <= aabs && babs <= cabs {
        if aabs > cabs {
            (-c, 0.0, a)
        } else {
            (c, 0.0, -a)
        }
    } else if aabs <= babs && aabs <= cabs {
        if babs > cabs {
            (0.0, -c, b)
        } else {
            (0.0, c, -b)
        }
    } else {
        if aabs > babs {
            (-b, a, 0.0)
        } else {
            (b, -a, 0.0)
        }
    };
    Dir::new(x, y, z)
}

/// A 3D coordinate system, right-handed (OCCT `gp_Ax2`).
///
/// Origin + main ("Z") direction + "X" direction; "Y" is `main × x`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ax2 {
    /// The axis = origin + main (Z) direction.
    pub axis: crate::axis::Ax1,
    vxdir: Dir,
    vydir: Dir,
}

impl Ax2 {
    /// A frame at `location` whose main direction is `main`; the X direction is
    /// chosen automatically (see [`pick_xdir`]).
    pub fn new(location: Pnt, main: Dir) -> Self {
        let axis = crate::axis::Ax1::new(location, main);
        let vxdir = pick_xdir(&main);
        let vydir = main.cross(&vxdir);
        Self { axis, vxdir, vydir }
    }

    /// A frame at `location` with main direction `main` and an explicit
    /// X direction `xdir` (which must be perpendicular to `main`).
    pub fn new_axes(location: Pnt, main: Dir, xdir: Dir) -> Self {
        assert!(
            main.dot(&xdir).abs() < 1e-6,
            "Ax2::new_axes: xdir must be perpendicular to main"
        );
        let vydir = main.cross(&xdir);
        Self {
            axis: crate::axis::Ax1::new(location, main),
            vxdir: xdir,
            vydir,
        }
    }

    /// The origin.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.axis.location()
    }
    /// The main ("Z") direction.
    #[inline]
    pub const fn direction(&self) -> Dir {
        self.axis.direction()
    }
    /// The "X" direction.
    #[inline]
    pub const fn x_direction(&self) -> Dir {
        self.vxdir
    }
    /// The "Y" direction.
    #[inline]
    pub const fn y_direction(&self) -> Dir {
        self.vydir
    }
    /// The underlying axis (origin + main direction).
    #[inline]
    pub const fn axis(&self) -> crate::axis::Ax1 {
        self.axis
    }
}

/// A 3D coordinate system, right- or left-handed (OCCT `gp_Ax3`).
///
/// Like [`Ax2`] but carrying a handedness flag. This is the frame surfaces and
/// 3D conics are built on.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ax3 {
    axis: crate::axis::Ax1,
    vxdir: Dir,
    vydir: Dir,
    direct: bool,
}

impl Ax3 {
    /// A right-handed frame at `location` with main direction `main` (X chosen
    /// automatically).
    pub fn new(location: Pnt, main: Dir) -> Self {
        let vxdir = pick_xdir(&main);
        let vydir = main.cross(&vxdir);
        Self {
            axis: crate::axis::Ax1::new(location, main),
            vxdir,
            vydir,
            direct: true,
        }
    }

    /// A frame at `location` with main direction `main` and explicit `xdir`
    /// (must be perpendicular to `main`); always direct (right-handed).
    pub fn new_axes(location: Pnt, main: Dir, xdir: Dir) -> Self {
        assert!(
            main.dot(&xdir).abs() < 1e-6,
            "Ax3::new_axes: xdir must be perpendicular to main"
        );
        let vydir = main.cross(&xdir);
        Self {
            axis: crate::axis::Ax1::new(location, main),
            vxdir: xdir,
            vydir,
            direct: true,
        }
    }

    /// A frame with an explicit pair of X/Y directions. `direct` is set from
    /// whether `(xdir, ydir, main)` is right-handed.
    pub fn new_full(location: Pnt, main: Dir, xdir: Dir, ydir: Dir) -> Self {
        let direct = xdir.cross(&ydir).dot(&main) > 0.0;
        Self {
            axis: crate::axis::Ax1::new(location, main),
            vxdir: xdir,
            vydir: ydir,
            direct,
        }
    }

    /// The origin.
    #[inline]
    pub const fn location(&self) -> Pnt {
        self.axis.location()
    }
    /// The main ("Z") direction — also the surface normal of a plane built on
    /// this frame.
    #[inline]
    pub const fn direction(&self) -> Dir {
        self.axis.direction()
    }
    /// The "X" direction.
    #[inline]
    pub const fn x_direction(&self) -> Dir {
        self.vxdir
    }
    /// The "Y" direction.
    #[inline]
    pub const fn y_direction(&self) -> Dir {
        self.vydir
    }
    /// The underlying axis.
    #[inline]
    pub const fn axis(&self) -> crate::axis::Ax1 {
        self.axis
    }
    /// True if the frame is right-handed ("direct"), false if left-handed.
    #[inline]
    pub const fn is_direct(&self) -> bool {
        self.direct
    }

    /// Apply a transform: the origin is mapped as a point, each direction as a
    /// direction (ignoring translation and scale), and handedness is preserved.
    pub fn transformed(&self, t: &Trsf) -> Self {
        Self {
            axis: crate::axis::Ax1::new(
                t.transform_point(&self.location()),
                t.transform_dir(&self.direction()),
            ),
            vxdir: t.transform_dir(&self.vxdir),
            vydir: t.transform_dir(&self.vydir),
            direct: self.direct,
        }
    }
}

impl From<Ax2> for Ax3 {
    #[inline]
    fn from(a: Ax2) -> Self {
        Self {
            axis: a.axis,
            vxdir: a.vxdir,
            vydir: a.vydir,
            direct: true,
        }
    }
}

/// A 2D axis: origin + direction (OCCT `gp_Ax2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ax2d {
    location: Pnt2d,
    direction: Dir2d,
}

impl Ax2d {
    /// Create from a location and a direction.
    #[inline]
    pub const fn new(location: Pnt2d, direction: Dir2d) -> Self {
        Self {
            location,
            direction,
        }
    }

    /// The origin.
    #[inline]
    pub const fn location(&self) -> Pnt2d {
        self.location
    }
    /// The direction.
    #[inline]
    pub const fn direction(&self) -> Dir2d {
        self.direction
    }
}

/// A 2D coordinate system: origin + X and Y directions (OCCT `gp_Ax22d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ax22d {
    location: Pnt2d,
    vxdir: Dir2d,
    vydir: Dir2d,
}

impl Ax22d {
    /// A frame at `location` with X direction `xdir`; the Y direction is
    /// `xdir` rotated +90°.
    pub fn new(location: Pnt2d, xdir: Dir2d) -> Self {
        Self {
            location,
            vxdir: xdir,
            vydir: xdir.rotated_90(),
        }
    }

    /// A frame with an explicit location and X/Y directions.
    pub const fn new_axes(location: Pnt2d, xdir: Dir2d, ydir: Dir2d) -> Self {
        Self {
            location,
            vxdir: xdir,
            vydir: ydir,
        }
    }

    /// The origin.
    #[inline]
    pub const fn location(&self) -> Pnt2d {
        self.location
    }
    /// The X direction.
    #[inline]
    pub const fn x_direction(&self) -> Dir2d {
        self.vxdir
    }
    /// The Y direction.
    #[inline]
    pub const fn y_direction(&self) -> Dir2d {
        self.vydir
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ax2_xyz_are_orthonormal() {
        let f = Ax2::new(Pnt::origin(), Dir::dz());
        let x = f.x_direction();
        let y = f.y_direction();
        let z = f.direction();
        // Z = +Z by construction.
        assert_eq!(z, Dir::dz());
        // X ⊥ Y ⊥ Z.
        assert!(x.dot(&y).abs() < 1e-9);
        assert!(x.dot(&z).abs() < 1e-9);
        assert!(y.dot(&z).abs() < 1e-9);
        // Right-handed: X × Y = Z.
        assert!(x.cross(&y).is_parallel(&z, 1e-9));
    }

    #[test]
    fn ax3_arbitrary_main_is_orthonormal() {
        let f = Ax3::new(Pnt::origin(), Dir::new(1.0, 1.0, 1.0));
        let x = f.x_direction();
        let y = f.y_direction();
        let z = f.direction();
        assert!(x.dot(&z).abs() < 1e-9);
        assert!(y.dot(&z).abs() < 1e-9);
        assert!(f.is_direct());
    }

    #[test]
    fn ax22d_ydir_is_xdir_plus_90() {
        let f = Ax22d::new(Pnt2d::origin(), Dir2d::DX);
        assert_eq!(f.y_direction(), Dir2d::DY);
    }
}
