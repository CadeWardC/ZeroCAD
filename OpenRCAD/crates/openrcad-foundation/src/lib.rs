//! Foundation classes for OpenRCAD — the OCCT `gp` / `TKernel` / `TKMath` layer.
//!
//! This crate is the bedrock every other OpenRCAD crate stands on. It contains
//! no internal dependencies (only `serde`) and provides the geometric primitives
//! a CAD kernel needs:
//!
//! - Coordinate storage: [`Xy`] / [`Xyz`] (OCCT `gp_XY` / `gp_XYZ`).
//! - Points, vectors, directions: [`Pnt`]/[`Pnt2d`], [`Vec`]/[`Vec2d`],
//!   [`Dir`]/[`Dir2d`] (OCCT `gp_Pnt`, `gp_Vec`, `gp_Dir`, …).
//! - Axes and frames: [`Ax1`], [`Ax2`]/[`Ax3`], [`Ax2d`]/[`Ax22d`].
//! - Lines: [`Lin`], [`Lin2d`].
//! - Matrices and transforms: [`Mat`]/[`Mat2d`], [`Trsf`]/[`GTrsf`].
//! - Bounding boxes: [`BndBox`]/[`BndBox2d`] (OCCT `Bnd`).
//! - Tolerance policy: [`tolerance`] (OCCT `Precision`).
//!
//! ## Why `gp`-style types instead of a generic linear-algebra crate?
//!
//! Like OCCT's `gp` package, OpenRCAD keeps a **point** ([`Pnt`]) and a
//! **vector** ([`Vec`]) as distinct types. A point locates; a vector displaces.
//! You may add a vector to a point and subtract two points, but adding two
//! points is meaningless — so it is a compile error, not a silent wrong answer.
//! That distinction becomes load-bearing once topology orientation and the
//! scale factor of a [`Trsf`] are involved.
//!
//! [`Dir`] is a unit vector: constructing one from a non-unit triple normalizes
//! it, mirroring `gp_Dir`'s invariant.
#![forbid(unsafe_code)]

pub mod axis;
pub mod bnd;
pub mod dir;
pub mod double_double;
pub mod frame;
pub mod interval;
pub mod mat;
pub mod pnt;
pub mod predicates;
pub mod tolerance;
pub mod trsf;
pub mod vec;
pub mod xyz;

pub use axis::{Ax1, Lin, Lin2d};
pub use bnd::{BndBox, BndBox2d};
pub use dir::{Dir, Dir2d};
pub use double_double::DoubleDouble;
pub use frame::{Ax2, Ax22d, Ax2d, Ax3};
pub use interval::{Interval, Interval2, Interval3};
pub use mat::{Mat, Mat2d};
pub use pnt::{Pnt, Pnt2d};
pub use trsf::{GTrsf, Trsf, TrsfForm};
pub use vec::{Vec, Vec2d};
pub use xyz::{Xy, Xyz};
