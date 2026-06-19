#![forbid(unsafe_code)]
//! Primitive solid builders for OpenRCAD (OCCT `TKPrim`).
//!
//! Each builder turns a handful of parameters into a watertight [`Solid`] built
//! from analytic [`openrcad_geom::GeomSurface`]s and [`openrcad_topo::Edge`]s /
//! [`openrcad_topo::Wire`]s:
//!
//! - [`make_box`] / [`make_wedge`] — planar prisms (6 planar faces).
//! - [`make_cylinder`] — cylindrical wall + two planar caps.
//! - [`make_cone`] — conical wall + cap(s), with a sharp-apex degenerate case.
//! - [`make_sphere`] — a single spherical surface split into eight faces.
//!
//! Curved primitives split their circular rims into arcs so that no two edges
//! share both endpoints (the endpoint-based deduplication in [`openrcad_topo::Solid`]
//! would otherwise collapse them). Every primitive satisfies the Euler–Poincaré
//! invariant V − E + F = 2.

mod common;

pub mod box_solid;
pub mod cone;
pub mod cylinder;
pub mod sphere;
pub mod wedge;

pub use box_solid::make_box;
pub use cone::make_cone;
pub use cylinder::make_cylinder;
pub use sphere::make_sphere;
pub use wedge::make_wedge;
