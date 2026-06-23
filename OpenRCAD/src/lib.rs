#![forbid(unsafe_code)]
//! OpenRCAD — a pure-Rust CAD kernel.
//!
//! A refactor of [OpenCASCADE]'s architecture, informed by [truck]. This crate
//! is the facade: depending on `openrcad` gives you the whole kernel through
//! its submodule re-exports.
//!
//! ```
//! use openrcad::foundation::{Pnt, Dir, Ax1, Trsf};
//! use openrcad::primitives::make_box;
//!
//! let solid = make_box(&Pnt::origin(), 10.0, 20.0, 30.0);
//! assert_eq!(solid.shell().faces().len(), 6);
//! ```
//!
//! [OpenCASCADE]: https://dev.opencascade.org/
//! [truck]: https://github.com/ricosjp/truck

pub use openrcad_algo as algo;
pub use openrcad_document as document;
pub use openrcad_exchange as exchange;
pub use openrcad_foundation as foundation;
pub use openrcad_geom as geom;
pub use openrcad_geom2d as geom2d;
pub use openrcad_mesh as mesh;
pub use openrcad_primitives as primitives;
pub use openrcad_sketch as sketch;
pub use openrcad_topo as topo;

/// The common imports for day-to-day modeling.
///
/// Glob-import this to get the core math types, the primitive builders, the
/// topology handles, and the fluent [`SolidExt`](crate::algo::SolidExt)
/// operations in one line:
///
/// ```
/// use openrcad::prelude::*;
///
/// // Build a box and round all of its edges with the fluent API.
/// let rounded = make_box(&Pnt::origin(), 10.0, 10.0, 10.0).fillet(1.0)?;
/// assert_eq!(rounded.face_count(), 26);
/// # Ok::<(), BlendError>(())
/// ```
pub mod prelude {
    pub use crate::algo::{
        boolean, chamfer, chamfer_edges, fillet, fillet_planar_edge, prism,
        rolling_ball_between_planar_faces, rolling_ball_fillet_edge, sew, shell_solid, sweep_prism,
        BlendError, BooleanOp, ChamferError, RollingBallBlend, RollingBallError, SolidExt,
        SweepError,
    };
    pub use crate::document::{
        load_zcad, read_zcad, save_zcad, write_zcad, CachedMesh, Document, DocumentError,
        FeatureId, LoadedZcad, Operation, SketchId, ZcadDocument, ZcadError, ZcadMetadata,
    };
    pub use crate::foundation::{Ax1, Ax2, Ax3, Dir, Pnt, Trsf};
    // OpenRCAD's 3D vector is `gp_Vec`-style `Vec`; alias it so a glob import of
    // the prelude never shadows `std::vec::Vec`.
    pub use crate::foundation::Vec as GeomVec;
    pub use crate::geom::{GeomCurve, GeomSurface};
    pub use crate::primitives::{make_box, make_cylinder, make_sphere};
    pub use crate::sketch::{EntityId, Profile, Sketch, SketchError, SketchPlane};
    pub use crate::topo::{Edge, Face, Shell, Solid, Vertex, Wire};
}

/// The workspace-wide version, as a constant, for feature/version checks.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
