#![forbid(unsafe_code)]
//! Boundary-representation topology for OpenRCAD (OCCT `TKBRep`).
//!
//! The topology layer sits above [`openrcad_geom`] and describes *how* geometric
//! entities are connected into a model:
//!
//! - [`Vertex`] — a point in space (`TopoDS_Vertex`).
//! - [`Edge`] — a bounded piece of a curve (`TopoDS_Edge`).
//! - [`Wire`] — a sequence of edges (`TopoDS_Wire`).
//! - [`Face`] — a trimmed patch of a surface (`TopoDS_Face`).
//! - [`Shell`] — a set of faces (`TopoDS_Shell`).
//! - [`Solid`] — a closed volume (`TopoDS_Solid`).
//! - [`Shape`] — a type-erased handle over any of the above (`TopoDS_Shape`).
//!
//! ## Representation
//!
//! Current topology storage uses the `BRep` generational arenas in
//! [`arena`]. Handles carry an `Arc<BRep>` plus an arena key, so constructed
//! shapes are cheap to clone and safe to traverse without pointer cycles.
//!
//! Geometry is stored **by value** inside arena records. Counting helpers on
//! [`Solid`] (`vertex_count`, `edge_count`) merge coincident boundary entities
//! using the shared tolerance policy.

pub mod arena;
pub mod builder;
pub mod edge;
pub mod face;
pub mod orientation;
pub mod shape;
pub mod shell;
pub mod solid;
pub mod validate;
pub mod vertex;
pub mod wire;

pub use arena::{BRep, EdgeId, FaceId, LoopId, ShellId, SolidId, VertexId};
pub use builder::BRepBuilder;

pub use edge::Edge;
pub use face::Face;
pub use orientation::Orientation;
pub use shape::Shape;
pub use shell::Shell;
pub use solid::Solid;
pub use validate::{HealthError, HealthReport, HealthWarning, ValidationError};
pub use vertex::Vertex;
pub use wire::Wire;

/// Re-export of the foundation tolerance policy, so topology consumers don't
/// have to depend on `openrcad-foundation` directly for the constant.
pub use openrcad_foundation::tolerance;
