pub mod expr;
pub mod geometry;
pub mod mock_kernel;
pub mod parametric;
pub mod sketch;
pub mod stl;
pub mod units;

/// How many segments a full circle is discretized into. This is the single
/// source of truth shared by sketch arrangement, ellipse faceting, cylinder
/// solids and cylinder wireframes — they must agree so a sketched circle and
/// its extruded/booleaned solid line up. Changing it here changes all of them.
pub const CIRCLE_SEGS: usize = 48;

// Re-export common structures for easy access
pub use expr::eval;
pub use geometry::{CoordinateSystem, SketchPlane, Vec3};
pub use mock_kernel::MockMesh;
pub use parametric::{EdgeRef, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, Variable};
pub use sketch::{
    build_sketch_curves, detect_regions, effective_curves, Circle, CornerKind, CornerMod,
    Dimension, LineSegment, Region, SketchCurves, SketchShape,
};
pub use stl::{meshes_to_binary_stl, write_binary_stl};
pub use units::{Parameter, Unit};
