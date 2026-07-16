use crate::geometry::{CoordinateSystem, Vec3};
use crate::mock_kernel::{EdgeCurveHint, KernelSolid, MeshTopologyEdgeRef, MockMesh};
use crate::sketch::{
    build_region_provenance, detect_regions, Circle, Region, RegionProvenance,
    RegionProvenanceFragment, ShapeLoop, SketchCurves,
};
use crate::units::Unit;
use petgraph::algo::toposort;
use petgraph::graph::{DiGraph, NodeIndex};
use std::cell::RefCell;
use std::collections::HashMap;
use std::hash::Hasher;

/// Shared New-body commit service. Feature evaluators build and validate their
/// tool geometry first, then enter the common body-operation boundary through
/// this function rather than mutating the live body collection ad hoc.
fn apply_new(live: &mut Vec<LiveBody>, body: LiveBody) {
    live.push(body);
}

mod cut;
mod datum;
mod edge_mod;
mod eval;
mod extrude;
mod join;
mod phase35;
pub mod thread;
pub mod topo_name;
mod types;

#[allow(unused_imports)]
pub(crate) use cut::*;
pub use edge_mod::edge_wedge_is_concave_mesh;
#[allow(unused_imports)]
pub(crate) use edge_mod::*;
#[allow(unused_imports)]
pub(crate) use eval::*;
#[allow(unused_imports)]
pub(crate) use extrude::*;
pub use extrude::{boolean_region_plan, complete_selected_circles, BooleanRegionPlan};
#[allow(unused_imports)]
pub(crate) use join::*;
pub use phase35::body_bounds_center;
pub(crate) use phase35::*;
pub use thread::{
    closest_preset, default_depth_mm, ThreadPreset, ThreadStandard, METRIC_COARSE, UNIFIED_COARSE,
    UNIFIED_FINE,
};
pub use types::*;

#[cfg(test)]
mod tests;
