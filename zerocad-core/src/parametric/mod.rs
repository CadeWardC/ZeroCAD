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

mod cut;
mod edge_mod;
mod eval;
mod extrude;
mod join;
mod types;

#[allow(unused_imports)]
pub(crate) use cut::*;
#[allow(unused_imports)]
pub(crate) use edge_mod::*;
#[allow(unused_imports)]
pub(crate) use eval::*;
#[allow(unused_imports)]
pub(crate) use extrude::*;
#[allow(unused_imports)]
pub(crate) use join::*;
pub use types::*;

#[cfg(test)]
mod tests;
