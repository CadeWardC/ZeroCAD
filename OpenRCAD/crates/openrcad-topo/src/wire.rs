//! A topological wire — a connected sequence of [`Edge`]s (OCCT `TopoDS_Wire`).
//!
//! A wire traces a path through space. In a closed face it forms the boundary
//! loop; in an open shell it may be a free path. The edges are stored in order;
//! their orientation (handled by [`Edge::source`]/[`Edge::target`]) determines
//! the traversal direction.

use openrcad_foundation::Trsf;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::arena::{BRep, LoopData, LoopId, OrientedEdge};
use crate::edge::Edge;

/// An ordered list of [`Edge`]s forming a path or a loop.
#[derive(Clone, Debug)]
pub struct Wire {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: LoopId,
}

impl Wire {
    /// A wire from an ordered list of edges.
    #[inline]
    pub fn new(edges: Vec<Edge>) -> Self {
        Self::from_edges(edges)
    }

    /// Build from anything iterable of edges.
    pub fn from_edges<I: IntoIterator<Item = Edge>>(edges: I) -> Self {
        let mut brep = BRep::new();
        let mut merged: std::collections::HashMap<usize, crate::arena::MergeMap> =
            std::collections::HashMap::new();
        let mut new_edges = Vec::new();
        for edge in edges {
            let ptr = Arc::as_ptr(&edge.brep) as usize;
            // Since HashMap insertion returns a borrow, we avoid multiple mutable/immutable borrows.
            let map = merged.entry(ptr).or_insert_with(|| brep.merge(&edge.brep));
            let new_edge_id = map.edges[&edge.id];

            // The edge is stored once in its natural sense; this *use* of it in the
            // loop carries the traversal orientation (the single source of truth).
            new_edges.push(OrientedEdge {
                id: new_edge_id,
                orientation: edge.orientation,
            });
        }
        let id = brep.loops.insert(LoopData { edges: new_edges });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }

    /// The edges, in order.
    #[inline]
    pub fn edges(&self) -> Vec<Edge> {
        let loop_data = &self.brep.loops[self.id];
        loop_data
            .edges
            .iter()
            .map(|&oe| Edge {
                brep: self.brep.clone(),
                id: oe.id,
                orientation: oe.orientation,
            })
            .collect()
    }

    /// Number of edges.
    #[inline]
    pub fn len(&self) -> usize {
        self.brep.loops[self.id].edges.len()
    }

    /// True if there are no edges.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// True when the wire's start and end vertices coincide (within tolerance),
    /// i.e. it forms a closed loop.
    pub fn is_closed(&self) -> bool {
        let edges = self.edges();
        match (edges.first(), edges.last()) {
            (Some(a), Some(b)) => a
                .start()
                .is_equal(&b.end(), openrcad_foundation::tolerance::CONFUSION),
            _ => false,
        }
    }

    /// Apply `t` to every edge.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
        }
    }
}

impl Default for Wire {
    fn default() -> Self {
        let mut brep = BRep::new();
        let id = brep.loops.insert(LoopData { edges: Vec::new() });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }
}

impl PartialEq for Wire {
    fn eq(&self, other: &Self) -> bool {
        self.edges() == other.edges()
    }
}

impl Serialize for Wire {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Wire {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id) = <(BRep, LoopId)>::deserialize(deserializer)?;
        Ok(Self {
            brep: Arc::new(brep),
            id,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;

    #[test]
    fn square_loop_is_closed() {
        let w = Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
        ]);
        assert_eq!(w.len(), 4);
        assert!(w.is_closed());
    }

    #[test]
    fn open_path_is_not_closed() {
        let w = Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(2.0, 0.0, 0.0)),
        ]);
        assert!(!w.is_closed());
    }
}
