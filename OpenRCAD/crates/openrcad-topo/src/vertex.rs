//! A topological vertex — a point in space (OCCT `TopoDS_Vertex`).

use openrcad_foundation::Pnt;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::arena::{BRep, VertexData, VertexId};

/// A vertex: a location.
#[derive(Clone, Debug)]
pub struct Vertex {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: VertexId,
}

impl Vertex {
    /// A vertex at `point`.
    #[inline]
    pub fn new(point: Pnt) -> Self {
        Self::new_with_tolerance(point, openrcad_foundation::tolerance::CONFUSION)
    }

    /// A vertex at `point` with a specific `tolerance`.
    #[inline]
    pub fn new_with_tolerance(point: Pnt, tolerance: f64) -> Self {
        let mut brep = BRep::new();
        let id = brep.vertices.insert(VertexData { point, tolerance });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }

    /// The vertex ID in the arena.
    #[inline]
    pub fn id(&self) -> VertexId {
        self.id
    }

    /// The location.
    #[inline]
    pub fn point(&self) -> Pnt {
        self.brep.vertices[self.id].point
    }

    /// The uncertainty tolerance.
    #[inline]
    pub fn tolerance(&self) -> f64 {
        self.brep.vertices[self.id].tolerance
    }

    /// True when this vertex coincides with `other` within `tol`.
    #[inline]
    pub fn is_equal(&self, other: &Self, tol: f64) -> bool {
        self.point().is_equal(&other.point(), tol)
    }
}

impl Default for Vertex {
    #[inline]
    fn default() -> Self {
        Self::new(Pnt::origin())
    }
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        self.point() == other.point()
    }
}

impl Serialize for Vertex {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let data = &self.brep.vertices[self.id];
        data.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Vertex {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = VertexData::deserialize(deserializer)?;
        let mut brep = BRep::new();
        let id = brep.vertices.insert(data);
        Ok(Self {
            brep: Arc::new(brep),
            id,
        })
    }
}
