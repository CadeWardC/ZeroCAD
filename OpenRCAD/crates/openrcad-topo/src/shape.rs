//! A type-erased topological [`Shape`] (OCCT `TopoDS_Shape`).
//!
//! OCCT's `TopoDS_Shape` is a single handle that may hold any of the topological
//! entity kinds; algorithms then dispatch on the dynamic type. [`Shape`] is the
//! Rust analogue — an owned sum type over [`Vertex`]/[`Edge`]/[`Wire`]/[`Face`]/
//! [`Shell`]/[`Solid`], plus a [`Compound`](Shape::Compound) for aggregates.

use serde::{Deserialize, Serialize};

use crate::edge::Edge;
use crate::face::Face;
use crate::shell::Shell;
use crate::solid::Solid;
use crate::vertex::Vertex;
use crate::wire::Wire;

/// Any topological entity, owned by value.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Shape {
    /// A single vertex.
    Vertex(Vertex),
    /// A single edge.
    Edge(Edge),
    /// A single wire.
    Wire(Wire),
    /// A single face.
    Face(Face),
    /// A shell.
    Shell(Shell),
    /// A solid.
    Solid(Solid),
    /// A heterogeneous collection of shapes (OCCT `TopoDS_Compound`).
    Compound(Vec<Shape>),
}

impl Shape {
    /// Wrap a vertex.
    #[inline]
    pub fn vertex(v: Vertex) -> Self {
        Self::Vertex(v)
    }
    /// Wrap an edge.
    #[inline]
    pub fn edge(e: Edge) -> Self {
        Self::Edge(e)
    }
    /// Wrap a wire.
    #[inline]
    pub fn wire(w: Wire) -> Self {
        Self::Wire(w)
    }
    /// Wrap a face.
    #[inline]
    pub fn face(f: Face) -> Self {
        Self::Face(f)
    }
    /// Wrap a shell.
    #[inline]
    pub fn shell(s: Shell) -> Self {
        Self::Shell(s)
    }
    /// Wrap a solid.
    #[inline]
    pub fn solid(s: Solid) -> Self {
        Self::Solid(s)
    }
    /// The spatial dimension of the entity: 0 (vertex) … 3 (solid).
    pub fn dimension(&self) -> u8 {
        match self {
            Self::Vertex(_) => 0,
            Self::Edge(_) => 1,
            Self::Wire(_) => 1,
            Self::Face(_) => 2,
            Self::Shell(_) => 2,
            Self::Solid(_) => 3,
            Self::Compound(_) => 3,
        }
    }
}

impl From<Vertex> for Shape {
    #[inline]
    fn from(v: Vertex) -> Self {
        Self::Vertex(v)
    }
}
impl From<Edge> for Shape {
    #[inline]
    fn from(e: Edge) -> Self {
        Self::Edge(e)
    }
}
impl From<Wire> for Shape {
    #[inline]
    fn from(w: Wire) -> Self {
        Self::Wire(w)
    }
}
impl From<Face> for Shape {
    #[inline]
    fn from(f: Face) -> Self {
        Self::Face(f)
    }
}
impl From<Shell> for Shape {
    #[inline]
    fn from(s: Shell) -> Self {
        Self::Shell(s)
    }
}
impl From<Solid> for Shape {
    #[inline]
    fn from(s: Solid) -> Self {
        Self::Solid(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimensions() {
        assert_eq!(Shape::vertex(Vertex::default()).dimension(), 0);
        assert_eq!(Shape::shell(Shell::default()).dimension(), 2);
        assert_eq!(Shape::solid(Solid::new(Shell::default())).dimension(), 3);
    }
}
