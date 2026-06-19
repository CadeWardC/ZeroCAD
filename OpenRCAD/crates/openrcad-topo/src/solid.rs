//! A topological solid — a closed volume bounded by a [`Shell`] (OCCT `TopoDS_Solid`).
//!
//! A solid is the outer boundary shell plus (optionally) internal voids. The
//! counting helpers here ([`vertex_count`](Solid::vertex_count),
//! [`edge_count`](Solid::edge_count)) **deduplicate** the entities shared between
//! adjacent faces — so a box, built from six independent four-edge faces, reports
//! 8 distinct vertices and 12 distinct edges rather than 48.

use openrcad_foundation::{BndBox, Trsf};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::Arc;

use crate::arena::{BRep, SolidData, SolidId};
use crate::edge::Edge;
use crate::shell::Shell;
use crate::vertex::Vertex;

/// A solid volume.
#[derive(Clone, Debug)]
pub struct Solid {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: SolidId,
}

impl Solid {
    /// Construct a Solid handle from a shared BRep storage and Solid ID.
    #[inline]
    pub fn from_id(brep: std::sync::Arc<BRep>, id: SolidId) -> Self {
        Self { brep, id }
    }

    /// Get the Solid ID in the arena.
    #[inline]
    pub fn id(&self) -> SolidId {
        self.id
    }

    /// Get the underlying BRep storage.
    #[inline]
    pub fn brep(&self) -> &Arc<BRep> {
        &self.brep
    }

    /// A solid bounded by `outer_shell`.
    #[inline]
    pub fn new(outer_shell: Shell) -> Self {
        let mut brep = BRep::new();
        let map = brep.merge(&outer_shell.brep);
        let new_shell = map.shells[&outer_shell.id];
        let id = brep.solids.insert(SolidData {
            shells: vec![new_shell],
        });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }

    /// The outer boundary shell.
    #[inline]
    pub fn shell(&self) -> Shell {
        let solid_data = &self.brep.solids[self.id];
        let shell_id = solid_data.shells[0];
        Shell {
            brep: self.brep.clone(),
            id: shell_id,
        }
    }

    /// All distinct [`Vertex`] locations (coincident endpoints merged).
    pub fn vertices(&self) -> Vec<Vertex> {
        let mut out: Vec<Vertex> = Vec::new();
        let mut seen = HashSet::new();
        for face in self.shell().faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    for v in [edge.start(), edge.end()] {
                        if seen.insert(vertex_key(&v)) {
                            out.push(v);
                        }
                    }
                }
            }
        }
        out
    }

    /// Number of distinct vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices().len()
    }

    /// All distinct [`Edge`]s (shared between two faces counted once). Two edges
    /// match when they share the same unordered endpoint pair.
    pub fn edges(&self) -> Vec<Edge> {
        let mut out: Vec<Edge> = Vec::new();
        let mut seen = HashSet::new();
        for face in self.shell().faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    if seen.insert(edge_key(&edge)) {
                        out.push(edge);
                    }
                }
            }
        }
        out
    }

    /// Number of distinct edges.
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.edges().len()
    }

    /// Number of faces.
    #[inline]
    pub fn face_count(&self) -> usize {
        self.shell().len()
    }

    /// The axis-aligned bounding box of all vertices.
    pub fn bounding_box(&self) -> BndBox {
        let mut b = BndBox::new();
        for v in self.vertices() {
            b.add(&v.point());
        }
        b
    }

    /// Apply `t` to the whole boundary shell.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
        }
    }
}

fn vertex_key(v: &Vertex) -> (i64, i64, i64) {
    point_key(&v.point())
}

fn edge_key(edge: &Edge) -> [(i64, i64, i64); 2] {
    let mut endpoints = [vertex_key(&edge.start()), vertex_key(&edge.end())];
    endpoints.sort_unstable();
    endpoints
}

fn point_key(p: &openrcad_foundation::Pnt) -> (i64, i64, i64) {
    let tol = openrcad_foundation::tolerance::CONFUSION;
    (
        (p.x() / tol).round() as i64,
        (p.y() / tol).round() as i64,
        (p.z() / tol).round() as i64,
    )
}

impl PartialEq for Solid {
    fn eq(&self, other: &Self) -> bool {
        self.shell() == other.shell()
    }
}

impl Serialize for Solid {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Solid {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id) = <(BRep, SolidId)>::deserialize(deserializer)?;
        Ok(Self {
            brep: Arc::new(brep),
            id,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use crate::face::Face;
    use crate::wire::Wire;
    use openrcad_foundation::Pnt;

    /// A degenerate "solid": two coincident triangles back-to-back sharing 3
    /// vertices and 3 edges. Good enough to exercise the dedup logic.
    fn flat_triangle_solid() -> Solid {
        let p = [
            Pnt::origin(),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(0.0, 1.0, 0.0),
        ];
        let wire = Wire::from_edges([
            Edge::between_points(p[0], p[1]),
            Edge::between_points(p[1], p[2]),
            Edge::between_points(p[2], p[0]),
        ]);
        let face = Face::new(None, wire);
        Solid::new(Shell::from_faces([face]))
    }

    #[test]
    fn triangle_dedups_to_3_vertices_and_3_edges() {
        let s = flat_triangle_solid();
        assert_eq!(s.vertex_count(), 3);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 1);
    }

    #[test]
    fn bounding_box_covers_vertices() {
        let s = flat_triangle_solid();
        let (lo, hi) = s.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::origin());
        assert_eq!(hi, Pnt::new(1.0, 1.0, 0.0));
    }
}
