//! Flat B-Rep storage utilizing generational index arenas (`slotmap`).
//!
//! Stores all elements (Vertices, Edges, Loops, Faces, Shells, Solids)
//! in flat pools. Adjacency is stored via versioned keys (e.g. `VertexId`, `EdgeId`)
//! which avoids circular pointers, references, or locks, and enables cache-friendly
//! traversal and lock-free parallel execution (via Arc).

use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SlotMap};

use crate::orientation::Orientation;
use openrcad_foundation::Pnt;
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};

new_key_type! {
    /// Generational index for a Vertex.
    pub struct VertexId;
    /// Generational index for an Edge.
    pub struct EdgeId;
    /// Generational index for a Loop (Wire).
    pub struct LoopId;
    /// Generational index for a Face.
    pub struct FaceId;
    /// Generational index for a Shell.
    pub struct ShellId;
    /// Generational index for a Solid.
    pub struct SolidId;
}

/// The geometric coordinate data of a Vertex in the B-Rep.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VertexData {
    /// The coordinates of the vertex.
    pub point: Pnt,
    /// The local uncertainty tolerance of this vertex.
    pub tolerance: f64,
}

/// The bounding curve and parameter data of an Edge in the B-Rep.
///
/// An edge is stored once, in its natural sense: it always runs from `start` to
/// `end` along the curve's increasing parameter (`first <= last`). It carries **no
/// orientation of its own** — traversal direction is a property of each *use* of
/// the edge within a loop, recorded per-use in [`OrientedEdge`]. This makes the
/// co-edge the single source of truth for orientation, so an edge shared by two
/// loops traversed in opposite directions stays unambiguous.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeData {
    /// The supporting 3D curve (optional if degenerate).
    pub curve: Option<GeomCurve>,
    /// The parameter of the start vertex on the curve.
    pub first: f64,
    /// The parameter of the end vertex on the curve.
    pub last: f64,
    /// The start vertex index.
    pub start: VertexId,
    /// The end vertex index.
    pub end: VertexId,
    /// The local uncertainty tolerance of this edge.
    pub tolerance: f64,
}
/// An edge reference with its loop-specific traversal orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OrientedEdge {
    /// The edge ID.
    pub id: EdgeId,
    /// The traversal orientation relative to the edge's natural direction.
    pub orientation: Orientation,
}

/// The ordered list of edges forming a Loop (Wire) in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoopData {
    /// The boundary edges in sequence.
    pub edges: Vec<OrientedEdge>,
}

/// The parametric surface and boundary loops of a Face in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceData {
    /// The carrying surface.
    pub surface: Option<GeomSurface>,
    /// The outer boundary loop.
    pub outer_wire: Option<LoopId>,
    /// The inner boundary loops (holes).
    pub inner_wires: Vec<LoopId>,
    /// The orientation of the face normal.
    pub orientation: Orientation,
}

/// The faces belonging to a Shell in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShellData {
    /// The face indices.
    pub faces: Vec<FaceId>,
}

/// The boundary shells of a Solid in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolidData {
    /// The boundary shell indices.
    pub shells: Vec<ShellId>,
}

/// Central topological arena containing all data of a CAD shape.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BRep {
    /// All vertices.
    pub vertices: SlotMap<VertexId, VertexData>,
    /// All edges.
    pub edges: SlotMap<EdgeId, EdgeData>,
    /// All loops.
    pub loops: SlotMap<LoopId, LoopData>,
    /// All faces.
    pub faces: SlotMap<FaceId, FaceData>,
    /// All shells.
    pub shells: SlotMap<ShellId, ShellData>,
    /// All solids.
    pub solids: SlotMap<SolidId, SolidData>,
}

impl BRep {
    /// Create an empty B-Rep arena.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Merges another B-Rep's contents into `self` while:
    /// 1. Re-mapping keys correctly.
    /// 2. Deduplicating vertices within Precision confusion tolerance (coincident).
    /// 3. Deduplicating edges sharing the same start/end vertices and curves.
    pub fn merge(&mut self, other: &Self) -> MergeMap {
        let mut map = MergeMap::new();

        // 1. Merge and deduplicate vertices
        for (v_id, v_data) in &other.vertices {
            let mut matched = None;
            for (self_v_id, self_v_data) in &self.vertices {
                if self_v_data
                    .point
                    .is_equal(&v_data.point, openrcad_foundation::tolerance::CONFUSION)
                {
                    matched = Some(self_v_id);
                    break;
                }
            }
            let new_id = if let Some(m_id) = matched {
                m_id
            } else {
                self.vertices.insert(*v_data)
            };
            map.vertices.insert(v_id, new_id);
        }

        // 2. Merge and deduplicate edges
        for (e_id, e_data) in &other.edges {
            let new_start = map.vertices[&e_data.start];
            let new_end = map.vertices[&e_data.end];

            let mut matched = None;
            for (self_e_id, self_e_data) in &self.edges {
                let endpoints_match = (self_e_data.start == new_start
                    && self_e_data.end == new_end)
                    || (self_e_data.start == new_end && self_e_data.end == new_start);
                if endpoints_match && self_e_data.curve == e_data.curve {
                    matched = Some(self_e_id);
                    break;
                }
            }

            let new_id = if let Some(m_id) = matched {
                m_id
            } else {
                self.edges.insert(EdgeData {
                    curve: e_data.curve.clone(),
                    first: e_data.first,
                    last: e_data.last,
                    start: new_start,
                    end: new_end,
                    tolerance: e_data.tolerance,
                })
            };
            map.edges.insert(e_id, new_id);
        }

        // 3. Merge loops
        for (l_id, l_data) in &other.loops {
            let new_edges: Vec<OrientedEdge> = l_data
                .edges
                .iter()
                .map(|&oe| OrientedEdge {
                    id: map.edges[&oe.id],
                    orientation: oe.orientation,
                })
                .collect();
            let new_id = self.loops.insert(LoopData { edges: new_edges });
            map.loops.insert(l_id, new_id);
        }

        // 4. Merge faces
        for (f_id, f_data) in &other.faces {
            let new_outer = f_data.outer_wire.map(|w| map.loops[&w]);
            let new_inner: Vec<LoopId> =
                f_data.inner_wires.iter().map(|&w| map.loops[&w]).collect();
            let new_id = self.faces.insert(FaceData {
                surface: f_data.surface.clone(),
                outer_wire: new_outer,
                inner_wires: new_inner,
                orientation: f_data.orientation,
            });
            map.faces.insert(f_id, new_id);
        }

        // 5. Merge shells
        for (s_id, s_data) in &other.shells {
            let new_faces: Vec<FaceId> = s_data.faces.iter().map(|&f| map.faces[&f]).collect();
            let new_id = self.shells.insert(ShellData { faces: new_faces });
            map.shells.insert(s_id, new_id);
        }

        // 6. Merge solids
        for (so_id, so_data) in &other.solids {
            let new_shells: Vec<ShellId> = so_data.shells.iter().map(|&s| map.shells[&s]).collect();
            let new_id = self.solids.insert(SolidData { shells: new_shells });
            map.solids.insert(so_id, new_id);
        }

        map
    }

    /// Discard every face not in `keep`, plus every loop, edge, and vertex no
    /// longer reachable from a kept face. Any shell that references a dropped
    /// face — and any solid that references a dropped shell — is removed too, so
    /// the arena never keeps a dangling reference. Callers that prune faces are
    /// expected to (re)build the shell they want afterwards.
    ///
    /// This is the garbage collector for [`BRep::merge`]: `merge` copies *all*
    /// entities of a source arena, so assembling a face from edges that live in
    /// another face's arena (as prism/sweep do) drags that whole face — and its
    /// shell — in as an orphan. Sewing then leaves the arena holding faces the
    /// shell never references — and worse, orphan faces can share a loop with a
    /// real face, so a later `partition_face` that removes that loop leaves the
    /// orphan dangling and any full-arena traversal (BVH build, validation,
    /// `merge`) panics. Pruning to the reachable set keeps the arena in
    /// lock-step with the shell.
    pub fn retain_faces(&mut self, keep: &[FaceId]) {
        use std::collections::HashSet;
        let keep_faces: HashSet<FaceId> = keep.iter().copied().collect();

        let mut keep_loops: HashSet<LoopId> = HashSet::new();
        for &f in &keep_faces {
            if let Some(fd) = self.faces.get(f) {
                if let Some(o) = fd.outer_wire {
                    keep_loops.insert(o);
                }
                keep_loops.extend(fd.inner_wires.iter().copied());
            }
        }

        let mut keep_edges: HashSet<EdgeId> = HashSet::new();
        for &l in &keep_loops {
            if let Some(ld) = self.loops.get(l) {
                keep_edges.extend(ld.edges.iter().map(|oe| oe.id));
            }
        }

        let mut keep_verts: HashSet<VertexId> = HashSet::new();
        for &e in &keep_edges {
            if let Some(ed) = self.edges.get(e) {
                keep_verts.insert(ed.start);
                keep_verts.insert(ed.end);
            }
        }

        self.faces.retain(|id, _| keep_faces.contains(&id));
        self.loops.retain(|id, _| keep_loops.contains(&id));
        self.edges.retain(|id, _| keep_edges.contains(&id));
        self.vertices.retain(|id, _| keep_verts.contains(&id));

        // Drop shells that referenced any dropped face, and solids that
        // referenced a dropped shell, so no dangling reference survives.
        self.shells
            .retain(|_, s| s.faces.iter().all(|f| keep_faces.contains(f)));
        let keep_shells: HashSet<ShellId> = self.shells.keys().collect();
        self.solids
            .retain(|_, so| so.shells.iter().all(|s| keep_shells.contains(s)));
    }

    /// Returns a transformed copy of this B-Rep, transforming all vertices, curves, and surfaces.
    pub fn transformed(&self, t: &openrcad_foundation::Trsf) -> Self {
        let mut cloned = self.clone();
        for (_, v_data) in &mut cloned.vertices {
            v_data.point = t.transform_point(&v_data.point);
        }
        for (_, e_data) in &mut cloned.edges {
            if let Some(ref mut c) = e_data.curve {
                *c = c.transformed(t);
            }
        }
        for (_, f_data) in &mut cloned.faces {
            if let Some(ref mut s) = f_data.surface {
                *s = s.transformed(t);
            }
        }
        cloned
    }
}

/// Helper mapping old keys to new keys after merging BReps.
pub struct MergeMap {
    /// Vertex mapping
    pub vertices: std::collections::HashMap<VertexId, VertexId>,
    /// Edge mapping
    pub edges: std::collections::HashMap<EdgeId, EdgeId>,
    /// Loop mapping
    pub loops: std::collections::HashMap<LoopId, LoopId>,
    /// Face mapping
    pub faces: std::collections::HashMap<FaceId, FaceId>,
    /// Shell mapping
    pub shells: std::collections::HashMap<ShellId, ShellId>,
    /// Solid mapping
    pub solids: std::collections::HashMap<SolidId, SolidId>,
}

impl MergeMap {
    fn new() -> Self {
        Self {
            vertices: std::collections::HashMap::new(),
            edges: std::collections::HashMap::new(),
            loops: std::collections::HashMap::new(),
            faces: std::collections::HashMap::new(),
            shells: std::collections::HashMap::new(),
            solids: std::collections::HashMap::new(),
        }
    }
}
