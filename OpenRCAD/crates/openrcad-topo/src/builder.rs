//! A mutable staging builder for topological BRep updates.
//!
//! Enables local Euler operators (e.g. splitting edges and faces)
//! on a mutable BRep state, which can then be sealed into an immutable Arc<BRep>.

use crate::arena::{BRep, EdgeData, EdgeId, FaceData, FaceId, LoopData, OrientedEdge, VertexId};
use crate::orientation::Orientation;
use openrcad_geom::GeomSurface;
use std::sync::Arc;

/// A mutable staging B-Rep builder.
#[derive(Clone, Debug, Default)]
pub struct BRepBuilder {
    brep: BRep,
}

impl BRepBuilder {
    /// Create an empty BRep staging builder.
    #[inline]
    pub fn new() -> Self {
        Self { brep: BRep::new() }
    }

    /// Create a staging builder from an existing BRep.
    #[inline]
    pub fn from_brep(brep: BRep) -> Self {
        Self { brep }
    }

    /// Seal the staging builder and return an immutable BRep.
    #[inline]
    pub fn build(self) -> Arc<BRep> {
        Arc::new(self.brep)
    }

    /// Access the underlying BRep read-only.
    #[inline]
    pub fn brep(&self) -> &BRep {
        &self.brep
    }

    /// Access the underlying BRep mutably.
    #[inline]
    pub fn brep_mut(&mut self) -> &mut BRep {
        &mut self.brep
    }

    /// Split an edge into two edges at a parameter `t` using an existing or new vertex `new_v`.
    ///
    /// Replaces the old edge `edge_id` in all referencing loops with the two new edges
    /// in the correct sequence depending on loop/edge traversal orientation.
    pub fn split_edge(&mut self, edge_id: EdgeId, new_v: VertexId, t: f64) -> (EdgeId, EdgeId) {
        let orig = self
            .brep
            .edges
            .get(edge_id)
            .expect("split_edge: edge not found")
            .clone();

        // 1. Insert the two new sub-edges.
        let e1_data = EdgeData {
            curve: orig.curve.clone(),
            first: orig.first,
            last: t,
            start: orig.start,
            end: new_v,
            tolerance: orig.tolerance,
        };

        let e2_data = EdgeData {
            curve: orig.curve.clone(),
            first: t,
            last: orig.last,
            start: new_v,
            end: orig.end,
            tolerance: orig.tolerance,
        };

        let e1_id = self.brep.edges.insert(e1_data);
        let e2_id = self.brep.edges.insert(e2_data);

        // 2. Replace the original edge in every loop with its two sub-edges, in
        //    the order that keeps that *specific* loop connected.
        //
        //    A single edge may be shared by two loops traversed in opposite
        //    directions (e.g. the seam between two cylinder wall faces). Because
        //    `EdgeData` stores only one orientation, we cannot rely on it to pick
        //    the sub-edge order — instead we look at the preceding edge in each
        //    loop and emit `[start-side, end-side]` or its reverse so the chain
        //    `prev -> e1 -> e2 -> next` (or `prev -> e2 -> e1 -> next`) stays
        //    contiguous. This is what makes splitting shared seams robust.
        let loop_ids: Vec<_> = self
            .brep
            .loops
            .iter()
            .filter(|(_, l)| l.edges.iter().any(|oe| oe.id == edge_id))
            .map(|(id, _)| id)
            .collect();
        for lid in loop_ids {
            let old = self.brep.loops[lid].edges.clone();
            let n = old.len();
            let mut new_edges = Vec::with_capacity(n + 1);
            for &oe in &old {
                if oe.id != edge_id {
                    new_edges.push(oe);
                    continue;
                }
                if oe.orientation == Orientation::Reversed {
                    new_edges.push(OrientedEdge {
                        id: e2_id,
                        orientation: Orientation::Reversed,
                    });
                    new_edges.push(OrientedEdge {
                        id: e1_id,
                        orientation: Orientation::Reversed,
                    });
                } else {
                    new_edges.push(OrientedEdge {
                        id: e1_id,
                        orientation: Orientation::Forward,
                    });
                    new_edges.push(OrientedEdge {
                        id: e2_id,
                        orientation: Orientation::Forward,
                    });
                }
            }
            self.brep.loops[lid].edges = new_edges;
        }

        // 3. Remove the original edge.
        self.brep.edges.remove(edge_id);

        (e1_id, e2_id)
    }

    /// Split a face into two faces along a path of splitting edges.
    ///
    /// The splitting edges must form a simple path connecting two vertices on the outer loop of the face.
    /// Distributes any inner loops (holes) of the original face to the correct new face using 2D parameter-space containment.
    pub fn split_face(&mut self, face_id: FaceId, splitting_edges: &[EdgeId]) -> (FaceId, FaceId) {
        let face_data = self
            .brep
            .faces
            .get(face_id)
            .expect("split_face: face not found")
            .clone();
        let outer_loop_id = face_data
            .outer_wire
            .expect("split_face: face has no outer wire");
        let outer_loop = self
            .brep
            .loops
            .get(outer_loop_id)
            .expect("split_face: loop not found")
            .clone();

        let get_edge_endpoints = |brep: &BRep, oe: OrientedEdge| {
            let e = &brep.edges[oe.id];
            match oe.orientation {
                Orientation::Reversed => (e.end, e.start),
                _ => (e.start, e.end),
            }
        };

        // 1. Trace the vertices of the outer loop in order.
        let outer_edges = &outer_loop.edges;
        let mut outer_vertices = Vec::with_capacity(outer_edges.len());
        for &oe in outer_edges {
            let (start, _) = get_edge_endpoints(&self.brep, oe);
            outer_vertices.push(start);
        }

        // 2. Find endpoints of the splitting path (V_A and V_B).
        let first_split_edge = splitting_edges[0];
        let last_split_edge = splitting_edges[splitting_edges.len() - 1];
        let (v_a, _) = get_edge_endpoints(
            &self.brep,
            OrientedEdge {
                id: first_split_edge,
                orientation: Orientation::Forward,
            },
        );
        let (_, v_b) = get_edge_endpoints(
            &self.brep,
            OrientedEdge {
                id: last_split_edge,
                orientation: Orientation::Forward,
            },
        );

        let idx_a = outer_vertices
            .iter()
            .position(|&v| v == v_a)
            .expect("split_face: V_A not found on outer loop");
        let idx_b = outer_vertices
            .iter()
            .position(|&v| v == v_b)
            .expect("split_face: V_B not found on outer loop");

        let mut loop1_edges = Vec::new();
        let mut loop2_edges = Vec::new();

        let n = outer_edges.len();

        // Path 1: From idx_a to idx_b along the outer loop.
        let mut curr = idx_a;
        while curr != idx_b {
            loop1_edges.push(outer_edges[curr]);
            curr = (curr + 1) % n;
        }

        // Path 2: From idx_b to idx_a along the outer loop.
        let mut curr = idx_b;
        while curr != idx_a {
            loop2_edges.push(outer_edges[curr]);
            curr = (curr + 1) % n;
        }

        // 3. Connect split loops using the splitting path.
        // Loop 1 needs to go from v_b back to v_a: add reversed splitting edges.
        for &e_id in splitting_edges.iter().rev() {
            loop1_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Reversed,
            });
        }

        // Loop 2 needs to go from v_a to v_b: add forward splitting edges.
        for &e_id in splitting_edges {
            loop2_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Forward,
            });
        }

        // 4. Create new LoopIds in the BRep.
        let loop1_id = self.brep.loops.insert(LoopData { edges: loop1_edges });
        let loop2_id = self.brep.loops.insert(LoopData { edges: loop2_edges });

        // 5. Distribute holes (inner loops) using 2D point-in-polygon checks on planar surfaces.
        let mut face1_inners = Vec::new();
        let mut face2_inners = Vec::new();

        if let Some(GeomSurface::Plane(plane)) = &face_data.surface {
            // Reconstruct Loop 1 outer boundary in 2D parametric coordinates.
            let mut loop1_poly = Vec::new();
            for &oe in &self.brep.loops[loop1_id].edges {
                let (start_v, _) = get_edge_endpoints(&self.brep, oe);
                let p = self.brep.vertices[start_v].point;
                let diff = p - plane.location();
                let u = diff.dot(&openrcad_foundation::Vec::from_dir(
                    plane.position().x_direction(),
                ));
                let v = diff.dot(&openrcad_foundation::Vec::from_dir(
                    plane.position().y_direction(),
                ));
                loop1_poly.push((u, v));
            }

            for &inner_loop_id in &face_data.inner_wires {
                let inner_loop = &self.brep.loops[inner_loop_id];
                if let Some(&first_edge) = inner_loop.edges.first() {
                    let (start_v, _) = get_edge_endpoints(&self.brep, first_edge);
                    let p = self.brep.vertices[start_v].point;
                    let diff = p - plane.location();
                    let u = diff.dot(&openrcad_foundation::Vec::from_dir(
                        plane.position().x_direction(),
                    ));
                    let v = diff.dot(&openrcad_foundation::Vec::from_dir(
                        plane.position().y_direction(),
                    ));

                    if point_in_polygon_2d((u, v), &loop1_poly) {
                        face1_inners.push(inner_loop_id);
                    } else {
                        face2_inners.push(inner_loop_id);
                    }
                }
            }
        } else {
            // Fallback: allocate all inner wires to Face 1.
            face1_inners.extend(&face_data.inner_wires);
        }

        // 6. Create the two new faces.
        let face1_data = FaceData {
            surface: face_data.surface.clone(),
            outer_wire: Some(loop1_id),
            inner_wires: face1_inners,
            orientation: face_data.orientation,
        };
        let face2_data = FaceData {
            surface: face_data.surface.clone(),
            outer_wire: Some(loop2_id),
            inner_wires: face2_inners,
            orientation: face_data.orientation,
        };

        let face1_id = self.brep.faces.insert(face1_data);
        let face2_id = self.brep.faces.insert(face2_data);

        // 7. Remove original face and outer loop.
        self.brep.loops.remove(outer_loop_id);
        self.brep.faces.remove(face_id);

        // 8. Update all shells referencing the original face.
        for (_, shell_data) in &mut self.brep.shells {
            let mut new_faces = Vec::with_capacity(shell_data.faces.len() + 1);
            for &f in &shell_data.faces {
                if f == face_id {
                    new_faces.push(face1_id);
                    new_faces.push(face2_id);
                } else {
                    new_faces.push(f);
                }
            }
            shell_data.faces = new_faces;
        }

        (face1_id, face2_id)
    }
}

/// Robust Jordan curve theorem point-in-polygon containment test for 2D.
fn point_in_polygon_2d(q: (f64, f64), poly: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.1 > q.1) != (pj.1 > q.1))
            && (q.0 < (pj.0 - pi.0) * (q.1 - pi.1) / (pj.1 - pi.1 + 1e-15) + pi.0)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use crate::face::Face;
    use crate::wire::Wire;
    use openrcad_foundation::Pnt;
    use openrcad_geom::{GeomSurface, Plane};

    #[test]
    fn test_split_edge() {
        let e = Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(10.0, 0.0, 0.0));
        let w = Wire::from_edges([e.clone()]);
        let face = Face::new(None, w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let edge_id = builder.brep.edges.keys().next().unwrap();
        let loop_id = builder.brep.loops.keys().next().unwrap();

        // Create a new vertex at (5, 0, 0)
        let new_v = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Split edge at parameter 5.0 (midpoint)
        let (e1, e2) = builder.split_edge(edge_id, new_v, 5.0);

        // Verify that original edge is gone and the loop contains the two sub-edges.
        assert!(!builder.brep.edges.contains_key(edge_id));
        assert!(builder.brep.edges.contains_key(e1));
        assert!(builder.brep.edges.contains_key(e2));

        let l_edges = &builder.brep.loops[loop_id].edges;
        assert_eq!(l_edges.len(), 2);
        assert_eq!(l_edges[0].id, e1);
        assert_eq!(l_edges[1].id, e2);

        // Verify geometry bounds of sub-edges
        assert_eq!(builder.brep.edges[e1].first, 0.0);
        assert_eq!(builder.brep.edges[e1].last, 5.0);
        assert_eq!(builder.brep.edges[e2].first, 5.0);
        assert_eq!(builder.brep.edges[e2].last, 10.0);
    }

    #[test]
    fn split_edge_keeps_both_loops_contiguous_across_a_shared_seam() {
        // A single edge `e` (a -> b) shared by two triangular loops, used in
        // OPPOSITE senses: Forward in loop1 (a,b,c), Reversed in loop2 (b,a,d).
        // This is the cylinder-seam case. Splitting `e` must keep BOTH loops
        // tracing cleanly, which only works if orientation is a per-use property
        // of each co-edge rather than a single value stored on the shared edge.
        use crate::arena::{LoopData, OrientedEdge, VertexData};
        use openrcad_foundation::tolerance::CONFUSION;

        let mut brep = BRep::new();
        let mk_v = |brep: &mut BRep, x: f64| {
            brep.vertices.insert(VertexData {
                point: Pnt::new(x, 0.0, 0.0),
                tolerance: CONFUSION,
            })
        };
        let a = mk_v(&mut brep, 0.0);
        let b = mk_v(&mut brep, 10.0);
        let c = mk_v(&mut brep, 5.0);
        let d = mk_v(&mut brep, -5.0);

        let mk_e = |brep: &mut BRep, s, t| {
            brep.edges.insert(EdgeData {
                curve: None,
                first: 0.0,
                last: 1.0,
                start: s,
                end: t,
                tolerance: CONFUSION,
            })
        };
        let e = mk_e(&mut brep, a, b); // the shared seam
        let bc = mk_e(&mut brep, b, c);
        let ca = mk_e(&mut brep, c, a);
        let ad = mk_e(&mut brep, a, d);
        let db = mk_e(&mut brep, d, b);

        let fwd = |id| OrientedEdge {
            id,
            orientation: Orientation::Forward,
        };
        let rev = |id| OrientedEdge {
            id,
            orientation: Orientation::Reversed,
        };
        let loop1 = brep.loops.insert(LoopData {
            edges: vec![fwd(e), fwd(bc), fwd(ca)],
        });
        let loop2 = brep.loops.insert(LoopData {
            edges: vec![rev(e), fwd(ad), fwd(db)],
        });

        let mut builder = BRepBuilder::from_brep(brep);
        let m = builder.brep.vertices.insert(VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: CONFUSION,
        });
        builder.split_edge(e, m, 0.5);

        // Directed endpoints of a co-edge, honoring its per-use orientation.
        let dir_ends = |brep: &BRep, oe: OrientedEdge| {
            let ed = &brep.edges[oe.id];
            match oe.orientation {
                Orientation::Reversed => (ed.end, ed.start),
                _ => (ed.start, ed.end),
            }
        };
        let assert_contiguous = |brep: &BRep, lid| {
            let edges = &brep.loops[lid].edges;
            assert_eq!(edges.len(), 4, "seam split should grow the loop to 4 edges");
            for i in 0..edges.len() {
                let (_, end) = dir_ends(brep, edges[i]);
                let (next_start, _) = dir_ends(brep, edges[(i + 1) % edges.len()]);
                assert_eq!(
                    end, next_start,
                    "loop edge {i} does not connect to the next"
                );
            }
        };
        assert_contiguous(&builder.brep, loop1);
        assert_contiguous(&builder.brep, loop2);
    }

    #[test]
    fn test_split_face() {
        // Create a planar square face [0, 10] x [0, 10] in XY plane.
        let p0 = Pnt::new(0.0, 0.0, 0.0);
        let p1 = Pnt::new(10.0, 0.0, 0.0);
        let p2 = Pnt::new(10.0, 10.0, 0.0);
        let p3 = Pnt::new(0.0, 10.0, 0.0);

        let w = Wire::from_edges([
            Edge::between_points(p0, p1),
            Edge::between_points(p1, p2),
            Edge::between_points(p2, p3),
            Edge::between_points(p3, p0),
        ]);

        let plane = GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dz(),
        ));
        let face = Face::new(Some(plane), w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let face_id = builder.brep.faces.keys().next().unwrap();

        // Let's create a splitting edge from (5, 0, 0) to (5, 10, 0) dividing the square in half.
        // We must first split the bottom and top boundary edges so that we have vertices at the split endpoints.
        let bottom_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p0.x()).abs() < 1e-5 && (p.y() - p0.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let top_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p2.x()).abs() < 1e-5 && (p.y() - p2.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let v_bottom_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
        let v_top_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 10.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Split the bottom and top edges.
        let (_, _) = builder.split_edge(bottom_edge_id, v_bottom_split, 5.0);
        let (_, _) = builder.split_edge(top_edge_id, v_top_split, 5.0);

        // Now insert the splitting edge from v_bottom_split to v_top_split.
        let split_edge = Edge::between_points(Pnt::new(5.0, 0.0, 0.0), Pnt::new(5.0, 10.0, 0.0));
        let map_split = builder.brep.merge(&split_edge.brep);
        let new_split_edge_id = map_split.edges[&split_edge.id];

        // Now split the face!
        let (f1, f2) = builder.split_face(face_id, &[new_split_edge_id]);

        // Verify that original face is removed, and we have two new faces.
        assert!(!builder.brep.faces.contains_key(face_id));
        assert!(builder.brep.faces.contains_key(f1));
        assert!(builder.brep.faces.contains_key(f2));

        // Verify loop counts of the new faces.
        let w1_id = builder.brep.faces[f1].outer_wire.unwrap();
        let w2_id = builder.brep.faces[f2].outer_wire.unwrap();
        assert_eq!(builder.brep.loops[w1_id].edges.len(), 4);
        assert_eq!(builder.brep.loops[w2_id].edges.len(), 4);
    }
}
