//! Local Euler operators (OCCT/Mäntylä-style `MEV`/`MEF`/`KEV`/`KEF`) over a
//! [`BRepBuilder`].
//!
//! Euler operators are the atomic, invariant-preserving edits the higher-level
//! modeling transactions (fillets, chamfers, local face edits) compose from.
//! Each one surgically mutates the arena graph in the neighborhood of an edit
//! and preserves the **Euler–Poincaré invariant**
//!
//! ```text
//! V − E + F = 2(S − G) + H
//! ```
//!
//! by construction. Their raw count deltas are:
//!
//! | op  | ΔV | ΔE | ΔF | Δχ |
//! |-----|----|----|----|----|
//! | MEV | +1 | +1 |  0 |  0 |
//! | MEF |  0 | +1 | +1 |  0 |
//! | KEV | −1 | −1 |  0 |  0 |
//! | KEF |  0 | −1 | −1 |  0 |
//!
//! `MEV`/`KEV` and `MEF`/`KEF` are mutual inverses.
//!
//! ## Known limitation
//!
//! Topology stores a **single** [`Orientation`] per [`EdgeData`], so an edge
//! cannot appear in two loops with opposite sense under one `EdgeId`. `MEF`
//! therefore references the *same* connecting `EdgeId` in both child loops
//! (one of the two traversals runs against the stored orientation). This keeps
//! the count algebra exact (ΔE = +1) and is faithfully undone by `KEF`; a fully
//! oriented half-edge representation would store the reversed twin separately.

use openrcad_foundation::Pnt;
use openrcad_geom::{GeomCurve, Line};
use openrcad_topo::arena::{
    BRep, EdgeData, EdgeId, FaceData, FaceId, LoopData, LoopId, OrientedEdge, VertexData, VertexId,
};
use openrcad_topo::{BRepBuilder, Orientation};

/// The Euler characteristic χ = V − E + F computed from raw arena entity counts.
#[inline]
pub fn euler_characteristic(brep: &BRep) -> i64 {
    brep.vertices.len() as i64 - brep.edges.len() as i64 + brep.faces.len() as i64
}

/// Build a straight-line [`GeomCurve`] between two points, returning the curve
/// (or `None` if degenerate) and its `(first, last)` parameter range.
fn line_curve(p0: Pnt, p1: Pnt) -> (Option<GeomCurve>, f64, f64) {
    let disp = p1 - p0;
    let len = disp.magnitude();
    match disp.normalized() {
        Some(dir) if len > openrcad_foundation::tolerance::CONFUSION => (
            Some(GeomCurve::line(Line::from_point_dir(p0, dir))),
            0.0,
            len,
        ),
        _ => (None, 0.0, 0.0),
    }
}

/// The start vertex of `oe` accounting for its loop-specific traversal orientation.
fn oriented_edge_source(brep: &BRep, oe: OrientedEdge) -> VertexId {
    let e = &brep.edges[oe.id];
    match oe.orientation {
        Orientation::Reversed => e.end,
        _ => e.start,
    }
}

/// **MEV** — *Make Edge & Vertex*. Spawns a new vertex at `new_point` and a
/// spur edge joining the existing `from` vertex to it, appended to `loop_id`.
///
/// Returns the new `(VertexId, EdgeId)`. Counts: ΔV = +1, ΔE = +1.
pub fn mev(
    builder: &mut BRepBuilder,
    loop_id: LoopId,
    from: VertexId,
    new_point: Pnt,
) -> (VertexId, EdgeId) {
    let p0 = builder.brep().vertices[from].point;
    let v_new = builder.brep_mut().vertices.insert(VertexData {
        point: new_point,
        tolerance: openrcad_foundation::tolerance::CONFUSION,
    });
    let (curve, first, last) = line_curve(p0, new_point);
    let e = builder.brep_mut().edges.insert(EdgeData {
        curve,
        first,
        last,
        start: from,
        end: v_new,
        tolerance: openrcad_foundation::tolerance::CONFUSION,
    });
    if let Some(l) = builder.brep_mut().loops.get_mut(loop_id) {
        l.edges.push(OrientedEdge {
            id: e,
            orientation: Orientation::Forward,
        });
    }
    (v_new, e)
}

/// **KEV** — *Kill Edge & Vertex*. The inverse of [`mev`]: removes the spur
/// `edge` (from every loop) and the degree-1 `vertex` it reaches.
///
/// Counts: ΔV = −1, ΔE = −1.
pub fn kev(builder: &mut BRepBuilder, vertex: VertexId, edge: EdgeId) {
    for (_, l) in &mut builder.brep_mut().loops {
        l.edges.retain(|&oe| oe.id != edge);
    }
    builder.brep_mut().edges.remove(edge);
    builder.brep_mut().vertices.remove(vertex);
}

/// **MEF** — *Make Edge & Face*. Connects two vertices `v1`, `v2` that both lie
/// on the outer loop of `face_id` with a new edge, splitting the loop into two
/// and creating a new face for the second half.
///
/// Returns the new `(FaceId, EdgeId)`. Counts: ΔE = +1, ΔF = +1.
pub fn mef(
    builder: &mut BRepBuilder,
    face_id: FaceId,
    v1: VertexId,
    v2: VertexId,
) -> (FaceId, EdgeId) {
    let face_data = builder.brep().faces[face_id].clone();
    let loop_id = face_data.outer_wire.expect("mef: face has no outer loop");
    let loop_edges = builder.brep().loops[loop_id].edges.clone();
    let n = loop_edges.len();
    assert!(n >= 2, "mef: degenerate loop");

    let starts: Vec<VertexId> = loop_edges
        .iter()
        .map(|&oe| oriented_edge_source(builder.brep(), oe))
        .collect();
    let idx1 = starts
        .iter()
        .position(|&v| v == v1)
        .expect("mef: v1 not on the outer loop");
    let idx2 = starts
        .iter()
        .position(|&v| v == v2)
        .expect("mef: v2 not on the outer loop");
    assert_ne!(idx1, idx2, "mef: v1 and v2 must be distinct loop vertices");

    // Walk the loop from v1 up to (not including) v2, and from v2 back to v1.
    let mut path_a = Vec::new();
    let mut c = idx1;
    while c != idx2 {
        path_a.push(loop_edges[c]);
        c = (c + 1) % n;
    }
    let mut path_b = Vec::new();
    let mut c = idx2;
    while c != idx1 {
        path_b.push(loop_edges[c]);
        c = (c + 1) % n;
    }

    let p1 = builder.brep().vertices[v1].point;
    let p2 = builder.brep().vertices[v2].point;
    let (curve, first, last) = line_curve(p1, p2);
    let e = builder.brep_mut().edges.insert(EdgeData {
        curve,
        first,
        last,
        start: v1,
        end: v2,
        tolerance: openrcad_foundation::tolerance::CONFUSION,
    });

    // The connecting edge closes both loops (shared id — see module docs).
    path_a.push(OrientedEdge {
        id: e,
        orientation: Orientation::Reversed,
    });
    path_b.push(OrientedEdge {
        id: e,
        orientation: Orientation::Forward,
    });
    let loop1_id = builder.brep_mut().loops.insert(LoopData { edges: path_a });
    let loop2_id = builder.brep_mut().loops.insert(LoopData { edges: path_b });

    builder
        .brep_mut()
        .faces
        .get_mut(face_id)
        .unwrap()
        .outer_wire = Some(loop1_id);
    let new_face = builder.brep_mut().faces.insert(FaceData {
        surface: face_data.surface.clone(),
        outer_wire: Some(loop2_id),
        inner_wires: Vec::new(),
        orientation: face_data.orientation,
    });
    builder.brep_mut().loops.remove(loop_id);

    for (_, shell) in &mut builder.brep_mut().shells {
        if shell.faces.contains(&face_id) {
            shell.faces.push(new_face);
        }
    }

    (new_face, e)
}

/// **KEF** — *Kill Edge & Face*. The inverse of [`mef`]: removes the connecting
/// `edge` and merges `new_face` back into `face_id`.
///
/// Counts: ΔE = −1, ΔF = −1.
pub fn kef(builder: &mut BRepBuilder, face_id: FaceId, new_face: FaceId, edge: EdgeId) {
    let l1 = builder.brep().faces[face_id]
        .outer_wire
        .expect("kef: face has no outer loop");
    let l2 = builder.brep().faces[new_face]
        .outer_wire
        .expect("kef: new_face has no outer loop");

    let mut merged: Vec<OrientedEdge> = builder.brep().loops[l1]
        .edges
        .iter()
        .copied()
        .filter(|&oe| oe.id != edge)
        .collect();
    merged.extend(
        builder.brep().loops[l2]
            .edges
            .iter()
            .copied()
            .filter(|&oe| oe.id != edge),
    );
    let new_loop = builder.brep_mut().loops.insert(LoopData { edges: merged });

    builder
        .brep_mut()
        .faces
        .get_mut(face_id)
        .unwrap()
        .outer_wire = Some(new_loop);
    builder.brep_mut().loops.remove(l1);
    builder.brep_mut().loops.remove(l2);
    builder.brep_mut().edges.remove(edge);
    builder.brep_mut().faces.remove(new_face);

    for (_, shell) in &mut builder.brep_mut().shells {
        shell.faces.retain(|&f| f != new_face);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir, Pnt};
    use openrcad_geom::{GeomSurface, Plane};
    use openrcad_topo::{Edge, Face, Wire};

    /// A planar unit-square face: V=4, E=4, F=1, χ=1.
    fn square_builder() -> (BRepBuilder, FaceId, LoopId, Vec<VertexId>) {
        let p = [
            Pnt::origin(),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(1.0, 1.0, 0.0),
            Pnt::new(0.0, 1.0, 0.0),
        ];
        let wire = Wire::from_edges([
            Edge::between_points(p[0], p[1]),
            Edge::between_points(p[1], p[2]),
            Edge::between_points(p[2], p[3]),
            Edge::between_points(p[3], p[0]),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let face = Face::new(Some(surf), wire);
        let builder = BRepBuilder::from_brep((**face.brep()).clone());
        let face_id = builder.brep().faces.keys().next().unwrap();
        let loop_id = builder.brep().faces[face_id].outer_wire.unwrap();
        // Loop-ordered start vertices, so callers can pick known corners.
        let verts: Vec<VertexId> = builder.brep().loops[loop_id]
            .edges
            .iter()
            .map(|&oe| oriented_edge_source(builder.brep(), oe))
            .collect();
        (builder, face_id, loop_id, verts)
    }

    #[test]
    fn mev_preserves_invariant_and_kev_inverts() {
        let (mut b, _f, loop_id, verts) = square_builder();
        let chi0 = euler_characteristic(b.brep());
        let (v0, e0, f0) = (
            b.brep().vertices.len(),
            b.brep().edges.len(),
            b.brep().faces.len(),
        );

        let (v_new, e_new) = mev(&mut b, loop_id, verts[0], Pnt::new(0.5, 0.5, 0.0));
        assert_eq!(b.brep().vertices.len(), v0 + 1);
        assert_eq!(b.brep().edges.len(), e0 + 1);
        assert_eq!(b.brep().faces.len(), f0);
        assert_eq!(euler_characteristic(b.brep()), chi0, "MEV must preserve χ");

        kev(&mut b, v_new, e_new);
        assert_eq!(b.brep().vertices.len(), v0);
        assert_eq!(b.brep().edges.len(), e0);
        assert_eq!(b.brep().faces.len(), f0);
        assert_eq!(euler_characteristic(b.brep()), chi0, "KEV must restore χ");
    }

    #[test]
    fn mef_preserves_invariant_and_kef_inverts() {
        let (mut b, face_id, _loop, verts) = square_builder();
        let chi0 = euler_characteristic(b.brep());
        let (v0, e0, f0) = (
            b.brep().vertices.len(),
            b.brep().edges.len(),
            b.brep().faces.len(),
        );

        // Connect two opposite corners of the square.
        let (new_face, e_new) = mef(&mut b, face_id, verts[0], verts[2]);
        assert_eq!(b.brep().vertices.len(), v0);
        assert_eq!(b.brep().edges.len(), e0 + 1);
        assert_eq!(b.brep().faces.len(), f0 + 1);
        assert_eq!(euler_characteristic(b.brep()), chi0, "MEF must preserve χ");

        kef(&mut b, face_id, new_face, e_new);
        assert_eq!(b.brep().vertices.len(), v0);
        assert_eq!(b.brep().edges.len(), e0);
        assert_eq!(b.brep().faces.len(), f0);
        assert_eq!(euler_characteristic(b.brep()), chi0, "KEF must restore χ");
    }
}
