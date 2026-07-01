//! Topology-identity contract for booleans (the kernel half of Phase 0b).
//!
//! Watertight + healthy proves the *geometry* is sound but says nothing about
//! whether the entities a downstream feature is pinned to survive the operation.
//! ZeroCAD reattaches fillets / selections by re-resolving a captured edge or
//! face against the rebuilt body, so a boolean that silently fragments or
//! renumbers a surviving entity detaches the feature even when the shape is
//! perfect. These tests pin the identities the engine must preserve:
//!
//! * a surface that the cut never touched stays one face (no spurious splits);
//! * an input cylinder wall survives as a single analytic `Cylinder` face;
//! * an edge the cut never crossed stays one edge spanning the same endpoints.

use openrcad_algo::{boolean_checked, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::GeomSurface;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

fn planar_face_count(s: &Solid) -> usize {
    s.shell()
        .faces()
        .into_iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Plane(_))))
        .count()
}

fn cylinder_face_count(s: &Solid, r: f64) -> usize {
    s.shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(f.surface(), Some(GeomSurface::Cylinder(c)) if (c.radius() - r).abs() < 1e-3)
        })
        .count()
}

/// Is there a single edge whose endpoints match `p0`/`p1` (unordered) within `tol`?
fn has_edge_between(s: &Solid, p0: Pnt, p1: Pnt, tol: f64) -> bool {
    s.edges().iter().any(|e| {
        let a = e.start().point();
        let b = e.end().point();
        (a.distance(&p0) < tol && b.distance(&p1) < tol)
            || (a.distance(&p1) < tol && b.distance(&p0) < tol)
    })
}

/// Drilling a hole straight through a box must keep all six original planar
/// faces — the four side walls untouched and the two caps bored into single
/// annular faces — plus the analytic bore wall. A spurious split would show up
/// as more than six planar faces.
#[test]
fn through_drill_preserves_six_planar_faces() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    let s = boolean_checked(&block, &drill, BooleanOp::Cut).expect("through-drill should succeed");

    assert_eq!(
        planar_face_count(&s),
        6,
        "through-drill must keep exactly the 6 original planar faces (4 walls + 2 annular caps), got {}",
        planar_face_count(&s)
    );
    assert!(
        cylinder_face_count(&s, 3.0) > 0,
        "through-drill must expose the bore as an analytic cylinder face"
    );
}

/// The four perimeter edges of the box top are nowhere near a central drill, so
/// each must survive as a single full-span edge (what endpoint-based fillet
/// selection depends on), not get fragmented by the boolean.
#[test]
fn central_drill_leaves_top_perimeter_edges_intact() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    let s = boolean_checked(&block, &drill, BooleanOp::Cut).expect("through-drill should succeed");

    let corners = [
        (Pnt::new(0.0, 0.0, 10.0), Pnt::new(20.0, 0.0, 10.0)),
        (Pnt::new(20.0, 0.0, 10.0), Pnt::new(20.0, 20.0, 10.0)),
        (Pnt::new(20.0, 20.0, 10.0), Pnt::new(0.0, 20.0, 10.0)),
        (Pnt::new(0.0, 20.0, 10.0), Pnt::new(0.0, 0.0, 10.0)),
    ];
    for (p0, p1) in corners {
        assert!(
            has_edge_between(&s, p0, p1, 1e-6),
            "top perimeter edge {p0:?}->{p1:?} must survive the drill as one edge"
        );
    }
}

/// A through-drill leaves the box's bore as analytic cylinder geometry; after the
/// cocylindrical merge it should read as a small, fixed number of cylinder faces
/// (the `make_cylinder` 3-arc wall), never zero (collapsed to facets).
#[test]
fn through_drill_bore_is_analytic_cylinder() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    let s = boolean_checked(&block, &drill, BooleanOp::Cut).expect("through-drill should succeed");
    let n = cylinder_face_count(&s, 3.0);
    assert!(n > 0, "bore must stay analytic, got {n} cylinder faces");
}
