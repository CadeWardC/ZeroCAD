//! Consolidated boolean-robustness gate for **cylinders and coplanar faces**.
//!
//! Every downstream stage (edge identity, render mesh, selection, fillets)
//! inherits whatever the boolean engine emits, so "reliable" must be measurable.
//! This file is the single comprehensive gate: a matrix of cylinder cut/join/boss
//! (straddling, coaxial, tangent) and coplanar join/flush-cut/near-coincident
//! cases, each asserting the result is watertight, healthy, Euler-2 per connected
//! body, deterministic, and — where the inputs were analytic — that cylindrical
//! walls survive as analytic `GeomSurface::Cylinder` faces rather than collapsing
//! to facets.
//!
//! The narrative regressions (`repro_circular_bite`, `repro_cylinder`,
//! `repro_screenshots`) stay as-is; this is the breadth gate that drives the
//! robustness phases. Cases that fail today are `#[ignore]`d with a reason and
//! flipped on as each phase lands.

use openrcad_algo::{boolean_checked, BooleanError, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::GeomSurface;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

// ---- shared assertions -----------------------------------------------------

/// Run the boolean and assert the result is structurally sound: watertight and
/// healthy. `health_report` already validates Euler–Poincaré consistency
/// (`SuspiciousEulerCharacteristic`), correctly accounting for ring-bearing
/// faces — a boss or blind pocket leaves an annular face whose proper Euler
/// value is `V−E+F = 2 + rings`, so a blanket `== 2` would wrongly reject valid
/// holed bodies. Each connected body must also be watertight on its own.
fn assert_sound(name: &str, r: Result<Solid, BooleanError>) -> Solid {
    let s = r.unwrap_or_else(|e| panic!("{name} should succeed: {e}"));
    assert!(s.is_watertight(), "{name} must be watertight");
    assert!(
        s.health_report().is_healthy(),
        "{name} must be healthy: {:?}",
        s.health_report().errors
    );
    for (i, body) in s.split_disconnected().iter().enumerate() {
        assert!(
            body.is_watertight(),
            "{name} connected body {i} must be watertight"
        );
    }
    s
}

/// The boolean must be deterministic: structurally identical on a re-run (same
/// face / edge / vertex counts). A flickering count betrays the ray-parity
/// nondeterminism the robust classifier targets.
fn assert_deterministic(name: &str, object: &Solid, tool: &Solid, op: BooleanOp) {
    let a = boolean_checked(object, tool, op)
        .unwrap_or_else(|e| panic!("{name} (run A) should succeed: {e}"));
    let b = boolean_checked(object, tool, op)
        .unwrap_or_else(|e| panic!("{name} (run B) should succeed: {e}"));
    assert_eq!(
        (a.face_count(), a.edge_count(), a.vertex_count()),
        (b.face_count(), b.edge_count(), b.vertex_count()),
        "{name} must be deterministic across runs"
    );
}

/// Count analytic cylindrical faces of radius `r` about an axis parallel to
/// `axis`. Zero means the cylinder collapsed to facets (or vanished).
fn analytic_cylinder_count(s: &Solid, r: f64, axis: Dir) -> usize {
    s.shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&axis).abs() > 0.999
                        && (c.radius() - r).abs() < 1.0e-3
            )
        })
        .count()
}

/// Every boundary point of every radius-`r` cylindrical wall about `axis` (at
/// `cx,cy` in the plane perpendicular to a vertical axis) lies on that cylinder —
/// i.e. the wall is a true analytic scallop, not facets masquerading as one.
fn cylinder_walls_stay_on_surface(s: &Solid, cx: f64, cy: f64, r: f64) -> bool {
    use openrcad_geom::Curve;
    let walls: Vec<_> = s
        .shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - r).abs() < 1.0e-3
            )
        })
        .collect();
    if walls.is_empty() {
        return false;
    }
    for wall in &walls {
        let Some(wire) = wall.outer_wire() else {
            return false;
        };
        for edge in wire.edges() {
            let Some(curve) = edge.curve() else {
                return false;
            };
            for k in 0..=12 {
                let t = edge.first() + (edge.last() - edge.first()) * k as f64 / 12.0;
                let p = curve.point(t);
                let radial = ((p.x() - cx).powi(2) + (p.y() - cy).powi(2)).sqrt();
                if (radial - r).abs() > 5.0e-3 {
                    return false;
                }
            }
        }
    }
    true
}

// ===========================================================================
// Group A — Cylinder cut / bite
// ===========================================================================

#[test]
fn cyl_cut_through_drill() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    let s = assert_sound("through-drill", boolean_checked(&block, &drill, BooleanOp::Cut));
    assert!(
        analytic_cylinder_count(&s, 3.0, Dir::dz()) > 0,
        "through-drill must keep an analytic cylindrical bore wall"
    );
    assert_deterministic("through-drill", &block, &drill, BooleanOp::Cut);
}

#[test]
fn cyl_cut_blind_pocket() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let blind = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 5.0), Dir::dz()), 3.0, 8.0);
    let s = assert_sound("blind-pocket", boolean_checked(&block, &blind, BooleanOp::Cut));
    assert!(
        analytic_cylinder_count(&s, 3.0, Dir::dz()) > 0,
        "blind pocket must keep an analytic cylindrical wall"
    );
}

#[test]
fn cyl_cut_straddling_bite() {
    // Circle straddles the bottom edge of the block (the "vanishing bite").
    let block = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let cutter = make_cylinder(&Ax2::new(Pnt::new(20.0, 8.0, -1.0), Dir::dz()), 14.0, 12.0);
    let s = assert_sound("straddling-bite", boolean_checked(&block, &cutter, BooleanOp::Cut));
    assert!(
        cylinder_walls_stay_on_surface(&s, 20.0, 8.0, 14.0),
        "straddling bite must keep an analytic radius-14 wall, not become a box"
    );
}

#[test]
fn cyl_cut_oblique_axis() {
    // A mild-tilt cylinder piercing the box cleanly front (y=0) to back (y=20),
    // staying within the z-extent — both pierced faces get an oblique-ellipse
    // imprint. Watertight now that plane∩cylinder returns an exact `Ellipse`
    // (Phase 3a) instead of a 160-point sampled B-spline.
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 20.0);
    let axis = Dir::from_vec(&openrcad_foundation::Vec::new(0.0, 1.0, 0.25)).unwrap();
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, -2.0, 8.0), axis), 3.0, 30.0);
    assert_sound("oblique-axis cut", boolean_checked(&block, &drill, BooleanOp::Cut));
}

#[test]
fn cyl_cut_oblique_axis_corner_exit() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 20.0);
    let axis = Dir::from_vec(&openrcad_foundation::Vec::new(0.0, 1.0, 1.0)).unwrap();
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, -2.0, 8.0), axis), 3.0, 30.0);
    assert_sound("oblique corner-exit cut", boolean_checked(&block, &drill, BooleanOp::Cut));
}

// ===========================================================================
// Group B — Cylinder join / boss
// ===========================================================================

#[test]
fn cyl_join_coplanar_boss() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let boss = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 10.0), Dir::dz()), 3.0, 8.0);
    let s = assert_sound("coplanar boss", boolean_checked(&block, &boss, BooleanOp::Fuse));
    assert!(
        analytic_cylinder_count(&s, 3.0, Dir::dz()) > 0,
        "boss must keep an analytic cylindrical wall"
    );
}

#[test]
fn cyl_join_boss_straddling_edge() {
    // Boss rim overhangs the host's top edge (partially off the face).
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let boss = make_cylinder(&Ax2::new(Pnt::new(20.0, 10.0, 10.0), Dir::dz()), 4.0, 8.0);
    assert_sound("boss straddling edge", boolean_checked(&block, &boss, BooleanOp::Fuse));
}

// ===========================================================================
// Group C — Coaxial cylinders
// ===========================================================================

#[test]
fn coaxial_concentric_boss_union() {
    // A fat boss with a thinner boss stacked on it, sharing the axis.
    let lower = make_cylinder(&Ax2::new(Pnt::new(0.0, 0.0, 0.0), Dir::dz()), 6.0, 5.0);
    let upper = make_cylinder(&Ax2::new(Pnt::new(0.0, 0.0, 5.0), Dir::dz()), 6.0, 5.0);
    // Same radius, stacked end-to-end, coaxial: should fuse into one taller cylinder.
    assert_sound("coaxial equal-radius stack", boolean_checked(&lower, &upper, BooleanOp::Fuse));
}

#[test]
fn coaxial_recut_existing_hole() {
    // Drill a hole, then cut a larger coaxial hole that re-opens / widens it.
    // Watertight now that `partition_face` assigns the pre-existing r2 hole to the
    // innermost containing face (the new r4 disk), so the inner ring + old wall are
    // removed cleanly instead of leaving the r2 wire orphaned on the cap.
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let small = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 2.0, 12.0);
    let drilled = boolean_checked(&block, &small, BooleanOp::Cut)
        .expect("initial drill should succeed");
    let big = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 4.0, 12.0);
    assert_sound("re-cut coaxial hole", boolean_checked(&drilled, &big, BooleanOp::Cut));
}

// ===========================================================================
// Group D — Coplanar
// ===========================================================================

#[test]
fn coplanar_two_box_flush_union() {
    let a = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let b = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let s = assert_sound("flush box union", boolean_checked(&a, &b, BooleanOp::Fuse));
    assert_eq!(s.face_count(), 6, "flush union must merge to a clean 6-face box");
}

#[test]
fn coplanar_flush_through_cut() {
    // Tool spans the full Y/Z and removes a middle X-slab (severing the bar).
    let bar = make_box(&Pnt::origin(), 30.0, 10.0, 10.0);
    let knife = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    assert_sound("flush through-cut", boolean_checked(&bar, &knife, BooleanOp::Cut));
}

#[test]
fn near_coincident_cut_snaps_clean() {
    // Cut cylinder whose back cap stops a sub-fuzz hair short of the back face:
    // the snap must collapse it onto the clean coincident path.
    let back = 11.5024;
    let cube = make_box(&Pnt::new(-13.4, 0.0, -12.8), 21.0, back, 17.6);
    let cap_y = back - 1e-7;
    let axis = Ax2::new_axes(Pnt::new(7.5, cap_y - 23.0, 5.0), Dir::dy(), Dir::dx());
    let cyl = make_cylinder(&axis, 7.81, 23.0);
    assert_sound("near-coincident snap cut", boolean_checked(&cube, &cyl, BooleanOp::Cut));
}
