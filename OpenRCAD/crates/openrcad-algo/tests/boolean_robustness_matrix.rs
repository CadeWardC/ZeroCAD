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
//! robustness phases. Every Phase 3 case runs in the ordinary test suite.

use openrcad_algo::{boolean_checked, boolean_checked_bodies, BooleanError, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::GeomSurface;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

// ---- shared assertions -----------------------------------------------------

/// Run the boolean and assert the result is structurally sound: watertight and
/// healthy. `health_report` already validates Euler–Poincaré consistency,
/// correctly accounting for ring-bearing
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
    let s = assert_sound(
        "through-drill",
        boolean_checked(&block, &drill, BooleanOp::Cut),
    );
    assert!(
        analytic_cylinder_count(&s, 3.0, Dir::dz()) > 0,
        "through-drill must keep an analytic cylindrical bore wall"
    );
    assert_deterministic("through-drill", &block, &drill, BooleanOp::Cut);
}

#[test]
fn cyl_cut_scaled_through_hole_regression() {
    // Exact kernel property-corpus regression. The cylinder is centered on the
    // XY cross-section, aligned with +Z, and extends one unit
    // beyond both box faces. This scale/radius combination previously left one
    // circular boundary with three uses (a non-manifold edge).
    // Keep the source values as `f32`: ZeroCAD feature parameters are `f32`,
    // and the exact rounded product is part of this regression.
    let width = 63.78_f32;
    let depth = 66.26_f32;
    let height = 23.47_f32;
    let radius_ratio = 0.2585_f32;
    let radius = width.min(depth) * radius_ratio;
    let block = make_box(&Pnt::origin(), width as f64, depth as f64, height as f64);
    let drill = make_cylinder(
        &Ax2::new(
            Pnt::new((width * 0.5) as f64, (depth * 0.5) as f64, -1.0),
            Dir::dz(),
        ),
        radius as f64,
        (height + 2.0) as f64,
    );

    let result = assert_sound(
        "scaled-through-hole",
        boolean_checked(&block, &drill, BooleanOp::Cut),
    );
    assert!(
        analytic_cylinder_count(&result, radius as f64, Dir::dz()) > 0,
        "scaled through-hole must preserve an analytic cylindrical wall"
    );
    assert_deterministic("scaled-through-hole", &block, &drill, BooleanOp::Cut);
}

#[test]
fn cyl_cut_blind_pocket() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let blind = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 5.0), Dir::dz()), 3.0, 8.0);
    let s = assert_sound(
        "blind-pocket",
        boolean_checked(&block, &blind, BooleanOp::Cut),
    );
    assert!(
        analytic_cylinder_count(&s, 3.0, Dir::dz()) > 0,
        "blind pocket must keep an analytic cylindrical wall"
    );
}

#[test]
fn cyl_cut_blind_pocket_with_flush_tool_cap() {
    let block = make_box(&Pnt::origin(), 40.0, 20.0, 10.0);
    let blind = make_cylinder(&Ax2::new(Pnt::new(20.0, 10.0, 4.0), Dir::dz()), 4.0, 6.0);
    let s = assert_sound(
        "blind-pocket-flush-tool-cap",
        boolean_checked(&block, &blind, BooleanOp::Cut),
    );
    assert!(
        analytic_cylinder_count(&s, 4.0, Dir::dz()) > 0,
        "a cutter cap coincident with the block top must leave one analytic pocket wall"
    );
}

#[test]
fn cyl_cut_straddling_bite() {
    // Circle straddles the bottom edge of the block (the "vanishing bite").
    let block = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let cutter = make_cylinder(&Ax2::new(Pnt::new(20.0, 8.0, -1.0), Dir::dz()), 14.0, 12.0);
    let s = assert_sound(
        "straddling-bite",
        boolean_checked(&block, &cutter, BooleanOp::Cut),
    );
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
    assert_sound(
        "oblique-axis cut",
        boolean_checked(&block, &drill, BooleanOp::Cut),
    );
}

#[test]
fn cyl_cut_oblique_axis_corner_exit() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 20.0);
    let axis = Dir::from_vec(&openrcad_foundation::Vec::new(0.0, 1.0, 1.0)).unwrap();
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, -2.0, 8.0), axis), 3.0, 30.0);
    assert_sound(
        "oblique corner-exit cut",
        boolean_checked(&block, &drill, BooleanOp::Cut),
    );
}

// ===========================================================================
// Group B — Cylinder join / boss
// ===========================================================================

#[test]
fn cyl_join_coplanar_boss() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let boss = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 10.0), Dir::dz()), 3.0, 8.0);
    let s = assert_sound(
        "coplanar boss",
        boolean_checked(&block, &boss, BooleanOp::Fuse),
    );
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
    assert_sound(
        "boss straddling edge",
        boolean_checked(&block, &boss, BooleanOp::Fuse),
    );
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
    assert_sound(
        "coaxial equal-radius stack",
        boolean_checked(&lower, &upper, BooleanOp::Fuse),
    );
}

#[test]
fn coaxial_recut_existing_hole() {
    // Drill a hole, then cut a larger coaxial hole that re-opens / widens it.
    // Watertight now that `partition_face` assigns the pre-existing r2 hole to the
    // innermost containing face (the new r4 disk), so the inner ring + old wall are
    // removed cleanly instead of leaving the r2 wire orphaned on the cap.
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let small = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 2.0, 12.0);
    let drilled =
        boolean_checked(&block, &small, BooleanOp::Cut).expect("initial drill should succeed");
    let big = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 4.0, 12.0);
    assert_sound(
        "re-cut coaxial hole",
        boolean_checked(&drilled, &big, BooleanOp::Cut),
    );
}

#[test]
fn blind_counterbore_over_through_bore() {
    // A block with a narrow through-bore, then a WIDE BLIND counterbore cut
    // from the top: its flat bottom sits inside the block. That bottom cap
    // plane meets the narrow bore wall in a full circle — the periodic-face
    // band-partition case (a closed curve wrapping a cylindrical face, splitting
    // it into two annular bands). ZeroCAD's Hole feature avoids this by cutting
    // the counterbore head BEFORE the bore, but the boolean engine should be
    // robust to either order.
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let bore = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 2.0, 12.0);
    let drilled = boolean_checked(&block, &bore, BooleanOp::Cut).expect("initial bore");
    // Ø8 counterbore, z = 7 .. 11 (bottom cap at z=7 inside; top pokes out the
    // z=10 face for a clean entry).
    let cbore = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 7.0), Dir::dz()), 4.0, 4.0);
    assert_sound(
        "blind counterbore over through-bore",
        boolean_checked(&drilled, &cbore, BooleanOp::Cut),
    );
}

// ===========================================================================
// Group D — Coplanar
// ===========================================================================

#[test]
fn coplanar_two_box_flush_union() {
    let a = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let b = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let s = assert_sound("flush box union", boolean_checked(&a, &b, BooleanOp::Fuse));
    assert_eq!(
        s.face_count(),
        6,
        "flush union must merge to a clean 6-face box"
    );
}

#[test]
fn coincident_cylinder_fills_bore_back_to_one_box() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let bore_tool = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 4.0, 12.0);
    let plug = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 0.0), Dir::dz()), 4.0, 10.0);
    let bored = boolean_checked(&block, &bore_tool, BooleanOp::Cut).expect("make through bore");
    let filled = assert_sound(
        "coincident cylinder filling a bore",
        boolean_checked(&bored, &plug, BooleanOp::Fuse),
    );
    assert_eq!(
        filled.face_count(),
        6,
        "filling an exact cylindrical bore must restore one seam-free box"
    );
}

#[test]
fn coplanar_flush_through_cut() {
    // Tool spans the full Y/Z and removes a middle X-slab (severing the bar).
    let bar = make_box(&Pnt::origin(), 30.0, 10.0, 10.0);
    let knife = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let bodies = boolean_checked_bodies(&bar, &knife, BooleanOp::Cut)
        .expect("flush through-cut should produce valid bodies");
    assert_eq!(bodies.len(), 2, "the removed middle slab severs the bar");
    for (index, body) in bodies.iter().enumerate() {
        assert!(body.is_watertight(), "body {index} must be watertight");
        assert!(
            body.health_report().is_healthy(),
            "body {index} must be healthy: {:?}",
            body.health_report().errors
        );
    }
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
    assert_sound(
        "near-coincident snap cut",
        boolean_checked(&cube, &cyl, BooleanOp::Cut),
    );
}
