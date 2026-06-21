//! Reproduction tests mirroring the ZeroCAD screenshots: thin plates, partial
//! overlaps, and off-axis (rotated) booleans — the general cases the existing
//! axis-aligned-cube suite never exercises. These document the *current* failure
//! modes; they are expected to fail until the classifier is made robust.

use openrcad_algo::{boolean, boolean_checked, BooleanOp};
use openrcad_foundation::{Ax1, Dir, Pnt, Trsf};
use openrcad_primitives::make_box;

fn report(name: &str, s: &openrcad_topo::Solid) {
    let hr = s.health_report();
    println!(
        "{name}: faces={} verts={} edges={} watertight={} healthy={}",
        s.face_count(),
        s.vertex_count(),
        s.edge_count(),
        s.is_watertight(),
        hr.is_healthy(),
    );
    if !hr.is_healthy() {
        println!("  ERRORS: {:?}", hr.errors);
    }
    if !hr.warnings.is_empty() {
        println!("  WARNINGS: {:?}", hr.warnings);
    }
}

/// Mirror ZeroCAD: it calls `boolean_checked`, which rejects unhealthy/leaky
/// output and returns Err -> ZeroCAD shows nothing / falls back.
fn report_checked(name: &str, object: &openrcad_topo::Solid, tool: &openrcad_topo::Solid, op: BooleanOp) {
    match boolean_checked(object, tool, op) {
        Ok(s) => println!("{name}: boolean_checked OK ({} faces)", s.face_count()),
        Err(e) => println!("{name}: boolean_checked REJECTED -> {e}"),
    }
}

/// Two thin plates forming an L (like screenshot 4): a 40x40x2 floor plate and a
/// 40x2x40 wall plate sharing an edge region. Union should be one watertight L.
#[test]
fn thin_plate_l_union_is_watertight() {
    let floor = make_box(&Pnt::new(0.0, 0.0, 0.0), 40.0, 40.0, 2.0);
    let wall = make_box(&Pnt::new(0.0, 0.0, 0.0), 40.0, 2.0, 40.0);
    let u = boolean(&floor, &wall, BooleanOp::Fuse);
    report("thin_plate_L_union", &u);
    assert!(u.is_watertight(), "thin-plate L union must be watertight");
}

/// A thin plate with a small box joined on top, partially overlapping (corner
/// overlap union, like screenshot 2's premise).
#[test]
fn corner_overlap_union_is_watertight() {
    let base = make_box(&Pnt::new(0.0, 0.0, 0.0), 40.0, 40.0, 10.0);
    let boss = make_box(&Pnt::new(30.0, 30.0, 5.0), 20.0, 20.0, 20.0);
    let u = boolean(&base, &boss, BooleanOp::Fuse);
    report("corner_overlap_union", &u);
    assert!(u.is_watertight(), "corner-overlap union must be watertight");
}

/// Off-axis cut: a box rotated 30 degrees about Z, cut by an axis-aligned box.
/// Mirrors the rotated-edge geometry in screenshots 1/2.
#[test]
fn rotated_box_cut_is_watertight() {
    let body = make_box(&Pnt::new(-20.0, -20.0, 0.0), 40.0, 40.0, 10.0);
    let rot = Trsf::rotation(&Ax1::new(Pnt::origin(), Dir::dz()), 30f64.to_radians());
    let body = body.transformed(&rot);
    let tool = make_box(&Pnt::new(0.0, -30.0, -5.0), 10.0, 60.0, 20.0);
    let c = boolean(&body, &tool, BooleanOp::Cut);
    report("rotated_box_cut", &c);
    assert!(c.is_watertight(), "rotated box cut must be watertight");
}

/// Thin plate cut by a thin slot (plate minus plate) — the configuration most
/// prone to grazing-ray misclassification.
#[test]
fn thin_plate_slot_cut_is_watertight() {
    let plate = make_box(&Pnt::new(0.0, 0.0, 0.0), 40.0, 40.0, 2.0);
    let slot = make_box(&Pnt::new(15.0, -5.0, -1.0), 10.0, 50.0, 4.0);
    let c = boolean(&plate, &slot, BooleanOp::Cut);
    report("thin_plate_slot_cut", &c);
    assert!(c.is_watertight(), "thin-plate slot cut must be watertight");
}

/// Every loop of a boolean result must be a contiguous, consistently-oriented
/// chain (each co-edge's oriented end meets the next's oriented start). This is
/// the invariant the `sew` re-threading pass restores; regressing it reintroduces
/// the `LoopNotContiguous` failure that made `boolean_checked` reject joins.
#[test]
fn cube_union_loops_are_all_contiguous() {
    let a = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let b = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let u = boolean(&a, &b, BooleanOp::Fuse);
    for (fi, face) in u.shell().faces().iter().enumerate() {
        for (wi, w) in face.wires().iter().enumerate() {
            let edges = w.edges();
            let n = edges.len();
            for i in 0..n {
                let cur_end = edges[i].target().point();
                let nxt_start = edges[(i + 1) % n].source().point();
                let gap = cur_end.distance(&nxt_start);
                assert!(
                    gap < 1e-6,
                    "face {fi} wire {wi} non-contiguous: gap {gap} between edge {i} and {}",
                    (i + 1) % n
                );
            }
        }
    }
}

/// Sanity: the existing clean axis-aligned union still works (control).
#[test]
fn control_clean_cube_union_is_watertight() {
    let a = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let b = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let u = boolean(&a, &b, BooleanOp::Fuse);
    report("control_cube_union", &u);
    report_checked("control_cube_union", &a, &b, BooleanOp::Fuse);
    assert!(u.is_watertight(), "control cube union must be watertight");
    // The blocker that broke ZeroCAD joins: the result must be *healthy* (no
    // non-contiguous loops) so `boolean_checked` accepts it rather than returning
    // None. Coplanar-face merge (14 -> 6 faces) is a separate quality pass.
    assert!(
        u.health_report().is_healthy(),
        "union must be healthy: {:?}",
        u.health_report().errors
    );
    assert!(
        boolean_checked(&a, &b, BooleanOp::Fuse).is_ok(),
        "boolean_checked must accept the clean cube union"
    );
    // Coplanar merge: the overlapping cubes simplify to a clean 6-face box.
    assert_eq!(
        u.face_count(),
        6,
        "coplanar-merged cube union should be a 6-face box"
    );
}
