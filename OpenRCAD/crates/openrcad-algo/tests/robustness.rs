//! Robustness regression suite for booleans and blends.
//!
//! Every modeling result a CAD kernel emits must be a closed, two-manifold
//! solid. These tests assert exactly that via [`Solid::is_watertight`] across
//! orientation-general inputs and a battery of adversarial boolean
//! configurations, so a regression that re-introduces a gap or sliver fails
//! loudly instead of silently shipping a leaky shell.
//!
//! A second, `#[ignore]`d group documents the current failure frontier — the
//! "partial imprint" cases where an intersection only partly crosses a face.
//! They are runnable with `cargo test --ignored` and encode the watertight goal
//! the boolean engine does not yet meet, so progress (or regress) is measurable.

use openrcad_algo::{boolean, boolean_checked, chamfer, fillet, shell_solid, BooleanOp, SolidExt};
use openrcad_foundation::{Ax1, Ax2, Dir, Pnt, Trsf};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

/// A generic rigid rotation about a non-axis-aligned axis — used to prove the
/// blend/boolean paths are frame-general, not secretly axis-aligned.
fn generic_rotation() -> Trsf {
    Trsf::rotation(
        &Ax1::new(Pnt::new(1.0, 2.0, 3.0), Dir::new(1.0, 1.0, 1.0)),
        0.6,
    )
}

fn box10(o: Pnt) -> Solid {
    make_box(&o, 10.0, 10.0, 10.0)
}

fn assert_closed(label: &str, s: &Solid) {
    assert!(
        s.validate().is_ok(),
        "{label}: structural validation failed: {:?}",
        s.validate()
    );
    assert!(
        s.is_watertight(),
        "{label}: not watertight ({:?}); euler={}",
        s.manifold_report(),
        s.euler_characteristic()
    );
}

// ----- Orientation generality: blends work at any rigid placement -----------

#[test]
fn fillet_rotated_box_is_watertight() {
    let rbox = box10(Pnt::origin()).transformed(&generic_rotation());
    let f = fillet(&rbox, 1.0).expect("rotated box should fillet");
    assert_eq!(
        f.face_count(),
        26,
        "6 planes + 12 cylinders + 8 sphere corners"
    );
    assert_closed("fillet rotated box", &f);
}

#[test]
fn chamfer_rotated_box_is_watertight() {
    let rbox = box10(Pnt::origin()).transformed(&generic_rotation());
    let c = chamfer(&rbox, 1.0).expect("rotated box should chamfer");
    assert_closed("chamfer rotated box", &c);
}

#[test]
fn fillet_rotated_cylinder_is_watertight() {
    let rcyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 8.0)
        .transformed(&generic_rotation());
    let f = fillet(&rcyl, 0.5).expect("rotated cylinder should fillet");
    assert_closed("fillet rotated cylinder", &f);
}

#[test]
fn shell_rotated_box_open_face_is_watertight() {
    let rbox = box10(Pnt::origin()).transformed(&generic_rotation());
    // Pick an outward-facing face in the rotated frame to leave open.
    let open = rbox.planar_faces();
    let one = vec![open.into_iter().next().expect("a planar face")];
    let s = shell_solid(&rbox, 1.0, &one).expect("rotated box should shell");
    assert_closed("shell rotated box", &s);
}

// ----- Adversarial booleans that currently DO produce closed solids ---------

#[test]
fn corner_overlap_intersection_is_watertight() {
    let r = boolean(
        &box10(Pnt::origin()),
        &box10(Pnt::new(5.0, 5.0, 5.0)),
        BooleanOp::Common,
    );
    assert_closed("corner-overlap common", &r);
}

#[test]
fn enclosed_void_cut_is_closed() {
    // A box with a fully interior cubic void: two nested closed shells, so it is
    // watertight and two-manifold (Euler 4 = two genus-0 shells).
    let r = boolean(
        &box10(Pnt::origin()),
        &make_box(&Pnt::new(3.0, 3.0, 3.0), 4.0, 4.0, 4.0),
        BooleanOp::Cut,
    );
    assert_closed("enclosed-void cut", &r);
    assert_eq!(
        r.euler_characteristic(),
        4,
        "outer shell + inner void shell"
    );
}

#[test]
fn face_flush_union_is_watertight() {
    let r = boolean(
        &box10(Pnt::origin()),
        &make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0),
        BooleanOp::Fuse,
    );
    assert_closed("face-flush fuse", &r);
}

#[test]
fn checked_boolean_accepts_side_drill() {
    let side = make_cylinder(&Ax2::new(Pnt::new(-1.0, 5.0, 5.0), Dir::dx()), 2.0, 12.0);
    let r = boolean_checked(&box10(Pnt::origin()), &side, BooleanOp::Cut)
        .expect("checked boolean must succeed for side-drill");
    assert_closed("checked side-drill", &r);
}

// ----- Known failure frontier: partial-imprint cases (watertight GOAL) ------
// These are the configurations where an intersection curve only partially
// crosses a face, so the split pass leaves an un-imprinted boundary and the sewn
// shell has free edges (or a non-contiguous loop). They encode the target; run
// with `cargo test --ignored`.
//
// Note: `is_watertight` (every edge shared by exactly two faces) and `validate`
// (each loop is closed and contiguous) are complementary — the side-drill below
// passes the former yet fails the latter, so both belong in the closed-solid bar.

#[test]
fn through_side_drill_should_be_closed() {
    let side = make_cylinder(&Ax2::new(Pnt::new(-1.0, 5.0, 5.0), Dir::dx()), 2.0, 12.0);
    let r = boolean(&box10(Pnt::origin()), &side, BooleanOp::Cut);
    assert_closed("through side-drill cut", &r);
}

#[test]
fn corner_overlap_union_should_be_watertight() {
    let r = boolean(
        &box10(Pnt::origin()),
        &box10(Pnt::new(5.0, 5.0, 5.0)),
        BooleanOp::Fuse,
    );
    assert_closed("corner-overlap fuse", &r);
}

#[test]
fn blind_pocket_cut_should_be_watertight() {
    let tool = make_box(&Pnt::new(3.0, 3.0, 5.0), 4.0, 4.0, 8.0);
    let r = boolean(&box10(Pnt::origin()), &tool, BooleanOp::Cut);
    assert_closed("blind-pocket cut", &r);
}

#[test]
fn rotated_tool_partial_cut_should_be_watertight() {
    let tool = make_box(&Pnt::new(5.0, 5.0, -2.0), 6.0, 6.0, 14.0).transformed(&Trsf::rotation(
        &Ax1::new(Pnt::new(8.0, 8.0, 0.0), Dir::dz()),
        0.52,
    ));
    let r = boolean(&box10(Pnt::origin()), &tool, BooleanOp::Cut);
    assert_closed("rotated-tool cut", &r);
}
