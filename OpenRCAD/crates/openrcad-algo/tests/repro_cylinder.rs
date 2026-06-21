//! Native (smooth) cylinder boolean capability.
//!
//! ZeroCAD facets cylinders into 48-gon prisms for booleans (producing the
//! striped wireframe) because of a stale "truck panics on cylinders" comment.
//! These tests pin down what the current kernel actually handles natively, so
//! that faceting can be retired incrementally:
//!
//! * Cylinder **cuts** (drill / pocket, tool or object) are watertight — the
//!   common case, and the one in the reported screenshot (a cylinder positioned
//!   above a part to drill it). ZeroCAD can use the smooth cylinder here.
//! * The cylinder **boss union** (a cap coplanar with the box top) is the
//!   remaining gap: it needs the imprint engine to bore a closed circular hole
//!   into a coplanar planar face. Until then `boolean_checked` *rejects* it
//!   (never returns broken geometry), so a join safely falls back to the facet.

use openrcad_algo::{boolean_checked, BooleanError, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Solid;

fn assert_clean(name: &str, r: Result<Solid, BooleanError>) {
    let s = r.unwrap_or_else(|e| panic!("{name} should succeed: {e}"));
    assert!(s.is_watertight(), "{name} must be watertight");
    assert!(s.health_report().is_healthy(), "{name} must be healthy");
}

#[test]
fn native_cylinder_through_drill_is_watertight() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let through = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    assert_clean(
        "box - through cylinder",
        boolean_checked(&block, &through, BooleanOp::Cut),
    );
}

#[test]
fn native_cylinder_blind_pocket_is_watertight() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let blind = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 5.0), Dir::dz()), 3.0, 8.0);
    assert_clean(
        "box - blind cylinder",
        boolean_checked(&block, &blind, BooleanOp::Cut),
    );
}

#[test]
fn native_cylinder_as_cut_object_is_watertight() {
    // Cylinder primitive as the OBJECT, cut by a box pocket.
    let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dy()), 6.0, 20.0);
    let tool = make_box(&Pnt::new(-7.0, 5.0, -1.0), 14.0, 10.0, 7.0);
    assert_clean(
        "cylinder - box pocket",
        boolean_checked(&cyl, &tool, BooleanOp::Cut),
    );
}

/// The cylinder boss union (cap coplanar with the box top) — previously the
/// failing case that forced ZeroCAD to facet cylinders. The coplanar split now
/// imprints the cap's full circle as a hole in the box top, so the union closes
/// watertight (box bottom + 4 sides + annular top + 3 walls + top cap = 10).
#[test]
fn native_cylinder_boss_union_is_watertight() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let boss = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, 10.0), Dir::dz()), 3.0, 8.0);
    assert_clean(
        "box + cylinder boss",
        boolean_checked(&block, &boss, BooleanOp::Fuse),
    );
}
