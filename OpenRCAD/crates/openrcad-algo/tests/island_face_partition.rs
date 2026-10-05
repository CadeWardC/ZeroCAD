//! A face split along a closed network that touches neither its outer
//! boundary nor anything else leaves an island: here a through slot cut into a
//! block top that already carries a small hole and the rim of a boss the slot
//! also crosses. The slot piece is a face inside the hole bounded by the slot
//! and boss outline. Face partition took any loop of opposite sign to its
//! innermost container for a hole, so the slot piece (nested in that hole
//! boundary) was dropped and the block top kept its original loops, leaving
//! free edges. Found by the ZeroCAD join/cut census (chain 426).
use openrcad_algo::{boolean::point_in_solid, boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};

#[test]
fn slot_through_a_hole_and_a_boss_rim_cuts_cleanly() {
    let (bx, by, bz) = (45.957026569490, 30.589970574867, 8.883916659143);
    let block = make_box_operation(&Pnt::origin(), bx, by, bz)
        .expect("block")
        .value;
    let (hole_c, hole_r) = ((20.714391931482, 14.668838109209), 1.798543419043);
    let hole = make_cylinder_operation(
        &Ax2::new(Pnt::new(hole_c.0, hole_c.1, -2.0), Dir::dz()),
        hole_r,
        28.9,
    )
    .expect("hole")
    .value;
    let (boss_c, boss_r, boss_z) = (
        (14.256463567809, 22.516381581650),
        5.071482884051,
        (6.689889717596, 18.356805811639),
    );
    let boss = make_cylinder_operation(
        &Ax2::new(Pnt::new(boss_c.0, boss_c.1, boss_z.0), Dir::dz()),
        boss_r,
        boss_z.1 - boss_z.0,
    )
    .expect("boss")
    .value;
    let (slot_lo, slot_hi) = (
        (7.368262982675, 9.321150928274),
        (31.677014705780, 26.385932084691),
    );
    let slot = make_box_operation(
        &Pnt::new(slot_lo.0, slot_lo.1, -2.0),
        slot_hi.0 - slot_lo.0,
        slot_hi.1 - slot_lo.1,
        28.9,
    )
    .expect("slot")
    .value;

    let mut solid = block;
    for (name, tool, op) in [
        ("hole", &hole, BooleanOp::Cut),
        ("boss", &boss, BooleanOp::Fuse),
        ("slot", &slot, BooleanOp::Cut),
    ] {
        solid = boolean_operation(&solid, tool, op)
            .unwrap_or_else(|error| panic!("{name}: {error:?}"))
            .value;
    }
    solid
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("valid solid");

    let within = |c: (f64, f64), r: f64, x: f64, y: f64| (x - c.0).hypot(y - c.1) < r;
    let mut checked = 0;
    for i in 0..48 {
        for j in 0..33 {
            for k in 0..10 {
                let (x, y, z) = (
                    0.473 + 0.97 * i as f64,
                    0.311 + 0.97 * j as f64,
                    0.217 + 1.9 * k as f64,
                );
                let near = |c: (f64, f64), r: f64| ((x - c.0).hypot(y - c.1) - r).abs() < 0.05;
                if near(hole_c, hole_r) || near(boss_c, boss_r) {
                    continue;
                }
                let in_block = x < bx && y < by && z < bz && !within(hole_c, hole_r, x, y);
                let in_boss = within(boss_c, boss_r, x, y) && z > boss_z.0 && z < boss_z.1;
                let in_slot = x > slot_lo.0 && x < slot_hi.0 && y > slot_lo.1 && y < slot_hi.1;
                checked += 1;
                assert_eq!(
                    point_in_solid(&Pnt::new(x, y, z), &solid),
                    (in_block || in_boss) && !in_slot,
                    "material at ({x}, {y}, {z})"
                );
            }
        }
    }
    assert!(checked > 10_000);
}
