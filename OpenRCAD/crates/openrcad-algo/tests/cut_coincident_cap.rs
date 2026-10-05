//! Regularized-difference contract for tools that only TOUCH the target.
//!
//! A difference `A − B` where `B`'s material merely abuts `A`'s boundary
//! (shared cap plane, interior void overlap) must be a material no-op: the
//! regularized difference removes no volume that was not interior to both.
//! These regressions pin two break-test findings (E16 family, found
//! 2026-09-11 through the turn-down fixture):
//!
//! * a coaxial cylinder whose base sits exactly on the rod's far cap used to
//!   nibble ~27 mm³ out of the rod instead of changing nothing;
//! * cutting where a void already exists (a second bore overlapping an
//!   existing hole) must still remove the crescent of NEW material.

use openrcad_algo::{boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};
use openrcad_topo::Solid;

/// Signed volume by the divergence theorem over the tessellated boundary.
fn volume(solid: &Solid) -> f64 {
    let mesh = openrcad_mesh::tessellate_checked(solid, 0.05, 0.35)
        .expect("bounds probing needs a tessellable solid");
    let mut six_v = 0.0;
    for [a, b, c] in &mesh.triangles {
        let (p, q, r) = (
            mesh.vertices[*a as usize],
            mesh.vertices[*b as usize],
            mesh.vertices[*c as usize],
        );
        six_v += p.x() * (q.y() * r.z() - q.z() * r.y()) - p.y() * (q.x() * r.z() - q.z() * r.x())
            + p.z() * (q.x() * r.y() - q.y() * r.x());
    }
    six_v / 6.0
}

/// A tool entirely OUTSIDE the target except for a shared cap plane must be a
/// no-op: the rod keeps its full volume (regularized difference ignores the
/// zero-measure contact disc).
#[test]
fn coaxial_tool_flush_with_cap_changes_nothing() {
    let rod = make_cylinder_operation(&Ax2::new(Pnt::origin(), Dir::dy()), 10.0, 30.0)
        .expect("rod")
        .value;
    // Base exactly at the rod's far cap (y = 30), sweeping outward.
    let flush = make_cylinder_operation(&Ax2::new(Pnt::new(0.0, 30.0, 0.0), Dir::dy()), 6.0, 15.0)
        .expect("flush tool")
        .value;
    let result = boolean_operation(&rod, &flush, BooleanOp::Cut)
        .expect("a flush outward tool is a no-op difference")
        .value;
    let full = std::f64::consts::PI * 100.0 * 30.0;
    let measured = volume(&result);
    assert!(
        (measured - full).abs() / full < 5.0e-3,
        "flush outward tool must remove nothing: {measured} vs {full}"
    );
}

/// The turn-down shape from the other side: a coaxial tool cutting INTO the
/// rod must remove the exact annulus (this direction already works; pinned so
/// any fix for the flush case cannot break it).
#[test]
fn coaxial_tool_into_cap_removes_exact_annulus() {
    let rod = make_cylinder_operation(&Ax2::new(Pnt::origin(), Dir::dy()), 10.0, 30.0)
        .expect("rod")
        .value;
    let into = make_cylinder_operation(&Ax2::new(Pnt::new(0.0, 15.0, 0.0), Dir::dy()), 6.0, 15.0)
        .expect("boring tool")
        .value;
    let result = boolean_operation(&rod, &into, BooleanOp::Cut)
        .expect("coaxial blind bore from the cap")
        .value;
    let expected = std::f64::consts::PI * (100.0 * 15.0 + 64.0 * 15.0);
    let measured = volume(&result);
    assert!(
        (measured - expected).abs() / expected < 5.0e-3,
        "exact annulus: {measured} vs {expected}"
    );
}

/// E16 kernel half (found 2026-09-11 via break-test E16 and the turn-down
/// fixture): a box already carrying one cylindrical void, cut by a second
/// cylinder overlapping that void, loses only the crescent of new material.
/// Parallel cylinders used to report no intersection at all, so neither wall
/// was split where the other crosses it and the result had free edges. They
/// now meet along their two common rulings.
#[test]
fn overlapping_second_bore_removes_only_the_crescent() {
    let block = make_box_operation(&Pnt::origin(), 30.0, 30.0, 10.0)
        .expect("block")
        .value;
    let first =
        make_cylinder_operation(&Ax2::new(Pnt::new(15.0, 15.0, -1.0), Dir::dz()), 4.0, 12.0)
            .expect("first bore")
            .value;
    let once = boolean_operation(&block, &first, BooleanOp::Cut)
        .expect("first bore")
        .value;
    let second =
        make_cylinder_operation(&Ax2::new(Pnt::new(20.0, 15.0, -1.0), Dir::dz()), 4.0, 12.0)
            .expect("second bore")
            .value;
    let result = boolean_operation(&once, &second, BooleanOp::Cut)
        .expect("the overlapping second bore")
        .value;
    // Two r=4 discs 5 mm apart remove their union area times height.
    let expected = 30.0 * 30.0 * 10.0 - disc_union_area(4.0, 4.0, 5.0) * 10.0;
    assert!(expected < volume(&once));
    let measured = volume(&result);
    assert!(
        (measured - expected).abs() / expected < 5.0e-3,
        "only the crescent goes: {measured} vs {expected}"
    );
}

/// Two overlapping parallel bosses of different radii fuse into one solid
/// whose volume is the union of their cross-sections times the height. Large
/// radii keep the tessellated volume's chordal error well under the tolerance.
#[test]
fn overlapping_parallel_bosses_fuse() {
    let first = make_cylinder_operation(&Ax2::new(Pnt::origin(), Dir::dz()), 20.0, 32.0)
        .expect("first boss")
        .value;
    let second =
        make_cylinder_operation(&Ax2::new(Pnt::new(16.0, 12.0, -8.0), Dir::dz()), 12.0, 48.0)
            .expect("second boss")
            .value;
    let fused = boolean_operation(&first, &second, BooleanOp::Fuse)
        .expect("parallel bosses fuse")
        .value;
    // The second boss overhangs both caps by 8 mm.
    let (a1, a2) = (400.0 * std::f64::consts::PI, 144.0 * std::f64::consts::PI);
    let lens = a1 + a2 - disc_union_area(20.0, 12.0, 20.0);
    let expected = a1 * 32.0 + a2 * 48.0 - lens * 32.0;
    let measured = volume(&fused);
    assert!(
        (measured - expected).abs() / expected < 5.0e-3,
        "union of the bosses: {measured} vs {expected}"
    );
}

/// A small bore straddling a large bore's wall (found by the ZeroCAD join/cut
/// census, chain 45). The bottom face's arc of the small bore must stop
/// exactly where it enters the large bore; the face-containment test sampled
/// the large bore's rim with six chords per edge, whose 0.1 mm sag took points
/// just inside the large bore for face material and left an open edge.
#[test]
fn small_bore_straddling_a_large_bore_wall() {
    let block = make_box_operation(&Pnt::origin(), 50.0, 45.0, 9.8)
        .expect("block")
        .value;
    let (c1, r1) = ((29.15, 14.77), 6.91);
    let (c2, r2) = ((28.98, 20.06), 2.06);
    let bore = |(x, y): (f64, f64), r: f64| {
        make_cylinder_operation(&Ax2::new(Pnt::new(x, y, -2.0), Dir::dz()), r, 14.0)
            .expect("bore")
            .value
    };
    let once = boolean_operation(&block, &bore(c1, r1), BooleanOp::Cut)
        .expect("large bore")
        .value;
    let twice = boolean_operation(&once, &bore(c2, r2), BooleanOp::Cut)
        .expect("small bore straddling the large one")
        .value;
    let s = f64::hypot(c2.0 - c1.0, c2.1 - c1.1);
    let expected = (50.0 * 45.0 - disc_union_area(r1, r2, s)) * 9.8;
    let measured = volume(&twice);
    assert!(
        (measured - expected).abs() / expected < 1.0e-3,
        "both bores removed: {measured} vs {expected}"
    );
}

/// Area of the union of two discs of radii `r1`, `r2` with centres `s` apart
/// (crossing, not nested).
fn disc_union_area(r1: f64, r2: f64, s: f64) -> f64 {
    let lens = r1 * r1 * ((s * s + r1 * r1 - r2 * r2) / (2.0 * s * r1)).acos()
        + r2 * r2 * ((s * s + r2 * r2 - r1 * r1) / (2.0 * s * r2)).acos()
        - 0.5 * ((-s + r1 + r2) * (s + r1 - r2) * (s - r1 + r2) * (s + r1 + r2)).sqrt();
    std::f64::consts::PI * (r1 * r1 + r2 * r2) - lens
}
