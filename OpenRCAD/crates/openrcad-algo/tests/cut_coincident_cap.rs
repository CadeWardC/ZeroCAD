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

/// E16 kernel half, characterized (found 2026-09-11 via break-test E16 and
/// the turn-down fixture): a box already carrying one cylindrical void, cut
/// by a second cylinder overlapping that void, SHOULD lose only the crescent
/// of new material — cutting air may never abort the whole subtraction.
/// The difference currently fails validation
/// (`InvalidOutput { InvalidEulerCharacteristic(1), FreeEdge }`): the
/// imprint/classification of the overlapping cylindrical walls produces an
/// unsewable face set. This pins today's honest outcome — the guarded
/// boolean REFUSES to return broken geometry and the caller keeps the
/// material. A successful result must satisfy the exact union-volume oracle
/// below. Both façade feature paths now warn and preserve the source on
/// rejection; this does not establish overlapping-bore support.
#[test]
fn overlapping_second_bore_currently_fails_validation_cleanly() {
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
    let outcome = boolean_operation(&once, &second, BooleanOp::Cut);
    match outcome {
        Ok(result) => {
            // Two r=4 discs 5 mm apart remove their union area times height:
            // (2*pi*r² - intersection lens) * height ≈ 875 mm³.
            let r = 4.0_f64;
            let s = 5.0_f64;
            let lens =
                2.0 * r * r * (s / (2.0 * r)).acos() - 0.5 * s * (4.0 * r * r - s * s).sqrt();
            let removed = (2.0 * std::f64::consts::PI * r * r - lens) * 10.0;
            let expected = 30.0 * 30.0 * 10.0 - removed;
            assert!(expected < volume(&once));
            let measured = volume(&result.value);
            assert!(
                (measured - expected).abs() / expected < 5.0e-3,
                "with the kernel repaired, only the crescent goes: {measured} vs {expected}"
            );
        }
        Err(error) => {
            // Today's pinned behavior: the guarded difference refuses the
            // unsewable overlapping-wall result instead of returning broken
            // geometry.
            assert!(
                matches!(error, openrcad_algo::BooleanError::InvalidOutput { .. }),
                "the failure must stay a clean validation rejection: {error:?}"
            );
        }
    }
}
