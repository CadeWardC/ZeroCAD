//! Widening a slot: a second cut whose floor and ceiling coincide with the
//! first slot's and whose footprint contains it. The old floor's outline is
//! imprinted on the cutter's floor as a free-standing rectangle, then the
//! block's front and back walls queue full-width lines over two of its
//! edges. Those spans overlapped without shared vertices, so the cutter's
//! floor partitioned into a hole touching its own outer boundary and the cut
//! came back with non-manifold edges. Found by the ZeroCAD join/cut property
//! sweep (a saved mixed-axis seed). The fix splits queued spans only where
//! they overlap; the ring test keeps a transverse tangency span dangling.
use openrcad_algo::{boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};
use openrcad_topo::Solid;

fn block(corner: (f64, f64, f64), size: (f64, f64, f64)) -> Solid {
    make_box_operation(
        &Pnt::new(corner.0, corner.1, corner.2),
        size.0,
        size.1,
        size.2,
    )
    .expect("box")
    .value
}

/// Signed volume by the divergence theorem over the tessellated boundary.
fn volume(solid: &Solid) -> f64 {
    let mesh = openrcad_mesh::tessellate_checked(solid, 0.05, 0.35).expect("tessellable solid");
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

fn cut(a: &Solid, b: &Solid) -> Solid {
    let result = boolean_operation(a, b, BooleanOp::Cut).expect("cut").value;
    result
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("valid solid");
    result
}

#[test]
fn widening_a_through_slot_at_the_same_height() {
    let part = block((0.0, 0.0, 0.0), (40.0, 30.0, 8.0));
    let slot = block((5.0, -2.0, 1.0), (4.0, 50.0, 2.0));
    let wider = block((3.0, -2.0, 1.0), (12.0, 50.0, 2.0));
    let once = cut(&part, &slot);
    let twice = cut(&once, &wider);
    let expected = 40.0 * 30.0 * 8.0 - 12.0 * 2.0 * 30.0;
    let measured = volume(&twice);
    assert!(
        (measured - expected).abs() / expected < 1.0e-6,
        "widened slot: {measured} vs {expected}"
    );
}

#[test]
fn widening_a_blind_slot_at_the_same_height() {
    // A blind slot (stops at y = 20) widened by a through cutter.
    let part = block((0.0, 0.0, 0.0), (40.0, 30.0, 8.0));
    let slot = block((5.0, -2.0, 1.0), (4.0, 22.0, 2.0));
    let wider = block((3.0, -2.0, 1.0), (12.0, 50.0, 2.0));
    let once = cut(&part, &slot);
    let twice = cut(&once, &wider);
    let expected = 40.0 * 30.0 * 8.0 - 12.0 * 2.0 * 30.0;
    let measured = volume(&twice);
    assert!(
        (measured - expected).abs() / expected < 1.0e-6,
        "widened blind slot: {measured} vs {expected}"
    );
}

#[test]
fn tangent_inner_wall_does_not_split_the_cut_floor() {
    // A ring about a Y axis on the block's bottom edge line (x = 20, z = 0):
    // its inner radius 10 touches the top face (z = 10) along x = 20. That
    // tangency queues a short line whose ends land mid-span on the ring's
    // side-wall lines. It must stay dangling, not become a partition chord.
    let part = block((0.0, 0.0, 0.0), (40.0, 40.0, 10.0));
    let axis = Ax2::new(Pnt::new(20.0, 32.0, 0.0), Dir::dy());
    let cylinder = |radius| {
        make_cylinder_operation(&axis, radius, 3.0)
            .expect("cylinder")
            .value
    };
    let ring = boolean_operation(&cylinder(20.0), &cylinder(10.0), BooleanOp::Cut)
        .expect("ring")
        .value;
    let grooved = cut(&part, &ring);
    // Quarter-disc strip integral: area of z in [0, h] under sqrt(r^2 - z^2).
    let strip = |r: f64, h: f64| 0.5 * h * (r * r - h * h).sqrt() + 0.5 * r * r * (h / r).asin();
    let removed = 3.0 * 2.0 * (strip(20.0, 10.0) - strip(10.0, 10.0));
    let expected = 40.0 * 40.0 * 10.0 - removed;
    let measured = volume(&grooved);
    assert!(
        (measured - expected).abs() < 0.02 * removed,
        "ring groove: {measured} vs {expected}"
    );
}
