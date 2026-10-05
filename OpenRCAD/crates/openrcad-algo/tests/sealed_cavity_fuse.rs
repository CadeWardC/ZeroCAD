//! A fuse can seal off a cavity: a block bridging a blind side tunnel closes
//! the tunnel's far end into an enclosed void. The result is one material
//! region bounded by an outer shell and a void shell. Found by the ZeroCAD
//! mixed-axis join/cut census (chain 108): only cuts were packaged as nested
//! shells, so the fuse came back as one shell with two disconnected
//! components and was rejected; the multi-body adapter's history then missed
//! the void shell's wires.
use openrcad_algo::{boolean_operation, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};
use openrcad_primitives::{make_box_operation, make_cylinder_operation};
use openrcad_topo::Solid;

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

#[test]
fn block_bridging_a_blind_tunnel_seals_a_void() {
    let block = make_box_operation(&Pnt::origin(), 60.0, 36.0, 11.3)
        .expect("block")
        .value;
    let radius = 2.9;
    // Blind tunnel along +Y, 19 mm into the block.
    let tunnel = make_cylinder_operation(
        &Ax2::new(Pnt::new(21.4, -2.0, 5.5), Dir::dy()),
        radius,
        21.0,
    )
    .expect("tunnel")
    .value;
    let tunnelled = boolean_operation(&block, &tunnel, BooleanOp::Cut)
        .expect("blind tunnel")
        .value;
    // Bridge across the tunnel between y = 10.3 and 15.5, overhanging x < 0.
    let bridge = make_box_operation(&Pnt::new(-14.0, 10.3, 1.2), 44.0, 5.2, 8.1)
        .expect("bridge")
        .value;
    let fused = boolean_operation(&tunnelled, &bridge, BooleanOp::Fuse)
        .expect("the bridge seals the tunnel's far end")
        .value;
    fused
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("valid solid");
    assert_eq!(fused.shells().len(), 2, "outer shell plus the sealed void");

    // The multi-body adapter must account for the void shell in its history.
    let bodies = openrcad_algo::boolean_bodies_operation_with_classes_policy_and_cancel(
        &tunnelled,
        &bridge,
        BooleanOp::Fuse,
        None,
        None,
        &TolerancePolicy::STANDARD,
        &openrcad_foundation::NeverCancelled,
    )
    .expect("the multi-body fuse seals the void too")
    .value
    .bodies;
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].shells().len(), 2);

    let disc = std::f64::consts::PI * radius * radius;
    // Block, minus the tunnel's 19 mm, plus the overhang, plus the tunnel
    // stretch the bridge refills.
    let expected = 60.0 * 36.0 * 11.3 - disc * 19.0 + 14.0 * 5.2 * 8.1 + disc * 5.2;
    let measured = volume(&fused);
    assert!(
        (measured - expected).abs() / expected < 1.0e-3,
        "sealed-void volume: {measured} vs {expected}"
    );
}
