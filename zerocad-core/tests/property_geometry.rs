//! Bounded generative checks for the invariants every modeling operation must
//! preserve. Inputs deliberately stay in ordinary mechanical-part ranges: this
//! is a required regression gate, not a catalogue of unsupported degeneracies.

use openrcad::algo::{boolean_checked, BooleanOp};
use openrcad::foundation::{Ax2, Dir, Pnt};
use openrcad::primitives::{make_box, make_cylinder};
use proptest::prelude::*;

fn finite_solid(solid: &openrcad::topo::Solid) -> bool {
    solid.vertices().iter().all(|v| {
        let p = v.point();
        p.x().is_finite() && p.y().is_finite() && p.z().is_finite()
    })
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 16,
        max_shrink_iters: 256,
        .. ProptestConfig::default()
    })]

    /// A central through-hole is a common, well-defined mechanical operation.
    /// Across a useful dimensional range it must never leak, corrupt topology,
    /// or produce non-finite coordinates.
    #[test]
    fn central_through_holes_remain_valid(
        width in 8.0f64..80.0,
        depth in 8.0f64..80.0,
        height in 2.0f64..40.0,
        radius_ratio in 0.08f64..0.35,
    ) {
        let block = make_box(&Pnt::origin(), width, depth, height);
        let radius = width.min(depth) * radius_ratio;
        let drill = make_cylinder(
            &Ax2::new(Pnt::new(width * 0.5, depth * 0.5, -1.0), Dir::dz()),
            radius,
            height + 2.0,
        );

        let cut = boolean_checked(&block, &drill, BooleanOp::Cut)
            .expect("a bounded central through-hole must resolve");
        prop_assert!(cut.is_watertight());
        prop_assert!(cut.health_report().is_healthy());
        prop_assert!(finite_solid(&cut));

        // The same immutable operands must produce the same coarse topology.
        let again = boolean_checked(&block, &drill, BooleanOp::Cut)
            .expect("repeated deterministic cut must resolve");
        prop_assert_eq!(cut.vertex_count(), again.vertex_count());
        prop_assert_eq!(cut.edge_count(), again.edge_count());
        prop_assert_eq!(cut.face_count(), again.face_count());
    }
}
