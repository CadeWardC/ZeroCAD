//! A transverse slab must cut the cylinder while preserving both rim branches.
use zerocad_core::{mock_kernel, CoordinateSystem};

#[test]
fn transverse_slab_cut_preserves_cylinder_remainder() {
    for radius in [6., 7.5] {
        let cylinder = mock_kernel::cylinder_solid(radius, 20.).unwrap();
        let slab = mock_kernel::extruded_region_solid(
            &[(-8., -8.), (8., -8.), (8., 8.), (-8., 8.)],
            &[],
            4.,
            &CoordinateSystem::XY,
        )
        .unwrap();
        let result = openrcad::algo::boolean_bodies_operation_with_classes_policy_and_cancel(
            &cylinder,
            &slab,
            openrcad::algo::BooleanOp::Cut,
            None,
            None,
            &openrcad::foundation::TolerancePolicy::STANDARD,
            &openrcad::foundation::NeverCancelled,
        )
        .unwrap_or_else(|error| panic!("radius {radius}: {error:?}"));
        let parts = result.value.bodies;
        assert_eq!(parts.len(), 1);
        assert!(parts[0].is_watertight());
        let r = f64::from(radius);
        let removed = 8. * (4. * (r * r - 16.).sqrt() + r * r * (4. / r).asin());
        let expected = std::f64::consts::PI * r * r * 20. - removed;
        let mesh = mock_kernel::MockMesh::from_solid(&parts[0]);
        let actual = mesh.mass_properties().unwrap().volume;
        assert!(
            (actual - expected).abs() < expected * 0.01,
            "{actual} != {expected}"
        );
    }
}
