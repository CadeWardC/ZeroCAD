use openrcad_algo::boolean::{
    boolean_bodies_operation_with_classes_policy_and_cancel_and_budget,
    boolean_operation_with_classes_policy_and_cancel_and_budget,
};
use openrcad_algo::{BandTopologyError, BooleanError, BooleanOp, GeometryWorkBudget};
use openrcad_foundation::{CancellationProbe, NeverCancelled, Pnt, TolerancePolicy};
use openrcad_primitives::make_box_operation;
use std::sync::atomic::{AtomicUsize, Ordering};

#[test]
fn exhausted_intersection_rejects_both_boolean_contracts_without_changing_operands() {
    let a = make_box_operation(&Pnt::origin(), 10., 10., 10.)
        .unwrap()
        .value;
    let b = make_box_operation(&Pnt::new(3., 2., 1.), 10., 10., 10.)
        .unwrap()
        .value;
    let before = (a.clone(), b.clone());
    let error = boolean_operation_with_classes_policy_and_cancel_and_budget(
        &a,
        &b,
        BooleanOp::Cut,
        None,
        None,
        &TolerancePolicy::STANDARD,
        &NeverCancelled,
        &mut GeometryWorkBudget::new(0),
    )
    .unwrap_err();
    assert!(matches!(
        error,
        BooleanError::Intersection(BandTopologyError::OperationBudgetExhausted { .. })
    ));
    let multi_error = boolean_bodies_operation_with_classes_policy_and_cancel_and_budget(
        &a,
        &b,
        BooleanOp::Cut,
        None,
        None,
        &TolerancePolicy::STANDARD,
        &NeverCancelled,
        &mut GeometryWorkBudget::new(0),
    )
    .unwrap_err();
    assert_eq!(multi_error, error);
    assert_eq!((a.clone(), b.clone()), before);
    assert!(openrcad_algo::boolean_operation(&a, &b, BooleanOp::Cut).is_ok());
}

#[test]
fn disjoint_union_needs_no_surface_intersection_budget() {
    let a = make_box_operation(&Pnt::origin(), 10., 10., 10.)
        .unwrap()
        .value;
    let b = make_box_operation(&Pnt::new(20., 0., 0.), 10., 10., 10.)
        .unwrap()
        .value;
    let result = boolean_bodies_operation_with_classes_policy_and_cancel_and_budget(
        &a,
        &b,
        BooleanOp::Fuse,
        None,
        None,
        &TolerancePolicy::STANDARD,
        &NeverCancelled,
        &mut GeometryWorkBudget::new(0),
    )
    .unwrap();
    assert_eq!(result.value.bodies.len(), 2);
}

struct CancelAfter(AtomicUsize);
impl CancellationProbe for CancelAfter {
    fn is_cancelled(&self) -> bool {
        self.0.fetch_add(1, Ordering::Relaxed) >= 2
    }
}

#[test]
fn recursive_surface_intersection_observes_cancellation() {
    use openrcad_foundation::{Ax3, Dir};
    use openrcad_geom::{GeomSurface, Plane, ToroidalSurface};
    let torus = GeomSurface::torus(ToroidalSurface::new(
        Ax3::new_axes(
            Pnt::origin(),
            Dir::new(0.36, 0.48, 0.8),
            Dir::new(0.8, -0.6, 0.0),
        ),
        4.,
        1.,
    ));
    let plane = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
    let mut budget = GeometryWorkBudget::intersection_default();
    let error = openrcad_algo::intersect::surface_surface_with_budget_and_cancel(
        &torus,
        &plane,
        1e-7,
        &mut budget,
        &CancelAfter(AtomicUsize::new(0)),
    )
    .unwrap_err();
    assert_eq!(error, BandTopologyError::Cancelled);
    assert!(budget.consumed() < budget.limit());
}
