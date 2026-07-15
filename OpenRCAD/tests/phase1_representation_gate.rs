//! One cross-cutting Phase 1 representation gate.
//!
//! Constructor-local checks are useful, but this test deliberately consumes
//! every canonical result from the outside and applies the same strict topology
//! and tessellation contract to primitives, sweeps, STEP imports, and booleans.

use std::time::{SystemTime, UNIX_EPOCH};

use openrcad::algo::{
    boolean_operation_with_policy, prism_operation_with_policy, revolve_operation_with_policy,
    BooleanOp,
};
use openrcad::exchange::{read_step_operation_with_policy, write_step};
use openrcad::foundation::{Ax2, Dir, Pnt, TolerancePolicy, Vec as GeomVec};
use openrcad::geom::{GeomSurface, Plane};
use openrcad::mesh::tessellate_checked_with_policy;
use openrcad::primitives::{
    make_box_operation_with_policy, make_cone_operation_with_policy,
    make_cylinder_operation_with_policy, make_sphere_operation_with_policy,
    make_wedge_operation_with_policy,
};
use openrcad::topo::{Edge, Face, OperationResult, Solid, Wire};

fn accept(
    label: &'static str,
    result: OperationResult<Solid>,
    policy: &TolerancePolicy,
    solids: &mut Vec<(&'static str, Solid)>,
) {
    assert!(
        result.validation.is_valid(),
        "{label}: invalid result metadata"
    );
    assert!(
        result
            .history
            .coverage_for_solid(&result.value)
            .is_complete(),
        "{label}: incomplete topology-history coverage"
    );
    result
        .value
        .validate_strict_with_policy(policy)
        .unwrap_or_else(|error| panic!("{label}: strict validation failed: {error}"));
    solids.push((label, result.value));
}

fn rectangle_profile() -> Face {
    let points = [
        Pnt::new(1.0, 0.0, 0.0),
        Pnt::new(3.0, 0.0, 0.0),
        Pnt::new(3.0, 0.0, 2.0),
        Pnt::new(1.0, 0.0, 2.0),
    ];
    Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            points[0],
            Dir::dy(),
        ))),
        Wire::from_edges([
            Edge::between_points(points[0], points[1]),
            Edge::between_points(points[1], points[2]),
            Edge::between_points(points[2], points[3]),
            Edge::between_points(points[3], points[0]),
        ]),
    )
}

#[test]
fn phase1_representation_gate_is_cross_cutting_and_strict() {
    let policy = TolerancePolicy::STANDARD;
    let axis = Ax2::new(Pnt::origin(), Dir::dz());
    let mut solids = Vec::new();

    accept(
        "box",
        make_box_operation_with_policy(&Pnt::origin(), 10.0, 8.0, 6.0, &policy).unwrap(),
        &policy,
        &mut solids,
    );
    accept(
        "cylinder",
        make_cylinder_operation_with_policy(&axis, 3.0, 7.0, &policy).unwrap(),
        &policy,
        &mut solids,
    );
    accept(
        "cone",
        make_cone_operation_with_policy(&axis, 4.0, 1.5, 7.0, &policy).unwrap(),
        &policy,
        &mut solids,
    );
    accept(
        "sphere",
        make_sphere_operation_with_policy(&Pnt::new(2.0, 1.0, 0.0), 3.0, &policy).unwrap(),
        &policy,
        &mut solids,
    );
    accept(
        "wedge",
        make_wedge_operation_with_policy(8.0, 5.0, 4.0, 3.0, &policy).unwrap(),
        &policy,
        &mut solids,
    );

    let profile = rectangle_profile();
    let prism =
        prism_operation_with_policy(&profile, GeomVec::new(0.0, 4.0, 0.0), &policy).unwrap();
    assert!(prism.recovery.actions.is_empty());
    accept("prism", prism, &policy, &mut solids);

    let revolve = revolve_operation_with_policy(
        &profile,
        Pnt::origin(),
        Dir::dz(),
        core::f64::consts::PI,
        &policy,
    )
    .unwrap();
    assert!(revolve.recovery.actions.is_empty());
    accept("partial revolve", revolve, &policy, &mut solids);

    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let step_path = std::env::temp_dir().join(format!(
        "openrcad-phase1-gate-{}-{stamp}.step",
        std::process::id()
    ));
    let step_path_text = step_path.to_string_lossy();
    write_step(&solids[0].1, &step_path_text).expect("strict STEP write");
    let imported =
        read_step_operation_with_policy(&step_path_text, &policy).expect("strict STEP read");
    let _ = std::fs::remove_file(&step_path);
    assert!(imported.recovery.actions.is_empty());
    let imported_solid = imported.value.clone();
    accept("STEP round trip", imported, &policy, &mut solids);

    let boolean_a =
        make_box_operation_with_policy(&Pnt::new(-2.0, -2.0, -2.0), 8.0, 8.0, 8.0, &policy)
            .unwrap()
            .value;
    let boolean_b =
        make_box_operation_with_policy(&Pnt::new(2.0, 0.0, 0.0), 8.0, 6.0, 6.0, &policy)
            .unwrap()
            .value;
    for (label, operation) in [
        ("boolean fuse", BooleanOp::Fuse),
        ("boolean cut", BooleanOp::Cut),
        ("boolean common", BooleanOp::Common),
    ] {
        accept(
            label,
            boolean_operation_with_policy(&boolean_a, &boolean_b, operation, &policy).unwrap(),
            &policy,
            &mut solids,
        );
    }
    let imported_cut_tool = make_cylinder_operation_with_policy(
        &Ax2::new(Pnt::new(5.0, 4.0, -1.0), Dir::dz()),
        1.0,
        8.0,
        &policy,
    )
    .unwrap()
    .value;
    accept(
        "imported/native boolean",
        boolean_operation_with_policy(&imported_solid, &imported_cut_tool, BooleanOp::Cut, &policy)
            .unwrap(),
        &policy,
        &mut solids,
    );

    for (label, solid) in solids {
        solid
            .validate_strict_with_policy(&policy)
            .unwrap_or_else(|error| panic!("{label}: final strict gate failed: {error}"));
        let mesh = tessellate_checked_with_policy(&solid, 0.05, 0.25, &policy)
            .unwrap_or_else(|error| panic!("{label}: strict tessellation failed: {error}"));
        assert!(!mesh.triangles.is_empty(), "{label}: empty mesh");
        assert!(
            mesh.triangles.iter().all(|triangle| {
                let a = mesh.vertices[triangle[0] as usize];
                let b = mesh.vertices[triangle[1] as usize];
                let c = mesh.vertices[triangle[2] as usize];
                (b - a).cross(&(c - a)).magnitude() > policy.linear * policy.linear
            }),
            "{label}: degenerate triangle escaped strict tessellation"
        );
    }
}
