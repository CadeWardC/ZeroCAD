//! Exact contact-curve continuation beyond a selected planar support.
//!
//! The verified first slice is a straight planar prism.  Its end profile is
//! clipped by the analytic rolling-ball arc and rebuilt exactly; a radius is
//! never made to "fit" by moving a contact point onto a neighboring vertex.

use core::f64::consts::FRAC_PI_2;

use openrcad_algo::{
    fillet_edges_operation_with_policy, fillet_planar_edge, prism_operation, FilletOverflowError,
    RollingBallError,
};
use openrcad_foundation::{Ax1, Ax3, Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_primitives::make_box_operation;
use openrcad_topo::{Edge, Face, Vertex, Wire};

fn transformed_overflow_case(
    scale: f64,
    height_aspect: f64,
    rotation_degrees: f64,
    translation: openrcad_foundation::Vec,
) -> (openrcad_topo::Solid, Edge, f64) {
    let height = scale * height_aspect;
    let rotation = Trsf::rotation(
        &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
        rotation_degrees.to_radians(),
    );
    let placement = Trsf::translation(translation);
    let transform = |point: Pnt| placement.transform_point(&rotation.transform_point(&point));
    let source = make_box_operation(&Pnt::origin(), scale, 4.0 * scale, height)
        .expect("overflow source box")
        .value
        .transformed(&rotation)
        .transformed(&placement);
    let selected = Edge::between_points(
        transform(Pnt::origin()),
        transform(Pnt::new(0.0, 0.0, height)),
    );
    (source, selected, 1.5 * scale)
}

fn assert_exact_overflow(
    scale: f64,
    height_aspect: f64,
    rotation_degrees: f64,
    translation: openrcad_foundation::Vec,
) {
    let (source, selected, radius) =
        transformed_overflow_case(scale, height_aspect, rotation_degrees, translation);
    let source_context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[source.bounding_box()],
        Some(radius),
        1.0,
    )
    .expect("overflow source tolerance context");
    source
        .validate_strict_with_policy(&source_context.policy)
        .unwrap_or_else(|error| {
            panic!(
                "overflow source is invalid at scale={scale}, aspect={height_aspect}, rotation={rotation_degrees}, translation={translation:?}: {error:?}"
            )
        });
    let result = fillet_planar_edge(&source, &selected, radius).unwrap_or_else(|error| {
        panic!(
            "planar overflow failed at scale={scale}, aspect={height_aspect}, rotation={rotation_degrees}, translation={translation:?}: {error:?}"
        )
    });
    let context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[result.bounding_box()],
        Some(radius),
        1.0,
    )
    .expect("overflow tolerance context");
    result
        .validate_strict_with_policy(&context.policy)
        .unwrap_or_else(|error| panic!("strict overflow result: {error:?}"));
    assert!(result.is_watertight_with_policy(&context.policy));
    assert!(result
        .health_report_with_policy(&context.policy)
        .is_healthy());
    let bands = result
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Cylinder(cylinder))
                if (cylinder.radius() - radius).abs() <= context.policy.intersection =>
            {
                Some(*cylinder)
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(bands.len(), 1, "one exact rolling-ball band is required");
    source
        .validate_strict_with_policy(&source_context.policy)
        .expect("candidate construction must leave the prevalidated source untouched");
}

#[test]
fn planar_overflow_has_the_requested_radius_and_volume() {
    assert_exact_overflow(1.0, 1.0, 0.0, openrcad_foundation::Vec::ZERO);
    let (source, selected, radius) =
        transformed_overflow_case(1.0, 1.0, 0.0, openrcad_foundation::Vec::ZERO);
    let result = fillet_planar_edge(&source, &selected, radius).unwrap();
    let mesh = openrcad_mesh::tessellate_checked_with_policy(
        &result,
        0.002,
        0.1,
        &TolerancePolicy::STANDARD,
    )
    .expect("overflow measurement mesh");
    let measured = openrcad_mesh::mass_properties(&mesh)
        .expect("overflow mass properties")
        .volume;

    // Remaining profile area for [0,1]x[0,4] after clipping the corner with
    // the radius-1.5 circle centered at (1.5, 1.5).  This independent analytic
    // certificate catches a watertight but wrong-side arc.
    let antiderivative = |x: f64| {
        0.5 * (x * (radius * radius - x * x).max(0.0).sqrt()
            + radius * radius * (x / radius).asin())
    };
    let circle_integral = antiderivative(1.0 - radius) - antiderivative(-radius);
    let removed_area = radius - circle_integral;
    let expected = 4.0 - removed_area;
    let relative_error = (measured - expected).abs() / expected;
    assert!(
        relative_error <= 2.0e-3,
        "overflow volume {measured}, expected {expected}, relative error {relative_error}"
    );
}

#[test]
fn planar_overflow_operation_has_complete_history_without_pcurve_recovery() {
    let (source, selected, radius) =
        transformed_overflow_case(1.0, 1.0, 37.0, openrcad_foundation::Vec::ZERO);
    let operation = fillet_edges_operation_with_policy(
        &source,
        &[selected],
        radius,
        &TolerancePolicy::STANDARD,
    )
    .expect("overflow operation result");
    assert!(operation.validation.is_valid());
    operation.history.validate().expect("overflow history");
    assert!(
        operation
            .history
            .coverage_for_solid(&operation.value)
            .is_complete(),
        "overflow history must cover every result entity"
    );
    assert!(
        operation.recovery.actions.iter().all(|action| !matches!(
            action,
            openrcad_topo::RecoveryAction::ReconstructPcurve { .. }
                | openrcad_topo::RecoveryAction::ReconstructPcurves { .. }
        )),
        "overflow pcurves must be construction-time exact: {:?}",
        operation.recovery.actions
    );
}

#[test]
fn planar_overflow_holds_across_scale_aspect_rotation_and_far_origin() {
    let cases = [
        (1.0e-3, 1.0, 0.0, openrcad_foundation::Vec::ZERO),
        (
            1.0e-3,
            1.0e3,
            37.0,
            openrcad_foundation::Vec::new(1.0e6, -1.0e6, 5.0e5),
        ),
        (1.0, 1.0e-3, 37.0, openrcad_foundation::Vec::ZERO),
        (
            1.0,
            1.0,
            90.0,
            openrcad_foundation::Vec::new(1.0e6, -1.0e6, 5.0e5),
        ),
        (
            1.0e3,
            1.0,
            0.0,
            openrcad_foundation::Vec::new(1.0e9, -1.0e9, 5.0e8),
        ),
        (1.0e3, 1.0e-3, 90.0, openrcad_foundation::Vec::ZERO),
    ];
    for (scale, aspect, rotation, translation) in cases {
        assert_exact_overflow(scale, aspect, rotation, translation);
    }
}

#[test]
fn planar_overflow_continues_across_multiple_successive_faces() {
    let points = [
        Pnt::origin(),
        Pnt::new(0.25, 0.0, 0.0),
        Pnt::new(0.6, 0.02, 0.0),
        Pnt::new(1.0, 0.1, 0.0),
        Pnt::new(1.0, 4.0, 0.0),
        Pnt::new(0.0, 4.0, 0.0),
    ];
    let edges = (0..points.len())
        .map(|index| Edge::between_points(points[index], points[(index + 1) % points.len()]))
        .collect::<Vec<_>>();
    let profile = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges(edges),
    );
    let source = prism_operation(&profile, openrcad_foundation::Vec::new(0.0, 0.0, 1.0))
        .expect("multi-successor prism")
        .value;
    let selected = Edge::between_points(Pnt::origin(), Pnt::new(0.0, 0.0, 1.0));
    let result = fillet_planar_edge(&source, &selected, 1.5)
        .expect("the exact contact curve must cross two successive planar faces");
    let context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[result.bounding_box()],
        Some(1.5),
        1.0,
    )
    .expect("multi-successor tolerance context");
    result
        .validate_strict_with_policy(&context.policy)
        .expect("multi-successor overflow must remain strict");
    assert!(result.is_watertight_with_policy(&context.policy));
    assert_eq!(
        result
            .faces()
            .into_iter()
            .filter(|face| matches!(
                face.surface(),
                Some(GeomSurface::Cylinder(cylinder))
                    if (cylinder.radius() - 1.5).abs() <= context.policy.intersection
            ))
            .count(),
        1
    );
}

#[test]
fn overflow_that_consumes_the_profile_rejects_atomically() {
    let source = make_box_operation(&Pnt::origin(), 1.0, 1.0, 1.0)
        .expect("atomic rejection source")
        .value;
    let selected = Edge::between_points(Pnt::origin(), Pnt::new(0.0, 0.0, 1.0));
    let rejection = fillet_planar_edge(&source, &selected, 4.0);
    assert!(
        matches!(
            rejection,
            Err(RollingBallError::Overflow(
                FilletOverflowError::ProfileTrim { .. }
            ))
        ),
        "profile-consuming overflow returned {rejection:?}"
    );
    source
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("a rejected overflow must not alter its source");
}

#[test]
fn curved_successor_rejects_instead_of_approximating_the_contact() {
    let circle = Circle::new(
        Ax3::new_axes(Pnt::new(1.0, 1.0, 0.0), Dir::dz(), Dir::dx()),
        1.0,
    );
    let arc = Edge::new(
        Some(GeomCurve::circle(circle)),
        -FRAC_PI_2,
        FRAC_PI_2,
        Vertex::new(circle.point(-FRAC_PI_2)),
        Vertex::new(circle.point(FRAC_PI_2)),
    );
    let profile = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
            arc,
            Edge::between_points(Pnt::new(1.0, 2.0, 0.0), Pnt::new(0.0, 2.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 2.0, 0.0), Pnt::origin()),
        ]),
    );
    let source = prism_operation(&profile, openrcad_foundation::Vec::new(0.0, 0.0, 1.0))
        .expect("D profile prism")
        .value;
    let selected = Edge::between_points(Pnt::origin(), Pnt::new(0.0, 0.0, 1.0));
    assert!(matches!(
        fillet_planar_edge(&source, &selected, 1.5),
        Err(RollingBallError::Overflow(
            FilletOverflowError::NonPrismaticSource
        ))
    ));
    source
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("a rejected curved successor must leave the source unchanged");
}
