use openrcad_algo::{
    apply_blend_contour, shell_solid_operation_with_policy, BlendContour, BlendCurveHint,
    BlendError, BlendKind, BooleanOp,
};
use openrcad_foundation::{
    Ax1, Ax2, Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec,
};
use openrcad_geom::{GeomCurve, GeomSurface};
use openrcad_primitives::{make_box_operation_with_policy, make_cylinder_operation_with_policy};
use openrcad_topo::{Edge, Face, Solid};

fn furthest_planar_face(solid: &Solid, direction: Dir) -> Face {
    let direction_vector = GeomVec::from_dir(direction);
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Plane(plane))
                if plane
                    .normal()
                    .is_parallel(&direction, TolerancePolicy::STANDARD.angular) =>
            {
                let score = (plane.location() - Pnt::origin()).dot(&direction_vector);
                Some((score, face))
            }
            _ => None,
        })
        .max_by(|left, right| left.0.total_cmp(&right.0))
        .expect("top planar opening")
        .1
}

fn torus_banded_source(scale: f64, origin: Pnt) -> Solid {
    let policy = &TolerancePolicy::STANDARD;
    let block =
        make_box_operation_with_policy(&origin, 30.0 * scale, 30.0 * scale, 10.0 * scale, policy)
            .expect("fillet-band block")
            .value;
    let cutter = make_cylinder_operation_with_policy(
        &Ax2::new(
            Pnt::new(origin.x() + 30.0 * scale, origin.y(), origin.z() - scale),
            Dir::dz(),
        ),
        10.0 * scale,
        12.0 * scale,
        policy,
    )
    .expect("fillet-band cutter")
    .value;
    let bitten =
        openrcad_algo::boolean_operation_with_policy(&block, &cutter, BooleanOp::Cut, policy)
            .expect("fillet-band bite")
            .value;
    let top = furthest_planar_face(&bitten, Dir::dz());
    let arcs: Vec<Edge> = top
        .wires()
        .into_iter()
        .flat_map(|wire| wire.edges())
        .filter(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_))))
        .collect();
    assert!(!arcs.is_empty());
    let contour = BlendContour::constant(
        arcs,
        BlendKind::Fillet,
        3.0 * scale,
        Some(BlendCurveHint::Circle),
    );
    let result = apply_blend_contour(&bitten, &contour).expect("constant-radius circular fillet");
    assert!(result
        .shell()
        .faces()
        .iter()
        .any(|face| matches!(face.surface(), Some(GeomSurface::Torus(_)))));
    result
}

fn assert_torus_band_shell(source: &Solid, scale: f64, opening_direction: Dir, label: &str) {
    let policy = &TolerancePolicy::STANDARD;
    let first_opening = furthest_planar_face(source, opening_direction);
    let second_opening = furthest_planar_face(
        source,
        Dir::new(
            -opening_direction.x(),
            -opening_direction.y(),
            -opening_direction.z(),
        ),
    );
    let source_signature = (
        source.vertex_count(),
        source.edge_count(),
        source.face_count(),
        source.bounding_box().corners(),
    );
    let result = shell_solid_operation_with_policy(
        source,
        0.5 * scale,
        &[first_opening.clone(), second_opening.clone()],
        policy,
    )
    .unwrap_or_else(|error| panic!("{label}: shell across a regular torus band: {error}"));
    let reversed = shell_solid_operation_with_policy(
        source,
        0.5 * scale,
        &[second_opening, first_opening],
        policy,
    )
    .unwrap_or_else(|error| panic!("{label}: reversed opening order: {error}"));
    assert_eq!(
        (
            result.value.vertex_count(),
            result.value.edge_count(),
            result.value.face_count(),
        ),
        (
            reversed.value.vertex_count(),
            reversed.value.edge_count(),
            reversed.value.face_count(),
        ),
        "{label}: opening traversal order changed topology counts"
    );
    let (result_min, result_max) = result.value.bounding_box().corners().expect("Shell bounds");
    let (reversed_min, reversed_max) = reversed
        .value
        .bounding_box()
        .corners()
        .expect("reversed Shell bounds");
    assert!(
        result_min.distance(&reversed_min) <= policy.classification
            && result_max.distance(&reversed_max) <= policy.classification,
        "{label}: opening traversal order changed bounds"
    );
    assert!(result.validation.is_valid(), "{label}: invalid result");
    assert!(
        result.value.validate_strict_with_policy(policy).is_ok(),
        "{label}: strict validation failed"
    );
    assert!(
        result.value.has_complete_pcurves(),
        "{label}: incomplete pcurves"
    );
    assert!(
        !result.recovery.actions.iter().any(|action| matches!(
            action,
            openrcad_topo::RecoveryAction::ReconstructPcurves { .. }
        )),
        "{label}: generated coedges must carry construction-time pcurves"
    );
    assert!(
        result
            .history
            .coverage_for_solid(&result.value)
            .is_complete(),
        "{label}: incomplete history"
    );
    let context = ToleranceContext::derive(
        policy,
        &[result.value.bounding_box()],
        Some(0.5 * scale),
        1.0,
    )
    .unwrap();
    let evidence = openrcad_algo::offset::measure_shell_wall_thickness(
        &result.value,
        0.5 * scale,
        &context.policy,
    )
    .unwrap_or_else(|error| panic!("{label}: wall-thickness certificate: {error}"));
    let reversed_evidence = openrcad_algo::offset::measure_shell_wall_thickness(
        &reversed.value,
        0.5 * scale,
        &context.policy,
    )
    .unwrap_or_else(|error| panic!("{label}: reversed wall-thickness certificate: {error}"));
    assert!(evidence.samples > 0, "{label}: no thickness samples");
    assert_eq!(evidence.samples, reversed_evidence.samples);
    assert!(
        (evidence.measured_min - reversed_evidence.measured_min).abs() <= evidence.tolerance
            && (evidence.measured_max - reversed_evidence.measured_max).abs() <= evidence.tolerance,
        "{label}: opening traversal order changed thickness evidence"
    );
    assert_eq!(
        (
            source.vertex_count(),
            source.edge_count(),
            source.face_count(),
            source.bounding_box().corners(),
        ),
        source_signature,
        "{label}: the input changed during candidate construction"
    );
}

#[test]
fn shell_crosses_constant_radius_torus_band_atomically() {
    let scale = 1.0;
    let source = torus_banded_source(scale, Pnt::origin());
    assert_torus_band_shell(&source, scale, Dir::dz(), "local unit fixture");
}

fn assert_transformed_torus_band_shell(scale: f64, placement: Trsf, label: &str) {
    // Build the canonical acceptance body through public operations once,
    // then apply an exact public transform for the sweep. This isolates the
    // Shell scale contract from the separately promoted Boolean corpus while
    // preserving analytic supports and topology.
    let transform = placement.multiply(&Trsf::scale(&Pnt::origin(), scale));
    let source = torus_banded_source(1.0, Pnt::origin()).transformed(&transform);
    assert_torus_band_shell(&source, scale, transform.transform_dir(&Dir::dz()), label);
}

#[test]
fn torus_band_shell_holds_at_small_scale() {
    assert_transformed_torus_band_shell(1.0e-3, Trsf::IDENTITY, "small local");
}

#[test]
fn torus_band_shell_holds_under_arbitrary_rotation() {
    assert_transformed_torus_band_shell(
        1.0,
        Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
            37.0_f64.to_radians(),
        ),
        "arbitrary rotation",
    );
}

#[test]
fn torus_band_shell_holds_at_large_far_origin() {
    assert_transformed_torus_band_shell(
        1.0e3,
        Trsf::translation(GeomVec::new(1.0e9, -2.0e9, 3.0e9)),
        "large far origin",
    );
}

#[test]
fn far_origin_raw_torus_band_shell_candidate_is_bounded_and_watertight() {
    let scale = 1.0e3;
    let placement = Trsf::translation(GeomVec::new(1.0e9, -2.0e9, 3.0e9));
    let transform = placement.multiply(&Trsf::scale(&Pnt::origin(), scale));
    let source = torus_banded_source(1.0, Pnt::origin()).transformed(&transform);
    let opening = furthest_planar_face(&source, transform.transform_dir(&Dir::dz()));
    let candidate = openrcad_algo::shell_solid_with_policy(
        &source,
        0.5 * scale,
        &[opening],
        &TolerancePolicy::STANDARD,
    )
    .expect("far-origin raw torus Shell candidate");
    assert!(candidate.is_watertight_with_policy(&TolerancePolicy::STANDARD));
}

#[test]
fn far_origin_torus_source_transform_is_bounded() {
    let transform = Trsf::translation(GeomVec::new(1.0e9, -2.0e9, 3.0e9))
        .multiply(&Trsf::scale(&Pnt::origin(), 1.0e3));
    let source = torus_banded_source(1.0, Pnt::origin()).transformed(&transform);
    assert!(source.face_count() > 0);
    assert!(source.bounding_box().corners().is_some());
}

#[test]
fn torus_band_collapse_rejects_before_mutating_the_source() {
    let scale = 1.0;
    let policy = &TolerancePolicy::STANDARD;
    let source = torus_banded_source(scale, Pnt::origin());
    let opening = furthest_planar_face(&source, Dir::dz());
    let source_signature = (
        source.vertex_count(),
        source.edge_count(),
        source.face_count(),
        source.bounding_box().corners(),
    );
    let error = openrcad_algo::shell_solid_with_policy(&source, 4.0, &[opening], policy)
        .expect_err("a collapsed torus offset must reject");
    assert!(
        matches!(error, BlendError::ParameterTooLarge { .. }),
        "collapse must be a typed size rejection, got {error:?}"
    );
    assert_eq!(
        (
            source.vertex_count(),
            source.edge_count(),
            source.face_count(),
            source.bounding_box().corners(),
        ),
        source_signature,
        "rejected Shell changed its input"
    );
}
