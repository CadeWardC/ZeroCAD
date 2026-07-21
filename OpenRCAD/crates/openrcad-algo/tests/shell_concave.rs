use openrcad_algo::{
    prism_operation_with_policy, shell_solid_operation_with_policy, BlendError,
    ModelingOperationError,
};
use openrcad_foundation::{Ax1, Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad_geom::{Curve, GeomSurface, Plane, Surface};
use openrcad_topo::{Edge, Face, Orientation, Solid, Wire};

fn l_prism(scale: f64, aspect: f64, angle_degrees: f64, far_origin: bool) -> Solid {
    // Keep the smallest modeled feature at `scale` while stretching either
    // the section or the height. This exercises 1e-3/1e3 aspect conditioning
    // without asking a fixed document policy to represent a feature below its
    // declared resolution at the simultaneous minimum scale.
    let section_scale = scale * (1.0 / aspect).max(1.0);
    let height_scale = scale * aspect.max(1.0);
    let point = |x: f64, y: f64| Pnt::new(x * section_scale, y * section_scale, 0.0);
    let points = [
        point(0.0, 0.0),
        point(8.0, 0.0),
        point(8.0, 3.0),
        point(3.0, 3.0),
        point(3.0, 8.0),
        point(0.0, 8.0),
    ];
    let edges = (0..points.len())
        .map(|index| Edge::between_points(points[index], points[(index + 1) % points.len()]))
        .collect::<Vec<_>>();
    let face = Face::with_wires(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Some(Wire::from_edges(edges)),
        Vec::new(),
        Orientation::Forward,
    );
    let mut solid = prism_operation_with_policy(
        &face,
        GeomVec::new(0.0, 0.0, 5.0 * height_scale),
        &TolerancePolicy::STANDARD,
    )
    .expect("L-profile prism")
    .value;
    if angle_degrees != 0.0 {
        solid = solid.transformed(&Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            angle_degrees.to_radians(),
        ));
    }
    if far_origin {
        solid = solid.transformed(&Trsf::translation(GeomVec::new(
            2.0e6 * scale,
            -3.0e6 * scale,
            5.0e5 * scale,
        )));
    }
    solid
}

fn top_face(solid: &Solid) -> Face {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Plane(plane)) if plane.normal().z().abs() > 0.9 => {
                Some((plane.location().z(), face))
            }
            _ => None,
        })
        .max_by(|(first, _), (second, _)| first.total_cmp(second))
        .expect("top face")
        .1
}

fn assert_coedges_are_parameter_consistent(solid: &Solid, label: &str) {
    let context = ToleranceContext::derive(
        &TolerancePolicy::STANDARD,
        &[solid.bounding_box()],
        None,
        1.0,
    )
    .expect("test tolerance context");
    for (face_index, face) in solid.shell().faces().iter().enumerate() {
        let surface = face.surface().expect("Shell face support");
        for wire in face.wires() {
            for (edge_index, edge) in wire.edges().into_iter().enumerate() {
                let tolerance = edge
                    .tolerance()
                    .max(context.policy.pcurve_consistency)
                    .max(context.convergence)
                    * 16.0;
                if let Some(curve) = edge.curve() {
                    assert!(
                        curve.point(edge.first()).distance(&edge.start().point()) <= tolerance,
                        "{label}: face {face_index} edge {edge_index} first/start mismatch"
                    );
                    assert!(
                        curve.point(edge.last()).distance(&edge.end().point()) <= tolerance,
                        "{label}: face {face_index} edge {edge_index} last/end mismatch"
                    );
                }
                let pcurve = wire
                    .pcurve(edge_index)
                    .expect("every generated concave Shell coedge has a pcurve");
                for (sample, fraction) in [0.0, 0.25, 0.5, 0.75, 1.0].into_iter().enumerate() {
                    let uv = pcurve.point_at_fraction(fraction);
                    let lifted = surface.point(uv.x(), uv.y());
                    let expected = edge.curve().map_or_else(
                        || edge.start().point().midpoint(&edge.end().point()),
                        |curve| curve.point(edge.first() + (edge.last() - edge.first()) * fraction),
                    );
                    let expected = if edge.curve().is_none() && sample != 2 {
                        if sample == 0 {
                            edge.start().point()
                        } else if sample == 4 {
                            edge.end().point()
                        } else {
                            edge.start().point()
                                + (edge.end().point() - edge.start().point()) * fraction
                        }
                    } else {
                        expected
                    };
                    assert!(
                        lifted.distance(&expected) <= tolerance,
                        "{label}: face {face_index} edge {edge_index} pcurve sample {sample} mismatch"
                    );
                }
            }
        }
    }
}

#[test]
fn l_prism_shell_is_atomic_watertight_and_measured() {
    let policy = &TolerancePolicy::STANDARD;
    let source = l_prism(1.0, 1.0, 0.0, false);
    let opening = top_face(&source);
    let operation = shell_solid_operation_with_policy(&source, 0.5, &[opening.clone()], policy)
        .expect("concave L-prism Shell");
    assert!(operation.validation.is_valid());
    assert!(operation.value.validate_strict_with_policy(policy).is_ok());
    assert!(operation.value.has_complete_pcurves());
    assert!(operation.history.validate().is_ok());
    assert_coedges_are_parameter_consistent(&operation.value, "concave L-prism Shell");
    let tessellation =
        openrcad_mesh::tessellate_checked_with_policy(&operation.value, 0.05, 0.1, policy)
            .expect("shared-edge concave Shell tessellation");
    assert!(openrcad_mesh::mass_properties(&tessellation)
        .is_some_and(|properties| properties.volume > 0.0));
    let evidence = openrcad_algo::offset::certify_concave_shell_with_policy(
        &source,
        &operation.value,
        0.5,
        &[opening],
        policy,
    )
    .expect("concave envelope certificate")
    .expect("L-prism must be classified as concave");
    assert!(evidence.concave_edges > 0);
    assert!(evidence.kept_cells > 0);
    assert_eq!(evidence.discarded_cells, 0);
    assert_eq!(evidence.unresolved_branches, 0);
    assert!((evidence.minimum_thickness - 0.5).abs() < 1.0e-9);
    assert!((evidence.maximum_thickness - 0.5).abs() < 1.0e-9);
    assert!(operation
        .diagnostics
        .iter()
        .any(|diagnostic| diagnostic.code == "shell.concave_certificate"));
}

#[test]
fn oversized_concave_shell_reports_its_typed_failure() {
    let policy = &TolerancePolicy::STANDARD;
    let source = l_prism(1.0, 1.0, 0.0, false);
    let source_before = source.clone();
    let opening = top_face(&source);
    let error = shell_solid_operation_with_policy(&source, 2.0, &[opening], policy)
        .expect_err("oversized concave Shell must reject");
    assert!(matches!(
        error,
        ModelingOperationError::Shell(BlendError::ConcaveShell(
            openrcad_algo::offset::ConcaveShellError::OffsetCollapse { .. }
        ))
    ));
    assert_eq!(source, source_before, "rejection mutated the source solid");
}

#[test]
fn concave_shell_full_scale_aspect_rotation_and_origin_sweep() {
    let policy = &TolerancePolicy::STANDARD;
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for aspect in [1.0e-3, 1.0, 1.0e3] {
            for angle in [0.0, 37.0, 90.0] {
                for far_origin in [false, true] {
                    let source = l_prism(scale, aspect, angle, far_origin);
                    let opening = top_face(&source);
                    let thickness = scale * 0.5;
                    let label =
                        format!("scale={scale} aspect={aspect} angle={angle} far={far_origin}");
                    let operation = shell_solid_operation_with_policy(
                        &source,
                        thickness,
                        &[opening.clone()],
                        policy,
                    )
                    .unwrap_or_else(|error| panic!("{label}: {error:?}"));
                    assert!(operation.validation.is_valid(), "{label}");
                    assert!(
                        operation.value.validate_strict_with_policy(policy).is_ok(),
                        "{label}"
                    );
                    let certificate = openrcad_algo::offset::certify_concave_shell_with_policy(
                        &source,
                        &operation.value,
                        thickness,
                        &[opening],
                        policy,
                    )
                    .unwrap_or_else(|error| panic!("{label}: certificate: {error}"))
                    .unwrap_or_else(|| panic!("{label}: source was not classified concave"));
                    let tolerance = (policy.linear * 10.0).max(thickness * 0.005);
                    assert!(
                        (certificate.minimum_thickness - thickness).abs() <= tolerance,
                        "{label}: {certificate:?}"
                    );
                    assert!(
                        (certificate.maximum_thickness - thickness).abs() <= tolerance,
                        "{label}: {certificate:?}"
                    );
                    assert_eq!(certificate.unresolved_branches, 0, "{label}");
                }
            }
        }
    }
}
