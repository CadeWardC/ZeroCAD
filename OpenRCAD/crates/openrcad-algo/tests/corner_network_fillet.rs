//! Production fixtures for N-valent rolling-ball corner networks.

use openrcad_algo::{
    fillet_edges, fillet_edges_operation_with_policy, sew_with_policy, CornerNetworkError,
    RollingBallError,
};
use openrcad_foundation::{Dir, Pnt, TolerancePolicy, Vec as GeomVec};
use openrcad_geom::{GeomSurface, Plane};
use openrcad_mesh::tessellate_checked;
use openrcad_topo::{Edge, Face, Solid, TopologyChange, TopologyKind, Wire};
use std::collections::HashMap;

fn planar_face(points: &[Pnt]) -> Face {
    assert!(points.len() >= 3);
    let normal = (points[1] - points[0])
        .cross(&(points[2] - points[0]))
        .normalized()
        .map(|normal| Dir::new(normal.x(), normal.y(), normal.z()))
        .expect("non-degenerate fixture face");
    let edges = (0..points.len())
        .map(|index| Edge::between_points(points[index], points[(index + 1) % points.len()]))
        .collect::<Vec<_>>();
    Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            points[0], normal,
        ))),
        Wire::from_edges(edges),
    )
}

fn regular_pyramid(
    valence: usize,
    scale: f64,
    rotation_degrees: f64,
    translation: GeomVec,
) -> (Solid, Vec<Pnt>, Pnt) {
    assert!((4..=7).contains(&valence));
    let angle = rotation_degrees.to_radians();
    let (sine, cosine) = angle.sin_cos();
    let map = |point: Pnt| {
        Pnt::new(
            cosine * point.x() - sine * point.y(),
            sine * point.x() + cosine * point.y(),
            point.z(),
        ) + translation
    };
    let base = (0..valence)
        .map(|index| {
            let angle = -core::f64::consts::FRAC_PI_2
                + 2.0 * core::f64::consts::PI * index as f64 / valence as f64;
            map(Pnt::new(
                10.0 * scale * angle.cos(),
                10.0 * scale * angle.sin(),
                0.0,
            ))
        })
        .collect::<Vec<_>>();
    let apex = map(Pnt::new(0.0, 0.0, 20.0 * scale));
    let mut reversed_base = base.clone();
    reversed_base.reverse();
    let mut faces = vec![planar_face(&reversed_base)];
    faces.extend(
        (0..valence).map(|index| planar_face(&[base[index], base[(index + 1) % valence], apex])),
    );
    let shell = sew_with_policy(&faces, &TolerancePolicy::STANDARD)
        .expect("pyramid fixture must sew")
        .value;
    let source = Solid::new(shell)
        .complete_missing_pcurves(&TolerancePolicy::STANDARD)
        .expect("pyramid fixture must admit exact planar pcurves")
        .0;
    source
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("pyramid fixture must be strict before filleting");
    (source, base, apex)
}

#[test]
fn four_through_six_valent_pyramid_corners_pass_the_mandatory_sweep() {
    for valence in 4..=6 {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for rotation in [0.0, 37.0] {
                for translation in [GeomVec::ZERO, GeomVec::new(1.0e9, -1.0e9, 5.0e8)] {
                    let (source, base, apex) =
                        regular_pyramid(valence, scale, rotation, translation);
                    let selected = base
                        .iter()
                        .map(|point| Edge::between_points(*point, apex))
                        .collect::<Vec<_>>();
                    let result = fillet_edges(&source, &selected, 2.0 * scale).unwrap_or_else(|error| {
                    panic!(
                        "verified {valence}-valent planar corner must fillet at scale={scale}, rotation={rotation}, translation={translation:?}: {error:?}"
                    )
                });
                    result
                    .validate_strict_with_policy(&TolerancePolicy::STANDARD)
                    .unwrap_or_else(|error| {
                        panic!(
                            "{valence}-valent result must pass strict validation at scale={scale}, rotation={rotation}, translation={translation:?}: {error:?}"
                        )
                    });
                    assert!(result.is_watertight());
                    assert!(result.health_report().is_healthy());
                    let spheres = result
                        .faces()
                        .into_iter()
                        .filter_map(|face| match face.surface() {
                            Some(GeomSurface::Sphere(sphere)) => Some(*sphere),
                            _ => None,
                        })
                        .collect::<Vec<_>>();
                    assert_eq!(spheres.len(), 1, "one exact corner sphere is required");
                    let radius_error = (spheres[0].radius() - 2.0 * scale).abs();
                    assert!(
                        radius_error <= TolerancePolicy::STANDARD.intersection,
                        "corner radius error {radius_error}"
                    );
                    source
                        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
                        .expect("candidate construction must not mutate its source");
                }
            }
        }
    }
}

#[test]
fn four_through_six_valent_corners_are_selection_order_independent() {
    for valence in 4..=6 {
        let (source, base, apex) = regular_pyramid(valence, 1.0, 37.0, GeomVec::ZERO);
        let selected = base
            .iter()
            .map(|point| Edge::between_points(*point, apex))
            .collect::<Vec<_>>();
        let expected = fillet_edges(&source, &selected, 2.0).unwrap();
        let expected_counts = (
            expected.vertex_count(),
            expected.edge_count(),
            expected.face_count(),
        );
        for shift in 0..selected.len() {
            let reordered = (0..selected.len())
                .rev()
                .map(|index| selected[(index + shift) % selected.len()].clone())
                .collect::<Vec<_>>();
            let actual = fillet_edges(&source, &reordered, 2.0).unwrap();
            actual
                .validate_strict_with_policy(&TolerancePolicy::STANDARD)
                .expect("every selection order must produce strict topology");
            assert_eq!(
                (
                    actual.vertex_count(),
                    actual.edge_count(),
                    actual.face_count(),
                ),
                expected_counts,
                "valence {valence}, shift {shift}"
            );
        }
    }
}

#[test]
fn four_through_six_valent_corner_meshes_use_shared_boundaries() {
    for valence in 4..=6 {
        let (source, base, apex) = regular_pyramid(valence, 1.0, 37.0, GeomVec::ZERO);
        let selected = base
            .iter()
            .map(|point| Edge::between_points(*point, apex))
            .collect::<Vec<_>>();
        let result = fillet_edges(&source, &selected, 2.0).unwrap();
        let mesh = tessellate_checked(&result, 0.05, 0.5).unwrap();
        let point_key = |index: u32| {
            let point = mesh.vertices[index as usize];
            let quantize = |value: f64| (value * 1.0e7).round() as i64;
            (
                quantize(point.x()),
                quantize(point.y()),
                quantize(point.z()),
            )
        };
        let mut incidence = HashMap::new();
        for triangle in &mesh.triangles {
            for (first, second) in [(0, 1), (1, 2), (2, 0)] {
                let mut edge = (point_key(triangle[first]), point_key(triangle[second]));
                if edge.1 < edge.0 {
                    edge = (edge.1, edge.0);
                }
                *incidence.entry(edge).or_insert(0usize) += 1;
            }
        }
        let open_edges = incidence.values().filter(|count| **count == 1).count();
        assert_eq!(open_edges, 0, "valence {valence} mesh has cracks");
    }
}

#[test]
fn incomplete_and_unverified_corner_networks_reject_before_mutation() {
    let (four_source, four_base, four_apex) = regular_pyramid(4, 1.0, 37.0, GeomVec::ZERO);
    let partial = four_base
        .iter()
        .take(3)
        .map(|point| Edge::between_points(*point, four_apex))
        .collect::<Vec<_>>();
    assert!(matches!(
        fillet_edges(&four_source, &partial, 2.0),
        Err(RollingBallError::CornerNetwork(
            CornerNetworkError::IncompleteSelection {
                selected_bands: 3,
                vertex_valence: 4
            }
        ))
    ));
    four_source
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("a rejected partial network must not mutate its source");

    let (seven_source, seven_base, seven_apex) = regular_pyramid(7, 1.0, 0.0, GeomVec::ZERO);
    let unsupported = seven_base
        .iter()
        .map(|point| Edge::between_points(*point, seven_apex))
        .collect::<Vec<_>>();
    assert!(matches!(
        fillet_edges(&seven_source, &unsupported, 2.0),
        Err(RollingBallError::CornerNetwork(
            CornerNetworkError::UnsupportedValence { valence: 7 }
        ))
    ));
    seven_source
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("an unsupported network must not mutate its source");
}

#[test]
fn corner_network_operation_reports_complete_face_and_edge_history() {
    let (source, base, apex) = regular_pyramid(4, 1.0, 37.0, GeomVec::ZERO);
    let selected = base
        .iter()
        .map(|point| Edge::between_points(*point, apex))
        .collect::<Vec<_>>();
    let result =
        fillet_edges_operation_with_policy(&source, &selected, 2.0, &TolerancePolicy::STANDARD)
            .expect("verified corner network must produce an operation candidate");
    result
        .history
        .validate()
        .expect("history must be well formed");
    assert!(result
        .history
        .coverage_for_solid(&result.value)
        .is_complete());
    for kind in [TopologyKind::Edge, TopologyKind::Face] {
        assert!(result.history.changes.iter().any(|change| matches!(
            change,
            TopologyChange::Generated { sources, result }
                if result.kind == kind && !sources.is_empty()
        )));
    }
}
