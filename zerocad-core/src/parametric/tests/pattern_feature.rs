//! Pattern & Mirror as parametric features: replicate a body node's solids by
//! linear/circular arrays or a mirror plane.

use super::*;
use crate::parametric::{AxisBase, PatternKind, PlaneBase};

fn add_box(g: &mut ParametricGraph, id: &str, w: f32, h: f32, d: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Box { w, h, d },
    });
}

fn add_pattern(g: &mut ParametricGraph, id: &str, source: &str, kind: PatternKind) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Pattern {
            source: source.to_string(),
            kind,
        },
    });
    g.add_dependency(source, id);
}

fn total_volume(bodies: &[(String, MockMesh)]) -> f64 {
    bodies
        .iter()
        .filter_map(|(_, m)| m.mass_properties())
        .map(|mp| mp.volume)
        .sum()
}

#[test]
fn linear_pattern_replicates_box() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 3.0, 4.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Linear {
            dir: AxisBase::X,
            spacing: 10.0,
            spacing_expr: None,
            count: 4,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    // Source body + one pattern body holding the 3 extra instances.
    assert_eq!(bodies.len(), 2);
    let v = total_volume(&bodies);
    assert!((v - 4.0 * 24.0).abs() < 1e-3, "total volume {v}");
}

#[test]
fn circular_pattern_ring_spacing() {
    // 4 instances through a full 360° about Y: instances at 0/90/180/270 —
    // step must be 360/count so nothing lands back on instance 0.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1.0, 1.0, 1.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Circular {
            axis: AxisBase::Y,
            count: 4,
            total_angle_deg: 360.0,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = total_volume(&bodies);
    assert!((v - 4.0).abs() < 1e-3, "total volume {v}");
    // The pattern body's mesh must span negative X and Z (ring around origin),
    // not stack on the original.
    let mesh = &bodies.iter().find(|(id, _)| id == "pattern_2").unwrap().1;
    let (mut min_x, mut min_z) = (f32::MAX, f32::MAX);
    for vtx in mesh.vertices.chunks(6) {
        min_x = min_x.min(vtx[0]);
        min_z = min_z.min(vtx[2]);
    }
    assert!(min_x < -0.5, "ring should reach -X, min_x {min_x}");
    assert!(min_z < -0.5, "ring should reach -Z, min_z {min_z}");
}

#[test]
fn mirror_produces_well_oriented_copy() {
    // Box at [0,2]³ mirrored across YZ → copy in x∈[-2,0]. The reflection
    // flips handedness; the re-sew must leave the copy watertight with
    // positive volume and outward normals (mass_properties would go wrong on
    // an inside-out mesh).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
            face: None,
            offset: 0.0,
            offset_expr: None,
            join: false,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    let mirrored = &bodies.iter().find(|(id, _)| id == "pattern_2").unwrap().1;
    let mp = mirrored.mass_properties().expect("closed mirrored mesh");
    assert!(
        (mp.volume - 8.0).abs() < 1e-3,
        "mirror volume {}",
        mp.volume
    );
    assert!(
        (mp.centroid[0] + 1.0).abs() < 1e-3,
        "mirror centroid x {}",
        mp.centroid[0]
    );
}

#[test]
fn mirror_can_follow_a_planar_body_face() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let top = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|face| face.normal[2] > 0.99)
        .expect("box top face")
        .clone();
    let face = FaceRef {
        centroid: top.centroid,
        normal: top.normal,
        topology: Some(TopologyFaceRef {
            body_id: Some("box_1".to_string()),
            component_id: top.topology.as_ref().and_then(|t| t.component_id.clone()),
            topology_version: top.topology.as_ref().and_then(|t| t.topology_version),
            face_id: top.topology.as_ref().and_then(|t| t.face_id.clone()),
            surface_kind: top.topology.as_ref().and_then(|t| t.surface_kind.clone()),
            producer_feature_id: top
                .topology
                .as_ref()
                .and_then(|t| t.producer_feature_id.clone()),
            source_entity_id: top
                .topology
                .as_ref()
                .and_then(|t| t.source_entity_id.clone()),
        }),
    };
    add_pattern(
        &mut g,
        "mirror_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::XY,
            face: Some(face),
            offset: 0.0,
            offset_expr: None,
            join: false,
        },
    );

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let mirrored = &bodies.iter().find(|(id, _)| id == "mirror_2").unwrap().1;
    let mp = mirrored.mass_properties().expect("closed mirrored mesh");
    assert!(
        (mp.centroid[2] - 3.0).abs() < 1.0e-3,
        "mirroring [0,2] across its z=2 top face should center at z=3, got {}",
        mp.centroid[2]
    );
}

#[test]
fn mirror_offset_moves_copy_along_plane_normal() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    add_pattern(
        &mut g,
        "mirror_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
            face: None,
            offset: 3.0,
            offset_expr: None,
            join: false,
        },
    );
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let mirrored = &bodies.iter().find(|(id, _)| id == "mirror_2").unwrap().1;
    let mp = mirrored.mass_properties().expect("closed mirrored mesh");
    assert!(
        (mp.centroid[0] - 2.0).abs() < 1.0e-3,
        "reflection centroid -1 translated +3 should be x=2, got {}",
        mp.centroid[0]
    );
}

#[test]
fn mirror_join_unions_connected_copy_into_source_body() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    let source = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let side = source[0]
        .1
        .face_refs
        .iter()
        .find(|face| face.normal[0] > 0.99)
        .expect("box +X face");
    let face = FaceRef {
        centroid: side.centroid,
        normal: side.normal,
        topology: Some(TopologyFaceRef {
            body_id: Some("box_1".to_string()),
            component_id: side.topology.as_ref().and_then(|t| t.component_id.clone()),
            topology_version: side.topology.as_ref().and_then(|t| t.topology_version),
            face_id: side.topology.as_ref().and_then(|t| t.face_id.clone()),
            surface_kind: side.topology.as_ref().and_then(|t| t.surface_kind.clone()),
            producer_feature_id: side
                .topology
                .as_ref()
                .and_then(|t| t.producer_feature_id.clone()),
            source_entity_id: side
                .topology
                .as_ref()
                .and_then(|t| t.source_entity_id.clone()),
        }),
    };
    add_pattern(
        &mut g,
        "mirror_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
            face: Some(face),
            // Reflected [2,4] shares the source's x=2 side face.
            offset: 0.0,
            offset_expr: None,
            join: true,
        },
    );

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1, "connected mirror should become one body");
    assert_eq!(
        bodies[0].0, "box_1",
        "Mirror Join modifies the source body identity"
    );
    let mp = bodies[0].1.mass_properties().expect("closed joined mirror");
    assert!(
        (mp.volume - 16.0).abs() < 0.1,
        "joined volume {}",
        mp.volume
    );
    let mirror_plane_edges = bodies[0]
        .1
        .edge_refs
        .iter()
        .filter(|edge| (edge.p0[0] - 2.0).abs() < 1.0e-3 && (edge.p1[0] - 2.0).abs() < 1.0e-3)
        .count();
    assert_eq!(
        mirror_plane_edges, 0,
        "joined mirror must not expose its internal centre seam"
    );
    let mirror_plane_segments = bodies[0]
        .1
        .edge_indices
        .chunks_exact(2)
        .filter(|edge| {
            edge.iter().all(|&vertex| {
                let base = vertex as usize * 3;
                (bodies[0].1.edge_vertices[base] - 2.0).abs() < 1.0e-3
            })
        })
        .count();
    assert_eq!(
        mirror_plane_segments, 0,
        "joined mirror must not draw or pick a centre-plane segment"
    );
    let internal_cap_triangles = bodies[0]
        .1
        .indices
        .chunks_exact(3)
        .filter(|triangle| {
            triangle.iter().all(|&vertex| {
                let base = vertex as usize * 6;
                (bodies[0].1.vertices[base] - 2.0).abs() < 1.0e-3
            })
        })
        .count();
    assert_eq!(
        internal_cap_triangles, 0,
        "joined mirror must remove the coincident internal cap faces"
    );
    assert_eq!(
        bodies[0]
            .1
            .face_refs
            .iter()
            .filter(|face| face.normal[2] > 0.99)
            .count(),
        1,
        "the coplanar top halves must be one continuous selectable face"
    );
    assert!(
        bodies[0].1.face_refs.iter().all(|face| {
            face.topology
                .as_ref()
                .and_then(|topology| topology.face_id.as_deref())
                .is_some()
                && face
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.body_id.as_deref())
                    == Some("box_1")
        }),
        "joined mirror faces must remain durably selectable on the source body"
    );
}

#[test]
fn offset_mirror_join_names_result_faces_without_plane_cleanup() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_1", 2.0, 2.0, 2.0);
    add_pattern(
        &mut graph,
        "mirror_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
            face: None,
            // Mirrored [-2,0] translated to [-1,1], overlapping source [0,2].
            offset: 1.0,
            offset_expr: None,
            join: true,
        },
    );
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("offset mirror join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "box_1");
    assert!(
        bodies[0].1.face_refs.iter().all(|face| {
            face.topology
                .as_ref()
                .and_then(|topology| topology.face_id.as_deref())
                .is_some()
        }),
        "joined offset mirror faces must all have durable identities"
    );
}

#[test]
fn mirror_join_keeps_disconnected_copy_separate() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    add_pattern(
        &mut g,
        "mirror_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
            face: None,
            offset: 10.0,
            offset_expr: None,
            join: true,
        },
    );
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 2, "disconnected mirror stays a separate body");
    assert!(bodies.iter().any(|(id, _)| id == "box_1"));
    assert!(bodies.iter().any(|(id, _)| id == "mirror_2"));
}

#[test]
fn missing_source_warns() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1.0, 1.0, 1.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "nonexistent".to_string(),
            kind: PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 5.0,
                spacing_expr: None,
                count: 2,
            },
        },
    });
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert_eq!(output.bodies.len(), 1); // just the box
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "pattern_2"
                && diagnostic.code.as_str() == DiagnosticCode::REFERENCE_MISSING
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

#[test]
fn pattern_candidate_contract_cold_warm_cancel_and_restore() {
    let mut graph = ParametricGraph::new();
    add_box(&mut graph, "box_contract", 2.0, 2.0, 2.0);
    add_pattern(
        &mut graph,
        "pattern_contract",
        "box_contract",
        PatternKind::Linear {
            dir: AxisBase::X,
            spacing: 5.0,
            spacing_expr: None,
            count: 3,
        },
    );
    let hidden = std::collections::HashSet::new();
    let cold = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let warm = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty() && warm.1.is_empty());
    assert_eq!(cold.0.len(), warm.0.len());
    for ((cold_id, cold_mesh), (warm_id, warm_mesh)) in cold.0.iter().zip(&warm.0) {
        assert_eq!(cold_id, warm_id);
        assert_eq!(cold_mesh.indices, warm_mesh.indices);
    }
    let cancelled = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        graph.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled),
        ),
        Err(EvaluationError::Cancelled)
    ));
    let restored: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&graph.clone_document()).unwrap()).unwrap();
    assert!(restored
        .evaluate_bodies_with_warnings(&hidden)
        .unwrap()
        .1
        .is_empty());
}
