use super::*;

fn box_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 12.0,
            d: 10.0,
        },
    });
    graph
}

fn extrude_graph(origin: [f32; 3], width: f32, height: f32, depth: f32) -> ParametricGraph {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (width, height));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY.with_origin(Vec3::new(origin[0], origin[1], origin[2])),
            curves,
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: Vec::new(),
            next_entity_id: 1,
            solver: None,
        },
    });
    add_extrude(
        &mut graph,
        "extrude_1",
        "sketch_1",
        depth,
        ExtrudeMode::NewBody,
    );
    graph
}

fn capture_face(face: &crate::mock_kernel::MeshFaceRef) -> FaceRef {
    FaceRef {
        centroid: face.centroid,
        normal: face.normal,
        topology: face.topology.as_ref().map(|topology| TopologyFaceRef {
            body_id: topology.body_id.clone(),
            component_id: topology.component_id.clone(),
            topology_version: topology.topology_version,
            face_id: topology.face_id.clone(),
            surface_kind: topology.surface_kind.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    }
}

fn box_faces(graph: &ParametricGraph) -> (MockMesh, FaceRef, FaceRef) {
    let bodies = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("box fixture");
    let mesh = bodies[0].1.clone();
    let side = mesh
        .face_refs
        .iter()
        .find(|face| face.normal[0] > 0.9)
        .expect("right side");
    let neutral = mesh
        .face_refs
        .iter()
        .find(|face| face.normal[2] < -0.9)
        .expect("base cap");
    let side = capture_face(side);
    let neutral = capture_face(neutral);
    (mesh, side, neutral)
}

fn add_draft(
    graph: &mut ParametricGraph,
    face: FaceRef,
    neutral: FaceRef,
    angle_deg: f32,
    flip_pull: bool,
) {
    graph.add_feature(FeatureNode {
        id: "draft_1".to_string(),
        name: "Draft".to_string(),
        feature: FeatureType::Draft {
            target: "box_1".to_string(),
            faces: vec![face],
            neutral: DraftNeutral::Face(neutral),
            angle_deg,
            angle_expr: None,
            flip_pull,
        },
    });
    graph.add_dependency("box_1", "draft_1");
}

#[test]
fn standalone_draft_tapers_selected_planar_side_and_reuses_warm() {
    let mut graph = box_graph();
    let (_, side, neutral) = box_faces(&graph);
    add_draft(&mut graph, side, neutral, 6.0, false);

    let (cold, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("cold Draft");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(cold.len(), 1);
    assert!(cold[0].1.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| name.starts_with("draft:draft_1:region:0:face:"))
    }));
    let inspection = graph.inspect_body("box_1").expect("drafted body volume");
    assert!(inspection.volume_mm3.expect("closed Draft") < 20.0 * 12.0 * 10.0);

    let (warm, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("warm Draft");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(cold[0].1.vertices, warm[0].1.vertices);
    assert_eq!(cold[0].1.indices, warm[0].1.indices);
}

#[test]
fn rejected_draft_is_atomic_and_zero_angle_is_exact_noop() {
    let mut invalid = box_graph();
    let (baseline, side, neutral) = box_faces(&invalid);
    add_draft(&mut invalid, side.clone(), neutral.clone(), 89.0, false);
    let (bodies, warnings) = invalid
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("invalid Draft remains non-fatal");
    assert!(!warnings.is_empty());
    assert_eq!(bodies[0].1.vertices, baseline.vertices);
    assert_eq!(bodies[0].1.indices, baseline.indices);
    let current = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = invalid
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, current),
        )
        .expect("typed Draft rejection");
    let diagnostic = output
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "draft_1")
        .expect("Draft diagnostic");
    assert_eq!(diagnostic.code.as_str(), DiagnosticCode::PARAMETER_INVALID);
    assert!(diagnostic.parameters.contains_key("angle_deg"));
    assert!(diagnostic.parameters.contains_key("target"));

    let mut zero = box_graph();
    let (baseline, side, neutral) = box_faces(&zero);
    add_draft(&mut zero, side, neutral, 0.0, false);
    let (bodies, warnings) = zero
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("zero Draft");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies[0].1.vertices, baseline.vertices);
    assert_eq!(bodies[0].1.indices, baseline.indices);
}

#[test]
fn missing_named_draft_face_suspends_without_retargeting() {
    let mut graph = box_graph();
    let (baseline, mut side, neutral) = box_faces(&graph);
    side.topology.as_mut().expect("named side").face_id = Some("missing:draft:side".to_string());
    add_draft(&mut graph, side, neutral, 5.0, false);
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("missing Draft face remains non-fatal");
    assert!(!warnings.is_empty());
    assert_eq!(bodies[0].1.vertices, baseline.vertices);
    assert_eq!(bodies[0].1.indices, baseline.indices);
}

#[test]
fn flip_pull_reverses_the_signed_material_change() {
    let mut inward = box_graph();
    let (_, side, neutral) = box_faces(&inward);
    add_draft(&mut inward, side, neutral, 5.0, false);
    let inward_volume = inward
        .inspect_body("box_1")
        .expect("inward Draft")
        .volume_mm3
        .expect("closed body");

    let mut outward = box_graph();
    let (_, side, neutral) = box_faces(&outward);
    add_draft(&mut outward, side, neutral, 5.0, true);
    let outward_volume = outward
        .inspect_body("box_1")
        .expect("outward Draft")
        .volume_mm3
        .expect("closed body");
    assert!(inward_volume < 20.0 * 12.0 * 10.0);
    assert!(outward_volume > 20.0 * 12.0 * 10.0);
}

#[test]
fn datum_plane_can_be_the_neutral_reference() {
    let mut graph = box_graph();
    let (_, side, _) = box_faces(&graph);
    graph.add_feature(FeatureNode {
        id: "neutral_plane".into(),
        name: "Neutral Plane".into(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: 0.0,
                distance_expr: None,
            },
        },
    });
    graph.add_feature(FeatureNode {
        id: "draft_1".into(),
        name: "Draft".into(),
        feature: FeatureType::Draft {
            target: "box_1".into(),
            faces: vec![side],
            neutral: DraftNeutral::Datum("neutral_plane".into()),
            angle_deg: 4.0,
            angle_expr: None,
            flip_pull: false,
        },
    });
    graph.add_dependency("box_1", "draft_1");
    graph.add_dependency("neutral_plane", "draft_1");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("Draft from datum plane");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert!(graph.inspect_body("box_1").unwrap().volume_mm3.unwrap() < 20.0 * 12.0 * 10.0);
}

#[test]
fn cancelled_draft_request_never_publishes_geometry() {
    let mut graph = box_graph();
    let (_, side, neutral) = box_faces(&graph);
    add_draft(&mut graph, side, neutral, 5.0, false);
    let superseded = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    let result = graph.evaluate_request(
        &std::collections::HashSet::new(),
        EvaluationQuality::Final,
        &EvaluationCancellation::new(1, superseded),
    );
    assert!(matches!(result, Err(EvaluationError::Cancelled)));
}

#[test]
fn standalone_draft_covers_scale_aspect_and_far_origin_matrix() {
    let cases = [
        ([0.0, 0.0, 0.0], 0.02, 0.012, 0.01),
        ([0.0, 0.0, 0.0], 1_000.0, 1.0, 10.0),
        ([100_000.0, -100_000.0, 50_000.0], 20.0, 12.0, 10.0),
    ];
    for (origin, width, height, depth) in cases {
        let mut graph = extrude_graph(origin, width, height, depth);
        let (bodies, source_warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("source prism");
        assert!(
            !bodies.is_empty(),
            "source case {origin:?}/{width}/{height}/{depth}: {source_warnings:?}"
        );
        let mesh = &bodies[0].1;
        let side = capture_face(
            mesh.face_refs
                .iter()
                .find(|face| face.normal[0] > 0.9)
                .expect("right side"),
        );
        let neutral = capture_face(
            mesh.face_refs
                .iter()
                .find(|face| face.normal[2] < -0.9)
                .expect("base cap"),
        );
        graph.add_feature(FeatureNode {
            id: "draft_1".into(),
            name: "Draft".into(),
            feature: FeatureType::Draft {
                target: "extrude_1".into(),
                faces: vec![side],
                neutral: DraftNeutral::Face(neutral),
                angle_deg: 3.0,
                angle_expr: None,
                flip_pull: false,
            },
        });
        graph.add_dependency("extrude_1", "draft_1");
        let (drafted, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("Draft matrix case");
        assert!(
            warnings.is_empty(),
            "case {origin:?}/{width}/{height}/{depth}: {warnings:?}"
        );
        assert_eq!(drafted.len(), 1);
        assert!(drafted[0].1.mass_properties().is_some());
    }
}
