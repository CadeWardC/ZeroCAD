use super::*;

fn step_text(solid: &KernelSolid, label: &str) -> String {
    static NEXT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let serial = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!(
        "zerocad_phase5_{label}_{}_{}.step",
        std::process::id(),
        serial
    ));
    openrcad::exchange::write_step(solid, path.to_str().unwrap()).expect("write STEP fixture");
    let text = std::fs::read_to_string(&path).expect("read STEP fixture");
    let _ = std::fs::remove_file(path);
    text
}

fn imported_box_graph() -> ParametricGraph {
    let solid = openrcad::primitives::make_box_operation(
        &openrcad::foundation::Pnt::origin(),
        10.0,
        10.0,
        10.0,
    )
    .expect("strict box fixture")
    .value;
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "foreign_box".into(),
        name: "Foreign box".into(),
        feature: FeatureType::Import {
            step_data: step_text(&solid, "box"),
            label: "foreign-box.step".into(),
        },
    });
    graph
}

fn captured_face(face: &crate::mock_kernel::MeshFaceRef) -> FaceRef {
    let topology = face.topology.as_ref().expect("import face topology");
    FaceRef {
        centroid: face.centroid,
        normal: face.normal,
        topology: Some(TopologyFaceRef {
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

fn top_face(graph: &ParametricGraph) -> FaceRef {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("evaluate imported face reference");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let face = bodies[0]
        .1
        .face_refs
        .iter()
        .max_by(|a, b| a.centroid[2].total_cmp(&b.centroid[2]))
        .expect("top imported face");
    captured_face(face)
}

fn assert_strict_output(graph: &ParametricGraph, body_id: &str) {
    let bodies = graph
        .debug_kernel_solids(&std::collections::HashSet::new())
        .expect("evaluate strict body");
    let parts = &bodies
        .iter()
        .find(|(id, _)| id == body_id)
        .unwrap_or_else(|| panic!("missing body {body_id}: {bodies:?}"))
        .1;
    assert!(!parts.is_empty());
    for solid in parts {
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
        assert!(solid.has_complete_pcurves());
        solid
            .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
            .expect("strict Phase 5 result");
        openrcad::mesh::tessellate_checked(solid, 0.05, 0.25)
            .expect("checked Phase 5 tessellation");
    }
}

#[test]
fn imported_planar_face_press_pull_is_strict_and_downstream_ready() {
    let mut graph = imported_box_graph();
    let face = top_face(&graph);
    graph.add_feature(FeatureNode {
        id: "press_pull".into(),
        name: "Press/Pull".into(),
        feature: FeatureType::FaceOffset {
            target: "foreign_box".into(),
            face,
            distance: 5.0,
            distance_expr: None,
        },
    });
    graph.add_dependency("foreign_box", "press_pull");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("press/pull evaluation");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "press_pull");
    let volume = bodies[0].1.mass_properties().expect("closed result").volume;
    assert!((volume - 1_500.0).abs() < 1.0, "volume={volume}");
    assert_strict_output(&graph, "press_pull");

    graph.add_feature(FeatureNode {
        id: "downstream_scale".into(),
        name: "Downstream scale".into(),
        feature: FeatureType::BodyScale {
            source: "press_pull".into(),
            factor: 0.5,
            factor_expr: None,
            center: [0.0, 0.0, 0.0],
        },
    });
    graph.add_dependency("press_pull", "downstream_scale");
    let downstream = graph
        .inspect_body("downstream_scale")
        .expect("downstream Part Design operation");
    assert!((downstream.volume_mm3 - 187.5).abs() < 1.0);
    assert_strict_output(&graph, "downstream_scale");
}

#[test]
fn unsupported_external_nist_edit_fails_atomically() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "nist_bracket".into(),
        name: "NIST bracket".into(),
        feature: FeatureType::Import {
            step_data: include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/fixtures/step/nist-bracket1-part.stp"
            ))
            .into(),
            label: "nist-bracket1-part.stp".into(),
        },
    });
    let (bodies, _) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("external STEP import");
    let face = bodies[0]
        .1
        .face_refs
        .iter()
        .max_by(|a, b| a.centroid[2].total_cmp(&b.centroid[2]))
        .expect("external face");
    graph.add_feature(FeatureNode {
        id: "nist_press_pull".into(),
        name: "NIST Press/Pull".into(),
        feature: FeatureType::FaceOffset {
            target: "nist_bracket".into(),
            face: captured_face(face),
            distance: 0.5,
            distance_expr: None,
        },
    });
    graph.add_dependency("nist_bracket", "nist_press_pull");
    let (edited, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("external STEP direct edit");
    assert!(edited.iter().any(|(id, _)| id == "nist_bracket"));
    assert!(!edited.iter().any(|(id, _)| id == "nist_press_pull"));
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("Press/Pull")));
}

#[test]
fn imported_planar_face_move_and_thicken_use_exact_prisms() {
    let mut moved = imported_box_graph();
    let face = top_face(&moved);
    moved.add_feature(FeatureNode {
        id: "move_face".into(),
        name: "Move Face".into(),
        feature: FeatureType::FaceMove {
            target: "foreign_box".into(),
            face,
            translation: [0.0, 0.0, 2.0],
        },
    });
    moved.add_dependency("foreign_box", "move_face");
    let inspection = moved.inspect_body("move_face").expect("moved face body");
    assert!((inspection.volume_mm3 - 1_200.0).abs() < 1.0);
    assert_strict_output(&moved, "move_face");

    let mut thickened = imported_box_graph();
    let face = top_face(&thickened);
    thickened.add_feature(FeatureNode {
        id: "thicken_face".into(),
        name: "Thicken Face".into(),
        feature: FeatureType::FaceThicken {
            target: "foreign_box".into(),
            face,
            thickness: 2.0,
            thickness_expr: None,
            reverse: false,
        },
    });
    thickened.add_dependency("foreign_box", "thicken_face");
    let inspection = thickened
        .inspect_body("thicken_face")
        .expect("thickened face body");
    assert!((inspection.volume_mm3 - 200.0).abs() < 1.0);
    assert_strict_output(&thickened, "thicken_face");
}

#[test]
fn tangential_move_fails_atomically_with_a_diagnostic() {
    let mut graph = imported_box_graph();
    let face = top_face(&graph);
    graph.add_feature(FeatureNode {
        id: "bad_move".into(),
        name: "Bad Move".into(),
        feature: FeatureType::FaceMove {
            target: "foreign_box".into(),
            face,
            translation: [1.0, 0.0, 2.0],
        },
    });
    graph.add_dependency("foreign_box", "bad_move");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("atomic failure");
    assert!(bodies.iter().any(|(id, _)| id == "foreign_box"));
    assert!(!bodies.iter().any(|(id, _)| id == "bad_move"));
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("tangential")));
}

#[test]
fn delete_internal_cylindrical_face_heals_an_imported_hole() {
    use openrcad::foundation::{Ax2, Dir, Pnt};
    let box_solid = openrcad::primitives::make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0)
        .expect("box")
        .value;
    let hole = openrcad::primitives::make_cylinder_operation(
        &Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()),
        2.0,
        12.0,
    )
    .expect("hole tool")
    .value;
    let bored =
        openrcad::algo::boolean_operation(&box_solid, &hole, openrcad::algo::BooleanOp::Cut)
            .expect("bored fixture")
            .value;
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "foreign_bored".into(),
        name: "Foreign bored block".into(),
        feature: FeatureType::Import {
            step_data: step_text(&bored, "bored"),
            label: "foreign-bored.step".into(),
        },
    });
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("import bored part");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let center = [5.0_f32, 5.0, 5.0];
    let wall = bodies[0]
        .1
        .face_refs
        .iter()
        .min_by(|a, b| {
            let distance = |face: &crate::mock_kernel::MeshFaceRef| {
                (0..3)
                    .map(|axis| (face.centroid[axis] - center[axis]).powi(2))
                    .sum::<f32>()
            };
            distance(a).total_cmp(&distance(b))
        })
        .expect("cylindrical hole wall");
    let face = captured_face(wall);
    graph.add_feature(FeatureNode {
        id: "delete_hole".into(),
        name: "Delete Hole Face".into(),
        feature: FeatureType::FaceDelete {
            target: "foreign_bored".into(),
            face,
        },
    });
    graph.add_dependency("foreign_bored", "delete_hole");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("delete hole face");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let healed = bodies
        .iter()
        .find(|(id, _)| id == "delete_hole")
        .expect("healed output");
    let volume = healed
        .1
        .mass_properties()
        .expect("healed closed mesh")
        .volume;
    assert!((volume - 1_000.0).abs() < 1.0, "volume={volume}");
    assert_strict_output(&graph, "delete_hole");
}

fn imported_cylinder_graph() -> ParametricGraph {
    use openrcad::foundation::{Ax2, Dir, Pnt};
    let cylinder = openrcad::primitives::make_cylinder_operation(
        &Ax2::new(Pnt::origin(), Dir::dz()),
        3.0,
        10.0,
    )
    .expect("cylinder fixture")
    .value;
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "foreign_cylinder".into(),
        name: "Foreign cylinder".into(),
        feature: FeatureType::Import {
            step_data: step_text(&cylinder, "cylinder"),
            label: "foreign-cylinder.step".into(),
        },
    });
    graph
}

fn cylindrical_wall(graph: &ParametricGraph) -> FaceRef {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("import cylinder");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let wall = bodies[0]
        .1
        .face_refs
        .iter()
        .min_by(|a, b| {
            (a.centroid[2] - 5.0)
                .abs()
                .total_cmp(&(b.centroid[2] - 5.0).abs())
        })
        .expect("cylinder wall");
    captured_face(wall)
}

#[test]
fn analytic_cylinder_offset_and_thicken_preserve_strict_geometry() {
    let mut offset = imported_cylinder_graph();
    let face = cylindrical_wall(&offset);
    offset.add_feature(FeatureNode {
        id: "offset_cylinder".into(),
        name: "Offset Cylinder".into(),
        feature: FeatureType::FaceOffset {
            target: "foreign_cylinder".into(),
            face,
            distance: 1.0,
            distance_expr: None,
        },
    });
    offset.add_dependency("foreign_cylinder", "offset_cylinder");
    let (_, warnings) = offset
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("evaluate cylinder offset");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let measured = offset
        .inspect_body("offset_cylinder")
        .expect("offset cylinder");
    assert!((measured.volume_mm3 - std::f64::consts::PI * 160.0).abs() < 1.0);
    assert_strict_output(&offset, "offset_cylinder");

    let mut thickened = imported_cylinder_graph();
    let face = cylindrical_wall(&thickened);
    thickened.add_feature(FeatureNode {
        id: "thicken_cylinder".into(),
        name: "Thicken Cylinder".into(),
        feature: FeatureType::FaceThicken {
            target: "foreign_cylinder".into(),
            face,
            thickness: 1.0,
            thickness_expr: None,
            reverse: false,
        },
    });
    thickened.add_dependency("foreign_cylinder", "thicken_cylinder");
    let measured = thickened
        .inspect_body("thicken_cylinder")
        .expect("thickened cylinder");
    assert!((measured.volume_mm3 - std::f64::consts::PI * 70.0).abs() < 1.0);
    assert_strict_output(&thickened, "thicken_cylinder");
}

#[test]
fn cylindrical_reference_face_splits_a_body_into_inside_and_outside() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "split_target".into(),
        name: "Split target".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "splitter_cylinder".into(),
        name: "Splitter cylinder".into(),
        feature: FeatureType::Cylinder { r: 3.0, h: 10.0 },
    });
    graph.add_feature(FeatureNode {
        id: "position_splitter".into(),
        name: "Position splitter".into(),
        feature: FeatureType::BodyTransform {
            source: "splitter_cylinder".into(),
            translation: [5.0, 0.0, 5.0],
            copy: false,
        },
    });
    graph.add_dependency("splitter_cylinder", "position_splitter");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("reference cylinder");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let reference_mesh = &bodies
        .iter()
        .find(|(id, _)| id == "position_splitter")
        .expect("positioned splitter")
        .1;
    let wall = reference_mesh
        .face_refs
        .iter()
        .min_by(|a, b| {
            let distance = |face: &crate::mock_kernel::MeshFaceRef| {
                (face.centroid[0] - 5.0).powi(2)
                    + (face.centroid[1] - 5.0).powi(2)
                    + (face.centroid[2] - 5.0).powi(2)
            };
            distance(a).total_cmp(&distance(b))
        })
        .expect("cylindrical reference wall");
    graph.add_feature(FeatureNode {
        id: "curved_split".into(),
        name: "Curved split".into(),
        feature: FeatureType::BodySplit {
            target: "split_target".into(),
            plane: PlaneBase::XY,
            face: Some(captured_face(wall)),
        },
    });
    graph.add_dependency("split_target", "curved_split");
    graph.add_dependency("position_splitter", "curved_split");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("cylindrical split");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let inside = bodies
        .iter()
        .find(|(id, _)| id == "curved_split")
        .expect("inside output")
        .1
        .mass_properties()
        .expect("inside mass")
        .volume;
    let outside_id = body_output_id("curved_split", 1);
    let outside = bodies
        .iter()
        .find(|(id, _)| id == &outside_id)
        .expect("outside output")
        .1
        .mass_properties()
        .expect("outside mass")
        .volume;
    assert!((inside + outside - 1_000.0).abs() < 1.0);
    assert!((inside - std::f64::consts::PI * 90.0).abs() < 1.0);
    assert_strict_output(&graph, "curved_split");
    assert_strict_output(&graph, &outside_id);
}

fn tetrahedron_stl() -> Vec<u8> {
    br#"solid tetrahedron
facet normal 0 0 -1
 outer loop
  vertex 0 0 0
  vertex 0 1 0
  vertex 1 0 0
 endloop
endfacet
facet normal 0 -1 0
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 0 1
 endloop
endfacet
facet normal -1 0 0
 outer loop
  vertex 0 0 0
  vertex 0 0 1
  vertex 0 1 0
 endloop
endfacet
facet normal 1 1 1
 outer loop
  vertex 1 0 0
  vertex 0 1 0
  vertex 0 0 1
 endloop
endfacet
endsolid tetrahedron
"#
    .to_vec()
}

fn stl_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "mesh".into(),
        name: "Mesh".into(),
        feature: FeatureType::ImportStl {
            stl_data: tetrahedron_stl(),
            label: "tetrahedron.stl".into(),
        },
    });
    graph
}

#[test]
fn stl_mesh_body_supports_transform_scale_measure_and_export() {
    let report = crate::stl::read_stl_mesh(&tetrahedron_stl())
        .expect("valid STL")
        .validation;
    assert!(report.is_closed_manifold(), "report={report:?}");

    let mut graph = stl_graph();
    graph.add_feature(FeatureNode {
        id: "mesh_move".into(),
        name: "Move mesh".into(),
        feature: FeatureType::BodyTransform {
            source: "mesh".into(),
            translation: [2.0, 3.0, 4.0],
            copy: false,
        },
    });
    graph.add_dependency("mesh", "mesh_move");
    graph.add_feature(FeatureNode {
        id: "mesh_scale".into(),
        name: "Scale mesh".into(),
        feature: FeatureType::BodyScale {
            source: "mesh_move".into(),
            factor: 2.0,
            factor_expr: None,
            center: [2.0, 3.0, 4.0],
        },
    });
    graph.add_dependency("mesh_move", "mesh_scale");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("mesh evaluation");
    assert!(warnings.is_empty(), "warnings={warnings:?}");
    let mesh = &bodies
        .iter()
        .find(|(id, _)| id == "mesh_scale")
        .expect("scaled mesh")
        .1;
    assert!(!mesh.indices.is_empty());
    let inspection = graph.inspect_body("mesh_scale").expect("mesh measurement");
    assert!((inspection.volume_mm3 - 8.0 / 6.0).abs() < 1.0e-4);
    let section =
        crate::mock_kernel::clip_mesh_by_plane(mesh, [2.5, 3.5, 4.5], [1.0, 1.0, 1.0], true)
            .expect("mesh section");
    assert!(!section.mesh.indices.is_empty());
    assert!(!section.contours.is_empty());
    assert!(!crate::meshes_to_binary_stl(std::iter::once(mesh)).is_empty());
}

#[test]
fn mesh_body_is_rejected_by_brep_boolean_without_consuming_inputs() {
    let mut graph = stl_graph();
    graph.add_feature(FeatureNode {
        id: "box".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 2.0,
            h: 2.0,
            d: 2.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "bad_cut".into(),
        name: "Bad cut".into(),
        feature: FeatureType::BodyCut {
            target: "box".into(),
            tool: "mesh".into(),
            keep_tool: false,
        },
    });
    graph.add_dependency("mesh", "bad_cut");
    graph.add_dependency("box", "bad_cut");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("boolean rejection");
    assert!(bodies.iter().any(|(id, _)| id == "mesh"));
    assert!(bodies.iter().any(|(id, _)| id == "box"));
    assert!(!bodies.iter().any(|(id, _)| id == "bad_cut"));
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("solid geometry")));
}

#[test]
fn direct_edit_semantics_support_reorder_suppression_and_explicit_provenance() {
    let mut graph = imported_box_graph();
    let face = top_face(&graph);
    graph.add_feature(FeatureNode {
        id: "semantic_offset".into(),
        name: "Semantic offset".into(),
        feature: FeatureType::FaceOffset {
            target: "foreign_box".into(),
            face,
            distance: 1.0,
            distance_expr: None,
        },
    });
    graph.add_dependency("foreign_box", "semantic_offset");

    graph
        .validate_semantic_contracts()
        .expect("direct-edit semantic contract");
    assert_eq!(
        graph.body_producer_feature_id("semantic_offset"),
        Some("semantic_offset")
    );
    assert!(graph.set_feature_sequence("semantic_offset", crate::document::SequenceKey(1)));
    assert!(graph.set_feature_sequence("foreign_box", crate::document::SequenceKey(2)));
    assert_eq!(
        graph.feature_sequence("semantic_offset"),
        Some(crate::document::SequenceKey(1))
    );
    assert!(graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("dependency-ordered evaluation")
        .iter()
        .any(|(id, _)| id == "semantic_offset"));

    assert!(graph.set_feature_suppressed("semantic_offset", true));
    let suppressed = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("suppressed direct edit");
    assert!(suppressed.iter().any(|(id, _)| id == "foreign_box"));
    assert!(!suppressed.iter().any(|(id, _)| id == "semantic_offset"));

    assert!(graph.set_feature_suppressed("semantic_offset", false));
    graph
        .validate_semantic_contracts()
        .expect("restored direct-edit semantic contract");
    assert_strict_output(&graph, "semantic_offset");
}
