//! Revolve as a parametric feature: sketch → Revolve node → live body, with
//! datum-axis support, partial angles, and cut mode.

use super::*;
use crate::parametric::{AxisBase, DatumAxisDef};

fn add_revolve(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    axis: AxisBase,
    angle_deg: f32,
    mode: ExtrudeMode,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Revolve {
            axis,
            angle_deg,
            angle_expr: None,
            region_indices: vec![],
            mode,
            target: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

fn volume(mesh: &MockMesh) -> f64 {
    mesh.mass_properties().expect("closed mesh").volume
}

#[test]
fn full_revolve_makes_cylinder_body() {
    // Rect x∈[0,2], y∈[0,5] on the XY plane, revolved about the sketch-plane
    // Y axis → cylinder r=2 h=5 along Y.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (2.0, 5.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    let exact = std::f64::consts::PI * 4.0 * 5.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "cylinder volume {v} vs {exact}"
    );
}

#[test]
fn partial_revolve_about_datum_axis() {
    // Quarter-turn washer about a datum axis at x=0 (the sketch Y axis).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "datumaxis_1".to_string(),
        name: "axis".to_string(),
        feature: FeatureType::DatumAxis {
            def: DatumAxisDef::TwoPoints {
                a: [0.0, 0.0, 0.0],
                b: [0.0, 1.0, 0.0],
            },
        },
    });
    add_sketch(&mut g, "sketch_2", rect_sketch((1.0, 0.0), (2.0, 3.0)));
    add_revolve(
        &mut g,
        "revolve_3",
        "sketch_2",
        AxisBase::Datum("datumaxis_1".to_string()),
        90.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    let exact = std::f64::consts::PI * (4.0 - 1.0) * 3.0 / 4.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "quarter washer volume {v} vs {exact}"
    );
}

#[test]
fn crossing_profile_warns_and_builds_nothing() {
    // Rect straddling the axis: fails loud, no body.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((-1.0, 0.0), (1.0, 2.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "revolve_2"
                && diagnostic.code.as_str() == DiagnosticCode::OPERATION_FAILED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

#[test]
fn revolve_contract_cold_warm_cancel_rollback_topology_and_lifecycle() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_0".into(),
        name: "Existing body".into(),
        feature: FeatureType::Box {
            w: 3.0,
            h: 3.0,
            d: 3.0,
        },
    });
    add_sketch(&mut g, "sketch_1", rect_sketch((1.0, 0.0), (2.0, 3.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let hidden = std::collections::HashSet::new();
    let cold = g.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty());
    let warm = g.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert_eq!(cold.0.len(), warm.0.len());
    for ((cold_id, cold_mesh), (warm_id, warm_mesh)) in cold.0.iter().zip(&warm.0) {
        assert_eq!(cold_id, warm_id);
        assert_eq!(cold_mesh.indices, warm_mesh.indices);
        assert_eq!(cold_mesh.face_ids, warm_mesh.face_ids);
    }
    let revolve_mesh = warm
        .0
        .iter()
        .find(|(id, _)| id == "revolve_2")
        .map(|(_, mesh)| mesh)
        .expect("revolve body");
    assert!(revolve_mesh.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| super::super::topo_name::TopoName::parse(name).is_durable())
    }));

    let cancelled_generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        g.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled_generation),
        ),
        Err(EvaluationError::Cancelled)
    ));

    let snapshot = serde_json::to_string(&g.clone_document()).unwrap();
    let mut restored: ParametricGraph = serde_json::from_str(&snapshot).unwrap();
    assert!(restored.remove_feature("revolve_2"));
    add_revolve(
        &mut restored,
        "revolve_3",
        "sketch_1",
        AxisBase::Y,
        180.0,
        ExtrudeMode::NewBody,
    );
    let rebuilt = restored.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(rebuilt.1.is_empty());
    assert!(rebuilt.0.iter().any(|(id, _)| id == "revolve_3"));

    // A rejected revolve candidate cannot consume or corrupt an existing body.
    let mut invalid = ParametricGraph::new();
    invalid.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Stable body".into(),
        feature: FeatureType::Box {
            w: 4.0,
            h: 4.0,
            d: 4.0,
        },
    });
    add_sketch(
        &mut invalid,
        "sketch_2",
        rect_sketch((-1.0, 0.0), (1.0, 2.0)),
    );
    add_revolve(
        &mut invalid,
        "revolve_3",
        "sketch_2",
        AxisBase::Y,
        360.0,
        ExtrudeMode::Cut,
    );
    invalid.add_dependency("box_1", "revolve_3");
    let (bodies, warnings) = invalid.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(!warnings.is_empty());
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "box_1");
    assert!((volume(&bodies[0].1) - 64.0).abs() < 1.0e-3);
}

#[test]
fn revolve_cut_bores_groove_into_box() {
    // 10×10×10 box, then cut a full-turn revolved ring (rect r∈[8,11] of the
    // box's local frame? keep simple: revolve about the Y axis at the box
    // corner, cutting a quarter-cylinder groove out of the box edge).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    // Sketch on XY: rect x∈[0,2], y∈[0,10] revolved 360° about the sketch Y
    // axis = cylinder r=2 through the box's z column at the origin corner.
    add_sketch(&mut g, "sketch_2", rect_sketch((0.0, 0.0), (2.0, 10.0)));
    add_revolve(
        &mut g,
        "revolve_3",
        "sketch_2",
        AxisBase::Y,
        360.0,
        ExtrudeMode::Cut,
    );
    g.add_dependency("box_1", "revolve_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    // Box minus the quarter of the cylinder that overlaps it (cylinder axis
    // on the box corner along Y): 1000 − (π·4·10)/4.
    let exact = 1000.0 - std::f64::consts::PI * 4.0 * 10.0 / 4.0;
    assert!(
        (v - exact).abs() / exact < 0.01,
        "cut volume {v} vs {exact}"
    );
}

/// End-to-end check of the display claim behind the planar surface-group fix:
/// a full revolve's flat cap — built by the kernel as three 120° pie wedges —
/// must reach the viewport as ONE selectable face with NO radial seam lines.
/// This runs the exact GUI pipeline (sketch → Revolve node → evaluate →
/// MockMesh), not the grouping function in isolation.
#[test]
fn full_revolve_cap_reads_as_one_face_without_radial_seams() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (2.0, 5.0)));
    add_revolve(
        &mut g,
        "revolve_2",
        "sketch_1",
        AxisBase::Y,
        360.0,
        ExtrudeMode::NewBody,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;

    // Selection: each cap disc (y=0 and y=5) carries exactly one face id.
    let vy = |vi: u32| mesh.vertices[vi as usize * 6 + 1];
    for cap_y in [0.0f32, 5.0] {
        let ids: std::collections::HashSet<u32> = mesh
            .indices
            .chunks_exact(3)
            .enumerate()
            .filter(|(_, t)| t.iter().all(|&v| (vy(v) - cap_y).abs() < 1e-4))
            .map(|(t, _)| mesh.face_ids[t])
            .collect();
        assert_eq!(
            ids.len(),
            1,
            "cap y={cap_y} must select as one face, got ids {ids:?}"
        );
    }

    // Wireframe: every drawn segment lying in a cap plane sits on the rim
    // (r≈2) — a segment endpoint well inside the disc would be a leaked
    // radial construction seam (the old three-wedge pie lines).
    let ep = |i: u32| {
        let b = i as usize * 3;
        (
            mesh.edge_vertices[b],
            mesh.edge_vertices[b + 1],
            mesh.edge_vertices[b + 2],
        )
    };
    let mut rim_segments = 0;
    for seg in mesh.edge_indices.chunks_exact(2) {
        let (a, b) = (ep(seg[0]), ep(seg[1]));
        for cap_y in [0.0f32, 5.0] {
            if (a.1 - cap_y).abs() < 1e-4 && (b.1 - cap_y).abs() < 1e-4 {
                for p in [a, b] {
                    let r = (p.0 * p.0 + p.2 * p.2).sqrt();
                    assert!(
                        r > 1.9,
                        "cap wireframe segment off the rim (r={r}): radial seam leaked"
                    );
                }
                rim_segments += 1;
            }
        }
    }
    assert!(
        rim_segments > 8,
        "expected dense rim polylines on the caps, got {rim_segments} segments"
    );

    // The rim segments of one cap chain into a single selectable edge group.
    if !mesh.edge_groups.is_empty() {
        let top_groups: std::collections::HashSet<u32> = mesh
            .edge_indices
            .chunks_exact(2)
            .enumerate()
            .filter(|(_, seg)| {
                let (a, b) = (ep(seg[0]), ep(seg[1]));
                (a.1 - 5.0).abs() < 1e-4 && (b.1 - 5.0).abs() < 1e-4
            })
            .map(|(s, _)| mesh.edge_groups[s])
            .collect();
        assert_eq!(
            top_groups.len(),
            1,
            "top rim must be one closed selectable edge, got groups {top_groups:?}"
        );
    }
}
