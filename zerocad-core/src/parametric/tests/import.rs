//! STEP import as a parametric feature: an embedded STEP body evaluates to a
//! live body with a renderable mesh and durable `import:{node}:face:{k}` names.

use super::*;

/// A 2×3×4 box as STEP text, produced by the kernel's own writer.
fn box_step_data() -> String {
    let solid = openrcad::primitives::make_box(&openrcad::foundation::Pnt::origin(), 2.0, 3.0, 4.0);
    // Unique per call: tests run in parallel and would otherwise write/delete
    // one shared file from under each other.
    static NEXT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!(
        "zerocad_test_import_box_{}_{n}.stp",
        std::process::id()
    ));
    let path_str = path.to_str().unwrap().to_string();
    openrcad::exchange::write_step(&solid, &path_str).expect("write_step");
    let data = std::fs::read_to_string(&path).expect("read step text");
    let _ = std::fs::remove_file(&path);
    data
}

fn import_graph(step_data: String) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "import_1".to_string(),
        name: "Imported Box".to_string(),
        feature: FeatureType::Import {
            step_data,
            label: "box".to_string(),
        },
    });
    g
}

#[test]
fn import_step_body_evaluates_to_mesh() {
    let g = import_graph(box_step_data());
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let (id, mesh) = &bodies[0];
    assert_eq!(id, "import_1");
    assert!(!mesh.indices.is_empty(), "imported body tessellated empty");

    // Bounding box of the mesh matches the 2×3×4 source solid.
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for v in mesh.vertices.chunks(6) {
        for a in 0..3 {
            lo[a] = lo[a].min(v[a]);
            hi[a] = hi[a].max(v[a]);
        }
    }
    let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let mut sorted = dims;
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!((sorted[0] - 2.0).abs() < 1e-3, "dims {dims:?}");
    assert!((sorted[1] - 3.0).abs() < 1e-3, "dims {dims:?}");
    assert!((sorted[2] - 4.0).abs() < 1e-3, "dims {dims:?}");
}

#[test]
fn import_step_faces_are_stamped_with_durable_names() {
    let g = import_graph(box_step_data());
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &bodies[0].1;
    assert!(!mesh.face_refs.is_empty());
    for face_ref in &mesh.face_refs {
        let topo = face_ref.topology.as_ref().expect("face stamped");
        assert_eq!(topo.body_id.as_deref(), Some("import_1"));
        let id = topo.face_id.as_deref().expect("face id");
        assert!(
            id.starts_with("import:import_1:face:"),
            "unexpected face id {id}"
        );
    }
}

#[test]
fn import_step_bad_data_warns_instead_of_failing() {
    let g = import_graph("garbage, not a STEP file".to_string());
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(bodies.is_empty());
    assert_eq!(warnings.len(), 1);
    assert!(warnings[0].contains("import_1"), "warning: {}", warnings[0]);
}

#[test]
fn import_graph_serde_round_trips() {
    let g = import_graph(box_step_data());
    let json = serde_json::to_string(&g).unwrap();
    let mut g2: ParametricGraph = serde_json::from_str(&json).unwrap();
    g2.rebuild_node_map();
    let (bodies, warnings) = g2
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert!(!bodies[0].1.indices.is_empty());
}
