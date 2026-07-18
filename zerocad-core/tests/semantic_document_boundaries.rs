use std::fs;
use std::path::{Path, PathBuf};

const LEGACY_DOCUMENT_MARKERS: &[&str] = &[
    "write_zcad(",
    "write_zcad_file(",
    "read_zcad(",
    "read_zcad_file(",
    "read_zcad_with_options(",
];

#[test]
fn phase2_production_code_uses_the_semantic_document_apis() {
    let core_src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let gui_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("zerocad-core must be inside the workspace")
        .join("zerocad-gui/src");
    let mut files = Vec::new();
    collect_rust_files(&core_src, &mut files);
    collect_rust_files(&gui_src, &mut files);
    let mut violations = Vec::new();

    for file in files {
        if file == core_src.join("zcad_format.rs")
            || file
                .strip_prefix(&core_src)
                .is_ok_and(|path| path.starts_with("parametric/tests"))
        {
            continue;
        }
        let text = fs::read_to_string(&file).expect("Rust source must be readable");
        for (line_index, line) in text.lines().enumerate() {
            for marker in LEGACY_DOCUMENT_MARKERS {
                if line.contains(marker) {
                    violations.push(format!("{}:{}: {marker}", file.display(), line_index + 1));
                }
            }
        }
    }

    assert!(
        violations.is_empty(),
        "production paths must use read_document/write_document APIs:\n{}",
        violations.join("\n")
    );
}

#[test]
fn phase3_body_evaluation_has_one_exact_registry_transaction_boundary() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let eval = fs::read_to_string(root.join("src/parametric/eval.rs"))
        .expect("evaluator source must be readable");
    assert_eq!(
        eval.matches("match &node.feature {").count(),
        0,
        "registry dispatch must not regress to one central FeatureType family match"
    );
    assert_eq!(
        eval.matches("match evaluator {").count(),
        1,
        "exact registered evaluator dispatch must remain isolated"
    );
    for stage in [
        "resolve_feature_evaluator(node)",
        "let mut candidate_live = live.clone()",
        "validate_live_body_state(&candidate_live)",
        "live = candidate_live",
        "checkpoints[i] = Some(EvalCheckpoint",
    ] {
        assert!(
            eval.contains(stage),
            "shared evaluator stage missing: {stage}"
        );
    }

    let module = fs::read_to_string(root.join("src/parametric/mod.rs"))
        .expect("parametric module must be readable");
    assert!(module.contains("fn apply_new(live: &mut Vec<LiveBody>, body: LiveBody)"));
    for file in ["eval.rs", "join.rs", "cut.rs"] {
        let source = fs::read_to_string(root.join("src/parametric").join(file))
            .expect("body operation source must be readable");
        assert!(
            !source.contains("live.push(LiveBody {"),
            "{file} bypasses the shared New-body operation"
        );
    }
}

#[test]
fn document_contract_is_authoritative_at_runtime_and_after_round_trip() {
    let mut graph = zerocad_core::ParametricGraph::new();
    graph.add_feature(zerocad_core::FeatureNode {
        id: "box".into(),
        name: "Box".into(),
        feature: zerocad_core::FeatureType::Box {
            w: 10.0,
            h: 8.0,
            d: 6.0,
        },
    });
    graph.add_feature(zerocad_core::FeatureNode {
        id: "move".into(),
        name: "Move".into(),
        feature: zerocad_core::FeatureType::BodyTransform {
            source: "box".into(),
            translation: [2.0, 0.0, 0.0],
            copy: false,
        },
    });
    graph.add_dependency("box", "move");
    graph
        .validate_semantic_contracts()
        .expect("fresh graph satisfies the semantic contract");

    let move_record = graph
        .graph
        .node_weights()
        .find(|feature| feature.id == "move")
        .expect("authoritative feature record");
    assert_eq!(move_record.kind_id.as_str(), move_record.feature.kind_id());
    assert_eq!(
        move_record.payload_version,
        move_record.feature.payload_version()
    );
    assert!(!move_record.inputs.is_empty());
    assert!(move_record
        .inputs
        .iter()
        .any(|input| input.role == "source"));
    assert_eq!(
        move_record.state,
        zerocad_core::document::FeatureState::Active
    );
    assert!(move_record.body.is_some());

    let mut ownership = graph.clone();
    ownership.semantics.body_outputs.insert(
        "opaque-runtime-output".into(),
        zerocad_core::document::FeatureId::from("move"),
    );
    assert_eq!(
        ownership.body_producer_feature_id("opaque-runtime-output"),
        Some("move")
    );
    assert_eq!(
        ownership.body_producer_feature_id("unknown::body:99"),
        None,
        "body ownership must not be inferred from an identifier suffix"
    );

    let mut malformed = graph.clone();
    let move_index = malformed
        .graph
        .node_indices()
        .find(|index| malformed.graph[*index].id == "move")
        .expect("move index");
    malformed.graph[move_index].inputs.clear();
    assert!(malformed
        .validate_semantic_contracts()
        .expect_err("drifted semantic inputs must fail")
        .contains("semantic inputs disagree"));

    let document = zerocad_core::Document::from_graph(graph, zerocad_core::Unit::Millimeter);
    assert!(document
        .evaluator_graph()
        .graph
        .node_weights()
        .any(|feature| feature.id == "box"));
    assert!(document
        .evaluator_graph()
        .graph
        .node_weights()
        .any(|feature| feature.id == "move"));
    let bytes = zerocad_core::write_document_to_vec(
        &document,
        &zerocad_core::SaveOptions::default(),
        &zerocad_core::HydrationBundle::default(),
    )
    .expect("canonical save");
    let loaded =
        zerocad_core::read_document_from_slice(&bytes, &zerocad_core::LoadOptions::default())
            .expect("canonical load")
            .document;
    loaded
        .validate_semantic_contracts()
        .expect("loaded document satisfies the same semantic contract");
    let (bodies, warnings) = loaded
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("loaded document evaluates");
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "move");
}

fn collect_rust_files(root: &Path, files: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(root).expect("source directory must be readable") {
        let path = entry.expect("directory entry must be readable").path();
        if path.is_dir() {
            collect_rust_files(&path, files);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            files.push(path);
        }
    }
}
