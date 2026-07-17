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
fn phase3_closes_the_document_deviation_ledger_and_keeps_guards() {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("zerocad-core must be inside the workspace");
    let ledger = fs::read_to_string(workspace.join("docs/phase2-document-architecture.md"))
        .expect("document architecture ledger must be readable");
    for debt in ["P2-D1", "P2-D2", "P2-D3", "P2-D4"] {
        assert_eq!(
            ledger.matches(debt).count(),
            1,
            "closed Phase 2 deviation must have exactly one ledger entry: {debt}"
        );
    }
    assert_eq!(ledger.matches("**Phase 3 closure:**").count(), 4);
    assert!(
        !ledger.contains("**Phase 3 removal gate:**"),
        "closed Phase 2 debt must not retain an open Phase 3 gate"
    );

    let document =
        fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/document.rs"))
            .expect("document model source must be readable");
    for contract in [
        "pub struct Document {",
        "runtime: ParametricGraph",
        "pub fn evaluator_graph(&self)",
        "pub fn evaluator_graph_mut(&mut self)",
    ] {
        assert!(
            document.contains(contract),
            "Document ownership contract missing: {contract}"
        );
    }

    let types =
        fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/parametric/types.rs"))
            .expect("feature record source must be readable");
    for field in [
        "pub struct FeatureRecord",
        "pub kind_id:",
        "pub payload_version:",
        "pub sequence:",
        "pub inputs:",
        "pub state:",
        "pub body:",
    ] {
        assert!(
            types.contains(field),
            "authoritative feature field missing: {field}"
        );
    }

    let eval =
        fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/parametric/eval.rs"))
            .expect("evaluator source must be readable");
    for guard in [
        "feature_inputs_for_runtime",
        "runtime body ownership disagrees",
        "must appear exactly once in body",
        "semantics.body_outputs",
        "semantics.body_outputs",
    ] {
        assert!(
            eval.contains(guard),
            "semantic integrity guard missing: {guard}"
        );
    }
    assert!(
        !eval.contains("body_output_owner_id")
            && !eval.contains("ensure_body_output_reference")
            && !eval.contains("split(\"::body:\")"),
        "Phase 6 forbids inferring body ownership from identifier spelling"
    );

    let format =
        fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/zcad_format.rs"))
            .expect("document decoder source must be readable");
    let canonical_tail = format
        .split("graph.semantics.bodies = body_records;")
        .nth(1)
        .expect("canonical body-record installation boundary");
    let validation = canonical_tail
        .find("validate_semantic_contracts")
        .expect("canonical semantic validation");
    let recovery = canonical_tail
        .find("rebuild_node_map")
        .expect("legacy node-map recovery boundary");
    assert!(
        validation < recovery,
        "canonical documents must be validated before legacy recovery"
    );

    for gui_file in [
        "src/main.rs",
        "src/document_worker.rs",
        "src/evaluation_worker.rs",
    ] {
        let source = fs::read_to_string(workspace.join("zerocad-gui").join(gui_file))
            .expect("GUI document owner source must be readable");
        assert!(
            source.contains("document: Document"),
            "{gui_file} must be rooted in Document"
        );
    }
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
