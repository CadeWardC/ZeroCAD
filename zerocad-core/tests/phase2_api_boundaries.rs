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
        "Phase 2 production paths must use read_document/write_document APIs; legacy wrappers are reserved for frozen compatibility tests and benchmarks:\n{}",
        violations.join("\n")
    );
}

#[test]
fn phase2_body_evaluation_has_one_registry_dispatch_boundary() {
    let eval =
        fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/parametric/eval.rs"))
            .expect("evaluator source must be readable");
    assert_eq!(
        eval.matches("match &node.feature {").count(),
        1,
        "feature-specific dispatch must stay isolated in invoke_registered_feature"
    );
    let invoke = eval
        .find("fn invoke_registered_feature(")
        .expect("registry-driven invocation boundary");
    let feature_match = eval.find("match &node.feature {").unwrap();
    assert!(feature_match > invoke);
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
