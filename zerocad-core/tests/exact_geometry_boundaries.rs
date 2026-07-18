use std::fs;
use std::path::{Path, PathBuf};

#[test]
fn phase3_production_has_no_replay_or_geometry_approximation_escape_hatches() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let source_root = root.join("src");
    let mut files = Vec::new();
    collect_rust_files(&source_root, &mut files);
    let forbidden = [
        "cut_replay",
        "thread_replay",
        "axis_aligned_through_cut",
        "axis_aligned_cut_parts",
        "tessellate_compatibility_for_display",
    ];
    let mut violations = Vec::new();
    for file in files {
        if file
            .strip_prefix(&source_root)
            .is_ok_and(|path| path.starts_with("parametric/tests"))
        {
            continue;
        }
        let text = fs::read_to_string(&file).expect("production source must be readable");
        for marker in forbidden {
            if text.contains(marker) {
                violations.push(format!("{}: {marker}", file.display()));
            }
        }
    }
    assert!(
        violations.is_empty(),
        "Phase 3 production escape hatch reintroduced:\n{}",
        violations.join("\n")
    );
}

#[test]
fn phase3_geometry_safety_net_is_enabled() {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("zerocad-core must be inside the workspace");
    let roots = [workspace.join("zerocad-core"), workspace.join("OpenRCAD")];
    let allowed = Path::new("src/parametric/tests/reattachment_matrix.rs");
    let mut ignored = Vec::new();
    for root in roots {
        let mut files = Vec::new();
        collect_rust_files(&root, &mut files);
        for file in files {
            let text = fs::read_to_string(&file).expect("test source must be readable");
            for (index, line) in text.lines().enumerate() {
                if line.trim_start().starts_with("#[ignore") {
                    let relative = file.strip_prefix(&root).expect("file below scan root");
                    if root.ends_with("zerocad-core") && relative == allowed {
                        continue;
                    }
                    ignored.push(format!("{}:{}", file.display(), index + 1));
                }
            }
        }
    }
    assert!(
        ignored.is_empty(),
        "Phase 3 geometry tests may not be ignored:\n{}",
        ignored.join("\n")
    );
}

#[test]
fn phase3_cross_cutting_and_construction_equivalence_gates_exist() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let gate = fs::read_to_string(root.join("tests/phase3_operation_gate.rs"))
        .expect("Phase 3 operation gate must exist");
    for operation in [
        "STEP import",
        "boolean {op:?}",
        "fillet",
        "chamfer",
        "shell",
        "prism",
        "full revolve",
        "partial revolve",
        "skin/loft",
        "transform",
    ] {
        assert!(
            gate.contains(operation),
            "operation gate missing {operation}"
        );
    }
    for invariant in [
        "health_report_with_policy",
        "is_watertight_with_policy",
        "has_complete_pcurves",
        "validate_strict_with_policy",
        "coverage_for_solid",
        "tessellate_checked_with_policy",
    ] {
        assert!(
            gate.contains(invariant),
            "operation invariant missing {invariant}"
        );
    }

    let equivalence =
        fs::read_to_string(root.join("src/parametric/tests/construction_equivalence.rs"))
            .expect("construction-equivalence regression must exist");
    assert!(equivalence.contains("equivalent_flanged_shaft_histories_both_accept_the_same_thread"));

    let matrix = fs::read_to_string(root.join("tests/phase3_equivalence_matrix.rs"))
        .expect("Phase 3 equivalence matrix must exist");
    for scenario in [
        "primitive_and_sketched_prism_are_equivalent",
        "disjoint_cut_and_fillet_orders_are_equivalent",
        "reordered_suppressed_timeline_survives_save_load_and_resume",
    ] {
        assert!(
            matrix.contains(scenario),
            "equivalence matrix missing {scenario}"
        );
    }

    let strategy = fs::read_to_string(
        root.parent()
            .expect("zerocad-core must be inside the workspace")
            .join("docs/phase3-boolean-strategy.md"),
    )
    .expect("Phase 3 boolean strategy must be documented");
    for contract in [
        "split/imprint boolean engine",
        "Deferred general face-arrangement replacement",
        "replacement trigger",
    ] {
        assert!(
            strategy.contains(contract),
            "boolean strategy missing {contract}"
        );
    }
}

#[test]
fn phase3_kernel_debt_apis_are_removed() {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("zerocad-core must be inside the workspace");
    let algo = fs::read_to_string(workspace.join("OpenRCAD/crates/openrcad-algo/src/boolean.rs"))
        .expect("boolean source must be readable");
    for removed in ["boolean_checked_with_history", "boolean_with_history"] {
        assert!(!algo.contains(removed), "removed API returned: {removed}");
    }

    let rolling_ball =
        fs::read_to_string(workspace.join("OpenRCAD/crates/openrcad-algo/src/rolling_ball.rs"))
            .expect("rolling-ball source must be readable");
    assert!(!rolling_ball.contains("ORC_DEBUG_FILLET"));

    let mesh = fs::read_to_string(workspace.join("OpenRCAD/crates/openrcad-mesh/src/lib.rs"))
        .expect("mesh source must be readable");
    assert!(!mesh.contains("tessellate_compatibility"));
    assert!(!mesh.contains("prepare_compatibility_solid"));

    let history =
        fs::read_to_string(workspace.join("OpenRCAD/crates/openrcad-topo/src/history.rs"))
            .expect("history source must be readable");
    assert!(!history.contains("TopologyHistoryChain"));
}

fn collect_rust_files(root: &Path, files: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(root).expect("source directory must be readable") {
        let path = entry.expect("directory entry must be readable").path();
        if path.is_dir() {
            if path.file_name().is_some_and(|name| name == "target") {
                continue;
            }
            collect_rust_files(&path, files);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            files.push(path);
        }
    }
}
