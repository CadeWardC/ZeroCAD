use std::fs;
use std::path::{Path, PathBuf};

const LEGACY_MARKERS: &[&str] = &[
    "openrcad::primitives::make_box(",
    "openrcad::primitives::make_cylinder(",
    "openrcad::primitives::make_cone(",
    "openrcad::primitives::make_sphere(",
    "openrcad::primitives::make_wedge(",
    "openrcad::algo::boolean_checked",
    "openrcad::algo::prism(",
    "openrcad::algo::revolve(",
    "openrcad::exchange::read_step_str(",
    "openrcad::mesh::tessellate(",
    "openrcad::mesh::tessellate_for_display_with_cancel(",
    "openrcad::mesh::tessellate_compatibility",
];

const PHASE3_ALLOWLIST: &[(&str, &str)] = &[];

#[test]
fn production_code_does_not_add_legacy_kernel_calls() {
    let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    collect_rust_files(&src, &mut files);
    let mut violations = Vec::new();

    for file in files {
        let relative = file
            .strip_prefix(&src)
            .expect("source path must be below src")
            .to_string_lossy()
            .replace('\\', "/");
        if relative.starts_with("parametric/tests/") {
            continue;
        }
        let text = fs::read_to_string(&file).expect("Rust source must be readable");
        for (line_index, line) in text.lines().enumerate() {
            for marker in LEGACY_MARKERS {
                if line.contains(marker)
                    && !PHASE3_ALLOWLIST
                        .iter()
                        .any(|(path, allowed)| relative == *path && line.contains(allowed))
                {
                    violations.push(format!("{relative}:{}: {marker}", line_index + 1));
                }
            }
        }
    }

    assert!(
        violations.is_empty(),
        "legacy OpenRCAD production calls must use a canonical operation result or be documented in docs/phase1-migration.md:\n{}",
        violations.join("\n")
    );
}

#[test]
fn evaluator_reachable_code_uses_fallible_dir2d_construction() {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("core crate must live in the workspace");
    let roots = [
        workspace.join("zerocad-core/src"),
        workspace.join("OpenRCAD/crates"),
    ];
    let mut violations = Vec::new();
    for root in roots {
        let mut files = Vec::new();
        collect_rust_files(&root, &mut files);
        for file in files {
            let normalized = file.to_string_lossy().replace('\\', "/");
            if normalized.ends_with("openrcad-foundation/src/dir.rs") {
                continue;
            }
            let text = fs::read_to_string(&file).expect("Rust source must be readable");
            for (line_index, line) in text.lines().enumerate() {
                if line.contains("Dir2d::new(") {
                    violations.push(format!("{normalized}:{}", line_index + 1));
                }
            }
        }
    }
    assert!(
        violations.is_empty(),
        "computed 2D directions must use Dir2d::try_new and propagate degeneracy:\n{}",
        violations.join("\n")
    );
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
