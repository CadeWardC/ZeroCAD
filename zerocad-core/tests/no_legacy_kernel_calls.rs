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
