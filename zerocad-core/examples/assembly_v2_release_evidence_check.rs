//! Validate measured Assembly V2 solver calibration and stress evidence.

use std::path::PathBuf;
use zerocad_core::{validate_assembly_v2_release_evidence_for_commit, AssemblyV2ReleaseEvidence};

fn main() {
    let path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target/assembly-v2-release-evidence.json"));
    let bytes = std::fs::read(&path).unwrap_or_else(|error| {
        eprintln!(
            "cannot read Assembly V2 release evidence at {}: {error}",
            path.display()
        );
        std::process::exit(2);
    });
    let evidence: AssemblyV2ReleaseEvidence =
        serde_json::from_slice(&bytes).unwrap_or_else(|error| {
            eprintln!(
                "invalid Assembly V2 evidence at {}: {error}",
                path.display()
            );
            std::process::exit(2);
        });
    let current_commit = current_git_commit();
    let failures = validate_assembly_v2_release_evidence_for_commit(&evidence, &current_commit);
    if !failures.is_empty() {
        for failure in failures {
            eprintln!("- {failure}");
        }
        std::process::exit(1);
    }
    println!(
        "Assembly V2 evidence passed: {} calibration cases and {} stress corpora.",
        evidence.calibration_cases.len(),
        evidence.stress_corpora.len()
    );
}

fn current_git_commit() -> String {
    let output = std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .unwrap_or_else(|error| {
            eprintln!("cannot determine the current git commit: {error}");
            std::process::exit(2);
        });
    if !output.status.success() {
        eprintln!("cannot determine the current git commit");
        std::process::exit(2);
    }
    String::from_utf8(output.stdout)
        .unwrap_or_else(|error| {
            eprintln!("git returned a non-UTF-8 commit id: {error}");
            std::process::exit(2);
        })
        .trim()
        .to_owned()
}
