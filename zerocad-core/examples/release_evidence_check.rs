//! Validate a collected Phase 7 release-evidence JSON document.

use std::path::PathBuf;
use zerocad_core::{validate_phase7_release_evidence, Phase7ReleaseEvidence};

fn main() {
    let path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target/phase7-release-evidence.json"));
    let bytes = std::fs::read(&path).unwrap_or_else(|error| {
        eprintln!(
            "cannot read Phase 7 evidence at {}: {error}",
            path.display()
        );
        std::process::exit(2);
    });
    let evidence: Phase7ReleaseEvidence = serde_json::from_slice(&bytes).unwrap_or_else(|error| {
        eprintln!("invalid Phase 7 evidence at {}: {error}", path.display());
        std::process::exit(2);
    });
    let failures = validate_phase7_release_evidence(&evidence);
    if failures.is_empty() {
        println!("Phase 7 Part Design 1.0 release evidence passed.");
        return;
    }
    eprintln!("Phase 7 release is blocked by {} item(s):", failures.len());
    for failure in failures {
        eprintln!("- {failure}");
    }
    std::process::exit(1);
}
