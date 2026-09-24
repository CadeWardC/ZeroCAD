#[path = "support/assembly_corpus.rs"]
mod corpus;
use zerocad_core::*;

#[test]
fn step_preserves_shared_parts_names_and_resolved_placements() {
    let (mut assembly, _) = corpus::corpus(3, false);
    assembly.mates = None;
    assembly.occurrences.get_mut(&2).unwrap().name = "Rotated plate".into();
    assembly
        .occurrences
        .get_mut(&2)
        .unwrap()
        .resolved_placement_override =
        Some(RigidPlacement::from_euler_xyz_degrees([30.0, 20.0, 5.0], [0.0, 0.0, 90.0]).unwrap());
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("assembly.step");
    assert_eq!(
        step_export::write_assembly_step(&assembly, &path).unwrap(),
        3
    );
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 1);
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 3);
    assert!(text.contains("Rotated plate"));
    assert!(text.contains("30.0, 20.0, 5.0"));
    if let Some(destination) = std::env::var_os("ZEROCAD_INTERCHANGE_EVIDENCE_DIR") {
        let destination = std::path::PathBuf::from(destination);
        std::fs::create_dir_all(&destination).unwrap();
        std::fs::copy(&path, destination.join("assembly.step")).unwrap();
        std::fs::write(destination.join("assembly.json"),
            r#"{"volume":480.0,"body_count":3,"centers":[[5.0,4.0,1.0],[26.0,25.0,6.0],[5.0,4.0,25.0]]}"#).unwrap();
    }
    assembly.presentation.hidden_occurrences.insert(3);
    assert_eq!(
        step_export::write_assembly_step(&assembly, &path).unwrap(),
        2
    );
    assembly.presentation.hidden_occurrences.extend([1, 2]);
    let saved = std::fs::read(&path).unwrap();
    assert!(step_export::write_assembly_step(&assembly, &path).is_err());
    assert_eq!(saved, std::fs::read(&path).unwrap());
}

#[test]
fn unsolved_mates_cannot_silently_export_manual_placements() {
    let (assembly, _) = corpus::corpus(3, false);
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("assembly.step");
    std::fs::write(&path, b"previous export").unwrap();
    assert!(step_export::write_assembly_step(&assembly, &path)
        .unwrap_err()
        .contains("Solve assembly mates"));
    assert_eq!(std::fs::read(path).unwrap(), b"previous export");
}
