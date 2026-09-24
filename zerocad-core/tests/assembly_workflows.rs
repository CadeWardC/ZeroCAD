#[path = "support/assembly_corpus.rs"]
mod corpus;
use zerocad_core::assembly::{AssemblyCommand, AssemblyMateEdit};
use zerocad_core::assembly_ops::prepare_occurrence_replacement;
use zerocad_core::*;

#[test]
fn plate_array_replacement_suppression_repair_and_history_roundtrip() {
    let (mut assembly, context) = corpus::corpus(4, true);
    let original = assembly.clone_authoritative();
    let solution = solve_assembly_mates_committed(&assembly, &context);
    assert!(solution.converged);
    apply_mate_solution(&mut assembly, &solution).unwrap();
    let (replacement, _) =
        prepare_occurrence_replacement(&assembly, &[2, 3], corpus::part(14.0)).unwrap();
    assert_eq!(replacement.occurrences[&2].id, 2);
    assert_eq!(
        replacement.occurrences[&2].manual_placement,
        original.occurrences[&2].manual_placement
    );
    assert_ne!(
        replacement.occurrences[&2].definition_model_hash,
        original.occurrences[&2].definition_model_hash
    );
    assert_eq!(
        replacement.occurrences[&4].definition_model_hash,
        original.occurrences[&4].definition_model_hash
    );
    assert_eq!(replacement.mates, original.mates);
    assembly = replacement;

    // A removed face remains authored; only resolution disables the mate.
    let mut broken = context.clone();
    broken.frames.remove(&1);
    let unresolved = solve_assembly_mates_committed(&assembly, &broken);
    assert_eq!(unresolved.statuses[&1], MateSolveStatus::Unresolved);
    let before = assembly.clone();
    let staged = assembly
        .prepare_command(AssemblyCommand::EditMate {
            mate_id: 1,
            edit: AssemblyMateEdit::SetSuppressed(true),
        })
        .unwrap();
    let (_, undo) = staged.commit(&mut assembly);
    assert_eq!(undo, before);
    assert_eq!(
        solve_assembly_mates_committed(&assembly, &broken).statuses[&1],
        MateSolveStatus::Suppressed
    );
    let redo = std::mem::replace(&mut assembly, undo);
    assert_eq!(assembly, before);
    let _ = std::mem::replace(&mut assembly, redo);
    let loaded = corpus::roundtrip(&assembly);
    assert_eq!(loaded.clone_authoritative(), assembly.clone_authoritative());
    assembly = loaded;
    assembly
        .prepare_command(AssemblyCommand::EditMate {
            mate_id: 1,
            edit: AssemblyMateEdit::SetSuppressed(false),
        })
        .unwrap()
        .commit(&mut assembly);
    let repaired = solve_assembly_mates_committed(&assembly, &context);
    assert!(repaired.converged);
    assert_ne!(repaired.statuses[&1], MateSolveStatus::Unresolved);
    for (id, placement) in repaired.placements {
        assert!((placement.translation()[2] - 12.0 * (id - 1) as f64).abs() < 1e-6);
    }
}

#[test]
fn invalid_replacement_and_mate_edit_leave_live_assembly_unchanged() {
    let (assembly, _) = corpus::corpus(3, false);
    let before = assembly.clone();
    assert!(prepare_occurrence_replacement(&assembly, &[999], corpus::part(20.0)).is_err());
    assert!(assembly
        .prepare_command(AssemblyCommand::EditMate {
            mate_id: 999,
            edit: AssemblyMateEdit::Delete
        })
        .is_err());
    assert_eq!(assembly, before);
    assert_eq!(
        corpus::roundtrip(&assembly).clone_authoritative(),
        before.clone_authoritative()
    );
}
