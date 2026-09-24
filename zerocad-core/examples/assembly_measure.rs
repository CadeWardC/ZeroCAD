//! Repeatable CPU assembly measurements. GPU upload/presentation are excluded.
#[path = "../tests/support/assembly_corpus.rs"]
mod corpus;
use std::{hint::black_box, path::PathBuf, time::Instant};
use zerocad_core::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if cfg!(debug_assertions) {
        return Err("run with --release".into());
    }
    let destination = PathBuf::from(std::env::args().nth(1).ok_or("supply output directory")?);
    std::fs::create_dir_all(&destination)?;
    let commit = String::from_utf8(
        std::process::Command::new("git")
            .args(["rev-parse", "HEAD"])
            .output()?
            .stdout,
    )?
    .trim()
    .to_owned();
    let machine = format!(
        "{}; {}; {}; {} logical CPUs",
        std::env::var("COMPUTERNAME").unwrap_or_else(|_| "local".into()),
        std::env::consts::OS,
        std::env::consts::ARCH,
        std::thread::available_parallelism()?
    );
    let mut sources = std::collections::BTreeMap::new();
    for path in [
        "Cargo.toml",
        "Cargo.lock",
        "zerocad-core/Cargo.toml",
        "zerocad-core/src/assembly.rs",
        "zerocad-core/src/assembly_ops.rs",
        "zerocad-core/src/assembly_mates.rs",
        "zerocad-core/src/assembly_solver.rs",
        "zerocad-core/src/assembly_solver_release.rs",
        "zerocad-core/src/assembly_release.rs",
        "zerocad-core/tests/support/assembly_corpus.rs",
        "zerocad-core/examples/assembly_measure.rs",
    ] {
        sources.insert(
            path,
            blake3::hash(&std::fs::read(path)?).to_hex().to_string(),
        );
    }
    std::fs::write(
        destination.join("environment.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "base_commit":commit,"machine":machine,
            "source_scope":"assembly solver and fixture inputs; working tree is not a committed release",
            "source_blake3":sources,
            "executable_blake3":blake3::hash(&std::fs::read(std::env::current_exe()?)?).to_hex().to_string(),
            "rustc":String::from_utf8(std::process::Command::new("rustc").arg("--version").output()?.stdout)?,
            "unix_seconds":std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs(),
            "warmup_samples":1,"measured_samples":31
        }))?,
    )?;
    let mut output = AssemblyV2ReleaseEvidence {
        schema: ASSEMBLY_V2_EVIDENCE_SCHEMA,
        commit,
        build_profile: "release".into(),
        machine_fingerprint: machine,
        calibration_cases: vec![],
        stress_corpora: vec![],
    };
    let mut observations = vec![];
    for (count, dense, name) in [
        (4, false, "four_plate_calibration"),
        (1000, false, "plate_sparse_chain"),
        (1000, true, "plate_closed_loops"),
    ] {
        eprintln!("Measuring {name}");
        let (assembly, context) = corpus::corpus(count, dense);
        let mut calibration = MateCalibrationCaseEvidence {
            name: name.into(),
            path: if count > 32 {
                MateSolvePath::SparsePcg
            } else {
                MateSolvePath::DenseSvd
            },
            assembly_diagonal_mm: context.assembly_diagonal_mm,
            final_translation_residual_mm: vec![],
            final_angle_residual_rad: vec![],
        };
        let mut stress = MateStressEvidence {
            name: name.into(),
            kind: if dense {
                MateStressCorpusKind::DenseConstraintGraph
            } else {
                MateStressCorpusKind::SparseChain
            },
            occurrence_count: count as u32,
            mate_count: assembly.mates.as_ref().unwrap().mates.len() as u32,
            offline_solve_ms: vec![],
            drag_frame_ms: vec![],
            nonlinear_iterations: vec![],
            damping_retries: vec![],
            sparse_iterations: vec![],
        };
        let mut convergence = vec![];
        let mut drag_convergence = vec![];
        for sample in 0..32 {
            // Fresh, unsolved input every sample; no repeated no-op warm-cache solve.
            let mut input = assembly.clone();
            let last = input.occurrences.get_mut(&count).unwrap();
            let mut translation = last.manual_placement.translation();
            translation[2] += 0.01;
            last.manual_placement = last.manual_placement.with_translation(translation)?;
            let started = Instant::now();
            let result = black_box(solve_assembly_mates_committed(&input, &context));
            let elapsed = started.elapsed().as_secs_f64() * 1000.0;
            let mut drag_context = context.clone();
            let mut target = assembly.occurrences[&count].manual_placement.translation();
            target[1] += 0.01;
            drag_context.temporary_pose_targets.insert(
                count,
                assembly.occurrences[&count]
                    .manual_placement
                    .with_translation(target)?,
            );
            let started = Instant::now();
            let drag = black_box(solve_assembly_mates_interactive(&assembly, &drag_context));
            let drag_ms = started.elapsed().as_secs_f64() * 1000.0;
            if sample == 0 {
                continue;
            }
            convergence.push(result.converged);
            drag_convergence.push(drag.converged);
            assert!(result.evidence.iter().all(|e| e.path == calibration.path));
            calibration.final_translation_residual_mm.push(
                result
                    .evidence
                    .iter()
                    .map(|e| e.final_position_residual_mm)
                    .fold(0.0, f64::max),
            );
            calibration.final_angle_residual_rad.push(
                result
                    .evidence
                    .iter()
                    .map(|e| e.final_angle_residual_rad)
                    .fold(0.0, f64::max),
            );
            stress.offline_solve_ms.push(elapsed);
            stress.drag_frame_ms.push(drag_ms);
            stress.nonlinear_iterations.push(
                result
                    .evidence
                    .iter()
                    .map(|e| e.nonlinear_iterations as u32)
                    .sum(),
            );
            stress.damping_retries.push(
                result
                    .evidence
                    .iter()
                    .map(|e| e.damping_retries as u32)
                    .sum(),
            );
            stress.sparse_iterations.push(
                result
                    .evidence
                    .iter()
                    .map(|e| e.sparse_iterations as u32)
                    .sum(),
            );
            eprintln!(
                "{name}: {sample}/31, solve {elapsed:.2} ms, drag {drag_ms:.2} ms, converged {}",
                result.converged
            );
        }
        observations.push(serde_json::json!({"name":name,"committed_converged":convergence,"interactive_converged":drag_convergence}));
        output.calibration_cases.push(calibration);
        if count == 1000 {
            output.stress_corpora.push(stress);
        }
        std::fs::write(
            destination.join("solver.json"),
            serde_json::to_vec_pretty(&output)?,
        )?;
        std::fs::write(
            destination.join("convergence.json"),
            serde_json::to_vec_pretty(&observations)?,
        )?;
    }
    let (assembly, _) = corpus::corpus(1000, false);
    let bytes = write_project_document_to_vec(
        &ProjectDocument::Assembly(assembly),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )?;
    let mut open_ms = vec![];
    for sample in 0..32 {
        let start = Instant::now();
        let loaded = read_project_document_from_slice(&bytes, &LoadOptions::default())?;
        let ProjectDocument::Assembly(assembly) = loaded.document else {
            unreachable!()
        };
        for definition in assembly.definitions.values() {
            black_box(prepare_part_definition(
                &definition.compact_snapshot,
                definition.source_basename.as_deref(),
                &LoadOptions::default(),
            )?);
        }
        black_box(
            assembly
                .occurrences
                .values()
                .map(|o| o.resolved_placement().to_scene_placement())
                .collect::<Vec<_>>(),
        );
        if sample > 0 {
            open_ms.push(start.elapsed().as_secs_f64() * 1000.0);
        }
    }
    let failures = validate_assembly_v2_release_evidence(&output);
    std::fs::write(
        destination.join("cpu-open.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "scope":"in-memory decode, definition hydration and CPU placement conversion; excludes file IO, mate solve, GPU and presentation",
            "occurrence_count":1000,"samples_ms":open_ms,"validator_failures":failures,
            "qualification":"local CPU observations only; source fingerprint and convergence must be checked; not V1 frame evidence"
        }))?,
    )?;
    eprintln!("Validator findings: {failures:?}");
    Ok(())
}
