#[path = "../benches/support/real_parts.rs"]
mod real_parts;
use zerocad_core::*;

#[test]
fn mechanical_parts_survive_cold_warm_save_reopen_and_step_export() {
    let directory = tempfile::tempdir().unwrap();
    for &(name, builder) in real_parts::PARTS {
        let graph = builder();
        let (cold, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "{name}: {warnings:?}");
        let (warm, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "{name}: {warnings:?}");
        let volume = |bodies: &Vec<(String, MockMesh)>| {
            bodies
                .iter()
                .map(|(_, mesh)| mesh.mass_properties().unwrap().volume)
                .sum::<f64>()
        };
        assert!((volume(&cold) - volume(&warm)).abs() < 1e-6);
        let document = Document::from_graph(graph, Unit::Millimeter);
        let bytes = write_document_to_vec(
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .unwrap();
        let loaded = read_document_from_slice(&bytes, &LoadOptions::default())
            .unwrap()
            .document;
        let reopened = loaded.evaluate_bodies(&Default::default()).unwrap();
        assert!((volume(&cold) - volume(&reopened)).abs() < 1e-6, "{name}");
        let path = directory.path().join(format!("{name}.step"));
        step_export::write_document_step(&loaded, &Default::default(), &path)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        if let Some(destination) = std::env::var_os("ZEROCAD_INTERCHANGE_EVIDENCE_DIR") {
            let destination = std::path::PathBuf::from(destination);
            std::fs::create_dir_all(&destination).unwrap();
            std::fs::copy(&path, destination.join(format!("{name}.step"))).unwrap();
            std::fs::write(
                destination.join(format!("{name}.json")),
                serde_json::to_vec(
                    &serde_json::json!({"volume": volume(&cold), "body_count": cold.len()}),
                )
                .unwrap(),
            )
            .unwrap();
        }
    }
}

#[test]
fn nested_text_counter_cutouts_export_after_save_and_reload() {
    for (label, through) in [
        ("ZeroCAD 07", false),
        ("0080", false),
        ("B8@0", false),
        ("0080", true),
    ] {
        let mut graph = real_parts::engraved_plate_text(label);
        if through {
            for node in graph.graph.node_weights_mut() {
                if let FeatureType::Extrude {
                    mode: ExtrudeMode::Cut,
                    depth,
                    ..
                } = &mut node.feature
                {
                    *depth = -9.;
                }
            }
        }
        let document = Document::from_graph(graph, Unit::Millimeter);
        let bytes = write_document_to_vec(
            &document,
            &SaveOptions::default(),
            &HydrationBundle::default(),
        )
        .unwrap();
        let document = read_document_from_slice(&bytes, &LoadOptions::default())
            .unwrap()
            .document;
        let bodies = document.evaluate_bodies(&Default::default()).unwrap();
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("nested.step");
        let solid_count =
            step_export::write_document_step(&document, &Default::default(), &path).unwrap();
        assert!(solid_count > 0);
        if let Some(destination) = std::env::var_os("ZEROCAD_INTERCHANGE_EVIDENCE_DIR") {
            let destination = std::path::PathBuf::from(destination);
            std::fs::create_dir_all(&destination).unwrap();
            let name = format!(
                "{}{}",
                label.replace([' ', '@'], "_"),
                if through { "_through" } else { "" }
            );
            std::fs::copy(&path, destination.join(format!("{name}.step"))).unwrap();
            std::fs::write(destination.join(format!("{name}.json")), serde_json::to_vec(&serde_json::json!({"volume": bodies.iter().map(|(_, mesh)| mesh.mass_properties().unwrap().volume).sum::<f64>(), "body_count": solid_count})).unwrap()).unwrap();
        }
    }
}

#[test]
fn closed_cavity_step_export_preserves_the_void_and_import_refuses_partial_solid() {
    let mut graph = real_parts::open_enclosure();
    for node in graph.graph.node_weights_mut() {
        if let FeatureType::Shell { open_faces, .. } = &mut node.feature {
            open_faces.clear();
        }
    }
    let bodies = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
    let mesh = openrcad::mesh::tessellate_checked(&bodies[0].1[0], 0.005, 0.05).unwrap();
    let properties = openrcad::mesh::mass_properties(&mesh).unwrap();
    assert!(
        (properties.volume - 15_744.0).abs() < 1e-6,
        "{properties:?}"
    );
    for (actual, expected) in properties.centroid.into_iter().zip([30.0, 20.0, 10.0]) {
        assert!((actual - expected).abs() < 1e-6, "{properties:?}");
    }
    let inspection = graph.inspect_body(&bodies[0].0).unwrap();
    assert!((inspection.volume_mm3.unwrap() - 15_744.0).abs() < 1e-6);
    for _ in 0..2 {
        let (display, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        assert!((display[0].1.mass_properties().unwrap().volume - 15_744.0).abs() < 1e-6);
    }
    let document = Document::from_graph(graph, Unit::Millimeter);
    let bytes = write_document_to_vec(
        &document,
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    let document = read_document_from_slice(&bytes, &LoadOptions::default())
        .unwrap()
        .document;
    assert!(
        (document
            .inspect_body(&bodies[0].0)
            .unwrap()
            .volume_mm3
            .unwrap()
            - 15_744.0)
            .abs()
            < 1e-6
    );
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("cavity.step");
    std::fs::write(&path, b"previous").unwrap();
    assert_eq!(
        step_export::write_document_step(&document, &Default::default(), &path).unwrap(),
        1
    );
    let text = std::fs::read_to_string(&path).unwrap();
    if let Some(destination) = std::env::var_os("ZEROCAD_INTERCHANGE_EVIDENCE_DIR") {
        let destination = std::path::PathBuf::from(destination);
        std::fs::create_dir_all(&destination).unwrap();
        std::fs::copy(&path, destination.join("closed_cavity.step")).unwrap();
        std::fs::write(
            destination.join("closed_cavity.json"),
            r#"{"volume":15744.0,"body_count":1}"#,
        )
        .unwrap();
    }
    assert!(text.contains("BREP_WITH_VOIDS"));
    let error = openrcad::exchange::read_step_str_operation(&text).unwrap_err();
    assert!(
        error.to_string().contains("cavity import is not qualified"),
        "{error}"
    );
}

#[test]
fn dense_text_placement_preserves_exact_linear_history_and_material() {
    use openrcad::foundation::{Pnt, TolerancePolicy, Trsf, Vec as GeomVec};
    use openrcad::topo::TopologyChange;
    let graph = real_parts::engraved_plate_text("ZeroCAD 07");
    let bodies = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
    let source = &bodies[0].1[0];
    let measure = |solid: &openrcad::topo::Solid| {
        openrcad::mesh::mass_properties(
            &openrcad::mesh::tessellate_checked(solid, 0.01, 0.1).unwrap(),
        )
        .unwrap()
    };
    let before = measure(source);
    for (transform, factor) in [
        (Trsf::translation(GeomVec::new(0.25, 0., 0.)), 1_f64),
        (Trsf::translation(GeomVec::new(250., -400., 50.)), 1.),
        (Trsf::scale(&Pnt::origin(), 0.5), 0.5),
        (Trsf::scale(&Pnt::origin(), 2.), 2.),
    ] {
        let result = openrcad::algo::transform_operation_with_policy(
            source,
            &transform,
            false,
            &TolerancePolicy::STANDARD,
        )
        .unwrap();
        assert!(result.recovery.actions.is_empty());
        assert!(result.validation.is_valid());
        assert!(result
            .history
            .coverage_for_solid(&result.value)
            .is_complete());
        result.history.validate().unwrap();
        assert!(result.history.changes.iter().all(|change| matches!(change,
            TopologyChange::Modified { source, result } if source.operand == 0 && source.entity == *result)));
        assert_eq!(result.value.face_count(), source.face_count());
        let after = measure(&result.value);
        assert!((after.volume - before.volume * factor.powi(3)).abs() < before.volume * 1e-4);
    }
    assert!((measure(source).volume - before.volume).abs() < 1e-9);
}
