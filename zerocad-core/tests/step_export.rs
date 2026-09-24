use std::collections::HashSet;
use zerocad_core::*;

fn part() -> Document {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Cade's plate".into(),
        feature: FeatureType::Box {
            w: 10.,
            h: 8.,
            d: 6.,
        },
    });
    Document::from_graph(graph, Unit::Inch)
}

#[test]
fn step_export_preserves_dimensions_and_replaces_atomically() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("part.step");
    std::fs::write(&path, b"previous").unwrap();
    assert_eq!(
        step_export::write_document_step(&part(), &HashSet::new(), &path).unwrap(),
        1
    );
    let bytes = std::fs::read(&path).unwrap();
    let text = std::str::from_utf8(&bytes).unwrap();
    assert!(text.contains("SI_UNIT(.MILLI.,.METRE.)"));
    assert!(text.contains("Cade''s plate"));
    let solid = openrcad::exchange::read_step_str_operation(text)
        .unwrap()
        .value;
    let properties = openrcad::mesh::mass_properties(
        &openrcad::mesh::tessellate_checked(&solid, 0.01, 0.1).unwrap(),
    )
    .unwrap();
    assert!((properties.volume - 480.).abs() < 1e-6);
    assert!(step_export::write_document_step(&Document::new(), &HashSet::new(), &path).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), bytes);
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
    assert!(step_export::write_document_step(&part(), &HashSet::new(), dir.path()).is_err());
}

#[test]
fn multiple_bodies_are_all_named_in_one_step_document() {
    let document = part();
    let mut graph = document.clone_authoritative();
    graph.add_feature(FeatureNode {
        id: "other".into(),
        name: "第二部品".into(),
        feature: FeatureType::Box {
            w: 2.,
            h: 3.,
            d: 4.,
        },
    });
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("multi.step");
    assert_eq!(
        step_export::write_document_step(&graph, &HashSet::new(), &path).unwrap(),
        2
    );
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    let error = openrcad::exchange::read_step_str_operation(&text).unwrap_err();
    assert!(
        error
            .to_string()
            .contains("multiple STEP solids cannot be discarded"),
        "{error}"
    );

    assert_eq!(text.matches("SHAPE_DEFINITION_REPRESENTATION(").count(), 2);
    if let Some(destination) = std::env::var_os("ZEROCAD_INTERCHANGE_EVIDENCE_DIR") {
        let destination = std::path::PathBuf::from(destination);
        std::fs::create_dir_all(&destination).unwrap();
        std::fs::copy(path, destination.join("multi.step")).unwrap();
        std::fs::write(
            destination.join("multi.json"),
            r#"{"volume":504.0,"body_count":2}"#,
        )
        .unwrap();
    }
}
