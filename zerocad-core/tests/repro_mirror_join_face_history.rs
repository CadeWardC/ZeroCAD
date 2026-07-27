use std::collections::HashSet;

use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{
    read_document_file, EvaluationCancellation, EvaluationQuality, FeatureNode, FeatureType,
    LoadOptions, PatternKind, PlaneBase,
};

#[test]
fn mirrored_bugcase_join_has_complete_face_attribution() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../BugCaseExtruded.zcad");
    let Ok(loaded) = read_document_file(path, &LoadOptions::default()) else {
        eprintln!("skipping: {path} not found");
        return;
    };
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = loaded
        .document
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .expect("evaluate mirrored bug case");
    let source_mesh = &output
        .bodies
        .iter()
        .find(|(body, _)| body == "extrude_5")
        .expect("source body")
        .1;

    let selected = source_mesh
        .face_refs
        .iter()
        .find(|face| face.normal[0] > 0.99 && (face.centroid[0] - 8.2).abs() < 1.0e-3)
        .expect("saved bug-case +X end face");
    let mut document = loaded.document.clone_authoritative();
    let face = FaceRef {
        centroid: selected.centroid,
        normal: selected.normal,
        topology: selected.topology.as_ref().map(|topology| TopologyFaceRef {
            body_id: topology.body_id.clone(),
            component_id: topology.component_id.clone(),
            topology_version: topology.topology_version,
            face_id: topology.face_id.clone(),
            surface_kind: topology.surface_kind.clone(),
            producer_feature_id: topology.producer_feature_id.clone(),
            source_entity_id: topology.source_entity_id.clone(),
        }),
    };
    document.add_feature(FeatureNode {
        id: "mirror_7".into(),
        name: "Mirror 7".into(),
        feature: FeatureType::Pattern {
            source: "extrude_5".into(),
            kind: PatternKind::Mirror {
                plane: PlaneBase::YZ,
                face: Some(face),
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
    });
    document.add_dependency("extrude_5", "mirror_7");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let mirrored = document
        .evaluate_request(
            &HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .expect("evaluate mirrored face");

    assert_eq!(
        mirrored.bodies.len(),
        1,
        "the mirror must join to its source"
    );
    assert!(
        mirrored.diagnostics.iter().all(|diagnostic| {
            !diagnostic
                .message
                .contains("boolean result face could not be attributed")
        }),
        "mirror Join must retain face lineage: {:#?}",
        mirrored.diagnostics
    );
}
