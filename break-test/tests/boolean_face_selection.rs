//! A boolean's generated bore must remain selectable without normal heuristics.
mod common;
use common::*;
use zerocad_core::*;

#[test]
fn drilled_faces_keep_surface_kind_and_producer_after_cold_rebuild() {
    for diameter in [10., 12.] {
        let mut graph = ParametricGraph::new();
        add_box(&mut graph, "box_1", 30., 30., 20.);
        add_hole(
            &mut graph,
            "hole_2",
            "box_1",
            [15., 15., 20.],
            [0., 0., -1.],
            diameter,
            None,
            HoleKind::Simple,
        );
        for candidate in [&graph, &graph.clone_document()] {
            let (bodies, warnings) = candidate
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            assert!(warnings.is_empty(), "{warnings:?}");
            assert_eq!(bodies.len(), 1);
            let bore_faces: Vec<_> = bodies[0]
                .1
                .face_refs
                .iter()
                .filter_map(|face| face.topology.as_ref())
                .filter(|topology| {
                    topology
                        .face_id
                        .as_deref()
                        .is_some_and(|id| id.starts_with("cut:hole_2:tool-face:"))
                })
                .collect();
            assert!(!bore_faces.is_empty(), "generated bore face must be named");
            for topology in bore_faces {
                assert_eq!(topology.surface_kind.as_deref(), Some("cylinder"));
                assert_eq!(topology.producer_feature_id.as_deref(), Some("hole_2"));
                assert_eq!(topology.body_id.as_deref(), Some("box_1"));
            }
        }
    }
}
