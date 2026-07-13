//! Timing probe for the Thread feature: how expensive is one full re-eval of a
//! threaded rod (what every preview step pays), and how dense is the resulting
//! mesh (what every CPU-painted frame pays). Run with:
//!   cargo test --release -p zerocad-core --test perf_thread_probe -- --nocapture

use std::collections::HashSet;
use std::time::Instant;
use zerocad_core::parametric::FaceRef;
use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

fn threaded_rod(r: f32, h: f32, pitch: f32, depth: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".into(),
        name: "cyl_1".into(),
        feature: FeatureType::Cylinder { r, h },
    });
    g.add_feature(FeatureNode {
        id: "thread_2".into(),
        name: "thread_2".into(),
        feature: FeatureType::Thread {
            target: "cyl_1".into(),
            face: FaceRef {
                centroid: [r, h / 2.0, 0.0],
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch,
            depth,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "probe".into(),
        },
    });
    g.add_dependency("cyl_1", "thread_2");
    g
}

#[test]
fn probe_thread_eval_and_mesh_cost() {
    let hidden = HashSet::new();
    for (label, r, h, pitch) in [
        ("M6x1 x 10mm", 3.0f32, 10.0f32, 1.0f32),
        ("M6x1 x 30mm", 3.0, 30.0, 1.0),
        ("M12x1.75 x 60mm", 6.0, 60.0, 1.75),
    ] {
        let g = threaded_rod(r, h, pitch, 0.6134 * pitch);
        // Cold eval (what a committed thread costs when previews re-run the model).
        let t0 = Instant::now();
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&hidden).unwrap();
        let cold = t0.elapsed();
        // Same instance again: full final-checkpoint hit.
        let t1 = Instant::now();
        let _ = g.evaluate_bodies_with_warnings(&hidden).unwrap();
        let same = t1.elapsed();
        // Clone cost (what each preview worker spawn pays up front).
        let t2 = Instant::now();
        let g2 = g.clone();
        let clone_t = t2.elapsed();
        // Eval on the clone (steady preview-step cost).
        let t3 = Instant::now();
        let _ = g2.evaluate_bodies_with_warnings(&hidden).unwrap();
        let warm = t3.elapsed();
        // Draft eval on a fresh clone (the actual preview-worker call).
        let g3 = g.clone();
        let t4 = Instant::now();
        let _ = g3.evaluate_bodies_draft(&hidden).unwrap();
        let draft = t4.elapsed();
        let mesh = &bodies.iter().find(|(id, _)| id == "cyl_1").unwrap().1;
        eprintln!(
            "{label}: cold {cold:?} | same-instance {same:?} | clone {clone_t:?} | clone re-eval {warm:?} | clone draft {draft:?} | {} tris, {} edge segs, warnings: {warnings:?}",
            mesh.indices.len() / 3,
            mesh.edge_indices.len() / 2,
        );
    }
}
