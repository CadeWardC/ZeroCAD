//! Perf probe (run with --release --nocapture): where does tessellation time go
//! for the bodies the GUI rebuilds per preview frame?

use std::time::Instant;

use openrcad_algo::{boolean, fillet_edges, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_mesh::tessellate;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::Edge;

#[test]
fn perf_tessellate_report() {
    // Plain box — baseline.
    let bx = make_box(&Pnt::new(0.0, 0.0, 0.0), 40.0, 30.0, 10.0);
    let t = Instant::now();
    let m = tessellate(&bx, 0.05, 0.13);
    println!(
        "box:              {:>8.1?}  ({} tris)",
        t.elapsed(),
        m.triangles.len()
    );

    // Circular-bite body (box minus straddling cylinder) — the GUI's cut case.
    let base = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let axis = Ax2::new(Pnt::new(20.0, 8.0, -0.25), Dir::dz());
    let cyl = make_cylinder(&axis, 14.0, 10.5);
    let t = Instant::now();
    let body = boolean(&base, &cyl, BooleanOp::Cut);
    println!("boolean(cut):     {:>8.1?}", t.elapsed());

    for (chord, angle) in [(0.05_f64, 0.13_f64), (0.5, 0.35)] {
        let t = Instant::now();
        let m = tessellate(&body, chord, angle);
        println!(
            "bite {chord}/{angle}:   {:>8.1?}  ({} tris)",
            t.elapsed(),
            m.triangles.len()
        );
    }

    // Filleted bite — the worst body a preview re-tessellates.
    let x = 20.0 - (14.0_f64 * 14.0 - 3.0_f64 * 3.0).sqrt();
    let edge = Edge::between_points(Pnt::new(0.0, 5.0, 10.0), Pnt::new(x, 5.0, 10.0));
    let t = Instant::now();
    let filleted = fillet_edges(&body, std::slice::from_ref(&edge), 3.0).expect("fillet");
    println!("fillet_edges:     {:>8.1?}", t.elapsed());

    let t = Instant::now();
    let m = tessellate(&filleted, 0.05, 0.13);
    println!(
        "filleted 0.05:    {:>8.1?}  ({} tris)",
        t.elapsed(),
        m.triangles.len()
    );

    // Phase breakdown on the filleted body.
    use openrcad_mesh::triangulate::{shared_edge_polylines, tessellate_face_budget};
    let faces = filleted.shell().faces();
    let t = Instant::now();
    let shared = shared_edge_polylines(&faces, 0.05, 0.13);
    println!("  shared_edges:   {:>8.1?}", t.elapsed());
    let mut face_times: Vec<(usize, std::time::Duration, usize)> = Vec::new();
    for (i, face) in faces.iter().enumerate() {
        let t = Instant::now();
        let fm = tessellate_face_budget(face, 0.05, 0.13, i as u32, Some(&shared));
        face_times.push((i, t.elapsed(), fm.triangles.len()));
    }
    face_times.sort_by_key(|(_, d, _)| std::cmp::Reverse(*d));
    let total: std::time::Duration = face_times.iter().map(|(_, d, _)| *d).sum();
    println!("  faces total:    {:>8.1?} (sequential)", total);
    for (i, d, tris) in face_times.iter().take(6) {
        let surf = faces[*i]
            .surface()
            .map(|s| format!("{s:?}").split('(').next().unwrap_or("?").to_string())
            .unwrap_or_default();
        println!("    face {i:>3} {surf:<18} {d:>8.1?}  ({tris} tris)");
    }
}
