//! Reproduce single-edge fillet behaviour (ZeroCAD's apply_fillet path):
//! fillet_edges on one selected edge must yield a closed, healthy solid.

use openrcad_algo::{fillet_edges, RollingBallError};
use openrcad_foundation::Pnt;
use openrcad_primitives::make_box;
use openrcad_topo::Edge;

fn report(name: &str, r: &Result<openrcad_topo::Solid, RollingBallError>) {
    match r {
        Ok(s) => {
            let hr = s.health_report();
            println!(
                "{name}: OK faces={} watertight={} healthy={} errors={:?}",
                s.face_count(), s.is_watertight(), hr.is_healthy(), hr.errors
            );
        }
        Err(e) => println!("{name}: ERR {e:?}"),
    }
}

#[test]
fn fillet_one_box_edge_closed() {
    // A 10-cube; round the top-front edge (y=0, z=10), running along X.
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(10.0, 0.0, 10.0));
    let r = fillet_edges(&cube, std::slice::from_ref(&edge), 2.0);
    report("fillet_one_box_edge", &r);
    let s = r.expect("box edge fillet should succeed");
    assert!(s.is_watertight(), "filleted box must be watertight");
    assert!(s.health_report().is_healthy(), "filleted box must be healthy");
}

#[test]
fn fillet_thin_plate_edge_closed() {
    // A thin plate (40x40x2); round a top long edge running along X.
    let plate = make_box(&Pnt::origin(), 40.0, 40.0, 2.0);
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 2.0), Pnt::new(40.0, 0.0, 2.0));
    let r = fillet_edges(&plate, std::slice::from_ref(&edge), 0.5);
    report("fillet_thin_plate_edge", &r);
    let s = r.expect("thin-plate edge fillet should succeed");
    assert!(s.is_watertight(), "filleted plate must be watertight");
    assert!(s.health_report().is_healthy(), "filleted plate must be healthy");
}

#[test]
fn fillet_edge_of_union_result() {
    use openrcad_algo::{boolean, BooleanOp};
    // Body produced by a union (how ZeroCAD makes most bodies), then fillet an
    // outer edge of it.
    let a = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let b = make_box(&Pnt::new(10.0, 0.0, 0.0), 20.0, 20.0, 10.0);
    let body = boolean(&a, &b, BooleanOp::Fuse); // 30x20x10 merged box
    println!("union body: faces={} watertight={}", body.face_count(), body.is_watertight());
    // Top-front edge of the merged box: y=0, z=10, x in [0,30].
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(30.0, 0.0, 10.0));
    let r = fillet_edges(&body, std::slice::from_ref(&edge), 2.0);
    report("fillet_edge_of_union", &r);
    // The merged box must expose its full-span top edge (collinear sub-edges
    // collapsed) so endpoint selection finds it instead of SpineNotOnFace.
    let s = r.expect("filleting a boolean-result edge must succeed");
    assert!(s.is_watertight() && s.health_report().is_healthy());
}

/// The tessellated fillet must be crack-free, not just the B-Rep. A cylinder
/// fillet's end-cap arc maps to a straight parameter line on the cylinder side
/// (chorded) but a true arc on the planar side: independent face tessellation
/// leaves a lens gap there (and a bowed tangent line on long edges) that renders
/// as a bright "white line" and leaks STL. `mesh::tessellate` (interior-sample
/// inset + boundary-lens stitch) must close it: every triangle edge is shared by
/// exactly two triangles (no single-referenced boundary edge on a closed solid).
#[test]
fn filleted_box_tessellation_is_crack_free() {
    use openrcad_mesh::tessellate;
    use std::collections::HashMap;

    // A spread of sizes/radii: small cube, long thin slab, big radius, thin plate.
    for (w, h, d, r) in [
        (10.0, 10.0, 10.0, 2.0),
        (60.0, 40.0, 20.0, 4.0),
        (60.0, 40.0, 20.0, 8.0),
        (120.0, 40.0, 20.0, 6.0),
        (40.0, 40.0, 3.0, 0.5),
    ] {
        let block = make_box(&Pnt::origin(), w, h, d);
        let edge = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
        let solid = fillet_edges(&block, std::slice::from_ref(&edge), r)
            .unwrap_or_else(|e| panic!("fillet {w}x{h}x{d} r{r} failed: {e:?}"));

        let mesh = tessellate(&solid, 0.05, 0.5);
        let gpu = mesh.gpu_mesh();
        // Quantized vertex position -> a small integer key, so the per-triangle
        // copies of a shared corner collapse to one id (edges are then keyed by
        // ordered id pairs, keeping the map's type simple).
        type Key = (i64, i64, i64);
        let q = |i: usize| -> Key {
            let b = i * 3;
            let g = |v: f32| (v as f64 * 1e4).round() as i64;
            (
                g(gpu.positions[b]),
                g(gpu.positions[b + 1]),
                g(gpu.positions[b + 2]),
            )
        };
        let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
        for t in gpu.indices.chunks_exact(3) {
            for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
                let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
                let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
                *edges.entry(k).or_insert(0) += 1;
            }
        }
        let cracks = edges.values().filter(|&&c| c == 1).count();
        assert_eq!(
            cracks, 0,
            "fillet {w}x{h}x{d} r{r}: tessellation has {cracks} boundary (crack) edges"
        );
    }
}

/// The user's reported case: a box with a circular pocket bored into its TOP
/// face (so the top face owns a hole as an inner wire), then fillet one of that
/// top face's *outer* boundary edges. Chamfer (a boolean cutter) handles this
/// fine, but the fillet's face-trimming used to bail with `UnsupportedTrimTopology`
/// the moment an adjacent face had any inner wire — so the fillet silently did
/// nothing. The hole sits in the face interior, far from the trimmed edge, so the
/// blend is perfectly well-defined; the trim must preserve the inner wires.
#[test]
fn fillet_top_edge_of_a_face_that_has_a_bored_hole() {
    use openrcad_algo::{boolean, BooleanOp};
    use openrcad_foundation::{Ax2, Dir};
    use openrcad_primitives::make_cylinder;

    // 40 x 20 x 10 block; pocket Ø8 bored 6mm into the top (z=10) face center.
    let block = make_box(&Pnt::origin(), 40.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(20.0, 10.0, 4.0), Dir::dz()), 4.0, 6.0);
    let body = boolean(&block, &drill, BooleanOp::Cut);
    println!(
        "holed body: faces={} watertight={}",
        body.face_count(),
        body.is_watertight()
    );
    assert!(body.is_watertight(), "the bored body itself must be watertight");

    // Top-front edge of the block (y=0, z=10, x in [0,40]) — on the holed top face.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(40.0, 0.0, 10.0));
    let r = fillet_edges(&body, std::slice::from_ref(&edge), 3.0);
    report("fillet top edge of holed face", &r);
    let s = r.expect("filleting an edge of a face that owns a hole must succeed");
    assert!(
        s.is_watertight() && s.health_report().is_healthy(),
        "filleted holed-face body must be watertight + healthy"
    );
}

#[test]
fn fillet_various_radii_on_plate() {
    let plate = make_box(&Pnt::origin(), 40.0, 40.0, 3.0);
    for radius in [0.5, 1.0, 1.4] {
        let edge = Edge::between_points(Pnt::new(0.0, 0.0, 3.0), Pnt::new(40.0, 0.0, 3.0));
        let r = fillet_edges(&plate, std::slice::from_ref(&edge), radius);
        report(&format!("plate r={radius}"), &r);
        let s = r.unwrap_or_else(|e| panic!("plate r={radius} should fillet: {e:?}"));
        assert!(
            s.is_watertight() && s.health_report().is_healthy(),
            "plate r={radius} must be watertight+healthy"
        );
    }
    // A radius larger than half the 3mm thickness can't fit — must be a clean
    // error, never an Ok with a degenerate (zero-length-edge) solid.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 3.0), Pnt::new(40.0, 0.0, 3.0));
    let too_big = fillet_edges(&plate, std::slice::from_ref(&edge), 2.9);
    report("plate r=2.9 (too big)", &too_big);
    match too_big {
        Err(_) => {}
        Ok(s) => assert!(
            s.is_watertight() && s.health_report().is_healthy(),
            "an accepted oversize fillet must still be valid (got degenerate solid)"
        ),
    }
}
