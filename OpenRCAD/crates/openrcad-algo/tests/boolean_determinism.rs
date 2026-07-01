//! Cross-process determinism probe for the boolean engine.
//!
//! Rust seeds `HashMap`/`HashSet` randomly per process, so any place the engine
//! lets map *iteration order* leak into its output makes the same inputs produce
//! a different (though often still valid) B-Rep from run to run — which is the
//! root of unstable downstream face/edge identity. This test prints a canonical
//! fingerprint (sorted, quantized vertex + edge geometry, hashed with the
//! fixed-key `DefaultHasher`) of a few representative boolean results. Run the
//! built test binary in several separate processes and compare the `FINGERPRINT`
//! lines: identical across runs ⇒ deterministic; differing ⇒ a nondeterminism
//! source remains. (Within one process the hash seed is fixed, so a single run
//! can't detect it — the comparison must be across processes.)

use openrcad_algo::{boolean, boolean_checked, fillet_edges, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Solid};
use std::hash::{Hash, Hasher};

/// A quantized 3D point — the unit of every fingerprint below.
type QPt = (i64, i64, i64);

fn fingerprint(s: &Solid) -> u64 {
    let q = |x: f64| (x * 1.0e6).round() as i64;

    let mut verts: Vec<QPt> = s
        .vertices()
        .iter()
        .map(|v| {
            let p = v.point();
            (q(p.x()), q(p.y()), q(p.z()))
        })
        .collect();
    verts.sort();

    let mut edges: Vec<(QPt, QPt)> = s
        .edges()
        .iter()
        .map(|e| {
            let a = e.start().point();
            let b = e.end().point();
            let ka = (q(a.x()), q(a.y()), q(a.z()));
            let kb = (q(b.x()), q(b.y()), q(b.z()));
            if ka <= kb {
                (ka, kb)
            } else {
                (kb, ka)
            }
        })
        .collect();
    edges.sort();

    // `DefaultHasher::new()` uses fixed keys (unlike `RandomState`), so this hash
    // is itself deterministic — the only variation it can report is real
    // variation in the sorted geometry.
    let mut h = std::collections::hash_map::DefaultHasher::new();
    verts.len().hash(&mut h);
    edges.len().hash(&mut h);
    s.face_count().hash(&mut h);
    verts.hash(&mut h);
    edges.hash(&mut h);
    h.finish()
}

/// Storage-order fingerprint: hashes faces in stored order, and within each face
/// the outer-loop edge endpoints in loop order. Unlike `fingerprint` (which sorts
/// first), this is sensitive to the *order* faces/edges are laid down — the FaceId
/// assignment order the fillet's topology resolution depends on.
fn fingerprint_ordered(s: &Solid) -> u64 {
    let q = |x: f64| (x * 1.0e6).round() as i64;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for f in s.shell().faces() {
        if let Some(w) = f.outer_wire() {
            for e in w.edges() {
                let p = e.start().point();
                (q(p.x()), q(p.y()), q(p.z())).hash(&mut h);
            }
        }
        0xFFu8.hash(&mut h); // face separator
    }
    h.finish()
}

fn report(name: &str, s: &Solid) {
    println!(
        "FINGERPRINT {name} = {:016x} / ord {:016x}  (V{} E{} F{})",
        fingerprint(s),
        fingerprint_ordered(s),
        s.vertex_count(),
        s.edge_count(),
        s.face_count()
    );
}

#[test]
fn print_boolean_fingerprints() {
    // Straddling circular bite (the flaky-downstream case's cut body).
    let block = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let cutter = make_cylinder(&Ax2::new(Pnt::new(20.0, 8.0, -1.0), Dir::dz()), 14.0, 12.0);
    let bite = boolean_checked(&block, &cutter, BooleanOp::Cut).expect("bite cut");
    report("straddling_bite", &bite);

    // Through-drill.
    let b2 = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(10.0, 10.0, -1.0), Dir::dz()), 3.0, 12.0);
    let drilled = boolean_checked(&b2, &drill, BooleanOp::Cut).expect("drill");
    report("through_drill", &drilled);

    // Two-box flush union.
    let u1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let u2 = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);
    let union = boolean_checked(&u1, &u2, BooleanOp::Fuse).expect("union");
    report("flush_union", &union);

    // Fillet of a drilled body's top edge — exercises the downstream fillet path
    // on a boolean result, where the flaky ghost-material test lives.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(20.0, 0.0, 10.0));
    if let Ok(filleted) = fillet_edges(&drilled, std::slice::from_ref(&edge), 2.0) {
        report("drill_then_fillet", &filleted);
    } else {
        println!("FINGERPRINT drill_then_fillet = ERR");
    }

    // Mesh tessellation of the drilled body — the flaky ghost-sample check
    // tessellates, so if the B-Rep is deterministic but this differs across
    // processes, the residual nondeterminism is in the mesher.
    let mesh = openrcad_mesh::tessellate(&drilled, 0.05, 0.5);
    let q = |x: f64| (x * 1.0e6).round() as i64;
    let mut tris: Vec<[QPt; 3]> = mesh
        .triangles
        .iter()
        .map(|t| {
            let mut tri = t.map(|i| {
                let p = mesh.vertices[i as usize];
                (q(p.x()), q(p.y()), q(p.z()))
            });
            tri.sort();
            tri
        })
        .collect();
    tris.sort();
    let mut h = std::collections::hash_map::DefaultHasher::new();
    tris.hash(&mut h);
    println!(
        "FINGERPRINT drill_mesh = {:016x}  (verts {} tris {})",
        h.finish(),
        mesh.vertices.len(),
        mesh.triangles.len()
    );

    // The circular-bite *cut body* — proven deterministic here. Its downstream
    // rolling-ball *fillet* is NOT yet deterministic (and can even hang under some
    // hash seeds — a separate fillet-subsystem defect, tracked apart from the
    // boolean engine), so it is deliberately not exercised in this gate.
    let cb_base = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let cb_axis = Ax2::new(Pnt::new(20.0, 8.0, -0.25), Dir::dz());
    let cb_cyl = make_cylinder(&cb_axis, 14.0, 10.5);
    let cb_body = boolean(&cb_base, &cb_cyl, BooleanOp::Cut);
    report("cbite_body", &cb_body);

    // The fillet of the circular-bite cutoff edge — no longer hangs (partition_face
    // rotation guard), but its result body / mesh may still vary per process.
    let x = 20.0 - (14.0_f64 * 14.0 - 3.0_f64 * 3.0).sqrt();
    let cb_edge = Edge::between_points(Pnt::new(0.0, 5.0, 10.0), Pnt::new(x, 5.0, 10.0));
    match fillet_edges(&cb_body, std::slice::from_ref(&cb_edge), 3.0) {
        Ok(cb_fil) => {
            report("cbite_fillet", &cb_fil);
            let m = openrcad_mesh::tessellate(&cb_fil, 0.05, 0.5);
            let mut t2: Vec<[QPt; 3]> = m
                .triangles
                .iter()
                .map(|t| {
                    let mut tri = t.map(|i| {
                        let p = m.vertices[i as usize];
                        (q(p.x()), q(p.y()), q(p.z()))
                    });
                    tri.sort();
                    tri
                })
                .collect();
            t2.sort();
            let mut h2 = std::collections::hash_map::DefaultHasher::new();
            t2.hash(&mut h2);
            println!(
                "FINGERPRINT cbite_mesh = {:016x}  (verts {} tris {})",
                h2.finish(),
                m.vertices.len(),
                m.triangles.len()
            );
        }
        Err(e) => println!("FINGERPRINT cbite_fillet = ERR {e:?}"),
    }
}
