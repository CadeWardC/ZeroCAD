//! Pinpoint the render-mesh cracks the GUI reports on EXTRUDED (prism) bodies and
//! cylinder boss-unions — the cases the existing `make_box` fillet test doesn't
//! cover. A closed solid's tessellation must have zero boundary edges (edges used
//! by exactly one triangle); each crack renders as a white line and breaks the
//! GUI's winding-orientation pass (→ inward "disappearing" faces).

use openrcad_algo::{boolean, fillet_edges, prism, BooleanOp};
use openrcad_foundation::{Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomSurface, Plane};
use openrcad_mesh::tessellate;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Wire};
use std::collections::HashMap;

type Key = (i64, i64, i64);

/// (#cracks, set of face-id pairs that border a crack edge).
fn crack_report(solid: &openrcad_topo::Solid) -> (usize, Vec<(u32, u32)>) {
    let mesh = tessellate(solid, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> Key {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    // edge -> (count, face_ids seen)
    let mut edges: HashMap<(Key, Key), (u32, Vec<u32>)> = HashMap::new();
    for (ti, t) in gpu.indices.chunks_exact(3).enumerate() {
        let fid = gpu.face_ids.get(ti).copied().unwrap_or(0);
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            let e = edges.entry(k).or_insert((0, Vec::new()));
            e.0 += 1;
            e.1.push(fid);
        }
    }
    let mut crack_faces: Vec<(u32, u32)> = Vec::new();
    let mut cracks = 0;
    for (c, fids) in edges.values() {
        if *c == 1 {
            cracks += 1;
            let f = fids[0];
            crack_faces.push((f, f));
        }
    }
    crack_faces.sort_unstable();
    crack_faces.dedup();
    (cracks, crack_faces)
}

fn rect_face(w: f64, h: f64) -> Face {
    Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            Dir::dz(),
        ))),
        Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(w, 0.0, 0.0)),
            Edge::between_points(Pnt::new(w, 0.0, 0.0), Pnt::new(w, h, 0.0)),
            Edge::between_points(Pnt::new(w, h, 0.0), Pnt::new(0.0, h, 0.0)),
            Edge::between_points(Pnt::new(0.0, h, 0.0), Pnt::origin()),
        ]),
    )
}

#[test]
fn diag_prism_plain() {
    let solid = prism(&rect_face(40.0, 30.0), GeomVec::new(0.0, 0.0, 15.0)).unwrap();
    let (cracks, faces) = crack_report(&solid);
    println!(
        "PLAIN PRISM: faces={} cracks={cracks} crack_face_ids={faces:?}",
        solid.face_count()
    );
    assert_eq!(cracks, 0, "a plain extruded box must tessellate crack-free");
}

#[test]
fn diag_prism_filleted() {
    let solid = prism(&rect_face(40.0, 30.0), GeomVec::new(0.0, 0.0, 15.0)).unwrap();
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 15.0), Pnt::new(40.0, 0.0, 15.0));
    let filleted = fillet_edges(&solid, std::slice::from_ref(&edge), 4.0)
        .expect("prism edge fillet should succeed");
    let (cracks, faces) = crack_report(&filleted);
    println!(
        "FILLETED PRISM: faces={} cracks={cracks} crack_face_ids={faces:?}",
        filleted.face_count()
    );
    assert_eq!(
        cracks, 0,
        "a filleted extruded box must tessellate crack-free"
    );
}

#[test]
fn diag_boss_union() {
    let box_ = make_box(&Pnt::origin(), 40.0, 30.0, 15.0);
    // Cylinder boss centered on the top, axis +Z, sitting on z=15, rising 12.
    let cyl = make_cylinder(
        &openrcad_foundation::Ax2::new(Pnt::new(20.0, 15.0, 15.0), Dir::dz()),
        6.0,
        12.0,
    );
    let body = boolean(&box_, &cyl, BooleanOp::Fuse);
    println!(
        "boss union: faces={} watertight={}",
        body.face_count(),
        body.is_watertight()
    );
    let (cracks, faces) = crack_report(&body);
    println!("BOSS UNION: cracks={cracks} crack_face_ids={faces:?}");
    assert_eq!(
        cracks, 0,
        "a cylinder boss-union must tessellate crack-free"
    );
}
