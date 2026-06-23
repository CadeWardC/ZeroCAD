//! Fillet a top edge that runs into a concave vertical cut-cylinder.
//!
//! Desired ("trim flush"): the fillet is trimmed where it meets the cut; the cut
//! stays a clean full-height vertical cylinder (NOT rounded into a sphere octant),
//! and the fillet ends in a curved trim edge shared with the cut wall.

use core::f64::consts::PI;
use openrcad_algo::{boolean, chamfer_edges, fillet_edges, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::{Curve, GeomSurface};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Solid};

/// Box 10^3 with a vertical Ø8 cylinder cut out of the (10,10) corner. The rim
/// seam is rotated to 30/150/270 deg so the corner's 180->270 cut arc stays
/// seam-free (one clean wall face -> watertight Cut).
fn cut_body() -> Solid {
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let xdir = Dir::new((PI / 6.0).cos(), (PI / 6.0).sin(), 0.0);
    let axis = Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), xdir);
    let cyl = make_cylinder(&axis, 4.0, 12.0);
    boolean(&cube, &cyl, BooleanOp::Cut)
}

/// The GUI case: the same Ø8 corner cut, but with the rim seam rotated to 90° so
/// one seam (at 210°) falls *inside* the corner's 180->270° cut arc. The boolean
/// therefore produces the concave wall as TWO cocylindrical fragments meeting at
/// that seam; the cocylindrical merge must re-unite them into one cut face.
fn cut_body_seam_crossing() -> Solid {
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let xdir = Dir::new((PI / 2.0).cos(), (PI / 2.0).sin(), 0.0);
    let axis = Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), xdir);
    let cyl = make_cylinder(&axis, 4.0, 12.0);
    boolean(&cube, &cyl, BooleanOp::Cut)
}

fn has_sphere(s: &Solid) -> bool {
    s.shell()
        .faces()
        .iter()
        .any(|f| matches!(f.surface(), Some(GeomSurface::Sphere(_))))
}

/// The vertical (Z-axis) cylinder face of `radius` ≈ `r`, if exactly one exists.
fn vertical_cut_face(s: &Solid, r: f64) -> Option<Face> {
    let mut found = None;
    for f in s.shell().faces() {
        if let Some(GeomSurface::Cylinder(c)) = f.surface() {
            let axis = c.position().direction();
            if axis.dot(&Dir::dz()).abs() > 0.999 && (c.radius() - r).abs() < 1e-3 {
                if found.is_some() {
                    return None; // more than one
                }
                found = Some(f);
            }
        }
    }
    found
}

/// Every edge of `face`'s outer wire stays on the cylinder of `radius` about the
/// vertical axis through `axis_xy` — i.e. the cut wall is still a clean cylinder.
fn boundary_on_cut_cylinder(face: &Face, axis_xy: (f64, f64), radius: f64) -> bool {
    let wire = face.outer_wire().expect("cut face has an outer wire");
    for e in wire.edges() {
        let Some(curve) = e.curve() else { return false };
        let (t0, t1) = (e.first(), e.last());
        for k in 0..=8 {
            let t = t0 + (t1 - t0) * (k as f64) / 8.0;
            let p = curve.point(t);
            let d = ((p.x() - axis_xy.0).powi(2) + (p.y() - axis_xy.1).powi(2)).sqrt();
            // The trim curve is a finely-chorded approximation: tolerate sub-mesh
            // chord deviation, but catch the old bug (off-surface blend cap arc was
            // ~0.27 off a Ø8 wall).
            if (d - radius).abs() > 5e-3 {
                return false;
            }
        }
    }
    true
}

fn cracks(s: &Solid) -> usize {
    use openrcad_mesh::tessellate;
    use std::collections::HashMap;
    let mesh = tessellate(s, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
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
    edges.values().filter(|&&c| c == 1).count()
}

/// Test A — the bug: filleting a top edge into the cut must trim flush, leaving
/// the cut a clean vertical cylinder (no sphere, no off-surface distortion).
///
/// Covers both top edges that run into the same corner cut (the y=10 edge whose
/// cut boundary is the curved rim, and the x=10 edge), at a spread of radii.
#[test]
fn fillet_into_cut_trims_flush_and_keeps_cut_clean() {
    let body = cut_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    // Two top edges, each running from a far corner into the (10,10) cut.
    let edges = [
        // y=10, z=10: from x=0 into the cut (cut boundary on this face is the rim).
        Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0)),
        // x=10, z=10: from y=0 into the cut (cut starts at y=6).
        Edge::between_points(Pnt::new(10.0, 0.0, 10.0), Pnt::new(10.0, 6.0, 10.0)),
    ];

    for edge in &edges {
        for r in [1.0_f64, 1.5, 2.5] {
            let s = fillet_edges(&body, std::slice::from_ref(edge), r)
                .unwrap_or_else(|e| panic!("fillet r={r} into cut must succeed: {e:?}"));

            assert!(s.is_watertight(), "r={r}: result must be watertight");
            assert!(
                s.health_report().is_healthy(),
                "r={r}: result must be healthy"
            );
            assert!(
                !has_sphere(&s),
                "r={r}: the cut must NOT be rounded into a sphere"
            );

            // The cut survives as exactly one clean vertical cylinder of radius 4,
            // and every boundary edge of it still lies on that cylinder (not the
            // off-surface blend end-cap the old corner trim spliced in).
            let cut = vertical_cut_face(&s, 4.0)
                .unwrap_or_else(|| panic!("r={r}: exactly one Ø8 cut wall must survive"));
            assert!(
                boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
                "r={r}: the cut wall's boundary must stay on the cut cylinder (clean)"
            );

            // The fillet blend cylinder (radius r) is present.
            let has_blend = s.shell().faces().iter().any(|f| {
                matches!(f.surface(), Some(GeomSurface::Cylinder(c)) if (c.radius() - r).abs() < 1e-3)
            });
            assert!(
                has_blend,
                "r={r}: the fillet blend cylinder must be present"
            );

            assert_eq!(cracks(&s), 0, "r={r}: result must tessellate crack-free");
        }
    }
}

/// Test A2 — the GUI bug: when the corner arc crosses a `make_cylinder` rim seam,
/// the boolean splits the concave wall into two cocylindrical faces. The
/// cocylindrical merge must collapse them back to ONE clean cut face, and
/// filleting the top edge into it must then trim flush.
#[test]
fn seam_crossing_cut_merges_to_one_face_and_fillets_flush() {
    let body = cut_body_seam_crossing();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    // The merge collapsed the split wall: exactly one Ø8 vertical cut face exists.
    assert!(
        vertical_cut_face(&body, 4.0).is_some(),
        "the concave cut must be a single Ø8 cylinder face (cocylindrical merge)"
    );

    for r in [1.0_f64, 1.5, 2.5] {
        let edge = Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0));
        let s = fillet_edges(&body, std::slice::from_ref(&edge), r)
            .unwrap_or_else(|e| panic!("fillet r={r} into default-seam cut must succeed: {e:?}"));

        assert!(s.is_watertight(), "r={r}: result must be watertight");
        assert!(
            s.health_report().is_healthy(),
            "r={r}: result must be healthy"
        );
        assert!(
            !has_sphere(&s),
            "r={r}: the cut must NOT be rounded into a sphere"
        );

        let cut = vertical_cut_face(&s, 4.0)
            .unwrap_or_else(|| panic!("r={r}: exactly one Ø8 cut wall must survive"));
        assert!(
            boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
            "r={r}: the cut wall's boundary must stay on the cut cylinder (clean)"
        );
        assert_eq!(cracks(&s), 0, "r={r}: result must tessellate crack-free");
    }
}

/// Test B — negative control: filleting an edge AWAY from the cut behaves exactly
/// as a normal box-edge fillet, and the cut is left untouched.
#[test]
fn fillet_away_from_cut_is_unaffected() {
    let body = cut_body();
    // Bottom-front edge (y=0, z=0), nowhere near the (10,10) cut.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(10.0, 0.0, 0.0));
    let s = fillet_edges(&body, std::slice::from_ref(&edge), 1.5)
        .expect("an edge away from the cut must fillet normally");
    assert!(s.is_watertight() && s.health_report().is_healthy());
    assert!(!has_sphere(&s));
    // The cut cylinder is untouched: still one clean vertical Ø8 wall.
    let cut = vertical_cut_face(&s, 4.0).expect("cut wall must survive untouched");
    assert!(boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0));
}

/// Test C — oversized radius: the public path must never emit garbage; it either
/// errors cleanly (body left unchanged upstream) or returns a valid solid.
#[test]
fn fillet_into_cut_oversized_radius_is_graceful() {
    let body = cut_body();
    let edge = Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0));
    match fillet_edges(&body, std::slice::from_ref(&edge), 6.0) {
        Err(_) => {} // clean failure -> upstream leaves the body unchanged
        Ok(s) => {
            assert!(
                s.is_watertight() && s.health_report().is_healthy(),
                "an accepted oversize fillet must still be valid"
            );
            assert!(!has_sphere(&s));
        }
    }
}

/// Native chamfer uses the same topology-first endpoint trim as fillet: the
/// bevel plane must end on the cut cylinder with one shared trim edge, leaving
/// the bored wall cylindrical and crack-free.
#[test]
fn chamfer_into_cut_trims_flush_and_keeps_cut_clean() {
    let body = cut_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    let edges = [
        Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0)),
        Edge::between_points(Pnt::new(10.0, 0.0, 10.0), Pnt::new(10.0, 6.0, 10.0)),
    ];

    for edge in &edges {
        for d in [0.75_f64, 1.5, 2.5] {
            let s = chamfer_edges(&body, std::slice::from_ref(edge), d)
                .unwrap_or_else(|e| panic!("chamfer d={d} into cut must succeed: {e:?}"));

            assert!(s.is_watertight(), "d={d}: result must be watertight");
            assert!(
                s.health_report().is_healthy(),
                "d={d}: result must be healthy"
            );

            let cut = vertical_cut_face(&s, 4.0)
                .unwrap_or_else(|| panic!("d={d}: exactly one Ø8 cut wall must survive"));
            assert!(
                boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
                "d={d}: the cut wall's boundary must stay on the cut cylinder"
            );
            assert_eq!(cracks(&s), 0, "d={d}: result must tessellate crack-free");
        }
    }
}
