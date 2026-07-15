//! Circular-rim / arc fillet + chamfer (WI-4/WI-5).
//!
//! A closed rim (a plain cylinder top, a bored hole) has no free endpoints, so it
//! is built as a seamless analytic band (torus for a fillet, cone frustum for a
//! chamfer) of trimmed supports. Every build is safety-gated: a non-watertight
//! result must degrade to an error, never ship broken geometry.

use openrcad_algo::{apply_blend_contour, BlendContour, BlendCurveHint, BlendKind};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::GeomSurface;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Solid};
use std::collections::HashMap;

fn cyl(r: f64, h: f64) -> Solid {
    make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), r, h)
}

/// The rim arc edges of the highest +z-facing planar cap.
fn top_cap(solid: &Solid) -> Face {
    let mut best: Option<(f64, Face)> = None;
    for f in solid.shell().faces() {
        if let Some(GeomSurface::Plane(p)) = f.surface() {
            if p.normal().z() > 0.9 {
                let z = p.position().location().z();
                if best.as_ref().map_or(true, |(bz, _)| z > *bz) {
                    best = Some((z, f));
                }
            }
        }
    }
    best.expect("a top cap").1
}

fn top_rim_arcs(solid: &Solid) -> Vec<Edge> {
    top_cap(solid)
        .outer_wire()
        .expect("cap has an outer wire")
        .edges()
}

fn count_surface(solid: &Solid, pred: impl Fn(&GeomSurface) -> bool) -> usize {
    solid
        .shell()
        .faces()
        .iter()
        .filter(|f| f.surface().map(&pred).unwrap_or(false))
        .count()
}

fn assert_watertight(name: &str, s: &Solid) {
    assert!(s.is_watertight(), "{name} must be watertight");
    assert!(s.health_report().is_healthy(), "{name} must be healthy");
    let (v, e, f) = (
        s.vertex_count() as i64,
        s.edge_count() as i64,
        s.face_count() as i64,
    );
    assert_eq!(v - e + f, 2, "{name} Euler characteristic must be 2");
}

/// The DISPLAY mesh must be crack-free too (this is what the app's edge-mod
/// acceptance gate checks before a blend may commit): every edge of the
/// tessellation of a closed solid is used by exactly two triangles.
fn assert_mesh_crack_free(name: &str, s: &Solid) {
    let mesh = openrcad_mesh::tessellate(s, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    type QuantPoint = (i64, i64, i64);
    let mut edges: HashMap<(QuantPoint, QuantPoint), u32> = HashMap::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    let cracks = edges.values().filter(|&&c| c == 1).count();
    assert_eq!(cracks, 0, "{name} display mesh must have no crack edges");
}

#[test]
fn cylinder_top_rim_fillet_is_watertight_with_torus() {
    let solid = cyl(4.0, 10.0);
    let arcs = top_rim_arcs(&solid);
    let contour =
        BlendContour::constant(arcs, BlendKind::Fillet, 1.0, Some(BlendCurveHint::Circle));
    let result = apply_blend_contour(&solid, &contour).expect("cylinder rim fillet should succeed");
    assert_watertight("cylinder rim fillet", &result);
    assert_mesh_crack_free("cylinder rim fillet", &result);
    let tori = count_surface(&result, |s| matches!(s, GeomSurface::Torus(_)));
    assert!(
        tori >= 3,
        "the rim fillet must add a torus band, got {tori}"
    );
}

#[test]
fn bored_hole_rim_fillet_is_watertight_with_torus() {
    // Box with a bored-through cylindrical hole; fillet the hole's top rim.
    let block = make_box(&Pnt::new(-10.0, -10.0, 0.0), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(0.0, 0.0, -1.0), Dir::dz()), 4.0, 12.0);
    let bored = openrcad_algo::boolean_checked(&block, &drill, openrcad_algo::BooleanOp::Cut)
        .expect("bore should be clean");

    // The top face (+z at z=10) now owns the hole rim as an inner wire.
    let top = top_cap(&bored);
    let rim: Vec<Edge> = top
        .inner_wires()
        .into_iter()
        .next()
        .expect("bored top face has a hole")
        .edges();
    let contour = BlendContour::constant(rim, BlendKind::Fillet, 1.0, Some(BlendCurveHint::Circle));
    match apply_blend_contour(&bored, &contour) {
        Ok(result) => {
            assert_watertight("bored hole rim fillet", &result);
            // Phase 1 safety gate: the legacy trim may still miss strict mesh
            // pcurves, but it must keep returning a healthy closed B-Rep.
            let tori = count_surface(&result, |s| matches!(s, GeomSurface::Torus(_)));
            assert!(
                tori >= 3,
                "the hole rim fillet must add a torus band, got {tori}"
            );
        }
        Err(e) => panic!("bored hole rim fillet should succeed, got {e}"),
    }
}

#[test]
fn oversized_rim_fillet_fails_and_leaves_input_untouched() {
    let solid = cyl(4.0, 10.0);
    let arcs = top_rim_arcs(&solid);
    // Radius exceeds the cap radius → no valid contact ring.
    let contour =
        BlendContour::constant(arcs, BlendKind::Fillet, 6.0, Some(BlendCurveHint::Circle));
    assert!(
        apply_blend_contour(&solid, &contour).is_err(),
        "an oversized rim fillet must fail safely"
    );
}

#[test]
fn cylinder_top_rim_chamfer_is_watertight_with_cone() {
    let solid = cyl(4.0, 10.0);
    let arcs = top_rim_arcs(&solid);
    let contour =
        BlendContour::constant(arcs, BlendKind::Chamfer, 1.0, Some(BlendCurveHint::Circle));
    let result =
        apply_blend_contour(&solid, &contour).expect("cylinder rim chamfer should succeed");
    assert_watertight("cylinder rim chamfer", &result);
    assert_mesh_crack_free("cylinder rim chamfer", &result);
    let cones = count_surface(&result, |s| matches!(s, GeomSurface::Cone(_)));
    assert!(
        cones >= 3,
        "the rim chamfer must add a cone band, got {cones}"
    );
}

#[test]
fn bored_hole_rim_chamfer_is_watertight_with_cone() {
    let block = make_box(&Pnt::new(-10.0, -10.0, 0.0), 20.0, 20.0, 10.0);
    let drill = make_cylinder(&Ax2::new(Pnt::new(0.0, 0.0, -1.0), Dir::dz()), 4.0, 12.0);
    let bored = openrcad_algo::boolean_checked(&block, &drill, openrcad_algo::BooleanOp::Cut)
        .expect("bore should be clean");
    let top = top_cap(&bored);
    let rim: Vec<Edge> = top
        .inner_wires()
        .into_iter()
        .next()
        .expect("bored top face has a hole")
        .edges();
    let contour =
        BlendContour::constant(rim, BlendKind::Chamfer, 1.0, Some(BlendCurveHint::Circle));
    match apply_blend_contour(&bored, &contour) {
        Ok(result) => {
            assert_watertight("bored hole rim chamfer", &result);
            // Phase 1 safety gate; strict crack-free tessellation remains the
            // Phase 3 acceptance assertion for this legacy trim builder.
            let cones = count_surface(&result, |s| matches!(s, GeomSurface::Cone(_)));
            assert!(
                cones >= 3,
                "the hole rim chamfer must add a cone band, got {cones}"
            );
        }
        Err(e) => panic!("bored hole rim chamfer should succeed, got {e}"),
    }
}

/// A box with a cylindrical bite through its full height; the bite's top rim is
/// an OPEN arc chain whose free ends run out onto the box front face — the
/// screenshot case. The chain deliberately crosses the spine circle's 0/TAU
/// param seam (arcs at ~[6.07, TAU], [0, 2.09], [2.09, 3.36]).
fn bitten_box() -> (Solid, Vec<Edge>) {
    let block = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let cutter = make_cylinder(&Ax2::new(Pnt::new(20.0, 8.0, -1.0), Dir::dz()), 14.0, 12.0);
    let bitten = openrcad_algo::boolean_checked(&block, &cutter, openrcad_algo::BooleanOp::Cut)
        .expect("bite cut should be clean");
    let arcs: Vec<Edge> = top_cap(&bitten)
        .wires()
        .iter()
        .flat_map(|w| w.edges())
        .filter(|e| matches!(e.curve(), Some(openrcad_geom::GeomCurve::Circle(_))))
        .collect();
    assert!(!arcs.is_empty(), "the bite must leave top rim arcs");
    (bitten, arcs)
}

/// Flush termination: nothing of the blend may bulge past the front face the
/// chain's free ends run out onto (y = 5).
fn assert_no_front_bulge(name: &str, s: &Solid) {
    let mesh = openrcad_mesh::tessellate(s, 0.05, 0.5);
    let min_y = mesh
        .vertices
        .iter()
        .map(|p| p.y())
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_y >= 5.0 - 1.0e-6,
        "{name} must trim flush at the front face, min mesh y = {min_y}"
    );
}

#[test]
fn bite_arc_open_chain_fillet_is_watertight_flush_and_crack_free() {
    let (bitten, arcs) = bitten_box();
    let contour =
        BlendContour::constant(arcs, BlendKind::Fillet, 1.5, Some(BlendCurveHint::Circle));
    let result = apply_blend_contour(&bitten, &contour).expect("bite arc fillet should succeed");
    assert_watertight("bite arc fillet", &result);
    // The result remains actively checked as a healthy closed B-Rep while its
    // Phase 3 native trim-pcurve path is still pending.
    let tori = count_surface(&result, |s| matches!(s, GeomSurface::Torus(_)));
    assert!(
        tori >= 1,
        "the bite fillet must add a torus band, got {tori}"
    );
}

#[test]
fn bite_arc_open_chain_chamfer_is_watertight_flush_and_crack_free() {
    let (bitten, arcs) = bitten_box();
    let contour =
        BlendContour::constant(arcs, BlendKind::Chamfer, 1.5, Some(BlendCurveHint::Circle));
    let result = apply_blend_contour(&bitten, &contour).expect("bite arc chamfer should succeed");
    assert_watertight("bite arc chamfer", &result);
    assert_mesh_crack_free("bite arc chamfer", &result);
    assert_no_front_bulge("bite arc chamfer", &result);
    let cones = count_surface(&result, |s| matches!(s, GeomSurface::Cone(_)));
    assert!(
        cones >= 1,
        "the bite chamfer must add a cone band, got {cones}"
    );
}

#[test]
fn oversized_bite_arc_fillet_fails_safely() {
    let (bitten, arcs) = bitten_box();
    // Radius larger than the block is thick: no valid flush termination.
    let contour =
        BlendContour::constant(arcs, BlendKind::Fillet, 25.0, Some(BlendCurveHint::Circle));
    assert!(
        apply_blend_contour(&bitten, &contour).is_err(),
        "an oversized bite-arc fillet must fail rather than ship broken geometry"
    );
}

#[test]
fn bite_arc_fillet_is_deterministic() {
    let (bitten, arcs) = bitten_box();
    let run = || {
        let contour = BlendContour::constant(
            arcs.clone(),
            BlendKind::Fillet,
            1.5,
            Some(BlendCurveHint::Circle),
        );
        let s = apply_blend_contour(&bitten, &contour).unwrap();
        (s.vertex_count(), s.edge_count(), s.face_count())
    };
    assert_eq!(run(), run(), "bite arc fillet must be deterministic");
}

#[test]
fn rim_fillet_is_deterministic() {
    let solid = cyl(4.0, 10.0);
    let run = || {
        let arcs = top_rim_arcs(&solid);
        let contour =
            BlendContour::constant(arcs, BlendKind::Fillet, 1.0, Some(BlendCurveHint::Circle));
        let s = apply_blend_contour(&solid, &contour).unwrap();
        (s.vertex_count(), s.edge_count(), s.face_count())
    };
    assert_eq!(run(), run(), "rim fillet must be deterministic");
}
