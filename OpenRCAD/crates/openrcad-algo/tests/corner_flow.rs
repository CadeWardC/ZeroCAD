//! Sequential adjacent-edge fillets must FLOW into each other (GUI round 3):
//! filleting a straight top edge and then the adjacent concave bite arc — or
//! the other order — with the same radius must miter the two bands along their
//! band∩band intersection seam instead of leaving a notch, and must never ship
//! cracked geometry.
//!
//! Geometry of the corner miter (box 30×30×10, quarter-cylinder bite r=10 at
//! the (30,0) corner, fillet radius 3): the straight band is the cylinder of
//! radius 3 about the axis {y=3, z=7}; the circular band is the torus of tube
//! radius 3 about the tube-centre circle {ρ=13 about (30,0), z=7}. Their seam
//! runs from the shared top tangency A=(30−√160, 3, 10) down to the shared
//! wall contact B=(20, 0, 7), and the final body is the same whichever fillet
//! was applied first.

use openrcad_algo::{apply_blend_contour, fillet_edges, BlendContour, BlendCurveHint, BlendKind};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::{GeomCurve, GeomSurface};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Solid};
use std::collections::HashMap;

const R: f64 = 3.0;

fn corner_bitten_box() -> Solid {
    let block = make_box(&Pnt::new(0.0, 0.0, 0.0), 30.0, 30.0, 10.0);
    let cutter = make_cylinder(&Ax2::new(Pnt::new(30.0, 0.0, -1.0), Dir::dz()), 10.0, 12.0);
    openrcad_algo::boolean_checked(&block, &cutter, openrcad_algo::BooleanOp::Cut)
        .expect("corner bite cut should be clean")
}

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

/// The bite's top rim arcs (circle edges of the top cap).
fn top_arcs(solid: &Solid) -> Vec<Edge> {
    let arcs: Vec<Edge> = top_cap(solid)
        .wires()
        .iter()
        .flat_map(|w| w.edges())
        .filter(|e| matches!(e.curve(), Some(GeomCurve::Circle(_))))
        .collect();
    assert!(!arcs.is_empty(), "the bite must leave top rim arcs");
    arcs
}

/// The straight top edge along y = 0 (from the box corner toward the bite; it
/// is shortened when the arc fillet was applied first).
fn straight_top_edge_on_y0(solid: &Solid) -> Edge {
    for f in solid.shell().faces() {
        for w in f.wires() {
            for e in w.edges() {
                let a = e.start().point();
                let b = e.end().point();
                if (a.z() - 10.0).abs() < 1e-6
                    && (b.z() - 10.0).abs() < 1e-6
                    && a.y().abs() < 1e-6
                    && b.y().abs() < 1e-6
                    && matches!(e.curve(), None | Some(GeomCurve::Line(_)))
                {
                    return e;
                }
            }
        }
    }
    panic!("no straight top edge on y=0 found");
}

fn fillet_arc_chain(solid: &Solid) -> Solid {
    let contour = BlendContour::constant(
        top_arcs(solid),
        BlendKind::Fillet,
        R,
        Some(BlendCurveHint::Circle),
    );
    apply_blend_contour(solid, &contour).expect("arc fillet should succeed")
}

fn fillet_straight(solid: &Solid) -> Solid {
    let edge = straight_top_edge_on_y0(solid);
    fillet_edges(solid, &[edge], R).expect("straight fillet should succeed")
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

type Segment = ([f64; 3], [f64; 3]);

fn point_segment_dist(p: [f64; 3], (a, b): &Segment) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if len2 <= 1e-18 {
        0.0
    } else {
        ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0)
    };
    let q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
}

fn mesh_segments_and_cracks(s: &Solid) -> (Vec<Segment>, usize) {
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
    let pos = |i: u32| -> [f64; 3] {
        let b = i as usize * 3;
        [
            gpu.positions[b] as f64,
            gpu.positions[b + 1] as f64,
            gpu.positions[b + 2] as f64,
        ]
    };
    let mut segments = Vec::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
            segments.push((pos(t[a]), pos(t[b])));
        }
    }
    let cracks = edges.values().filter(|&&c| c == 1).count();
    (segments, cracks)
}

/// The miter must (a) carry the analytic band∩band seam — every sample of the
/// exact seam curve appears on the display mesh — and (b) remove the whole
/// corner: nothing may remain near the old sharp corner line (the pre-fix
/// result left a notch of it protruding between the two flush-cut bands).
fn assert_flows_at_corner(name: &str, s: &Solid) {
    let (segments, cracks) = mesh_segments_and_cracks(s);
    assert_eq!(cracks, 0, "{name} display mesh must have no crack edges");
    let min_dist = |p: [f64; 3]| -> f64 {
        segments
            .iter()
            .map(|seg| point_segment_dist(p, seg))
            .fold(f64::INFINITY, f64::min)
    };
    // Analytic seam of the two equal-radius bands, from the shared top tangency
    // A=(30−√160, 3, 10) to the shared wall contact B=(20, 0, 7): at tube angle
    // v ∈ [π/2, π],  z = 7 + 3·sin v,  y = 3 + 3·cos v,
    // x = 30 − √((13 + 3·cos v)² − y²).
    let mut worst = (0.0f64, [0.0f64; 3]);
    for k in 0..=16 {
        let v = std::f64::consts::FRAC_PI_2 * (1.0 + k as f64 / 16.0);
        let y = 3.0 + 3.0 * v.cos();
        let rho = 13.0 + 3.0 * v.cos();
        let x = 30.0 - (rho * rho - y * y).sqrt();
        let z = 7.0 + 3.0 * v.sin();
        let d = min_dist([x, y, z]);
        if d > worst.0 {
            worst = (d, [x, y, z]);
        }
    }
    assert!(
        worst.0 < 0.15,
        "{name}: miter seam sample ({:.3}, {:.3}, {:.3}) missing from the mesh (nearest vertex {:.3} away)",
        worst.1[0],
        worst.1[1],
        worst.1[2],
        worst.0
    );
    // The old sharp corner line (top edge y=0, z=10 between the bands) must be
    // fully consumed — no notch.
    for k in 0..=8 {
        let x = 17.6 + (19.4 - 17.6) * k as f64 / 8.0;
        let d = min_dist([x, 0.0, 10.0]);
        assert!(
            d > 0.3,
            "{name}: material still near the old sharp corner at x={x:.2} (nearest vertex {d:.3})"
        );
    }
}

fn signature(s: &Solid) -> (usize, usize, usize, usize, usize) {
    let tori = s
        .shell()
        .faces()
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Torus(_))))
        .count();
    let cyls = s
        .shell()
        .faces()
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
        .count();
    (s.vertex_count(), s.edge_count(), s.face_count(), tori, cyls)
}

#[test]
fn straight_then_arc_fillets_flow_through_the_corner() {
    let s = corner_bitten_box();
    let f1 = fillet_straight(&s);
    assert_watertight("straight fillet", &f1);
    let f2 = fillet_arc_chain(&f1);
    assert_watertight("straight→arc", &f2);
    assert_flows_at_corner("straight→arc", &f2);
    assert!(
        signature(&f2).3 >= 1,
        "the arc fillet must add a torus band"
    );
}

#[test]
fn arc_then_straight_fillets_flow_through_the_corner() {
    let s = corner_bitten_box();
    let f1 = fillet_arc_chain(&s);
    assert_watertight("arc fillet", &f1);
    let f2 = fillet_straight(&f1);
    assert_watertight("arc→straight", &f2);
    assert_flows_at_corner("arc→straight", &f2);
}

/// The two application orders must converge on the SAME body (the miter seam is
/// the mutual band∩band intersection either way).
#[test]
fn corner_flow_is_order_independent() {
    let s = corner_bitten_box();
    let via_straight_first = fillet_arc_chain(&fillet_straight(&s));
    let via_arc_first = fillet_straight(&fillet_arc_chain(&s));
    assert_eq!(
        signature(&via_straight_first),
        signature(&via_arc_first),
        "both orders must produce the same topology"
    );
    // Same geometry too: identical tessellation bounds and total area within
    // chording noise.
    let bounds_area = |s: &Solid| {
        let mesh = openrcad_mesh::tessellate(s, 0.05, 0.5);
        let gpu = mesh.gpu_mesh();
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in gpu.positions.chunks_exact(3) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k] as f64);
                hi[k] = hi[k].max(p[k] as f64);
            }
        }
        let mut area = 0.0f64;
        for t in gpu.indices.chunks_exact(3) {
            let g = |i: u32| {
                let b = i as usize * 3;
                [
                    gpu.positions[b] as f64,
                    gpu.positions[b + 1] as f64,
                    gpu.positions[b + 2] as f64,
                ]
            };
            let (a, b, c) = (g(t[0]), g(t[1]), g(t[2]));
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let x = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            area += 0.5 * (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
        }
        (lo, hi, area)
    };
    let (lo1, hi1, area1) = bounds_area(&via_straight_first);
    let (lo2, hi2, area2) = bounds_area(&via_arc_first);
    for k in 0..3 {
        assert!((lo1[k] - lo2[k]).abs() < 1e-6, "bounds must match");
        assert!((hi1[k] - hi2[k]).abs() < 1e-6, "bounds must match");
    }
    assert!(
        (area1 - area2).abs() < 0.01 * area1,
        "surface areas must match: {area1} vs {area2}"
    );
}

/// Chamfers do not miter (only fillets flow); the sequential chamfer must still
/// fail SAFELY or terminate flush — never ship broken geometry.
#[test]
fn sequential_chamfer_after_straight_fillet_degrades_safely() {
    let s = corner_bitten_box();
    let f1 = fillet_straight(&s);
    let contour = BlendContour::constant(
        top_arcs(&f1),
        BlendKind::Chamfer,
        R,
        Some(BlendCurveHint::Circle),
    );
    // A safe decline (`Err`) is acceptable; success must be watertight.
    if let Ok(out) = apply_blend_contour(&f1, &contour) {
        assert_watertight("chamfer after straight fillet", &out);
    }
}
