//! Shared construction + sanity helpers for the break-test suite.
//!
//! The philosophy of this crate: build parts the way the GUI would, then push
//! every feature through both realistic and adversarial inputs. The universal
//! invariants under test are:
//!   1. evaluation never panics,
//!   2. any produced mesh geometry is finite (no NaN/inf vertices),
//!   3. evaluation is deterministic (same graph, same volumes, bit for bit).
//!
//! Individual tests additionally assert documented graceful-failure behavior
//! (structured errors / unresolved-feature warnings) where the evaluator
//! promises it.

#![allow(dead_code)]

use std::collections::HashSet;

use zerocad_core::{
    AxisBase, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, HoleKind, MockMesh,
    ParametricGraph, SketchCurves, Vec3,
};

pub fn add_box(g: &mut ParametricGraph, id: &str, w: f32, h: f32, d: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Box { w, h, d },
    });
}

pub fn add_cylinder(g: &mut ParametricGraph, id: &str, r: f32, h: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Cylinder { r, h },
    });
}

pub fn add_sketch(g: &mut ParametricGraph, id: &str, curves: SketchCurves) {
    add_sketch_cs(g, id, CoordinateSystem::XY, curves);
}

pub fn add_sketch_cs(
    g: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    curves: SketchCurves,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
}

pub fn add_extrude(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
) {
    add_extrude_full(g, id, sketch_id, depth, mode, None, 0.0);
}

pub fn add_extrude_full(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
    target: Option<String>,
    draft_angle_deg: f32,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            target,
            depth_expr: None,
            draft_angle_deg,
            draft_angle_expr: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

pub fn add_hole(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    position: [f32; 3],
    direction: [f32; 3],
    diameter: f32,
    depth: Option<f32>,
    kind: HoleKind,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Hole {
            target: target.to_string(),
            position,
            direction,
            diameter,
            diameter_expr: None,
            depth,
            kind,
            standard: None,
            manufacturing: None,
        },
    });
    g.add_dependency(target, id);
}

pub fn rect_sketch(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle(min, max);
    c
}

pub fn circle_sketch(center: (f32, f32), radius: f32) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_circle(center, radius);
    c
}

/// Evaluate and return bodies, converting a hard error into a panic with
/// context (a panic in these tests IS the failure signal we want recorded).
pub fn eval(g: &ParametricGraph) -> Vec<(String, MockMesh)> {
    match g.evaluate_bodies_with_warnings(&HashSet::new()) {
        Ok((bodies, _warnings)) => bodies,
        Err(e) => panic!("evaluation returned hard error: {e}"),
    }
}

/// Evaluate and require a hard, structured error (documented graceful failure).
pub fn eval_err(g: &ParametricGraph) -> String {
    match g.evaluate_bodies_with_warnings(&HashSet::new()) {
        Ok((bodies, _)) => panic!(
            "expected a structured error, got {} bodies: {:?}",
            bodies.len(),
            bodies.iter().map(|(id, _)| id).collect::<Vec<_>>()
        ),
        Err(e) => e,
    }
}

/// Assert every vertex and index of every produced mesh is finite and in
/// range. NaN/inf leaking into geometry is the classic silent breakage mode.
pub fn assert_meshes_finite(bodies: &[(String, MockMesh)]) {
    for (id, mesh) in bodies {
        for v in &mesh.vertices {
            assert!(
                v.is_finite(),
                "body {id} has a non-finite vertex coordinate"
            );
        }
        let vert_count = (mesh.vertices.len() / 6) as u32;
        assert!(vert_count > 0, "body {id} produced no vertices");
        for &i in &mesh.indices {
            assert!(i < vert_count, "body {id} index {i} out of range");
        }
        for tri in mesh.indices.chunks_exact(3) {
            assert!(
                tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2],
                "body {id} has a degenerate (repeated-index) triangle"
            );
        }
    }
}

pub fn total_volume(bodies: &[(String, MockMesh)]) -> f64 {
    bodies
        .iter()
        .map(|(id, m)| {
            m.mass_properties()
                .unwrap_or_else(|| panic!("body {id} has no solid mass properties"))
                .volume
        })
        .sum()
}

pub fn assert_parameter_invalid(g: &ParametricGraph, id: &str) {
    use zerocad_core::{EvaluationCancellation, EvaluationQuality};
    let token =
        EvaluationCancellation::new(0, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)));
    let output = g
        .evaluate_request(&HashSet::new(), EvaluationQuality::Final, &token)
        .unwrap();
    assert!(
        output.diagnostics.iter().any(|d| d.feature_id == id
            && d.code == zerocad_core::parametric::DiagnosticCode::parameter_invalid()),
        "missing typed parameter diagnostic for {id}: {:?}",
        output.diagnostics
    );
    assert!(output
        .statuses
        .iter()
        .any(|s| s.feature_id.as_str() == id && s.is_unresolved()));
}

/// Evaluate twice and require bit-identical total volume. Non-determinism in
/// the evaluator (HashMap ordering, parallel races) breaks file reload and undo.
pub fn assert_deterministic(g: &ParametricGraph) -> f64 {
    let a = total_volume(&eval(g));
    let b = total_volume(&eval(&g.clone_document()));
    assert_eq!(a.to_bits(), b.to_bits(), "evaluation is not deterministic");
    a
}

/// The standard "did it break?" gate: evaluate, sanity-check geometry,
/// check determinism, and hand the total volume back for assertions.
pub fn assert_part_sane(g: &ParametricGraph) -> f64 {
    let bodies = eval(g);
    assert_meshes_finite(&bodies);
    assert_deterministic(g);
    total_volume(&bodies)
}

/// Sweep offsets across exact tangency (shared-face / shared-edge booleans are
/// the historically fragile zone). Every step must stay finite and
/// deterministic; the closure records failures with the offset that caused
/// them so the panic message pinpoints the break.
pub fn tangency_sweep(offsets: &[f32], mut build: impl FnMut(f32) -> ParametricGraph) {
    for &off in offsets {
        let g = build(off);
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
        let v1 = total_volume(&bodies);
        let v2 = total_volume(&eval(&g));
        assert_eq!(
            v1.to_bits(),
            v2.to_bits(),
            "non-deterministic evaluation at offset {off}"
        );
    }
}

pub fn xy_plane_at(z: f32) -> CoordinateSystem {
    CoordinateSystem::new(Vec3::new(0.0, 0.0, z), Vec3::X, Vec3::Y)
}

pub fn z_axis() -> AxisBase {
    AxisBase::Z
}

// ---------------------------------------------------------------------------
// Campaign helpers (adversarial-testing plan §3): the shared assertions are
// deliberately stricter than the universal gates above. `allowed_error =
// abs + rel * |reference|` is the declared tolerance policy; occupancy is a
// ray-parity oracle so material can be checked away from volume alone.
// ---------------------------------------------------------------------------

/// Plan §3 tolerance policy: `allowed = abs_tol + rel_tol * |reference|`.
pub fn close(actual: f64, reference: f64, abs_tol: f64, rel_tol: f64) -> bool {
    let allowed = abs_tol + rel_tol * reference.abs();
    (actual - reference).abs() <= allowed
}

pub fn assert_close(actual: f64, reference: f64, abs_tol: f64, rel_tol: f64, what: &str) {
    assert!(
        close(actual, reference, abs_tol, rel_tol),
        "{what}: got {actual}, reference {reference} (abs {abs_tol}, rel {rel_tol})"
    );
}

/// Volume of one named body — plan §3: check each expected body individually;
/// equal totals can hide a missing body and an enlarged survivor.
pub fn body_volume(bodies: &[(String, MockMesh)], id: &str) -> f64 {
    let mesh = &bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .unwrap_or_else(|| {
            panic!(
                "body '{id}' missing among [{}]",
                bodies
                    .iter()
                    .map(|(b, _)| b.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
        .1;
    mesh.mass_properties()
        .unwrap_or_else(|| panic!("body '{id}' has no solid mass properties"))
        .volume
}

pub fn body_ids(bodies: &[(String, MockMesh)]) -> Vec<String> {
    bodies.iter().map(|(id, _)| id.clone()).collect()
}

/// Strict mesh gate: everything in `assert_meshes_finite`, plus zero-area
/// triangles with three distinct indices (plan §3: degeneracy is not only
/// repeated indices), finite normals, and positive oriented volume.
pub fn assert_meshes_valid(bodies: &[(String, MockMesh)]) {
    assert_meshes_finite(bodies);
    for (id, mesh) in bodies {
        let pos = |i: usize| -> [f64; 3] {
            [
                f64::from(mesh.vertices[i * 6]),
                f64::from(mesh.vertices[i * 6 + 1]),
                f64::from(mesh.vertices[i * 6 + 2]),
            ]
        };
        // Scale reference for the zero-area threshold: the body diagonal, so
        // legitimately fine tessellation of small geometry is not flagged.
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for v in (0..mesh.vertices.len() / 6).map(pos) {
            for a in 0..3 {
                lo[a] = lo[a].min(v[a]);
                hi[a] = hi[a].max(v[a]);
            }
        }
        let diag2: f64 = (0..3)
            .map(|a| (hi[a] - lo[a]).powi(2))
            .sum::<f64>()
            .max(1e-30);
        for n in (3..mesh.vertices.len()).step_by(6) {
            assert!(
                mesh.vertices[n].is_finite()
                    && mesh.vertices[n + 1].is_finite()
                    && mesh.vertices[n + 2].is_finite(),
                "body {id} has a non-finite normal"
            );
        }
        for tri in mesh.indices.chunks_exact(3) {
            let (a, b, c) = (
                pos(tri[0] as usize),
                pos(tri[1] as usize),
                pos(tri[2] as usize),
            );
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let cross = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let area2 = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
            assert!(
                area2 > 1e-14 * diag2,
                "body {id} has a zero-area triangle with distinct indices {tri:?}"
            );
        }
        if let Some(mp) = mesh.mass_properties() {
            assert!(
                mp.volume > 0.0,
                "body {id} has non-positive volume {} (inverted orientation?)",
                mp.volume
            );
        }
    }
}

/// Material occupancy at a point, by ray-parity against the triangle soup.
/// The ray direction is deliberately skewed off all axes to avoid edge and
/// boundary degeneracies; callers must sample away from surfaces.
pub fn occupancy(mesh: &MockMesh, p: [f64; 3]) -> bool {
    let dir = {
        let mut d: [f64; 3] = [0.577_215_7, 0.573_842_1, 0.582_937_7];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        d = [d[0] / n, d[1] / n, d[2] / n];
        d
    };
    let pos = |i: usize| -> [f64; 3] {
        [
            f64::from(mesh.vertices[i * 6]),
            f64::from(mesh.vertices[i * 6 + 1]),
            f64::from(mesh.vertices[i * 6 + 2]),
        ]
    };
    let mut crossings = 0u64;
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (
            pos(tri[0] as usize),
            pos(tri[1] as usize),
            pos(tri[2] as usize),
        );
        // Möller–Trumbore against ray p + t*dir, counting forward hits.
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let h = [
            dir[1] * e2[2] - dir[2] * e2[1],
            dir[2] * e2[0] - dir[0] * e2[2],
            dir[0] * e2[1] - dir[1] * e2[0],
        ];
        let det = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2];
        if det.abs() < 1e-14 {
            continue;
        }
        let inv = 1.0 / det;
        let s = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let u = inv * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = [
            s[1] * e1[2] - s[2] * e1[1],
            s[2] * e1[0] - s[0] * e1[2],
            s[0] * e1[1] - s[1] * e1[0],
        ];
        let v = inv * (dir[0] * q[0] + dir[1] * q[1] + dir[2] * q[2]);
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        let t = inv * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]);
        if t > 1e-9 {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

/// Assert occupancy at deliberately chosen interior and exterior points.
pub fn assert_occupancy(mesh: &MockMesh, inside: &[[f64; 3]], outside: &[[f64; 3]]) {
    for &p in inside {
        assert!(occupancy(mesh, p), "point {p:?} should be inside material");
    }
    for &p in outside {
        assert!(
            !occupancy(mesh, p),
            "point {p:?} should be outside material"
        );
    }
}

/// Convenience feature add with dependencies (the per-file helpers
/// consolidated for the catalog suites).
pub fn add_feature(g: &mut ParametricGraph, id: &str, feature: FeatureType, deps: &[&str]) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature,
    });
    for d in deps {
        g.add_dependency(d, id);
    }
}

/// The §3 "did it break?" gate for the catalog suites: strict mesh validity,
/// determinism, and the total volume.
pub fn assert_catalog_sane(g: &ParametricGraph) -> f64 {
    let bodies = eval(g);
    assert_meshes_valid(&bodies);
    assert_deterministic(g);
    total_volume(&bodies)
}

/// Capture a face reference from an evaluated mesh the way the GUI does:
/// durable topology names attached, centroid/normal kept for fallback.
pub fn capture_face(
    g: &ParametricGraph,
    body_id: &str,
    want_normal: [f32; 3],
) -> zerocad_core::parametric::FaceRef {
    use zerocad_core::parametric::TopologyFaceRef;
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies
        .iter()
        .find(|(id, _)| id == body_id)
        .unwrap_or_else(|| panic!("body {body_id} missing"))
        .1;
    let f = mesh
        .face_refs
        .iter()
        .find(|f| {
            (f.normal[0] - want_normal[0]).abs() < 1e-3
                && (f.normal[1] - want_normal[1]).abs() < 1e-3
                && (f.normal[2] - want_normal[2]).abs() < 1e-3
        })
        .unwrap_or_else(|| {
            panic!(
                "no face with normal {want_normal:?} among {} faces of {body_id}",
                mesh.face_refs.len()
            )
        });
    zerocad_core::parametric::FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    }
}
