//! The 3D edge fillet/chamfer tool: a live (uncommitted) edge modification with
//! a real-time preview of the rounded/beveled body and a Fusion-style floating
//! inline size box, committed into the parametric graph on confirm.
//!
//! It mirrors the extrude tool's shape ([`crate::extrude`]): an in-progress op
//! drives an immediate lightweight mesh, an exact worker-computed preview, and an
//! inline dialog ([`ZeroCadApp::show_edge_mod_dialog`]) until the user commits.

use eframe::egui;
use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{AllEdgeSelector, CornerKind, EdgeRef, FeatureNode, FeatureType, MockMesh};

use crate::{PendingCommitVisual, PendingVisualMode, SharedBodyMeshes, ZeroCadApp};

const EDGE_MOD_PREVIEW_FILLET_SEGS: usize = 8;
/// How many recently-solved sizes to keep so scrubbing back to one is instant.
const EDGE_MOD_ARC_LRU_CAP: usize = 8;

fn v_add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn v_scale(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn v_dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn v_cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn v_len(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn v_norm(v: [f32; 3]) -> Option<[f32; 3]> {
    let len = v_len(v);
    (len > 1.0e-6).then(|| v_scale(v, 1.0 / len))
}

fn push_preview_vertex(mesh: &mut MockMesh, p: [f32; 3], n: [f32; 3]) {
    mesh.vertices
        .extend_from_slice(&[p[0], p[1], p[2], n[0], n[1], n[2]]);
}

fn push_preview_edge(mesh: &mut MockMesh, a: [f32; 3], b: [f32; 3]) {
    let i = (mesh.edge_vertices.len() / 3) as u32;
    mesh.edge_vertices
        .extend_from_slice(&[a[0], a[1], a[2], b[0], b[1], b[2]]);
    mesh.edge_indices.extend_from_slice(&[i, i + 1]);
}

fn edge_mod_edge_preview_mesh(
    edge: &EdgeRef,
    dist: f32,
    kind: CornerKind,
    concave: bool,
) -> Option<MockMesh> {
    if matches!(edge.curve, Some(EdgeCurveHint::Circle { .. })) {
        return edge_mod_circular_edge_preview_mesh(edge, dist, kind);
    }
    if !matches!(edge.curve, None | Some(EdgeCurveHint::Line)) {
        return None;
    }

    let n1 = v_norm(edge.n1)?;
    let n2 = v_norm(edge.n2)?;
    let dist = dist.max(0.05);
    // A convex blend carves the corner off, so its band sits on the INWARD side
    // of both faces (offsets along −n). A concave (inner-corner) blend ADDS a
    // wedge in the void, so the band sits on the OUTWARD side (offsets along +n)
    // and the fillet arc faces the corner — the exact mirror.
    let s = if concave { 1.0 } else { -1.0 };
    // cos of the angle between the face normals; the wedge trig below must use
    // the real angle — the old ±n·dist offsets were the 90°-only special case,
    // which floated the ribbon off the body at sharp/blunt corners.
    let c = v_dot(n1, n2);
    let mut rails: Vec<([f32; 3], [f32; 3])> = Vec::new();
    match kind {
        CornerKind::Chamfer => {
            // Feet sit ON each face at `dist` along it from the edge: the
            // in-face, edge-perpendicular direction of face i is the other
            // face's normal projected onto face i, (−n_j + n_i·c)/sinθ.
            let sin_theta = (1.0 - c * c).max(1.0e-6).sqrt();
            let t1 = v_scale(v_add(v_scale(n2, -1.0), v_scale(n1, c)), 1.0 / sin_theta);
            let t2 = v_scale(v_add(v_scale(n1, -1.0), v_scale(n2, c)), 1.0 / sin_theta);
            let normal = v_norm(v_scale(v_add(n1, n2), s)).unwrap_or(n1);
            rails.push((v_scale(t1, -s * dist), normal));
            rails.push((v_scale(t2, -s * dist), normal));
        }
        CornerKind::Fillet => {
            // Rolling-ball center: r/sin(θ/2) along the bisector, which is
            // (n1+n2)·r/(1 + n1·n2) — reduces to (n1+n2)·r at 90°.
            let center_offset = v_scale(v_add(n1, n2), s * dist / (1.0 + c).max(1.0e-3));
            for i in 0..=EDGE_MOD_PREVIEW_FILLET_SEGS {
                let theta =
                    i as f32 / EDGE_MOD_PREVIEW_FILLET_SEGS as f32 * std::f32::consts::FRAC_PI_2;
                // Sweep the profile from face to face; on the concave side the arc
                // bows back toward the corner (dir negated), so the ribbon hugs
                // the added material instead of the removed wedge. Any unit blend
                // of the two normals lies on the profile circle, and its ends
                // (±n1, ±n2) land exactly on the tangency feet for every wedge
                // angle, so only the center needed the general-angle fix.
                let dir = v_norm(v_add(v_scale(n2, theta.cos()), v_scale(n1, theta.sin())))?;
                let dir = v_scale(dir, -s);
                rails.push((v_add(center_offset, v_scale(dir, dist)), dir));
            }
        }
    }
    if rails.len() < 2 {
        return None;
    }

    let mut mesh = MockMesh::empty();
    for (offset, normal) in &rails {
        push_preview_vertex(&mut mesh, v_add(edge.p0, *offset), *normal);
        push_preview_vertex(&mut mesh, v_add(edge.p1, *offset), *normal);
    }
    for i in 0..rails.len() - 1 {
        let a0 = (i * 2) as u32;
        let a1 = a0 + 1;
        let b0 = a0 + 2;
        let b1 = a0 + 3;
        mesh.indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
        mesh.face_ids.push(i as u32 + 1);
        mesh.face_ids.push(i as u32 + 1);
    }

    let rail_edges: Vec<[f32; 3]> = match kind {
        CornerKind::Chamfer => rails.iter().map(|(offset, _)| *offset).collect(),
        CornerKind::Fillet => rails
            .first()
            .zip(rails.last())
            .map(|(first, last)| vec![first.0, last.0])
            .unwrap_or_default(),
    };
    for offset in rail_edges {
        push_preview_edge(&mut mesh, v_add(edge.p0, offset), v_add(edge.p1, offset));
    }
    for window in rails.windows(2) {
        push_preview_edge(
            &mut mesh,
            v_add(edge.p0, window[0].0),
            v_add(edge.p0, window[1].0),
        );
        push_preview_edge(
            &mut mesh,
            v_add(edge.p1, window[0].0),
            v_add(edge.p1, window[1].0),
        );
    }

    Some(mesh)
}

/// Circular analogue of [`edge_mod_edge_preview_mesh`]: sweep the same blend
/// profile (a quarter-arc of rails for a fillet, two rails for a chamfer)
/// AROUND the selected rim arc instead of along a straight edge, so a bite arc
/// or a cylinder/bored-hole rim gets the same immediate visual feedback while
/// the size is being edited.
fn edge_mod_circular_edge_preview_mesh(
    edge: &EdgeRef,
    dist: f32,
    kind: CornerKind,
) -> Option<MockMesh> {
    let Some(EdgeCurveHint::Circle {
        center,
        axis,
        x_dir,
        radius,
        start,
        end,
        closed,
    }) = edge.curve.clone()
    else {
        return None;
    };
    let tau = std::f32::consts::TAU;
    let a = v_norm(axis)?;
    let x = v_norm(v_add(x_dir, v_scale(a, -v_dot(x_dir, a))))?;
    let y = v_cross(a, x);
    // The hint's arc may run in either direction (`end` below `start`).
    let raw = end - start;
    let span = if closed {
        tau
    } else {
        let s = raw.abs().rem_euclid(tau);
        if s <= 1.0e-3 {
            tau
        } else {
            s
        }
    };
    let dir_sign = if raw < 0.0 { -1.0 } else { 1.0 };
    let dist = dist.max(0.05);
    // The mesh edge's two adjacent-face normals arrive in arbitrary
    // tessellation order — which one is the CAP and which the WALL must be
    // read off the geometry: the cap normal is the more axial of the two.
    // Trusting the order (the old behavior) swept every profile around one
    // point's radial direction whenever the pair arrived swapped, collapsing
    // the ribbon into a jagged flat ring around the rim.
    let na = v_norm(edge.n1)?;
    let nb = v_norm(edge.n2)?;
    let (n1, n2) = if v_dot(na, a).abs() >= v_dot(nb, a).abs() {
        (na, nb)
    } else {
        (nb, na)
    };

    let radial_at =
        |theta: f32| -> [f32; 3] { v_add(v_scale(x, theta.cos()), v_scale(y, theta.sin())) };
    // `n2` is the wall normal captured at one (unknown) point of the arc:
    // its radial sign is read where it aligns best with the local radial
    // direction (that's the capture angle), so a concave bite wall (normal
    // toward the axis) and a convex rim (normal outward) both offset inward
    // into their own material.
    let segs = ((span * radius / 1.5) as usize).clamp(8, 96);
    // Radial direction at the edge's own p0/p1 chord (projected off the axis).
    // For a closed rim that chord is where the adjacent-face normals were
    // captured, so the wall's inward/outward sign is read THERE. The |max|
    // alignment scan below is sign-ambiguous on a full circle (±radial both
    // peak at 1 somewhere — a boss wall and a hole wall differ only in WHERE),
    // which made convex rims flip outward on a numeric coin toss.
    let chord_radial = {
        let mid = [
            (edge.p0[0] + edge.p1[0]) * 0.5 - center[0],
            (edge.p0[1] + edge.p1[1]) * 0.5 - center[1],
            (edge.p0[2] + edge.p1[2]) * 0.5 - center[2],
        ];
        v_norm(v_add(mid, v_scale(a, -v_dot(mid, a))))
    };
    let wall_sign = match chord_radial {
        Some(rp) if closed => {
            if v_dot(n2, rp) >= 0.0 {
                1.0
            } else {
                -1.0
            }
        }
        _ => {
            // Open arc: the span-limited scan is meaningful (the sampled range
            // may only cover one lobe), so keep it.
            let mut best = (0.0f32, 1.0f32);
            for k in 0..=segs {
                let theta = start + dir_sign * span * (k as f32 / segs as f32);
                let d = v_dot(n2, radial_at(theta));
                if d.abs() > best.0 {
                    best = (d.abs(), d.signum());
                }
            }
            best.1
        }
    };

    // Blend profile rails per arc sample: offsets from the rim point plus the
    // rail's shading normal, exactly like the straight-edge version.
    let rails_at = |theta: f32| -> Option<Vec<([f32; 3], [f32; 3])>> {
        let n2_loc = v_scale(radial_at(theta), wall_sign);
        // Real wedge angle between cap and wall normals at this sample — the
        // same general-angle trig as the straight-edge ribbon (a rim where the
        // wall meets the cap off-perpendicular, e.g. under a slanted face,
        // otherwise floats the ribbon off the body).
        let c = v_dot(n1, n2_loc);
        let mut rails = Vec::new();
        match kind {
            CornerKind::Chamfer => {
                let sin_theta = (1.0 - c * c).max(1.0e-6).sqrt();
                let t1 = v_scale(
                    v_add(v_scale(n2_loc, -1.0), v_scale(n1, c)),
                    1.0 / sin_theta,
                );
                let t2 = v_scale(
                    v_add(v_scale(n1, -1.0), v_scale(n2_loc, c)),
                    1.0 / sin_theta,
                );
                let normal = v_norm(v_add(n1, n2_loc)).unwrap_or(n1);
                rails.push((v_scale(t1, dist), normal));
                rails.push((v_scale(t2, dist), normal));
            }
            CornerKind::Fillet => {
                let center_offset = v_scale(v_add(n1, n2_loc), -dist / (1.0 + c).max(1.0e-3));
                for i in 0..=EDGE_MOD_PREVIEW_FILLET_SEGS {
                    let phi = i as f32 / EDGE_MOD_PREVIEW_FILLET_SEGS as f32
                        * std::f32::consts::FRAC_PI_2;
                    let dir = v_norm(v_add(v_scale(n2_loc, phi.cos()), v_scale(n1, phi.sin())))?;
                    rails.push((v_add(center_offset, v_scale(dir, dist)), dir));
                }
            }
        }
        (rails.len() >= 2).then_some(rails)
    };

    let point_at = |theta: f32| -> [f32; 3] { v_add(center, v_scale(radial_at(theta), radius)) };

    let mut mesh = MockMesh::empty();
    let n_rails = match kind {
        CornerKind::Chamfer => 2,
        CornerKind::Fillet => EDGE_MOD_PREVIEW_FILLET_SEGS + 1,
    };
    // Vertices: sample-major grid, `n_rails` per arc sample.
    let mut ring_points = Vec::with_capacity(segs + 1);
    for k in 0..=segs {
        let theta = start + dir_sign * span * (k as f32 / segs as f32);
        let p = point_at(theta);
        ring_points.push((theta, p));
        for (offset, normal) in rails_at(theta)? {
            push_preview_vertex(&mut mesh, v_add(p, offset), normal);
        }
    }
    for k in 0..segs {
        for r in 0..n_rails - 1 {
            let a0 = (k * n_rails + r) as u32;
            let a1 = a0 + 1;
            let b0 = a0 + n_rails as u32;
            let b1 = b0 + 1;
            mesh.indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
            mesh.face_ids.push(r as u32 + 1);
            mesh.face_ids.push(r as u32 + 1);
        }
    }

    // Wire feedback: the two boundary rails swept along the arc, plus the
    // profile fan at both free ends of an open arc.
    let boundary_rails: Vec<usize> = match kind {
        CornerKind::Chamfer => vec![0, 1],
        CornerKind::Fillet => vec![0, n_rails - 1],
    };
    for &r in &boundary_rails {
        for k in 0..segs {
            let (t0, p0) = ring_points[k];
            let (t1, p1) = ring_points[k + 1];
            let o0 = rails_at(t0)?[r].0;
            let o1 = rails_at(t1)?[r].0;
            push_preview_edge(&mut mesh, v_add(p0, o0), v_add(p1, o1));
        }
    }
    if !closed {
        for (theta, p) in [*ring_points.first()?, *ring_points.last()?] {
            let rails = rails_at(theta)?;
            for w in rails.windows(2) {
                push_preview_edge(&mut mesh, v_add(p, w[0].0), v_add(p, w[1].0));
            }
        }
    }

    Some(mesh)
}

/// A live, uncommitted 3D edge fillet/chamfer. Holds the captured edge geometry
/// and the editable size; the viewport shows the resulting body in real time.
///
/// One op can round/bevel **several** selected edges at once (Fusion's multi-edge
/// fillet). Explicit selections remain a chain of single-edge `EdgeMod`
/// features; whole-body commands use one exact selector and one atomic kernel
/// operation. The inline size box and drag handle anchor on the first edge.
#[derive(Debug, Clone)]
pub(crate) struct EdgeModOp {
    /// Node id of the body being modified.
    pub(crate) target: String,
    /// The edges being rounded/beveled, captured in world space. Always non-empty.
    pub(crate) edges: Vec<EdgeRef>,
    /// Seed edges plus tangent-continuous neighbours, used by the overlay and
    /// status count. Circular seeds carry this chain into the kernel as one
    /// simultaneous contour, so the propagated neighbours are not separate
    /// history features.
    pub(crate) display_edges: Vec<EdgeRef>,
    /// Whether each edge is concave (inner-corner, ADDS material), one entry per
    /// edge, captured at selection time from the body mesh. Drives the preview
    /// ribbon to the correct (outward) side; convex edges stay `false`.
    pub(crate) concave: Vec<bool>,
    /// Fillet (round) or Chamfer (bevel).
    pub(crate) kind: CornerKind,
    /// Resolved size in base units (mm), kept in sync with `dist_text`.
    pub(crate) dist: f32,
    /// Editable text buffer for the inline size box (a number or a variable
    /// expression).
    pub(crate) dist_text: String,
    /// True until the inline box has grabbed keyboard focus once.
    pub(crate) focus_request: bool,
    /// Whole-body convenience command: commit only after the complete exact
    /// preview resolves without a single blocker.
    pub(crate) strict_all_edges: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum AllEdgeSelectionError {
    NoEligibleEdges,
    MissingDurableNames { count: usize },
    AmbiguousGroup { group: u32 },
    AmbiguousDurableName { name: String },
}

fn durable_all_edge_refs(
    node_id: &str,
    mesh: &MockMesh,
) -> Result<Vec<EdgeRef>, AllEdgeSelectionError> {
    let mut eligible_groups = mesh.edge_groups.clone();
    eligible_groups.extend(mesh.edge_refs.iter().map(|edge| edge.group));
    eligible_groups.sort_unstable();
    eligible_groups.dedup();
    if eligible_groups.is_empty() {
        return Err(AllEdgeSelectionError::NoEligibleEdges);
    }
    if let Some(group) = eligible_groups.iter().copied().find(|group| {
        mesh.edge_refs
            .iter()
            .filter(|edge| edge.group == *group)
            .count()
            > 1
    }) {
        return Err(AllEdgeSelectionError::AmbiguousGroup { group });
    }

    let mut missing = eligible_groups
        .iter()
        .filter(|group| !mesh.edge_refs.iter().any(|edge| edge.group == **group))
        .count();
    let mut named = Vec::with_capacity(mesh.edge_refs.len());
    for edge in &mesh.edge_refs {
        let Some(topology) = edge.topology.as_ref() else {
            missing += 1;
            continue;
        };
        let Some(key) = AllEdgeSelector::durable_edge_key(
            topology.edge_id.as_deref(),
            &topology.adjacent_face_ids,
        ) else {
            missing += 1;
            continue;
        };
        named.push((key, edge.group));
    }
    if missing > 0 {
        return Err(AllEdgeSelectionError::MissingDurableNames { count: missing });
    }
    named.sort();
    if let Some(pair) = named.windows(2).find(|pair| pair[0].0 == pair[1].0) {
        return Err(AllEdgeSelectionError::AmbiguousDurableName {
            name: pair[0].0.clone(),
        });
    }
    named
        .into_iter()
        .map(|(_, group)| {
            ZeroCadApp::edge_ref_from_mesh(node_id, mesh, group)
                .ok_or(AllEdgeSelectionError::MissingDurableNames { count: 1 })
        })
        .collect()
}

impl EdgeModOp {
    /// The primary (first) edge — the anchor for the inline box and drag handle.
    pub(crate) fn primary(&self) -> &EdgeRef {
        &self.edges[0]
    }

    /// Produce the existing `EdgeMod` payload shape used for persistence.
    /// Strict whole-body mode records its exact durable-name set in one
    /// selector-backed edge, so replay remains an atomic operation.
    fn persisted_edges(&self) -> Option<Vec<EdgeRef>> {
        if !self.strict_all_edges {
            return Some(self.edges.clone());
        }
        let names = self.edges.iter().map(|edge| {
            let topology = edge.topology.as_ref()?;
            AllEdgeSelector::durable_edge_key(
                topology.edge_id.as_deref(),
                &topology.adjacent_face_ids,
            )
        });
        let selector = AllEdgeSelector::new(names.collect::<Option<Vec<_>>>()?)
            .ok()?
            .encode()
            .ok()?;
        let mut edge = self.primary().clone();
        edge.topology.as_mut()?.edge_id = Some(selector);
        Some(vec![edge])
    }

    /// World-space midpoint of the primary edge — the anchor for the inline box.
    pub(crate) fn edge_midpoint(&self) -> [f32; 3] {
        let e = self.primary();
        [
            (e.p0[0] + e.p1[0]) * 0.5,
            (e.p0[1] + e.p1[1]) * 0.5,
            (e.p0[2] + e.p1[2]) * 0.5,
        ]
    }

    /// Cheap immediate visual feedback for selected straight edges. The exact
    /// committed B-Rep still comes from the worker-computed edge-mod graph.
    pub(crate) fn immediate_preview_mesh(&self) -> MockMesh {
        let mut mesh = MockMesh::empty();
        for (i, edge) in self.display_edges.iter().enumerate() {
            let concave = self.concave.get(i).copied().unwrap_or(false);
            if let Some(edge_mesh) = edge_mod_edge_preview_mesh(edge, self.dist, self.kind, concave)
            {
                mesh.append(edge_mesh);
            }
        }
        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocad_core::{
        CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves,
    };

    /// Evaluate a circle sketched on the GROUND plane and extruded up — the
    /// user's cylinder — and return its display mesh.
    fn extruded_cylinder_mesh(r: f32, h: f32) -> MockMesh {
        extruded_cylinder_graph(r, h)
            .evaluate_bodies(&std::collections::HashSet::new())
            .expect("cylinder evaluates")
            .remove(0)
            .1
    }

    fn extruded_cylinder_graph(r: f32, h: f32) -> ParametricGraph {
        let mut g = ParametricGraph::new();
        let mut curves = SketchCurves::new();
        curves.add_circle((0.0, 0.0), r);
        g.add_feature(FeatureNode {
            id: "sketch_1".into(),
            name: "Sketch".into(),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XZ,
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
        g.add_feature(FeatureNode {
            id: "extrude_2".into(),
            name: "Extrude".into(),
            feature: FeatureType::Extrude {
                target: None,
                depth: h,
                region_indices: vec![],
                mode: ExtrudeMode::NewBody,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        g.add_dependency("sketch_1", "extrude_2");
        g
    }

    /// The rim edge GROUP at the cap farthest from the sketch plane: every
    /// segment endpoint at that height and at the wall radius.
    fn far_rim_group(mesh: &MockMesh, r: f32) -> u32 {
        let mut y_cap = 0.0f32;
        for v in mesh.edge_vertices.chunks_exact(3) {
            if v[1].abs() > y_cap.abs() {
                y_cap = v[1];
            }
        }
        let seg_count = mesh.edge_indices.len() / 2;
        let vpos = |seg: usize, w: usize| -> [f32; 3] {
            let vi = mesh.edge_indices[seg * 2 + w] as usize * 3;
            [
                mesh.edge_vertices[vi],
                mesh.edge_vertices[vi + 1],
                mesh.edge_vertices[vi + 2],
            ]
        };
        let on_rim = |p: [f32; 3]| -> bool {
            (p[1] - y_cap).abs() < 0.01 && (p[0].hypot(p[2]) - r).abs() < 0.05
        };
        let mut groups: Vec<(u32, usize, usize)> = Vec::new(); // (group, on-rim segs, total segs)
        for s in 0..seg_count {
            let g = mesh.edge_groups.get(s).copied().unwrap_or(s as u32);
            let hit = on_rim(vpos(s, 0)) && on_rim(vpos(s, 1));
            match groups.iter_mut().find(|(gid, _, _)| *gid == g) {
                Some((_, on, total)) => {
                    *on += hit as usize;
                    *total += 1;
                }
                None => groups.push((g, hit as usize, 1)),
            }
        }
        groups
            .into_iter()
            .find(|(_, on, total)| *on == *total && *total >= 3)
            .expect("closed rim group exists")
            .0
    }

    /// End-to-end through the REAL selection pipeline: evaluate an extruded
    /// circle, derive the rim EdgeRef exactly like the GUI's `edge_ref_from`,
    /// and check the immediate ribbon hugs the rim (reported screenshots: it
    /// flared OUTWARD above the cap on real bodies while hand-built EdgeRefs
    /// passed).
    #[test]
    fn real_extruded_cylinder_rim_ribbon_hugs_the_rim() {
        for (r, h, dist) in [(4.0f32, 15.5f32, 2.0f32), (13.0, 10.0, 3.2)] {
            let mesh = extruded_cylinder_mesh(r, h);
            let group = far_rim_group(&mesh, r);
            let edge = crate::ZeroCadApp::edge_ref_from_mesh("extrude_2", &mesh, group)
                .expect("rim edge resolves");
            let mut y_cap = 0.0f32;
            for v in mesh.edge_vertices.chunks_exact(3) {
                if v[1].abs() > y_cap.abs() {
                    y_cap = v[1];
                }
            }
            eprintln!(
                "r={r}: p0={:?} p1={:?} n1={:?} n2={:?} curve={:?}",
                edge.p0, edge.p1, edge.n1, edge.n2, edge.curve
            );
            for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
                let ribbon = edge_mod_edge_preview_mesh(&edge, dist, kind, false)
                    .expect("rim ribbon builds");
                for v in ribbon.vertices.chunks_exact(6) {
                    let rho = v[0].hypot(v[2]);
                    let d_rim = (rho - r).hypot(v[1] - y_cap);
                    assert!(
                        d_rim <= dist * 1.5 + 0.05,
                        "r={r} {kind:?}: ribbon vertex strayed {d_rim} from the rim at [{}, {}, {}]",
                        v[0],
                        v[1],
                        v[2]
                    );
                    assert!(
                        v[1] <= y_cap + 1.0e-3,
                        "r={r} {kind:?}: ribbon rose above the cap (y={})",
                        v[1]
                    );
                    assert!(
                        rho <= r + 0.05,
                        "r={r} {kind:?}: ribbon bulged outside the wall (rho={rho})"
                    );
                }
            }
        }
    }

    /// The refined preview is the REAL blended body computed by the worker
    /// (`build_edge_mod_arc_graph` → `evaluate_bodies_with_warnings`). If that
    /// solve fails the user is stuck with the overlay ribbon forever — the
    /// reported "preview is just lines on the body". Reproduce the worker graph
    /// for the reported case (sketched circle → extrude → rim fillet/chamfer at
    /// 1.47mm) and require a warning-free solve that actually cuts the rim.
    #[test]
    fn real_extruded_cylinder_rim_exact_preview_solves() {
        let (r, h, dist) = (5.0f32, 12.0f32, 1.47f32);
        let hidden = std::collections::HashSet::new();
        let mesh = extruded_cylinder_mesh(r, h);
        let group = far_rim_group(&mesh, r);
        let edge = crate::ZeroCadApp::edge_ref_from_mesh("extrude_2", &mesh, group)
            .expect("rim edge resolves");
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            let mut g = extruded_cylinder_graph(r, h);
            // Body nodes evaluate in creation order = numeric id suffix; the
            // GUI names spec nodes past `id_counter`, so use a trailing suffix.
            g.add_feature(FeatureNode {
                id: "edgemod_spec_3".into(),
                name: "spec edge mod 3".into(),
                feature: FeatureType::EdgeMod {
                    target: "extrude_2".into(),
                    edge: edge.clone(),
                    dist,
                    dist_expr: None,
                    kind,
                },
            });
            g.add_dependency("extrude_2", "edgemod_spec_3");
            let (bodies, warnings) = g
                .evaluate_bodies_with_warnings(&hidden)
                .unwrap_or_else(|e| panic!("{kind:?}: exact preview graph failed: {e}"));
            assert!(
                warnings.is_empty(),
                "{kind:?}: preview warnings {warnings:?}"
            );
            let blended = &bodies
                .iter()
                .find(|(id, _)| id == "extrude_2")
                .expect("blended body keys by target")
                .1;
            // The sharp rim corner (rho≈r AND y≈h) must be gone.
            let y_cap = h;
            for v in blended.vertices.chunks_exact(6) {
                let rho = v[0].hypot(v[2]);
                assert!(
                    (rho - r).abs() > 0.05 || (v[1] - y_cap).abs() > 0.05,
                    "{kind:?}: sharp rim corner survived at [{}, {}, {}]",
                    v[0],
                    v[1],
                    v[2]
                );
            }
        }
    }

    fn straight_box_edge() -> EdgeRef {
        EdgeRef {
            p0: [0.0, 0.0, 15.0],
            p1: [40.0, 0.0, 15.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        }
    }

    fn test_op(kind: CornerKind, dist: f32) -> EdgeModOp {
        let edge = straight_box_edge();
        EdgeModOp {
            target: "body".to_string(),
            edges: vec![edge.clone()],
            display_edges: vec![edge],
            concave: vec![false],
            kind,
            dist,
            dist_text: format!("{dist:.2}"),
            focus_request: false,
            strict_all_edges: false,
        }
    }

    fn named_mesh_edge(group: u32, name: Option<&str>) -> zerocad_core::mock_kernel::MeshEdgeRef {
        zerocad_core::mock_kernel::MeshEdgeRef {
            group,
            p0: [group as f32, 0.0, 0.0],
            p1: [group as f32, 0.0, 1.0],
            n1: [1.0, 0.0, 0.0],
            n2: [0.0, 1.0, 0.0],
            curve: None,
            topology: name.map(|name| zerocad_core::mock_kernel::MeshTopologyEdgeRef {
                body_id: Some("body".to_string()),
                edge_id: Some(name.to_string()),
                ..Default::default()
            }),
        }
    }

    #[test]
    fn all_edge_materialization_is_durable_unique_and_name_ordered() {
        let mut mesh = MockMesh::empty();
        mesh.edge_groups = vec![9, 2, 9, 2];
        mesh.edge_refs = vec![
            named_mesh_edge(9, Some("entity:9:edge")),
            named_mesh_edge(2, Some("entity:2:edge")),
        ];
        let edges = durable_all_edge_refs("body", &mesh).expect("durable all-edge selection");
        assert_eq!(edges.len(), 2);
        assert_eq!(
            edges
                .iter()
                .map(|edge| edge
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.edge_id.as_deref())
                    .expect("durable edge name"))
                .collect::<Vec<_>>(),
            ["entity:2:edge", "entity:9:edge"]
        );
    }

    #[test]
    fn all_edge_materialization_rejects_missing_or_ambiguous_names() {
        let mut missing = MockMesh::empty();
        missing.edge_groups = vec![1, 2];
        missing.edge_refs = vec![
            named_mesh_edge(1, Some("entity:1:edge")),
            named_mesh_edge(2, None),
        ];
        assert_eq!(
            durable_all_edge_refs("body", &missing),
            Err(AllEdgeSelectionError::MissingDurableNames { count: 1 })
        );

        let mut ambiguous = MockMesh::empty();
        ambiguous.edge_groups = vec![1, 2];
        ambiguous.edge_refs = vec![
            named_mesh_edge(1, Some("entity:1:edge")),
            named_mesh_edge(2, Some("entity:1:edge")),
        ];
        assert_eq!(
            durable_all_edge_refs("body", &ambiguous),
            Err(AllEdgeSelectionError::AmbiguousDurableName {
                name: "entity:1:edge".to_string()
            })
        );

        let mut duplicate_group = MockMesh::empty();
        duplicate_group.edge_groups = vec![1];
        duplicate_group.edge_refs = vec![
            named_mesh_edge(1, Some("entity:1:edge")),
            named_mesh_edge(1, Some("entity:2:edge")),
        ];
        assert_eq!(
            durable_all_edge_refs("body", &duplicate_group),
            Err(AllEdgeSelectionError::AmbiguousGroup { group: 1 })
        );
    }

    #[test]
    fn all_edge_materialization_accepts_primitive_durable_face_pairs() {
        let mut first = named_mesh_edge(1, Some("mesh:1"));
        first
            .topology
            .as_mut()
            .expect("first topology")
            .adjacent_face_ids = vec![
            "box_box_1:face:+x".to_string(),
            "box_box_1:face:+y".to_string(),
        ];
        let mut second = named_mesh_edge(2, Some("mesh:2"));
        second
            .topology
            .as_mut()
            .expect("second topology")
            .adjacent_face_ids = vec![
            "box_box_1:face:+x".to_string(),
            "box_box_1:face:+z".to_string(),
        ];
        let mut mesh = MockMesh::empty();
        mesh.edge_groups = vec![1, 2];
        mesh.edge_refs = vec![first, second];
        let edges = durable_all_edge_refs("box_1", &mesh)
            .expect("primitive face pairs are durable edge identities");
        assert_eq!(edges.len(), 2);
        let op = EdgeModOp {
            target: "box_1".to_string(),
            display_edges: edges.clone(),
            edges,
            concave: vec![false, false],
            kind: CornerKind::Chamfer,
            dist: 0.4,
            dist_text: "0.4".to_string(),
            focus_request: false,
            strict_all_edges: true,
        };
        let persisted = op.persisted_edges().expect("face-pair selector");
        let selector = AllEdgeSelector::decode(
            persisted[0]
                .topology
                .as_ref()
                .and_then(|topology| topology.edge_id.as_deref())
                .expect("selector edge id"),
        )
        .expect("selector prefix")
        .expect("selector payload");
        assert_eq!(selector.edge_names.len(), 2);
    }

    #[test]
    fn strict_all_edge_op_persists_one_exact_selector_feature() {
        let mut first = straight_box_edge();
        first.topology = Some(zerocad_core::TopologyEdgeRef {
            body_id: Some("body".to_string()),
            edge_id: Some("entity:9:edge".to_string()),
            ..Default::default()
        });
        let mut second = straight_box_edge();
        second.p0[0] = 1.0;
        second.p1[0] = 1.0;
        second.topology = Some(zerocad_core::TopologyEdgeRef {
            body_id: Some("body".to_string()),
            edge_id: Some("entity:2:edge".to_string()),
            ..Default::default()
        });
        let op = EdgeModOp {
            target: "body".to_string(),
            edges: vec![first, second],
            display_edges: Vec::new(),
            concave: Vec::new(),
            kind: CornerKind::Fillet,
            dist: 0.4,
            dist_text: "0.4".to_string(),
            focus_request: false,
            strict_all_edges: true,
        };

        let persisted = op.persisted_edges().expect("selector payload");
        assert_eq!(persisted.len(), 1);
        let edge_id = persisted[0]
            .topology
            .as_ref()
            .and_then(|topology| topology.edge_id.as_deref())
            .expect("selector id");
        let selector = AllEdgeSelector::decode(edge_id)
            .expect("reserved selector")
            .expect("valid selector");
        assert_eq!(selector.edge_names, ["entity:2:edge", "entity:9:edge"]);
    }

    /// A concave bite arc on a body's top face (z = 10): rim circle about +Z,
    /// wall normal pointing toward the axis, spanning 90°..270°.
    fn bite_arc_edge(closed: bool, reversed: bool) -> EdgeRef {
        let (start, end) = if reversed {
            (
                3.0 * std::f32::consts::FRAC_PI_2,
                std::f32::consts::FRAC_PI_2,
            )
        } else {
            (
                std::f32::consts::FRAC_PI_2,
                3.0 * std::f32::consts::FRAC_PI_2,
            )
        };
        EdgeRef {
            p0: [20.0, 22.0, 10.0],
            p1: [20.0, -6.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0], // at 90°, pointing back toward the axis (concave)
            curve: Some(EdgeCurveHint::Circle {
                center: [20.0, 8.0, 10.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 14.0,
                start,
                end,
                closed,
            }),
            topology: None,
        }
    }

    #[test]
    fn circular_edge_preview_mesh_sweeps_the_arc_and_respects_direction() {
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            for reversed in [false, true] {
                let mesh =
                    edge_mod_edge_preview_mesh(&bite_arc_edge(false, reversed), 3.0, kind, false)
                        .expect("circular selections must get an immediate preview");
                assert!(!mesh.indices.is_empty(), "{kind:?} ribbon has triangles");
                assert!(
                    !mesh.edge_indices.is_empty(),
                    "{kind:?} ribbon has wire feedback"
                );
                // Every vertex stays within the blend's reach of the rim circle
                // (on-arc, offset at most ~dist·√2 from the rim), regardless of
                // which direction the hint encodes the arc.
                for v in mesh.vertices.chunks_exact(6) {
                    let (dx, dy, dz) = (v[0] - 20.0, v[1] - 8.0, v[2] - 10.0);
                    let rho = dx.hypot(dy);
                    let d_rim = (rho - 14.0).hypot(dz);
                    assert!(
                        d_rim <= 3.0 * 1.5 + 1.0e-3,
                        "{kind:?} ribbon vertex strayed {d_rim} from the rim"
                    );
                    // Concave wall: the profile offsets outward (away from the
                    // axis) and down into the material, never up above the cap.
                    assert!(v[2] <= 10.0 + 1.0e-3, "{kind:?} ribbon rose above the cap");
                }
            }
        }
    }

    /// The top rim of a plain cylinder (r=13, cap at z=30), with the adjacent
    /// face normals in BOTH orders — tessellation order is arbitrary, so the
    /// ribbon must identify cap vs wall geometrically. n2/n1 here are the wall
    /// normal captured at 0° (+x) and the cap normal (+z).
    fn cylinder_rim_edge(swapped: bool) -> EdgeRef {
        let (n1, n2) = if swapped {
            ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0])
        } else {
            ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        };
        EdgeRef {
            p0: [13.0, 0.0, 30.0],
            p1: [13.0, 0.0, 30.0],
            n1,
            n2,
            curve: Some(EdgeCurveHint::Circle {
                center: [0.0, 0.0, 30.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 13.0,
                start: 0.0,
                end: std::f32::consts::TAU,
                closed: true,
            }),
            topology: None,
        }
    }

    #[test]
    fn cylinder_rim_preview_hugs_the_rim_for_either_normal_order() {
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            for swapped in [false, true] {
                let mesh =
                    edge_mod_edge_preview_mesh(&cylinder_rim_edge(swapped), 2.0, kind, false)
                        .expect("closed rim preview");
                for v in mesh.vertices.chunks_exact(6) {
                    let rho = v[0].hypot(v[1]);
                    let d_rim = (rho - 13.0).hypot(v[2] - 30.0);
                    // Convex rim: the band carves into the corner — never above
                    // the cap, never outside the wall, always near the rim.
                    assert!(
                        d_rim <= 2.0 * 1.5 + 1.0e-3,
                        "{kind:?} swapped={swapped}: vertex strayed {d_rim} from the rim"
                    );
                    assert!(
                        v[2] <= 30.0 + 1.0e-3,
                        "{kind:?} swapped={swapped}: ribbon rose above the cap"
                    );
                    assert!(
                        rho <= 13.0 + 1.0e-3,
                        "{kind:?} swapped={swapped}: ribbon bulged outside the wall (rho {rho})"
                    );
                }
            }
        }
    }

    #[test]
    fn closed_rim_preview_mesh_wraps_without_end_fans() {
        let open = edge_mod_edge_preview_mesh(
            &bite_arc_edge(false, false),
            3.0,
            CornerKind::Fillet,
            false,
        )
        .expect("open preview");
        let closed =
            edge_mod_edge_preview_mesh(&bite_arc_edge(true, false), 3.0, CornerKind::Fillet, false)
                .expect("closed preview");
        // The closed rim sweeps the full circle: more triangles, and no
        // end-profile wire fans beyond the two boundary rails.
        assert!(closed.indices.len() > open.indices.len());
    }

    fn y_z_bounds(mesh: &MockMesh) -> (f32, f32, f32, f32) {
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for v in mesh.vertices.chunks_exact(6) {
            min_y = min_y.min(v[1]);
            max_y = max_y.max(v[1]);
            min_z = min_z.min(v[2]);
            max_z = max_z.max(v[2]);
        }
        (min_y, max_y, min_z, max_z)
    }

    /// A concave (inner pocket corner) edge at (5,5), z 4..10, with outward wall
    /// normals +x/+y. The blend ADDS material in the corner void, so the preview
    /// ribbon must sit on the +x/+y side (into the pocket), the mirror of a
    /// convex edge whose ribbon would carve toward −x/−y.
    #[test]
    fn concave_edge_preview_mesh_mirrors_to_the_corner_void() {
        let edge = EdgeRef {
            p0: [5.0, 5.0, 4.0],
            p1: [5.0, 5.0, 10.0],
            n1: [1.0, 0.0, 0.0],
            n2: [0.0, 1.0, 0.0],
            curve: None,
            topology: None,
        };
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            let concave = edge_mod_edge_preview_mesh(&edge, 2.0, kind, true)
                .unwrap_or_else(|| panic!("{kind:?} concave preview"));
            let convex = edge_mod_edge_preview_mesh(&edge, 2.0, kind, false)
                .unwrap_or_else(|| panic!("{kind:?} convex preview"));
            // Every concave ribbon vertex lies in the corner void (x ≥ 5, y ≥ 5);
            // every convex one carves the other way (x ≤ 5, y ≤ 5).
            for v in concave.vertices.chunks_exact(6) {
                assert!(
                    v[0] >= 5.0 - 1.0e-4 && v[1] >= 5.0 - 1.0e-4,
                    "{kind:?} concave ribbon must stay in the corner void, got [{}, {}]",
                    v[0],
                    v[1]
                );
            }
            for v in convex.vertices.chunks_exact(6) {
                assert!(
                    v[0] <= 5.0 + 1.0e-4 && v[1] <= 5.0 + 1.0e-4,
                    "{kind:?} convex ribbon must carve inward, got [{}, {}]",
                    v[0],
                    v[1]
                );
            }
        }
    }

    /// A SHARP (non-90°) wedge: vertical edge whose faces have outward normals
    /// (0,−1,0) and (sin20°·…) — a ~20° corner. The ribbon's tangency rails
    /// must lie ON the faces (perpendicular distance ~0) and the whole ribbon
    /// must stay INSIDE the material wedge. The old ±(n1+n2)·dist center was
    /// the 90°-only special case and floated the ribbon off the body here.
    #[test]
    fn sharp_wedge_preview_ribbon_hugs_the_faces() {
        // Wedge angle 20°: faces y=0 (outward −y) and a plane rotated 20° up
        // from it (outward normal at 20° from +y within the xy-plane, i.e.
        // n2 = (sin20°, cos20°, 0) for a corner opening toward −x).
        let a = 20.0f32.to_radians();
        let edge = EdgeRef {
            p0: [0.0, 0.0, 0.0],
            p1: [0.0, 0.0, 6.0],
            n1: [0.0, -1.0, 0.0],
            n2: [a.sin(), a.cos(), 0.0],
            curve: None,
            topology: None,
        };
        let dist = 2.52;
        for kind in [CornerKind::Fillet, CornerKind::Chamfer] {
            let mesh = edge_mod_edge_preview_mesh(&edge, dist, kind, false)
                .unwrap_or_else(|| panic!("{kind:?} sharp preview"));
            // First and last rails are the tangency/foot rails; they must lie
            // in their face planes (both planes pass through the edge origin).
            let verts: Vec<[f32; 3]> = mesh
                .vertices
                .chunks_exact(6)
                .map(|v| [v[0], v[1], v[2]])
                .collect();
            // Each end rail must lie on one of the two faces (which face pairs
            // with which rail differs between fillet and chamfer).
            for p in [verts[0], verts[verts.len() - 1]] {
                let to_face1 = p[1].abs();
                let to_face2 = (p[0] * edge.n2[0] + p[1] * edge.n2[1]).abs();
                assert!(
                    to_face1.min(to_face2) <= 1.0e-3,
                    "{kind:?}: end rail [{}, {}, {}] lies on neither face \
                     (d1={to_face1}, d2={to_face2})",
                    p[0],
                    p[1],
                    p[2]
                );
            }
            // Every ribbon vertex stays inside the material wedge (behind BOTH
            // faces): x·n ≤ small for each outward normal.
            for v in &verts {
                let d1 = -v[1]; // signed distance beyond face 1 (outward −y)
                let d2 = v[0] * edge.n2[0] + v[1] * edge.n2[1];
                assert!(
                    d1 <= 1.0e-3 && d2 <= 1.0e-3,
                    "{kind:?}: ribbon vertex [{}, {}, {}] floats outside the wedge (d1={d1}, d2={d2})",
                    v[0],
                    v[1],
                    v[2]
                );
            }
        }
    }

    #[test]
    fn immediate_edge_mod_preview_mesh_is_bounded_and_kind_specific() {
        let fillet = test_op(CornerKind::Fillet, 3.0).immediate_preview_mesh();
        let chamfer = test_op(CornerKind::Chamfer, 3.0).immediate_preview_mesh();
        assert!(
            !fillet.vertices.is_empty(),
            "fillet overlay should be available immediately"
        );
        assert!(
            !chamfer.vertices.is_empty(),
            "chamfer overlay should be available immediately"
        );
        assert!(
            fillet.indices.len() > chamfer.indices.len(),
            "fillet overlay should be faceted while chamfer is one bevel strip"
        );
        assert_eq!(
            fillet.edge_indices.len() / 2,
            EDGE_MOD_PREVIEW_FILLET_SEGS * 2 + 2,
            "fillet overlay should draw tangent/end edges, not every internal rail"
        );
        assert_eq!(
            chamfer.edge_indices.len() / 2,
            4,
            "chamfer overlay should draw the two bevel rails and two end edges"
        );

        for (label, mesh) in [("fillet", &fillet), ("chamfer", &chamfer)] {
            let (min_y, max_y, min_z, max_z) = y_z_bounds(mesh);
            assert!(
                min_y >= -1.0e-4,
                "{label} preview dipped outside front face"
            );
            assert!(
                max_y <= 3.0 + 1.0e-4,
                "{label} preview exceeded selected distance"
            );
            assert!(
                min_z >= 12.0 - 1.0e-4,
                "{label} preview cut deeper than selected distance"
            );
            assert!(
                max_z <= 15.0 + 1.0e-4,
                "{label} preview rose above top face"
            );
        }

        let larger = test_op(CornerKind::Fillet, 5.0).immediate_preview_mesh();
        let (_min_y, max_y, min_z, _max_z) = y_z_bounds(&larger);
        assert!(
            max_y > 4.9,
            "larger radius should visibly widen the overlay"
        );
        assert!(
            min_z < 10.1,
            "larger radius should visibly deepen the overlay"
        );
    }
}

impl ZeroCadApp {
    /// Begin a live fillet/chamfer of the currently selected body edge. Seeds the
    /// size from `edge_mod_dist_text` (remembered across uses) and opens the
    /// preview; nothing is committed until [`commit_edge_mod`](Self::commit_edge_mod).
    pub(crate) fn begin_edge_mod(&mut self, kind: CornerKind) {
        let Some((node_id, selected_edge_ids)) = self.selected_body_edges() else {
            self.status_msg = "Select one or more body edges first.".to_string();
            return;
        };
        let edge_ids = self.tangent_edge_chain(&node_id, &selected_edge_ids);
        let display_edges: Vec<EdgeRef> = edge_ids
            .iter()
            .filter_map(|&e| self.edge_ref_from(&node_id, e))
            .collect();
        let selected_edges: Vec<EdgeRef> = selected_edge_ids
            .iter()
            .filter_map(|&e| self.edge_ref_from(&node_id, e))
            .collect();
        // An arc-led tangent chain is one kernel contour. Persist its circular
        // member so the kernel rediscovers the complete G1 chain. A straight
        // seed deliberately stays a single edge; additional straight edges are
        // included only through explicit multi-selection.
        let edges = if selected_edges.len() == 1 {
            display_edges
                .iter()
                .find(|edge| matches!(edge.curve, Some(EdgeCurveHint::Circle { .. })))
                .cloned()
                .map(|edge| vec![edge])
                .unwrap_or(selected_edges)
        } else {
            selected_edges
        };
        if edges.is_empty() {
            self.status_msg = "Those edges have no usable geometry to fillet/chamfer.".to_string();
            return;
        }
        // Classify each edge's wedge from the body's display mesh so the preview
        // ribbon draws on the correct side (concave edges add material outward).
        let body_mesh = self
            .body_meshes
            .iter()
            .find(|(id, _)| *id == node_id)
            .map(|(_, mesh)| mesh);
        let concave: Vec<bool> = display_edges
            .iter()
            .map(|edge| {
                body_mesh
                    .and_then(|mesh| {
                        zerocad_core::edge_wedge_is_concave_mesh(
                            mesh, edge.p0, edge.p1, edge.n1, edge.n2,
                        )
                    })
                    .unwrap_or(false)
            })
            .collect();
        let text = self.edge_mod_dist_text.clone();
        let dist = self.eval_dim(&text).unwrap_or(3.0).max(0.2);
        self.edge_mod_op = Some(EdgeModOp {
            target: node_id,
            edges,
            display_edges,
            concave,
            kind,
            dist,
            dist_text: text,
            focus_request: true,
            strict_all_edges: false,
        });
        // Start each edit with a clean speculative edge-mod slate so a stale
        // precompute from a previous edit can't be mistaken for this one.
        self.clear_edge_mod_speculation();
        let propagated = edge_ids.len().saturating_sub(selected_edge_ids.len());
        self.status_msg = if propagated > 0 {
            format!(
                "Tangent chain: {} adjacent edge(s) included. Set the size, then Enter / OK to apply.",
                propagated
            )
        } else {
            "Set the size, then Enter / OK to apply (Esc cancels).".to_string()
        };
    }

    /// Begin a strict whole-body Fillet/Chamfer by materializing every eligible
    /// selectable edge as a uniquely named durable [`EdgeRef`]. No history is
    /// changed unless the exact full-chain preview succeeds without warnings.
    pub(crate) fn begin_all_edge_mod(&mut self, kind: CornerKind) {
        let Some(node_id) = self.selected_whole_body() else {
            self.status_msg = "Fully select one body first.".to_string();
            return;
        };
        let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| *id == node_id) else {
            self.status_msg = "The selected body has no evaluated geometry.".to_string();
            return;
        };
        let edges = match durable_all_edge_refs(&node_id, mesh) {
            Ok(edges) => edges,
            Err(AllEdgeSelectionError::NoEligibleEdges) => {
                self.status_msg = "The selected body has no eligible edges.".to_string();
                return;
            }
            Err(AllEdgeSelectionError::MissingDurableNames { count }) => {
                self.status_msg = format!(
                    "All-edge operation refused: {count} eligible edge(s) lack durable identities."
                );
                return;
            }
            Err(AllEdgeSelectionError::AmbiguousGroup { group }) => {
                self.status_msg = format!(
                    "All-edge operation refused: selectable edge group {group} resolves more than once."
                );
                return;
            }
            Err(AllEdgeSelectionError::AmbiguousDurableName { name }) => {
                self.status_msg = format!(
                    "All-edge operation refused: durable edge identity '{name}' is ambiguous."
                );
                return;
            }
        };
        let concave = edges
            .iter()
            .map(|edge| {
                zerocad_core::edge_wedge_is_concave_mesh(mesh, edge.p0, edge.p1, edge.n1, edge.n2)
                    .unwrap_or(false)
            })
            .collect::<Vec<_>>();
        let text = self.edge_mod_dist_text.clone();
        let dist = self.eval_dim(&text).unwrap_or(3.0).max(0.2);
        self.edge_mod_op = Some(EdgeModOp {
            target: node_id,
            display_edges: edges.clone(),
            edges,
            concave,
            kind,
            dist,
            dist_text: text,
            focus_request: true,
            strict_all_edges: true,
        });
        self.clear_edge_mod_speculation();
        let count = self
            .edge_mod_op
            .as_ref()
            .map(|operation| operation.edges.len())
            .unwrap_or(0);
        self.status_msg = format!(
            "Checking all {count} durable edge(s) atomically. Commit is enabled after the exact preview passes."
        );
    }

    /// Reset all speculative edge-mod precompute state (cache and in-flight job).
    /// Called when an edit begins, commits, or is cancelled.
    pub(crate) fn clear_edge_mod_speculation(&mut self) {
        self.edge_mod_arc_cache = None;
        self.edge_mod_arc_lru.clear();
        self.edge_mod_arc_inflight = None;
        self.edge_mod_preview_mesh_cache = None;
        self.edge_mod_arc_failed = None;
    }

    /// Record a completed exact solve in the recent-sizes cache (most-recent
    /// first, deduped by key, bounded to [`EDGE_MOD_ARC_LRU_CAP`]).
    pub(crate) fn remember_edge_mod_arc(
        &mut self,
        key: u64,
        bodies: SharedBodyMeshes,
        warnings: &[String],
    ) {
        self.edge_mod_arc_lru.retain(|(k, _, _)| *k != key);
        self.edge_mod_arc_lru
            .insert(0, (key, bodies, warnings.to_vec()));
        self.edge_mod_arc_lru.truncate(EDGE_MOD_ARC_LRU_CAP);
    }

    /// The recently-solved bodies for `key`, if this size was solved earlier in
    /// the edit (scrubbing back to it).
    fn edge_mod_arc_lru_get(&self, key: u64) -> Option<&SharedBodyMeshes> {
        self.edge_mod_arc_lru
            .iter()
            .find(|(k, _, _)| *k == key)
            .map(|(_, bodies, _)| bodies)
    }

    fn hash_quantized_f32(h: &mut impl std::hash::Hasher, v: f32, scale: f64) {
        use std::hash::Hash;
        ((v as f64 / scale).round() as i64).hash(h);
    }

    fn hash_edge_ref(h: &mut impl std::hash::Hasher, edge: &EdgeRef) {
        use std::hash::Hash;
        for c in edge
            .p0
            .iter()
            .chain(edge.p1.iter())
            .chain(edge.n1.iter())
            .chain(edge.n2.iter())
        {
            Self::hash_quantized_f32(h, *c, 1.0e-4);
        }
        match &edge.curve {
            None => 0u8.hash(h),
            Some(EdgeCurveHint::Line) => 1u8.hash(h),
            Some(EdgeCurveHint::Circle {
                center,
                axis,
                x_dir,
                radius,
                start,
                end,
                closed,
            }) => {
                2u8.hash(h);
                for c in center.iter().chain(axis.iter()).chain(x_dir.iter()) {
                    Self::hash_quantized_f32(h, *c, 1.0e-4);
                }
                Self::hash_quantized_f32(h, *radius, 1.0e-4);
                Self::hash_quantized_f32(h, *start, 1.0e-6);
                Self::hash_quantized_f32(h, *end, 1.0e-6);
                closed.hash(h);
            }
        }
        if let Some(topology) = &edge.topology {
            topology.body_id.hash(h);
            topology.topology_version.hash(h);
            topology.edge_id.hash(h);
            topology.curve_kind.hash(h);
            topology.adjacent_face_ids.hash(h);
            topology.adjacent_surface_kinds.hash(h);
        } else {
            0u8.hash(h);
        }
    }

    /// Hash of everything that determines an edge-mod's committed geometry — the
    /// exact size, kind, target body, selected edge identity, and hidden nodes.
    /// The preview worker result is only reusable when this full identity matches.
    fn edge_mod_arc_key(op: &EdgeModOp, hidden_nodes: &std::collections::HashSet<String>) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        ((op.dist.max(0.2) / 0.01).round() as i64).hash(&mut h);
        (op.kind as u8).hash(&mut h);
        op.strict_all_edges.hash(&mut h);
        op.target.hash(&mut h);
        for edge in &op.display_edges {
            Self::hash_edge_ref(&mut h, edge);
        }
        let mut hidden: Vec<&String> = hidden_nodes.iter().collect();
        hidden.sort();
        for id in hidden {
            id.hash(&mut h);
        }
        h.finish()
    }

    /// Append the operation to a cloned graph. Explicit multi-selection keeps
    /// its dependency-ordered chain, while strict whole-body mode writes one
    /// selector-backed node so preview and commit share the atomic kernel path.
    fn append_edge_mod_chain(
        &self,
        graph: &mut zerocad_core::ParametricGraph,
        op: &EdgeModOp,
        dist: f32,
        tag: &str,
    ) -> bool {
        let Some(edges) = op.persisted_edges() else {
            return false;
        };
        let mut prev = op.target.clone();
        for (i, edge) in edges.into_iter().enumerate() {
            let id = format!("edgemod_{tag}_{}", self.id_counter + i);
            graph.add_feature(FeatureNode {
                id: id.clone(),
                name: format!("{tag} edge mod {i}"),
                feature: FeatureType::EdgeMod {
                    target: op.target.clone(),
                    edge,
                    dist,
                    dist_expr: None,
                    kind: op.kind,
                },
            });
            graph.add_dependency(&prev, &id);
            prev = id;
        }
        true
    }

    /// Build the graph the speculative precompute evaluates: the current model
    /// plus the live edit as real `EdgeMod` nodes, using the same `dist.max(0.2)`
    /// the commit will. Evaluated on a worker thread, it yields exactly the bodies
    /// a commit at this size would. Bodies key by `target`, not the node id, so
    /// this matches the committed result despite the throwaway node name.
    fn build_edge_mod_arc_document(&self) -> Option<zerocad_core::Document> {
        let op = self.edge_mod_op.as_ref()?;
        let mut graph = self.document.clone();
        if !self.append_edge_mod_chain(&mut graph, op, op.dist.max(0.2), "spec") {
            return None;
        }
        Some(graph)
    }

    fn spawn_edge_mod_arc_eval(&mut self, ctx: &egui::Context, key: u64) {
        let Some(document) = self.build_edge_mod_arc_document() else {
            return;
        };
        self.evaluator.submit(
            crate::evaluation_worker::EvaluationPurpose::EdgeModPreview(key),
            document,
            self.hidden_nodes.clone(),
            zerocad_core::EvaluationQuality::Interactive,
            Some(ctx.clone()),
        );
        self.edge_mod_arc_inflight = Some(key);
    }

    /// Drive the speculative edge-mod precompute. Called once per frame. Exact
    /// work starts immediately and runs on the shared evaluator; a newer key
    /// cancels and supersedes obsolete work.
    pub(crate) fn tick_speculative_edge_mod(&mut self, ctx: &egui::Context) {
        let Some(op) = self.edge_mod_op.as_ref() else {
            return;
        };
        if self.eval_pending {
            return;
        }
        let key = Self::edge_mod_arc_key(op, &self.hidden_nodes);

        // Already computed (or computing) the arc for this exact size.
        if matches!(&self.edge_mod_arc_cache, Some((k, _, _)) if *k == key) {
            return;
        }
        // Solved earlier in this edit (size scrubbed back): promote it to the
        // active cache so the preview shows it instantly — no re-solve.
        if self.edge_mod_arc_cache.as_ref().map(|(k, _, _)| *k) != Some(key) {
            if let Some((k, bodies, warnings)) = self
                .edge_mod_arc_lru
                .iter()
                .find(|(k, _, _)| *k == key)
                .cloned()
            {
                self.edge_mod_arc_cache = Some((k, bodies, warnings));
                self.edge_mod_arc_failed = None;
                return;
            }
        }
        // This exact size already failed — don't burn CPU re-solving it every
        // tick. The failure was surfaced in the status bar; a size change makes
        // a new key and retries.
        if self.edge_mod_arc_failed == Some(key) {
            return;
        }
        if self.edge_mod_arc_inflight == Some(key) {
            return;
        }
        if self
            .edge_mod_arc_inflight
            .is_some_and(|inflight| inflight != key)
        {
            // The user has moved on to a different size/edge state. Drop the
            // shared evaluator request so its stale result cannot populate the
            // cache, then let the current key schedule normally below.
            self.evaluator.cancel();
            self.edge_mod_arc_inflight = None;
        }

        // Only one speculative job at a time. A stale job was cancelled above;
        // otherwise the active job already matched this key and returned early.
        if self.edge_mod_arc_inflight.is_some() {
            return;
        }

        self.spawn_edge_mod_arc_eval(ctx, key);
    }

    /// Select the best available edge-mod preview: an exact worker result for the
    /// current size when ready, otherwise a cached lightweight overlay. Exact
    /// results are retained by size so scrubbing back does not launch a new solve.
    pub(crate) fn cached_preview_edge_mod_bodies(&mut self) -> Option<SharedBodyMeshes> {
        use std::hash::{Hash, Hasher};
        let Some(op) = self.edge_mod_op.as_ref() else {
            self.edge_mod_preview_cache = None;
            return None;
        };
        // Prefer the speculative smooth arc result if it's ready for this exact
        // size: showing the final one-face round *in the preview* means committing
        // changes nothing on screen — the round refines gently while the user is
        // still adjusting, instead of the body popping faceted→arc after commit.
        let arc_key = Self::edge_mod_arc_key(op, &self.hidden_nodes);
        if let Some((k, bodies, _)) = self.edge_mod_arc_cache.as_ref() {
            if *k == arc_key {
                return Some(bodies.clone());
            }
        }
        // A size scrubbed back to one solved earlier this edit: exact preview
        // straight from the recent-sizes cache, no re-solve.
        if let Some(bodies) = self.edge_mod_arc_lru_get(arc_key) {
            return Some(bodies.clone());
        }
        let key = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            // Quantize size to 0.05mm: idle frames and slow drags reuse the cache,
            // and a sub-0.05mm preview lag is invisible (commit uses the exact size).
            ((op.dist / 0.05).round() as i64).hash(&mut h);
            (op.kind as u8).hash(&mut h);
            op.target.hash(&mut h);
            // The edges themselves — two edges of the same body share `target`, so
            // without this a fillet on edge B could reuse edge A's cached result.
            for edge in &op.display_edges {
                Self::hash_edge_ref(&mut h, edge);
            }
            self.id_counter.hash(&mut h);
            let mut hidden: Vec<&String> = self.hidden_nodes.iter().collect();
            hidden.sort();
            for id in hidden {
                id.hash(&mut h);
            }
            h.finish()
        };
        if let Some((cached_key, bodies)) = self.edge_mod_preview_cache.as_ref() {
            if *cached_key == key {
                return Some(bodies.clone());
            }
        }
        self.edge_mod_preview_cache = None;
        None
    }

    /// Memoized lightweight overlay mesh used until the exact worker result is
    /// available. This path is cheap and synchronous, so the edge edit responds on
    /// the first frame even when the kernel solve takes much longer.
    pub(crate) fn cached_preview_edge_mod_mesh(&mut self) -> Option<MockMesh> {
        use std::hash::{Hash, Hasher};
        let Some(op) = self.edge_mod_op.as_ref() else {
            self.edge_mod_preview_mesh_cache = None;
            return None;
        };
        let key = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            ((op.dist / 0.05).round() as i64).hash(&mut h);
            (op.kind as u8).hash(&mut h);
            op.target.hash(&mut h);
            for edge in &op.display_edges {
                for c in edge
                    .p0
                    .iter()
                    .chain(edge.p1.iter())
                    .chain(edge.n1.iter())
                    .chain(edge.n2.iter())
                {
                    ((*c as f64 / 1.0e-4).round() as i64).hash(&mut h);
                }
            }
            h.finish()
        };
        if let Some((cached_key, mesh)) = self.edge_mod_preview_mesh_cache.as_ref() {
            if *cached_key == key {
                return Some(mesh.clone());
            }
        }
        let mesh = op.immediate_preview_mesh();
        if mesh.vertices.is_empty() {
            self.edge_mod_preview_mesh_cache = None;
            None
        } else {
            self.edge_mod_preview_mesh_cache = Some((key, mesh.clone()));
            Some(mesh)
        }
    }

    /// Commit the live edge mod into history as a real `EdgeMod` feature, binding
    /// the size to a variable expression when the text references one.
    pub(crate) fn commit_edge_mod(&mut self) {
        if let Some(operation) = self
            .edge_mod_op
            .as_ref()
            .filter(|operation| operation.strict_all_edges)
        {
            let key = Self::edge_mod_arc_key(operation, &self.hidden_nodes);
            let exact = self
                .edge_mod_arc_cache
                .as_ref()
                .filter(|(cached_key, _, _)| *cached_key == key)
                .or_else(|| {
                    self.edge_mod_arc_lru
                        .iter()
                        .find(|(cached_key, _, _)| *cached_key == key)
                });
            let Some((_, _, warnings)) = exact else {
                self.status_msg =
                    "Still checking every edge; wait for the exact preview before committing."
                        .to_string();
                return;
            };
            if !warnings.is_empty() {
                let noun = match operation.kind {
                    CornerKind::Fillet => "Fillet",
                    CornerKind::Chamfer => "Chamfer",
                };
                self.status_msg = format!("All-edge {noun} rejected atomically. {}", warnings[0]);
                return;
            }
        }
        let Some(persisted_edges) = self
            .edge_mod_op
            .as_ref()
            .and_then(EdgeModOp::persisted_edges)
        else {
            self.status_msg =
                "The durable edge selection could not be encoded; the model was not changed."
                    .to_string();
            return;
        };
        // Resolve preview state first to avoid borrow-check conflicts
        let cached_bodies = self.cached_preview_edge_mod_bodies();
        let exact_bodies = cached_bodies.is_some();
        let bodies = cached_bodies
            .map(|bodies| (*bodies).clone())
            .unwrap_or_else(|| (*self.body_meshes).clone());
        let mesh = self.cached_preview_edge_mod_mesh();

        let Some(op) = self.edge_mod_op.take() else {
            return;
        };

        // Capture pending visual
        self.pending_visual = Some(PendingCommitVisual {
            bodies,
            mesh,
            mode: PendingVisualMode::EdgeMod,
            exact_bodies,
        });
        // Key the speculative precompute before `op`'s fields are moved below.
        let arc_key = Self::edge_mod_arc_key(&op, &self.hidden_nodes);
        self.push_undo();
        let dist_expr = if zerocad_core::expr::preserves_source(&op.dist_text) {
            Some(op.dist_text.trim().to_string())
        } else {
            None
        };
        // Explicit selection keeps one feature per edge. Strict whole-body mode
        // contributes exactly one selector-backed feature, preventing partial
        // history on rebuild or reload.
        let dist = op.dist.max(0.2);
        let edge_count = op.display_edges.len();
        let mut prev = op.target.clone();
        for edge in persisted_edges {
            let id = format!("edgemod_{}", self.next_id());
            let name = self.next_edge_mod_name(op.kind);
            self.document.add_feature(FeatureNode {
                id: id.clone(),
                name,
                feature: FeatureType::EdgeMod {
                    target: op.target.clone(),
                    edge,
                    dist,
                    dist_expr: dist_expr.clone(),
                    kind: op.kind,
                },
            });
            self.document.add_dependency(&prev, &id);
            prev = id;
        }
        // Remember the size for the next edge.
        self.edge_mod_dist_text = op.dist_text;
        self.selected_body.clear();
        self.selected_edges.clear();
        self.selected_sketch_points.clear();
        // If the smooth one-face arc geometry for this exact size was already
        // computed while the user was adjusting it, apply it instantly — no
        // faceted-then-arc "pop" a second later. Otherwise fall back to the normal
        // path (instant faceted draft + background arc refine).
        let precomputed = match self.edge_mod_arc_cache.take() {
            Some((k, bodies, warnings)) if k == arc_key && warnings.is_empty() => {
                Some((bodies, warnings))
            }
            _ => None,
        };
        let applied_immediately = if let Some((bodies, warnings)) = precomputed {
            // The speculative bodies were meshed at the coarse preview budget:
            // show them instantly (no faceted-draft flash, no wait), then refine
            // to full-quality tessellation in the background. `spawn_refine_eval`
            // bumps the generation, so any older in-flight refine is superseded.
            self.apply_eval_result((*bodies).clone(), warnings, false);
            self.spawn_refine_eval();
            true
        } else {
            self.spawn_refine_eval();
            self.eval_pending = true;
            false
        };
        self.clear_edge_mod_speculation();
        let noun = match op.kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        if !applied_immediately {
            self.status_msg = format!("Applying {} to {} edge(s)…", noun, edge_count);
        } else if self.error_msg.is_none() {
            self.status_msg = format!("{} applied to {} edge(s).", noun, edge_count);
        } else {
            self.status_msg = format!("{} couldn't be applied (see message).", noun);
        }
    }

    /// Abort the live edge mod, leaving the body untouched.
    pub(crate) fn cancel_edge_mod(&mut self) {
        if self.edge_mod_op.take().is_some() {
            self.status_msg = "Fillet/Chamfer cancelled.".to_string();
        }
        self.clear_edge_mod_speculation();
    }

    /// The Fusion-style floating size box for the live edge mod: an editable
    /// value (number / variable / expression) with a unit suffix, a Fillet ↔
    /// Chamfer toggle, and OK / Cancel. Enter commits, Esc cancels. Anchored at
    /// the edge midpoint (`edge_mod_dim_pos`, projected in the renderer).
    pub(crate) fn show_edge_mod_dialog(&mut self, ctx: &egui::Context) {
        if self.edge_mod_op.is_none() {
            return;
        }

        let unit_suffix = self.current_unit.suffix();
        let var_names = self.visible_variable_names();
        let varmap = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        let mut suppress_commit = false;
        // Set by the OK / Cancel buttons; applied after the borrow of `self.edge_mod_op` ends.
        let mut do_commit = false;
        let mut do_cancel = false;

        let pos = self
            .edge_mod_dim_pos
            .unwrap_or_else(|| ctx.screen_rect().center());

        egui::Area::new(egui::Id::new("edgemod_inline"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(3.0)
                    .stroke(egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgb(170, 180, 190),
                    ))
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 2.0),
                        blur: 8.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(35),
                    })
                    .inner_margin(egui::Margin::symmetric(8.0, 5.0))
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                        ui.horizontal(|ui| {
                            if let Some(op) = self.edge_mod_op.as_mut() {
                                ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                                ui.style_mut().visuals.widgets.inactive.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.widgets.hovered.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.selection.bg_fill =
                                    egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);

                                let field_id = egui::Id::new("edgemod_size_field");
                                let outcome = crate::expr::autocomplete_field(
                                    ui,
                                    field_id,
                                    &mut op.dist_text,
                                    56.0,
                                    true,
                                    op.focus_request,
                                    op.focus_request,
                                    &var_names,
                                    &mut ac,
                                );
                                let resp = outcome.response;
                                op.focus_request = false;
                                if outcome.accepted_via_key {
                                    suppress_commit = true;
                                }
                                if resp.has_focus() || resp.changed() {
                                    if let Ok(v) = crate::expr::eval(&op.dist_text, &varmap) {
                                        op.dist = (v as f32).clamp(0.05, 300.0);
                                    }
                                } else if !zerocad_core::expr::preserves_source(&op.dist_text) {
                                    op.dist_text = format!("{:.2}", op.dist);
                                }
                                if resp.has_focus() {
                                    let r = resp.rect;
                                    ui.painter().line_segment(
                                        [r.left_bottom(), r.right_bottom()],
                                        egui::Stroke::new(
                                            1.5,
                                            egui::Color32::from_rgb(0, 120, 215),
                                        ),
                                    );
                                }
                                ui.label(
                                    egui::RichText::new(unit_suffix)
                                        .color(egui::Color32::from_rgb(110, 110, 110))
                                        .size(12.0),
                                );
                            }
                        });

                        // Fillet ↔ Chamfer toggle.
                        if let Some(op) = self.edge_mod_op.as_ref() {
                            if zerocad_core::expr::preserves_source(&op.dist_text) {
                                if let Ok(value) = crate::expr::eval(&op.dist_text, &varmap) {
                                    ui.label(
                                        egui::RichText::new(format!("= {value:.2} {unit_suffix}"))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(70, 120, 70)),
                                    );
                                }
                            }
                        }
                        ui.add_space(5.0);
                        if let Some(op) = self.edge_mod_op.as_mut() {
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(3.0, 0.0);
                                for (kind, label) in [
                                    (CornerKind::Fillet, "Fillet"),
                                    (CornerKind::Chamfer, "Chamfer"),
                                ] {
                                    let selected = op.kind == kind;
                                    let (fill, text) = if selected {
                                        (egui::Color32::from_rgb(0, 120, 215), egui::Color32::WHITE)
                                    } else {
                                        (
                                            egui::Color32::from_rgb(238, 241, 245),
                                            egui::Color32::from_rgb(70, 75, 82),
                                        )
                                    };
                                    let btn = ui.add(
                                        egui::Button::new(
                                            egui::RichText::new(label).color(text).size(11.5),
                                        )
                                        .fill(fill)
                                        .rounding(3.0)
                                        .min_size(egui::vec2(56.0, 20.0)),
                                    );
                                    if btn.clicked() {
                                        op.kind = kind;
                                    }
                                }
                            });
                        }

                        // OK / Cancel.
                        ui.add_space(5.0);
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing = egui::vec2(3.0, 0.0);
                            let ok = ui.add(
                                egui::Button::new(
                                    egui::RichText::new("✓ OK")
                                        .color(egui::Color32::WHITE)
                                        .size(11.5),
                                )
                                .fill(egui::Color32::from_rgb(34, 139, 84))
                                .rounding(3.0)
                                .min_size(egui::vec2(56.0, 20.0)),
                            );
                            if ok.clicked() {
                                do_commit = true;
                            }
                            let cancel = ui.add(
                                egui::Button::new(
                                    egui::RichText::new("Cancel")
                                        .color(egui::Color32::from_rgb(70, 75, 82))
                                        .size(11.5),
                                )
                                .fill(egui::Color32::from_rgb(238, 241, 245))
                                .rounding(3.0)
                                .min_size(egui::vec2(56.0, 20.0)),
                            );
                            if cancel.clicked() {
                                do_cancel = true;
                            }
                        });
                    });
            });

        let enter = ctx.input(|i| i.key_pressed(egui::Key::Enter));
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        self.autocomplete = ac;

        if do_cancel || escape {
            self.cancel_edge_mod();
        } else if do_commit || (enter && !suppress_commit) {
            self.commit_edge_mod();
        }
    }

    /// The Fusion-style drag manipulator for the live edge mod: a handle offset
    /// from the edge along its outward bisector, joined to the edge by a guide
    /// line. Dragging it along that axis grows/shrinks the fillet/chamfer live —
    /// the drag's pixels are converted back to millimetres via the axis's
    /// px-per-mm length (`d·axis / |axis|²`), so it tracks the cursor 1:1 in the
    /// view. Both the handle and the typed size box edit the same `dist`.
    pub(crate) fn drag_edge_mod_handle(&mut self, ctx: &egui::Context) {
        if self.edge_mod_op.is_none() {
            return;
        }
        let Some((mid, hpos, axis)) = self.edge_mod_handle else {
            return;
        };
        let len2 = axis.length_sq();
        let r = 7.0;

        egui::Area::new(egui::Id::new("edgemod_handle"))
            .order(egui::Order::Foreground)
            .fixed_pos(hpos - egui::vec2(r, r))
            .show(ctx, |ui| {
                // Draw across the whole viewport, not just the handle's tiny rect.
                ui.set_clip_rect(ctx.screen_rect());
                let (_rect, resp) =
                    ui.allocate_exact_size(egui::vec2(r * 2.0, r * 2.0), egui::Sense::drag());
                let painter = ui.painter();

                let active = resp.hovered() || resp.dragged();
                let accent = if active {
                    egui::Color32::from_rgb(0, 120, 215)
                } else {
                    egui::Color32::from_rgb(255, 140, 0)
                };
                // Guide line from the edge to the handle, then the grab dot.
                painter.line_segment([mid, hpos], egui::Stroke::new(1.5, accent));
                painter.circle_filled(hpos, r, accent);
                painter.circle_stroke(hpos, r, egui::Stroke::new(1.5, egui::Color32::WHITE));

                if resp.dragged() && len2 > 1.0e-6 {
                    let d = resp.drag_delta();
                    let delta_mm = (d.x * axis.x + d.y * axis.y) / len2;
                    if let Some(op) = self.edge_mod_op.as_mut() {
                        op.dist = (op.dist + delta_mm).clamp(0.05, 300.0);
                        op.dist_text = format!("{:.2}", op.dist);
                        // The handle owns the value while dragging — don't let the
                        // size box yank focus and re-seed it.
                        op.focus_request = false;
                    }
                }
                resp.on_hover_cursor(egui::CursorIcon::ResizeHorizontal);
            });
    }
}
