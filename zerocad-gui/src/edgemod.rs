//! The 3D edge fillet/chamfer tool: a live (uncommitted) edge modification with
//! a real-time preview of the rounded/beveled body and a Fusion-style floating
//! inline size box, committed into the parametric graph on confirm.
//!
//! It mirrors the extrude tool's shape ([`crate::extrude`]): an in-progress op
//! drives an immediate lightweight mesh, an exact worker-computed preview, and an
//! inline dialog ([`ZeroCadApp::show_edge_mod_dialog`]) until the user commits.

use eframe::egui;
use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{
    CornerKind, EdgeModReplayIntent, EdgeModScope, EdgeRef, FeatureNode, FeatureType, MockMesh,
};

use crate::ZeroCadApp;

/// How long an edge-mod size must hold steady before its preview geometry is
/// computed on a worker thread after the first instant solve (see
/// [`ZeroCadApp::tick_speculative_edge_mod`]). Short enough to be ready by the
/// time the user reaches for OK, long enough that a fast drag through many sizes
/// doesn't spawn a job per step.
const EDGE_MOD_SETTLE: std::time::Duration = std::time::Duration::from_millis(160);
const EDGE_MOD_PREVIEW_FILLET_SEGS: usize = 8;

fn v_add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn v_scale(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
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

fn edge_mod_edge_preview_mesh(edge: &EdgeRef, dist: f32, kind: CornerKind) -> Option<MockMesh> {
    if !matches!(edge.curve, None | Some(EdgeCurveHint::Line)) {
        return None;
    }

    let n1 = v_norm(edge.n1)?;
    let n2 = v_norm(edge.n2)?;
    let dist = dist.max(0.05);
    let mut rails: Vec<([f32; 3], [f32; 3])> = Vec::new();
    match kind {
        CornerKind::Chamfer => {
            let normal = v_norm(v_add(n1, n2)).unwrap_or(n1);
            rails.push((v_scale(n1, -dist), normal));
            rails.push((v_scale(n2, -dist), normal));
        }
        CornerKind::Fillet => {
            let center_offset = v_add(v_scale(n1, -dist), v_scale(n2, -dist));
            for i in 0..=EDGE_MOD_PREVIEW_FILLET_SEGS {
                let theta =
                    i as f32 / EDGE_MOD_PREVIEW_FILLET_SEGS as f32 * std::f32::consts::FRAC_PI_2;
                let dir = v_norm(v_add(v_scale(n2, theta.cos()), v_scale(n1, theta.sin())))?;
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

/// A live, uncommitted 3D edge fillet/chamfer. Holds the captured edge geometry
/// and the editable size; the viewport shows the resulting body in real time.
///
/// One op can round/bevel **several** selected edges at once (Fusion's multi-edge
/// fillet): they are applied as a chain of single-edge `EdgeMod` features, each
/// re-locating its edge on the evolving body, so edges that share a corner blend
/// correctly. The inline size box and drag handle anchor on the first edge.
#[derive(Debug, Clone)]
pub(crate) struct EdgeModOp {
    /// Node id of the body being modified.
    pub(crate) target: String,
    /// The edges being rounded/beveled, captured in world space. Always non-empty.
    pub(crate) edges: Vec<EdgeRef>,
    /// Replay intent captured at selection time, one entry per edge.
    pub(crate) replay: Vec<EdgeModReplayIntent>,
    /// Fillet (round) or Chamfer (bevel).
    pub(crate) kind: CornerKind,
    /// Resolved size in base units (mm), kept in sync with `dist_text`.
    pub(crate) dist: f32,
    /// Editable text buffer for the inline size box (a number or a variable
    /// expression).
    pub(crate) dist_text: String,
    /// True until the inline box has grabbed keyboard focus once.
    pub(crate) focus_request: bool,
}

impl EdgeModOp {
    /// The primary (first) edge — the anchor for the inline box and drag handle.
    pub(crate) fn primary(&self) -> &EdgeRef {
        &self.edges[0]
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
        for edge in &self.edges {
            if let Some(edge_mesh) = edge_mod_edge_preview_mesh(edge, self.dist, self.kind) {
                mesh.append(edge_mesh);
            }
        }
        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        EdgeModOp {
            target: "body".to_string(),
            edges: vec![straight_box_edge()],
            replay: vec![EdgeModReplayIntent::default()],
            kind,
            dist,
            dist_text: format!("{dist:.2}"),
            focus_request: false,
        }
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
        let Some((node_id, edge_ids)) = self.selected_body_edges() else {
            self.status_msg = "Select one or more body edges first.".to_string();
            return;
        };
        let edges: Vec<EdgeRef> = edge_ids
            .iter()
            .filter_map(|&e| self.edge_ref_from(&node_id, e))
            .collect();
        if edges.is_empty() {
            self.status_msg = "Those edges have no usable geometry to fillet/chamfer.".to_string();
            return;
        }
        let replay = edges
            .iter()
            .map(|edge| {
                self.graph
                    .edge_mod_replay_intent_for_edge(&node_id, edge, &self.hidden_nodes)
            })
            .collect();
        let text = self.edge_mod_dist_text.clone();
        let dist = self.eval_dim(&text).unwrap_or(3.0).max(0.2);
        self.edge_mod_op = Some(EdgeModOp {
            target: node_id,
            edges,
            replay,
            kind,
            dist,
            dist_text: text,
            focus_request: true,
        });
        // Start each edit with a clean speculative edge-mod slate so a stale
        // precompute from a previous edit can't be mistaken for this one.
        self.clear_edge_mod_speculation();
        self.status_msg = "Set the size, then Enter / OK to apply (Esc cancels).".to_string();
    }

    /// Reset all speculative edge-mod precompute state (cache, in-flight job,
    /// debounce). Any worker thread still running harmlessly sends into a dropped
    /// channel. Called when an edit begins, commits, or is cancelled.
    pub(crate) fn clear_edge_mod_speculation(&mut self) {
        self.edge_mod_arc_cache = None;
        self.edge_mod_arc_inflight = None;
        self.edge_mod_arc_rx = None;
        self.edge_mod_settle = None;
        self.edge_mod_preview_mesh_cache = None;
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

    fn hash_replay_intent(h: &mut impl std::hash::Hasher, replay: &EdgeModReplayIntent) {
        use std::hash::Hash;
        (replay.mode as u8).hash(h);
        replay.pre_cut_target.hash(h);
        replay.replay_cut_nodes.hash(h);
        if let Some(edge) = replay.selected_span.as_ref() {
            1u8.hash(h);
            Self::hash_edge_ref(h, edge);
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
        op.target.hash(&mut h);
        for edge in &op.edges {
            Self::hash_edge_ref(&mut h, edge);
        }
        for replay in &op.replay {
            Self::hash_replay_intent(&mut h, replay);
        }
        let mut hidden: Vec<&String> = hidden_nodes.iter().collect();
        hidden.sort();
        for id in hidden {
            id.hash(&mut h);
        }
        h.finish()
    }

    /// Append the op's edges as a chain of single-edge `EdgeMod` nodes onto a
    /// cloned graph — each depending on the previous so they apply in order, each
    /// at size `dist`. Temp ids are suffixed `id_counter + i` so `creation_key`
    /// orders them after every committed node and in edge order. The kernel
    /// re-locates each edge on the evolving body, so edges sharing a corner blend
    /// correctly. Used by both the speculative and live-preview graphs.
    fn append_edge_mod_chain(
        &self,
        graph: &mut zerocad_core::ParametricGraph,
        op: &EdgeModOp,
        dist: f32,
        tag: &str,
    ) {
        let mut prev = op.target.clone();
        for (i, edge) in op.edges.iter().enumerate() {
            let id = format!("edgemod_{tag}_{}", self.id_counter + i);
            let replay = op.replay.get(i).cloned().unwrap_or_else(|| {
                graph.edge_mod_replay_intent_for_edge(&op.target, edge, &self.hidden_nodes)
            });
            graph.add_feature(FeatureNode {
                id: id.clone(),
                name: format!("{tag} edge mod {i}"),
                feature: FeatureType::EdgeMod {
                    target: op.target.clone(),
                    edge: edge.clone(),
                    dist,
                    dist_expr: None,
                    scope: EdgeModScope::default(),
                    replay,
                    kind: op.kind,
                },
            });
            graph.add_dependency(&prev, &id);
            prev = id;
        }
    }

    /// Build the graph the speculative precompute evaluates: the current model
    /// plus the live edit as real `EdgeMod` nodes, using the same `dist.max(0.2)`
    /// the commit will. Evaluated on a worker thread, it yields exactly the bodies
    /// a commit at this size would. Bodies key by `target`, not the node id, so
    /// this matches the committed result despite the throwaway node name.
    fn build_edge_mod_arc_graph(&self) -> Option<zerocad_core::ParametricGraph> {
        let op = self.edge_mod_op.as_ref()?;
        let mut graph = self.graph.clone();
        self.append_edge_mod_chain(&mut graph, op, op.dist.max(0.2), "spec");
        Some(graph)
    }

    fn spawn_edge_mod_arc_eval(&mut self, ctx: &egui::Context, key: u64) {
        let Some(graph) = self.build_edge_mod_arc_graph() else {
            return;
        };
        let hidden = self.hidden_nodes.clone();
        let ctx = ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        self.edge_mod_arc_rx = Some(rx);
        self.edge_mod_arc_inflight = Some(key);
        std::thread::spawn(move || {
            let result = graph.evaluate_bodies_with_warnings(&hidden);
            let _ = tx.send((key, result));
            ctx.request_repaint();
        });
    }

    /// Drive the speculative edge-mod precompute. Called once per frame. The
    /// first exact solve starts immediately so the preview can refine as soon as
    /// possible; later size changes wait for [`EDGE_MOD_SETTLE`] before spawning
    /// another worker job. At most one job runs at a time.
    pub(crate) fn tick_speculative_edge_mod(&mut self, ctx: &egui::Context) {
        // Drain a finished job into the cache first.
        if let Some(rx) = self.edge_mod_arc_rx.as_ref() {
            match rx.try_recv() {
                Ok((key, result)) => {
                    self.edge_mod_arc_rx = None;
                    self.edge_mod_arc_inflight = None;
                    if let Ok((bodies, warnings)) = result {
                        if warnings.is_empty() {
                            self.edge_mod_arc_cache = Some((key, bodies, warnings));
                        } else {
                            // Speculative graph nodes use temporary ids like
                            // `edgemod_spec_*`. Failed previews must not leak
                            // those warnings into the real document or be reused
                            // on OK; the committed feature will evaluate with its
                            // stable id if the user applies it.
                            self.edge_mod_arc_cache = None;
                        }
                    }
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.edge_mod_arc_rx = None;
                    self.edge_mod_arc_inflight = None;
                }
            }
        }

        let Some(op) = self.edge_mod_op.as_ref() else {
            return;
        };
        let key = Self::edge_mod_arc_key(op, &self.hidden_nodes);

        // Already computed (or computing) the arc for this exact size.
        if matches!(&self.edge_mod_arc_cache, Some((k, _, _)) if *k == key) {
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
            // receiver so the stale worker result cannot populate the cache, and
            // let the current key schedule normally below.
            self.edge_mod_arc_rx = None;
            self.edge_mod_arc_inflight = None;
        }

        if self.edge_mod_settle.is_none() {
            self.edge_mod_settle = Some((key, std::time::Instant::now()));
            self.spawn_edge_mod_arc_eval(ctx, key);
            return;
        }

        // Debounce: wait until this size has been stable for EDGE_MOD_SETTLE before
        // spending a ~1s solve on it.
        let settled_at = match self.edge_mod_settle {
            Some((k, t)) if k == key => t,
            _ => {
                self.edge_mod_settle = Some((key, std::time::Instant::now()));
                ctx.request_repaint_after(EDGE_MOD_SETTLE);
                return;
            }
        };
        let waited = settled_at.elapsed();
        if waited < EDGE_MOD_SETTLE {
            ctx.request_repaint_after(EDGE_MOD_SETTLE - waited);
            return;
        }
        // Only one speculative job at a time; if one's busy on an older size, let
        // it finish — the next tick will spawn this size once the slot frees.
        if self.edge_mod_arc_inflight.is_some() {
            return;
        }

        self.spawn_edge_mod_arc_eval(ctx, key);
    }

    /// Synchronous exact preview builder retained for diagnostics. Interactive
    /// preview uses [`EdgeModOp::immediate_preview_mesh`] immediately and refines
    /// through [`tick_speculative_edge_mod`](Self::tick_speculative_edge_mod).
    #[allow(dead_code)]
    pub(crate) fn preview_edge_mod_bodies(&self) -> Option<Vec<(String, MockMesh)>> {
        let op = self.edge_mod_op.as_ref()?;
        let mut graph = self.graph.clone();
        // Temp ids past the live counter — never persisted.
        self.append_edge_mod_chain(&mut graph, op, op.dist.max(0.05), "preview");
        // The preview fillet is appended as a trailing node, so the parametric
        // graph's per-node geometry cache (carried by the clone above) reuses the
        // committed prefix — the upstream booleans (e.g. a box∪boss union) are NOT
        // re-solved each frame; only this one edge-mod runs. The native rolling-ball
        // fillet is exact, so draft and commit already match (the `draft` flag is a
        // no-op); it is kept only for API symmetry with the extrude preview.
        graph.evaluate_bodies_draft(&self.hidden_nodes).ok()
    }

    /// Memoized [`preview_edge_mod_bodies`]. egui repaints continuously while the
    /// inline size box is focused or the handle is dragged. This GUI-level cache
    /// recomputes only when the size (quantized to a sub-visible step), kind, or
    /// target change, so idle frames and the still points of a slow drag are free;
    /// the parametric graph's per-node cache then keeps the recompute cheap on the
    /// frames that *do* change (the upstream booleans are reused, only the fillet
    /// re-runs). Mirrors `cached_preview_extrude_bodies`.
    pub(crate) fn cached_preview_edge_mod_bodies(&mut self) -> Option<Vec<(String, MockMesh)>> {
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
        let key = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            // Quantize size to 0.05mm: idle frames and slow drags reuse the cache,
            // and a sub-0.05mm preview lag is invisible (commit uses the exact size).
            ((op.dist / 0.05).round() as i64).hash(&mut h);
            (op.kind as u8).hash(&mut h);
            op.target.hash(&mut h);
            // The edges themselves — two edges of the same body share `target`, so
            // without this a fillet on edge B could reuse edge A's cached result.
            for edge in &op.edges {
                Self::hash_edge_ref(&mut h, edge);
            }
            for replay in &op.replay {
                Self::hash_replay_intent(&mut h, replay);
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
            for edge in &op.edges {
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
        let Some(op) = self.edge_mod_op.take() else {
            return;
        };
        // Key the speculative precompute before `op`'s fields are moved below.
        let arc_key = Self::edge_mod_arc_key(&op, &self.hidden_nodes);
        self.push_undo();
        let dist_expr = if zerocad_core::expr::references_variable(&op.dist_text) {
            Some(op.dist_text.trim().to_string())
        } else {
            None
        };
        // One single-edge `EdgeMod` feature per selected edge, chained so they
        // apply in order. The kernel re-locates each edge on the evolving body, so
        // edges sharing a corner blend correctly (the earlier blend shortens the
        // survivor, which `fillet_edges` tracks).
        let dist = op.dist.max(0.2);
        let edge_count = op.edges.len();
        let mut prev = op.target.clone();
        let replays = op.replay;
        for (i, edge) in op.edges.into_iter().enumerate() {
            let id = format!("edgemod_{}", self.next_id());
            let name = self.next_edge_mod_name(op.kind);
            let replay = replays.get(i).cloned().unwrap_or_else(|| {
                self.graph
                    .edge_mod_replay_intent_for_edge(&op.target, &edge, &self.hidden_nodes)
            });
            self.graph.add_feature(FeatureNode {
                id: id.clone(),
                name,
                feature: FeatureType::EdgeMod {
                    target: op.target.clone(),
                    edge,
                    dist,
                    dist_expr: dist_expr.clone(),
                    scope: EdgeModScope::default(),
                    replay,
                    kind: op.kind,
                },
            });
            self.graph.add_dependency(&prev, &id);
            prev = id;
        }
        // Remember the size for the next edge.
        self.edge_mod_dist_text = op.dist_text;
        self.selected_body.clear();
        self.selected_edges.clear();
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
            // Supersede any in-flight refine so its late result can't clobber this.
            self.eval_gen += 1;
            self.eval_rx = None;
            self.eval_pending = false;
            self.apply_eval_result(bodies, warnings);
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
                        extrusion: 8.0,
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
                                } else {
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
