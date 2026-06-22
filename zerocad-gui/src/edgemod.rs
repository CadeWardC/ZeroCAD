//! The 3D edge fillet/chamfer tool: a live (uncommitted) edge modification with
//! a real-time preview of the rounded/beveled body and a Fusion-style floating
//! inline size box, committed into the parametric graph on confirm.
//!
//! It mirrors the extrude tool's shape ([`crate::extrude`]): an in-progress op
//! drives a temp-node preview ([`ZeroCadApp::preview_edge_mod_bodies`]) and an
//! inline dialog ([`ZeroCadApp::show_edge_mod_dialog`]) until the user commits.

use eframe::egui;
use zerocad_core::{CornerKind, EdgeRef, FeatureNode, FeatureType, MockMesh};

use crate::ZeroCadApp;

/// How long a fillet size must hold steady before its slow analytic-arc geometry
/// is precomputed on a worker thread (see [`ZeroCadApp::tick_speculative_edge_mod`]).
/// Short enough to be ready by the time the user reaches for OK, long enough that
/// a fast drag through many sizes doesn't spawn a job per step.
const EDGE_MOD_SETTLE: std::time::Duration = std::time::Duration::from_millis(150);

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
            self.status_msg =
                "Those edges have no usable geometry to fillet/chamfer.".to_string();
            return;
        }
        let text = self.edge_mod_dist_text.clone();
        let dist = self.eval_dim(&text).unwrap_or(3.0).max(0.2);
        self.edge_mod_op = Some(EdgeModOp {
            target: node_id,
            edges,
            kind,
            dist,
            dist_text: text,
            focus_request: true,
        });
        // Start each edit with a clean speculative-arc slate so a stale precompute
        // from a previous fillet can't be mistaken for this one.
        self.clear_edge_mod_speculation();
        self.status_msg =
            "Set the size, then Enter / OK to apply (Esc cancels).".to_string();
    }

    /// Reset all speculative arc-fillet precompute state (cache, in-flight job,
    /// debounce). Any worker thread still running harmlessly sends into a dropped
    /// channel. Called when an edit begins, commits, or is cancelled.
    pub(crate) fn clear_edge_mod_speculation(&mut self) {
        self.edge_mod_arc_cache = None;
        self.edge_mod_arc_inflight = None;
        self.edge_mod_arc_rx = None;
        self.edge_mod_settle = None;
    }

    /// Hash of everything that determines a fillet's committed arc geometry — the
    /// size (quantized to 0.01mm, finer than the faceted preview's 0.05mm so the
    /// precompute matches the exact committed size), kind, target body, edge, and
    /// the hidden set. [`commit_edge_mod`](Self::commit_edge_mod) recomputes this
    /// to decide whether the speculative result applies, and
    /// [`tick_speculative_edge_mod`](Self::tick_speculative_edge_mod) uses it both
    /// to debounce and to tag the job.
    fn edge_mod_arc_key(op: &EdgeModOp, hidden_len: usize) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        ((op.dist.max(0.2) / 0.01).round() as i64).hash(&mut h);
        (op.kind as u8).hash(&mut h);
        op.target.hash(&mut h);
        for edge in &op.edges {
            for c in edge.p0.iter().chain(edge.p1.iter()) {
                ((*c as f64 / 1.0e-4).round() as i64).hash(&mut h);
            }
        }
        hidden_len.hash(&mut h);
        h.finish()
    }

    /// Append the op's edges as a chain of single-edge `EdgeMod` nodes onto a
    /// cloned graph — each depending on the previous so they apply in order, each
    /// at size `dist`. Temp ids are suffixed `id_counter + i` so `creation_key`
    /// orders them after every committed node and in edge order. The kernel
    /// re-locates each edge on the evolving body, so edges sharing a corner blend
    /// correctly. Used by both the speculative-arc and live-preview graphs.
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
            graph.add_feature(FeatureNode {
                id: id.clone(),
                name: format!("{tag} edge mod {i}"),
                feature: FeatureType::EdgeMod {
                    target: op.target.clone(),
                    edge: edge.clone(),
                    dist,
                    dist_expr: None,
                    kind: op.kind,
                },
            });
            graph.add_dependency(&prev, &id);
            prev = id;
        }
    }

    /// Build the graph the speculative precompute evaluates: the current model
    /// plus the live fillet as a real `EdgeMod` node, using the same `dist.max(0.2)`
    /// the commit will. Evaluated **non-draft** (arc cutter) on a worker thread, it
    /// yields exactly the bodies a commit at this size would — the round becomes one
    /// cylindrical B-rep face. Bodies key by `target`, not the node id, so this
    /// matches the committed result despite the throwaway node name.
    fn build_edge_mod_arc_graph(&self) -> Option<zerocad_core::ParametricGraph> {
        let op = self.edge_mod_op.as_ref()?;
        let mut graph = self.graph.clone();
        self.append_edge_mod_chain(&mut graph, op, op.dist.max(0.2), "spec");
        Some(graph)
    }

    /// Drive the speculative arc-fillet precompute. Called once per frame. While a
    /// fillet is being edited, the moment its size has held steady for
    /// [`EDGE_MOD_SETTLE`] this spawns the slow analytic-arc evaluation for that
    /// size on a worker thread and caches the result, so committing at that size
    /// applies the smooth one-face geometry instantly instead of showing the
    /// faceted draft and swapping the arc in ~1s later. At most one job runs at a
    /// time; chamfers (already one planar face) and the no-edit case do nothing.
    pub(crate) fn tick_speculative_edge_mod(&mut self, ctx: &egui::Context) {
        // Drain a finished job into the cache first.
        if let Some(rx) = self.edge_mod_arc_rx.as_ref() {
            match rx.try_recv() {
                Ok((key, result)) => {
                    self.edge_mod_arc_rx = None;
                    self.edge_mod_arc_inflight = None;
                    if let Ok((bodies, warnings)) = result {
                        self.edge_mod_arc_cache = Some((key, bodies, warnings));
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
        // A chamfer's bevel is a single planar face in draft already — nothing
        // slower to precompute.
        if !matches!(op.kind, CornerKind::Fillet) {
            return;
        }
        let key = Self::edge_mod_arc_key(op, self.hidden_nodes.len());

        // Already computed (or computing) the arc for this exact size.
        if matches!(&self.edge_mod_arc_cache, Some((k, _, _)) if *k == key) {
            return;
        }
        if self.edge_mod_arc_inflight == Some(key) {
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

    /// Evaluate the model as if the live edge mod had been committed, so the
    /// viewport shows the actual rounded/beveled body in real time. Mirrors
    /// [`preview_extrude_bodies`](Self::preview_extrude_bodies).
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
        let arc_key = Self::edge_mod_arc_key(op, self.hidden_nodes.len());
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
                for c in edge.p0.iter().chain(edge.p1.iter()) {
                    ((*c as f64 / 1.0e-4).round() as i64).hash(&mut h);
                }
            }
            self.id_counter.hash(&mut h);
            self.hidden_nodes.len().hash(&mut h);
            h.finish()
        };
        if let Some((cached_key, bodies)) = self.edge_mod_preview_cache.as_ref() {
            if *cached_key == key {
                return Some(bodies.clone());
            }
        }
        let bodies = self.preview_edge_mod_bodies();
        self.edge_mod_preview_cache = bodies.as_ref().map(|b| (key, b.clone()));
        bodies
    }

    /// Commit the live edge mod into history as a real `EdgeMod` feature, binding
    /// the size to a variable expression when the text references one.
    pub(crate) fn commit_edge_mod(&mut self) {
        let Some(op) = self.edge_mod_op.take() else {
            return;
        };
        // Key the speculative precompute before `op`'s fields are moved below.
        let arc_key = Self::edge_mod_arc_key(&op, self.hidden_nodes.len());
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
        for edge in op.edges {
            let id = format!("edgemod_{}", self.next_id());
            let name = self.next_edge_mod_name(op.kind);
            self.graph.add_feature(FeatureNode {
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
            Some((k, bodies, warnings)) if k == arc_key => Some((bodies, warnings)),
            _ => None,
        };
        if let Some((bodies, warnings)) = precomputed {
            // Supersede any in-flight refine so its late result can't clobber this.
            self.eval_gen += 1;
            self.eval_rx = None;
            self.eval_pending = false;
            self.apply_eval_result(bodies, warnings);
        } else {
            self.reevaluate_geometry();
        }
        self.clear_edge_mod_speculation();
        let noun = match op.kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        if self.error_msg.is_none() {
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
                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(170, 180, 190)))
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
                                        egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 120, 215)),
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
                                for (kind, label) in
                                    [(CornerKind::Fillet, "Fillet"), (CornerKind::Chamfer, "Chamfer")]
                                {
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
