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

/// A live, uncommitted 3D edge fillet/chamfer. Holds the captured edge geometry
/// and the editable size; the viewport shows the resulting body in real time.
#[derive(Debug, Clone)]
pub(crate) struct EdgeModOp {
    /// Node id of the body being modified.
    pub(crate) target: String,
    /// The edge being rounded/beveled, captured in world space.
    pub(crate) edge: EdgeRef,
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
    /// World-space midpoint of the edge — the anchor for the inline size box.
    pub(crate) fn edge_midpoint(&self) -> [f32; 3] {
        [
            (self.edge.p0[0] + self.edge.p1[0]) * 0.5,
            (self.edge.p0[1] + self.edge.p1[1]) * 0.5,
            (self.edge.p0[2] + self.edge.p1[2]) * 0.5,
        ]
    }
}

impl ZeroCadApp {
    /// Begin a live fillet/chamfer of the currently selected body edge. Seeds the
    /// size from `edge_mod_dist_text` (remembered across uses) and opens the
    /// preview; nothing is committed until [`commit_edge_mod`](Self::commit_edge_mod).
    pub(crate) fn begin_edge_mod(&mut self, kind: CornerKind) {
        let Some((node_id, e)) = self.selected_body_edge() else {
            self.status_msg = "Select a single body edge first.".to_string();
            return;
        };
        let Some(edge) = self.edge_ref_from(&node_id, e) else {
            self.status_msg = "That edge has no usable geometry to fillet/chamfer.".to_string();
            return;
        };
        let text = self.edge_mod_dist_text.clone();
        let dist = self.eval_dim(&text).unwrap_or(3.0).max(0.2);
        self.edge_mod_op = Some(EdgeModOp {
            target: node_id,
            edge,
            kind,
            dist,
            dist_text: text,
            focus_request: true,
        });
        self.status_msg =
            "Set the size, then Enter / OK to apply (Esc cancels).".to_string();
    }

    /// Evaluate the model as if the live edge mod had been committed, so the
    /// viewport shows the actual rounded/beveled body in real time. Mirrors
    /// [`preview_extrude_bodies`](Self::preview_extrude_bodies).
    pub(crate) fn preview_edge_mod_bodies(&self) -> Option<Vec<(String, MockMesh)>> {
        let op = self.edge_mod_op.as_ref()?;
        let mut graph = self.graph.clone();
        // A temp id past the live counter — never persisted.
        let id = format!("edgemod_preview_{}", self.id_counter);
        graph.add_feature(FeatureNode {
            id: id.clone(),
            name: "Preview Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: op.target.clone(),
                edge: op.edge.clone(),
                dist: op.dist.max(0.05),
                dist_expr: None,
                kind: op.kind,
            },
        });
        graph.add_dependency(&op.target, &id);
        graph.evaluate_bodies(&self.hidden_nodes).ok()
    }

    /// Commit the live edge mod into history as a real `EdgeMod` feature, binding
    /// the size to a variable expression when the text references one.
    pub(crate) fn commit_edge_mod(&mut self) {
        let Some(op) = self.edge_mod_op.take() else {
            return;
        };
        let dist_expr = if zerocad_core::expr::references_variable(&op.dist_text) {
            Some(op.dist_text.trim().to_string())
        } else {
            None
        };
        let id = format!("edgemod_{}", self.next_id());
        let name = self.next_edge_mod_name(op.kind);
        self.graph.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature: FeatureType::EdgeMod {
                target: op.target.clone(),
                edge: op.edge,
                dist: op.dist.max(0.2),
                dist_expr,
                kind: op.kind,
            },
        });
        self.graph.add_dependency(&op.target, &id);
        // Remember the size for the next edge.
        self.edge_mod_dist_text = op.dist_text;
        self.selected_body.clear();
        self.selected_edges.clear();
        self.reevaluate_geometry();
        let noun = match op.kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        if self.error_msg.is_none() {
            self.status_msg = format!("{} applied to the edge.", noun);
        } else {
            self.status_msg = format!("{} couldn't be applied (see message).", noun);
        }
    }

    /// Abort the live edge mod, leaving the body untouched.
    pub(crate) fn cancel_edge_mod(&mut self) {
        if self.edge_mod_op.take().is_some() {
            self.status_msg = "Fillet/Chamfer cancelled.".to_string();
        }
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
