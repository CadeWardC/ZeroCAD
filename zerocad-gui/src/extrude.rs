//! The extrude tool: the live (uncommitted) extrude operation and its preview,
//! plus committing it into the parametric graph and the inline distance dialog.

use std::collections::HashMap;

use eframe::egui;
use zerocad_core::{
    detect_regions, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, Region,
};

use crate::ZeroCadApp;

/// The default extrude mode for a freshly started op: a sketch on a body face
/// pulled **outward** (depth ≥ 0, along the outward face normal) adds material
/// (Join); pushed **inward** (depth < 0) removes it (Cut). A sketch on an origin
/// plane has no body to combine with, so it makes a new body.
pub(crate) fn default_extrude_mode(on_face: bool, depth: f32) -> ExtrudeMode {
    if !on_face {
        ExtrudeMode::NewBody
    } else if depth >= 0.0 {
        ExtrudeMode::Join
    } else {
        ExtrudeMode::Cut
    }
}

/// Faces from one sketch that participate in an extrude, plus the geometry
/// needed to build the preview mesh without touching the parametric graph.
#[derive(Debug, Clone)]
pub(crate) struct ExtrudeTarget {
    pub(crate) sketch_id: String,
    pub(crate) cs: CoordinateSystem,
    pub(crate) regions: Vec<Region>,
    pub(crate) indices: Vec<usize>,
    /// True when the source sketch sits on an existing body face (vs an origin
    /// plane). Drives the auto-chosen extrude mode (face → Join/Cut, plane →
    /// New Body).
    pub(crate) on_face: bool,
}

/// A live, uncommitted extrude. Drives the right-hand tool window and the
/// real-time push/pull preview (Fusion-style) until the user confirms.
#[derive(Debug, Clone)]
pub(crate) struct ExtrudeOp {
    pub(crate) targets: Vec<ExtrudeTarget>,
    pub(crate) depth: f32,
    /// Editable text buffer for the inline distance box. Kept in sync with
    /// `depth` (typing parses into `depth`; dragging reformats this).
    pub(crate) depth_text: String,
    /// True until the inline box has grabbed keyboard focus once.
    pub(crate) focus_request: bool,
    /// Whether this extrude makes a new body, joins existing bodies, or cuts.
    pub(crate) mode: ExtrudeMode,
    /// True when at least one source sketch is on a body face — enables the
    /// direction-driven default mode (pull out → Join, push in → Cut).
    pub(crate) on_face: bool,
    /// Set once the user clicks a mode button, which freezes the mode so the
    /// direction-driven default stops overriding their choice.
    pub(crate) mode_user_set: bool,
}

impl ExtrudeOp {
    /// Build the combined preview mesh for the current depth.
    pub(crate) fn preview_mesh(&self) -> MockMesh {
        let mut mesh = MockMesh::empty();
        if self.depth.abs() < f32::EPSILON {
            return mesh;
        }
        for t in &self.targets {
            for &ri in &t.indices {
                if let Some(r) = t.regions.get(ri) {
                    mesh.append(MockMesh::make_extruded_sketch(
                        &r.boundary,
                        &r.holes,
                        self.depth,
                        &t.cs,
                    ));
                }
            }
        }
        mesh
    }
}

impl ZeroCadApp {
    /// Evaluate the model as if the active extrude had already been committed.
    /// Used for live Cut/Join previews so the viewport shows the resulting body
    /// instead of an additive orange tool volume.
    pub(crate) fn preview_extrude_bodies(&self) -> Option<Vec<(String, MockMesh)>> {
        let op = self.extrude_op.as_ref()?;
        if op.depth.abs() < 0.01 {
            return Some(self.body_meshes.clone());
        }

        let mut graph = self.graph.clone();
        for (i, target) in op.targets.iter().enumerate() {
            if target.indices.is_empty() {
                continue;
            }

            let mut region_indices = target.indices.clone();
            region_indices.sort();
            region_indices.dedup();

            let extrude_id = format!("extrude_{}", self.id_counter + i);
            graph.add_feature(FeatureNode {
                id: extrude_id.clone(),
                name: format!("Preview Extrude {}", i + 1),
                feature: FeatureType::Extrude {
                    depth: op.depth,
                    region_indices,
                    mode: op.mode,
                    // The preview uses the already-resolved live depth; the
                    // committed node (in `build_extrude_body`) keeps the expr.
                    depth_expr: None,
                },
            });
            graph.add_dependency(&target.sketch_id, &extrude_id);
        }

        graph.evaluate_bodies(&self.hidden_nodes).ok()
    }

    /// Memoized [`preview_extrude_bodies`]. The underlying call clones the whole
    /// parametric graph and re-runs every truck boolean — far too slow to redo on
    /// every frame the Cut/Join dialog is open (egui repaints continuously while a
    /// field is focused or the depth is dragged). This caches the result and only
    /// recomputes when the depth (quantized to a sub-visible step), mode, or
    /// targets actually change, so idle frames and slow drags are nearly free.
    pub(crate) fn cached_preview_extrude_bodies(&mut self) -> Option<Vec<(String, MockMesh)>> {
        use std::hash::{Hash, Hasher};
        if self.extrude_op.is_none() {
            self.extrude_preview_cache = None;
            return None;
        }
        let key = {
            let op = self.extrude_op.as_ref().unwrap();
            let mut h = std::collections::hash_map::DefaultHasher::new();
            // Quantize depth to 0.05mm: identical/idle frames and slow drags reuse
            // the cache, and a sub-0.05mm preview lag is invisible (the commit
            // still uses the exact depth).
            ((op.depth / 0.05).round() as i64).hash(&mut h);
            let mode_id: u8 = match op.mode {
                ExtrudeMode::NewBody => 0,
                ExtrudeMode::Join => 1,
                ExtrudeMode::Cut => 2,
            };
            mode_id.hash(&mut h);
            for t in &op.targets {
                t.sketch_id.hash(&mut h);
                t.indices.hash(&mut h);
            }
            self.id_counter.hash(&mut h);
            self.hidden_nodes.len().hash(&mut h);
            h.finish()
        };
        if let Some((cached_key, bodies)) = self.extrude_preview_cache.as_ref() {
            if *cached_key == key {
                return Some(bodies.clone());
            }
        }
        let bodies = self.preview_extrude_bodies();
        self.extrude_preview_cache = bodies.as_ref().map(|b| (key, b.clone()));
        bodies
    }

    /// Memoized [`ExtrudeOp::preview_mesh`] (the tool ghost volume). Rebuilt only
    /// when the depth or targets change, so repaints that don't move the depth
    /// (mouse hovering, field focus) don't re-tessellate the tool every frame.
    pub(crate) fn cached_preview_mesh(&mut self) -> Option<MockMesh> {
        use std::hash::{Hash, Hasher};
        if self.extrude_op.is_none() {
            self.extrude_preview_mesh_cache = None;
            return None;
        }
        let key = {
            let op = self.extrude_op.as_ref().unwrap();
            let mut h = std::collections::hash_map::DefaultHasher::new();
            ((op.depth / 0.05).round() as i64).hash(&mut h);
            for t in &op.targets {
                t.sketch_id.hash(&mut h);
                t.indices.hash(&mut h);
            }
            h.finish()
        };
        if let Some((cached_key, mesh)) = self.extrude_preview_mesh_cache.as_ref() {
            if *cached_key == key {
                return Some(mesh.clone());
            }
        }
        let mesh = self.extrude_op.as_ref().unwrap().preview_mesh();
        self.extrude_preview_mesh_cache = Some((key, mesh.clone()));
        Some(mesh)
    }

    /// Inline extrude distance box — the same Fusion-style floating input used
    /// for sketch dimensions. Shows the live distance near the previewed body;
    /// typing a value or dragging in the viewport both update it. Enter / the OK
    /// button commit, Escape / Cancel abort.
    pub(crate) fn show_extrude_dialog(&mut self, ctx: &egui::Context) {
        if self.extrude_op.is_none() {
            return;
        }

        let unit_suffix = self.current_unit.suffix();
        // Pulled out of `self` so the field widget can borrow the variable list
        // and the shared autocomplete state while the closure holds `&mut self`.
        let var_names = self.visible_variable_names();
        let varmap = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        // Set when the autocomplete swallows an Enter (to accept a suggestion),
        // so we don't also treat that Enter as "commit the extrude".
        let mut suppress_commit = false;

        let pos = self
            .extrude_dim_pos
            .unwrap_or_else(|| ctx.screen_rect().center());

        // Fusion-style floating value field: just the editable distance, a unit
        // suffix, and a "⋮" affordance. Commit/cancel live on the right panel
        // and on Enter/Esc; this box only edits the value.
        egui::Area::new(egui::Id::new("extrude_inline"))
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
                            if let Some(op) = self.extrude_op.as_mut() {
                                ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                                ui.style_mut().visuals.widgets.inactive.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.widgets.hovered.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.selection.bg_fill =
                                    egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);

                                let field_id = egui::Id::new("extrude_depth_field");
                                let outcome = crate::expr::autocomplete_field(
                                    ui,
                                    field_id,
                                    &mut op.depth_text,
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
                                    // User is typing → evaluate the expression
                                    // (number, variable, or arithmetic) into depth.
                                    if let Ok(v) = crate::expr::eval(&op.depth_text, &varmap) {
                                        op.depth = (v as f32).clamp(-300.0, 300.0);
                                    }
                                } else {
                                    // Not focused → reflect drag/slider changes.
                                    op.depth_text = format!("{:.2}", op.depth);
                                }

                                // Blue active underline beneath the value.
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

                            // Vertical "more" affordance, like the Fusion field.
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new("⋮")
                                    .color(egui::Color32::from_rgb(150, 150, 150))
                                    .size(14.0),
                            );
                        });

                        // Operation mode: New Body / Join / Cut — a compact
                        // segmented control beneath the distance, Fusion-style.
                        ui.add_space(5.0);
                        if let Some(op) = self.extrude_op.as_mut() {
                            // Until the user picks a mode, follow the sketch's
                            // context: plane → New Body, face pulled out → Join,
                            // face pushed in → Cut (re-evaluated as they drag).
                            if !op.mode_user_set {
                                op.mode = default_extrude_mode(op.on_face, op.depth);
                            }
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(3.0, 0.0);
                                for (mode, label) in [
                                    (ExtrudeMode::NewBody, "New Body"),
                                    (ExtrudeMode::Join, "Join"),
                                    (ExtrudeMode::Cut, "Cut"),
                                ] {
                                    let selected = op.mode == mode;
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
                                        .min_size(egui::vec2(52.0, 20.0)),
                                    );
                                    if btn.clicked() {
                                        op.mode = mode;
                                        op.mode_user_set = true;
                                    }
                                }
                            });
                        }
                    });
            });

        // Read commit/cancel keys *after* the field has rendered, so an Enter the
        // autocomplete consumed (to accept a suggestion) never leaks into commit.
        let enter = ctx.input(|i| i.key_pressed(egui::Key::Enter));
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        self.autocomplete = ac;

        if enter && !suppress_commit {
            self.commit_extrude_op();
        } else if escape {
            self.cancel_extrude_op();
        }
    }

    /// Create one Extrude body from `region_indices` of `sketch_id` and return
    /// its new id. Does not clear selection or re-evaluate — callers do that.
    fn build_extrude_body(
        &mut self,
        sketch_id: &str,
        mut region_indices: Vec<usize>,
        depth: f32,
        mode: ExtrudeMode,
        depth_expr: Option<String>,
    ) -> Option<String> {
        if region_indices.is_empty() {
            return None;
        }
        region_indices.sort();
        region_indices.dedup();

        let feature_name = self.next_operation_name(mode);
        let extrude_id = format!("extrude_{}", self.next_id());
        log::info!(
            "Creating {:?} extrude {} ({}) from sketch {} ({} faces, depth {}, expr {:?})",
            mode,
            extrude_id,
            feature_name,
            sketch_id,
            region_indices.len(),
            depth,
            depth_expr,
        );

        let extrude_node = FeatureNode {
            id: extrude_id.clone(),
            name: feature_name,
            feature: FeatureType::Extrude {
                depth,
                region_indices,
                mode,
                depth_expr,
            },
        };
        self.graph.add_feature(extrude_node);
        self.graph.add_dependency(sketch_id, &extrude_id);
        Some(extrude_id)
    }

    /// Find a sketch node by id and return its coordinate system, detected
    /// faces, and whether it sits on a body face.
    fn lookup_sketch(&self, sketch_id: &str) -> Option<(CoordinateSystem, Vec<Region>, bool)> {
        let var_map = self.graph.variable_map();
        self.graph.graph.node_indices().find_map(|idx| {
            let node = &self.graph.graph[idx];
            if node.id == sketch_id {
                if let FeatureType::Sketch {
                    cs,
                    curves,
                    shapes,
                    corner_mods,
                    on_face,
                } = &node.feature
                {
                    // Resolve variable-driven dimensions before detecting faces.
                    let eff = zerocad_core::effective_curves(curves, shapes, corner_mods, &var_map);
                    return Some((*cs, detect_regions(&eff), *on_face));
                }
            }
            None
        })
    }

    /// Begin a live extrude operation from a set of `(sketch_id, region_index)`
    /// faces. Opens the right-hand tool window and starts the push/pull preview
    /// instead of committing immediately.
    fn begin_extrude_op(&mut self, faces: Vec<(String, usize)>) {
        if self.extrude_op.is_some() {
            return; // already extruding
        }
        let mut by_sketch: HashMap<String, Vec<usize>> = HashMap::new();
        for (sid, ri) in faces {
            by_sketch.entry(sid).or_default().push(ri);
        }

        let mut targets = Vec::new();
        for (sid, mut idxs) in by_sketch {
            idxs.sort();
            idxs.dedup();
            if let Some((cs, regions, on_face)) = self.lookup_sketch(&sid) {
                idxs.retain(|&i| i < regions.len());
                if !idxs.is_empty() {
                    targets.push(ExtrudeTarget {
                        sketch_id: sid,
                        cs,
                        regions,
                        indices: idxs,
                        on_face,
                    });
                }
            }
        }

        if targets.is_empty() {
            self.status_msg = "No valid faces to extrude.".to_string();
            return;
        }

        let depth = self.extrude_depth.max(1.0);
        // A sketch on a body face defaults to a direction-driven Join/Cut; a
        // sketch on an origin plane defaults to a new body. The user can still
        // override with the mode buttons (which set `mode_user_set`).
        let on_face = targets.iter().any(|t| t.on_face);
        let mode = default_extrude_mode(on_face, depth);
        self.extrude_op = Some(ExtrudeOp {
            targets,
            depth,
            depth_text: format!("{:.2}", depth),
            focus_request: true,
            mode,
            on_face,
            mode_user_set: false,
        });
        // Fresh op — drop any preview cached for a previous one.
        self.extrude_preview_cache = None;
        self.extrude_preview_mesh_cache = None;
        self.status_msg =
            "Extrude: drag up/down in the viewport to push/pull, or type a distance, then OK."
                .to_string();
    }

    /// Start an extrude from the currently selected faces.
    pub(crate) fn begin_extrude_from_selection(&mut self) {
        if self.selected_faces.is_empty() {
            self.status_msg =
                "Select one or more sketch faces in the viewport first, then click Extrude."
                    .to_string();
            return;
        }
        let faces: Vec<(String, usize)> = self.selected_faces.iter().cloned().collect();
        self.begin_extrude_op(faces);
    }

    /// Start an extrude from every face of one sketch (property-panel shortcut).
    pub(crate) fn begin_extrude_whole_sketch(&mut self, sketch_id: &str) {
        let regions = self
            .lookup_sketch(sketch_id)
            .map(|(_, r, _)| r)
            .unwrap_or_default();
        if regions.is_empty() {
            self.status_msg =
                "That sketch has no closed faces to extrude (close your shapes).".to_string();
            return;
        }
        let faces: Vec<(String, usize)> = (0..regions.len())
            .map(|i| (sketch_id.to_string(), i))
            .collect();
        self.begin_extrude_op(faces);
    }

    /// Commit the in-progress extrude: build one body per source sketch.
    pub(crate) fn commit_extrude_op(&mut self) {
        let Some(op) = self.extrude_op.take() else {
            return;
        };
        self.extrude_depth = op.depth; // remember for next time
        self.extrude_mode = op.mode; // remember the mode too

        if op.depth.abs() < 0.01 {
            self.status_msg = "Extrude distance is zero — nothing created.".to_string();
            return;
        }

        // If the distance box held a variable/expression (not a bare number),
        // persist it so the extrude re-evaluates when the variable changes.
        let depth_expr = if zerocad_core::expr::references_variable(&op.depth_text) {
            Some(op.depth_text.trim().to_string())
        } else {
            None
        };

        let mut last_id = None;
        let mut count = 0;
        for t in &op.targets {
            if let Some(id) = self.build_extrude_body(
                &t.sketch_id,
                t.indices.clone(),
                op.depth,
                op.mode,
                depth_expr.clone(),
            ) {
                last_id = Some(id);
                count += 1;
                // The source sketch is consumed into the body — hide it (like
                // Fusion). The user can unhide it from the browser.
                self.hidden_nodes.insert(t.sketch_id.clone());
            }
        }

        self.selected_faces.clear();
        if let Some(id) = last_id {
            self.selected_node_id = Some(id);
        }
        self.reevaluate_geometry();
        self.status_msg = match op.mode {
            ExtrudeMode::NewBody => {
                format!("Extruded {} new body(ies). Source sketch hidden.", count)
            }
            ExtrudeMode::Join => {
                format!(
                    "Joined {} extrude(s) into the model. Source sketch hidden.",
                    count
                )
            }
            ExtrudeMode::Cut => {
                format!(
                    "Cut {} profile(s) out of the model. Source sketch hidden.",
                    count
                )
            }
        };
    }

    /// Discard the in-progress extrude, keeping the face selection.
    pub(crate) fn cancel_extrude_op(&mut self) {
        self.extrude_op = None;
        self.status_msg = "Extrude cancelled.".to_string();
    }
}
