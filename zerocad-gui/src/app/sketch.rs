use crate::*;

impl ZeroCadApp {
    /// Undo the most recently committed action in the live sketch instead of
    /// stepping the document history. Sketch edits do not enter the document
    /// undo stack until Finish Sketch, so routing Ctrl/Cmd+Z to `undo()` while
    /// this mode is active either did nothing or changed an unrelated feature.
    pub(crate) fn undo_last_sketch_action(&mut self) {
        if self.clear_pending_corners() {
            self.status_msg = "Undid the staged sketch corner.".to_string();
            return;
        }

        let Some(_shape) = self.sketch_shapes.pop() else {
            if self.sketch_mirrors.pop().is_some() || self.sketch_corner_mods.pop().is_some() {
                self.rebuild_active_sketch_curves();
                self.status_msg = "Undid the last sketch operation.".to_string();
            } else {
                self.status_msg = "Nothing to undo in this sketch.".to_string();
            }
            return;
        };

        // Edit Sketch uses the solver model as its geometry source. Remove the
        // entities promoted from the popped shape as well, plus constraints and
        // points that would otherwise keep that shape visible.
        if let Some(owner) = self.sketch_entity_ids.pop() {
            if let Some(model) = &mut self.sketch_solver_model {
                model
                    .entities
                    .retain(|entity| entity.derived_from() != Some(owner));

                let mut entity_ids = std::collections::HashSet::new();
                let mut point_ids = std::collections::HashSet::new();
                for entity in &model.entities {
                    entity_ids.insert(entity.id());
                    match entity {
                        zerocad_core::sketch::SketchEntity::Line { p0, p1, .. } => {
                            point_ids.insert(*p0);
                            point_ids.insert(*p1);
                        }
                        zerocad_core::sketch::SketchEntity::Circle { center, .. } => {
                            point_ids.insert(*center);
                        }
                        zerocad_core::sketch::SketchEntity::Arc {
                            center, start, end, ..
                        } => {
                            point_ids.insert(*center);
                            point_ids.insert(*start);
                            point_ids.insert(*end);
                        }
                    }
                }
                model.points.retain(|point| point_ids.contains(&point.id));
                model.constraints.retain(|constraint| {
                    use zerocad_core::sketch::Constraint;
                    match constraint {
                        Constraint::Coincident { a, b, .. } | Constraint::Distance { a, b, .. } => {
                            point_ids.contains(a) && point_ids.contains(b)
                        }
                        Constraint::Horizontal { line, .. } | Constraint::Vertical { line, .. } => {
                            entity_ids.contains(line)
                        }
                        Constraint::Radius { circle, .. } => entity_ids.contains(circle),
                        Constraint::Parallel { a, b, .. }
                        | Constraint::Perpendicular { a, b, .. }
                        | Constraint::Equal { a, b, .. } => {
                            entity_ids.contains(a) && entity_ids.contains(b)
                        }
                        Constraint::Tangent { line, circle, .. } => {
                            entity_ids.contains(line) && entity_ids.contains(circle)
                        }
                        Constraint::Fixed { p, .. } => point_ids.contains(p),
                    }
                });
            }
        }

        self.line_chain_start = None;
        self.cancel_in_progress_shape();
        self.rebuild_active_sketch_curves();
        self.status_msg = "Undid the last drawn shape.".to_string();
    }

    /// Commit an **associative** sketch Mirror across the 2-click axis `p0`→`p1`.
    /// Rather than baking a reflected copy, this stores a [`SketchMirror`] record
    /// that [`zerocad_core::effective_curves_solved`] re-applies on every rebuild
    /// — so editing (or drag-solving) the source half updates the mirror. Works
    /// the same in a fresh drawing session and an Edit Sketch session, since the
    /// record is geometry-independent.
    pub(crate) fn commit_sketch_mirror(&mut self, p0: (f32, f32), p1: (f32, f32)) {
        let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
        if (dx * dx + dy * dy).sqrt() < 1.0e-4 {
            self.status_msg = "Mirror axis too short — click two distinct points.".to_string();
            return;
        }
        if self.sketch_curves.is_empty() {
            self.status_msg = "Nothing to mirror — draw geometry first.".to_string();
            return;
        }
        self.sketch_mirrors
            .push(zerocad_core::SketchMirror { a: p0, b: p1 });
        self.rebuild_active_sketch_curves();
        self.status_msg = "Mirrored sketch across the axis (associative).".to_string();
    }

    /// The reflected copy of the live sketch across the in-progress mirror axis
    /// `p0`→`cursor`, for the drawing preview. `None` if the axis is degenerate.
    pub(crate) fn mirror_preview_curves(
        &self,
        p0: (f32, f32),
        cursor: (f32, f32),
    ) -> Option<SketchCurves> {
        let (dx, dy) = (cursor.0 - p0.0, cursor.1 - p0.1);
        if (dx * dx + dy * dy).sqrt() < 1.0e-4 {
            return None;
        }
        Some(zerocad_core::reflect_curves_across(
            &self.sketch_curves,
            p0,
            cursor,
        ))
    }

    /// Re-run planar region detection on the active sketch curves.
    /// Trims selected indices that no longer correspond to a region.
    ///
    /// A sketch on a body face folds the projected face boundary in as
    /// reference curves — appended AFTER the drawn curves, the same merge
    /// order the evaluator and every committed-sketch region site use, so the
    /// region indices seen live match the ones an extrude will store.
    pub(crate) fn recompute_sketch_regions(&mut self) {
        let mut curves = self.sketch_curves.clone();
        curves.extend_curves(&self.active_face_boundary);
        self.detected_regions = detect_regions(&curves);
        let n = self.detected_regions.len();
        self.selected_region_indices.retain(|i| *i < n);
    }

    /// Reset everything related to the in-progress sketch.
    pub(crate) fn reset_sketch_state(&mut self) {
        self.sketch_curves = SketchCurves::new();
        self.active_face_boundary = SketchCurves::new();
        self.sketch_shapes.clear();
        self.sketch_corner_mods.clear();
        self.sketch_mirrors.clear();
        self.pending_corners.clear();
        self.detected_regions.clear();
        self.selected_region_indices.clear();
        self.editing_sketch_id = None;
        self.sketch_solver_model = None;
        self.sketch_entity_ids.clear();
        self.sketch_next_entity_id = 0;
        self.sketch_drag_point = None;
        self.line_chain_start = None;
        self.sketch_selected_ids.clear();
        self.sketch_selected_constraint = None;
        self.sketch_conflict_constraint = None;
        self.cancel_in_progress_shape();
    }

    /// Rebuild the live sketch geometry from the parametric `sketch_shapes` (plus
    /// any committed fillet/chamfer corner mods AND the uncommitted pending ones)
    /// against the current variables, then re-detect regions. Folding the pending
    /// corners in here is what gives the Fillet/Chamfer tool its live preview:
    /// every staged corner shows rounded/beveled at the current radius before the
    /// user commits. Called after any change to the shape list, the pending set,
    /// or the radius text.
    pub(crate) fn rebuild_active_sketch_curves(&mut self) {
        let vars = self.document.variable_map();
        let mut mods = self.sketch_corner_mods.clone();
        mods.extend(self.pending_corner_mods());
        // A solver model (Edit Sketch session) is the source of truth for the
        // live geometry; a fresh drawing session bakes from the shape list.
        self.sketch_curves = zerocad_core::effective_curves_solved(
            &SketchCurves::new(),
            &self.sketch_shapes,
            &mods,
            &self.sketch_mirrors,
            self.sketch_solver_model.as_ref(),
            &vars,
        );
        self.recompute_sketch_regions();
    }

    /// Re-solve the live sketch's constraint model in place (drag frames, a
    /// constraint edit) and rebuild the displayed curves from the result. The
    /// solve is warm-started from the current positions, so an under-constrained
    /// sketch moves minimally. On a failed solve the model keeps its current
    /// (last-valid) positions — the geometry degrades, never blanks.
    pub(crate) fn solve_live_sketch(&mut self) {
        let vars = self.document.variable_map();
        if let Some(model) = &mut self.sketch_solver_model {
            let report = zerocad_core::sketch::solve_model(model, &vars);
            if report.outcome == zerocad_core::sketch::SolveOutcome::Converged {
                zerocad_core::sketch::solve::apply_solution(model, &report);
                self.sketch_conflict_constraint = None;
            } else {
                // Cache the culprit for the red badge/list highlight; geometry
                // keeps its last-valid positions.
                self.sketch_conflict_constraint = report.conflicting;
            }
        }
        self.rebuild_active_sketch_curves();
    }

    /// Build the current radius/setback `Dimension` from the toolbar text (a
    /// number or a variable expression).
    pub(crate) fn corner_radius_dim(&self) -> Dimension {
        let text = self.corner_radius_text.clone();
        let value = self.eval_dim(&text).unwrap_or(5.0).max(0.0);
        if zerocad_core::expr::preserves_source(&text) {
            Dimension {
                value,
                expr: Some(text.trim().to_string()),
            }
        } else {
            Dimension::literal(value)
        }
    }

    /// The uncommitted corner mods: one per pending corner, all sharing the
    /// current toolbar radius and the active tool's kind. Empty unless the
    /// Fillet/Chamfer tool is armed.
    pub(crate) fn pending_corner_mods(&self) -> Vec<CornerMod> {
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return Vec::new();
        };
        if self.pending_corners.is_empty() {
            return Vec::new();
        }
        let radius = self.corner_radius_dim();
        self.pending_corners
            .iter()
            .map(|&at| CornerMod {
                at,
                radius: radius.clone(),
                kind,
            })
            .collect()
    }

    /// Stage the sketch corner nearest `at` for a fillet/chamfer. It previews
    /// immediately (live) at the current radius; nothing is committed until the
    /// user presses Enter / clicks OK. The core snaps `at` to the actual corner
    /// vertex, so clicking near a corner is enough.
    pub(crate) fn stage_corner_at(&mut self, at: (f32, f32), kind: CornerKind) {
        if self.sketch_curves.segments.is_empty() {
            self.status_msg =
                "Draw straight edges first, then fillet/chamfer a corner.".to_string();
            return;
        }
        self.pending_corners.push(at);
        self.rebuild_active_sketch_curves();
        let noun = match kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        self.status_msg = format!(
            "{} previewing {} corner(s) — adjust R, click more, then Enter / OK to apply.",
            noun,
            self.pending_corners.len()
        );
    }

    /// Commit the staged fillet/chamfer corners into the sketch, capturing the
    /// current radius on each. Rebuilds the live curves (an identity rebuild,
    /// since the geometry already previewed the same mods).
    pub(crate) fn commit_pending_corners(&mut self) {
        if self.pending_corners.is_empty() {
            return;
        }
        let mods = self.pending_corner_mods();
        let n = mods.len();
        self.sketch_corner_mods.extend(mods);
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg = format!("Applied to {} corner(s).", n);
    }

    /// Drop the staged (uncommitted) fillet/chamfer corners and rebuild so the
    /// preview disappears. Returns true if anything was pending.
    pub(crate) fn clear_pending_corners(&mut self) -> bool {
        if self.pending_corners.is_empty() {
            return false;
        }
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        true
    }

    /// The sharp corner nearest `at` and its **interior bisector** (unit, in
    /// sketch coords), computed from the un-rounded geometry. Used to place and
    /// orient the 2D radius drag handle. `None` for a straight/degenerate corner.
    pub(crate) fn corner_bisector(&self, at: (f32, f32)) -> Option<((f32, f32), (f32, f32))> {
        let vars = self.document.variable_map();
        // Geometry without ANY corner mods, so the pending corner is still sharp.
        let sharp =
            zerocad_core::effective_curves(&SketchCurves::new(), &self.sketch_shapes, &[], &vars);

        // Nearest segment endpoint = the corner vertex.
        let mut best: Option<((f32, f32), f32)> = None;
        for s in &sharp.segments {
            for v in [s.a, s.b] {
                let d = (v.0 - at.0).hypot(v.1 - at.1);
                if best.map_or(true, |(_, bd)| d < bd) {
                    best = Some((v, d));
                }
            }
        }
        let (v, _) = best?;

        // Unit directions of the (up to two) segments leaving that vertex.
        let mut dirs: Vec<(f32, f32)> = Vec::new();
        for s in &sharp.segments {
            let other = if (s.a.0 - v.0).hypot(s.a.1 - v.1) < 1.0e-3 {
                Some(s.b)
            } else if (s.b.0 - v.0).hypot(s.b.1 - v.1) < 1.0e-3 {
                Some(s.a)
            } else {
                None
            };
            if let Some(o) = other {
                let (dx, dy) = (o.0 - v.0, o.1 - v.1);
                let l = dx.hypot(dy);
                if l > 1.0e-4 {
                    dirs.push((dx / l, dy / l));
                }
            }
            if dirs.len() == 2 {
                break;
            }
        }
        if dirs.len() < 2 {
            return None;
        }
        let (bx, by) = (dirs[0].0 + dirs[1].0, dirs[0].1 + dirs[1].1);
        let bl = (bx * bx + by * by).sqrt();
        if bl < 1.0e-4 {
            return None; // 180° "corner" — no bisector
        }
        Some((v, (bx / bl, by / bl)))
    }

    /// Fusion-style floating radius/setback box for the 2D Fillet/Chamfer tool,
    /// anchored on the staged corner (or the live cursor) via `corner_dim_pos`.
    /// Edits the same `corner_radius_text` the toolbar shows; while corners are
    /// staged, every keystroke re-previews them live. Variables/expressions are
    /// accepted via the shared autocomplete.
    pub(crate) fn show_corner_radius_box(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode {
            return;
        }
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return;
        };
        let Some(pos) = self.corner_dim_pos else {
            return;
        };
        let label = match kind {
            CornerKind::Fillet => "R",
            CornerKind::Chamfer => "D",
        };
        let unit_suffix = self.current_unit.suffix();
        let var_names = self.visible_variable_names();
        let var_map = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        let mut changed = false;

        egui::Area::new(egui::Id::new("corner_radius_inline"))
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
                            ui.label(
                                egui::RichText::new(label)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                            ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                            ui.style_mut().visuals.widgets.inactive.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.widgets.hovered.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.selection.bg_fill =
                                egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);
                            let field_id = egui::Id::new("corner_radius_field");
                            let outcome = crate::expr::autocomplete_field(
                                ui,
                                field_id,
                                &mut self.corner_radius_text,
                                50.0,
                                true,
                                false,
                                false,
                                &var_names,
                                &mut ac,
                            );
                            if outcome.response.changed() {
                                changed = true;
                            }
                            ui.label(
                                egui::RichText::new(unit_suffix)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                            if zerocad_core::expr::preserves_source(&self.corner_radius_text) {
                                if let Ok(value) =
                                    crate::expr::eval(&self.corner_radius_text, &var_map)
                                {
                                    ui.label(
                                        egui::RichText::new(format!("= {value:.2}"))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(70, 120, 70)),
                                    );
                                }
                            }
                        });
                    });
            });
        self.autocomplete = ac;
        // Live: editing the size re-previews the staged corners.
        if changed && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }

    /// The Fusion-style drag manipulator for the 2D Fillet/Chamfer radius: a
    /// handle on the staged corner's bisector, joined to the corner by a guide
    /// line. Dragging it along that axis grows/shrinks the radius live, in sync
    /// with the size box and toolbar field. Mirrors the 3D edge handle, in the
    /// sketch plane.
    pub(crate) fn drag_corner_radius_handle(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.active_tool.and_then(|t| t.corner_kind()).is_none() {
            return;
        }
        let Some((corner, hpos, axis)) = self.corner_handle else {
            return;
        };
        let len2 = axis.length_sq();
        let r = 7.0;
        let mut dragged = false;

        egui::Area::new(egui::Id::new("corner_radius_handle"))
            .order(egui::Order::Foreground)
            .fixed_pos(hpos - egui::vec2(r, r))
            .show(ctx, |ui| {
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
                painter.line_segment([corner, hpos], egui::Stroke::new(1.5, accent));
                painter.circle_filled(hpos, r, accent);
                painter.circle_stroke(hpos, r, egui::Stroke::new(1.5, egui::Color32::WHITE));

                if resp.dragged() && len2 > 1.0e-6 {
                    let d = resp.drag_delta();
                    let delta_mm = (d.x * axis.x + d.y * axis.y) / len2;
                    let cur = self.eval_dim(&self.corner_radius_text).unwrap_or(5.0);
                    let next = (cur + delta_mm).clamp(0.1, 1000.0);
                    self.corner_radius_text = format!("{:.2}", next);
                    dragged = true;
                }
                resp.on_hover_cursor(egui::CursorIcon::ResizeHorizontal);
            });

        // Re-preview the staged corners only when the handle actually moved.
        if dragged && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }
}
