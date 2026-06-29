use crate::*;

impl ZeroCadApp {
    /// Re-run planar region detection on the active sketch curves.
    /// Trims selected indices that no longer correspond to a region.
    pub(crate) fn recompute_sketch_regions(&mut self) {
        self.detected_regions = detect_regions(&self.sketch_curves);
        let n = self.detected_regions.len();
        self.selected_region_indices.retain(|i| *i < n);
    }

    /// Reset everything related to the in-progress sketch.
    pub(crate) fn reset_sketch_state(&mut self) {
        self.sketch_curves = SketchCurves::new();
        self.sketch_shapes.clear();
        self.sketch_corner_mods.clear();
        self.pending_corners.clear();
        self.detected_regions.clear();
        self.selected_region_indices.clear();
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
        let vars = self.graph.variable_map();
        let mut mods = self.sketch_corner_mods.clone();
        mods.extend(self.pending_corner_mods());
        self.sketch_curves =
            zerocad_core::effective_curves(&SketchCurves::new(), &self.sketch_shapes, &mods, &vars);
        self.recompute_sketch_regions();
    }

    /// Build the current radius/setback `Dimension` from the toolbar text (a
    /// number or a variable expression).
    pub(crate) fn corner_radius_dim(&self) -> Dimension {
        let text = self.corner_radius_text.clone();
        let value = self.eval_dim(&text).unwrap_or(5.0).max(0.0);
        if zerocad_core::expr::references_variable(&text) {
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
        let vars = self.graph.variable_map();
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
                        extrusion: 8.0,
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
