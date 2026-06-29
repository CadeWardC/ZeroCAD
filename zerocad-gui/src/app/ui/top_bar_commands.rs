use crate::*;

impl ZeroCadApp {
    pub(crate) fn draw_top_bar_modeling_commands(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        active_sketching: bool,
    ) {
        // Draw Sketch / Finish Sketch CTA Button
        if active_sketching {
            let finish_btn = icons::Icon::Check.labeled_button(
                ui,
                "Finish Sketch",
                egui::Color32::from_rgb(16, 185, 129), // Emerald Green CTA
                egui::Color32::from_rgb(5, 150, 105),  // Hover
                egui::Color32::WHITE,
                egui::Stroke::NONE,
            );
            if finish_btn
                .on_hover_text("Complete sketch and save as a 2D profile")
                .clicked()
            {
                log::info!("Finishing sketch — saving it as a 2D object.");
                // Bake any still-staged fillet/chamfer corners so finishing
                // the sketch never silently drops them.
                self.commit_pending_corners();
                self.recompute_sketch_regions();
                if !self.sketch_curves.is_empty() {
                    let sketch_id = format!("sketch_{}", self.next_id());
                    let sketch_name = self.next_sketch_name();
                    log::info!(
                        "Saving sketch {} ({}) ({} curves, {} faces).",
                        sketch_id,
                        sketch_name,
                        self.sketch_curves.segments.len() + self.sketch_curves.circles.len(),
                        self.detected_regions.len(),
                    );

                    let sketch_node = FeatureNode {
                        id: sketch_id.clone(),
                        name: sketch_name,
                        feature: FeatureType::Sketch {
                            cs: self.active_sketch_cs,
                            curves: self.sketch_curves.clone(),
                            shapes: self.sketch_shapes.clone(),
                            corner_mods: self.sketch_corner_mods.clone(),
                            on_face: self.active_sketch_on_face,
                        },
                    };

                    self.push_undo();
                    self.graph.add_feature(sketch_node);
                    self.selected_node_id = Some(sketch_id);
                    self.reset_sketch_state();
                    self.status_msg =
                        "Sketch saved as a 2D object. Use the Extrude tool to make a body."
                            .to_string();
                } else {
                    self.status_msg = "Empty sketch discarded.".to_string();
                    log::warn!("Sketch discarded: nothing drawn.");
                    self.reset_sketch_state();
                }

                // Animate camera BACK to previous 3D state
                log::info!(
                    "Restoring previous 3D camera state: pitch: {:.2}, yaw: {:.2}",
                    self.pre_sketch_pitch,
                    self.pre_sketch_yaw
                );
                self.restore_camera(ctx);

                self.is_sketch_mode = false;
                self.is_plane_selection_mode = false;
            }
        } else {
            let draw_btn = icons::Icon::Sketch.labeled_button(
                ui,
                "Draw Sketch",
                egui::Color32::from_rgb(241, 245, 249), // Clean slate grey
                egui::Color32::from_rgb(226, 232, 240), // Hover
                self.pal().text_strong,
                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
            );
            if draw_btn.on_hover_text("Enter sketch mode — sketches on the selected body face if one is selected, else pick an origin plane").clicked() {
                        // Context-aware: if exactly one body FACE is selected,
                        // sketch directly on it; otherwise open the plane picker.
                        let face_sel = if self.selected_body.len() == 1 {
                            self.selected_body.iter().next().and_then(|(nid, p)| match p {
                                BodyPick::Face(f) => Some((nid.clone(), *f)),
                                _ => None,
                            })
                        } else {
                            None
                        };

                        match face_sel.and_then(|(nid, fid)| self.face_cs(&nid, fid)) {
                            Some(cs) => {
                                log::info!("Sketching on a selected body face.");
                                let now = ui.input(|i| i.time);
                                self.active_sketch_on_face = true;
                                self.begin_sketch_on(cs, now);
                                self.status_msg =
                                    "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
                            }
                            None => {
                                log::info!("Entering sketch plane selection mode. Viewport remains in 3D.");
                                self.active_sketch_on_face = false;
                                self.is_plane_selection_mode = true;
                                self.is_sketch_mode = false;
                                self.reset_sketch_state();
                                self.status_msg = "Click on one of the origin planes (XY Red, XZ Green, YZ Blue) in the viewport to sketch on it.".to_string();
                            }
                        }
                    }
        }

        // EXTRUDE: select faces in the 3D viewport, then start the tool.
        if !active_sketching && self.extrude_op.is_none() {
            ui.separator();
            let sel = self.selected_faces.len();
            let extrude_enabled = sel > 0;

            if extrude_enabled {
                let extrude_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    &format!("Extrude ({})", sel),
                    egui::Color32::from_rgb(37, 99, 235), // vibrant active blue
                    egui::Color32::from_rgb(29, 78, 216),
                    egui::Color32::WHITE,
                    egui::Stroke::NONE,
                );
                if extrude_btn
                    .on_hover_text("Extrude the selected 3D face(s) into a solid body")
                    .clicked()
                {
                    self.begin_extrude_from_selection();
                }
            } else {
                // Inert (no selection): same fill on hover so it reads disabled.
                icons::Icon::Extrude
                    .labeled_button(
                        ui,
                        "Extrude",
                        egui::Color32::from_rgb(241, 245, 249),
                        egui::Color32::from_rgb(241, 245, 249),
                        self.pal().text_faint,
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)),
                    )
                    .on_hover_text("Select one or more 3D faces first");
            }

            if sel > 0 {
                let clear_sel_btn = ui.add(
                    egui::Button::new(
                        egui::RichText::new("Clear Selection")
                            .color(egui::Color32::from_rgb(220, 38, 38)) // red text
                            .size(12.0),
                    )
                    .fill(egui::Color32::from_rgb(254, 226, 226)) // soft red wash
                    .rounding(egui::Rounding::same(6.0))
                    .min_size(egui::vec2(90.0, 26.0)),
                );
                if clear_sel_btn.clicked() {
                    self.selected_faces.clear();
                    self.selected_edges.clear();
                    self.selected_body.clear();
                }
            }
        }

        // 3D edge fillet / chamfer: shown when one or more body EDGES are
        // selected (and we're not sketching/extruding). Rounds or bevels the
        // real solid; several edges (Shift/Ctrl-click) fillet at once.
        let edge_sel =
            (!active_sketching && self.extrude_op.is_none() && self.edge_mod_op.is_none())
                .then(|| self.selected_body_edges())
                .flatten();
        if let Some((_, edge_ids)) = edge_sel {
            let n_edges = edge_ids.len();
            ui.separator();
            ui.label(
                egui::RichText::new(if n_edges > 1 {
                    format!("Modify {n_edges} Edges")
                } else {
                    "Modify Edge".to_string()
                })
                .strong()
                .size(12.0)
                .color(self.pal().text_strong),
            );
            ui.add_space(4.0);
            // One button: a left-click starts a Fillet (the default); a
            // right-click (or the ▾) opens a flyout to pick Fillet or
            // Chamfer — the same convention the Rectangle/Circle tools use.
            // Either way it begins a live preview; the size is set in the
            // floating box, and the inline dialog can still toggle the kind.
            let popup_id = ui.make_persistent_id("edgemod_flyout");
            ui.horizontal(|ui| {
                let btn = icons::Icon::Fillet.labeled_button(
                    ui,
                    "Fillet  ▾",
                    egui::Color32::from_rgb(37, 99, 235),
                    egui::Color32::from_rgb(29, 78, 216),
                    egui::Color32::WHITE,
                    egui::Stroke::NONE,
                );
                let btn = btn.on_hover_text(
                    "Round the edge (live preview). Right-click or ▾ to choose Chamfer.",
                );
                if btn.clicked() {
                    self.begin_edge_mod(CornerKind::Fillet);
                }
                if btn.secondary_clicked() {
                    ui.memory_mut(|m| m.toggle_popup(popup_id));
                }
                egui::popup_below_widget(ui, popup_id, &btn, |ui| {
                    ui.set_min_width(140.0);
                    if icons::Icon::Fillet.menu_button(ui, "Fillet").clicked() {
                        self.begin_edge_mod(CornerKind::Fillet);
                        ui.memory_mut(|m| m.close_popup());
                    }
                    if icons::Icon::Chamfer.menu_button(ui, "Chamfer").clicked() {
                        self.begin_edge_mod(CornerKind::Chamfer);
                        ui.memory_mut(|m| m.close_popup());
                    }
                });
            });
            ui.label(
                egui::RichText::new("Works best on a convex edge of a plain box/extrude.")
                    .size(10.0)
                    .color(self.pal().text_faint),
            );
        }

        if self.is_plane_selection_mode {
            ui.separator();
            ui.label(
                        egui::RichText::new("🖱️ Hover & Click a 3D Plane sheet in the viewport to select your sketching plane.")
                            .color(egui::Color32::from_rgb(217, 119, 6)) // elegant warm amber text
                            .strong()
                    );
        }
    }
}
