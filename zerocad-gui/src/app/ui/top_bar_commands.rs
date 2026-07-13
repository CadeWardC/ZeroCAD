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
                // Editing an existing sketch: update the node IN PLACE (same id
                // → sketch_face_refs, dependency edges, and every downstream
                // captured reference survive), then rebuild.
                if let Some(editing_id) = self.editing_sketch_id.clone() {
                    if !self.sketch_curves.is_empty() {
                        self.push_undo();
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id != editing_id {
                                continue;
                            }
                            if let FeatureType::Sketch {
                                curves,
                                shapes,
                                corner_mods,
                                mirrors,
                                entity_ids,
                                next_entity_id,
                                solver,
                                ..
                            } = &mut self.graph.graph[idx].feature
                            {
                                *curves = self.sketch_curves.clone();
                                *shapes = self.sketch_shapes.clone();
                                *corner_mods = self.sketch_corner_mods.clone();
                                *mirrors = self.sketch_mirrors.clone();
                                // Surviving shapes keep their durable ids; any
                                // shape drawn during this edit session gets a
                                // fresh one (ids are never reused).
                                let mut ids = self.sketch_entity_ids.clone();
                                let mut next = self.sketch_next_entity_id;
                                while ids.len() < self.sketch_shapes.len() {
                                    ids.push(zerocad_core::sketch::EntityId(next));
                                    next += 1;
                                }
                                ids.truncate(self.sketch_shapes.len());
                                *entity_ids = ids;
                                *next_entity_id = next;
                                *solver = self.sketch_solver_model.clone();
                            }
                            break;
                        }
                        self.selected_node_id = Some(editing_id);
                        self.reset_sketch_state();
                        self.restore_camera(ctx);
                        self.is_sketch_mode = false;
                        self.is_plane_selection_mode = false;
                        self.reevaluate_geometry();
                        self.status_msg = "Sketch updated.".to_string();
                        return;
                    } else {
                        self.status_msg = "Empty sketch edit discarded.".to_string();
                        self.reset_sketch_state();
                        self.restore_camera(ctx);
                        self.is_sketch_mode = false;
                        self.is_plane_selection_mode = false;
                        return;
                    }
                }
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
                            mirrors: self.sketch_mirrors.clone(),
                            on_face: self.active_sketch_on_face,
                            // New sketches allocate real per-shape ids at commit;
                            // identity is the id, never the Vec position.
                            entity_ids: zerocad_core::sketch::EntityId::sequence(
                                self.sketch_shapes.len(),
                            ),
                            next_entity_id: self.sketch_shapes.len() as u32,
                            solver: None,
                        },
                    };

                    self.push_undo();
                    self.graph.add_feature(sketch_node);
                    // A sketch placed on a body face records that face + a dependency
                    // on the body, so on rebuild its plane is re-derived from where
                    // the face now is (the sketch follows the body).
                    if let Some(fref) = self.active_sketch_face_ref.take() {
                        if let Some(body_id) =
                            fref.topology.as_ref().and_then(|t| t.body_id.clone())
                        {
                            self.graph.add_dependency(&body_id, &sketch_id);
                        }
                        self.graph.sketch_face_refs.insert(sketch_id.clone(), fref);
                    }
                    // The projected face outline persists with the sketch so
                    // rebuilds (and later edit sessions) detect the same
                    // boundary-split regions the user saw while drawing.
                    if self.active_sketch_on_face && !self.active_face_boundary.is_empty() {
                        self.graph
                            .sketch_face_boundaries
                            .insert(sketch_id.clone(), self.active_face_boundary.clone());
                    }
                    // A sketch placed on a datum plane records the datum + a
                    // dependency, so its plane re-derives from the datum's current
                    // resolution on every rebuild (editing the datum moves it).
                    if let Some(datum_id) = self.active_sketch_datum_ref.take() {
                        self.graph.add_dependency(&datum_id, &sketch_id);
                        self.graph
                            .sketch_datum_refs
                            .insert(sketch_id.clone(), datum_id);
                    }
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

                        let cs_and_ref = face_sel.and_then(|(nid, fid)| {
                            self.face_cs(&nid, fid)
                                .map(|cs| (cs, self.face_ref(&nid, fid), nid, fid))
                        });
                        match cs_and_ref {
                            Some((cs, fref, nid, fid)) => {
                                log::info!("Sketching on a selected body face.");
                                let boundary = self.face_boundary_curves(&nid, fid, &cs);
                                let now = ui.input(|i| i.time);
                                self.active_sketch_on_face = true;
                                // Remember which face, so the finished sketch follows it.
                                self.active_sketch_face_ref = fref;
                                self.begin_sketch_on(cs, now);
                                // After begin (it resets sketch state): the face
                                // outline joins the sketch as reference geometry.
                                self.active_face_boundary = boundary;
                                self.recompute_sketch_regions();
                                self.status_msg =
                                    "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
                            }
                            None => {
                                log::info!("Entering sketch plane selection mode. Viewport remains in 3D.");
                                self.active_sketch_on_face = false;
                                self.active_sketch_face_ref = None;
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

            // Direct push/pull: exactly one planar BODY face selected (and no
            // sketch faces) → the Extrude button pulls/pushes that face via a
            // hidden helper sketch of its projected outline.
            let body_face = (sel == 0)
                .then(|| self.hole_face_candidate())
                .flatten()
                .filter(|(n, f)| self.face_is_planar(n, *f));

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
            } else if let Some((node, fid)) = body_face {
                let extrude_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Extrude Face",
                    egui::Color32::from_rgb(37, 99, 235),
                    egui::Color32::from_rgb(29, 78, 216),
                    egui::Color32::WHITE,
                    egui::Stroke::NONE,
                );
                if extrude_btn
                    .on_hover_text(
                        "Push/pull the selected body face: pull out to add material (Join), push in to remove it (Cut)",
                    )
                    .clicked()
                {
                    self.begin_extrude_on_body_face(node, fid);
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

            // REVOLVE: same sketch-face selection as Extrude, spun about an axis.
            if extrude_enabled && self.revolve_op.is_none() {
                let revolve_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Revolve",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if revolve_btn
                    .on_hover_text(
                        "Revolve the selected sketch face(s) about an axis in the sketch plane",
                    )
                    .clicked()
                {
                    self.begin_revolve_from_selection();
                }
            }

            // SWEEP: one selected profile face, swept along a path sketch.
            if self.sweep_op.is_none() {
                if let Some((profile_sketch, profile_region)) = self.sweep_profile_candidate() {
                    let sweep_btn = icons::Icon::Extrude.labeled_button(
                        ui,
                        "Sweep",
                        egui::Color32::from_rgb(241, 245, 249),
                        egui::Color32::from_rgb(226, 232, 240),
                        self.pal().text_strong,
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                    );
                    if sweep_btn
                        .on_hover_text("Sweep the selected profile along a path sketch")
                        .clicked()
                    {
                        self.begin_sweep(profile_sketch, profile_region);
                    }
                }
            }

            // LOFT: two or more selected sketch faces across sketches.
            if let Some(sections) = self.loft_sections() {
                let loft_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    &format!("Loft ({})", sections.len()),
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if loft_btn
                    .on_hover_text("Loft through the selected section profiles (creation order)")
                    .clicked()
                {
                    self.commit_loft(sections);
                }
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
                egui::popup_below_widget(
                    ui,
                    popup_id,
                    &btn,
                    egui::PopupCloseBehavior::CloseOnClickOutside,
                    |ui| {
                        ui.set_min_width(140.0);
                        if icons::Icon::Fillet.menu_button(ui, "Fillet").clicked() {
                            self.begin_edge_mod(CornerKind::Fillet);
                            ui.memory_mut(|m| m.close_popup());
                        }
                        if icons::Icon::Chamfer.menu_button(ui, "Chamfer").clicked() {
                            self.begin_edge_mod(CornerKind::Chamfer);
                            ui.memory_mut(|m| m.close_popup());
                        }
                    },
                );
            });
            ui.label(
                egui::RichText::new("Works best on a convex edge of a plain box/extrude.")
                    .size(10.0)
                    .color(self.pal().text_faint),
            );
        }

        // HOLE: drill into the selected body face.
        if !active_sketching
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.hole_op.is_none()
        {
            if let Some((node, fid)) = self.hole_face_candidate() {
                ui.separator();
                let hole_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Hole",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if hole_btn
                    .on_hover_text(
                        "Drill a hole into the selected face (simple, counterbore, or countersink)",
                    )
                    .clicked()
                {
                    self.begin_hole(node, fid);
                }
            }
        }

        // THREAD: cut a modeled thread into the selected cylindrical face.
        if !active_sketching
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.hole_op.is_none()
            && self.thread_op.is_none()
        {
            if let Some((node, fid)) = self.thread_face_candidate() {
                ui.separator();
                let thread_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Thread",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if thread_btn
                    .on_hover_text(
                        "Cut a modeled thread into the selected cylindrical face (metric / Unified / custom)",
                    )
                    .clicked()
                {
                    self.begin_thread(node, fid);
                }
            }
        }

        // SHELL: hollow the selected body, removing the selected face(s).
        if !active_sketching
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.shell_op.is_none()
        {
            if let Some((target, fids)) = self.shell_candidate() {
                let shell_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Shell",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if shell_btn
                    .on_hover_text("Hollow the body, removing the selected face(s)")
                    .clicked()
                {
                    self.begin_shell(target, fids);
                }
            }
        }

        // COMBINE: shown only for two fully-selected bodies; its dialog offers
        // strict Join and body-on-body Cut.
        if !active_sketching
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.move_op.is_none()
            && self.pattern_op.is_none()
            && self.combine_op.is_none()
        {
            if let Some(sources) = self.selected_bodies_to_combine() {
                ui.separator();
                let join_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Combine",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if join_btn
                    .on_hover_text("Join touching bodies or cut one overlapping body with another")
                    .clicked()
                {
                    self.begin_combine(sources);
                }
            }
        }

        // MOVE: shown only for one fully-selected body. The same command is in
        // the viewport's right-click menu.
        if !active_sketching && self.move_op.is_none() {
            if let Some(source) = self.selected_whole_body() {
                ui.separator();
                let move_btn = icons::Icon::Mirror.labeled_button(
                    ui,
                    "Move",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if move_btn
                    .on_hover_text("Translate the selected body or align one of its faces")
                    .clicked()
                {
                    self.begin_move_body(source);
                }
            }
        }

        // PATTERN / MIRROR: separate commands over the selected body. Mirror
        // immediately enters the viewport plane/face picker.
        if !active_sketching
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.pattern_op.is_none()
        {
            if let Some(source) = self.pattern_source_candidate() {
                ui.separator();
                let pattern_btn = icons::Icon::Extrude.labeled_button(
                    ui,
                    "Pattern",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if pattern_btn
                    .on_hover_text("Create a linear or circular array of the selected body")
                    .clicked()
                {
                    self.begin_pattern(source.clone());
                }
                let mirror_btn = icons::Icon::Mirror.labeled_button(
                    ui,
                    "Mirror",
                    egui::Color32::from_rgb(241, 245, 249),
                    egui::Color32::from_rgb(226, 232, 240),
                    self.pal().text_strong,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if mirror_btn
                    .on_hover_text("Mirror the selected body across a picked plane or planar face")
                    .clicked()
                {
                    self.begin_mirror(source);
                }
            }
        }

        // DATUM: construction geometry (reference planes / axes / points).
        // Created with defaults, then tuned in the Properties panel; a datum
        // plane becomes a sketchable target in the plane picker.
        if !active_sketching && self.extrude_op.is_none() && self.edge_mod_op.is_none() {
            ui.separator();
            let datum_btn_id = ui.make_persistent_id("datum_menu_dropdown");
            let datum_btn = icons::Icon::Sketch.labeled_button(
                ui,
                "Datum  ▾",
                egui::Color32::from_rgb(241, 245, 249),
                egui::Color32::from_rgb(226, 232, 240),
                self.pal().text_strong,
                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
            );
            let datum_btn = datum_btn.on_hover_text("Create a reference plane, axis, or point");
            if datum_btn.clicked() {
                ui.memory_mut(|mem| mem.toggle_popup(datum_btn_id));
            }
            egui::popup_below_widget::<()>(
                ui,
                datum_btn_id,
                &datum_btn,
                egui::PopupCloseBehavior::CloseOnClickOutside,
                |ui| {
                    ui.set_min_width(190.0);
                    ui.style_mut().spacing.button_padding = egui::vec2(14.0, 5.0);
                    ui.label(
                        egui::RichText::new("Offset plane from…")
                            .size(10.5)
                            .color(self.pal().text_faint),
                    );
                    for (label, base) in [
                        ("XY Plane", zerocad_core::PlaneBase::XY),
                        ("XZ Plane (ground)", zerocad_core::PlaneBase::XZ),
                        ("YZ Plane", zerocad_core::PlaneBase::YZ),
                    ] {
                        if ui.button(label).clicked() {
                            ui.memory_mut(|mem| mem.close_popup());
                            self.create_datum(DatumKind::OffsetPlane(base));
                        }
                    }
                    ui.separator();
                    if ui.button("Angle Plane").clicked() {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.create_datum(DatumKind::AnglePlane);
                    }
                    if ui.button("3-Point Plane").clicked() {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.create_datum(DatumKind::ThreePointPlane);
                    }
                    if ui.button("Axis (2 points)").clicked() {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.create_datum(DatumKind::Axis);
                    }
                    if ui.button("Point").clicked() {
                        ui.memory_mut(|mem| mem.close_popup());
                        self.create_datum(DatumKind::Point);
                    }
                },
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
