use crate::*;
use zerocad_core::{DatumAxisDef, DatumPlaneDef, DatumPointDef};

impl ZeroCadApp {
    pub(crate) fn draw_selected_feature_properties(&mut self, ui: &mut egui::Ui) {
        ui.add_space(15.0);
        ui.label(
            egui::RichText::new("Properties")
                .font(egui::FontId::proportional(14.0))
                .strong()
                .color(self.pal().text_strong), // Slate-900
        );
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(8.0);

        // Render dynamic sliders based on selected node's feature type
        if let Some(ref selected_id) = self.selected_node_id {
            let mut node_idx = None;
            for idx in self.graph.graph.node_indices() {
                if self.graph.graph[idx].id == *selected_id {
                    node_idx = Some(idx);
                    break;
                }
            }

            if let Some(idx) = node_idx {
                // Deferred action: extruding needs `&mut self`, but
                // `node` holds a mutable borrow of the graph below.
                let mut extrude_request: Option<String> = None;
                let mut edit_sketch_request: Option<String> = None;
                let mut modified = false;

                // Capture palette + unit + the variable map before
                // borrowing the graph mutably (so the Extrude panel can
                // show what an expression-driven depth resolves to).
                let pal = self.pal();
                let current_unit = self.current_unit;
                let var_map = self.graph.variable_map();
                // Sketch-on-face reference outline, captured before the
                // mutable node borrow below (used by the Sketch panel).
                let face_boundary = self
                    .graph
                    .sketch_face_boundaries
                    .get(&self.graph.graph[idx].id)
                    .cloned();
                let node = &mut self.graph.graph[idx];

                // Render inside a highly visual white inspector card
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(8.0)
                    .stroke(egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgb(226, 232, 240),
                    ))
                    .inner_margin(12.0)
                    .show(ui, |ui| {
                        ui.vertical(|ui| {
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("ID:").weak().size(11.5));
                                ui.label(
                                    egui::RichText::new(&node.id)
                                        .strong()
                                        .size(12.0)
                                        .color(pal.text_body),
                                );
                            });
                            ui.add_space(6.0);
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("Label:").size(12.0));
                                ui.text_edit_singleline(&mut node.name);
                            });
                            ui.add_space(8.0);
                            ui.separator();
                            ui.add_space(8.0);

                            match &mut node.feature {
                                FeatureType::Origin => {
                                    ui.label(
                                        egui::RichText::new(
                                            "📍 Base Origin coordinate planes space (0,0,0).",
                                        )
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::Box { w, h, d } => {
                                    ui.label(
                                        egui::RichText::new("Dimensions:")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(6.0);
                                    egui::Grid::new("box_grid")
                                        .spacing(egui::vec2(10.0, 10.0))
                                        .show(ui, |ui| {
                                            ui.label(egui::RichText::new("Width").size(12.0));
                                            let w_resp = ui.add(
                                                egui::Slider::new(w, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if w_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Height").size(12.0));
                                            let h_resp = ui.add(
                                                egui::Slider::new(h, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if h_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Depth").size(12.0));
                                            let d_resp = ui.add(
                                                egui::Slider::new(d, 5.0..=150.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if d_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();
                                        });
                                }
                                FeatureType::Import { step_data, label } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "📦 Imported STEP body \"{}\" ({:.1} KB embedded).",
                                            label,
                                            step_data.len() as f64 / 1024.0
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "The STEP file's contents are embedded in this \
                                             design, so it opens anywhere without the original \
                                             file.",
                                        )
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::DatumPlane { def } => match def {
                                    DatumPlaneDef::Offset { distance, .. } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Offset").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(distance)
                                                        .speed(0.5)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    DatumPlaneDef::Angle { angle_deg, .. } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Angle").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(angle_deg)
                                                        .speed(1.0)
                                                        .suffix("°"),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    DatumPlaneDef::ThreePoints { a, b, c } => {
                                        for (label, p) in [("A", a), ("B", b), ("C", c)] {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new(label).size(12.0));
                                                for coord in p.iter_mut() {
                                                    if ui
                                                        .add(egui::DragValue::new(coord).speed(0.5))
                                                        .changed()
                                                    {
                                                        modified = true;
                                                    }
                                                }
                                            });
                                        }
                                    }
                                    DatumPlaneDef::MidPlane { .. } => {
                                        ui.label(
                                            egui::RichText::new("Mid-plane between two planes.")
                                                .size(11.5)
                                                .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::DatumAxis { def } => match def {
                                    DatumAxisDef::TwoPoints { a, b } => {
                                        for (label, p) in [("From", a), ("To", b)] {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new(label).size(12.0));
                                                for coord in p.iter_mut() {
                                                    if ui
                                                        .add(egui::DragValue::new(coord).speed(0.5))
                                                        .changed()
                                                    {
                                                        modified = true;
                                                    }
                                                }
                                            });
                                        }
                                    }
                                    DatumAxisDef::PlaneIntersection { .. } => {
                                        ui.label(
                                            egui::RichText::new("Intersection line of two planes.")
                                                .size(11.5)
                                                .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::DatumPoint { def } => match def {
                                    DatumPointDef::Coords { p } => {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Position").size(12.0));
                                            for coord in p.iter_mut() {
                                                if ui
                                                    .add(egui::DragValue::new(coord).speed(0.5))
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            }
                                        });
                                    }
                                },
                                FeatureType::Revolve {
                                    angle_deg, mode, ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Mode: {:?}", mode))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Angle").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(angle_deg)
                                                    .speed(1.0)
                                                    .range(0.1..=360.0)
                                                    .suffix("°"),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                }
                                FeatureType::Loft { sections, mode, .. } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Loft through {} sections ({:?}).",
                                            sections.len(),
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::Sweep {
                                    path_sketch, mode, ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Sweep along '{path_sketch}' ({:?}).",
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                }
                                FeatureType::Shell { thickness, .. } => {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Thickness").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(thickness)
                                                    .speed(0.1)
                                                    .range(0.1..=100.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                }
                                FeatureType::Hole {
                                    position,
                                    diameter,
                                    depth,
                                    ..
                                } => {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Diameter").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(diameter)
                                                    .speed(0.2)
                                                    .range(0.1..=500.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    if let Some(d) = depth {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Depth").size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(d)
                                                        .speed(0.2)
                                                        .range(0.1..=1000.0)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Position").size(12.0));
                                        for coord in position.iter_mut() {
                                            if ui
                                                .add(egui::DragValue::new(coord).speed(0.5))
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        }
                                    });
                                }
                                FeatureType::Pattern { source, kind } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source: {source}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    match kind {
                                        zerocad_core::PatternKind::Linear {
                                            spacing,
                                            count,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Spacing").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(spacing)
                                                            .speed(0.5)
                                                            .suffix(current_unit.suffix()),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(2..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::PatternKind::Circular {
                                            count,
                                            total_angle_deg,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(2..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Total angle").size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(total_angle_deg)
                                                            .speed(1.0)
                                                            .range(1.0..=360.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::PatternKind::Mirror { .. } => {
                                            ui.label(
                                                egui::RichText::new("Mirrored copy of the source.")
                                                    .size(11.5)
                                                    .color(pal.text_muted),
                                            );
                                        }
                                    }
                                }
                                FeatureType::Cylinder { r, h } => {
                                    ui.label(
                                        egui::RichText::new("Dimensions:")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(6.0);
                                    egui::Grid::new("cyl_grid")
                                        .spacing(egui::vec2(10.0, 10.0))
                                        .show(ui, |ui| {
                                            ui.label(egui::RichText::new("Radius").size(12.0));
                                            let r_resp = ui.add(
                                                egui::Slider::new(r, 2.0..=80.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if r_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();

                                            ui.label(egui::RichText::new("Height").size(12.0));
                                            let h_resp = ui.add(
                                                egui::Slider::new(h, 5.0..=200.0)
                                                    .suffix(self.current_unit.suffix()),
                                            );
                                            if h_resp.changed() {
                                                modified = true;
                                            }
                                            ui.end_row();
                                        });
                                }
                                FeatureType::Sketch {
                                    cs,
                                    curves,
                                    shapes,
                                    corner_mods,
                                    solver,
                                    ..
                                } => {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Plane:").size(12.0));
                                        ui.label(
                                            egui::RichText::new(Self::cs_label(cs))
                                                .strong()
                                                .size(12.0)
                                                .color(egui::Color32::from_rgb(37, 99, 235)),
                                        );
                                    });
                                    ui.add_space(4.0);
                                    // Resolve against the current variables so the counts
                                    // (and any extrude below) reflect variable-driven dims.
                                    let mut eff = zerocad_core::effective_curves_solved(
                                        curves,
                                        shapes,
                                        corner_mods,
                                        solver.as_ref(),
                                        &var_map,
                                    );
                                    // Sketch-on-face: the projected boundary joins
                                    // region detection, so the face count here
                                    // matches the viewport and the evaluator.
                                    if let Some(b) = face_boundary.as_ref() {
                                        eff.extend_curves(b);
                                    }
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Curves: {} segments, {} circles",
                                            eff.segments.len(),
                                            eff.circles.len()
                                        ))
                                        .size(11.5)
                                        .weak(),
                                    );
                                    let regions = detect_regions(&eff);
                                    ui.label(
                                        egui::RichText::new(format!("Faces: {}", regions.len()))
                                            .size(11.5)
                                            .weak(),
                                    );
                                    // Surface any variable-bound dimensions so the user knows
                                    // the sketch is parametric (editing is done by redrawing).
                                    let bound = sketch_variable_dims(shapes);
                                    if !bound.is_empty() {
                                        ui.add_space(2.0);
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "🔗 Variable dims: {}",
                                                bound.join(", ")
                                            ))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(37, 99, 235)),
                                        );
                                    }
                                    ui.add_space(8.0);

                                    let has_faces = !regions.is_empty();
                                    let extrude_btn = icons::Icon::Extrude.labeled_button(
                                        ui,
                                        "Extrude whole Sketch",
                                        if has_faces {
                                            egui::Color32::from_rgb(37, 99, 235)
                                        } else {
                                            egui::Color32::from_rgb(241, 245, 249)
                                        },
                                        if has_faces {
                                            egui::Color32::from_rgb(29, 78, 216)
                                        } else {
                                            egui::Color32::from_rgb(241, 245, 249)
                                        },
                                        if has_faces {
                                            egui::Color32::WHITE
                                        } else {
                                            pal.text_faint
                                        },
                                        if has_faces {
                                            egui::Stroke::NONE
                                        } else {
                                            egui::Stroke::new(
                                                1.0,
                                                egui::Color32::from_rgb(226, 232, 240),
                                            )
                                        },
                                    );
                                    if has_faces && extrude_btn.clicked() {
                                        extrude_request = Some(node.id.clone());
                                    }
                                    ui.add_space(4.0);
                                    if ui
                                        .button(egui::RichText::new("✏ Edit Sketch").size(12.0))
                                        .on_hover_text(
                                            "Re-open this sketch: drag points, geometry \
                                             re-solves against its constraints; Finish \
                                             commits in place (downstream features follow).",
                                        )
                                        .clicked()
                                    {
                                        edit_sketch_request = Some(node.id.clone());
                                    }
                                    // Constraint status: DOF / fully constrained /
                                    // conflict, from the solver model when present.
                                    if let Some(model) = solver.as_ref().filter(|m| !m.is_empty()) {
                                        let report =
                                            zerocad_core::sketch::solve_model(model, &var_map);
                                        let (text, color) = match report.outcome {
                                            zerocad_core::sketch::SolveOutcome::Conflicting => (
                                                match report.conflicting {
                                                    Some(id) => format!(
                                                        "⚠ Over-constrained (conflict: {})",
                                                        id.0
                                                    ),
                                                    None => "⚠ Over-constrained".to_string(),
                                                },
                                                egui::Color32::from_rgb(220, 38, 38),
                                            ),
                                            zerocad_core::sketch::SolveOutcome::DidNotConverge => (
                                                "⚠ Constraints did not converge".to_string(),
                                                egui::Color32::from_rgb(220, 38, 38),
                                            ),
                                            zerocad_core::sketch::SolveOutcome::Converged => {
                                                if report.dof == 0 {
                                                    (
                                                        "● Fully constrained".to_string(),
                                                        egui::Color32::from_rgb(22, 163, 74),
                                                    )
                                                } else {
                                                    (
                                                        format!("◐ {} DOF remaining", report.dof),
                                                        egui::Color32::from_rgb(37, 99, 235),
                                                    )
                                                }
                                            }
                                        };
                                        ui.add_space(2.0);
                                        ui.label(egui::RichText::new(text).size(11.0).color(color));
                                    }
                                }
                                FeatureType::Extrude {
                                    depth,
                                    region_indices,
                                    mode,
                                    depth_expr,
                                    ..
                                } => {
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("Extrusion Depth:").size(12.0),
                                        );
                                        let d_resp = ui.add(
                                            egui::Slider::new(depth, 1.0..=150.0)
                                                .suffix(current_unit.suffix()),
                                        );
                                        if d_resp.changed() {
                                            // Dragging sets a literal depth — drop the binding.
                                            *depth_expr = None;
                                            modified = true;
                                        }
                                    });
                                    // Variable/expression binding: a depth like `width / 2`
                                    // re-evaluates whenever the variable changes. Empty clears it.
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buf = depth_expr.clone().unwrap_or_default();
                                        let r = ui.add(
                                            egui::TextEdit::singleline(&mut buf)
                                                .hint_text("expression, e.g. width / 2")
                                                .desired_width(150.0),
                                        );
                                        if r.changed() {
                                            let t = buf.trim();
                                            *depth_expr = if t.is_empty() {
                                                None
                                            } else {
                                                Some(t.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(e) = depth_expr.as_ref() {
                                        let txt = match zerocad_core::expr::eval(e, &var_map) {
                                            Ok(v) => {
                                                format!("→ {:.2} {}", v, current_unit.suffix())
                                            }
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(txt).size(11.0).weak());
                                    }
                                    ui.add_space(6.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Operation:").size(12.0));
                                        for (m, label) in [
                                            (ExtrudeMode::NewBody, "New Body"),
                                            (ExtrudeMode::Join, "Join"),
                                            (ExtrudeMode::Cut, "Cut"),
                                        ] {
                                            if ui.selectable_label(*mode == m, label).clicked()
                                                && *mode != m
                                            {
                                                *mode = m;
                                                modified = true;
                                            }
                                        }
                                    });
                                    ui.add_space(6.0);
                                    if region_indices.is_empty() {
                                        ui.label(
                                            egui::RichText::new("Regions: all detected")
                                                .size(11.5)
                                                .weak(),
                                        );
                                    } else {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "Regions: {:?}",
                                                region_indices
                                            ))
                                            .size(11.5)
                                            .weak(),
                                        );
                                    }
                                }
                                FeatureType::EdgeMod {
                                    dist,
                                    dist_expr,
                                    kind,
                                    ..
                                } => {
                                    let noun = match kind {
                                        CornerKind::Fillet => "Fillet radius:",
                                        CornerKind::Chamfer => "Chamfer distance:",
                                    };
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new(noun).size(12.0));
                                        let d_resp = ui.add(
                                            egui::Slider::new(dist, 0.2..=40.0)
                                                .suffix(current_unit.suffix()),
                                        );
                                        if d_resp.changed() {
                                            *dist_expr = None; // a literal drag drops the binding
                                            modified = true;
                                        }
                                    });
                                    // Variable/expression binding, mirroring the extrude depth.
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buf = dist_expr.clone().unwrap_or_default();
                                        let r = ui.add(
                                            egui::TextEdit::singleline(&mut buf)
                                                .hint_text("expression, e.g. fillet_r")
                                                .desired_width(150.0),
                                        );
                                        if r.changed() {
                                            let t = buf.trim();
                                            *dist_expr = if t.is_empty() {
                                                None
                                            } else {
                                                Some(t.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(e) = dist_expr.as_ref() {
                                        let txt = match zerocad_core::expr::eval(e, &var_map) {
                                            Ok(v) => {
                                                format!("→ {:.2} {}", v, current_unit.suffix())
                                            }
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(txt).size(11.0).weak());
                                    }
                                    ui.add_space(6.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Type:").size(12.0));
                                        for (k, label) in [
                                            (CornerKind::Fillet, "Fillet"),
                                            (CornerKind::Chamfer, "Chamfer"),
                                        ] {
                                            if ui.selectable_label(*kind == k, label).clicked()
                                                && *kind != k
                                            {
                                                *kind = k;
                                                modified = true;
                                            }
                                        }
                                    });
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "Edge captured in 3D; edits re-cut the body.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::VariableSet { variables } => {
                                    // Section header: "Variables" + count.
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("Variables")
                                                .strong()
                                                .size(12.0)
                                                .color(pal.text_strong),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!("({})", variables.len()))
                                                .size(11.5)
                                                .color(pal.text_faint),
                                        );
                                    });
                                    ui.add_space(8.0);

                                    if variables.is_empty() {
                                        egui::Frame::none()
                                            .fill(egui::Color32::from_rgb(248, 250, 252))
                                            .rounding(6.0)
                                            .inner_margin(10.0)
                                            .show(ui, |ui| {
                                                ui.label(
                                                    egui::RichText::new(
                                                        "No variables yet. Add one below.",
                                                    )
                                                    .size(11.5)
                                                    .color(pal.text_muted),
                                                );
                                            });
                                    }

                                    // Each variable is its own soft card: a full-width
                                    // name field on top, then value + unit + delete.
                                    let mut remove_idx: Option<usize> = None;
                                    for (i, var) in variables.iter_mut().enumerate() {
                                        egui::Frame::none()
                                            .fill(egui::Color32::from_rgb(248, 250, 252)) // slate-50
                                            .rounding(6.0)
                                            .stroke(egui::Stroke::new(
                                                1.0,
                                                egui::Color32::from_rgb(226, 232, 240),
                                            ))
                                            .inner_margin(8.0)
                                            .show(ui, |ui| {
                                                ui.add(
                                                    egui::TextEdit::singleline(&mut var.name)
                                                        .desired_width(f32::INFINITY)
                                                        .hint_text("name")
                                                        .font(egui::FontId::proportional(12.5)),
                                                );
                                                ui.add_space(6.0);
                                                ui.horizontal(|ui| {
                                                    ui.add(
                                                        egui::DragValue::new(&mut var.value)
                                                            .speed(0.1)
                                                            .min_decimals(0)
                                                            .max_decimals(3),
                                                    );
                                                    egui::ComboBox::from_id_salt(("var_unit", i))
                                                        .selected_text(var.unit.suffix())
                                                        .width(50.0)
                                                        .show_ui(ui, |ui| {
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Millimeter,
                                                                "mm",
                                                            );
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Inch,
                                                                "in",
                                                            );
                                                            ui.selectable_value(
                                                                &mut var.unit,
                                                                Unit::Meter,
                                                                "m",
                                                            );
                                                        });
                                                    ui.with_layout(
                                                        egui::Layout::right_to_left(
                                                            egui::Align::Center,
                                                        ),
                                                        |ui| {
                                                            let del = icons::Icon::Trash
                                                                .icon_button(
                                                                    ui,
                                                                    egui::Color32::TRANSPARENT,
                                                                    egui::Color32::from_rgb(
                                                                        254, 226, 226,
                                                                    ),
                                                                    egui::Color32::from_rgb(
                                                                        185, 28, 28,
                                                                    ),
                                                                );
                                                            if del
                                                                .on_hover_text("Delete variable")
                                                                .clicked()
                                                            {
                                                                remove_idx = Some(i);
                                                            }
                                                        },
                                                    );
                                                });
                                            });
                                        ui.add_space(6.0);
                                    }
                                    if let Some(i) = remove_idx {
                                        variables.remove(i);
                                    }

                                    ui.add_space(2.0);
                                    let add = icons::Icon::Sketch.labeled_button(
                                        ui,
                                        "Add Variable",
                                        egui::Color32::from_rgb(37, 99, 235),
                                        egui::Color32::from_rgb(29, 78, 216),
                                        egui::Color32::WHITE,
                                        egui::Stroke::NONE,
                                    );
                                    if add.clicked() {
                                        let n = variables.len() + 1;
                                        variables
                                            .push(Variable::new(format!("var{}", n), current_unit));
                                    }
                                }
                            }
                        });
                    });

                if modified {
                    self.reevaluate_geometry();
                }

                if let Some(sketch_id) = extrude_request {
                    self.begin_extrude_whole_sketch(&sketch_id);
                }
                if let Some(sketch_id) = edit_sketch_request {
                    let now = ui.input(|i| i.time);
                    self.edit_sketch(&sketch_id, now);
                }

                // Measure: physical properties of the selected body's mesh.
                // Computed on demand from the already-tessellated buffers, so
                // it's exact for what's on screen.
                let measured = self
                    .selected_node_id
                    .as_ref()
                    .and_then(|id| self.body_meshes.iter().find(|(mid, _)| mid == id))
                    .and_then(|(_, mesh)| mesh.mass_properties());
                if let Some(mp) = measured {
                    ui.add_space(10.0);
                    egui::Frame::none()
                        .fill(egui::Color32::WHITE)
                        .rounding(8.0)
                        .stroke(egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgb(226, 232, 240),
                        ))
                        .inner_margin(12.0)
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new("Measure")
                                    .strong()
                                    .size(12.5)
                                    .color(self.pal().text_strong),
                            );
                            ui.add_space(4.0);
                            let muted = self.pal().text_muted;
                            let row = |ui: &mut egui::Ui, k: &str, v: String| {
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new(k).size(11.5).color(muted));
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.label(egui::RichText::new(v).monospace().size(11.5));
                                        },
                                    );
                                });
                            };
                            row(ui, "Volume", format!("{:.3} mm³", mp.volume));
                            row(ui, "Surface area", format!("{:.3} mm²", mp.surface_area));
                            row(
                                ui,
                                "Centroid",
                                format!(
                                    "({:.2}, {:.2}, {:.2})",
                                    mp.centroid[0], mp.centroid[1], mp.centroid[2]
                                ),
                            );
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new("Density (g/cm³)")
                                        .size(11.5)
                                        .color(self.pal().text_muted),
                                );
                                ui.add(
                                    egui::DragValue::new(&mut self.measure_density)
                                        .speed(0.05)
                                        .range(0.0..=30.0),
                                );
                            });
                            // mm³ → cm³ is ÷1000; g/cm³ × cm³ = grams.
                            let grams = mp.volume / 1000.0 * self.measure_density as f64;
                            row(ui, "Mass", format!("{grams:.2} g"));
                        });
                }
            }
        } else {
            // Render a clean fallback banner
            egui::Frame::none()
                .fill(egui::Color32::from_rgb(248, 250, 252))
                .stroke(egui::Stroke::new(
                    1.0,
                    egui::Color32::from_rgb(226, 232, 240),
                ))
                .rounding(6.0)
                .inner_margin(12.0)
                .show(ui, |ui| {
                    ui.centered_and_justified(|ui| {
                        ui.label(
                            egui::RichText::new(
                                "ℹ️ Select a feature from the tree to edit properties.",
                            )
                            .weak()
                            .size(12.0),
                        );
                    });
                });
        }
    }
}
