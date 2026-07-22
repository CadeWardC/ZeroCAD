use crate::*;
use zerocad_core::{DatumAxisDef, DatumPlaneDef, DatumPointDef, LoftSurfaceMode};

/// Edit the persisted source of an expression-backed numeric property and show
/// its current evaluated value without replacing the source text.
fn expression_editor(
    ui: &mut egui::Ui,
    expression: &mut Option<String>,
    vars: &std::collections::HashMap<String, f64>,
    hint: &str,
    suffix: &str,
    modified: &mut bool,
) {
    ui.add_space(3.0);
    ui.horizontal(|ui| {
        ui.label("=");
        let mut source = expression.clone().unwrap_or_default();
        if ui
            .add(
                egui::TextEdit::singleline(&mut source)
                    .hint_text(hint)
                    .desired_width(150.0),
            )
            .changed()
        {
            let source = source.trim();
            *expression = (!source.is_empty()).then(|| source.to_string());
            *modified = true;
        }
    });
    if let Some(source) = expression.as_ref() {
        let result = match zerocad_core::expr::eval(source, vars) {
            Ok(value) => format!("→ {value:.2} {suffix}"),
            Err(error) => format!("→ {error}"),
        };
        ui.label(egui::RichText::new(result).size(11.0).weak());
    }
}

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
            for idx in self.document.graph.node_indices() {
                if self.document.graph[idx].id == *selected_id {
                    node_idx = Some(idx);
                    break;
                }
            }

            if let Some(idx) = node_idx {
                // Deferred action: extruding needs `&mut self`, but
                // `node` holds a mutable borrow of the graph below.
                let mut extrude_request: Option<String> = None;
                let mut edit_sketch_request: Option<String> = None;
                let selected_feature_id = self.document.graph[idx].id.clone();
                let feature_suppressed = self.document.is_feature_suppressed(&selected_feature_id);
                let mut suppression_request: Option<bool> = None;
                let mut modified = false;

                // Capture palette + unit + the variable map before
                // borrowing the graph mutably (so the Extrude panel can
                // show what an expression-driven depth resolves to).
                let pal = self.pal();
                let current_unit = self.current_unit;
                let var_map = self.document.variable_map();
                let mirror_join_outcome = match &self.document.graph[idx].feature {
                    FeatureType::Pattern {
                        source,
                        kind: zerocad_core::PatternKind::Mirror { join: true, .. },
                    } => Some(self.mirror_join_outcome(&selected_feature_id, source)),
                    _ => None,
                };
                let current_body_center = self
                    .body_meshes
                    .iter()
                    .find(|(body_id, _)| body_id == &selected_feature_id)
                    .and_then(|(_, mesh)| zerocad_core::parametric::body_bounds_center(mesh));
                // Sketch-on-face reference outline, captured before the
                // mutable node borrow below (used by the Sketch panel).
                let face_boundary = self
                    .document
                    .sketch_face_boundaries
                    .get(self.document.graph[idx].id.as_str())
                    .cloned();
                let node = &mut self.document.graph[idx];

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
                            if node.id != "origin" {
                                ui.add_space(4.0);
                                let mut participates_in_rebuild = !feature_suppressed;
                                if ui
                                    .checkbox(
                                        &mut participates_in_rebuild,
                                        "Participates in rebuild",
                                    )
                                    .on_hover_text(
                                        "Suppressed features stay in the timeline but do not evaluate",
                                    )
                                    .changed()
                                {
                                    suppression_request = Some(!participates_in_rebuild);
                                }
                            }
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
                                    DatumPlaneDef::PlanarFace { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative plane from a named planar face.",
                                            )
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
                                    DatumAxisDef::Edge { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis from a named edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumAxisDef::CylindricalOrConicalFace { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis from an analytic cylindrical or conical face.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumAxisDef::TwoVertices { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative axis through two named vertices.",
                                            )
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
                                    DatumPointDef::Vertex { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative point from a named vertex.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::Midpoint { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative midpoint between two named vertices.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::EdgeMidpoint { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative midpoint of a named edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                    DatumPointDef::CircleCenter { .. } => {
                                        ui.label(
                                            egui::RichText::new(
                                                "Associative center of a named circular edge.",
                                            )
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                    }
                                },
                                FeatureType::Revolve {
                                    angle_deg,
                                    angle_expr,
                                    mode,
                                    ..
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
                                            *angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        angle_expr,
                                        &var_map,
                                        "expression, e.g. 360/2",
                                        "°",
                                        &mut modified,
                                    );
                                }
                                FeatureType::Loft {
                                    sections,
                                    surface_mode,
                                    mode,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Loft through {} sections ({:?}).",
                                            sections.len(),
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    let previous = *surface_mode;
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Surface").size(12.0));
                                        egui::ComboBox::from_id_salt("loft_surface_mode")
                                            .selected_text(format!("{surface_mode:?}"))
                                            .show_ui(ui, |ui| {
                                                ui.selectable_value(
                                                    surface_mode,
                                                    LoftSurfaceMode::Ruled,
                                                    "Ruled",
                                                );
                                                ui.selectable_value(
                                                    surface_mode,
                                                    LoftSurfaceMode::Smooth,
                                                    "Smooth",
                                                );
                                            });
                                    });
                                    modified |= *surface_mode != previous;
                                }
                                FeatureType::Sweep {
                                    path_sketch,
                                    guide,
                                    mode,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Sweep along '{path_sketch}' ({:?}).",
                                            mode
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    if let Some(guide) = guide {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "Guide '{}' Â· profile anchor {:.0}%",
                                                guide.sketch,
                                                guide.profile_parameter * 100.0
                                            ))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                        );
                                        ui.horizontal(|ui| {
                                            ui.label(
                                                egui::RichText::new("Anchor position").size(12.0),
                                            );
                                            if ui
                                                .add(
                                                    egui::DragValue::new(
                                                        &mut guide.profile_parameter,
                                                    )
                                                    .range(0.0..=1.0)
                                                    .speed(0.01),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                }
                                FeatureType::Shell {
                                    thickness,
                                    thickness_expr,
                                    ..
                                } => {
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
                                            *thickness_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        thickness_expr,
                                        &var_map,
                                        "expression, e.g. 5/2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                }
                                FeatureType::Hole {
                                    position,
                                    diameter,
                                    diameter_expr,
                                    depth,
                                    kind,
                                    standard,
                                    manufacturing,
                                    ..
                                } => {
                                    if let Some(reference) = standard.as_ref() {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{} {} · {} v{}",
                                                reference.family.label(),
                                                reference.designation,
                                                reference.table,
                                                reference.library_version
                                            ))
                                            .strong()
                                            .size(11.5)
                                            .color(pal.text_strong),
                                        );
                                        if let Some(class) = &reference.class {
                                            ui.label(
                                                egui::RichText::new(format!("Class {class}"))
                                                    .small()
                                                    .color(pal.text_muted),
                                            );
                                        }
                                    }
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
                                            *diameter_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        diameter_expr,
                                        &var_map,
                                        "expression, e.g. 43/2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
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
                                    if let Some(metadata) = manufacturing.as_mut() {
                                        if metadata.cosmetic_thread {
                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "Tapped-hole metadata: {} · class {} · pitch {} mm (cosmetic)",
                                                    metadata
                                                        .thread_designation
                                                        .as_deref()
                                                        .unwrap_or("custom"),
                                                    metadata.thread_class.as_deref().unwrap_or("—"),
                                                    metadata
                                                        .tap_pitch_mm
                                                        .map_or_else(|| "—".to_string(), |pitch| pitch.to_string())
                                                ))
                                                .small()
                                                .color(pal.text_muted),
                                            );
                                        }
                                        if let Some(angle) = &mut metadata.drill_point_angle_deg {
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Drill-point angle")
                                                        .size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(angle)
                                                            .speed(0.5)
                                                            .range(1.0..=179.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
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
                                    if let Some(reference) = standard.as_mut() {
                                        let (head_diameter, head_depth, head_angle) = match kind {
                                            zerocad_core::HoleKind::Simple => (None, None, None),
                                            zerocad_core::HoleKind::Counterbore {
                                                diameter,
                                                depth,
                                            } => {
                                                (Some(*diameter), Some(*depth), None)
                                            }
                                            zerocad_core::HoleKind::Countersink {
                                                diameter,
                                                angle_deg,
                                            } => (Some(*diameter), None, Some(*angle_deg)),
                                        };
                                        if let zerocad_core::ResolvedStandardGeometry::Hole {
                                            bore_diameter_mm,
                                            head_diameter_mm,
                                            head_depth_mm,
                                            head_angle_deg,
                                            ..
                                        } = &mut reference.resolved
                                        {
                                            *bore_diameter_mm = *diameter;
                                            *head_diameter_mm = head_diameter;
                                            *head_depth_mm = head_depth;
                                            *head_angle_deg = head_angle;
                                        }
                                    }
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
                                            spacing_expr,
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
                                                    *spacing_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                spacing_expr,
                                                &var_map,
                                                "expression, e.g. pitch*2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
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
                                        zerocad_core::PatternKind::Mirror {
                                            offset,
                                            offset_expr,
                                            join,
                                            ..
                                        } => {
                                            ui.label(
                                                egui::RichText::new("Mirrored copy of the source.")
                                                    .size(11.5)
                                                    .color(pal.text_muted),
                                            );
                                            if *join {
                                                let (text, color) = match mirror_join_outcome
                                                    .as_ref()
                                                    .expect("Mirror+Join outcome is precomputed")
                                                {
                                                    MirrorJoinOutcome::Evaluating => (
                                                        "Outcome: Evaluating join".to_string(),
                                                        pal.text_muted,
                                                    ),
                                                    MirrorJoinOutcome::Joined => (
                                                        "Outcome: Joined into source body".to_string(),
                                                        egui::Color32::from_rgb(22, 101, 52),
                                                    ),
                                                    MirrorJoinOutcome::Separate => (
                                                        "Outcome: Separate body (Join could not connect)"
                                                            .to_string(),
                                                        egui::Color32::from_rgb(180, 83, 9),
                                                    ),
                                                    MirrorJoinOutcome::Unresolved(reason) => (
                                                        format!("Outcome: Unresolved — {reason}"),
                                                        egui::Color32::from_rgb(185, 28, 28),
                                                    ),
                                                };
                                                ui.label(
                                                    egui::RichText::new(text)
                                                        .strong()
                                                        .size(11.5)
                                                        .color(color),
                                                );
                                            }
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Offset").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(offset)
                                                            .speed(0.2)
                                                            .suffix(current_unit.suffix()),
                                                    )
                                                    .changed()
                                                {
                                                    *offset_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                offset_expr,
                                                &var_map,
                                                "expression, e.g. width/2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
                                        }
                                    }
                                }
                                FeatureType::FeaturePattern {
                                    target,
                                    source_feature,
                                    kind,
                                    compute_mode,
                                    extent_policy,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Source feature: {source_feature}  •  Target: {target}"
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Compute: {compute_mode:?}  •  Extent: {extent_policy:?}"
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    match kind {
                                        zerocad_core::parametric::FeaturePatternKind::Linear {
                                            spacing,
                                            spacing_expr,
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
                                                    *spacing_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                spacing_expr,
                                                &var_map,
                                                "expression, e.g. pitch*2",
                                                current_unit.suffix(),
                                                &mut modified,
                                            );
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(0..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                        zerocad_core::parametric::FeaturePatternKind::Circular {
                                            total_angle_deg,
                                            total_angle_expr,
                                            count,
                                            ..
                                        } => {
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    egui::RichText::new("Total angle").size(12.0),
                                                );
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(total_angle_deg)
                                                            .speed(1.0)
                                                            .suffix("°"),
                                                    )
                                                    .changed()
                                                {
                                                    *total_angle_expr = None;
                                                    modified = true;
                                                }
                                            });
                                            expression_editor(
                                                ui,
                                                total_angle_expr,
                                                &var_map,
                                                "expression, e.g. full_angle",
                                                "°",
                                                &mut modified,
                                            );
                                            ui.horizontal(|ui| {
                                                ui.label(egui::RichText::new("Count").size(12.0));
                                                if ui
                                                    .add(
                                                        egui::DragValue::new(count)
                                                            .speed(0.1)
                                                            .range(0..=200),
                                                    )
                                                    .changed()
                                                {
                                                    modified = true;
                                                }
                                            });
                                        }
                                    }
                                }
                                FeatureType::Draft {
                                    target,
                                    faces,
                                    neutral,
                                    angle_deg,
                                    angle_expr,
                                    flip_pull,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Target: {target}  •  Selected side faces: {}",
                                            faces.len()
                                        ))
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    let neutral_label = match neutral {
                                        zerocad_core::parametric::DraftNeutral::Face(face) => face
                                            .topology
                                            .as_ref()
                                            .and_then(|topology| topology.face_id.as_deref())
                                            .map_or_else(
                                                || "legacy face reference".to_string(),
                                                ToString::to_string,
                                            ),
                                        zerocad_core::parametric::DraftNeutral::Datum(id) => {
                                            format!("datum {id}")
                                        }
                                    };
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Neutral reference: {neutral_label}"
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Draft angle").size(12.0));
                                        if ui
                                            .add(
                                                egui::DragValue::new(angle_deg)
                                                    .speed(0.25)
                                                    .range(-88.9..=88.9)
                                                    .suffix("°"),
                                            )
                                            .changed()
                                        {
                                            *angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        angle_expr,
                                        &var_map,
                                        "expression, e.g. taper_angle",
                                        "°",
                                        &mut modified,
                                    );
                                    if ui
                                        .checkbox(flip_pull, "Reverse pull direction")
                                        .changed()
                                    {
                                        modified = true;
                                    }
                                    ui.add_space(4.0);
                                    ui.label(
                                        egui::RichText::new(
                                            "Draft v1 supports straight planar side faces on retained prismatic bodies.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_muted),
                                    );
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
                                    mirrors,
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
                                        mirrors,
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
                                    // Surface stored expression dimensions so their editable
                                    // source remains visible after evaluation.
                                    let bound = sketch_variable_dims(shapes);
                                    if !bound.is_empty() {
                                        ui.add_space(2.0);
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "🔗 Expression dims: {}",
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
                                    draft_angle_deg,
                                    draft_angle_expr,
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
                                        ui.label(egui::RichText::new("Draft Angle:").size(12.0));
                                        let response = ui.add(
                                            egui::Slider::new(draft_angle_deg, -88.9..=88.9)
                                                .suffix("°"),
                                        );
                                        if response.changed() {
                                            *draft_angle_expr = None;
                                            modified = true;
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("=")
                                                .size(13.0)
                                                .color(pal.text_muted),
                                        );
                                        let mut buffer =
                                            draft_angle_expr.clone().unwrap_or_default();
                                        let response = ui.add(
                                            egui::TextEdit::singleline(&mut buffer)
                                                .hint_text("angle expression")
                                                .desired_width(150.0),
                                        );
                                        if response.changed() {
                                            let value = buffer.trim();
                                            *draft_angle_expr = if value.is_empty() {
                                                None
                                            } else {
                                                Some(value.to_string())
                                            };
                                            modified = true;
                                        }
                                    });
                                    if let Some(expression) = draft_angle_expr.as_ref() {
                                        let text = match zerocad_core::expr::eval(
                                            expression,
                                            &var_map,
                                        ) {
                                            Ok(value) if value.abs() < 89.0 => {
                                                format!("→ {value:.3}°")
                                            }
                                            Ok(value) => format!(
                                                "→ {value:.3}° (invalid: must be between -89° and 89°)"
                                            ),
                                            Err(_) => {
                                                "→ unresolved (check variable names)".to_string()
                                            }
                                        };
                                        ui.label(egui::RichText::new(text).size(11.0).weak());
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
                                FeatureType::BodyTransform {
                                    source,
                                    translation,
                                    copy,
                                } => {
                                    ui.label(
                                        egui::RichText::new(if *copy {
                                            format!("Copy of {source}")
                                        } else {
                                            format!("Moved from {source}")
                                        })
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.add_space(4.0);
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new(label).size(12.0));
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut translation[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                }
                                FeatureType::BodyJoin { sources } => {
                                    ui.label(
                                        egui::RichText::new("Joined bodies")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.add_space(4.0);
                                    for source in sources {
                                        ui.label(
                                            egui::RichText::new(source.as_str())
                                                .size(11.5)
                                                .color(pal.text_muted),
                                        );
                                    }
                                }
                                FeatureType::BodyCut {
                                    target,
                                    tool,
                                    keep_tool,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Target: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!("Cutting body: {tool}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    if ui.checkbox(keep_tool, "Keep cutting body").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::BodyIntersect {
                                    target,
                                    tool,
                                    keep_tool,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Target: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!("Intersecting body: {tool}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    if ui.checkbox(keep_tool, "Keep tool body").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::BodySplit {
                                    target, plane, face, ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(if face.is_some() {
                                            "Reference: planar body face".to_string()
                                        } else {
                                            format!("Reference: {plane:?}")
                                        })
                                        .size(11.5)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new("Outputs: negative and positive side")
                                            .size(10.5)
                                            .color(pal.text_faint),
                                    );
                                }
                                FeatureType::BodyScale {
                                    source,
                                    factor,
                                    factor_expr,
                                    center,
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source: {source}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Factor");
                                        if let Some(expression) = factor_expr {
                                            if ui.text_edit_singleline(expression).changed() {
                                                modified = true;
                                            }
                                        } else if ui
                                            .add(egui::DragValue::new(factor).speed(0.05))
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.label("Pivot");
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(label);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut center[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    if let Some(body_center) = current_body_center {
                                        if ui.button("Reset pivot to current center").clicked() {
                                            *center = body_center;
                                            modified = true;
                                        }
                                    }
                                }
                                FeatureType::FaceOffset {
                                    target,
                                    distance,
                                    distance_expr,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Signed distance");
                                        if ui
                                            .add(
                                                egui::DragValue::new(distance)
                                                    .speed(0.25)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *distance_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        distance_expr,
                                        &var_map,
                                        "expression, e.g. wall+2",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Positive adds material; negative removes it.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceMove {
                                    target,
                                    translation,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label("Normal translation");
                                    for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.label(label);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut translation[axis])
                                                        .speed(0.25)
                                                        .suffix(current_unit.suffix()),
                                                )
                                                .changed()
                                            {
                                                modified = true;
                                            }
                                        });
                                    }
                                    ui.label(
                                        egui::RichText::new(
                                            "Tangential movement is rejected in Phase 5.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceDelete { target, .. } => {
                                    ui.label(
                                        egui::RichText::new(format!("Body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Heals an internal cylindrical face (hole removal).",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::FaceThicken {
                                    target,
                                    thickness,
                                    thickness_expr,
                                    reverse,
                                    ..
                                } => {
                                    ui.label(
                                        egui::RichText::new(format!("Source body: {target}"))
                                            .size(11.5)
                                            .color(pal.text_muted),
                                    );
                                    ui.horizontal(|ui| {
                                        ui.label("Thickness");
                                        if ui
                                            .add(
                                                egui::DragValue::new(thickness)
                                                    .speed(0.25)
                                                    .range(0.001..=1.0e6)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            *thickness_expr = None;
                                            modified = true;
                                        }
                                    });
                                    expression_editor(
                                        ui,
                                        thickness_expr,
                                        &var_map,
                                        "expression, e.g. sheet_thickness",
                                        current_unit.suffix(),
                                        &mut modified,
                                    );
                                    if ui.checkbox(reverse, "Reverse direction").changed() {
                                        modified = true;
                                    }
                                }
                                FeatureType::ImportStl { stl_data, label } => {
                                    ui.label(
                                        egui::RichText::new("STL mesh body")
                                            .strong()
                                            .size(12.0)
                                            .color(pal.text_strong),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{} · {} bytes · mesh-only",
                                            label,
                                            stl_data.len()
                                        ))
                                        .size(11.0)
                                        .color(pal.text_muted),
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            "Supports move, scale, measure, section, and mesh export; B-Rep booleans are disabled.",
                                        )
                                        .size(10.5)
                                        .color(pal.text_faint),
                                    );
                                }
                                FeatureType::Thread {
                                    internal,
                                    pitch,
                                    depth,
                                    angle_deg,
                                    right_handed,
                                    designation,
                                    standard,
                                    ..
                                } => {
                                    let original_internal = *internal;
                                    if let Some(reference) = standard.as_ref() {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{} · {} v{} · class {}",
                                                reference.table,
                                                reference.library_id,
                                                reference.library_version,
                                                reference.class.as_deref().unwrap_or("—")
                                            ))
                                            .small()
                                            .color(pal.text_muted),
                                        );
                                    }
                                    if !designation.is_empty() {
                                        ui.label(
                                            egui::RichText::new(designation.as_str())
                                                .strong()
                                                .size(12.0)
                                                .color(pal.text_strong),
                                        );
                                        ui.add_space(4.0);
                                    }
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Pitch").size(12.0));
                                        if ui
                                            .add(
                                                egui::Slider::new(pitch, 0.2..=6.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new("Depth").size(12.0));
                                        if ui
                                            .add(
                                                egui::Slider::new(depth, 0.05..=3.0)
                                                    .suffix(current_unit.suffix()),
                                            )
                                            .changed()
                                        {
                                            modified = true;
                                        }
                                    });
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        if ui.selectable_label(!*internal, "External").clicked()
                                            && *internal
                                        {
                                            *internal = false;
                                            modified = true;
                                        }
                                        if ui.selectable_label(*internal, "Internal").clicked()
                                            && !*internal
                                        {
                                            *internal = true;
                                            modified = true;
                                        }
                                        ui.separator();
                                        if ui.selectable_label(*right_handed, "RH").clicked()
                                            && !*right_handed
                                        {
                                            *right_handed = true;
                                            modified = true;
                                        }
                                        if ui.selectable_label(!*right_handed, "LH").clicked()
                                            && *right_handed
                                        {
                                            *right_handed = false;
                                            modified = true;
                                        }
                                    });
                                    ui.add_space(4.0);
                                    if let Some(reference) = standard.as_mut() {
                                        let thread_standard = match reference.family {
                                            zerocad_core::StandardsFamily::Iso => Some(
                                                zerocad_core::parametric::ThreadStandard::MetricCoarse,
                                            ),
                                            zerocad_core::StandardsFamily::Ansi
                                                if reference.table.contains("UNF") =>
                                            {
                                                Some(
                                                    zerocad_core::parametric::ThreadStandard::UnifiedFine,
                                                )
                                            }
                                            zerocad_core::StandardsFamily::Ansi => Some(
                                                zerocad_core::parametric::ThreadStandard::UnifiedCoarse,
                                            ),
                                        };
                                        if let Some(thread_standard) = thread_standard {
                                            let classes = zerocad_core::parametric::thread_classes(
                                                thread_standard,
                                                *internal,
                                            );
                                            if original_internal != *internal
                                                || reference.class.as_deref().is_none_or(|class| {
                                                    !classes.contains(&class)
                                                })
                                            {
                                                reference.class = classes
                                                    .get(1)
                                                    .copied()
                                                    .map(str::to_string);
                                            }
                                            let mut selected =
                                                reference.class.clone().unwrap_or_default();
                                            ui.horizontal(|ui| {
                                                ui.label("Class");
                                                egui::ComboBox::from_id_salt(
                                                    "thread_property_class",
                                                )
                                                .selected_text(&selected)
                                                .show_ui(ui, |ui| {
                                                    for class in classes {
                                                        ui.selectable_value(
                                                            &mut selected,
                                                            (*class).to_string(),
                                                            *class,
                                                        );
                                                    }
                                                });
                                            });
                                            if reference.class.as_deref()
                                                != Some(selected.as_str())
                                            {
                                                reference.class = Some(selected);
                                                modified = true;
                                            }
                                        }
                                        if let zerocad_core::ResolvedStandardGeometry::Thread {
                                            pitch_mm,
                                            radial_depth_mm,
                                            angle_deg: resolved_angle,
                                            ..
                                        } = &mut reference.resolved
                                        {
                                            *pitch_mm = *pitch;
                                            *radial_depth_mm = *depth;
                                            *resolved_angle = *angle_deg;
                                        }
                                    }
                                    ui.label(
                                        egui::RichText::new(
                                            "Cut into the selected cylindrical face. A failed \
                                             helical boolean leaves the body intact (cosmetic).",
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

                if let Some(suppressed) = suppression_request {
                    self.push_undo();
                    if self
                        .document
                        .set_feature_suppressed(&selected_feature_id, suppressed)
                    {
                        self.status_msg = if suppressed {
                            format!("Suppressed feature '{selected_feature_id}'.")
                        } else {
                            format!("Resumed feature '{selected_feature_id}'.")
                        };
                        modified = true;
                    }
                }

                if modified {
                    match self.document.commit_feature_edit(&selected_feature_id) {
                        Ok(()) => self.reevaluate_geometry(),
                        Err(error) => {
                            log::error!(
                                "Failed to synchronize feature '{}': {error}",
                                selected_feature_id
                            );
                            self.status_msg = format!(
                                "Feature '{}' has inconsistent document relationships: {error}",
                                selected_feature_id
                            );
                        }
                    }
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
