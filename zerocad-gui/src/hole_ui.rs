//! Hole tool UI: select a body face → set diameter/depth/head style → commit
//! a `FeatureType::Hole` node. v1 places the hole at the selected face's
//! centroid (drilling along the inward normal); the position is editable in
//! the Properties panel afterwards.

use crate::*;
use zerocad_core::{HoleApplication, HoleFit, HoleKind, HoleStandardPreset, StandardsFamily};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum HoleKindChoice {
    Simple,
    Counterbore,
    Countersink,
}

/// State of the in-progress hole tool.
#[derive(Debug, Clone)]
pub(crate) struct HoleOp {
    pub(crate) target: String,
    pub(crate) position: [f32; 3],
    pub(crate) direction: [f32; 3],
    pub(crate) diameter_text: String,
    pub(crate) through: bool,
    pub(crate) depth_text: String,
    pub(crate) use_drill_point: bool,
    pub(crate) tip_angle_text: String,
    pub(crate) kind: HoleKindChoice,
    pub(crate) head_diameter_text: String,
    pub(crate) head_depth_text: String,
    pub(crate) head_angle_text: String,
    pub(crate) use_standard: bool,
    pub(crate) standard_family: StandardsFamily,
    pub(crate) standard_application: HoleApplication,
    pub(crate) standard_fit: HoleFit,
    pub(crate) standard_preset_idx: usize,
}

impl HoleOp {
    fn standard_presets(&self) -> Vec<&'static HoleStandardPreset> {
        zerocad_core::parametric::hole_presets(
            self.standard_family,
            self.standard_application,
            self.standard_fit,
        )
        .collect()
    }

    fn selected_standard(&self) -> Option<&'static HoleStandardPreset> {
        self.standard_presets()
            .get(self.standard_preset_idx)
            .copied()
    }

    fn seed_from_standard(&mut self) {
        let Some(preset) = self.selected_standard().copied() else {
            return;
        };
        self.diameter_text = format_dimension(preset.bore_diameter_mm);
        self.kind = match preset.application {
            HoleApplication::Clearance | HoleApplication::Tapped => HoleKindChoice::Simple,
            HoleApplication::Counterbore => HoleKindChoice::Counterbore,
            HoleApplication::Countersink => HoleKindChoice::Countersink,
        };
        if let Some(value) = preset.head_diameter_mm {
            self.head_diameter_text = format_dimension(value);
        }
        if let Some(value) = preset.head_depth_mm {
            self.head_depth_text = format_dimension(value);
        }
        if let Some(value) = preset.head_angle_deg {
            self.head_angle_text = format_dimension(value);
        }
    }
}

fn format_dimension(value: f32) -> String {
    let text = format!("{value:.4}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn draw_hole_standard_picker(ui: &mut egui::Ui, op: &mut HoleOp) {
    ui.checkbox(&mut op.use_standard, "Use engineering standard");
    if !op.use_standard {
        return;
    }
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Family");
        for family in [StandardsFamily::Iso, StandardsFamily::Ansi] {
            if ui
                .selectable_label(op.standard_family == family, family.label())
                .clicked()
            {
                op.standard_family = family;
                op.standard_preset_idx = 0;
                changed = true;
            }
        }
    });
    ui.horizontal_wrapped(|ui| {
        ui.label("Type");
        for application in [
            HoleApplication::Clearance,
            HoleApplication::Tapped,
            HoleApplication::Counterbore,
            HoleApplication::Countersink,
        ] {
            if ui
                .selectable_label(op.standard_application == application, application.label())
                .clicked()
            {
                op.standard_application = application;
                op.standard_fit = HoleFit::Normal;
                op.standard_preset_idx = 0;
                changed = true;
            }
        }
    });
    if op.standard_application == HoleApplication::Clearance {
        ui.horizontal(|ui| {
            ui.label("Fit");
            for fit in [HoleFit::Close, HoleFit::Normal, HoleFit::Loose] {
                let has_rows = zerocad_core::parametric::hole_presets(
                    op.standard_family,
                    op.standard_application,
                    fit,
                )
                .next()
                .is_some();
                if ui
                    .add_enabled(
                        has_rows,
                        egui::SelectableLabel::new(op.standard_fit == fit, fit.label()),
                    )
                    .clicked()
                {
                    op.standard_fit = fit;
                    op.standard_preset_idx = 0;
                    changed = true;
                }
            }
        });
    }
    let presets = op.standard_presets();
    let selected = presets
        .get(op.standard_preset_idx)
        .map_or("—", |preset| preset.designation);
    ui.horizontal(|ui| {
        ui.label("Size");
        egui::ComboBox::from_id_salt("hole_standard_size")
            .selected_text(selected)
            .show_ui(ui, |ui| {
                for (index, preset) in presets.iter().enumerate() {
                    if ui
                        .selectable_value(&mut op.standard_preset_idx, index, preset.designation)
                        .changed()
                    {
                        changed = true;
                    }
                }
            });
        if ui.small_button("Reset dimensions").clicked() {
            changed = true;
        }
    });
    if changed {
        op.seed_from_standard();
    }
    if let Some(preset) = op.selected_standard() {
        ui.label(
            egui::RichText::new(format!(
                "{} v{} · {} · class {}",
                zerocad_core::STANDARDS_LIBRARY_ID,
                zerocad_core::STANDARDS_LIBRARY_VERSION,
                preset.table,
                preset.class.unwrap_or("—")
            ))
            .small()
            .weak(),
        );
    }
    ui.label(
        egui::RichText::new(
            "Resolved dimensions below are stored with the feature and may be overridden.",
        )
        .small()
        .weak(),
    );
}

impl ZeroCadApp {
    /// The face the hole tool would drill into: exactly one selected body face.
    pub(crate) fn hole_face_candidate(&self) -> Option<(String, u32)> {
        let faces: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(id, pick)| match pick {
                BodyPick::Face(f) => Some((id.clone(), *f)),
                _ => None,
            })
            .collect();
        (faces.len() == 1).then(|| faces.into_iter().next().unwrap())
    }

    pub(crate) fn begin_hole(&mut self, node: String, fid: u32) {
        if self.hole_op.is_some() {
            return;
        }
        let Some(fref) = self.face_ref(&node, fid) else {
            self.status_msg = "Couldn't resolve the selected face for a hole.".to_string();
            return;
        };
        let n = fref.normal;
        self.hole_op = Some(HoleOp {
            target: node,
            position: fref.centroid,
            direction: [-n[0], -n[1], -n[2]],
            diameter_text: "6.6".to_string(),
            through: true,
            depth_text: "10".to_string(),
            use_drill_point: false,
            tip_angle_text: "118".to_string(),
            kind: HoleKindChoice::Simple,
            head_diameter_text: "11".to_string(),
            head_depth_text: "3".to_string(),
            head_angle_text: "90".to_string(),
            use_standard: true,
            standard_family: StandardsFamily::Iso,
            standard_application: HoleApplication::Clearance,
            standard_fit: HoleFit::Normal,
            standard_preset_idx: 3,
        });
        self.status_msg =
            "Hole: set the size and head style, then OK. Placed at the face centre (editable \
             afterwards in Properties)."
                .to_string();
    }

    /// The floating Hole dialog (drawn every frame while an op is active).
    pub(crate) fn show_hole_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.hole_op.clone() else {
            return;
        };
        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        let var_map = self.visible_variable_map();
        egui::Area::new(egui::Id::new("hole_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(240.0);
                    ui.label(
                        egui::RichText::new("Hole")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);
                    draw_hole_standard_picker(ui, &mut op_new);
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("Diameter");
                        ui.add(
                            egui::TextEdit::singleline(&mut op_new.diameter_text)
                                .desired_width(50.0),
                        );
                        ui.label("mm");
                        crate::expr::evaluation_hint(ui, &op_new.diameter_text, &var_map, "mm");
                    });
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut op_new.through, "Through all");
                        if !op_new.through {
                            ui.label("Depth");
                            ui.add(
                                egui::TextEdit::singleline(&mut op_new.depth_text)
                                    .desired_width(50.0),
                            );
                            ui.label("mm");
                            crate::expr::evaluation_hint(ui, &op_new.depth_text, &var_map, "mm");
                        }
                    });
                    ui.add_enabled_ui(!op_new.through, |ui| {
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut op_new.use_drill_point, "Modeled drill point");
                            if op_new.use_drill_point {
                                ui.label("Included angle");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.tip_angle_text)
                                        .desired_width(42.0),
                                );
                                ui.label("Â°");
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.tip_angle_text,
                                    &var_map,
                                    "Â°",
                                );
                            }
                        });
                    });
                    ui.add_space(4.0);
                    if !op_new.use_standard {
                        ui.horizontal(|ui| {
                            for (k, label) in [
                                (HoleKindChoice::Simple, "Simple"),
                                (HoleKindChoice::Counterbore, "Counterbore"),
                                (HoleKindChoice::Countersink, "Countersink"),
                            ] {
                                if ui.selectable_label(op_new.kind == k, label).clicked() {
                                    op_new.kind = k;
                                }
                            }
                        });
                    }
                    match op_new.kind {
                        HoleKindChoice::Simple => {}
                        HoleKindChoice::Counterbore => {
                            ui.horizontal(|ui| {
                                ui.label("C'bore Ø");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.head_diameter_text)
                                        .desired_width(45.0),
                                );
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.head_diameter_text,
                                    &var_map,
                                    "mm",
                                );
                                ui.label("depth");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.head_depth_text)
                                        .desired_width(40.0),
                                );
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.head_depth_text,
                                    &var_map,
                                    "mm",
                                );
                            });
                        }
                        HoleKindChoice::Countersink => {
                            ui.horizontal(|ui| {
                                ui.label("C'sink Ø");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.head_diameter_text)
                                        .desired_width(45.0),
                                );
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.head_diameter_text,
                                    &var_map,
                                    "mm",
                                );
                                ui.label("angle");
                                ui.add(
                                    egui::TextEdit::singleline(&mut op_new.head_angle_text)
                                        .desired_width(40.0),
                                );
                                ui.label("°");
                                crate::expr::evaluation_hint(
                                    ui,
                                    &op_new.head_angle_text,
                                    &var_map,
                                    "°",
                                );
                            });
                        }
                    }
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked() {
                            commit = true;
                        }
                        if ui.button("Cancel").clicked() {
                            cancel = true;
                        }
                    });
                });
            });

        if cancel {
            self.hole_op = None;
            self.status_msg = "Hole cancelled.".to_string();
            return;
        }
        self.hole_op = Some(op_new.clone());
        if commit {
            self.commit_hole_op(op_new);
        }
    }

    fn commit_hole_op(&mut self, op: HoleOp) {
        let Some(diameter) = self.eval_dim(&op.diameter_text) else {
            self.status_msg = "Hole diameter must be a number or valid expression.".to_string();
            return;
        };
        if diameter <= 0.0 {
            self.status_msg = "Hole diameter must be positive.".to_string();
            return;
        }
        let depth = if op.through {
            None
        } else {
            let d = self.eval_dim(&op.depth_text).unwrap_or(0.0);
            if d <= 0.0 {
                self.status_msg = "Hole depth must be positive (or use Through all).".to_string();
                return;
            }
            Some(d)
        };
        let kind = match op.kind {
            HoleKindChoice::Simple => HoleKind::Simple,
            HoleKindChoice::Counterbore => {
                let d = self.eval_dim(&op.head_diameter_text).unwrap_or(0.0);
                let dep = self.eval_dim(&op.head_depth_text).unwrap_or(0.0);
                if d <= diameter || dep <= 0.0 {
                    self.status_msg =
                        "Counterbore needs a diameter larger than the bore and a positive depth."
                            .to_string();
                    return;
                }
                HoleKind::Counterbore {
                    diameter: d,
                    depth: dep,
                }
            }
            HoleKindChoice::Countersink => {
                let d = self.eval_dim(&op.head_diameter_text).unwrap_or(0.0);
                let a = self.eval_dim(&op.head_angle_text).unwrap_or(0.0);
                if d <= diameter || !(a > 0.0 && a < 180.0) {
                    self.status_msg = "Countersink needs a diameter larger than the bore and an \
                                       angle in (0, 180)."
                        .to_string();
                    return;
                }
                HoleKind::Countersink {
                    diameter: d,
                    angle_deg: a,
                }
            }
        };
        let drill_point_angle_deg = if depth.is_some() && op.use_drill_point {
            let angle = self.eval_dim(&op.tip_angle_text).unwrap_or(0.0);
            if !(angle > 0.0 && angle < 180.0) {
                self.status_msg = "Drill-point angle must be in (0, 180).".to_string();
                return;
            }
            Some(angle)
        } else {
            None
        };
        let standard = if op.use_standard {
            let Some(preset) = op.selected_standard().copied() else {
                self.status_msg = "The selected standard has no available size.".to_string();
                return;
            };
            let (head_diameter, head_depth, head_angle) = match &kind {
                HoleKind::Simple => (None, None, None),
                HoleKind::Counterbore { diameter, depth } => (Some(*diameter), Some(*depth), None),
                HoleKind::Countersink {
                    diameter,
                    angle_deg,
                } => (Some(*diameter), None, Some(*angle_deg)),
            };
            Some(preset.reference_with_resolved(diameter, head_diameter, head_depth, head_angle))
        } else {
            None
        };
        let manufacturing =
            if let Some(preset) = op.selected_standard().copied().filter(|_| op.use_standard) {
                Some(preset.manufacturing_metadata(drill_point_angle_deg))
            } else if drill_point_angle_deg.is_some() {
                Some(zerocad_core::HoleManufacturingMetadata {
                    application: HoleApplication::Clearance,
                    drill_point_angle_deg,
                    cosmetic_thread: false,
                    thread_designation: None,
                    thread_class: None,
                    tap_pitch_mm: None,
                })
            } else {
                None
            };
        self.push_undo();
        let n = self.next_id();
        let id = format!("hole_{n}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Hole {n}"),
            feature: FeatureType::Hole {
                target: op.target.clone(),
                position: op.position,
                direction: op.direction,
                diameter,
                diameter_expr: zerocad_core::expr::preserves_source(&op.diameter_text)
                    .then(|| op.diameter_text.trim().to_string()),
                depth,
                kind,
                standard,
                manufacturing,
            },
        });
        self.document.add_dependency(&op.target, &id);
        self.selected_body.clear();
        self.selected_node_id = Some(id);
        self.hole_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Hole created.".to_string();
    }
}
