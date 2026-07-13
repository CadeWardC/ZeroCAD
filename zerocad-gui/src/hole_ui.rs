//! Hole tool UI: select a body face → set diameter/depth/head style → commit
//! a `FeatureType::Hole` node. v1 places the hole at the selected face's
//! centroid (drilling along the inward normal); the position is editable in
//! the Properties panel afterwards.

use crate::*;
use zerocad_core::HoleKind;

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
    pub(crate) kind: HoleKindChoice,
    pub(crate) head_diameter_text: String,
    pub(crate) head_depth_text: String,
    pub(crate) head_angle_text: String,
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
            diameter_text: "6".to_string(),
            through: true,
            depth_text: "10".to_string(),
            kind: HoleKindChoice::Simple,
            head_diameter_text: "11".to_string(),
            head_depth_text: "3".to_string(),
            head_angle_text: "90".to_string(),
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
                    ui.add_space(4.0);
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
        self.push_undo();
        let n = self.next_id();
        let id = format!("hole_{n}");
        self.graph.add_feature(FeatureNode {
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
            },
        });
        self.graph.add_dependency(&op.target, &id);
        self.selected_body.clear();
        self.selected_node_id = Some(id);
        self.hole_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Hole created.".to_string();
    }
}
