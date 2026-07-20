//! Standalone Draft creation UI.

use crate::*;
use zerocad_core::parametric::DraftNeutral;

#[derive(Debug, Clone)]
struct DraftNeutralChoice {
    value: DraftNeutral,
    label: String,
}

#[derive(Debug, Clone)]
pub(crate) struct DraftOp {
    target: String,
    faces: Vec<FaceRef>,
    neutral_choices: Vec<DraftNeutralChoice>,
    neutral_index: usize,
    angle_text: String,
    flip_pull: bool,
}

impl ZeroCadApp {
    /// Selected planar body faces from exactly one body.
    pub(crate) fn draft_candidate(&self) -> Option<(String, Vec<FaceRef>)> {
        let mut selected: Vec<_> = self
            .selected_body
            .iter()
            .filter_map(|(body, pick)| match pick {
                BodyPick::Face(face) => Some((body.clone(), *face)),
                _ => None,
            })
            .collect();
        selected.sort();
        let target = selected.first()?.0.clone();
        if selected.iter().any(|(body, _)| body != &target) {
            return None;
        }
        let faces: Vec<_> = selected
            .into_iter()
            .filter_map(|(body, face)| self.face_ref(&body, face))
            .collect();
        (!faces.is_empty()).then_some((target, faces))
    }

    pub(crate) fn begin_draft(&mut self, target: String, faces: Vec<FaceRef>) {
        if self.draft_op.is_some() {
            return;
        }
        let selected_names: std::collections::HashSet<_> = faces
            .iter()
            .filter_map(|face| {
                face.topology
                    .as_ref()
                    .and_then(|topology| topology.face_id.clone())
            })
            .collect();
        let side_normal = faces[0].normal;
        let mut neutral_choices = Vec::new();
        if let Some((_, mesh)) = self.body_meshes.iter().find(|(body, _)| body == &target) {
            let mut candidates: Vec<_> = mesh.face_refs.iter().collect();
            candidates.sort_by_key(|face| face.face_id);
            for face in candidates {
                let name = face
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.face_id.as_deref());
                if name.is_some_and(|name| selected_names.contains(name)) {
                    continue;
                }
                let surface_kind = face
                    .topology
                    .as_ref()
                    .and_then(|topology| topology.surface_kind.as_deref())
                    .unwrap_or_default();
                if !surface_kind.is_empty() && !surface_kind.contains("plane") {
                    continue;
                }
                let dot = face.normal[0] * side_normal[0]
                    + face.normal[1] * side_normal[1]
                    + face.normal[2] * side_normal[2];
                if dot.abs() > 0.2 {
                    continue;
                }
                if let Some(reference) = self.face_ref(&target, face.face_id) {
                    neutral_choices.push(DraftNeutralChoice {
                        value: DraftNeutral::Face(reference),
                        label: name
                            .map(ToString::to_string)
                            .unwrap_or_else(|| format!("Face {}", face.face_id)),
                    });
                }
            }
        }
        for (id, name, _) in self.resolved_datum_planes() {
            neutral_choices.push(DraftNeutralChoice {
                value: DraftNeutral::Datum(id),
                label: format!("Datum: {name}"),
            });
        }
        if neutral_choices.is_empty() {
            self.status_msg =
                "Draft needs an unselected prism end face or a resolved datum plane.".to_string();
            return;
        }
        self.draft_op = Some(DraftOp {
            target,
            faces,
            neutral_choices,
            neutral_index: 0,
            angle_text: "3".to_string(),
            flip_pull: false,
        });
        self.status_msg = "Draft: choose the neutral reference and angle, then OK.".to_string();
    }

    pub(crate) fn show_draft_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut operation) = self.draft_op.clone() else {
            return;
        };
        let variables = self.visible_variable_map();
        let mut commit = false;
        let mut cancel = false;
        egui::Area::new(egui::Id::new("draft_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(270.0);
                    ui.label(
                        egui::RichText::new("Draft")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.label(format!(
                        "{} selected side face(s) on {}",
                        operation.faces.len(),
                        operation.target
                    ));
                    ui.add_space(5.0);
                    ui.horizontal(|ui| {
                        ui.label("Neutral");
                        let label = &operation.neutral_choices[operation.neutral_index].label;
                        egui::ComboBox::from_id_salt("draft_neutral")
                            .selected_text(label)
                            .show_ui(ui, |ui| {
                                for (index, choice) in operation.neutral_choices.iter().enumerate()
                                {
                                    ui.selectable_value(
                                        &mut operation.neutral_index,
                                        index,
                                        &choice.label,
                                    );
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Angle");
                        ui.add(
                            egui::TextEdit::singleline(&mut operation.angle_text)
                                .desired_width(75.0),
                        );
                        ui.label("°");
                        crate::expr::evaluation_hint(ui, &operation.angle_text, &variables, "°");
                    });
                    ui.checkbox(&mut operation.flip_pull, "Reverse pull direction");
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
            self.draft_op = None;
            self.status_msg = "Draft cancelled.".to_string();
            return;
        }
        self.draft_op = Some(operation.clone());
        if commit {
            self.commit_draft(operation);
        }
    }

    fn commit_draft(&mut self, operation: DraftOp) {
        let Some(angle) = self.eval_dim(&operation.angle_text) else {
            self.status_msg = "Draft angle must be a number or valid expression.".to_string();
            return;
        };
        if !angle.is_finite() || angle.abs() >= 89.0 {
            self.status_msg = "Draft angle must be strictly between -89° and 89°.".to_string();
            return;
        }
        let neutral = operation.neutral_choices[operation.neutral_index]
            .value
            .clone();
        self.push_undo();
        let number = self.next_id();
        let id = format!("draft_{number}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Draft {number}"),
            feature: FeatureType::Draft {
                target: operation.target.clone(),
                faces: operation.faces,
                neutral: neutral.clone(),
                angle_deg: angle,
                angle_expr: zerocad_core::expr::preserves_source(&operation.angle_text)
                    .then(|| operation.angle_text.trim().to_string()),
                flip_pull: operation.flip_pull,
            },
        });
        self.document.add_dependency(&operation.target, &id);
        if let DraftNeutral::Datum(datum) = neutral {
            self.document.add_dependency(&datum, &id);
        }
        self.selected_node_id = Some(id);
        self.selected_body.clear();
        self.draft_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Draft created.".to_string();
    }
}
