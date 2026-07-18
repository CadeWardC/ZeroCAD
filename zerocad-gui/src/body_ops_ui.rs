//! Phase 3.5 body-operation dialogs: planar Split and uniform Scale.

use crate::*;
use zerocad_core::parametric::{body_bounds_center, FaceRef};
use zerocad_core::PlaneBase;

#[derive(Debug, Clone)]
pub(crate) struct SplitBodyOp {
    pub(crate) target: String,
    pub(crate) plane: PlaneBase,
    pub(crate) face: Option<FaceRef>,
    pub(crate) picking_face: bool,
}

#[derive(Debug, Clone)]
pub(crate) struct ScaleBodyOp {
    pub(crate) source: String,
    pub(crate) factor_text: String,
    pub(crate) factor: f32,
    pub(crate) center: [f32; 3],
    pub(crate) initial_center: [f32; 3],
}

impl ZeroCadApp {
    pub(crate) fn begin_split_body(&mut self, target: String) {
        if self.split_body_op.is_some() || !self.body_meshes.iter().any(|(id, _)| id == &target) {
            return;
        }
        self.split_body_op = Some(SplitBodyOp {
            target,
            plane: PlaneBase::XY,
            face: None,
            picking_face: false,
        });
        self.status_msg =
            "Split Body: choose an origin/datum plane or pick a planar face.".to_string();
    }

    pub(crate) fn pick_split_face(&mut self, body: String, face_id: u32) {
        let Some(op) = self.split_body_op.as_ref() else {
            return;
        };
        if !op.picking_face {
            return;
        }
        if !self.face_is_planar(&body, face_id) {
            self.status_msg = "Split Body only accepts planar faces in Phase 3.5.".to_string();
            return;
        }
        let Some(face) = self.face_ref(&body, face_id) else {
            self.status_msg = "That face could not be resolved.".to_string();
            return;
        };
        if let Some(op) = self.split_body_op.as_mut() {
            op.face = Some(face);
            op.picking_face = false;
        }
        self.status_msg = "Planar split face captured.".to_string();
    }

    pub(crate) fn show_split_body_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut next) = self.split_body_op.clone() else {
            return;
        };
        let datums = self
            .document
            .graph
            .node_indices()
            .filter_map(|index| {
                let node = &self.document.graph[index];
                matches!(node.feature, FeatureType::DatumPlane { .. })
                    .then(|| (node.id.clone(), node.name.clone()))
            })
            .collect::<Vec<_>>();
        let mut commit = false;
        let mut cancel = false;
        egui::Window::new("Split Body")
            .id(egui::Id::new("split_body_dialog"))
            .collapsible(false)
            .resizable(false)
            .default_pos(egui::pos2(24.0, 90.0))
            .show(ctx, |ui| {
                ui.set_min_width(320.0);
                ui.label(format!("Body: {}", next.target));
                ui.add_space(4.0);
                ui.label("Splitting reference");
                ui.horizontal(|ui| {
                    for (label, plane) in [
                        ("XY", PlaneBase::XY),
                        ("XZ", PlaneBase::XZ),
                        ("YZ", PlaneBase::YZ),
                    ] {
                        if ui
                            .selectable_label(next.face.is_none() && next.plane == plane, label)
                            .clicked()
                        {
                            next.plane = plane;
                            next.face = None;
                            next.picking_face = false;
                        }
                    }
                });
                if !datums.is_empty() {
                    egui::ComboBox::from_label("Datum plane")
                        .selected_text(match &next.plane {
                            PlaneBase::Datum(id) if next.face.is_none() => datums
                                .iter()
                                .find(|(candidate, _)| candidate == id)
                                .map(|(_, name)| name.as_str())
                                .unwrap_or("Missing datum"),
                            _ => "None",
                        })
                        .show_ui(ui, |ui| {
                            for (id, name) in &datums {
                                if ui
                                    .selectable_label(
                                        next.face.is_none()
                                            && next.plane == PlaneBase::Datum(id.clone()),
                                        name,
                                    )
                                    .clicked()
                                {
                                    next.plane = PlaneBase::Datum(id.clone());
                                    next.face = None;
                                    next.picking_face = false;
                                }
                            }
                        });
                }
                ui.separator();
                if let Some(face) = &next.face {
                    ui.label(format!(
                        "Planar face at ({:.2}, {:.2}, {:.2})",
                        face.centroid[0], face.centroid[1], face.centroid[2]
                    ));
                }
                if ui
                    .button(if next.picking_face {
                        "Click a planar face in the viewport…"
                    } else {
                        "Pick planar face"
                    })
                    .clicked()
                {
                    next.picking_face = true;
                }
                ui.label(
                    egui::RichText::new("Both positive-volume sides become independent bodies.")
                        .size(11.0)
                        .color(self.pal().text_muted),
                );
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        commit = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if cancel {
            self.split_body_op = None;
            self.status_msg = "Split Body cancelled.".to_string();
        } else if commit {
            self.commit_split_body(next);
        } else {
            self.split_body_op = Some(next);
        }
    }

    fn commit_split_body(&mut self, op: SplitBodyOp) {
        self.push_undo();
        let n = self.next_id();
        let id = format!("split_body_{n}");
        let name = self.next_body_split_name();
        let datum_dependency = match &op.plane {
            PlaneBase::Datum(id) if op.face.is_none() => Some(id.clone()),
            _ => None,
        };
        let face_dependency = op.face.as_ref().and_then(|face| {
            let topology = face.topology.as_ref()?;
            topology
                .body_id
                .as_deref()
                .and_then(|body| self.document.body_producer_feature_id(body))
                .or(topology.producer_feature_id.as_deref())
                .map(str::to_owned)
        });
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature: FeatureType::BodySplit {
                target: op.target.clone(),
                plane: op.plane,
                face: op.face,
            },
        });
        self.document.add_dependency(&op.target, &id);
        if let Some(datum) = datum_dependency {
            self.document.add_dependency(&datum, &id);
        }
        if let Some(reference) = face_dependency.filter(|reference| reference != &op.target) {
            self.document.add_dependency(&reference, &id);
        }
        self.split_body_op = None;
        self.selected_body.clear();
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = "Body split into two independent outputs.".to_string();
    }

    pub(crate) fn begin_scale_body(&mut self, source: String) {
        if self.scale_body_op.is_some() {
            return;
        }
        let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| id == &source) else {
            self.status_msg = "The selected body no longer exists.".to_string();
            return;
        };
        let Some(center) = body_bounds_center(mesh) else {
            self.status_msg = "The selected body has no display bounds.".to_string();
            return;
        };
        self.scale_body_op = Some(ScaleBodyOp {
            source,
            factor_text: "1.0".to_string(),
            factor: 1.0,
            center,
            initial_center: center,
        });
        self.refresh_scale_preview();
        self.status_msg = "Scale Body: enter a positive uniform factor and pivot.".to_string();
    }

    /// Update the immediate display ghost by scaling the already-tessellated
    /// mesh. The committed feature still runs through the normal evaluator and
    /// strict kernel transform; this preview deliberately avoids a worker round
    /// trip because uniform mesh scaling is exact for displayed positions.
    fn refresh_scale_preview(&mut self) {
        let Some(op) = self.scale_body_op.as_ref() else {
            self.move_preview_bodies = None;
            return;
        };
        if !op.factor.is_finite() || op.factor <= 0.0 {
            self.move_preview_bodies = None;
            return;
        }
        let mut bodies = (*self.body_meshes).clone();
        if let Some((_, mesh)) = bodies.iter_mut().find(|(id, _)| id == &op.source) {
            scale_mesh_about(mesh, op.factor, op.center);
        }
        self.move_preview_bodies = Some(std::sync::Arc::new(bodies));
    }

    pub(crate) fn show_scale_body_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut next) = self.scale_body_op.clone() else {
            return;
        };
        let variables = self.document.variable_map();
        let mut changed = false;
        let mut commit = false;
        let mut cancel = false;
        egui::Window::new("Scale Body")
            .id(egui::Id::new("scale_body_dialog"))
            .collapsible(false)
            .resizable(false)
            .default_pos(egui::pos2(24.0, 90.0))
            .show(ctx, |ui| {
                ui.set_min_width(300.0);
                ui.label(format!("Body: {}", next.source));
                ui.horizontal(|ui| {
                    ui.label("Factor");
                    changed |= ui
                        .text_edit_singleline(&mut next.factor_text)
                        .on_hover_text("A positive number or parameter expression")
                        .changed();
                });
                crate::expr::evaluation_hint(ui, &next.factor_text, &variables, "×");
                if changed {
                    if let Ok(value) = crate::expr::eval(&next.factor_text, &variables) {
                        next.factor = value as f32;
                    }
                }
                ui.separator();
                ui.label("Pivot (world coordinates)");
                for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        changed |= ui
                            .add(egui::DragValue::new(&mut next.center[axis]).speed(0.25))
                            .changed();
                    });
                }
                if ui.button("Reset to current body center").clicked() {
                    next.center = next.initial_center;
                    changed = true;
                }
                let valid = next.factor.is_finite()
                    && next.factor > 1.0e-6
                    && (next.factor - 1.0).abs() > 1.0e-6
                    && next.center.iter().all(|value| value.is_finite());
                if !valid {
                    ui.label(
                        egui::RichText::new("Use a finite positive factor other than 1.")
                            .size(11.0)
                            .color(egui::Color32::from_rgb(180, 70, 70)),
                    );
                }
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.add_enabled(valid, egui::Button::new("OK")).clicked() {
                        commit = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if cancel {
            self.scale_body_op = None;
            self.move_preview_bodies = None;
            self.status_msg = "Scale Body cancelled.".to_string();
        } else if commit {
            self.commit_scale_body(next);
        } else {
            self.scale_body_op = Some(next);
            if changed {
                self.refresh_scale_preview();
            }
        }
    }

    fn commit_scale_body(&mut self, op: ScaleBodyOp) {
        if !op.factor.is_finite() || op.factor <= 1.0e-6 || (op.factor - 1.0).abs() <= 1.0e-6 {
            return;
        }
        self.push_undo();
        let n = self.next_id();
        let id = format!("scale_body_{n}");
        let name = self.next_body_scale_name();
        let expression =
            zerocad_core::expr::preserves_source(&op.factor_text).then(|| op.factor_text.clone());
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature: FeatureType::BodyScale {
                source: op.source.clone(),
                factor: op.factor,
                factor_expr: expression,
                center: op.center,
            },
        });
        self.document.add_dependency(&op.source, &id);
        self.scale_body_op = None;
        self.move_preview_bodies = None;
        self.selected_body.clear();
        self.selected_body.insert((id.clone(), BodyPick::Whole));
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = "Body scaled.".to_string();
    }
}

fn scale_mesh_about(mesh: &mut MockMesh, factor: f32, center: [f32; 3]) {
    let scale_point = |point: &mut [f32]| {
        for axis in 0..3 {
            point[axis] = center[axis] + (point[axis] - center[axis]) * factor;
        }
    };
    for vertex in mesh.vertices.chunks_exact_mut(6) {
        scale_point(&mut vertex[..3]);
    }
    for point in mesh.edge_vertices.chunks_exact_mut(3) {
        scale_point(point);
    }
    for edge in &mut mesh.edge_refs {
        scale_point(&mut edge.p0);
        scale_point(&mut edge.p1);
    }
    for face in &mut mesh.face_refs {
        scale_point(&mut face.centroid);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_preview_uses_persisted_pivot() {
        let mut mesh = MockMesh::make_box(2.0, 2.0, 2.0);
        scale_mesh_about(&mut mesh, 2.0, [1.0, 1.0, 1.0]);
        assert_eq!(body_bounds_center(&mesh), Some([1.0, 1.0, 1.0]));
    }

    #[test]
    fn split_commit_registers_both_outputs_and_survives_undo_redo() {
        let mut app = ZeroCadApp::new();
        app.document.add_feature(FeatureNode {
            id: "box_1".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        app.document.add_feature(FeatureNode {
            id: "datum_2".into(),
            name: "Mid plane".into(),
            feature: FeatureType::DatumPlane {
                def: zerocad_core::DatumPlaneDef::Offset {
                    base: PlaneBase::XY,
                    distance: 5.0,
                    distance_expr: None,
                },
            },
        });
        app.commit_split_body(SplitBodyOp {
            target: "box_1".into(),
            plane: PlaneBase::Datum("datum_2".into()),
            face: None,
            picking_face: false,
        });
        let split_id = app
            .document
            .graph
            .node_weights()
            .find(|node| matches!(node.feature, FeatureType::BodySplit { .. }))
            .map(|node| node.id.clone())
            .expect("BodySplit feature");
        let second = zerocad_core::parametric::body_output_id(&split_id, 1);
        assert_eq!(
            app.document.body_producer_feature_id(&second),
            Some(split_id.as_str())
        );
        assert_eq!(app.selected_node_id.as_deref(), Some(split_id.as_str()));

        app.undo();
        assert!(!app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodySplit { .. })));
        app.redo();
        assert!(app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodySplit { .. })));
    }

    #[test]
    fn split_face_reference_records_its_body_dependency() {
        let mut app = ZeroCadApp::new();
        for id in ["target", "reference"] {
            app.document.add_feature(FeatureNode {
                id: id.into(),
                name: id.into(),
                feature: FeatureType::Box {
                    w: 10.0,
                    h: 10.0,
                    d: 10.0,
                },
            });
        }
        app.commit_split_body(SplitBodyOp {
            target: "target".into(),
            plane: PlaneBase::XY,
            face: Some(FaceRef {
                centroid: [5.0, 5.0, 5.0],
                normal: [1.0, 0.0, 0.0],
                topology: Some(zerocad_core::parametric::TopologyFaceRef {
                    body_id: Some("reference".into()),
                    producer_feature_id: Some("reference".into()),
                    ..zerocad_core::parametric::TopologyFaceRef::default()
                }),
            }),
            picking_face: false,
        });
        let split = app
            .document
            .graph
            .node_indices()
            .find(|index| {
                matches!(
                    app.document.graph[*index].feature,
                    FeatureType::BodySplit { .. }
                )
            })
            .expect("BodySplit feature");
        let find_feature = |id: &str| {
            app.document
                .graph
                .node_indices()
                .find(|index| app.document.graph[*index].id == id)
                .expect("feature node")
        };
        let target = find_feature("target");
        let reference = find_feature("reference");
        assert!(app.document.graph.find_edge(target, split).is_some());
        assert!(app.document.graph.find_edge(reference, split).is_some());
        assert!(app.document.graph[split]
            .inputs
            .iter()
            .any(|input| input.role == "plane_face"));
    }

    #[test]
    fn scale_commit_persists_expression_and_pivot_and_survives_undo_redo() {
        let mut app = ZeroCadApp::new();
        app.document.add_feature(FeatureNode {
            id: "box_1".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 2.0,
                h: 4.0,
                d: 6.0,
            },
        });
        app.set_body_meshes(vec![("box_1".into(), MockMesh::make_box(2.0, 4.0, 6.0))]);
        app.commit_scale_body(ScaleBodyOp {
            source: "box_1".into(),
            factor_text: "scale_factor".into(),
            factor: 2.0,
            center: [1.0, 2.0, 3.0],
            initial_center: [1.0, 2.0, 3.0],
        });
        let scale = app
            .document
            .graph
            .node_weights()
            .find_map(|node| match &node.feature {
                FeatureType::BodyScale {
                    factor,
                    factor_expr,
                    center,
                    ..
                } => Some((*factor, factor_expr.clone(), *center)),
                _ => None,
            })
            .expect("BodyScale feature");
        assert_eq!(scale, (2.0, Some("scale_factor".into()), [1.0, 2.0, 3.0]));

        app.undo();
        assert!(!app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodyScale { .. })));
        app.redo();
        assert!(app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::BodyScale { .. })));
    }
}
