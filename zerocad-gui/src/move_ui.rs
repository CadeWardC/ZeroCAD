//! Body copy/paste and translate/face-align UI.

use crate::*;
use zerocad_core::parametric::FaceRef;

#[derive(Debug, Clone)]
pub(crate) struct BodyClipboard {
    pub(crate) source: String,
    pub(crate) paste_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MoveMethod {
    Translate,
    AlignFaces,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FacePlacement {
    Touching,
    Coplanar,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MoveFacePick {
    Source,
    Target,
}

#[derive(Debug, Clone)]
pub(crate) struct MoveOp {
    pub(crate) source: String,
    pub(crate) translation: [f32; 3],
    pub(crate) method: MoveMethod,
    pub(crate) placement: FacePlacement,
    pub(crate) source_face: Option<FaceRef>,
    pub(crate) target_face: Option<FaceRef>,
    pub(crate) picking: Option<MoveFacePick>,
    pub(crate) drag_axis: Option<usize>,
    pub(crate) drag_start_pointer: Option<egui::Pos2>,
    pub(crate) drag_start_translation: [f32; 3],
}

fn translate_mesh(mesh: &mut MockMesh, translation: [f32; 3]) {
    for vertex in mesh.vertices.chunks_exact_mut(6) {
        vertex[0] += translation[0];
        vertex[1] += translation[1];
        vertex[2] += translation[2];
    }
    for point in mesh.edge_vertices.chunks_exact_mut(3) {
        point[0] += translation[0];
        point[1] += translation[1];
        point[2] += translation[2];
    }
    for edge in &mut mesh.edge_refs {
        for i in 0..3 {
            edge.p0[i] += translation[i];
            edge.p1[i] += translation[i];
        }
    }
    for face in &mut mesh.face_refs {
        for (coord, delta) in face.centroid.iter_mut().zip(translation) {
            *coord += delta;
        }
    }
}

fn mesh_bounds(mesh: &MockMesh) -> Option<([f32; 3], [f32; 3])> {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    let mut any = false;
    for vertex in mesh.vertices.chunks_exact(6) {
        any = true;
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    any.then_some((min, max))
}

impl ZeroCadApp {
    pub(crate) fn selected_whole_body(&self) -> Option<String> {
        (self.selected_body.len() == 1)
            .then(|| self.selected_body.iter().next())
            .flatten()
            .and_then(|(id, pick)| matches!(pick, BodyPick::Whole).then(|| id.clone()))
    }

    pub(crate) fn copy_selected_body(&mut self) {
        let Some(source) = self.selected_whole_body() else {
            self.status_msg = "Fully select one body before copying it.".to_string();
            return;
        };
        self.body_clipboard = Some(BodyClipboard {
            source: source.clone(),
            paste_count: 0,
        });
        self.status_msg = format!("Copied {source}. Press Ctrl+V to place a copy beside it.");
    }

    pub(crate) fn paste_copied_body(&mut self) {
        let Some(mut clipboard) = self.body_clipboard.clone() else {
            self.status_msg = "Nothing has been copied yet.".to_string();
            return;
        };
        let Some((_, mesh)) = self
            .body_meshes
            .iter()
            .find(|(id, _)| id == &clipboard.source)
        else {
            self.status_msg = "The copied source body no longer exists.".to_string();
            self.body_clipboard = None;
            return;
        };
        let Some((min, max)) = mesh_bounds(mesh) else {
            self.status_msg = "The copied body has no display geometry.".to_string();
            return;
        };
        clipboard.paste_count += 1;
        let width = (max[0] - min[0]).abs().max(1.0);
        let gap = (width * 0.1).max(2.0);
        let translation = [(width + gap) * clipboard.paste_count as f32, 0.0, 0.0];

        self.push_undo();
        let n = self.next_id();
        let id = format!("copy_{n}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Body Copy {n}"),
            feature: FeatureType::BodyTransform {
                source: clipboard.source.clone(),
                translation,
                copy: true,
            },
        });
        self.document.add_dependency(&clipboard.source, &id);
        self.body_clipboard = Some(clipboard);
        self.selected_body.clear();
        self.selected_body.insert((id.clone(), BodyPick::Whole));
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = "Body copy created beside the source.".to_string();
    }

    pub(crate) fn begin_move_body(&mut self, source: String) {
        if self.move_op.is_some() {
            return;
        }
        if !self.body_meshes.iter().any(|(id, _)| id == &source) {
            self.status_msg = "The selected body no longer exists.".to_string();
            return;
        }
        self.move_op = Some(MoveOp {
            source,
            translation: [0.0; 3],
            method: MoveMethod::Translate,
            placement: FacePlacement::Touching,
            source_face: None,
            target_face: None,
            picking: None,
            drag_axis: None,
            drag_start_pointer: None,
            drag_start_translation: [0.0; 3],
        });
        self.refresh_move_preview();
        self.status_msg =
            "Move: drag an axis arrow, enter X/Y/Z, or switch to Align faces.".to_string();
    }

    pub(crate) fn cancel_move_body(&mut self) {
        self.move_op = None;
        self.move_preview_bodies = None;
        self.status_msg = "Move cancelled.".to_string();
    }

    pub(crate) fn commit_move_body(&mut self) {
        let Some(op) = self.move_op.take() else {
            return;
        };
        if op.translation.iter().all(|value| value.abs() < 1.0e-6) {
            self.move_preview_bodies = None;
            self.status_msg = "Move distance is zero; nothing changed.".to_string();
            return;
        }
        self.push_undo();
        let n = self.next_id();
        let id = format!("move_{n}");
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Move Body {n}"),
            feature: FeatureType::BodyTransform {
                source: op.source.clone(),
                translation: op.translation,
                copy: false,
            },
        });
        self.document.add_dependency(&op.source, &id);
        self.selected_body.clear();
        self.selected_body.insert((id.clone(), BodyPick::Whole));
        self.selected_node_id = Some(id);
        // Keep the cheap preview visible until the background authoritative
        // evaluation replaces it.
        self.reevaluate_geometry();
        self.status_msg = "Body moved.".to_string();
    }

    pub(crate) fn refresh_move_preview(&mut self) {
        let Some(op) = self.move_op.as_ref() else {
            self.move_preview_bodies = None;
            return;
        };
        let mut bodies = (*self.body_meshes).clone();
        if let Some((_, mesh)) = bodies.iter_mut().find(|(id, _)| id == &op.source) {
            translate_mesh(mesh, op.translation);
        }
        self.move_preview_bodies = Some(std::sync::Arc::new(bodies));
    }

    pub(crate) fn pick_move_face(&mut self, body: String, face_id: u32) {
        let Some(stage) = self.move_op.as_ref().and_then(|op| op.picking) else {
            return;
        };
        let source = self.move_op.as_ref().unwrap().source.clone();
        if stage == MoveFacePick::Source && body != source {
            self.status_msg = "Pick a face on the body being moved first.".to_string();
            return;
        }
        if stage == MoveFacePick::Target && body == source {
            self.status_msg = "Pick the target face on a different body.".to_string();
            return;
        }
        let Some(face) = self.face_ref(&body, face_id) else {
            self.status_msg = "That face could not be resolved.".to_string();
            return;
        };
        if let Some(op) = self.move_op.as_mut() {
            match stage {
                MoveFacePick::Source => op.source_face = Some(face),
                MoveFacePick::Target => op.target_face = Some(face),
            }
            op.picking = None;
        }
        self.update_face_alignment();
    }

    fn update_face_alignment(&mut self) {
        let Some(op) = self.move_op.as_mut() else {
            return;
        };
        let (Some(source), Some(target)) = (&op.source_face, &op.target_face) else {
            return;
        };
        let sn = Vec3::new(source.normal[0], source.normal[1], source.normal[2]).normalize();
        let tn = Vec3::new(target.normal[0], target.normal[1], target.normal[2]).normalize();
        if sn.dot(tn).abs() < 0.98 {
            self.status_msg =
                "Those faces are not parallel. Translation can align parallel faces; rotation is not applied."
                    .to_string();
            return;
        }
        let source_center = Vec3::new(source.centroid[0], source.centroid[1], source.centroid[2]);
        let target_center = Vec3::new(target.centroid[0], target.centroid[1], target.centroid[2]);
        let delta = match op.placement {
            FacePlacement::Touching => target_center.sub(source_center),
            FacePlacement::Coplanar => {
                let distance = target_center.sub(source_center).dot(tn);
                tn.mul(distance)
            }
        };
        op.translation = [delta.x, delta.y, delta.z];
        let placement = op.placement;
        self.refresh_move_preview();
        self.status_msg = match placement {
            FacePlacement::Touching => "Faces positioned touching, with their centers aligned.",
            FacePlacement::Coplanar => "Faces placed in line (coplanar), preserving side offset.",
        }
        .to_string();
    }

    pub(crate) fn show_move_dialog(&mut self, ctx: &egui::Context) {
        if self.move_op.is_none() {
            return;
        }
        let mut commit = false;
        let mut cancel = false;
        let mut changed = false;
        egui::Window::new("Move Body")
            .id(egui::Id::new("move_body_dialog"))
            .collapsible(false)
            .resizable(false)
            .default_pos(egui::pos2(24.0, 90.0))
            .show(ctx, |ui| {
                let op = self.move_op.as_mut().unwrap();
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut op.method, MoveMethod::Translate, "Translate");
                    ui.selectable_value(&mut op.method, MoveMethod::AlignFaces, "Align faces");
                });
                ui.separator();
                match op.method {
                    MoveMethod::Translate => {
                        ui.label("Drag the red/green/blue arrows, or enter an offset:");
                        for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(label);
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut op.translation[axis])
                                            .speed(0.25)
                                            .suffix(" mm"),
                                    )
                                    .changed()
                                {
                                    changed = true;
                                }
                            });
                        }
                    }
                    MoveMethod::AlignFaces => {
                        ui.horizontal(|ui| {
                            if ui
                                .selectable_label(
                                    op.placement == FacePlacement::Touching,
                                    "Touching",
                                )
                                .clicked()
                            {
                                op.placement = FacePlacement::Touching;
                                changed = true;
                            }
                            if ui
                                .selectable_label(
                                    op.placement == FacePlacement::Coplanar,
                                    "In line",
                                )
                                .clicked()
                            {
                                op.placement = FacePlacement::Coplanar;
                                changed = true;
                            }
                        });
                        ui.label(match op.placement {
                            FacePlacement::Touching => {
                                "Touching aligns the source-face center to the target-face center."
                            }
                            FacePlacement::Coplanar => {
                                "In line makes the faces coplanar without changing the side offset."
                            }
                        });
                        if ui.button("Pick moving-body face").clicked() {
                            op.picking = Some(MoveFacePick::Source);
                        }
                        if ui.button("Pick target-body face").clicked() {
                            op.picking = Some(MoveFacePick::Target);
                        }
                        let picked =
                            |face: &Option<FaceRef>| if face.is_some() { "✓" } else { "—" };
                        ui.label(format!(
                            "Moving face: {}    Target face: {}",
                            picked(&op.source_face),
                            picked(&op.target_face)
                        ));
                    }
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        commit = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if changed {
            if self
                .move_op
                .as_ref()
                .is_some_and(|op| op.method == MoveMethod::AlignFaces)
            {
                self.update_face_alignment();
            } else {
                self.refresh_move_preview();
            }
        }
        if commit {
            self.commit_move_body();
        } else if cancel {
            self.cancel_move_body();
        }
    }

    /// Draw and interact with the three-axis translation gizmo. `project`
    /// returns screen x/y for a world point.
    pub(crate) fn move_gizmo(
        &mut self,
        painter: &egui::Painter,
        response: &egui::Response,
        project: &dyn Fn([f32; 3]) -> egui::Pos2,
    ) {
        let Some(op) = self.move_op.as_ref() else {
            return;
        };
        if op.method != MoveMethod::Translate {
            return;
        }
        let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| id == &op.source) else {
            return;
        };
        let Some((min, max)) = mesh_bounds(mesh) else {
            return;
        };
        let center = [
            (min[0] + max[0]) * 0.5 + op.translation[0],
            (min[1] + max[1]) * 0.5 + op.translation[1],
            (min[2] + max[2]) * 0.5 + op.translation[2],
        ];
        let origin = project(center);
        let length = self.camera_zoom * 0.65;
        let colors = [
            egui::Color32::from_rgb(220, 55, 55),
            egui::Color32::from_rgb(40, 175, 75),
            egui::Color32::from_rgb(55, 105, 225),
        ];
        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut screen_axes = Vec::with_capacity(3);
        for axis in 0..3 {
            let tip_world = [
                center[0] + axes[axis][0] * length,
                center[1] + axes[axis][1] * length,
                center[2] + axes[axis][2] * length,
            ];
            let tip = project(tip_world);
            painter.line_segment([origin, tip], egui::Stroke::new(3.0, colors[axis]));
            painter.circle_filled(tip, 5.0, colors[axis]);
            painter.text(
                tip + egui::vec2(7.0, -7.0),
                egui::Align2::LEFT_BOTTOM,
                ["X", "Y", "Z"][axis],
                egui::FontId::proportional(12.0),
                colors[axis],
            );
            screen_axes.push((tip, tip - origin));
        }

        if response.drag_started_by(egui::PointerButton::Primary) {
            if let Some(pointer) = response.interact_pointer_pos() {
                let mut best: Option<(usize, f32)> = None;
                for (axis, (tip, _)) in screen_axes.iter().enumerate() {
                    let distance = crate::geom2d::dist_point_to_segment(pointer, origin, *tip);
                    if distance <= 10.0 && best.is_none_or(|(_, d)| distance < d) {
                        best = Some((axis, distance));
                    }
                }
                if let Some((axis, _)) = best {
                    if let Some(op) = self.move_op.as_mut() {
                        op.drag_axis = Some(axis);
                        op.drag_start_pointer = Some(pointer);
                        op.drag_start_translation = op.translation;
                    }
                }
            }
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            let drag = self.move_op.as_ref().and_then(|op| {
                Some((
                    op.drag_axis?,
                    op.drag_start_pointer?,
                    op.drag_start_translation,
                ))
            });
            if let (Some((axis, start_pointer, start_translation)), Some(pointer)) =
                (drag, response.interact_pointer_pos())
            {
                let screen_axis = screen_axes[axis].1;
                let pixels_per_world = (screen_axis.length() / length).max(1.0e-4);
                let delta =
                    (pointer - start_pointer).dot(screen_axis.normalized()) / pixels_per_world;
                if let Some(op) = self.move_op.as_mut() {
                    op.translation = start_translation;
                    op.translation[axis] += delta;
                }
                self.refresh_move_preview();
            }
        }
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            if let Some(op) = self.move_op.as_mut() {
                op.drag_axis = None;
                op.drag_start_pointer = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn face(centroid: [f32; 3]) -> FaceRef {
        FaceRef {
            centroid,
            normal: [0.0, 0.0, 1.0],
            topology: None,
        }
    }

    fn app_with_faces(placement: FacePlacement) -> ZeroCadApp {
        let mut app = ZeroCadApp::new();
        app.move_op = Some(MoveOp {
            source: "body_1".to_string(),
            translation: [0.0; 3],
            method: MoveMethod::AlignFaces,
            placement,
            source_face: Some(face([1.0, 2.0, 0.0])),
            target_face: Some(face([10.0, 20.0, 5.0])),
            picking: None,
            drag_axis: None,
            drag_start_pointer: None,
            drag_start_translation: [0.0; 3],
        });
        app
    }

    #[test]
    fn touching_alignment_matches_face_centers() {
        let mut app = app_with_faces(FacePlacement::Touching);
        app.update_face_alignment();
        assert_eq!(app.move_op.unwrap().translation, [9.0, 18.0, 5.0]);
    }

    #[test]
    fn in_line_alignment_moves_only_normal_to_face_plane() {
        let mut app = app_with_faces(FacePlacement::Coplanar);
        app.update_face_alignment();
        assert_eq!(app.move_op.unwrap().translation, [0.0, 0.0, 5.0]);
    }
}
