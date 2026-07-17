//! Compact Phase 5 direct-edit commands for one selected B-Rep face.

use crate::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DirectFaceCommand {
    PressPull,
    Move,
    Delete,
    Thicken,
}

impl ZeroCadApp {
    pub(crate) fn selected_direct_face(&self) -> Option<(String, FaceRef)> {
        let (target, face_id) = self.selected_faces.iter().next()?;
        (self.selected_faces.len() == 1).then(|| {
            u32::try_from(*face_id)
                .ok()
                .and_then(|face_id| self.face_ref(target, face_id))
                .map(|face| (target.clone(), face))
        })?
    }

    pub(crate) fn commit_direct_face_command(&mut self, command: DirectFaceCommand) {
        let Some((target, face)) = self.selected_direct_face() else {
            self.status_msg = "Select one B-Rep face for direct editing.".to_string();
            return;
        };
        if face
            .topology
            .as_ref()
            .and_then(|topology| topology.surface_kind.as_deref())
            == Some("mesh_triangle")
        {
            self.status_msg =
                "STL mesh faces cannot enter B-Rep direct-edit operations.".to_string();
            return;
        }
        let number = self.next_id();
        let (id, name, feature) = match command {
            DirectFaceCommand::PressPull => (
                format!("face_offset_{number}"),
                format!(
                    "PressPull_{}",
                    self.next_feature_index(|feature| matches!(
                        feature,
                        FeatureType::FaceOffset { .. }
                    ))
                ),
                FeatureType::FaceOffset {
                    target: target.clone(),
                    face,
                    distance: 5.0,
                    distance_expr: None,
                },
            ),
            DirectFaceCommand::Move => {
                let normal = face.normal;
                (
                    format!("face_move_{number}"),
                    format!(
                        "MoveFace_{}",
                        self.next_feature_index(|feature| matches!(
                            feature,
                            FeatureType::FaceMove { .. }
                        ))
                    ),
                    FeatureType::FaceMove {
                        target: target.clone(),
                        face,
                        translation: [normal[0] * 5.0, normal[1] * 5.0, normal[2] * 5.0],
                    },
                )
            }
            DirectFaceCommand::Delete => (
                format!("face_delete_{number}"),
                format!(
                    "DeleteFace_{}",
                    self.next_feature_index(|feature| matches!(
                        feature,
                        FeatureType::FaceDelete { .. }
                    ))
                ),
                FeatureType::FaceDelete {
                    target: target.clone(),
                    face,
                },
            ),
            DirectFaceCommand::Thicken => (
                format!("face_thicken_{number}"),
                format!(
                    "Thicken_{}",
                    self.next_feature_index(|feature| matches!(
                        feature,
                        FeatureType::FaceThicken { .. }
                    ))
                ),
                FeatureType::FaceThicken {
                    target: target.clone(),
                    face,
                    thickness: 2.0,
                    thickness_expr: None,
                    reverse: false,
                },
            ),
        };
        self.push_undo();
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name,
            feature,
        });
        let producer = self
            .document
            .body_producer_feature_id(&target)
            .unwrap_or(target.as_str())
            .to_string();
        self.document.add_dependency(&producer, &id);
        self.selected_faces.clear();
        self.selected_body.clear();
        self.selected_body.insert((id.clone(), BodyPick::Whole));
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = match command {
            DirectFaceCommand::PressPull => {
                "Press/Pull created. Edit its signed distance in Properties."
            }
            DirectFaceCommand::Move => {
                "Move Face created. Edit its normal translation in Properties."
            }
            DirectFaceCommand::Delete => "Delete Face created.",
            DirectFaceCommand::Thicken => {
                "Thicken Face created. Edit thickness and direction in Properties."
            }
        }
        .to_string();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn press_pull_command_records_face_provenance_and_dependency() {
        let mut app = ZeroCadApp::new();
        app.document.add_feature(FeatureNode {
            id: "imported".into(),
            name: "Imported".into(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        let bodies = app
            .document
            .evaluate_bodies(&std::collections::HashSet::new())
            .expect("evaluated GUI fixture");
        app.set_body_meshes(bodies);
        app.selected_faces.insert(("imported".into(), 0));
        app.commit_direct_face_command(DirectFaceCommand::PressPull);
        assert!(app.document.graph.node_weights().any(|node| {
            matches!(
                &node.feature,
                FeatureType::FaceOffset { target, .. } if target == "imported"
            )
        }));
        app.undo();
        assert!(!app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::FaceOffset { .. })));
        app.redo();
        assert!(app
            .document
            .graph
            .node_weights()
            .any(|node| matches!(node.feature, FeatureType::FaceOffset { .. })));
    }

    #[test]
    fn every_direct_face_command_has_semantics_and_undo_redo() {
        for command in [
            DirectFaceCommand::PressPull,
            DirectFaceCommand::Move,
            DirectFaceCommand::Delete,
            DirectFaceCommand::Thicken,
        ] {
            let mut app = ZeroCadApp::new();
            app.document.add_feature(FeatureNode {
                id: "imported".into(),
                name: "Imported".into(),
                feature: FeatureType::Box {
                    w: 10.0,
                    h: 10.0,
                    d: 10.0,
                },
            });
            let bodies = app
                .document
                .evaluate_bodies(&std::collections::HashSet::new())
                .expect("evaluated GUI fixture");
            app.set_body_meshes(bodies);
            app.selected_faces.insert(("imported".into(), 0));
            app.commit_direct_face_command(command);
            app.document
                .validate_semantic_contracts()
                .expect("GUI direct-edit semantics");
            let kind = match command {
                DirectFaceCommand::PressPull => "direct.face_offset",
                DirectFaceCommand::Move => "direct.face_move",
                DirectFaceCommand::Delete => "direct.face_delete",
                DirectFaceCommand::Thicken => "direct.face_thicken",
            };
            assert!(app
                .document
                .graph
                .node_weights()
                .any(|node| node.feature.kind_id() == kind));
            app.undo();
            assert!(!app
                .document
                .graph
                .node_weights()
                .any(|node| node.feature.kind_id() == kind));
            app.redo();
            assert!(app
                .document
                .graph
                .node_weights()
                .any(|node| node.feature.kind_id() == kind));
        }
    }
}
