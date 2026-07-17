use crate::*;

impl ZeroCadApp {
    pub(crate) fn import_dxf(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("AutoCAD DXF", &["dxf"])
            .pick_file()
        else {
            return;
        };
        match zerocad_core::read_dxf_file(&path) {
            Ok(imported) => self.commit_dxf_import(imported),
            Err(error) => {
                self.error_msg = Some(format!("Could not import DXF: {error}"));
            }
        }
    }

    fn commit_dxf_import(&mut self, imported: zerocad_core::DxfImport) {
        let layer_count = imported.shapes.len();
        let curve_count = imported.imported_curve_count();
        let unit_name = imported.unit.name;
        let warnings: Vec<String> = imported
            .diagnostics
            .iter()
            .filter(|diagnostic| {
                diagnostic.severity == zerocad_core::DxfDiagnosticSeverity::Warning
            })
            .map(|diagnostic| diagnostic.message.clone())
            .collect();

        if self.is_sketch_mode {
            for shape in imported.shapes {
                if let Some(model) = &mut self.sketch_solver_model {
                    let owner = zerocad_core::sketch::EntityId(self.sketch_next_entity_id);
                    let vars = self.document.variable_map();
                    let (addition, next) =
                        zerocad_core::sketch::constraints::promote_shapes_to_entities(
                            std::slice::from_ref(&shape),
                            &[owner],
                            &vars,
                            self.sketch_next_entity_id + 1,
                        );
                    model.points.extend(addition.points);
                    model.entities.extend(addition.entities);
                    model.constraints.extend(addition.constraints);
                    self.sketch_entity_ids.push(owner);
                    self.sketch_next_entity_id = next;
                }
                self.sketch_shapes.push(shape);
            }
            self.rebuild_active_sketch_curves();
            self.status_msg = import_status(layer_count, curve_count, unit_name, &warnings);
            return;
        }

        let source_name = imported
            .shapes
            .first()
            .and_then(|shape| match shape {
                SketchShape::Imported { metadata, .. } => Some(metadata.source_name.clone()),
                _ => None,
            })
            .unwrap_or_else(|| "DXF".to_string());
        let id = format!("sketch_{}", self.next_id());
        let node = FeatureNode {
            id: id.clone(),
            name: format!("DXF — {source_name}"),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XY,
                curves: SketchCurves::new(),
                entity_ids: zerocad_core::sketch::EntityId::sequence(imported.shapes.len()),
                next_entity_id: imported.shapes.len() as u32,
                shapes: imported.shapes,
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                solver: None,
            },
        };
        self.push_undo();
        self.document.add_feature(node);
        self.selected_node_id = Some(id);
        self.reevaluate_geometry();
        self.status_msg = import_status(layer_count, curve_count, unit_name, &warnings);
    }
}

fn import_status(
    layer_count: usize,
    curve_count: usize,
    unit_name: &str,
    warnings: &[String],
) -> String {
    let mut status = format!(
        "Imported {curve_count} DXF curve(s) on {layer_count} layer(s); source units: {unit_name}."
    );
    if !warnings.is_empty() {
        status.push_str(&format!(
            " {} warning(s): {}",
            warnings.len(),
            warnings.join("; ")
        ));
    }
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn import_outside_sketch_creates_an_xy_sketch_with_metadata() {
        let text = "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\nPROFILE\n10\n0\n20\n0\n11\n10\n21\n0\n0\nENDSEC\n0\nEOF\n";
        let imported = zerocad_core::read_dxf_str("profile.dxf", text).unwrap();
        let mut app = ZeroCadApp::new();
        app.commit_dxf_import(imported);
        let node = app
            .document
            .graph
            .node_indices()
            .map(|index| &app.document.graph[index])
            .find(|node| node.id.starts_with("sketch_"))
            .unwrap();
        let FeatureType::Sketch { cs, shapes, .. } = &node.feature else {
            panic!("DXF must create a sketch feature")
        };
        assert_eq!(*cs, CoordinateSystem::XY);
        assert!(matches!(
            &shapes[0],
            SketchShape::Imported { metadata, .. }
                if metadata.layer == "PROFILE" && metadata.source_name == "profile.dxf"
        ));
    }
}
