use crate::*;

impl ZeroCadApp {
    #[cfg(test)]
    pub(crate) fn new() -> Self {
        Self::new_with_settings(settings::AppSettings::load())
    }

    pub(crate) fn new_with_settings(prefs: settings::AppSettings) -> Self {
        let document = Document::new();
        let body_meshes = std::sync::Arc::new(Vec::new());
        let evaluated_scene = app::ViewportSceneState::new(std::sync::Arc::new(
            EvaluatedScene::from_part_bodies(body_meshes.clone()),
        ));
        let scene_stats = evaluated_scene.stats();
        let recovery = recovery::RecoveryManager::new();
        let recovery_available = recovery.has_recovery();

        Self {
            pending_visual: None,
            pending_mirror_join_feedback: None,
            document,
            selected_node_id: None,
            body_meshes,
            evaluated_scene,
            datum_values: std::collections::HashMap::new(),
            scene_stats,
            gpu: gpu_viewport::GpuViewport::default(),
            gpu_render: prefs.gpu_render,
            graphics_backend: prefs.backend,
            msaa_level: prefs.msaa,
            hydrated_cache_mb: match prefs.hydrated_cache_mb {
                64 | 128 | 256 | 512 => prefs.hydrated_cache_mb,
                _ => 128,
            },
            gpu_texture_id: None,
            frame_preview_plan: None,
            evaluator: evaluation_worker::ModelEvaluator::new(),
            document_worker: document_worker::DocumentWorker::new(),
            recovery,
            pending_save: None,
            last_slow_frame_log: None,
            export_completions: std::sync::Arc::new(std::sync::Mutex::new(Vec::new())),
            eval_generation: 0,
            document_revision: 0,
            eval_pending: false,
            eval_started: None,
            egui_ctx: None,
            error_msg: None,
            status_msg: if recovery_available {
                "A crash-safe autosave is available from File → Recover Autosave.".to_string()
            } else {
                "Welcome to ZeroCAD. Ready for modeling.".to_string()
            },
            unresolved_features: std::collections::HashMap::new(),
            active_sketch_face_ref: None,
            active_sketch_datum_ref: None,
            doc_created_unix: None,
            current_document_path: None,
            // Positive pitch starts the camera above the XZ ground plane,
            // looking down at it (negative would start underneath).
            camera_pitch: 0.7,
            camera_yaw: 0.7,
            camera_zoom: 7.5,
            camera_pan: egui::Vec2::ZERO,
            is_perspective: true,
            orbiting: false,
            pre_sketch_pitch: 0.7,
            pre_sketch_yaw: 0.7,
            pre_sketch_perspective: true,
            camera_anim_active: false,
            camera_anim_start_pitch: 0.0,
            camera_anim_start_yaw: 0.0,
            camera_anim_target_pitch: 0.0,
            camera_anim_target_yaw: 0.0,
            camera_anim_start_time: 0.0,
            camera_anim_duration: 0.4, // 400ms transition
            sketch_curves: SketchCurves::new(),
            sketch_shapes: Vec::new(),
            sketch_corner_mods: Vec::new(),
            sketch_mirrors: Vec::new(),
            pending_corners: Vec::new(),
            corner_radius_text: "5".to_string(),
            offset_distance_text: String::new(),
            sketch_pattern_x_text: "1".to_string(),
            sketch_pattern_y_text: "0".to_string(),
            sketch_pattern_spacing_text: "10".to_string(),
            sketch_pattern_count_text: "3".to_string(),
            sketch_pattern_angle_text: "360".to_string(),
            sketch_trim_preview: None,
            polygon_sides: 6,
            edge_mod_dist_text: "3".to_string(),
            detected_regions: Vec::new(),
            selected_region_indices: HashSet::new(),
            is_sketch_mode: false,
            is_plane_selection_mode: false,
            active_sketch_cs: CoordinateSystem::XY,
            active_sketch_on_face: false,
            active_face_boundary: SketchCurves::new(),
            active_tool: None,
            editing_sketch_id: None,
            sketch_solver_model: None,
            sketch_entity_ids: Vec::new(),
            sketch_next_entity_id: 0,
            sketch_drag_point: None,
            sketch_selected_ids: Vec::new(),
            sketch_selected_constraint: None,
            sketch_dimension_editor: None,
            sketch_dimension_positions: HashMap::new(),
            sketch_conflict_constraint: None,
            sketch_temp_start: None,
            sketch_points: Vec::new(),
            line_chain_start: None,
            hovered_plane: None,
            hovered_datum_plane: None,
            measure_density: 1.0,
            inspection_dialog: None,
            section_view: None,
            hovered_sketch_face: None,
            selected_faces: HashSet::new(),
            selected_edges: HashSet::new(),
            selected_body: HashSet::new(),
            body_clipboard: None,
            move_op: None,
            combine_op: None,
            split_body_op: None,
            scale_body_op: None,
            move_preview_bodies: None,
            extrude_depth: 25.0,
            extrude_mode: ExtrudeMode::NewBody,
            extrude_op: None,
            revolve_op: None,
            pattern_op: None,
            hole_op: None,
            shell_op: None,
            thread_op: None,
            sweep_op: None,
            draft_op: None,
            extrude_preview_cache: None,
            extrude_preview_mesh_cache: None,
            extrude_ghost_base: None,
            extrude_preview_inflight: None,
            extrude_preview_settle: None,
            edge_mod_preview_cache: None,
            edge_mod_preview_mesh_cache: None,
            edge_mod_arc_cache: None,
            edge_mod_arc_lru: Vec::new(),
            edge_mod_arc_inflight: None,
            edge_mod_arc_failed: None,
            extrude_depth_dragging: false,
            extrude_dim_pos: None,
            edge_mod_op: None,
            edge_mod_dim_pos: None,
            edge_mod_handle: None,
            corner_dim_pos: None,
            corner_handle: None,
            hidden_nodes: HashSet::new(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            working_sketch_undo: Vec::new(),
            dim_input: None,
            dim_anchor: None,
            last_cursor: None,
            cursor_snap_kind: None,
            snap_guides: Vec::new(),
            cursor_snap_guides: Vec::new(),
            dim_screen_positions: Vec::new(),
            id_counter: 1,
            current_unit: prefs.unit,
            snap_enabled: prefs.snap_enabled,
            grid_visible: prefs.grid_visible,
            show_preferences: false,
            show_about: false,
            parameters_dialog: None,
            settings_tab: SettingsTab::General,
            keymap: Keymap::load(),
            capturing_shortcut: None,
            show_onboarding: prefs.show_onboarding,
            save_dialog: None,
            dark_mode: prefs.dark_mode,
            theme_applied: None,
            renaming_node: None,
            rename_buffer: String::new(),
            rename_focus_pending: false,
            autocomplete: None,
            recent_files: settings::RecentFiles::load(),
            onboarding_visible: prefs.show_onboarding,
            onboarding_textures: HashMap::new(),
            pending_onboarding_texture_evictions: HashSet::new(),
            settings_baseline: prefs,
        }
    }

    /// Visit every `(name, base-unit value)` in a **visible** variable set —
    /// values in the **base unit (mm)**, the same form the parametric engine
    /// resolves expressions in (`ParametricGraph::variable_map`), so a typed
    /// preview matches the committed geometry exactly. Hidden sets are excluded
    /// from the *suggestion list* only; resolution in core always sees them.
    ///
    /// Shared by the `visible_variable_*` helpers so the graph walk + filtering
    /// lives in one place and each consumer builds exactly the collection it
    /// needs (no intermediate `Vec` just to `collect` it into something else).
    pub(crate) fn for_each_visible_variable(&self, mut f: impl FnMut(&str, f64)) {
        let resolved = self.document.resolve_variables();
        for idx in self.document.graph.node_indices() {
            let node = &self.document.graph[idx];
            if self.hidden_nodes.contains(&node.id) {
                continue;
            }
            if let FeatureType::VariableSet { variables } = &node.feature {
                for v in variables {
                    let name = v.name.trim();
                    if let Some(value) = resolved.values.get(name).copied() {
                        f(name, value);
                    }
                }
            }
        }
    }

    /// Sorted, de-duplicated variable names for the autocomplete suggestion list.
    pub(crate) fn visible_variable_names(&self) -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        self.for_each_visible_variable(|name, _| names.push(name.to_string()));
        names.sort();
        names.dedup();
        names
    }

    /// Variable lookup map for expression evaluation. Built directly in a single
    /// allocation rather than via an intermediate `Vec`.
    pub(crate) fn visible_variable_map(&self) -> std::collections::HashMap<String, f64> {
        let mut map = std::collections::HashMap::new();
        self.for_each_visible_variable(|name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    /// Evaluate a dimension field's text as an arithmetic expression over the
    /// visible variables, returning the numeric value in the current unit.
    /// `None` while the text is empty or malformed (so callers hold the last
    /// good value as the user types).
    pub(crate) fn eval_dim(&self, text: &str) -> Option<f32> {
        expr::eval(text, &self.visible_variable_map())
            .ok()
            .map(|v| v as f32)
    }

    /// The semantic text palette for the currently active theme.
    pub(crate) fn pal(&self) -> Palette {
        if self.dark_mode {
            Palette::dark()
        } else {
            Palette::light()
        }
    }
}

#[cfg(test)]
mod variable_expression_tests {
    use super::*;

    #[test]
    fn visible_variable_map_uses_resolved_expression_values() {
        let mut app = ZeroCadApp::new();
        app.document.add_feature(FeatureNode {
            id: "variables_1".to_string(),
            name: "Variables".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![
                    Variable {
                        name: "blade_width".to_string(),
                        value: 42.8,
                        unit: Unit::Millimeter,
                        expression: None,
                    },
                    Variable {
                        name: "half_width".to_string(),
                        value: 0.0,
                        unit: Unit::Millimeter,
                        expression: Some("blade_width / 2".to_string()),
                    },
                ],
            },
        });

        let variables = app.visible_variable_map();
        assert!((variables["blade_width"] - 42.8).abs() < 1.0e-9);
        assert!((variables["half_width"] - 21.4).abs() < 1.0e-9);
    }
}
