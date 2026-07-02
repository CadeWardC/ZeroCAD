use crate::*;

impl ZeroCadApp {
    pub(crate) fn new() -> Self {
        let graph = ParametricGraph::new();
        let prefs = settings::AppSettings::load();

        Self {
            pending_visual: None,
            graph,
            selected_node_id: None,
            body_meshes: Vec::new(),
            mesh_stats: (0, 0),
            eval_gen: 0,
            eval_rx: None,
            eval_pending: false,
            egui_ctx: None,
            error_msg: None,
            status_msg: "Welcome to ZeroCAD. Ready for modeling.".to_string(),
            unresolved_features: std::collections::HashMap::new(),
            active_sketch_face_ref: None,
            doc_created_unix: None,
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
            pending_corners: Vec::new(),
            corner_radius_text: "5".to_string(),
            edge_mod_dist_text: "3".to_string(),
            detected_regions: Vec::new(),
            selected_region_indices: HashSet::new(),
            is_sketch_mode: false,
            is_plane_selection_mode: false,
            active_sketch_cs: CoordinateSystem::XY,
            active_sketch_on_face: false,
            active_tool: None,
            sketch_temp_start: None,
            sketch_points: Vec::new(),
            hovered_plane: None,
            hovered_sketch_face: None,
            selected_faces: HashSet::new(),
            selected_edges: HashSet::new(),
            selected_body: HashSet::new(),
            extrude_depth: 25.0,
            extrude_mode: ExtrudeMode::NewBody,
            extrude_op: None,
            extrude_preview_cache: None,
            extrude_preview_mesh_cache: None,
            extrude_preview_inflight: None,
            extrude_preview_rx: None,
            edge_mod_preview_cache: None,
            edge_mod_preview_mesh_cache: None,
            edge_mod_arc_cache: None,
            edge_mod_arc_inflight: None,
            edge_mod_arc_rx: None,
            edge_mod_settle: None,
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
            dim_input: None,
            dim_anchor: None,
            last_cursor: None,
            dim_screen_positions: Vec::new(),
            id_counter: 1,
            current_unit: prefs.unit,
            show_preferences: false,
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
        for idx in self.graph.graph.node_indices() {
            let node = &self.graph.graph[idx];
            if self.hidden_nodes.contains(&node.id) {
                continue;
            }
            if let FeatureType::VariableSet { variables } = &node.feature {
                for v in variables {
                    // Trim only gates emptiness; the *untrimmed* name is the key
                    // so it matches `ParametricGraph::variable_map` exactly (a
                    // typed preview must resolve to the same value core commits).
                    if !v.name.trim().is_empty() {
                        f(&v.name, v.value_in_base());
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
