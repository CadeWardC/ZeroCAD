use crate::*;

impl ZeroCadApp {
    pub(crate) fn apply_theme(&mut self, ctx: &egui::Context) {
        // Apply the active theme aesthetics (light by default, optional dark).
        // Rebuilding the egui Visuals + Style is costly, so only do it when the
        // theme actually changed rather than on every frame.
        if self.theme_applied != Some(self.dark_mode) {
            if self.dark_mode {
                apply_premium_dark_theme(ctx);
            } else {
                apply_premium_light_theme(ctx);
            }
            self.theme_applied = Some(self.dark_mode);
        }
    }

    pub(crate) fn animate_camera(&mut self, ctx: &egui::Context) {
        // Handle camera animation interpolation if active
        if self.camera_anim_active {
            let current_time = ctx.input(|i| i.time);
            let elapsed = current_time - self.camera_anim_start_time;
            if elapsed >= self.camera_anim_duration {
                self.camera_pitch = self.camera_anim_target_pitch;
                self.camera_yaw = self.camera_anim_target_yaw;
                self.camera_anim_active = false;
                log::debug!(
                    "Camera animation complete. Locked at pitch: {:.2}, yaw: {:.2}",
                    self.camera_pitch,
                    self.camera_yaw
                );
            } else {
                let t = (elapsed / self.camera_anim_duration) as f32;
                // Smootherstep: zero velocity and acceleration at both ends,
                // avoiding the small start/stop jerk of cubic smoothstep.
                let t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
                self.camera_pitch = self.camera_anim_start_pitch
                    + (self.camera_anim_target_pitch - self.camera_anim_start_pitch) * t_smooth;
                self.camera_yaw = self.camera_anim_start_yaw
                    + (self.camera_anim_target_yaw - self.camera_anim_start_yaw) * t_smooth;
                ctx.request_repaint(); // Smooth animation repaint request
            }
        }
    }

    pub(crate) fn handle_sketch_keys(&mut self, ctx: &egui::Context) {
        if self.is_sketch_mode
            && self.dim_input.is_none()
            && self.sketch_dimension_editor.is_none()
            && self.active_tool.is_some_and(SketchTool::is_spline)
            && ctx.input(|i| i.key_pressed(egui::Key::Enter))
        {
            self.finish_in_progress_spline();
        }

        // Enter commits the staged Fillet/Chamfer corners (the dimension dialog,
        // when open, owns Enter for its own fields — so only act when it's not).
        if self.is_sketch_mode
            && self.dim_input.is_none()
            && self.sketch_dimension_editor.is_none()
            && !self.pending_corners.is_empty()
            && ctx.input(|i| i.key_pressed(egui::Key::Enter))
        {
            self.commit_pending_corners();
        }

        // Global Escape handling while sketching. The 2-point dimension dialog
        // handles its own Escape; here we cover the cases it doesn't: discarding
        // staged fillet/chamfer corners, a half-placed multi-point shape (no
        // dialog), and deselecting the tool.
        if self.is_sketch_mode
            && self.dim_input.is_none()
            && self.sketch_dimension_editor.is_none()
            && ctx.input(|i| i.key_pressed(egui::Key::Escape))
        {
            if self.clear_pending_corners() {
                // Discarded the staged corners, keep the tool armed.
                self.status_msg = "Staged corners discarded.".to_string();
            } else if !self.sketch_points.is_empty() {
                // Abort the in-progress (multi-point) shape or Line chain, keep
                // the tool armed. Committed segments are kept.
                self.line_chain_start = None;
                self.cancel_in_progress_shape();
                self.status_msg = "Shape cancelled.".to_string();
            } else if self.active_tool.is_some() {
                self.line_chain_start = None;
                self.active_tool = None;
                self.status_msg = "Tool deselected — select faces, edges, or points.".to_string();
                log::info!("Escape: switched to Select mode");
            }
        }
    }

    pub(crate) fn handle_3d_escape(&mut self, ctx: &egui::Context) {
        if self.draft_op.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.draft_op = None;
            self.status_msg = "Draft cancelled.".to_string();
            return;
        }
        if self.combine_op.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.combine_op = None;
            self.status_msg = "Combine cancelled.".to_string();
            return;
        }
        if self.move_op.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.cancel_move_body();
            return;
        }
        if self.split_body_op.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.split_body_op = None;
            self.status_msg = "Split Body cancelled.".to_string();
            return;
        }
        if self.scale_body_op.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.scale_body_op = None;
            self.move_preview_bodies = None;
            self.status_msg = "Scale Body cancelled.".to_string();
            return;
        }
        // In the plain 3D view (no sketch, no live op or dialog), Escape returns to
        // the neutral Select state by clearing the current selection.
        if !self.is_sketch_mode
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.move_op.is_none()
            && self.combine_op.is_none()
            && self.split_body_op.is_none()
            && self.scale_body_op.is_none()
            && self.dim_input.is_none()
            && !self.is_plane_selection_mode
            && ctx.input(|i| i.key_pressed(egui::Key::Escape))
            && (!self.selected_body.is_empty()
                || !self.selected_faces.is_empty()
                || !self.selected_edges.is_empty())
        {
            self.selected_body.clear();
            self.selected_faces.clear();
            self.selected_edges.clear();
            self.status_msg = "Selection cleared.".to_string();
        }
    }

    pub(crate) fn persist_settings(&mut self) {
        // Persist preferences whenever the unit, theme, or onboarding toggle
        // changed this frame. One diff against the last-saved snapshot covers
        // every edit site (Settings window, Ctrl+D theme toggle, the Welcome
        // footer checkbox) without threading a save into each.
        let current = settings::AppSettings {
            show_onboarding: self.show_onboarding,
            dark_mode: self.dark_mode,
            unit: self.current_unit,
            gpu_render: self.gpu_render,
            backend: self.graphics_backend,
            msaa: self.msaa_level,
            hydrated_cache_mb: self.hydrated_cache_mb,
            snap_enabled: self.snap_enabled,
            grid_visible: self.grid_visible,
        };
        if current != self.settings_baseline {
            current.save();
            self.settings_baseline = current;
        }
    }
}
