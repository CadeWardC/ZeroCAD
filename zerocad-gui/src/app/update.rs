use crate::*;

impl eframe::App for ZeroCadApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        let frame_started = std::time::Instant::now();
        // A thumbnail queued last frame is no longer referenced by that frame's
        // command buffer, so it is now safe to let egui release the texture.
        self.flush_onboarding_texture_evictions();
        // Keep a context handle so a background refine worker can wake the UI.
        if self.egui_ctx.is_none() {
            self.egui_ctx = Some(ctx.clone());
        }
        // Hand the GPU viewport eframe's wgpu device/queue for this frame. Cloned
        // (the render state is Arc-backed) so nothing borrows `frame` past here.
        self.gpu
            .set_render_state(frame.wgpu_render_state().cloned());
        // Swap in any finished background refine.
        self.poll_refine_eval();
        self.poll_document_worker();
        self.tick_speculative_edge_mod(ctx);
        // While the Welcome modal is up the workspace is inert, so its hotkeys
        // are suppressed (the modal reads Esc itself).
        if !self.onboarding_visible {
            self.handle_shortcuts(ctx);
        }

        self.apply_theme(ctx);

        // Welcome modal (drawn as a Foreground layer over everything below).
        self.draw_onboarding(ctx);

        self.animate_camera(ctx);

        // SAVE DIALOG (modal overlay, drawn before the Settings window).
        self.show_save_dialog(ctx);

        self.draw_settings_window(ctx);

        self.show_parameters_dialog(ctx);

        self.show_inspection_dialog(ctx);

        self.draw_top_bar(ctx);

        self.show_move_dialog(ctx);
        self.show_combine_dialog(ctx);
        self.show_split_body_dialog(ctx);
        self.show_scale_body_dialog(ctx);

        self.draw_feature_tree(ctx);

        self.draw_status_bar(ctx);

        self.draw_extrude_panel(ctx);

        self.draw_workspace_viewport(ctx);

        self.handle_sketch_keys(ctx);

        self.handle_3d_escape(ctx);

        self.persist_settings();
        let elapsed = frame_started.elapsed();
        if elapsed >= std::time::Duration::from_millis(17)
            && self
                .last_slow_frame_log
                .is_none_or(|last| last.elapsed() >= std::time::Duration::from_secs(1))
        {
            log::debug!("slow UI frame: {elapsed:?}");
            self.last_slow_frame_log = Some(std::time::Instant::now());
        }
    }
}
