use crate::*;

impl eframe::App for ZeroCadApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keep a context handle so a background refine worker can wake the UI.
        if self.egui_ctx.is_none() {
            self.egui_ctx = Some(ctx.clone());
        }
        // Swap in any finished background refine.
        self.poll_refine_eval();
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

        self.draw_top_bar(ctx);

        self.draw_feature_tree(ctx);

        self.draw_status_bar(ctx);

        self.draw_extrude_panel(ctx);

        self.draw_workspace_viewport(ctx);

        self.handle_sketch_keys(ctx);

        self.handle_3d_escape(ctx);

        self.persist_settings();
    }
}
