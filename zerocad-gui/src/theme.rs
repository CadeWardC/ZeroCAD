//! Viewport/UI theming: the adaptive text palette plus the light and dark
//! egui visual themes. Kept separate from the app logic so the color choices
//! live in one place.

use eframe::egui;

/// Semantic text colors that adapt to the active (light/dark) theme, so panel
/// labels stay readable in both. Mirrors the slate ramp used throughout the UI.
#[derive(Clone, Copy)]
pub struct Palette {
    /// Headings, logo, emphasized labels (light: slate-900).
    pub text_strong: egui::Color32,
    /// Primary body text (light: slate-600/700).
    pub text_body: egui::Color32,
    /// Secondary / metadata text (light: slate-500).
    pub text_muted: egui::Color32,
    /// Hints and disabled-looking text (light: slate-400).
    pub text_faint: egui::Color32,
}

impl Palette {
    pub fn light() -> Self {
        Self {
            text_strong: egui::Color32::from_rgb(15, 23, 42), // slate-900
            text_body: egui::Color32::from_rgb(71, 85, 105),  // slate-600
            text_muted: egui::Color32::from_rgb(100, 116, 139), // slate-500
            text_faint: egui::Color32::from_rgb(148, 163, 184), // slate-400
        }
    }

    pub fn dark() -> Self {
        Self {
            text_strong: egui::Color32::from_rgb(241, 245, 249), // slate-100
            text_body: egui::Color32::from_rgb(203, 213, 225),   // slate-300
            text_muted: egui::Color32::from_rgb(148, 163, 184),  // slate-400
            text_faint: egui::Color32::from_rgb(100, 116, 139),  // slate-500
        }
    }
}

pub fn apply_premium_light_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::light();

    // Core panels color palette (clean slates and white papers)
    visuals.panel_fill = egui::Color32::from_rgb(248, 250, 252); // soft off-white/gray slate (slate-50)
    visuals.window_fill = egui::Color32::WHITE;
    visuals.extreme_bg_color = egui::Color32::from_rgb(255, 255, 255); // perfect white canvas

    // Non-active body text and widgets (warm light gray buttons, slate text)
    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(241, 245, 249); // slate-100
    visuals.widgets.inactive.weak_bg_fill = egui::Color32::from_rgb(241, 245, 249);
    visuals.widgets.inactive.fg_stroke =
        egui::Stroke::new(1.0, egui::Color32::from_rgb(51, 65, 85)); // dark slate-700
    visuals.widgets.inactive.rounding = egui::Rounding::same(6.0);

    // Hovered elements (subtle warm slate highlight)
    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(226, 232, 240); // slate-200
    visuals.widgets.hovered.weak_bg_fill = egui::Color32::from_rgb(226, 232, 240);
    visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(15, 23, 42)); // dark slate-900
    visuals.widgets.hovered.rounding = egui::Rounding::same(6.0);

    // Active clicked elements (sleek premium bright blue accent)
    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(37, 99, 235); // blue-600
    visuals.widgets.active.weak_bg_fill = egui::Color32::from_rgb(37, 99, 235);
    visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    visuals.widgets.active.rounding = egui::Rounding::same(6.0);

    // Open/selected state (light blue background, darker blue stroke)
    visuals.widgets.open.bg_fill = egui::Color32::from_rgb(219, 234, 254); // blue-100
    visuals.widgets.open.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(29, 78, 216)); // blue-700

    visuals.selection.bg_fill = egui::Color32::from_rgb(191, 219, 254); // blue-200
    visuals.selection.stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(37, 99, 235));

    ctx.set_visuals(visuals);

    // Rounded margins, clean padding, and spacious fonts for a premium look
    let mut style = (*ctx.style()).clone();
    style.spacing.button_padding = egui::vec2(12.0, 7.0);
    style.spacing.item_spacing = egui::vec2(8.0, 8.0);
    ctx.set_style(style);
}

pub fn apply_premium_dark_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();

    // Core panels color palette (deep slate surfaces).
    visuals.panel_fill = egui::Color32::from_rgb(15, 23, 42); // slate-900
    visuals.window_fill = egui::Color32::from_rgb(30, 41, 59); // slate-800
    visuals.window_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(51, 65, 85)); // slate-700
    visuals.extreme_bg_color = egui::Color32::from_rgb(2, 6, 23); // near-black canvas

    // Default body text/widget colors (light slate so labels read on dark).
    visuals.override_text_color = Some(egui::Color32::from_rgb(203, 213, 225)); // slate-300

    // Non-active widgets (raised slate buttons, light text).
    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(51, 65, 85); // slate-700
    visuals.widgets.inactive.weak_bg_fill = egui::Color32::from_rgb(51, 65, 85);
    visuals.widgets.inactive.fg_stroke =
        egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)); // slate-200
    visuals.widgets.inactive.rounding = egui::Rounding::same(6.0);

    // Hovered elements (lighter slate highlight).
    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(71, 85, 105); // slate-600
    visuals.widgets.hovered.weak_bg_fill = egui::Color32::from_rgb(71, 85, 105);
    visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    visuals.widgets.hovered.rounding = egui::Rounding::same(6.0);

    // Active clicked elements (bright blue accent, same as light theme).
    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(37, 99, 235); // blue-600
    visuals.widgets.active.weak_bg_fill = egui::Color32::from_rgb(37, 99, 235);
    visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    visuals.widgets.active.rounding = egui::Rounding::same(6.0);

    // Open/selected state.
    visuals.widgets.open.bg_fill = egui::Color32::from_rgb(30, 58, 138); // blue-900
    visuals.widgets.open.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(191, 219, 254)); // blue-200

    visuals.selection.bg_fill = egui::Color32::from_rgb(30, 64, 175); // blue-800
    visuals.selection.stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(96, 165, 250)); // blue-400

    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.button_padding = egui::vec2(12.0, 7.0);
    style.spacing.item_spacing = egui::vec2(8.0, 8.0);
    ctx.set_style(style);
}
