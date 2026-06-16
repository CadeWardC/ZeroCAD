//! The Fusion-style inline dimension dialog shown while drawing a sketch shape,
//! plus the small value types that back it.

use eframe::egui;
use zerocad_core::Unit;

use crate::{SketchTool, ZeroCadApp};

/// One editable dimension field in the shape-creation dialog.
#[derive(Debug, Clone)]
pub(crate) struct DimField {
    /// Human label for the dimension ("Width", "Diameter", …). Set by
    /// [`dim_fields_for`]; retained for tooltips/labels even though the compact
    /// inline box doesn't render it today.
    #[allow(dead_code)]
    pub(crate) label: &'static str,
    pub(crate) value: String,
    /// True once the user pressed Enter on it — its value is fixed.
    pub(crate) locked: bool,
    /// True once the user typed in it — stop overwriting it with the live value.
    pub(crate) edited: bool,
}

/// Fusion 360-style inline dimension inputs shown at edge midpoints during
/// shape creation. Disappears once the shape is finalized.
#[derive(Debug, Clone)]
pub(crate) struct DimInput {
    pub(crate) fields: Vec<DimField>,
    /// Field index to grab keyboard focus on the next render, if any.
    pub(crate) focus_request: Option<usize>,
    /// Which field is currently active (receives keyboard input).
    pub(crate) active_field: usize,
    /// True on the frame a field just received focus — triggers select-all.
    pub(crate) select_all: bool,
}

/// Build the dimension fields for a tool.
pub(crate) fn dim_fields_for(tool: SketchTool) -> Vec<DimField> {
    let labels: &[&'static str] = match tool {
        SketchTool::Rectangle | SketchTool::RectangleCenter => &["Width", "Height"],
        SketchTool::Circle => &["Diameter"],
        SketchTool::Line => &["Length", "Angle (°)"],
        // 3-point tools (rotated rectangle, 3-point circle, ellipses) draw by
        // clicking points; the corner tools (fillet/chamfer) take their radius
        // from the toolbar. None use inline dimension fields.
        SketchTool::RectangleThreePoint
        | SketchTool::ThreePointCircle
        | SketchTool::Ellipse
        | SketchTool::ThreePointEllipse
        | SketchTool::Fillet
        | SketchTool::Chamfer => &[],
    };
    labels
        .iter()
        .map(|&label| DimField {
            label,
            value: String::new(),
            locked: false,
            edited: false,
        })
        .collect()
}

impl ZeroCadApp {
    /// Render Fusion 360-style inline dimension inputs at shape edge midpoints.
    /// Only shown while drawing (before the shape is finalized). Each field
    /// appears as a small input box at the edge midpoint. The active field has a
    /// blue border and its text is selected so typing replaces the value.
    /// Tab switches fields, Enter locks a field (finalizes when all locked),
    /// Escape cancels.
    pub(crate) fn show_dimension_dialog(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.dim_input.is_none() {
            return;
        }

        // Pulled out of `self` so the field widget can borrow the variable list
        // and the shared autocomplete state while the per-field closures hold
        // `&mut self`.
        let var_names = self.visible_variable_names();
        let mut ac = self.autocomplete.take();
        // Set when the autocomplete swallowed Enter/Tab to accept a suggestion,
        // so that key isn't also treated as "lock field" / "next field".
        let mut suppress_keys = false;

        let field_count = self.dim_input.as_ref().map(|d| d.fields.len()).unwrap_or(0);
        let active = self.dim_input.as_ref().map(|d| d.active_field).unwrap_or(0);
        let do_select_all = self
            .dim_input
            .as_ref()
            .map(|d| d.select_all)
            .unwrap_or(false);
        let focus_req = self.dim_input.as_ref().and_then(|d| d.focus_request);

        let unit_suffix = match self.current_unit {
            Unit::Millimeter => " mm",
            Unit::Inch => " in",
            Unit::Meter => " m",
        };

        // Render each field as an inline input at its screen position. Values may
        // be numbers, variables, or arithmetic expressions (evaluated later).
        for i in 0..field_count {
            let pos = self
                .dim_screen_positions
                .get(i)
                .copied()
                .unwrap_or(egui::pos2(100.0 + i as f32 * 120.0, 100.0));
            let is_active = i == active;

            // Read field state for rendering.
            let is_locked = self
                .dim_input
                .as_ref()
                .map(|d| d.fields[i].locked)
                .unwrap_or(false);

            // Fusion 360 style: active = blue border, inactive = subtle gray.
            let border_color = if is_active && !is_locked {
                egui::Color32::from_rgb(0, 120, 215)
            } else if is_locked {
                egui::Color32::from_rgb(100, 160, 100)
            } else {
                egui::Color32::from_rgb(160, 160, 160)
            };
            let border_width = if is_active && !is_locked { 1.5 } else { 1.0 };
            let bg = if is_active && !is_locked {
                egui::Color32::WHITE
            } else {
                egui::Color32::from_rgb(245, 245, 245)
            };

            let area_id = egui::Id::new("dim_inline").with(i);
            egui::Area::new(area_id)
                .order(egui::Order::Foreground)
                .interactable(false)
                .fixed_pos(pos - egui::vec2(40.0, 10.0))
                .show(ctx, |ui| {
                    egui::Frame::none()
                        .fill(bg)
                        .rounding(3.0)
                        .stroke(egui::Stroke::new(border_width, border_color))
                        .inner_margin(egui::Margin::symmetric(4.0, 2.0))
                        .show(ui, |ui| {
                            ui.spacing_mut().item_spacing.x = 2.0;
                            ui.horizontal(|ui| {
                                if let Some(dim) = self.dim_input.as_mut() {
                                    let f = &mut dim.fields[i];
                                    ui.style_mut().visuals.extreme_bg_color = bg;
                                    ui.style_mut().visuals.widgets.inactive.bg_stroke =
                                        egui::Stroke::NONE;
                                    ui.style_mut().visuals.widgets.hovered.bg_stroke =
                                        egui::Stroke::NONE;
                                    ui.style_mut().visuals.selection.bg_fill =
                                        egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.3);

                                    // Force-focus the active field; keep its text
                                    // selected until the user starts typing.
                                    let req_focus =
                                        focus_req == Some(i) || (is_active && !is_locked);
                                    let sel_all = do_select_all || !f.edited;
                                    let field_id = egui::Id::new(("sketch_dim_field", i));
                                    let outcome = crate::expr::autocomplete_field(
                                        ui,
                                        field_id,
                                        &mut f.value,
                                        50.0,
                                        false,
                                        req_focus,
                                        sel_all,
                                        &var_names,
                                        &mut ac,
                                    );
                                    if outcome.response.changed() || outcome.accepted {
                                        f.edited = true;
                                    }
                                    if outcome.accepted_via_key {
                                        suppress_keys = true;
                                    }
                                }

                                // Unit suffix label.
                                ui.label(
                                    egui::RichText::new(unit_suffix)
                                        .color(egui::Color32::from_rgb(120, 120, 120))
                                        .size(11.0),
                                );
                            });
                        });
                });
        }

        // Clear pending focus/select-all (applied above) and stash autocomplete.
        if let Some(d) = self.dim_input.as_mut() {
            d.focus_request = None;
            d.select_all = false;
        }
        self.autocomplete = ac;

        // Commit/navigation keys are read *after* the fields render, so an
        // Enter/Tab the autocomplete consumed (to accept a suggestion) never
        // leaks into locking the field or finalizing the shape.
        let enter = ctx.input(|i| i.key_pressed(egui::Key::Enter));
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        let tab = ctx.input(|i| i.key_pressed(egui::Key::Tab));

        // Tab: advance to the next unlocked field.
        if tab && !suppress_keys && field_count > 0 {
            if let Some(dim) = self.dim_input.as_mut() {
                let mut next = (active + 1) % field_count;
                for _ in 0..field_count {
                    if !dim.fields[next].locked {
                        break;
                    }
                    next = (next + 1) % field_count;
                }
                dim.active_field = next;
                dim.focus_request = Some(next);
                dim.select_all = true;
            }
        }

        // Enter: lock the active field, advance to the next, or finalize.
        if enter && !suppress_keys {
            if let Some(dim) = self.dim_input.as_mut() {
                let a = dim.active_field;
                if a < dim.fields.len() && !dim.fields[a].locked {
                    dim.fields[a].locked = true;
                    dim.fields[a].edited = true;
                    let next = (0..dim.fields.len()).find(|&j| !dim.fields[j].locked);
                    let all_locked = next.is_none();
                    if let Some(n) = next {
                        dim.active_field = n;
                        dim.focus_request = Some(n);
                        dim.select_all = true;
                    }
                    if all_locked {
                        let start = self.sketch_temp_start.unwrap_or((0.0, 0.0));
                        let cursor = self.last_cursor.unwrap_or((start.0 + 1.0, start.1 + 1.0));
                        self.finalize_shape(cursor);
                    }
                }
            }
        }

        if escape {
            self.cancel_in_progress_shape();
            self.autocomplete = None;
            self.status_msg = "Shape cancelled.".to_string();
        }
    }
}
