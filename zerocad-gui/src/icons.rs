//! Vector icons for toolbar/panel buttons.
//!
//! The `icons/` folder holds simple iconoir-style SVGs (24×24 viewBox, 1.5
//! stroke). egui can't render SVG directly, and rasterizing them (egui_extras +
//! resvg) would bake a fixed color and pull in a heavy dependency. Instead we
//! flatten each path's `M/L/H/V/C/Z` commands ourselves and stroke/fill them
//! with the painter in a caller-supplied color — so an icon recolors with its
//! button's state (active/hover) and stays crisp at any DPI, matching the
//! viewport's own CPU-vector style.
//!
//! ### ADDING OR MODIFYING ICONS IN THE FUTURE:
//! 1. Place the new SVG file under the `icons/` folder.
//! 2. Define a new `pub const NAME: &str` at the top of this file using `include_str!("path/to/icon.svg")`.
//! 3. Add a corresponding variant to the `Icon` enum (e.g., `Icon::MyNewIcon`).
//! 4. Map the new enum variant to its constant in the `Icon::svg(&self)` match block.
//! 
//! Call `.labeled_button()`, `.icon_button()`, or `.menu_button()` directly on the `Icon` variant in `main.rs` to render.

use eframe::egui;

// Embedded at compile time so the binary needs no icon files at runtime.
pub const LINE: &str = include_str!("../../icons/sketch/line.svg");
pub const RECTANGLE: &str = include_str!("../../icons/sketch/square-corner-to-corner.svg");
pub const CIRCLE: &str = include_str!("../../icons/sketch/one-point-circle.svg");
pub const RECTANGLE_FROM_CENTER: &str = include_str!("../../icons/sketch/square3d-from-center.svg");
pub const RECTANGLE_THREE_POINTS: &str = include_str!("../../icons/sketch/square3d-three-points.svg");
pub const THREE_POINT_CIRCLE: &str = include_str!("../../icons/sketch/three-points-circle.svg");
pub const ELLIPSE: &str = include_str!("../../icons/sketch/ellipse3d.svg");
pub const THREE_POINT_ELLIPSE: &str = include_str!("../../icons/sketch/ellipse3d-three-points.svg");
pub const FILLET: &str = include_str!("../../icons/sketch/fillet3d.svg");
pub const CHAMFER: &str = include_str!("../../icons/sketch/chamfer3d.svg");
pub const SKETCH: &str = include_str!("../../icons/sketch/sketch.svg");
pub const EXTRUDE: &str = include_str!("../../icons/3d/extrude.svg");
pub const CHECK: &str = include_str!("../../icons/general/check.svg");
pub const EYE_OPEN: &str = include_str!("../../icons/general/eye-solid.svg");
pub const EYE_CLOSED: &str = include_str!("../../icons/general/eye-closed.svg");
pub const TRASH: &str = include_str!("../../icons/general/trash.svg");
pub const SAVE: &str = include_str!("../../icons/general/floppy-disk-arrow-out.svg");
pub const DOWNLOAD: &str = include_str!("../../icons/general/download.svg");
pub const SETTINGS: &str = include_str!("../../icons/general/settings.svg");
pub const FOLDER: &str = include_str!("../../icons/general/folder.svg");
pub const LOG_OUT: &str = include_str!("../../icons/general/log-out.svg");
pub const NEW_DESIGN: &str = include_str!("../../icons/general/new-design.svg");

/// Unified icon registry. Adding or replacing an icon is as simple as updating
/// this enum and its path mapping in `svg()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Icon {
    Line,
    Rectangle,
    RectangleFromCenter,
    RectangleThreePoints,
    Circle,
    ThreePointCircle,
    Ellipse,
    ThreePointEllipse,
    Fillet,
    Chamfer,
    Extrude,
    Check,
    EyeOpen,
    EyeClosed,
    Trash,
    Save,
    Download,
    Settings,
    New,
    Exit,
    Folder,
    Sketch,
}

impl Icon {
    /// Return the raw SVG string for this icon.
    pub fn svg(&self) -> &'static str {
        match self {
            Icon::Line => LINE,
            Icon::Rectangle => RECTANGLE,
            Icon::RectangleFromCenter => RECTANGLE_FROM_CENTER,
            Icon::RectangleThreePoints => RECTANGLE_THREE_POINTS,
            Icon::Circle => CIRCLE,
            Icon::ThreePointCircle => THREE_POINT_CIRCLE,
            Icon::Ellipse => ELLIPSE,
            Icon::ThreePointEllipse => THREE_POINT_ELLIPSE,
            Icon::Fillet => FILLET,
            Icon::Chamfer => CHAMFER,
            Icon::Extrude => EXTRUDE,
            Icon::Check => CHECK,
            Icon::EyeOpen => EYE_OPEN,
            Icon::EyeClosed => EYE_CLOSED,
            Icon::Trash => TRASH,
            Icon::Save => SAVE,
            Icon::Download => DOWNLOAD,
            Icon::Settings => SETTINGS,
            Icon::Folder => FOLDER,
            Icon::Exit => LOG_OUT,
            Icon::New => NEW_DESIGN,
            Icon::Sketch => SKETCH,
        }
    }

    /// Draw the icon with the painter at `rect` in the specified `color`.
    pub fn draw(&self, painter: &egui::Painter, rect: egui::Rect, color: egui::Color32) {
        draw(painter, rect, color, self.svg());
    }

    /// Draw a premium pill button containing this icon and a text label.
    pub fn labeled_button(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        bg: egui::Color32,
        hover: egui::Color32,
        accent: egui::Color32,
        stroke: egui::Stroke,
    ) -> egui::Response {
        labeled_button(ui, *self, label, bg, hover, accent, stroke)
    }

    /// Draw a compact square button containing only this icon.
    pub fn icon_button(
        &self,
        ui: &mut egui::Ui,
        bg: egui::Color32,
        hover: egui::Color32,
        accent: egui::Color32,
    ) -> egui::Response {
        icon_button(ui, *self, bg, hover, accent)
    }

    /// Draw a premium dropdown menu button containing this icon and a text label.
    pub fn menu_button(&self, ui: &mut egui::Ui, label: &str) -> egui::Response {
        let h = 24.0f32;
        let w = ui.available_width().max(140.0);
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::click());
        if ui.is_rect_visible(rect) {
            let is_hovered = resp.hovered();
            let fill = if is_hovered {
                ui.visuals().widgets.hovered.bg_fill
            } else {
                egui::Color32::TRANSPARENT
            };
            ui.painter().rect(rect, 4.0, fill, egui::Stroke::NONE);

            let icon_rect = egui::Rect::from_min_size(
                egui::pos2(rect.left() + 6.0, rect.center().y - 8.0),
                egui::vec2(16.0, 16.0),
            );
            let text_color = if is_hovered {
                ui.visuals().widgets.hovered.fg_stroke.color
            } else {
                ui.visuals().widgets.inactive.fg_stroke.color
            };
            self.draw(ui.painter(), icon_rect, text_color);
            ui.painter().text(
                egui::pos2(icon_rect.right() + 8.0, rect.center().y),
                egui::Align2::LEFT_CENTER,
                label,
                egui::FontId::proportional(12.0),
                text_color,
            );
        }
        resp
    }

    /// Like [`menu_button`](Self::menu_button) but with a right-aligned, dimmed
    /// keyboard-shortcut hint (e.g. "Ctrl+S"). An empty `hint` renders the same
    /// as a plain menu button.
    pub fn menu_button_hint(&self, ui: &mut egui::Ui, label: &str, hint: &str) -> egui::Response {
        let h = 24.0f32;
        let w = ui.available_width().max(140.0);
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::click());
        if ui.is_rect_visible(rect) {
            let is_hovered = resp.hovered();
            let fill = if is_hovered {
                ui.visuals().widgets.hovered.bg_fill
            } else {
                egui::Color32::TRANSPARENT
            };
            ui.painter().rect(rect, 4.0, fill, egui::Stroke::NONE);

            let icon_rect = egui::Rect::from_min_size(
                egui::pos2(rect.left() + 6.0, rect.center().y - 8.0),
                egui::vec2(16.0, 16.0),
            );
            let text_color = if is_hovered {
                ui.visuals().widgets.hovered.fg_stroke.color
            } else {
                ui.visuals().widgets.inactive.fg_stroke.color
            };
            self.draw(ui.painter(), icon_rect, text_color);
            ui.painter().text(
                egui::pos2(icon_rect.right() + 8.0, rect.center().y),
                egui::Align2::LEFT_CENTER,
                label,
                egui::FontId::proportional(12.0),
                text_color,
            );
            if !hint.is_empty() {
                ui.painter().text(
                    egui::pos2(rect.right() - 8.0, rect.center().y),
                    egui::Align2::RIGHT_CENTER,
                    hint,
                    egui::FontId::proportional(11.0),
                    text_color.gamma_multiply(0.65),
                );
            }
        }
        resp
    }
}

/// How finely cubic Béziers are sampled. 10 segments is smooth at icon scale.
const BEZIER_SEGMENTS: usize = 10;

/// Parsed icon geometry: one entry per `<path>`, each carrying its fill flag and
/// the flattened subpaths (points in the 24×24 viewBox space).
type ParsedIcon = Vec<(bool, Vec<Vec<(f32, f32)>>)>;

thread_local! {
    /// Tokenizing the path `d` data and sampling its cubic Béziers is pure,
    /// input-independent work, but `draw` runs every frame for every visible
    /// icon button. Cache the parsed geometry by SVG-string identity so each
    /// icon is flattened exactly once per process instead of on every paint.
    static PARSED: std::cell::RefCell<
        std::collections::HashMap<usize, std::rc::Rc<ParsedIcon>>,
    > = std::cell::RefCell::new(std::collections::HashMap::new());
}

/// Fetch (or build and cache) the flattened geometry for `svg`.
///
/// Keyed by the string's pointer: every caller passes one of the embedded
/// `&'static str` icon constants (through [`Icon::svg`]), so the identity is
/// stable for the lifetime of the program.
fn parsed_icon(svg: &str) -> std::rc::Rc<ParsedIcon> {
    let key = svg.as_ptr() as usize;
    PARSED.with(|cache| {
        if let Some(parsed) = cache.borrow().get(&key) {
            return parsed.clone();
        }
        let built: ParsedIcon = paths(svg)
            .map(|(data, filled)| (filled, flatten(data)))
            .collect();
        let rc = std::rc::Rc::new(built);
        cache.borrow_mut().insert(key, rc.clone());
        rc
    })
}

/// Draw an embedded SVG into `rect`, stroking (or filling) every path in `color`.
/// The 24×24 viewBox is mapped uniformly into `rect`; stroke width tracks the
/// icon size so it reads the same as the rest of the UI.
pub fn draw(painter: &egui::Painter, rect: egui::Rect, color: egui::Color32, svg: &str) {
    let sx = rect.width() / 24.0;
    let sy = rect.height() / 24.0;
    let origin = rect.left_top();
    let map = |p: (f32, f32)| egui::pos2(origin.x + p.0 * sx, origin.y + p.1 * sy);
    let stroke_w = (1.5 * sx).clamp(1.1, 2.0);
    let stroke = egui::Stroke::new(stroke_w, color);

    for (filled, subpaths) in parsed_icon(svg).iter() {
        for sub in subpaths {
            if sub.len() < 2 {
                continue;
            }
            let pts: Vec<egui::Pos2> = sub.iter().map(|&p| map(p)).collect();
            if *filled {
                painter.add(egui::Shape::convex_polygon(pts, color, egui::Stroke::NONE));
            } else {
                painter.add(egui::Shape::line(pts, stroke));
            }
        }
    }
}

/// A pill button: SVG icon + label, auto-sized to the text, matching the sketch
/// toolbar's look. `accent` colors the icon, label and outline; `bg`/`hover` the
/// fill. Returns the click response so callers handle `.clicked()`/`.on_hover`.
pub fn labeled_button(
    ui: &mut egui::Ui,
    icon: Icon,
    label: &str,
    bg: egui::Color32,
    hover: egui::Color32,
    accent: egui::Color32,
    stroke: egui::Stroke,
) -> egui::Response {
    let font = egui::FontId::proportional(12.0);
    let text_w = ui
        .fonts(|f| f.layout_no_wrap(label.to_owned(), font.clone(), accent))
        .size()
        .x;
    let (icon_s, pad, gap, h) = (16.0f32, 9.0f32, 6.0f32, 28.0f32);
    let w = pad + icon_s + gap + text_w + pad;
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::click());
    if ui.is_rect_visible(rect) {
        let fill = if resp.hovered() { hover } else { bg };
        ui.painter().rect(rect, 4.0, fill, stroke);
        let icon_rect = egui::Rect::from_min_size(
            egui::pos2(rect.left() + pad, rect.center().y - icon_s / 2.0),
            egui::vec2(icon_s, icon_s),
        );
        icon.draw(ui.painter(), icon_rect, accent);
        ui.painter().text(
            egui::pos2(icon_rect.right() + gap, rect.center().y),
            egui::Align2::LEFT_CENTER,
            label,
            font,
            accent,
        );
    }
    resp
}

/// A compact square icon-only button (e.g. the browser's show/hide eye).
pub fn icon_button(
    ui: &mut egui::Ui,
    icon: Icon,
    bg: egui::Color32,
    hover: egui::Color32,
    accent: egui::Color32,
) -> egui::Response {
    let s = 24.0f32;
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(s, s), egui::Sense::click());
    if ui.is_rect_visible(rect) {
        let fill = if resp.hovered() { hover } else { bg };
        ui.painter().rect(rect, 4.0, fill, egui::Stroke::NONE);
        let icon_s = 16.0;
        let icon_rect = egui::Rect::from_center_size(rect.center(), egui::vec2(icon_s, icon_s));
        icon.draw(ui.painter(), icon_rect, accent);
    }
    resp
}

/// Iterate `(path_data, is_filled)` for each `<path>` in the SVG. A path counts
/// as filled when it carries an explicit `fill="#..."` (the dots and the eye
/// pupil); everything else is stroke-only (`fill="none"` / inherited).
fn paths(svg: &str) -> impl Iterator<Item = (&str, bool)> {
    svg.split("<path").skip(1).filter_map(|tag| {
        let end = tag.find('>')?;
        let head = &tag[..end];
        let dstart = head.find("d=\"")? + 3;
        let dlen = head[dstart..].find('"')?;
        let data = &head[dstart..dstart + dlen];
        let filled = head.contains("fill=\"#");
        Some((data, filled))
    })
}

/// Flatten one path's `d` data into screen-space-ready subpaths of points
/// (cubic Béziers sampled, `Z` closing the loop). Supports the absolute and
/// relative `M/L/H/V/C/Z` commands these icons use; anything else is skipped.
fn flatten(d: &str) -> Vec<Vec<(f32, f32)>> {
    let toks = tokenize(d);
    let mut subpaths: Vec<Vec<(f32, f32)>> = Vec::new();
    let mut cur: Vec<(f32, f32)> = Vec::new();
    let (mut x, mut y) = (0.0f32, 0.0f32);
    let (mut start_x, mut start_y) = (0.0f32, 0.0f32);
    let mut i = 0;
    let mut cmd = ' ';
    let num = |toks: &[Tok], i: &mut usize| -> Option<f32> {
        while *i < toks.len() {
            match toks[*i] {
                Tok::Num(n) => {
                    *i += 1;
                    return Some(n);
                }
                Tok::Cmd(_) => return None,
            }
        }
        None
    };
    while i < toks.len() {
        if let Tok::Cmd(c) = toks[i] {
            cmd = c;
            i += 1;
        }
        let rel = cmd.is_ascii_lowercase();
        match cmd.to_ascii_uppercase() {
            'M' => {
                let (Some(nx), Some(ny)) = (num(&toks, &mut i), num(&toks, &mut i)) else {
                    break;
                };
                if !cur.is_empty() {
                    subpaths.push(std::mem::take(&mut cur));
                }
                x = if rel { x + nx } else { nx };
                y = if rel { y + ny } else { ny };
                start_x = x;
                start_y = y;
                cur.push((x, y));
                // Extra coordinate pairs after an M are implicit line-tos.
                while let (Some(nx), Some(ny)) = (num(&toks, &mut i), num(&toks, &mut i)) {
                    x = if rel { x + nx } else { nx };
                    y = if rel { y + ny } else { ny };
                    cur.push((x, y));
                }
            }
            'L' => {
                while let (Some(nx), Some(ny)) = (num(&toks, &mut i), num(&toks, &mut i)) {
                    x = if rel { x + nx } else { nx };
                    y = if rel { y + ny } else { ny };
                    cur.push((x, y));
                }
            }
            'H' => {
                while let Some(nx) = num(&toks, &mut i) {
                    x = if rel { x + nx } else { nx };
                    cur.push((x, y));
                }
            }
            'V' => {
                while let Some(ny) = num(&toks, &mut i) {
                    y = if rel { y + ny } else { ny };
                    cur.push((x, y));
                }
            }
            'C' => {
                while let (
                    Some(x1),
                    Some(y1),
                    Some(x2),
                    Some(y2),
                    Some(ex),
                    Some(ey),
                ) = (
                    num(&toks, &mut i),
                    num(&toks, &mut i),
                    num(&toks, &mut i),
                    num(&toks, &mut i),
                    num(&toks, &mut i),
                    num(&toks, &mut i),
                ) {
                    let (c1x, c1y) = if rel { (x + x1, y + y1) } else { (x1, y1) };
                    let (c2x, c2y) = if rel { (x + x2, y + y2) } else { (x2, y2) };
                    let (px, py) = if rel { (x + ex, y + ey) } else { (ex, ey) };
                    let (p0x, p0y) = (x, y);
                    for k in 1..=BEZIER_SEGMENTS {
                        let t = k as f32 / BEZIER_SEGMENTS as f32;
                        cur.push(cubic(p0x, p0y, c1x, c1y, c2x, c2y, px, py, t));
                    }
                    x = px;
                    y = py;
                }
            }
            'Z' => {
                cur.push((start_x, start_y));
                if !cur.is_empty() {
                    subpaths.push(std::mem::take(&mut cur));
                }
                x = start_x;
                y = start_y;
            }
            other => {
                // Only M/L/H/V/C/Z are supported. A new icon exported with
                // quadratic/arc/smooth commands (Q/T/S/A) would otherwise be
                // silently truncated here — warn so it's caught at author time.
                log::warn!(
                    "icons: unsupported SVG path command '{other}' — icon will render incompletely"
                );
                break;
            }
        }
    }
    if !cur.is_empty() {
        subpaths.push(cur);
    }
    subpaths
}

fn cubic(
    p0x: f32, p0y: f32, c1x: f32, c1y: f32, c2x: f32, c2y: f32, p1x: f32, p1y: f32, t: f32,
) -> (f32, f32) {
    let u = 1.0 - t;
    let w0 = u * u * u;
    let w1 = 3.0 * u * u * t;
    let w2 = 3.0 * u * t * t;
    let w3 = t * t * t;
    (
        w0 * p0x + w1 * c1x + w2 * c2x + w3 * p1x,
        w0 * p0y + w1 * c1y + w2 * c2y + w3 * p1y,
    )
}

enum Tok {
    Cmd(char),
    Num(f32),
}

/// Split path `d` data into command letters and numbers. Numbers may be
/// separated by spaces, commas, or just a sign / leading dot.
fn tokenize(d: &str) -> Vec<Tok> {
    let b = d.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i] as char;
        if c.is_ascii_alphabetic() && c != 'e' && c != 'E' {
            out.push(Tok::Cmd(c));
            i += 1;
        } else if c == '-' || c == '+' || c == '.' || c.is_ascii_digit() {
            let start = i;
            i += 1; // consume the leading sign / digit / dot
            let mut seen_dot = c == '.';
            while i < b.len() {
                let d = b[i] as char;
                if d.is_ascii_digit() {
                    i += 1;
                } else if d == '.' && !seen_dot {
                    seen_dot = true;
                    i += 1;
                } else if d == 'e' || d == 'E' {
                    i += 1;
                    if i < b.len() && (b[i] as char == '-' || b[i] as char == '+') {
                        i += 1;
                    }
                } else {
                    break;
                }
            }
            if let Ok(n) = d[start..i].parse::<f32>() {
                out.push(Tok::Num(n));
            }
        } else {
            i += 1; // separator
        }
    }
    out
}
