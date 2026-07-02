use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use eframe::egui;
use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{
    detect_regions, CoordinateSystem, CornerKind, CornerMod, Dimension, EdgeRef, ExtrudeMode,
    FeatureNode, FeatureType, MockMesh, ParametricGraph, Region, SketchCurves, SketchPlane,
    SketchShape, Unit, Variable, Vec3,
};

mod edgemod;
mod expr;
mod extrude;
mod geom2d;
mod icons;
mod render;
mod settings;
mod shortcuts;
mod sketch_ui;
mod theme;
mod thumbnail;
use edgemod::EdgeModOp;
use expr::Autocomplete;
use extrude::ExtrudeOp;
use geom2d::{circumcircle, dist_point_to_segment, is_point_in_quad, project_point_on_segment};
use shortcuts::{Keymap, ShortcutAction};
use sketch_ui::{dim_fields_for, DimInput};
use theme::{apply_premium_dark_theme, apply_premium_light_theme, Palette};

fn main() -> eframe::Result<()> {
    let mut builder = env_logger::Builder::from_default_env();
    if std::env::var("RUST_LOG").is_err() {
        builder.filter_level(log::LevelFilter::Debug);
    }
    builder.init();

    log::info!("========================================================");
    log::info!("Starting ZeroCAD - Premium 3D Parametric CAD Designer...");
    log::info!("Console debug logger initialized at level: DEBUG");
    log::info!("========================================================");

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ZeroCAD - 3D Parametric CAD Designer")
            .with_inner_size([1200.0, 800.0]),
        ..Default::default()
    };

    eframe::run_native(
        "ZeroCAD",
        options,
        Box::new(|_cc| Box::new(ZeroCadApp::new())),
    )
}

/// A drawing tool *mode*. Each toolbar button (a [`ToolFamily`]) exposes one or
/// more of these via its flyout; the first listed is that button's default.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchTool {
    Line,
    /// Corner-to-corner rectangle (the Rectangle button's default).
    Rectangle,
    /// Rectangle from its center to a corner.
    RectangleCenter,
    /// Rotated rectangle from a base edge (2 points) + a height (3rd point).
    RectangleThreePoint,
    /// Center + radius circle (the Circle button's default).
    Circle,
    /// Circle through three points on its circumference.
    ThreePointCircle,
    /// Center ellipse: center, major-axis endpoint, then minor radius.
    Ellipse,
    /// Ellipse from a major-axis diameter (2 points) + minor radius (3rd point).
    ThreePointEllipse,
    /// Round a sketch corner (click the corner). Not a draw tool.
    Fillet,
    /// Bevel a sketch corner (click the corner). Not a draw tool.
    Chamfer,
}

/// The toolbar button a [`SketchTool`] lives under. Switching buttons is by
/// family; the flyout picks the exact mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolFamily {
    Line,
    Rectangle,
    Circle,
    /// The corner-modifier button, holding both Fillet and Chamfer (its flyout
    /// switches between them) — mirroring the single 3D edge fillet/chamfer button.
    Corner,
}

impl SketchTool {
    /// Which toolbar button this mode belongs to.
    pub fn family(self) -> ToolFamily {
        match self {
            SketchTool::Line => ToolFamily::Line,
            SketchTool::Rectangle
            | SketchTool::RectangleCenter
            | SketchTool::RectangleThreePoint => ToolFamily::Rectangle,
            SketchTool::Circle
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse => ToolFamily::Circle,
            SketchTool::Fillet => ToolFamily::Corner,
            SketchTool::Chamfer => ToolFamily::Corner,
        }
    }

    /// How many points (clicks) the *drawing* tool places before the shape
    /// finalizes. The corner tools (Fillet/Chamfer) aren't drawn this way.
    pub fn point_count(self) -> usize {
        match self {
            SketchTool::Line
            | SketchTool::Rectangle
            | SketchTool::RectangleCenter
            | SketchTool::Circle => 2,
            SketchTool::RectangleThreePoint
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse => 3,
            SketchTool::Fillet | SketchTool::Chamfer => 1,
        }
    }

    /// The corner-modifier kind for the Fillet/Chamfer tools, else `None`.
    pub fn corner_kind(self) -> Option<CornerKind> {
        match self {
            SketchTool::Fillet => Some(CornerKind::Fillet),
            SketchTool::Chamfer => Some(CornerKind::Chamfer),
            _ => None,
        }
    }

    /// The icon representing this mode.
    pub fn icon(self) -> icons::Icon {
        match self {
            SketchTool::Line => icons::Icon::Line,
            SketchTool::Rectangle => icons::Icon::Rectangle,
            SketchTool::RectangleCenter => icons::Icon::RectangleFromCenter,
            SketchTool::RectangleThreePoint => icons::Icon::RectangleThreePoints,
            SketchTool::Circle => icons::Icon::Circle,
            SketchTool::ThreePointCircle => icons::Icon::ThreePointCircle,
            SketchTool::Ellipse => icons::Icon::Ellipse,
            SketchTool::ThreePointEllipse => icons::Icon::ThreePointEllipse,
            SketchTool::Fillet => icons::Icon::Fillet,
            SketchTool::Chamfer => icons::Icon::Chamfer,
        }
    }

    /// Short menu label for the flyout.
    pub fn label(self) -> &'static str {
        match self {
            SketchTool::Line => "Line",
            SketchTool::Rectangle => "Rectangle",
            SketchTool::RectangleCenter => "Center Rectangle",
            SketchTool::RectangleThreePoint => "3-Point Rectangle",
            SketchTool::Circle => "Center Circle",
            SketchTool::ThreePointCircle => "3-Point Circle",
            SketchTool::Ellipse => "Ellipse",
            SketchTool::ThreePointEllipse => "3-Point Ellipse",
            SketchTool::Fillet => "Fillet",
            SketchTool::Chamfer => "Chamfer",
        }
    }
}

impl ToolFamily {
    /// The default mode selected when its toolbar button is first clicked.
    pub fn default_mode(self) -> SketchTool {
        match self {
            ToolFamily::Line => SketchTool::Line,
            ToolFamily::Rectangle => SketchTool::Rectangle,
            ToolFamily::Circle => SketchTool::Circle,
            ToolFamily::Corner => SketchTool::Fillet,
        }
    }

    /// The modes offered in this button's flyout, in order (first = default).
    pub fn modes(self) -> &'static [SketchTool] {
        match self {
            ToolFamily::Line => &[SketchTool::Line],
            ToolFamily::Rectangle => &[
                SketchTool::Rectangle,
                SketchTool::RectangleCenter,
                SketchTool::RectangleThreePoint,
            ],
            ToolFamily::Circle => &[
                SketchTool::Circle,
                SketchTool::ThreePointCircle,
                SketchTool::Ellipse,
                SketchTool::ThreePointEllipse,
            ],
            ToolFamily::Corner => &[SketchTool::Fillet, SketchTool::Chamfer],
        }
    }
}

/// The variable expressions bound to a sketch's dimensions, for display in the
/// property panel (so the user can see which dimensions follow a variable).
fn sketch_variable_dims(shapes: &[SketchShape]) -> Vec<String> {
    let mut out = Vec::new();
    for s in shapes {
        let dims: Vec<&Dimension> = match s {
            SketchShape::Rectangle { w, h, .. } => vec![w, h],
            SketchShape::Circle { diameter, .. } => vec![diameter],
            SketchShape::Line {
                length, angle_deg, ..
            } => vec![length, angle_deg],
            SketchShape::Raw { .. } => vec![],
        };
        for d in dims {
            if let Some(e) = &d.expr {
                out.push(e.clone());
            }
        }
    }
    out
}

/// A selected element of a solid body, paired with the body's node id in
/// `selected_body`. `Face` carries the B-rep face id (see `MockMesh::face_ids`),
/// `Edge`/`Vertex` carry indices into the body's wireframe, `Whole` is the
/// whole body (from a double-click).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BodyPick {
    Face(u32),
    Edge(u32),
    Vertex(u32),
    Whole,
}

/// What the user did on a feature-tree row this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RowAction {
    None,
    Delete,
    ToggleVisibility,
    /// Right-clicked "Add Variable" on a variable-set row.
    AddVariable,
}

/// Which tab is selected in the Settings window (left rail). More tabs can be
/// added here as settings grow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SettingsTab {
    General,
    Shortcuts,
}

impl SettingsTab {
    /// Tabs shown in the left rail, in display order.
    const ALL: &'static [SettingsTab] = &[SettingsTab::General, SettingsTab::Shortcuts];

    fn label(self) -> &'static str {
        match self {
            SettingsTab::General => "General",
            SettingsTab::Shortcuts => "Shortcuts",
        }
    }
}

/// File format choices offered in the save dialog.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SaveFormat {
    /// Full `.zcad` with embedded mesh cache for instant open.
    ZcadFull,
    /// Lightweight `.zcad` — recipe only, geometry regenerated on open.
    ZcadLightweight,
}

impl SaveFormat {
    fn label(self) -> &'static str {
        match self {
            SaveFormat::ZcadFull => "ZeroCAD Full (.zcadh)",
            SaveFormat::ZcadLightweight => "ZeroCAD (.zcad)",
        }
    }

    fn extension(self) -> &'static str {
        match self {
            SaveFormat::ZcadFull => "zcadh",
            SaveFormat::ZcadLightweight => "zcad",
        }
    }
}

/// State for the in-app save dialog modal.
struct SaveDialogState {
    /// User-editable project title (becomes the file stem).
    project_title: String,
    /// Which format to save in.
    save_format: SaveFormat,
    /// Target directory.
    save_dir: PathBuf,
}

/// Result delivered by a background refine evaluation: its generation tag (to
/// discard superseded jobs) and the evaluated bodies + warnings (or an error).
type EvalResult = (u64, Result<(Vec<(String, MockMesh)>, Vec<String>), String>);
/// Result delivered by an asynchronous live-preview evaluation.
type PreviewBodiesResult = (u64, Result<Vec<(String, MockMesh)>, String>);

pub(crate) enum PendingVisualMode {
    Extrude(ExtrudeMode),
    EdgeMod(CornerKind),
}

pub(crate) struct PendingCommitVisual {
    pub(crate) bodies: Vec<(String, MockMesh)>,
    pub(crate) mesh: Option<MockMesh>,
    pub(crate) mode: PendingVisualMode,
}

struct ZeroCadApp {
    pending_visual: Option<PendingCommitVisual>,
    graph: ParametricGraph,
    selected_node_id: Option<String>,
    /// One mesh per solid body (node id + mesh), so faces/edges/points can be
    /// picked per body. Replaces the old single combined `current_mesh`.
    body_meshes: Vec<(String, MockMesh)>,
    /// Cached `(vertices, triangles)` totals across `body_meshes`, refreshed
    /// only when the meshes change so the status bar doesn't re-sum every
    /// vertex/index of the whole model on every frame.
    mesh_stats: (usize, usize),
    /// Background "refine" evaluation for slow arc fillets. `reevaluate_geometry`
    /// shows the fast faceted draft instantly, then — when the model has a fillet
    /// — spawns the arc-cutter evaluation on a worker thread and swaps the result
    /// in here, so committing a fillet never stalls the UI (the arc boolean is
    /// ~1s; see `ParametricGraph::has_arc_fillet`). Stale jobs are ignored by
    /// generation.
    eval_gen: u64,
    eval_rx: Option<std::sync::mpsc::Receiver<EvalResult>>,
    /// True while a background refine is in flight (drives a "Refining…" hint).
    eval_pending: bool,
    /// A clone of the egui context, captured each frame, so a worker thread can
    /// wake the UI (`request_repaint`) the instant its result is ready.
    egui_ctx: Option<egui::Context>,
    error_msg: Option<String>,
    status_msg: String,
    /// Per-feature "unresolved" reasons from the last evaluation, keyed by feature
    /// id. A feature here failed to resolve/apply (its edge/face reference didn't
    /// reattach, its boolean couldn't run) and is flagged in the history tree with
    /// a ⚠ marker instead of silently coming out wrong.
    unresolved_features: std::collections::HashMap<String, String>,
    /// When sketching on a body face, the captured reference to that face, carried
    /// from face-pick to sketch-commit so the finished sketch stores it (and the
    /// sketch plane then follows the body). `None` for origin-plane sketches.
    active_sketch_face_ref: Option<zerocad_core::parametric::FaceRef>,
    /// Creation timestamp (Unix seconds) of the document currently open, carried
    /// from the loaded `.zcad` so re-saving preserves "created" rather than
    /// stamping it anew. `None` for a fresh/never-saved or legacy document.
    doc_created_unix: Option<u64>,

    // Camera parameters for the 3D Viewport
    camera_pitch: f32,      // Pitch (up/down rotation) in radians
    camera_yaw: f32,        // Yaw (left/right rotation) in radians
    camera_zoom: f32,       // Zoom factor
    camera_pan: egui::Vec2, // Pan offsets in screen coordinates
    is_perspective: bool,   // Toggle between Perspective and Orthographic projection
    /// True while a middle-button orbit/pan drag is in progress. Latched on
    /// middle-press over the viewport and held until release, so orbiting never
    /// stalls if egui momentarily drops its own drag tracking mid-motion.
    orbiting: bool,

    // Saved camera state before entering sketch mode
    pre_sketch_pitch: f32,
    pre_sketch_yaw: f32,
    pre_sketch_perspective: bool,

    // Camera animation state
    camera_anim_active: bool,
    camera_anim_start_pitch: f32,
    camera_anim_start_yaw: f32,
    camera_anim_target_pitch: f32,
    camera_anim_target_yaw: f32,
    camera_anim_start_time: f64,
    camera_anim_duration: f64,

    // Active Sketching state
    /// Live (resolved) geometry of the sketch being drawn — derived from
    /// `sketch_shapes` against the current variables. Used for rendering, region
    /// detection, and snapping while drawing.
    sketch_curves: SketchCurves,
    /// Parametric source of the in-progress sketch: one entry per finalized
    /// shape, capturing any variable-bound dimensions. Persisted on the node at
    /// Finish Sketch so dimensions keep following their variables.
    sketch_shapes: Vec<SketchShape>,
    /// Fillet/chamfer modifiers applied to corners of the in-progress sketch.
    sketch_corner_mods: Vec<CornerMod>,
    /// Corners the user has clicked with the Fillet/Chamfer tool but not yet
    /// committed. They preview live with the current radius and are only folded
    /// into `sketch_corner_mods` when the user presses Enter / clicks OK.
    pending_corners: Vec<(f32, f32)>,
    /// Editable radius/setback for the Fillet/Chamfer tools (a number or a
    /// variable expression).
    corner_radius_text: String,
    /// Editable radius/setback for the **3D** edge Fillet/Chamfer (applied to a
    /// selected body edge). A number or a variable expression.
    edge_mod_dist_text: String,
    detected_regions: Vec<Region>,
    selected_region_indices: HashSet<usize>,
    is_sketch_mode: bool,
    is_plane_selection_mode: bool,
    /// The plane the active sketch is being drawn on (an origin plane or an
    /// arbitrary body face). Drives the click→(u,v) mapping and rendering.
    active_sketch_cs: CoordinateSystem,
    /// True when the active sketch sits on an existing body face (not an origin
    /// plane). Persisted onto the sketch node so the extrude tool can pick a
    /// sensible default mode (face → Join/Cut by direction, plane → New Body).
    active_sketch_on_face: bool,
    active_tool: Option<SketchTool>,
    /// First click of any 2-click tool (line/rect/circle). When `None`, the
    /// next click sets the starting point; when `Some(pt)` it completes the shape.
    /// Mirrors `sketch_points[0]` (kept for the dim dialog + preview anchor).
    sketch_temp_start: Option<(f32, f32)>,
    /// All points placed so far for the in-progress shape *except* the final one
    /// (which the next click / cursor supplies). 2-point tools hold one entry;
    /// 3-point tools hold up to two. Cleared when the shape finalizes or cancels.
    sketch_points: Vec<(f32, f32)>,
    hovered_plane: Option<SketchPlane>,
    /// In plane-selection mode, the planar body face `(node_id, face_id)` under
    /// the cursor. A hovered face takes priority over the origin plane quads and,
    /// when clicked, starts a sketch on that face (the same path as pre-selecting
    /// a face and pressing Draw Sketch).
    hovered_sketch_face: Option<(String, u32)>,

    /// Faces of finished sketches the user has selected (in 3D) for extrusion,
    /// keyed by `(sketch_id, region_index)`. Selection persists until extruded
    /// or cleared, so the user can pick faces first and extrude afterwards.
    selected_faces: HashSet<(String, usize)>,
    /// Selected sketch edges, keyed by `(sketch_id, edge_index)` where the index
    /// is `segment i` for `i < segments.len()` else `circle (i - segments.len())`.
    selected_edges: HashSet<(String, usize)>,
    /// Selected elements of solid bodies, keyed by `(body_node_id, BodyPick)`.
    /// Separate from the sketch selection above so the extrude workflow is
    /// unaffected.
    selected_body: HashSet<(String, BodyPick)>,
    /// Depth used by the Extrude action.
    extrude_depth: f32,
    /// Last-used extrude mode (new body / join / cut), seeded into each new op.
    extrude_mode: ExtrudeMode,
    /// Active (uncommitted) extrude operation with its live preview, if any.
    extrude_op: Option<ExtrudeOp>,
    /// Memoized live Cut/Join preview: `(input hash, evaluated bodies)`. The
    /// preview re-runs the whole parametric model (truck booleans), which is far
    /// too slow to redo every frame, so it's cached and only recomputed when the
    /// extrude's depth / mode / targets actually change. Cleared when the op ends.
    extrude_preview_cache: Option<(u64, Vec<(String, MockMesh)>)>,
    /// Memoized live extrude *tool* ghost (the orange New-Body volume / red Cut
    /// volume): `(input hash, mesh)`. Like `extrude_preview_cache`, it's rebuilt
    /// only when the depth/targets change, not on every repaint (e.g. mouse moves
    /// over the viewport while the dialog is open).
    extrude_preview_mesh_cache: Option<(u64, MockMesh)>,
    /// In-flight exact extrude preview job. The lightweight tool mesh is shown
    /// until this worker returns the real Cut/Join/overlap-NewBody result.
    extrude_preview_inflight: Option<u64>,
    extrude_preview_rx: Option<std::sync::mpsc::Receiver<PreviewBodiesResult>>,
    /// Memoized live edge fillet/chamfer preview: `(input hash, bodies)`. Like
    /// `extrude_preview_cache`, the underlying `preview_edge_mod_bodies` clones the
    /// graph and re-runs every truck boolean — far too slow to redo on every
    /// repaint while the size box is open or the handle is dragged. Recomputed only
    /// when the size/kind/target actually change. Cleared when the op ends.
    edge_mod_preview_cache: Option<(u64, Vec<(String, MockMesh)>)>,
    /// Memoized lightweight edge fillet/chamfer overlay mesh shown immediately
    /// while the exact worker-computed preview bodies are still pending.
    edge_mod_preview_mesh_cache: Option<(u64, MockMesh)>,
    /// **Speculative** arc-fillet precompute for the live edge mod. While the user
    /// is still adjusting a fillet, the moment its size settles (stops changing for
    /// `EDGE_MOD_SETTLE`) the slow analytic-arc geometry for that size is computed
    /// on a worker thread and cached here, keyed by the same hash `commit_edge_mod`
    /// recomputes. If the user then commits at that size, the one-face result is
    /// already done and is applied instantly — no faceted→arc "pop" a second later.
    /// Holds `(key, bodies, warnings)`; cleared when the op ends.
    edge_mod_arc_cache: Option<(u64, Vec<(String, MockMesh)>, Vec<String>)>,
    /// In-flight speculative arc job: its key (so a finished result can be matched
    /// to the size it was computed for) and the channel it reports on. At most one
    /// runs at a time — while it's busy, size changes don't spawn more.
    edge_mod_arc_inflight: Option<u64>,
    edge_mod_arc_rx: Option<
        std::sync::mpsc::Receiver<(u64, Result<(Vec<(String, MockMesh)>, Vec<String>), String>)>,
    >,
    /// Debounce tracker for the speculative precompute: the current size key and
    /// when it was first observed. The arc job is spawned only once a key has been
    /// stable for `EDGE_MOD_SETTLE`, so a fast drag doesn't kick off a job per step.
    edge_mod_settle: Option<(u64, std::time::Instant)>,
    /// True while the user is actively push/pull dragging the extrude depth in
    /// the viewport. During the drag we render only the cheap ghost tool volume
    /// (`cached_preview_mesh`) following the cursor live, and SKIP the expensive
    /// truck boolean (`cached_preview_extrude_bodies`). The real booleaned result
    /// is computed once on release, when this flag clears.
    extrude_depth_dragging: bool,
    /// Screen anchor for the inline extrude distance box (preview centroid).
    extrude_dim_pos: Option<egui::Pos2>,
    /// Active (uncommitted) 3D edge fillet/chamfer with its live preview, if any.
    edge_mod_op: Option<EdgeModOp>,
    /// Screen anchor for the inline edge-mod size box (projected edge midpoint).
    edge_mod_dim_pos: Option<egui::Pos2>,
    /// The drag manipulator for the active edge mod: `(edge midpoint screen pos,
    /// handle screen pos, outward axis in px-per-mm)`. Dragging the handle along
    /// the axis changes the fillet/chamfer size live. Set each frame in the
    /// renderer while an edge mod is active.
    edge_mod_handle: Option<(egui::Pos2, egui::Pos2, egui::Vec2)>,
    /// Screen anchor for the inline 2D corner-radius box (last staged corner).
    corner_dim_pos: Option<egui::Pos2>,
    /// The drag manipulator for the 2D Fillet/Chamfer radius: `(corner screen
    /// pos, handle screen pos, bisector axis in px-per-mm)`. Dragging the handle
    /// along the corner's bisector changes the radius live. Set in the renderer
    /// while a corner tool is armed and at least one corner is staged.
    corner_handle: Option<(egui::Pos2, egui::Pos2, egui::Vec2)>,
    /// Node ids (sketches or bodies) the user has hidden in the browser.
    hidden_nodes: HashSet<String>,

    /// Snapshot stack for Undo (Ctrl+Z). Each entry is a serialized
    /// `ParametricGraph`; capped at 50 entries to bound memory.
    undo_stack: Vec<String>,
    /// Snapshot stack for Redo (Ctrl+Y / Ctrl+Shift+Z). Cleared whenever a new
    /// destructive change is committed.
    redo_stack: Vec<String>,

    /// Active shape-dimension dialog (after the first click of a shape).
    dim_input: Option<DimInput>,
    /// Screen anchor (last cursor position) for placing the dimension dialog.
    dim_anchor: Option<egui::Pos2>,
    /// Last cursor position in sketch-plane coordinates (for live dims / finalize).
    last_cursor: Option<(f32, f32)>,
    /// Screen-space positions for inline dimension labels (Fusion 360 style).
    dim_screen_positions: Vec<egui::Pos2>,

    /// Monotonic counter for generating unique feature ids (survives deletes).
    id_counter: usize,

    // Unit settings
    current_unit: Unit,

    /// Whether the Settings window is open.
    show_preferences: bool,
    /// Which tab is selected in the Settings window.
    settings_tab: SettingsTab,
    /// User-configurable keyboard shortcuts (loaded from disk on startup).
    keymap: Keymap,
    /// The action whose binding the Shortcuts tab is currently capturing a new
    /// key for, if any. While `Some`, the next key press is recorded as its new
    /// binding and the normal shortcut dispatcher is suspended.
    capturing_shortcut: Option<ShortcutAction>,
    /// Whether the onboarding screen is shown on startup.
    show_onboarding: bool,
    /// The in-app save dialog, if currently open.
    save_dialog: Option<SaveDialogState>,
    /// Dark theme toggle.
    dark_mode: bool,
    /// Last theme actually pushed to egui (`Some(dark_mode)`). The full
    /// `Visuals` + `Style` rebuild is expensive, so we only re-apply when this
    /// disagrees with `dark_mode` rather than every frame.
    theme_applied: Option<bool>,

    /// Id of the document-browser node currently being renamed inline (via
    /// double-click or the right-click "Rename"), if any.
    renaming_node: Option<String>,
    /// Edit buffer for the inline rename.
    rename_buffer: String,
    /// Set when a rename just started, so the text field grabs focus exactly
    /// once (re-requesting every frame would block click-away).
    rename_focus_pending: bool,

    /// The single live variable-name autocomplete popup shared by every
    /// dimension field (sketch + extrude). `None` when no popup is open.
    autocomplete: Option<Autocomplete>,

    /// Recently saved/opened `.zcad` projects (newest first), shown on the
    /// onboarding screen. Loaded from disk on startup; missing files pruned.
    recent_files: settings::RecentFiles,
    /// Whether the onboarding (Welcome) card is showing **right now**. Seeded
    /// from the persisted `show_onboarding` preference at startup, but distinct
    /// from it: dismissing the card for this session doesn't change the
    /// "pop up on startup" preference.
    onboarding_visible: bool,
    /// Lazily-uploaded GPU textures for Recent thumbnails, keyed by project path,
    /// so the onboarding screen uploads each `.thumb` to egui only once.
    onboarding_textures: HashMap<PathBuf, egui::TextureHandle>,
    /// Last preference snapshot persisted to `settings.json`. Compared at the end
    /// of every frame so any change to the unit / dark mode / onboarding toggle
    /// is saved without threading a save call through each edit site.
    settings_baseline: settings::AppSettings,
}

fn circle_from_three_2d(a: (f32, f32), b: (f32, f32), c: (f32, f32)) -> Option<(f32, f32, f32)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1.0e-7 {
        return None;
    }
    let a2 = a.0 * a.0 + a.1 * a.1;
    let b2 = b.0 * b.0 + b.1 * b.1;
    let c2 = c.0 * c.0 + c.1 * c.1;
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    let r = ((ux - a.0).powi(2) + (uy - a.1).powi(2)).sqrt();
    Some((ux, uy, r))
}

fn normalize_positive(mut a: f32) -> f32 {
    while a < 0.0 {
        a += std::f32::consts::TAU;
    }
    while a >= std::f32::consts::TAU {
        a -= std::f32::consts::TAU;
    }
    a
}

fn angle_in_span_f32(angle: f32, start: f32, end: f32, tol: f32) -> bool {
    if (end - start).abs() >= std::f32::consts::TAU - tol {
        return true;
    }
    if end >= start {
        let mut rel = angle - start;
        while rel < -tol {
            rel += std::f32::consts::TAU;
        }
        while rel > std::f32::consts::TAU + tol {
            rel -= std::f32::consts::TAU;
        }
        rel <= end - start + tol
    } else {
        let mut rel = start - angle;
        while rel < -tol {
            rel += std::f32::consts::TAU;
        }
        while rel > std::f32::consts::TAU + tol {
            rel -= std::f32::consts::TAU;
        }
        rel <= start - end + tol
    }
}

mod app;

/// How far to push/pull an extrude for a mouse `delta`, given the extrude axis
/// `n` (the sketch-plane normal) and the current view (`pitch`, `yaw`,
/// `view_scale` = screen px per world unit).
///
/// The axis is projected into screen space exactly as the viewport's
/// `project_3d` projects points (orthographic form; screen-y grows downward),
/// and the drag's component *along that projected axis* is converted back to
/// world units. This makes push/pull track the cursor for any plane
/// orientation, instead of reading vertical mouse motion — which only matched
/// the axis on planes whose normal happens to point up the screen (XY/XZ) and
/// ran backwards on YZ and tilted face planes.
///
/// Returns `None` when the axis projects to under ~15% of its full on-screen
/// length (you are sighting almost straight down it), where mapping would be
/// hyper-sensitive; the caller then falls back to plain vertical drag.
fn extrude_depth_delta(
    n: Vec3,
    pitch: f32,
    yaw: f32,
    view_scale: f32,
    delta: egui::Vec2,
) -> Option<f32> {
    let (cos_p, sin_p) = (pitch.cos(), pitch.sin());
    let (cos_y, sin_y) = (yaw.cos(), yaw.sin());
    let rx = cos_y * n.x - sin_y * n.z;
    let rz = sin_y * n.x + cos_y * n.z;
    let ry = cos_p * n.y - sin_p * rz;
    let (sx, sy) = (rx * view_scale, -ry * view_scale);
    let len2 = sx * sx + sy * sy;
    if len2 > (view_scale * 0.15).powi(2) {
        // |projected unit step| = sqrt(len2) px per world unit, so the world
        // distance along the axis is the drag's component along it ÷ that.
        Some((delta.x * sx + delta.y * sy) / len2)
    } else {
        None
    }
}

#[cfg(test)]
mod push_pull_tests {
    use super::*;

    // A straight-on-ish view: yaw 0.6 rad, pitch 0.5 rad, 20 px per world unit.
    const PITCH: f32 = 0.5;
    const YAW: f32 = 0.6;
    const SCALE: f32 = 20.0;

    // Dragging the mouse along an axis's on-screen projection must INCREASE depth
    // (and the opposite drag must decrease it) — for every origin plane. This is
    // the regression for "YZ/tilted planes push/pull backwards".
    fn screen_dir(n: Vec3) -> egui::Vec2 {
        // Reproduce the projected-axis direction the helper uses.
        let (cos_p, sin_p) = (PITCH.cos(), PITCH.sin());
        let (cos_y, sin_y) = (YAW.cos(), YAW.sin());
        let rx = cos_y * n.x - sin_y * n.z;
        let rz = sin_y * n.x + cos_y * n.z;
        let ry = cos_p * n.y - sin_p * rz;
        egui::vec2(rx * SCALE, -ry * SCALE).normalized()
    }

    #[test]
    fn drag_along_axis_increases_depth_on_every_plane() {
        for n in [Vec3::X, Vec3::Y, Vec3::Z] {
            let dir = screen_dir(n);
            let forward = extrude_depth_delta(n, PITCH, YAW, SCALE, dir * 10.0).unwrap();
            let backward = extrude_depth_delta(n, PITCH, YAW, SCALE, dir * -10.0).unwrap();
            assert!(
                forward > 0.0,
                "dragging along the projected axis must grow depth for n={n:?}, got {forward}"
            );
            assert!(
                backward < 0.0,
                "dragging against the projected axis must shrink depth for n={n:?}, got {backward}"
            );
        }
    }

    // The magnitude must be in world units: dragging |projected-axis| pixels
    // should move depth by ~1 world unit along the axis.
    #[test]
    fn drag_magnitude_is_world_units() {
        let n = Vec3::X; // YZ-plane extrude axis
        let dir = screen_dir(n);
        // Length on screen of one world unit along the axis.
        let (cos_p, sin_p) = (PITCH.cos(), PITCH.sin());
        let (cos_y, sin_y) = (YAW.cos(), YAW.sin());
        let rx = cos_y * n.x - sin_y * n.z;
        let rz = sin_y * n.x + cos_y * n.z;
        let ry = cos_p * n.y - sin_p * rz;
        let px_per_world = (rx * SCALE).hypot(-ry * SCALE);
        let d = extrude_depth_delta(n, PITCH, YAW, SCALE, dir * px_per_world).unwrap();
        assert!((d - 1.0).abs() < 1e-3, "expected ~1 world unit, got {d}");
    }

    // Sighting straight down the axis (axis parallel to the view direction) is
    // un-trackable → None, so the caller can fall back to vertical drag.
    #[test]
    fn edge_on_axis_returns_none() {
        // With yaw=0, pitch=0 the view looks along +Z, so a +Z extrude axis is
        // dead-on: its screen projection collapses.
        assert!(extrude_depth_delta(Vec3::Z, 0.0, 0.0, SCALE, egui::vec2(3.0, 4.0)).is_none());
    }
}
