use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use eframe::egui;
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
            SketchShape::Line { length, angle_deg, .. } => vec![length, angle_deg],
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
type EvalResult = (
    u64,
    Result<(Vec<(String, MockMesh)>, Vec<String>), String>,
);

struct ZeroCadApp {
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
    /// Memoized live edge fillet/chamfer preview: `(input hash, bodies)`. Like
    /// `extrude_preview_cache`, the underlying `preview_edge_mod_bodies` clones the
    /// graph and re-runs every truck boolean — far too slow to redo on every
    /// repaint while the size box is open or the handle is dragged. Recomputed only
    /// when the size/kind/target actually change. Cleared when the op ends.
    edge_mod_preview_cache: Option<(u64, Vec<(String, MockMesh)>)>,
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
    edge_mod_arc_rx: Option<std::sync::mpsc::Receiver<(u64, Result<(Vec<(String, MockMesh)>, Vec<String>), String>)>>,
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

impl ZeroCadApp {
    fn new() -> Self {
        let graph = ParametricGraph::new();
        let prefs = settings::AppSettings::load();

        Self {
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
            selected_faces: HashSet::new(),
            selected_edges: HashSet::new(),
            selected_body: HashSet::new(),
            extrude_depth: 25.0,
            extrude_mode: ExtrudeMode::NewBody,
            extrude_op: None,
            extrude_preview_cache: None,
            extrude_preview_mesh_cache: None,
            edge_mod_preview_cache: None,
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
    fn for_each_visible_variable(&self, mut f: impl FnMut(&str, f64)) {
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
    fn visible_variable_names(&self) -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        self.for_each_visible_variable(|name, _| names.push(name.to_string()));
        names.sort();
        names.dedup();
        names
    }

    /// Variable lookup map for expression evaluation. Built directly in a single
    /// allocation rather than via an intermediate `Vec`.
    fn visible_variable_map(&self) -> std::collections::HashMap<String, f64> {
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
    fn eval_dim(&self, text: &str) -> Option<f32> {
        expr::eval(text, &self.visible_variable_map())
            .ok()
            .map(|v| v as f32)
    }

    /// The semantic text palette for the currently active theme.
    fn pal(&self) -> Palette {
        if self.dark_mode {
            Palette::dark()
        } else {
            Palette::light()
        }
    }

    /// Re-run planar region detection on the active sketch curves.
    /// Trims selected indices that no longer correspond to a region.
    fn recompute_sketch_regions(&mut self) {
        self.detected_regions = detect_regions(&self.sketch_curves);
        let n = self.detected_regions.len();
        self.selected_region_indices.retain(|i| *i < n);
    }

    /// Reset everything related to the in-progress sketch.
    fn reset_sketch_state(&mut self) {
        self.sketch_curves = SketchCurves::new();
        self.sketch_shapes.clear();
        self.sketch_corner_mods.clear();
        self.pending_corners.clear();
        self.detected_regions.clear();
        self.selected_region_indices.clear();
        self.cancel_in_progress_shape();
    }

    /// Rebuild the live sketch geometry from the parametric `sketch_shapes` (plus
    /// any committed fillet/chamfer corner mods AND the uncommitted pending ones)
    /// against the current variables, then re-detect regions. Folding the pending
    /// corners in here is what gives the Fillet/Chamfer tool its live preview:
    /// every staged corner shows rounded/beveled at the current radius before the
    /// user commits. Called after any change to the shape list, the pending set,
    /// or the radius text.
    fn rebuild_active_sketch_curves(&mut self) {
        let vars = self.graph.variable_map();
        let mut mods = self.sketch_corner_mods.clone();
        mods.extend(self.pending_corner_mods());
        self.sketch_curves = zerocad_core::effective_curves(
            &SketchCurves::new(),
            &self.sketch_shapes,
            &mods,
            &vars,
        );
        self.recompute_sketch_regions();
    }

    /// Build the current radius/setback `Dimension` from the toolbar text (a
    /// number or a variable expression).
    fn corner_radius_dim(&self) -> Dimension {
        let text = self.corner_radius_text.clone();
        let value = self.eval_dim(&text).unwrap_or(5.0).max(0.0);
        if zerocad_core::expr::references_variable(&text) {
            Dimension {
                value,
                expr: Some(text.trim().to_string()),
            }
        } else {
            Dimension::literal(value)
        }
    }

    /// The uncommitted corner mods: one per pending corner, all sharing the
    /// current toolbar radius and the active tool's kind. Empty unless the
    /// Fillet/Chamfer tool is armed.
    fn pending_corner_mods(&self) -> Vec<CornerMod> {
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return Vec::new();
        };
        if self.pending_corners.is_empty() {
            return Vec::new();
        }
        let radius = self.corner_radius_dim();
        self.pending_corners
            .iter()
            .map(|&at| CornerMod {
                at,
                radius: radius.clone(),
                kind,
            })
            .collect()
    }

    /// Stage the sketch corner nearest `at` for a fillet/chamfer. It previews
    /// immediately (live) at the current radius; nothing is committed until the
    /// user presses Enter / clicks OK. The core snaps `at` to the actual corner
    /// vertex, so clicking near a corner is enough.
    fn stage_corner_at(&mut self, at: (f32, f32), kind: CornerKind) {
        if self.sketch_curves.segments.is_empty() {
            self.status_msg =
                "Draw straight edges first, then fillet/chamfer a corner.".to_string();
            return;
        }
        self.pending_corners.push(at);
        self.rebuild_active_sketch_curves();
        let noun = match kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        self.status_msg = format!(
            "{} previewing {} corner(s) — adjust R, click more, then Enter / OK to apply.",
            noun,
            self.pending_corners.len()
        );
    }

    /// Commit the staged fillet/chamfer corners into the sketch, capturing the
    /// current radius on each. Rebuilds the live curves (an identity rebuild,
    /// since the geometry already previewed the same mods).
    fn commit_pending_corners(&mut self) {
        if self.pending_corners.is_empty() {
            return;
        }
        let mods = self.pending_corner_mods();
        let n = mods.len();
        self.sketch_corner_mods.extend(mods);
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        self.status_msg = format!("Applied to {} corner(s).", n);
    }

    /// Drop the staged (uncommitted) fillet/chamfer corners and rebuild so the
    /// preview disappears. Returns true if anything was pending.
    fn clear_pending_corners(&mut self) -> bool {
        if self.pending_corners.is_empty() {
            return false;
        }
        self.pending_corners.clear();
        self.rebuild_active_sketch_curves();
        true
    }

    /// The sharp corner nearest `at` and its **interior bisector** (unit, in
    /// sketch coords), computed from the un-rounded geometry. Used to place and
    /// orient the 2D radius drag handle. `None` for a straight/degenerate corner.
    fn corner_bisector(&self, at: (f32, f32)) -> Option<((f32, f32), (f32, f32))> {
        let vars = self.graph.variable_map();
        // Geometry without ANY corner mods, so the pending corner is still sharp.
        let sharp =
            zerocad_core::effective_curves(&SketchCurves::new(), &self.sketch_shapes, &[], &vars);

        // Nearest segment endpoint = the corner vertex.
        let mut best: Option<((f32, f32), f32)> = None;
        for s in &sharp.segments {
            for v in [s.a, s.b] {
                let d = (v.0 - at.0).hypot(v.1 - at.1);
                if best.map_or(true, |(_, bd)| d < bd) {
                    best = Some((v, d));
                }
            }
        }
        let (v, _) = best?;

        // Unit directions of the (up to two) segments leaving that vertex.
        let mut dirs: Vec<(f32, f32)> = Vec::new();
        for s in &sharp.segments {
            let other = if (s.a.0 - v.0).hypot(s.a.1 - v.1) < 1.0e-3 {
                Some(s.b)
            } else if (s.b.0 - v.0).hypot(s.b.1 - v.1) < 1.0e-3 {
                Some(s.a)
            } else {
                None
            };
            if let Some(o) = other {
                let (dx, dy) = (o.0 - v.0, o.1 - v.1);
                let l = dx.hypot(dy);
                if l > 1.0e-4 {
                    dirs.push((dx / l, dy / l));
                }
            }
            if dirs.len() == 2 {
                break;
            }
        }
        if dirs.len() < 2 {
            return None;
        }
        let (bx, by) = (dirs[0].0 + dirs[1].0, dirs[0].1 + dirs[1].1);
        let bl = (bx * bx + by * by).sqrt();
        if bl < 1.0e-4 {
            return None; // 180° "corner" — no bisector
        }
        Some((v, (bx / bl, by / bl)))
    }

    /// Fusion-style floating radius/setback box for the 2D Fillet/Chamfer tool,
    /// anchored on the staged corner (or the live cursor) via `corner_dim_pos`.
    /// Edits the same `corner_radius_text` the toolbar shows; while corners are
    /// staged, every keystroke re-previews them live. Variables/expressions are
    /// accepted via the shared autocomplete.
    fn show_corner_radius_box(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode {
            return;
        }
        let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) else {
            return;
        };
        let Some(pos) = self.corner_dim_pos else {
            return;
        };
        let label = match kind {
            CornerKind::Fillet => "R",
            CornerKind::Chamfer => "D",
        };
        let unit_suffix = self.current_unit.suffix();
        let var_names = self.visible_variable_names();
        let mut ac = self.autocomplete.take();
        let mut changed = false;

        egui::Area::new(egui::Id::new("corner_radius_inline"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(3.0)
                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(170, 180, 190)))
                    .shadow(egui::epaint::Shadow {
                        extrusion: 8.0,
                        color: egui::Color32::from_black_alpha(35),
                    })
                    .inner_margin(egui::Margin::symmetric(8.0, 5.0))
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(label)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                            ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                            ui.style_mut().visuals.widgets.inactive.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.widgets.hovered.bg_stroke = egui::Stroke::NONE;
                            ui.style_mut().visuals.selection.bg_fill =
                                egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);
                            let field_id = egui::Id::new("corner_radius_field");
                            let outcome = crate::expr::autocomplete_field(
                                ui,
                                field_id,
                                &mut self.corner_radius_text,
                                50.0,
                                true,
                                false,
                                false,
                                &var_names,
                                &mut ac,
                            );
                            if outcome.response.changed() {
                                changed = true;
                            }
                            ui.label(
                                egui::RichText::new(unit_suffix)
                                    .size(12.0)
                                    .color(egui::Color32::from_rgb(110, 110, 110)),
                            );
                        });
                    });
            });
        self.autocomplete = ac;
        // Live: editing the size re-previews the staged corners.
        if changed && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }

    /// The Fusion-style drag manipulator for the 2D Fillet/Chamfer radius: a
    /// handle on the staged corner's bisector, joined to the corner by a guide
    /// line. Dragging it along that axis grows/shrinks the radius live, in sync
    /// with the size box and toolbar field. Mirrors the 3D edge handle, in the
    /// sketch plane.
    fn drag_corner_radius_handle(&mut self, ctx: &egui::Context) {
        if !self.is_sketch_mode || self.active_tool.and_then(|t| t.corner_kind()).is_none() {
            return;
        }
        let Some((corner, hpos, axis)) = self.corner_handle else {
            return;
        };
        let len2 = axis.length_sq();
        let r = 7.0;
        let mut dragged = false;

        egui::Area::new(egui::Id::new("corner_radius_handle"))
            .order(egui::Order::Foreground)
            .fixed_pos(hpos - egui::vec2(r, r))
            .show(ctx, |ui| {
                ui.set_clip_rect(ctx.screen_rect());
                let (_rect, resp) =
                    ui.allocate_exact_size(egui::vec2(r * 2.0, r * 2.0), egui::Sense::drag());
                let painter = ui.painter();
                let active = resp.hovered() || resp.dragged();
                let accent = if active {
                    egui::Color32::from_rgb(0, 120, 215)
                } else {
                    egui::Color32::from_rgb(255, 140, 0)
                };
                painter.line_segment([corner, hpos], egui::Stroke::new(1.5, accent));
                painter.circle_filled(hpos, r, accent);
                painter.circle_stroke(hpos, r, egui::Stroke::new(1.5, egui::Color32::WHITE));

                if resp.dragged() && len2 > 1.0e-6 {
                    let d = resp.drag_delta();
                    let delta_mm = (d.x * axis.x + d.y * axis.y) / len2;
                    let cur = self.eval_dim(&self.corner_radius_text).unwrap_or(5.0);
                    let next = (cur + delta_mm).clamp(0.1, 1000.0);
                    self.corner_radius_text = format!("{:.2}", next);
                    dragged = true;
                }
                resp.on_hover_cursor(egui::CursorIcon::ResizeHorizontal);
            });

        // Re-preview the staged corners only when the handle actually moved.
        if dragged && !self.pending_corners.is_empty() {
            self.rebuild_active_sketch_curves();
        }
    }

    /// The selected body **edges** of a single body, as `(node_id, [edge_index,…])`,
    /// or `None` when no edge is selected. With edges of more than one body
    /// selected, only the first body's edges are returned (a fillet/chamfer feature
    /// targets one body); non-edge picks (faces, points) are ignored. Gates the 3D
    /// fillet/chamfer affordance and drives a multi-edge fillet.
    fn selected_body_edges(&self) -> Option<(String, Vec<u32>)> {
        let mut edges: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(nid, pick)| match pick {
                BodyPick::Edge(e) => Some((nid.clone(), *e)),
                _ => None,
            })
            .collect();
        // Deterministic: pick the lowest-id body, then its edges in id order, so the
        // primary (anchor) edge and the apply order are stable across frames.
        edges.sort();
        let node = edges.first()?.0.clone();
        let ids: Vec<u32> = edges
            .into_iter()
            .filter(|(n, _)| *n == node)
            .map(|(_, e)| e)
            .collect();
        Some((node, ids))
    }

    /// Read a body edge's world-space geometry (endpoints + the two adjacent
    /// face normals) straight from its wireframe, packaged for an [`EdgeRef`].
    ///
    /// `e` is a topological **edge group** id (see [`BodyPick::Edge`]): the chord
    /// segments of one whole edge. The endpoints returned are the chain's two free
    /// ends — for a straight edge that's its own two corners; for a multi-chord
    /// fillet arc, the arc's ends.
    fn edge_ref_from(&self, node_id: &str, e: u32) -> Option<EdgeRef> {
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
        let seg_count = mesh.edge_indices.len() / 2;

        // Gather the group's chord segments. A legacy mesh without grouping treats
        // `e` as a single raw segment index.
        let segs: Vec<usize> = if mesh.edge_groups.is_empty() {
            if (e as usize) < seg_count {
                vec![e as usize]
            } else {
                return None;
            }
        } else {
            (0..seg_count)
                .filter(|&s| mesh.edge_groups.get(s).copied() == Some(e))
                .collect()
        };
        let &first = segs.first()?;

        let vpos = |seg: usize, which: usize| -> [f32; 3] {
            let vi = mesh.edge_indices[seg * 2 + which] as usize * 3;
            [
                mesh.edge_vertices[vi],
                mesh.edge_vertices[vi + 1],
                mesh.edge_vertices[vi + 2],
            ]
        };
        // A welded endpoint touched by exactly one of the group's chords is a free
        // end of the chain. Two of them bound the edge; a closed loop has none, so
        // fall back to the first chord's endpoints.
        let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
            let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
            (q(p[0]), q(p[1]), q(p[2]))
        };
        let mut uses: HashMap<(i64, i64, i64), (u32, [f32; 3])> = HashMap::new();
        for &s in &segs {
            for w in 0..2 {
                let p = vpos(s, w);
                uses.entry(qkey(p)).or_insert((0, p)).0 += 1;
            }
        }
        let mut ends: Vec<[f32; 3]> = uses
            .values()
            .filter(|(c, _)| *c == 1)
            .map(|(_, p)| *p)
            .collect();
        // Deterministic order so the fillet's speculative precompute key is stable.
        ends.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let (p0, p1) = if ends.len() >= 2 {
            (ends[0], ends[1])
        } else {
            (vpos(first, 0), vpos(first, 1))
        };

        // Adjacent face normals from the first chord (constant along a straight
        // edge; representative for an arc). Absent on legacy meshes → can't orient
        // a cutter, so bail (the user sees the action do nothing).
        let fo = first * 6;
        if mesh.edge_face_normals.len() < fo + 6 {
            return None;
        }
        let n1 = [
            mesh.edge_face_normals[fo],
            mesh.edge_face_normals[fo + 1],
            mesh.edge_face_normals[fo + 2],
        ];
        let n2 = [
            mesh.edge_face_normals[fo + 3],
            mesh.edge_face_normals[fo + 4],
            mesh.edge_face_normals[fo + 5],
        ];
        Some(EdgeRef { p0, p1, n1, n2 })
    }

    /// Display name for the next 3D fillet/chamfer (Fillet_1, Chamfer_2, …).
    fn next_edge_mod_name(&self, kind: CornerKind) -> String {
        let prefix = match kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        let n = self
            .graph
            .graph
            .node_indices()
            .filter(|&i| matches!(self.graph.graph[i].feature, FeatureType::EdgeMod { kind: k, .. } if k == kind))
            .count()
            + 1;
        format!("{}_{}", prefix, n)
    }

    /// Discard the half-drawn shape (placed points + dimension dialog) without
    /// touching the curves already committed to the sketch.
    fn cancel_in_progress_shape(&mut self) {
        self.sketch_temp_start = None;
        self.sketch_points.clear();
        self.dim_input = None;
        self.dim_screen_positions.clear();
    }

    /// Allocate a fresh unique id suffix and bump the counter.
    fn next_id(&mut self) -> usize {
        let id = self.id_counter;
        self.id_counter += 1;
        id
    }

    /// True when the viewport is locked to the 2D drawing plane (sketching).
    fn is_planar_view(&self) -> bool {
        self.is_sketch_mode
    }

    /// Animate the camera back to the pre-sketch 3D state.
    fn restore_camera(&mut self, ctx: &egui::Context) {
        self.camera_anim_active = true;
        self.camera_anim_start_pitch = self.camera_pitch;
        self.camera_anim_start_yaw = self.camera_yaw;
        self.camera_anim_target_pitch = self.pre_sketch_pitch;
        self.camera_anim_target_yaw = self.pre_sketch_yaw;
        self.camera_anim_start_time = ctx.input(|i| i.time);
        self.is_perspective = self.pre_sketch_perspective;
    }

    /// The selected region indices belonging to one sketch.
    fn selected_regions_for(&self, sketch_id: &str) -> HashSet<usize> {
        self.selected_faces
            .iter()
            .filter(|(sid, _)| sid == sketch_id)
            .map(|(_, ri)| *ri)
            .collect()
    }

    /// The selected edge indices belonging to one sketch.
    fn selected_edges_for(&self, sketch_id: &str) -> HashSet<usize> {
        self.selected_edges
            .iter()
            .filter(|(sid, _)| sid == sketch_id)
            .map(|(_, ei)| *ei)
            .collect()
    }

    /// Pick the body element under `click`, in priority vertex > edge > face.
    /// `proj` maps world (x,y,z) to (screen_x, screen_y, depth) — larger depth is
    /// nearer the camera. The `sin/cos` are the camera angles, used to cull
    /// back-facing triangles so only visible faces are pickable. Returns the
    /// body node id and which element was hit.
    fn pick_body_element(
        &self,
        click: egui::Pos2,
        proj: &dyn Fn(f32, f32, f32) -> (f32, f32, f32),
        sin_p: f32,
        cos_p: f32,
        sin_y: f32,
        cos_y: f32,
    ) -> Option<(String, BodyPick)> {
        const VERT_TOL_PX: f32 = 7.0;
        const EDGE_TOL_PX: f32 = 6.0;

        let mut best_vertex: Option<(String, u32, f32)> = None; // (node, vert, px)
        let mut best_edge: Option<(String, u32, f32)> = None; // (node, edge GROUP, px)
        let mut best_face: Option<(String, u32, f32)> = None; // (node, face, depth)

        let faces_camera = |n: (f32, f32, f32)| -> bool {
            let rz_n = sin_y * n.0 + cos_y * n.2;
            sin_p * n.1 + cos_p * rz_n > 0.0
        };
        // 2D point-in-triangle via consistent winding sign.
        let point_in_tri = |p: egui::Pos2, a: egui::Pos2, b: egui::Pos2, c: egui::Pos2| -> bool {
            let s = |u: egui::Pos2, v: egui::Pos2, w: egui::Pos2| {
                (v.x - u.x) * (w.y - u.y) - (v.y - u.y) * (w.x - u.x)
            };
            let d1 = s(a, b, p);
            let d2 = s(b, c, p);
            let d3 = s(c, a, p);
            let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
            let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
            !(has_neg && has_pos)
        };

        for (node_id, mesh) in &self.body_meshes {
            if self.hidden_nodes.contains(node_id) {
                continue;
            }

            // Vertices (corners of the wireframe).
            let vcount = mesh.edge_vertices.len() / 3;
            for v in 0..vcount {
                let p = proj(
                    mesh.edge_vertices[v * 3],
                    mesh.edge_vertices[v * 3 + 1],
                    mesh.edge_vertices[v * 3 + 2],
                );
                let d = (egui::pos2(p.0, p.1) - click).length();
                if d < VERT_TOL_PX && best_vertex.as_ref().map_or(true, |b| d < b.2) {
                    best_vertex = Some((node_id.clone(), v as u32, d));
                }
            }

            // Edges (wireframe segments).
            let ecount = mesh.edge_indices.len() / 2;
            for e in 0..ecount {
                let i0 = mesh.edge_indices[e * 2] as usize * 3;
                let i1 = mesh.edge_indices[e * 2 + 1] as usize * 3;
                let a = proj(
                    mesh.edge_vertices[i0],
                    mesh.edge_vertices[i0 + 1],
                    mesh.edge_vertices[i0 + 2],
                );
                let b = proj(
                    mesh.edge_vertices[i1],
                    mesh.edge_vertices[i1 + 1],
                    mesh.edge_vertices[i1 + 2],
                );
                let d = dist_point_to_segment(click, egui::pos2(a.0, a.1), egui::pos2(b.0, b.1));
                if d < EDGE_TOL_PX && best_edge.as_ref().map_or(true, |b| d < b.2) {
                    // Map the hit chord to its topological edge group, so the whole
                    // curve (a fillet arc, a circular rim) selects as one. Legacy
                    // meshes without grouping fall back to the raw segment index.
                    let g = mesh.edge_groups.get(e).copied().unwrap_or(e as u32);
                    best_edge = Some((node_id.clone(), g, d));
                }
            }

            // Faces (front-facing triangles under the cursor; nearest wins).
            let tcount = mesh.indices.len() / 3;
            for t in 0..tcount {
                let i0 = mesh.indices[t * 3] as usize * 6;
                let i1 = mesh.indices[t * 3 + 1] as usize * 6;
                let i2 = mesh.indices[t * 3 + 2] as usize * 6;
                let normal = (
                    mesh.vertices[i0 + 3],
                    mesh.vertices[i0 + 4],
                    mesh.vertices[i0 + 5],
                );
                if !faces_camera(normal) {
                    continue;
                }
                let p0 = proj(
                    mesh.vertices[i0],
                    mesh.vertices[i0 + 1],
                    mesh.vertices[i0 + 2],
                );
                let p1 = proj(
                    mesh.vertices[i1],
                    mesh.vertices[i1 + 1],
                    mesh.vertices[i1 + 2],
                );
                let p2 = proj(
                    mesh.vertices[i2],
                    mesh.vertices[i2 + 1],
                    mesh.vertices[i2 + 2],
                );
                if point_in_tri(
                    click,
                    egui::pos2(p0.0, p0.1),
                    egui::pos2(p1.0, p1.1),
                    egui::pos2(p2.0, p2.1),
                ) {
                    let depth = (p0.2 + p1.2 + p2.2) / 3.0;
                    if best_face.as_ref().map_or(true, |b| depth > b.2) {
                        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
                        best_face = Some((node_id.clone(), fid, depth));
                    }
                }
            }
        }

        if let Some((n, v, _)) = best_vertex {
            Some((n, BodyPick::Vertex(v)))
        } else if let Some((n, e, _)) = best_edge {
            Some((n, BodyPick::Edge(e)))
        } else if let Some((n, f, _)) = best_face {
            Some((n, BodyPick::Face(f)))
        } else {
            None
        }
    }

    /// A short human label for a sketch plane, by its normal.
    fn cs_label(cs: &CoordinateSystem) -> &'static str {
        let n = cs.n;
        let near = |a: f32, b: f32| (a - b).abs() < 1e-3;
        let on_origin =
            cs.origin.x.abs() < 1e-3 && cs.origin.y.abs() < 1e-3 && cs.origin.z.abs() < 1e-3;
        let axis = if near(n.x.abs(), 1.0) {
            "Right (YZ)"
        } else if near(n.y.abs(), 1.0) {
            "Top (XZ)"
        } else if near(n.z.abs(), 1.0) {
            "Front (XY)"
        } else {
            "Face"
        };
        if on_origin {
            axis
        } else {
            "Face"
        }
    }

    /// Build a sketch coordinate system from a body face: origin at the face
    /// centroid, normal = the face's outward normal, with in-plane axes derived
    /// so `u × v == n`. Returns `None` if the face/body isn't found.
    fn face_cs(&self, node_id: &str, fid: u32) -> Option<CoordinateSystem> {
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
        let ntris = mesh.indices.len() / 3;
        let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
        let (mut nx, mut ny, mut nz) = (0.0f32, 0.0f32, 0.0f32);
        let mut count = 0.0f32;
        for t in 0..ntris {
            if mesh.face_ids.get(t).copied() != Some(fid) {
                continue;
            }
            for k in 0..3 {
                let i = mesh.indices[t * 3 + k] as usize * 6;
                cx += mesh.vertices[i];
                cy += mesh.vertices[i + 1];
                cz += mesh.vertices[i + 2];
                count += 1.0;
            }
            let i0 = mesh.indices[t * 3] as usize * 6;
            nx += mesh.vertices[i0 + 3];
            ny += mesh.vertices[i0 + 4];
            nz += mesh.vertices[i0 + 5];
        }
        if count == 0.0 {
            return None;
        }
        let origin = Vec3::new(cx / count, cy / count, cz / count);
        let n = Vec3::new(nx, ny, nz).normalize();
        // In-plane axes: u perpendicular to both world-up and n (fall back to
        // world-X if the face is horizontal), v completes the right-handed frame.
        let mut u = Vec3::Y.cross(n);
        if u.length() < 1e-4 {
            u = Vec3::X.cross(n);
        }
        let u = u.normalize();
        let v = n.cross(u).normalize();
        Some(CoordinateSystem::new(origin, u, v))
    }

    /// Enter sketch mode on coordinate system `cs`: save the current camera,
    /// animate to look straight at the plane, switch to orthographic, and clear
    /// any in-progress sketch. `now` is the current input time (for the anim).
    fn begin_sketch_on(&mut self, cs: CoordinateSystem, now: f64) {
        self.pre_sketch_pitch = self.camera_pitch;
        self.pre_sketch_yaw = self.camera_yaw;
        self.pre_sketch_perspective = self.is_perspective;

        let (target_pitch, target_yaw) = Self::camera_look_at_normal(cs.n);
        self.camera_anim_active = true;
        self.camera_anim_start_pitch = self.camera_pitch;
        self.camera_anim_start_yaw = self.camera_yaw;
        self.camera_anim_target_pitch = target_pitch;
        self.camera_anim_target_yaw = target_yaw;
        self.camera_anim_start_time = now;

        self.active_sketch_cs = cs;
        self.is_plane_selection_mode = false;
        self.is_sketch_mode = true;
        self.reset_sketch_state();
        self.is_perspective = false;
        self.selected_body.clear();
    }

    /// Camera (pitch, yaw) that looks straight at a plane with outward normal
    /// `n` (the normal points toward the camera). Reproduces the XY/XZ/YZ locks
    /// for axis-aligned normals and works for any orientation.
    fn camera_look_at_normal(n: Vec3) -> (f32, f32) {
        let yaw = n.x.atan2(n.z);
        let horiz = (n.x * n.x + n.z * n.z).sqrt();
        let pitch = n.y.atan2(horiz);
        (pitch, yaw)
    }

    /// Map a screen point to (u, v) coordinates on `cs`'s plane by intersecting
    /// the click ray with the plane. Assumes the orthographic projection used
    /// while sketching, so the inverse is exact and what-you-draw lands under
    /// the cursor for ANY plane orientation.
    fn screen_to_sketch(
        &self,
        screen: egui::Pos2,
        rect: egui::Rect,
        cs: &CoordinateSystem,
    ) -> (f32, f32) {
        let center_x = rect.center().x + self.camera_pan.x;
        let center_y = rect.center().y + self.camera_pan.y;
        let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
        let (sp, cp) = (self.camera_pitch.sin(), self.camera_pitch.cos());
        let (sy, cy) = (self.camera_yaw.sin(), self.camera_yaw.cos());

        // Camera-space click coords; depth `d` is free along the view ray.
        let rx = (screen.x - center_x) / scale;
        let ry = (center_y - screen.y) / scale;

        // World point P(d) = A + B*d (inverse of the ortho rotation).
        let (y_a, y_b) = (cp * ry, sp);
        let (rz_a, rz_b) = (-sp * ry, cp);
        let (x_a, x_b) = (cy * rx + sy * rz_a, sy * rz_b);
        let (z_a, z_b) = (-sy * rx + cy * rz_a, cy * rz_b);

        let n = cs.n;
        let o = cs.origin;
        // Solve (A + B d - o)·n = 0 for d.
        let bn = x_b * n.x + y_b * n.y + z_b * n.z;
        let d = if bn.abs() < 1e-6 {
            0.0
        } else {
            ((o.x - x_a) * n.x + (o.y - y_a) * n.y + (o.z - z_a) * n.z) / bn
        };
        let rel = Vec3::new(
            x_a + x_b * d - o.x,
            y_a + y_b * d - o.y,
            z_a + z_b * d - o.z,
        );
        (rel.dot(cs.u), rel.dot(cs.v))
    }

    /// Snap a raw sketch-plane point. With Shift held, returns it unchanged
    /// (free placement). Otherwise it prefers, in order: a nearby endpoint /
    /// circle-centre / segment-midpoint, then the nearest point on a segment,
    /// then a fine 0.2-unit grid. `scale` is screen-pixels-per-unit so the snap
    /// radius stays a constant on-screen distance.
    fn snap_sketch_point(&self, raw: (f32, f32), scale: f32, shift: bool) -> (f32, f32) {
        if shift {
            return raw;
        }
        let tol = 9.0 / scale.max(1e-4); // ~9 px in world units
        let dist2 = |a: (f32, f32), b: (f32, f32)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);

        // 1. Snap points: endpoints, midpoints, circle centres.
        let mut best_pt: Option<((f32, f32), f32)> = None;
        let mut consider = |p: (f32, f32)| {
            let d = dist2(p, raw);
            if d < tol * tol && best_pt.map_or(true, |(_, bd)| d < bd) {
                best_pt = Some((p, d));
            }
        };
        for s in &self.sketch_curves.segments {
            consider(s.a);
            consider(s.b);
            consider(((s.a.0 + s.b.0) * 0.5, (s.a.1 + s.b.1) * 0.5));
        }
        for c in &self.sketch_curves.circles {
            consider(c.center);
        }
        if let Some((p, _)) = best_pt {
            return p;
        }

        // 2. Snap to the nearest point on a segment.
        let mut best_line: Option<((f32, f32), f32)> = None;
        for s in &self.sketch_curves.segments {
            let proj = project_point_on_segment(raw, s.a, s.b);
            let d = dist2(proj, raw);
            if d < tol * tol && best_line.map_or(true, |(_, bd)| d < bd) {
                best_line = Some((proj, d));
            }
        }
        if let Some((p, _)) = best_line {
            return p;
        }

        // 3. Fine grid snap (0.2 units).
        ((raw.0 * 5.0).round() / 5.0, (raw.1 * 5.0).round() / 5.0)
    }

    /// Refresh the live (unlocked, untyped) dimension fields from the current
    /// cursor position so the dialog shows the value the cursor would produce.
    /// Only the 2-point tools carry inline dimensions; 3-point tools have none.
    fn update_dim_live(&mut self, start: (f32, f32), cursor: (f32, f32)) {
        let Some(tool) = self.active_tool else {
            return;
        };
        let Some(dim) = self.dim_input.as_mut() else {
            return;
        };
        let dx = cursor.0 - start.0;
        let dy = cursor.1 - start.1;
        let live: Vec<f32> = match tool {
            SketchTool::Rectangle => vec![dx.abs(), dy.abs()],
            SketchTool::RectangleCenter => vec![2.0 * dx.abs(), 2.0 * dy.abs()],
            SketchTool::Circle => vec![2.0 * (dx * dx + dy * dy).sqrt()],
            SketchTool::Line => vec![(dx * dx + dy * dy).sqrt(), dy.atan2(dx).to_degrees()],
            // 3-point tools draw without inline dimension fields.
            _ => vec![],
        };
        for (i, f) in dim.fields.iter_mut().enumerate() {
            if !f.locked && !f.edited {
                if let Some(v) = live.get(i) {
                    f.value = format!("{:.2}", v);
                }
            }
        }
    }

    /// Build a parametric [`Dimension`] for dimension field `i`: it captures the
    /// raw expression text when it references a variable (so the dimension
    /// follows that variable), else a plain literal. `fallback` (the
    /// cursor-derived value) is used when the field is empty or invalid.
    fn dim_param(&self, i: usize, fallback: f32) -> Dimension {
        let text = self
            .dim_input
            .as_ref()
            .and_then(|d| d.fields.get(i))
            .map(|f| f.value.clone());
        match text {
            Some(t) if zerocad_core::expr::references_variable(&t) => Dimension {
                value: self.eval_dim(&t).unwrap_or(fallback),
                expr: Some(t.trim().to_string()),
            },
            Some(t) => Dimension {
                value: self.eval_dim(&t).unwrap_or(fallback),
                expr: None,
            },
            None => Dimension::literal(fallback),
        }
    }

    /// Baked geometry for the point-driven tools (rotated rectangle, 3-point
    /// circle, ellipses), which have no dimension fields to bind to variables.
    fn raw_curves_from_points(
        &self,
        tool: SketchTool,
        p0: (f32, f32),
        p1: (f32, f32),
        last: (f32, f32),
    ) -> SketchCurves {
        let mut sc = SketchCurves::new();
        match tool {
            SketchTool::RectangleThreePoint => {
                // p0→p1 is one edge; the third point sets the perpendicular height.
                let (bx, by) = (p1.0 - p0.0, p1.1 - p0.1);
                let blen = (bx * bx + by * by).sqrt();
                if blen > 1e-4 {
                    let (ux, uy) = (bx / blen, by / blen);
                    let (px, py) = (-uy, ux); // unit perpendicular
                    let h = (last.0 - p1.0) * px + (last.1 - p1.1) * py;
                    let c2 = (p1.0 + px * h, p1.1 + py * h);
                    let c3 = (p0.0 + px * h, p0.1 + py * h);
                    sc.add_line(p0, p1);
                    sc.add_line(p1, c2);
                    sc.add_line(c2, c3);
                    sc.add_line(c3, p0);
                }
            }
            SketchTool::ThreePointCircle => {
                if let Some((c, r)) = circumcircle(p0, p1, last) {
                    sc.add_circle(c, r);
                }
            }
            SketchTool::Ellipse => {
                // p0 = center, p1 = major-axis endpoint, last = minor extent.
                let major = (p1.0 - p0.0, p1.1 - p0.1);
                let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
                if rx > 1e-4 {
                    let (pxu, pyu) = (-major.1 / rx, major.0 / rx);
                    let ry = ((last.0 - p0.0) * pxu + (last.1 - p0.1) * pyu).abs();
                    sc.add_ellipse(p0, major, ry.max(0.01));
                }
            }
            SketchTool::ThreePointEllipse => {
                // p0,p1 = major-axis diameter endpoints; last = minor extent.
                let c = ((p0.0 + p1.0) * 0.5, (p0.1 + p1.1) * 0.5);
                let major = ((p1.0 - p0.0) * 0.5, (p1.1 - p0.1) * 0.5);
                let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
                if rx > 1e-4 {
                    let (pxu, pyu) = (-major.1 / rx, major.0 / rx);
                    let ry = ((last.0 - c.0) * pxu + (last.1 - c.1) * pyu).abs();
                    sc.add_ellipse(c, major, ry.max(0.01));
                }
            }
            _ => {}
        }
        sc
    }

    /// Build the **parametric record** for the in-progress shape from the placed
    /// points + `last`. Dimensioned 2-point tools capture their dimension
    /// expressions (so they follow variables); point-driven tools are baked into
    /// [`SketchShape::Raw`].
    fn shape_record_from_points(&self, last: (f32, f32)) -> Option<SketchShape> {
        let tool = self.active_tool?;
        let &p0 = self.sketch_points.first()?;
        let p1 = self.sketch_points.get(1).copied().unwrap_or(last);
        let dx = last.0 - p0.0;
        let dy = last.1 - p0.1;
        let shape = match tool {
            SketchTool::Line => SketchShape::Line {
                start: p0,
                length: self.dim_param(0, (dx * dx + dy * dy).sqrt()),
                angle_deg: self.dim_param(1, dy.atan2(dx).to_degrees()),
            },
            SketchTool::Rectangle => SketchShape::Rectangle {
                origin: p0,
                sx: if dx < 0.0 { -1.0 } else { 1.0 },
                sy: if dy < 0.0 { -1.0 } else { 1.0 },
                w: self.dim_param(0, dx.abs()),
                h: self.dim_param(1, dy.abs()),
                from_center: false,
            },
            SketchTool::RectangleCenter => SketchShape::Rectangle {
                origin: p0,
                sx: 1.0,
                sy: 1.0,
                w: self.dim_param(0, 2.0 * dx.abs()),
                h: self.dim_param(1, 2.0 * dy.abs()),
                from_center: true,
            },
            SketchTool::Circle => SketchShape::Circle {
                center: p0,
                diameter: self.dim_param(0, 2.0 * (dx * dx + dy * dy).sqrt()),
            },
            SketchTool::RectangleThreePoint
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse => SketchShape::Raw {
                curves: self.raw_curves_from_points(tool, p0, p1, last),
            },
            // Fillet/Chamfer modify existing corners; they don't create shapes.
            SketchTool::Fillet | SketchTool::Chamfer => return None,
        };
        Some(shape)
    }

    /// The in-progress shape resolved to [`SketchCurves`] — the parametric record
    /// built against the current variables. Single source of truth for the live
    /// preview and the committed geometry, so they can never diverge.
    fn shape_from_points(&self, last: (f32, f32)) -> SketchCurves {
        match self.shape_record_from_points(last) {
            Some(shape) => shape.build(&self.graph.variable_map()),
            None => SketchCurves::new(),
        }
    }

    /// Commit the in-progress shape: append its parametric record to the sketch,
    /// then rebuild the live curves from the shape list. Clears the drawing state.
    fn finalize_shape(&mut self, last: (f32, f32)) {
        if self.sketch_points.is_empty() {
            return;
        }
        if let Some(shape) = self.shape_record_from_points(last) {
            self.sketch_shapes.push(shape);
        }
        self.rebuild_active_sketch_curves();
        self.cancel_in_progress_shape();
        self.status_msg = "Shape added — click to start another.".to_string();
    }

    /// Render one row in the feature tree. Returns what the user did (selecting
    /// is handled inline). `hidden` controls the eye icon.
    fn feature_tree_row(
        &mut self,
        ui: &mut egui::Ui,
        id: &str,
        name: &str,
        hidden: bool,
        is_var_set: bool,
    ) -> RowAction {
        let mut action = RowAction::None;
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

            if id != "origin" {
                let eye_color = if hidden {
                    egui::Color32::from_rgb(148, 163, 184) // muted slate for hidden
                } else {
                    self.pal().text_body
                };
                let icon = if hidden { icons::Icon::EyeClosed } else { icons::Icon::EyeOpen };
                let eye_btn = icon.icon_button(
                    ui,
                    egui::Color32::TRANSPARENT,
                    egui::Color32::from_rgb(226, 232, 240),
                    eye_color,
                );
                if eye_btn
                    .on_hover_text(if hidden {
                        "Show node in 3D View"
                    } else {
                        "Hide node from 3D View"
                    })
                    .clicked()
                {
                    action = RowAction::ToggleVisibility;
                }
            } else {
                // Spacer to align with eye button
                ui.add_space(24.0);
            }

            let is_selected = self.selected_node_id.as_deref() == Some(id);

            // Inline rename: a text field replaces the label for the node being
            // renamed. Commits on Enter / click-away, cancels on Escape.
            if self.renaming_node.as_deref() == Some(id) {
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.rename_buffer)
                        .desired_width(f32::INFINITY)
                        .font(egui::FontId::proportional(13.0)),
                );
                if self.rename_focus_pending {
                    resp.request_focus();
                    self.rename_focus_pending = false;
                }
                let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
                if escaped {
                    self.renaming_node = None;
                } else if resp.lost_focus() {
                    let new_name = self.rename_buffer.trim().to_string();
                    if !new_name.is_empty() {
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == id {
                                self.graph.graph[idx].name = new_name.clone();
                                break;
                            }
                        }
                    }
                    self.renaming_node = None;
                }
                return; // skip the normal label + context menu this frame
            }

            let label_color = if is_selected {
                egui::Color32::from_rgb(29, 78, 216) // deep blue-700
            } else if hidden {
                self.pal().text_faint // muted slate-400
            } else {
                self.pal().text_strong // dark slate-800
            };

            let rich_text = egui::RichText::new(name)
                .color(label_color)
                .size(13.0);

            let rich_text = if is_selected {
                rich_text.strong()
            } else {
                rich_text
            };

            let response = ui.selectable_label(is_selected, rich_text);
            if response.double_clicked() {
                // Double-click starts an inline rename.
                self.renaming_node = Some(id.to_string());
                self.rename_buffer = name.to_string();
                self.rename_focus_pending = true;
            } else if response.clicked() {
                self.selected_node_id = Some(id.to_string());
                log::info!("Selected browser node: {}", id);
            }

            response.context_menu(|ui| {
                let rename_btn = icons::Icon::Sketch.labeled_button(
                    ui,
                    "Rename",
                    egui::Color32::from_rgb(248, 250, 252),
                    egui::Color32::from_rgb(241, 245, 249),
                    self.pal().text_body,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)),
                );
                if rename_btn.clicked() {
                    self.renaming_node = Some(id.to_string());
                    self.rename_buffer = name.to_string();
                    self.rename_focus_pending = true;
                    ui.close_menu();
                }
                if is_var_set {
                    let add_btn = icons::Icon::Sketch.labeled_button(
                        ui,
                        "Add Variable",
                        egui::Color32::from_rgb(239, 246, 255),
                        egui::Color32::from_rgb(219, 234, 254),
                        egui::Color32::from_rgb(29, 78, 216),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(191, 219, 254)),
                    );
                    if add_btn.clicked() {
                        action = RowAction::AddVariable;
                        ui.close_menu();
                        log::info!("Requested add variable to set: {}", id);
                    }
                }
                if id != "origin" {
                    let del_btn = icons::Icon::Trash.labeled_button(
                        ui,
                        "Delete Feature",
                        egui::Color32::from_rgb(254, 242, 242),
                        egui::Color32::from_rgb(254, 226, 226),
                        egui::Color32::from_rgb(185, 28, 28),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(252, 165, 165)),
                    );
                    if del_btn.clicked() {
                        action = RowAction::Delete;
                        ui.close_menu();
                        log::info!("Requested delete of browser node: {}", id);
                    }
                }
            });
        });
        action
    }

    /// Total `(vertices, triangles)` across a body-mesh list. Vertices are 6
    /// floats (pos + normal); indices are 3 per triangle.
    fn mesh_totals(meshes: &[(String, MockMesh)]) -> (usize, usize) {
        meshes.iter().fold((0, 0), |(v, t), (_, m)| {
            (v + m.vertices.len() / 6, t + m.indices.len() / 3)
        })
    }

    /// Count graph features matching `pred`, then add one — the 1-based index
    /// for the next feature of that kind. Shared by the `next_*_name` helpers.
    fn next_feature_index(&self, pred: impl Fn(&FeatureType) -> bool) -> usize {
        self.graph
            .graph
            .node_indices()
            .filter(|&i| pred(&self.graph.graph[i].feature))
            .count()
            + 1
    }

    /// Display name for the next sketch (Sketch_1, Sketch_2, …).
    fn next_sketch_name(&self) -> String {
        let n = self.next_feature_index(|f| matches!(f, FeatureType::Sketch { .. }));
        format!("Sketch_{}", n)
    }

    /// Display name for the next body (Body_1, Body_2, …) across all solid types.
    fn next_body_name(&self) -> String {
        let n = self.next_feature_index(|f| {
            matches!(
                f,
                FeatureType::Box { .. }
                    | FeatureType::Cylinder { .. }
                    | FeatureType::Extrude {
                        mode: ExtrudeMode::NewBody,
                        ..
                    }
            )
        });
        format!("Body_{}", n)
    }

    /// Display name for an extrude that modifies existing bodies rather than
    /// owning a standalone body.
    fn next_operation_name(&self, mode: ExtrudeMode) -> String {
        let prefix = match mode {
            ExtrudeMode::NewBody => return self.next_body_name(),
            ExtrudeMode::Join => "Join",
            ExtrudeMode::Cut => "Cut",
        };
        let n = self
            .next_feature_index(|f| matches!(f, FeatureType::Extrude { mode: m, .. } if *m == mode));
        format!("{}_{}", prefix, n)
    }

    /// Display name for the next variable set (VariableSet_1, VariableSet_2, …).
    fn next_variable_set_name(&self) -> String {
        let n = self.next_feature_index(|f| matches!(f, FeatureType::VariableSet { .. }));
        format!("VariableSet_{}", n)
    }

    /// Recalculates the geometry after a parametric history change (skipping
    /// hidden bodies).
    /// Snapshot the current `ParametricGraph` onto the undo stack (capped at 50)
    /// and clear the redo stack. Call before any destructive graph mutation.
    pub(crate) fn push_undo(&mut self) {
        if let Ok(snap) = serde_json::to_string(&self.graph) {
            if self.undo_stack.len() >= 50 {
                self.undo_stack.remove(0);
            }
            self.undo_stack.push(snap);
            self.redo_stack.clear();
        }
    }

    /// Restore the previous graph snapshot (Ctrl+Z).
    pub(crate) fn undo(&mut self) {
        if let Some(snap) = self.undo_stack.pop() {
            if let Ok(current) = serde_json::to_string(&self.graph) {
                self.redo_stack.push(current);
            }
            if let Ok(graph) = serde_json::from_str::<zerocad_core::ParametricGraph>(&snap) {
                self.graph = graph;
                self.selected_node_id = None;
                self.selected_faces.clear();
                self.selected_body.clear();
                self.extrude_op = None;
                self.edge_mod_op = None;
                self.reevaluate_geometry();
                self.status_msg = "Undo.".to_string();
            }
        } else {
            self.status_msg = "Nothing to undo.".to_string();
        }
    }

    /// Reapply the previously undone change (Ctrl+Y / Ctrl+Shift+Z).
    pub(crate) fn redo(&mut self) {
        if let Some(snap) = self.redo_stack.pop() {
            if let Ok(current) = serde_json::to_string(&self.graph) {
                if self.undo_stack.len() >= 50 {
                    self.undo_stack.remove(0);
                }
                self.undo_stack.push(current);
            }
            if let Ok(graph) = serde_json::from_str::<zerocad_core::ParametricGraph>(&snap) {
                self.graph = graph;
                self.selected_node_id = None;
                self.selected_faces.clear();
                self.selected_body.clear();
                self.extrude_op = None;
                self.edge_mod_op = None;
                self.reevaluate_geometry();
                self.status_msg = "Redo.".to_string();
            }
        } else {
            self.status_msg = "Nothing to redo.".to_string();
        }
    }

    /// Replace the model with a fresh empty design (undoable).
    pub(crate) fn new_design(&mut self) {
        log::info!("Creating new empty model.");
        self.push_undo();
        self.graph = ParametricGraph::new();
        self.doc_created_unix = None;
        self.body_meshes = Vec::new();
        self.mesh_stats = (0, 0);
        self.selected_node_id = None;
        self.reset_sketch_state();
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_body.clear();
        self.status_msg = "New blank design created.".to_string();
    }

    /// Open the in-app save dialog. The dialog presents a project title, format
    /// dropdown, recent folders, and a browse button.
    pub(crate) fn open_save_dialog(&mut self) {
        // Default directory: parent of last saved/opened project, else the
        // user's home / documents folder.
        let default_dir = self
            .recent_files
            .entries
            .first()
            .and_then(|e| e.path.parent().map(|p| p.to_path_buf()))
            .or_else(|| {
                std::env::var_os("USERPROFILE")
                    .or_else(|| std::env::var_os("HOME"))
                    .map(PathBuf::from)
            })
            .unwrap_or_else(|| PathBuf::from("."));

        // Default title: the stem of the most-recent project, or "Untitled".
        let default_title = self
            .recent_files
            .entries
            .first()
            .and_then(|e| e.path.file_stem().map(|s| s.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "Untitled".to_string());

        self.save_dialog = Some(SaveDialogState {
            project_title: default_title,
            save_format: SaveFormat::ZcadLightweight,
            save_dir: default_dir,
        });
    }

    /// Execute the save using the current save-dialog parameters.
    fn do_save(&mut self) {
        let state = match self.save_dialog.take() {
            Some(s) => s,
            None => return,
        };

        // If an arc-fillet refine is still in flight, finish it synchronously so
        // the embedded thumbnail and mesh cache persist the final geometry, not
        // the faceted draft.
        if self.eval_pending {
            self.reevaluate_geometry_blocking();
        }

        let ext = state.save_format.extension();
        let file_name = format!("{}.{ext}", state.project_title);
        let path = state.save_dir.join(&file_name);
        let embed_mesh = state.save_format == SaveFormat::ZcadFull;

        // A PNG preview rendered from the current bodies, embedded so the file
        // carries its own thumbnail (portable across machines).
        let thumbnail_png = if self.body_meshes.is_empty() {
            None
        } else {
            let (w, h, rgba) = thumbnail::render_thumbnail(&self.body_meshes, 256);
            thumbnail::encode_png(w, h, &rgba)
        };

        // For the mesh cache, exclude hidden bodies so they stay hidden on open.
        let visible_bodies: Vec<(String, MockMesh)> = self
            .body_meshes
            .iter()
            .filter(|(id, _)| !self.hidden_nodes.contains(id))
            .cloned()
            .collect();

        let doc = zerocad_core::ZcadDocument {
            graph: &self.graph,
            thumbnail_png,
            mesh_cache: if embed_mesh {
                Some(&visible_bodies)
            } else {
                None
            },
            units: self.current_unit,
            bbox: Self::bodies_bbox(&self.body_meshes),
            created_unix: self.doc_created_unix,
            hidden_nodes: self.hidden_nodes.clone(),
        };

        let bytes = match zerocad_core::write_zcad(&doc) {
            Ok(b) => b,
            Err(e) => {
                self.status_msg = format!("Save failed: {e}");
                return;
            }
        };
        match std::fs::write(&path, bytes) {
            Ok(()) => {
                log::info!("Design saved to {:?}", path);
                let how = if embed_mesh { "" } else { " (lightweight)" };
                self.status_msg = format!("Design saved to {}{how}", path.display());
                self.remember_project(&path);
            }
            Err(e) => self.status_msg = format!("Save failed: {e}"),
        }
    }

    /// Render the in-app save dialog as a centered modal overlay.
    fn show_save_dialog(&mut self, ctx: &egui::Context) {
        let is_open = self.save_dialog.is_some();
        if !is_open {
            return;
        }

        // Semi-transparent backdrop.
        egui::Area::new(egui::Id::new("save_dialog_backdrop"))
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                ui.painter().rect_filled(
                    screen,
                    0.0,
                    egui::Color32::from_black_alpha(120),
                );
                // Consume clicks on the backdrop so they don't fall through.
                ui.allocate_rect(screen, egui::Sense::click());
            });

        let mut close = false;
        let mut do_save = false;

        egui::Window::new("Save Design")
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .collapsible(false)
            .resizable(false)
            .min_width(420.0)
            .show(ctx, |ui| {
                let state = self.save_dialog.as_mut().unwrap();

                ui.add_space(4.0);

                // --- Project Title ---
                ui.horizontal(|ui| {
                    ui.label("Project Title:");
                    ui.text_edit_singleline(&mut state.project_title);
                });

                ui.add_space(6.0);

                // --- File Format ---
                ui.horizontal(|ui| {
                    ui.label("File Format:");
                    egui::ComboBox::from_id_source("save_format")
                        .selected_text(state.save_format.label())
                        .show_ui(ui, |ui: &mut egui::Ui| {
                            ui.selectable_value(
                                &mut state.save_format,
                                SaveFormat::ZcadLightweight,
                                SaveFormat::ZcadLightweight.label(),
                            );
                            ui.selectable_value(
                                &mut state.save_format,
                                SaveFormat::ZcadFull,
                                SaveFormat::ZcadFull.label(),
                            );
                        });
                });

                ui.add_space(6.0);

                // --- Save Location ---
                ui.horizontal(|ui| {
                    ui.label("Save to:");
                    let display = state.save_dir.display().to_string();
                    ui.add(
                        egui::TextEdit::singleline(&mut display.clone())
                            .desired_width(260.0)
                            .interactive(false),
                    );
                    if ui.button("Browse…").clicked() {
                        if let Some(dir) = rfd::FileDialog::new()
                            .set_title("Choose save folder")
                            .set_directory(&state.save_dir)
                            .pick_folder()
                        {
                            state.save_dir = dir;
                        }
                    }
                });

                ui.add_space(6.0);

                // --- Recent Folders ---
                let folders = self.recent_files.recent_folders();
                if !folders.is_empty() {
                    ui.label("Recent Folders:");
                    let state = self.save_dialog.as_mut().unwrap();
                    egui::ScrollArea::vertical()
                        .max_height(100.0)
                        .show(ui, |ui| {
                            for folder in &folders {
                                let label = folder.display().to_string();
                                let selected = *folder == state.save_dir;
                                if ui
                                    .selectable_label(selected, &label)
                                    .clicked()
                                {
                                    state.save_dir = folder.clone();
                                }
                            }
                        });
                    ui.add_space(6.0);
                }

                // --- Full path preview ---
                let state = self.save_dialog.as_ref().unwrap();
                let full_path = state.save_dir.join(format!("{}.{}", state.project_title, state.save_format.extension()));
                ui.horizontal(|ui| {
                    ui.label("File:");
                    ui.monospace(full_path.display().to_string());
                });

                ui.add_space(8.0);

                // --- Buttons ---
                ui.horizontal(|ui| {
                    if ui.button("Save").clicked() {
                        do_save = true;
                    }
                    if ui.button("Cancel").clicked() {
                        close = true;
                    }
                });
            });

        if do_save {
            self.do_save();
        } else if close {
            self.save_dialog = None;
        }
    }

    /// Axis-aligned bounding box `[min_x, min_y, min_z, max_x, max_y, max_z]` of
    /// every body's vertices (interleaved `[x,y,z,nx,ny,nz]`), or all-zero when
    /// there is no geometry.
    fn bodies_bbox(bodies: &[(String, MockMesh)]) -> [f32; 6] {
        let mut lo = [f32::MAX; 3];
        let mut hi = [f32::MIN; 3];
        let mut any = false;
        for (_, m) in bodies {
            for v in m.vertices.chunks_exact(6) {
                for k in 0..3 {
                    lo[k] = lo[k].min(v[k]);
                    hi[k] = hi[k].max(v[k]);
                }
                any = true;
            }
        }
        if !any {
            return [0.0; 6];
        }
        [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]]
    }

    /// Record `path` in the recent-projects list and (re)bake a thumbnail of the
    /// currently-evaluated bodies for the onboarding screen. Called after a
    /// successful save/open, when `body_meshes` reflects `path`'s model.
    fn remember_project(&mut self, path: &Path) {
        self.recent_files.record(path);
        if !self.body_meshes.is_empty() {
            let (w, h, rgba) = thumbnail::render_thumbnail(&self.body_meshes, 256);
            settings::save_thumb(path, w, h, &rgba);
        }
        // Drop any stale cached texture so the next onboarding render reloads it.
        self.onboarding_textures.remove(path);
    }

    /// Fetch (uploading once, then caching) the egui texture for a project's
    /// cached thumbnail, or `None` if there's no `.thumb` for it yet.
    fn thumb_texture(&mut self, ctx: &egui::Context, path: &Path) -> Option<egui::TextureHandle> {
        if let Some(t) = self.onboarding_textures.get(path) {
            return Some(t.clone());
        }
        let (w, h, rgba) = settings::load_thumb(path)?;
        let image = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
        let tex = ctx.load_texture(
            format!("thumb_{}", path.display()),
            image,
            egui::TextureOptions::LINEAR,
        );
        self.onboarding_textures.insert(path.to_path_buf(), tex.clone());
        Some(tex)
    }

    /// The centered Welcome modal: New / Open / Recent. Drawn over a dimmed,
    /// click-swallowing backdrop so the workspace beneath is inert. A no-op
    /// unless `onboarding_visible`. Esc, the Close button, or choosing any action
    /// dismisses it (without touching the persisted "show on startup" preference,
    /// which the footer checkbox edits separately).
    fn draw_onboarding(&mut self, ctx: &egui::Context) {
        if !self.onboarding_visible {
            return;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.onboarding_visible = false;
            return;
        }

        let pal = self.pal();

        // Dim + swallow input to the workspace behind the card (Middle sits above
        // the Background panels but below the Foreground card). Clicking the
        // backdrop — anywhere outside the card, since the card sits on top and
        // consumes clicks over its own rect — dismisses onboarding so the user
        // drops straight into the (blank) workspace and can start modeling.
        let backdrop_clicked = egui::Area::new(egui::Id::new("onboarding_dim"))
            .order(egui::Order::Middle)
            .fixed_pos(egui::Pos2::ZERO)
            .show(ctx, |ui| {
                let screen = ctx.screen_rect();
                let resp = ui.allocate_rect(screen, egui::Sense::click_and_drag());
                ui.painter()
                    .rect_filled(screen, 0.0, egui::Color32::from_black_alpha(120));
                resp.clicked()
            })
            .inner;

        // Top 5 recents, snapshotted (path + display name) so the draw closure
        // borrows locals, not `self.recent_files`. Textures are pre-loaded for
        // the same reason (uploading mutably borrows `self`).
        let recents: Vec<(PathBuf, String)> = self
            .recent_files
            .entries
            .iter()
            .take(5)
            .map(|e| {
                let name = e
                    .path
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| e.path.to_string_lossy().into_owned());
                (e.path.clone(), name)
            })
            .collect();
        let textures: Vec<Option<egui::TextureHandle>> = recents
            .iter()
            .map(|(p, _)| self.thumb_texture(ctx, p))
            .collect();

        let mut do_new = false;
        let mut do_open = false;
        let mut open_recent: Option<PathBuf> = None;
        let mut close = false;

        // The window defaults to Order::Middle and is registered after the dim
        // Area (also Middle), so it draws on top of the dim — and both sit above
        // the Background-order workspace panels.
        egui::Window::new("onboarding_window")
            .title_bar(false)
            .collapsible(false)
            .resizable(false)
            .movable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .fixed_size(egui::vec2(560.0, 430.0))
            .frame(egui::Frame::window(&ctx.style()).inner_margin(egui::Margin::same(22.0)))
            .show(ctx, |ui| {
                // Brand title.
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 1.0;
                    ui.label(
                        egui::RichText::new("Welcome to ")
                            .size(22.0)
                            .color(pal.text_strong),
                    );
                    ui.label(
                        egui::RichText::new("Zero")
                            .strong()
                            .size(22.0)
                            .color(pal.text_strong),
                    );
                    ui.label(
                        egui::RichText::new("CAD")
                            .strong()
                            .size(22.0)
                            .color(egui::Color32::from_rgb(37, 99, 235)),
                    );
                });
                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new("Start a new project, open one, or pick up where you left off.")
                        .size(13.0)
                        .color(pal.text_muted),
                );
                ui.add_space(16.0);

                // New / Open.
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 10.0;
                    if icons::Icon::New
                        .labeled_button(
                            ui,
                            "New Project",
                            egui::Color32::from_rgb(37, 99, 235),
                            egui::Color32::from_rgb(29, 78, 216),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        )
                        .clicked()
                    {
                        do_new = true;
                    }
                    if icons::Icon::Folder
                        .labeled_button(
                            ui,
                            "Open Project",
                            egui::Color32::from_rgb(241, 245, 249),
                            egui::Color32::from_rgb(226, 232, 240),
                            pal.text_body,
                            egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                        )
                        .clicked()
                    {
                        do_open = true;
                    }
                });

                ui.add_space(16.0);
                ui.separator();
                ui.add_space(8.0);
                ui.label(
                    egui::RichText::new("Recent")
                        .strong()
                        .size(14.0)
                        .color(pal.text_strong),
                );
                ui.add_space(8.0);

                if recents.is_empty() {
                    ui.add_space(20.0);
                    ui.vertical_centered(|ui| {
                        ui.label(
                            egui::RichText::new("No recent projects yet.")
                                .size(13.0)
                                .color(pal.text_muted),
                        );
                        ui.label(
                            egui::RichText::new("Saved and opened projects will appear here.")
                                .size(12.0)
                                .color(pal.text_muted),
                        );
                    });
                } else {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 10.0;
                        for ((path, name), tex) in recents.iter().zip(textures.iter()) {
                            if Self::recent_card(ui, &pal, name, tex.as_ref())
                                .on_hover_text(path.to_string_lossy())
                                .clicked()
                            {
                                open_recent = Some(path.clone());
                            }
                        }
                    });
                }

                ui.add_space(16.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_onboarding, "Show on startup");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add(egui::Button::new("Close").min_size(egui::vec2(80.0, 28.0)))
                            .clicked()
                        {
                            close = true;
                        }
                    });
                });
            });

        // Apply deferred actions (outside the borrow of the draw closure).
        if do_new {
            self.new_design();
            self.onboarding_visible = false;
        } else if do_open {
            self.open_design();
            self.onboarding_visible = false;
        } else if let Some(path) = open_recent {
            self.load_design_from(path);
            self.onboarding_visible = false;
        } else if close || backdrop_clicked {
            // Close button, or a click anywhere off the card: dismiss and let the
            // user model in the current (blank-on-startup) workspace.
            self.onboarding_visible = false;
        }
    }

    /// One clickable Recent card: thumbnail (or placeholder) above the project
    /// name, with a hover highlight. Returns its click response.
    fn recent_card(
        ui: &mut egui::Ui,
        pal: &Palette,
        name: &str,
        tex: Option<&egui::TextureHandle>,
    ) -> egui::Response {
        const CARD: egui::Vec2 = egui::vec2(96.0, 120.0);
        const IMG: f32 = 84.0;
        let (rect, resp) = ui.allocate_exact_size(CARD, egui::Sense::click());
        if !ui.is_rect_visible(rect) {
            return resp;
        }
        let painter = ui.painter();
        let hovered = resp.hovered();
        painter.rect(
            rect,
            6.0,
            if hovered {
                egui::Color32::from_rgb(226, 232, 240)
            } else {
                egui::Color32::TRANSPARENT
            },
            if hovered {
                egui::Stroke::new(1.0, egui::Color32::from_rgb(37, 99, 235))
            } else {
                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225))
            },
        );
        let img_rect = egui::Rect::from_min_size(
            egui::pos2(rect.center().x - IMG * 0.5, rect.top() + 6.0),
            egui::vec2(IMG, IMG),
        );
        match tex {
            Some(t) => {
                painter.image(
                    t.id(),
                    img_rect,
                    egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                    egui::Color32::WHITE,
                );
            }
            None => {
                painter.rect_filled(img_rect, 4.0, egui::Color32::from_rgb(238, 241, 245));
                let icon = egui::Rect::from_center_size(img_rect.center(), egui::vec2(28.0, 28.0));
                icons::Icon::Sketch.draw(painter, icon, pal.text_muted);
            }
        }
        // Project name, truncated to fit.
        let label: String = if name.chars().count() > 13 {
            format!("{}…", name.chars().take(12).collect::<String>())
        } else {
            name.to_string()
        };
        painter.text(
            egui::pos2(rect.center().x, img_rect.bottom() + 8.0),
            egui::Align2::CENTER_TOP,
            label,
            egui::FontId::proportional(12.0),
            pal.text_body,
        );
        resp
    }

    /// Prompt for a `.zcad` file and load it, replacing the current model.
    pub(crate) fn open_design(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Open ZeroCAD Design")
            .add_filter("ZeroCAD Design", &["zcad", "zcadh"])
            .pick_file()
        else {
            return;
        };
        self.load_design_from(path);
    }

    /// Load the `.zcad` file at `path`, replacing the current model (undoable).
    /// Selection / preview state is reset to match the new graph. Shared by the
    /// Open dialog and the onboarding Recent list.
    pub(crate) fn load_design_from(&mut self, path: PathBuf) {
        let bytes = match std::fs::read(&path) {
            Ok(b) => b,
            Err(e) => {
                self.status_msg = format!("Could not read file: {e}");
                return;
            }
        };
        let loaded = match zerocad_core::read_zcad(&bytes) {
            Ok(l) => l,
            Err(e) => {
                self.status_msg = format!("Load failed: {e}");
                return;
            }
        };

        self.push_undo();
        self.graph = loaded.graph;
        // Preserve the original creation time for legacy/unknown files we stamp anew.
        self.doc_created_unix = (!loaded.was_legacy_json && loaded.metadata.created_unix != 0)
            .then_some(loaded.metadata.created_unix);
        // Restore the document's display unit (binary files only; legacy JSON has
        // no metadata, so we keep the user's current preference).
        if !loaded.was_legacy_json {
            self.current_unit = loaded.metadata.units;
        }
        self.selected_node_id = None;
        self.selected_faces.clear();
        self.selected_edges.clear();
        self.selected_body.clear();
        self.hidden_nodes = loaded.hidden_nodes;
        self.extrude_op = None;
        self.edge_mod_op = None;

        // Show the embedded geometry cache immediately (instant open). It's only
        // present when fresh (its hash matched the loaded graph), so it's safe to
        // display; `reevaluate_geometry` then swaps in freshly-computed bodies.
        if let Some(cache) = loaded.mesh_cache {
            self.body_meshes = cache;
            self.mesh_stats = Self::mesh_totals(&self.body_meshes);
        }
        // Seed the onboarding thumbnail cache from the file's embedded preview so
        // a `.zcad` from another machine shows its real thumbnail even if it has
        // no geometry to re-render (e.g. evaluation fails).
        if let Some(png) = &loaded.thumbnail_png {
            if let Some((w, h, rgba)) = thumbnail::decode_png(png) {
                settings::save_thumb(&path, w, h, &rgba);
                self.onboarding_textures.remove(path.as_path());
            }
        }

        // Regenerate from the recipe (authoritative). On failure, the cached
        // bodies above remain on screen so the model is never lost.
        self.reevaluate_geometry();
        self.status_msg = format!("Design loaded from {}", path.display());
        self.remember_project(&path);
    }

    /// Prompt for a path and write all current bodies as one binary STL mesh.
    /// STL is a triangle soup (no history/units), so this is export-only — the
    /// editable document stays the `.zcad` JSON.
    pub(crate) fn export_stl(&mut self) {
        if self.body_meshes.is_empty() {
            self.status_msg = "Nothing to export — the model has no solid bodies.".to_string();
            return;
        }
        // Export the final arc geometry, not a faceted draft mid-refine.
        if self.eval_pending {
            self.reevaluate_geometry_blocking();
        }
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export STL")
            .add_filter("STL mesh", &["stl"])
            .save_file()
        else {
            return;
        };
        let bytes = zerocad_core::meshes_to_binary_stl(self.body_meshes.iter().map(|(_, m)| m));
        let tris = bytes.len().saturating_sub(84) / 50;
        match std::fs::write(&path, bytes) {
            Ok(()) => {
                log::info!("Exported STL to {:?} ({tris} triangles)", path);
                self.status_msg = format!("Exported {tris} triangles to {}", path.display());
            }
            Err(e) => self.status_msg = format!("STL export failed: {e}"),
        }
    }

    /// Delete the currently selected browser node (sketch/body/variable set), if
    /// any (undoable). Mirrors the per-row delete button in the document browser.
    pub(crate) fn delete_selected_node(&mut self) {
        let Some(del_id) = self.selected_node_id.clone() else {
            self.status_msg = "Nothing selected to delete.".to_string();
            return;
        };
        let target = self
            .graph
            .graph
            .node_indices()
            .find(|idx| self.graph.graph[*idx].id == del_id);
        if let Some(idx) = target {
            self.push_undo();
            self.graph.graph.remove_node(idx);
            self.selected_node_id = None;
            self.selected_faces.retain(|(sid, _)| sid != &del_id);
            self.selected_edges.retain(|(sid, _)| sid != &del_id);
            self.selected_body.retain(|(nid, _)| nid != &del_id);
            self.hidden_nodes.remove(&del_id);
            self.reevaluate_geometry();
            self.status_msg = "Deleted selection.".to_string();
        }
    }

    /// Run a keyboard-shortcut action. Single dispatch point shared by the global
    /// hotkey handler and (where relevant) menu items.
    fn run_shortcut(&mut self, action: ShortcutAction) {
        match action {
            ShortcutAction::NewDesign => self.new_design(),
            ShortcutAction::OpenDesign => self.open_design(),
            ShortcutAction::SaveDesign => self.open_save_dialog(),
            ShortcutAction::ExportStl => self.export_stl(),
            ShortcutAction::Undo => self.undo(),
            ShortcutAction::Redo => self.redo(),
            ShortcutAction::DeleteSelection => self.delete_selected_node(),
            ShortcutAction::ToggleTheme => self.dark_mode = !self.dark_mode,
            ShortcutAction::OpenSettings => self.show_preferences = true,
        }
    }

    /// Process global keyboard shortcuts, or capture a new binding when the
    /// Shortcuts settings tab is waiting for one. Called first thing each frame,
    /// before any UI, so bindings fire regardless of which panel is hovered.
    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        // Rebinding capture mode: the Shortcuts tab is waiting for a key combo.
        if let Some(action) = self.capturing_shortcut {
            // Escape cancels the capture, leaving the existing binding intact.
            if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.capturing_shortcut = None;
                return;
            }
            let captured = ctx.input(|i| {
                i.events.iter().find_map(|ev| match ev {
                    egui::Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        modifiers,
                        ..
                    } => Some((*key, *modifiers)),
                    _ => None,
                })
            });
            if let Some((key, mods)) = captured {
                self.keymap.set(action, shortcuts::Hotkey::from_event(key, mods));
                self.keymap.save();
                self.capturing_shortcut = None;
            }
            // Suppress normal dispatch while capturing so the captured combo does
            // not also trigger an action on this frame.
            return;
        }

        // Normal dispatch. Skip entirely while a widget holds keyboard focus, so
        // typing in a text field (dimensions, variable names, …) never fires a
        // command. At most one action runs per frame.
        if ctx.memory(|m| m.focus().is_some()) {
            return;
        }
        let mut fire = None;
        for &action in ShortcutAction::ALL {
            if let Some(hk) = self.keymap.get(action) {
                if hk.pressed(ctx) {
                    fire = Some(action);
                    break;
                }
            }
        }
        if let Some(action) = fire {
            self.run_shortcut(action);
        }
    }

    /// Rebuild the model. The **fast faceted draft** is computed synchronously and
    /// shown immediately, so committing a fillet (or any edit) never stalls the
    /// UI. If the model has a 3D fillet — whose true arc geometry needs the ~1s
    /// boolean — the arc result is computed on a **background thread** and swapped
    /// in when ready (see [`reevaluate_geometry_blocking`] for the rare callers
    /// that must have the final geometry before continuing).
    fn reevaluate_geometry(&mut self) {
        // Instant draft: faceted fillets + the (fast, planar) cut/join booleans.
        match self.graph.evaluate_bodies_with_warnings_draft(&self.hidden_nodes) {
            Ok((bodies, warnings)) => self.apply_eval_result(bodies, warnings),
            Err(err) => {
                self.error_msg = Some(err);
                self.status_msg = "Error: Model evaluation failed.".to_string();
                return;
            }
        }
        // Refine to the smooth single-face arc fillet off the UI thread. Skipped
        // when there's no fillet, since then the draft already *is* the final.
        if self.graph.has_arc_fillet(&self.hidden_nodes) {
            self.spawn_refine_eval();
            self.status_msg = "Smoothing fillet…".to_string();
        } else {
            self.eval_pending = false;
            self.eval_rx = None;
        }
    }

    /// Apply an evaluation result to the displayed model + status line.
    fn apply_eval_result(&mut self, bodies: Vec<(String, MockMesh)>, warnings: Vec<String>) {
        self.body_meshes = bodies;
        self.mesh_stats = Self::mesh_totals(&self.body_meshes);
        if warnings.is_empty() {
            self.error_msg = None;
            self.status_msg = "Model evaluated successfully.".to_string();
        } else {
            // Non-fatal: the model evaluated, but a boolean didn't do what the
            // user asked. Surface it instead of letting the geometry come out
            // wrong silently.
            self.status_msg = format!("Model evaluated with {} warning(s).", warnings.len());
            self.error_msg = Some(warnings.join("\n"));
        }
    }

    /// Spawn the background arc-fillet evaluation. Tagged with a generation so a
    /// later edit's job supersedes this one (stale results are dropped on
    /// arrival). The worker wakes the UI via `request_repaint` the moment it's
    /// done so the refined geometry appears without waiting for the next input.
    fn spawn_refine_eval(&mut self) {
        self.eval_gen += 1;
        let gen = self.eval_gen;
        let graph = self.graph.clone();
        let hidden = self.hidden_nodes.clone();
        let ctx = self.egui_ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        self.eval_rx = Some(rx);
        self.eval_pending = true;
        std::thread::spawn(move || {
            let result = graph.evaluate_bodies_with_warnings(&hidden);
            let _ = tx.send((gen, result));
            if let Some(ctx) = ctx {
                ctx.request_repaint();
            }
        });
    }

    /// Poll the background refine channel; apply the result if it's the current
    /// generation. Called once per frame.
    fn poll_refine_eval(&mut self) {
        let Some(rx) = self.eval_rx.as_ref() else {
            return;
        };
        match rx.try_recv() {
            Ok((gen, result)) => {
                self.eval_rx = None;
                self.eval_pending = false;
                // Drop a superseded job's result; a newer one is (or will be) in
                // flight and owns the display.
                if gen != self.eval_gen {
                    return;
                }
                match result {
                    Ok((bodies, warnings)) => self.apply_eval_result(bodies, warnings),
                    // A failed refine leaves the faceted draft on screen — still a
                    // valid model — rather than blanking it.
                    Err(err) => log::warn!("Background refine failed: {err}"),
                }
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.eval_rx = None;
                self.eval_pending = false;
            }
        }
    }

    /// Synchronous rebuild that waits for the **final** arc geometry. Use only
    /// where the result must be current before the next line runs (export, save
    /// thumbnail); interactive edits use [`reevaluate_geometry`] so they don't
    /// stall.
    fn reevaluate_geometry_blocking(&mut self) {
        // A pending background job is now obsolete — bump the generation so its
        // late result is discarded in favour of this authoritative one.
        self.eval_gen += 1;
        self.eval_rx = None;
        self.eval_pending = false;
        match self.graph.evaluate_bodies_with_warnings(&self.hidden_nodes) {
            Ok((bodies, warnings)) => self.apply_eval_result(bodies, warnings),
            Err(err) => {
                self.error_msg = Some(err);
                self.status_msg = "Error: Model evaluation failed.".to_string();
            }
        }
    }
}

impl eframe::App for ZeroCadApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keep a context handle so a background refine worker can wake the UI.
        if self.egui_ctx.is_none() {
            self.egui_ctx = Some(ctx.clone());
        }
        // Swap in any finished background arc-fillet refine.
        self.poll_refine_eval();
        // Speculatively precompute the smooth arc geometry for a settling fillet so
        // committing it is instant rather than popping in ~1s later.
        self.tick_speculative_edge_mod(ctx);

        // While the Welcome modal is up the workspace is inert, so its hotkeys
        // are suppressed (the modal reads Esc itself).
        if !self.onboarding_visible {
            self.handle_shortcuts(ctx);
        }

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

        // Welcome modal (drawn as a Foreground layer over everything below).
        self.draw_onboarding(ctx);

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
                // Easing: Smoothstep
                let t_smooth = t * t * (3.0 - 2.0 * t);
                self.camera_pitch = self.camera_anim_start_pitch
                    + (self.camera_anim_target_pitch - self.camera_anim_start_pitch) * t_smooth;
                self.camera_yaw = self.camera_anim_start_yaw
                    + (self.camera_anim_target_yaw - self.camera_anim_start_yaw) * t_smooth;
                ctx.request_repaint(); // Smooth animation repaint request
            }
        }

        // SAVE DIALOG (modal overlay, drawn before the Settings window).
        self.show_save_dialog(ctx);

        // SETTINGS WINDOW (floating, modal-style; tab rail left, content right)
        if self.show_preferences {
            let mut open = self.show_preferences;
            let strong = self.pal().text_strong;
            egui::Window::new("Settings")
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .default_width(460.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        let (rect, _) = ui.allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::hover());
                        icons::Icon::Settings.draw(ui.painter(), rect, strong);
                        ui.label(egui::RichText::new("Settings").strong().size(14.0));
                    });
                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(8.0);

                    // Fixed-height row so the vertical separator below doesn't
                    // stretch to fill the whole (auto-sized) window.
                    ui.allocate_ui_with_layout(
                        egui::vec2(444.0, 270.0),
                        egui::Layout::left_to_right(egui::Align::Min),
                        |ui| {
                        // Left rail: tab list.
                        ui.allocate_ui_with_layout(
                            egui::vec2(110.0, 270.0),
                            egui::Layout::top_down_justified(egui::Align::Min),
                            |ui| {
                                for &tab in SettingsTab::ALL {
                                    ui.selectable_value(&mut self.settings_tab, tab, tab.label());
                                }
                            },
                        );

                        ui.separator();

                        // Right pane: content for the selected tab.
                        ui.vertical(|ui| {
                            match self.settings_tab {
                                SettingsTab::General => {
                                    // --- Units ---
                                    ui.label(egui::RichText::new("Default measurement unit").strong());
                                    ui.add_space(2.0);
                                    egui::ComboBox::from_id_source("pref_unit_select")
                                        .selected_text(match self.current_unit {
                                            Unit::Millimeter => "Millimeters (mm)",
                                            Unit::Inch => "Inches (in)",
                                            Unit::Meter => "Meters (m)",
                                        })
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(
                                                &mut self.current_unit,
                                                Unit::Millimeter,
                                                "Millimeters (mm)",
                                            );
                                            ui.selectable_value(&mut self.current_unit, Unit::Inch, "Inches (in)");
                                            ui.selectable_value(&mut self.current_unit, Unit::Meter, "Meters (m)");
                                        });

                                    ui.add_space(12.0);

                                    // --- Onboarding ---
                                    ui.checkbox(&mut self.show_onboarding, "Onboarding Screen");
                                }
                                SettingsTab::Shortcuts => {
                                    ui.label(
                                        egui::RichText::new("Keyboard shortcuts").strong(),
                                    );
                                    ui.add_space(2.0);
                                    ui.weak(
                                        "Click a shortcut, then press the new combo. Esc cancels.",
                                    );
                                    ui.add_space(8.0);

                                    // Deferred mutations so the keymap isn't borrowed
                                    // mutably while the rows read it.
                                    let mut toggle_capture: Option<ShortcutAction> = None;
                                    let mut clear_action: Option<ShortcutAction> = None;

                                    egui::ScrollArea::vertical()
                                        .max_height(190.0)
                                        .show(ui, |ui| {
                                            for &action in ShortcutAction::ALL {
                                                ui.horizontal(|ui| {
                                                    ui.add_sized(
                                                        [150.0, 24.0],
                                                        egui::Label::new(action.label()),
                                                    );
                                                    let capturing =
                                                        self.capturing_shortcut == Some(action);
                                                    let text = if capturing {
                                                        "Press a key…".to_string()
                                                    } else {
                                                        self.keymap
                                                            .get(action)
                                                            .map(|h| h.label())
                                                            .unwrap_or_else(|| "Unbound".to_string())
                                                    };
                                                    let mut btn = egui::Button::new(text)
                                                        .min_size(egui::vec2(120.0, 24.0));
                                                    if capturing {
                                                        btn = btn.fill(egui::Color32::from_rgb(
                                                            0, 120, 215,
                                                        ));
                                                    }
                                                    if ui.add(btn).clicked() {
                                                        toggle_capture = Some(action);
                                                    }
                                                    if ui
                                                        .small_button("✕")
                                                        .on_hover_text("Unbind")
                                                        .clicked()
                                                    {
                                                        clear_action = Some(action);
                                                    }
                                                });
                                                ui.add_space(2.0);
                                            }
                                        });

                                    ui.add_space(10.0);
                                    if ui.button("Reset to defaults").clicked() {
                                        self.keymap.reset_to_defaults();
                                        self.keymap.save();
                                        self.capturing_shortcut = None;
                                    }

                                    // Apply the deferred row actions.
                                    if let Some(action) = toggle_capture {
                                        // Clicking the row already capturing cancels it.
                                        self.capturing_shortcut =
                                            if self.capturing_shortcut == Some(action) {
                                                None
                                            } else {
                                                Some(action)
                                            };
                                    }
                                    if let Some(action) = clear_action {
                                        self.keymap.unbind(action);
                                        self.keymap.save();
                                        if self.capturing_shortcut == Some(action) {
                                            self.capturing_shortcut = None;
                                        }
                                    }
                                }
                            }
                        });
                    });

                    ui.add_space(12.0);
                    ui.separator();
                    ui.add_space(8.0);

                    ui.horizontal(|ui| {
                        if ui
                            .add(egui::Button::new("Done").min_size(egui::vec2(80.0, 28.0)))
                            .clicked()
                        {
                            self.show_preferences = false;
                        }
                        ui.weak("Changes apply immediately.");
                    });
                    ui.add_space(4.0);
                });
            // Respect the window's close (✕) button as well as the Done button.
            if !open {
                self.show_preferences = false;
            }
        }

        // TOP PANEL: Operations Toolbar
        egui::TopBottomPanel::top("operations_toolbar")
            .exact_height(48.0)
            .show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                ui.style_mut().spacing.button_padding = egui::vec2(12.0, 7.0);
                ui.style_mut().spacing.item_spacing = egui::vec2(10.0, 10.0);

                // Premium branded logo
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing = egui::vec2(1.0, 0.0);
                    ui.label(egui::RichText::new("Zero").strong().size(18.0).color(self.pal().text_strong));
                    ui.label(egui::RichText::new("CAD").strong().size(18.0).color(egui::Color32::from_rgb(37, 99, 235)));
                });

                // File Button Tab dropdown menu
                let file_btn_id = ui.make_persistent_id("file_menu_dropdown");
                let file_btn = icons::Icon::Folder.labeled_button(
                    ui,
                    "File",
                    egui::Color32::from_rgb(241, 245, 249), // Clean slate grey
                    egui::Color32::from_rgb(226, 232, 240), // Hover
                    self.pal().text_body,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                );
                if file_btn.clicked() {
                    ui.memory_mut(|mem| mem.toggle_popup(file_btn_id));
                }
                egui::popup_below_widget::<()>(
                    ui,
                    file_btn_id,
                    &file_btn,
                    |ui| {
                        ui.set_min_width(180.0);
                        ui.style_mut().spacing.button_padding = egui::vec2(16.0, 6.0);

                        // Shortcut hint for a menu action, taken from the live keymap.
                        let hint = |app: &ZeroCadApp, action: ShortcutAction| {
                            app.keymap
                                .get(action)
                                .map(|h| h.label())
                                .unwrap_or_default()
                        };

                        if icons::Icon::New
                            .menu_button_hint(ui, "New Design", &hint(self, ShortcutAction::NewDesign))
                            .clicked()
                        {
                            ui.memory_mut(|mem| mem.close_popup());
                            self.new_design();
                        }

                        if icons::Icon::Save
                            .menu_button_hint(ui, "Save Design", &hint(self, ShortcutAction::SaveDesign))
                            .clicked()
                        {
                            ui.memory_mut(|mem| mem.close_popup());
                            self.open_save_dialog();
                        }

                        if icons::Icon::Download
                            .menu_button_hint(ui, "Open Design", &hint(self, ShortcutAction::OpenDesign))
                            .clicked()
                        {
                            ui.memory_mut(|mem| mem.close_popup());
                            self.open_design();
                        }

                        ui.separator();

                        if icons::Icon::Download
                            .menu_button_hint(ui, "Export STL", &hint(self, ShortcutAction::ExportStl))
                            .clicked()
                        {
                            ui.memory_mut(|mem| mem.close_popup());
                            self.export_stl();
                        }

                        ui.separator();

                        if icons::Icon::Settings
                            .menu_button_hint(ui, "Settings", &hint(self, ShortcutAction::OpenSettings))
                            .clicked()
                        {
                            ui.memory_mut(|mem| mem.close_popup());
                            log::info!("Opening Settings window.");
                            self.show_preferences = true;
                        }

                        ui.separator();

                        if icons::Icon::Exit.menu_button(ui, "Exit ZeroCAD").clicked() {
                            ui.memory_mut(|mem| mem.close_popup());
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    },
                );

                ui.separator();

                let active_sketching = self.is_sketch_mode || self.is_plane_selection_mode;

                // Draw Sketch / Finish Sketch CTA Button
                if active_sketching {
                    let finish_btn = icons::Icon::Check.labeled_button(
                        ui,
                        "Finish Sketch",
                        egui::Color32::from_rgb(16, 185, 129), // Emerald Green CTA
                        egui::Color32::from_rgb(5, 150, 105),  // Hover
                        egui::Color32::WHITE,
                        egui::Stroke::NONE,
                    );
                    if finish_btn.on_hover_text("Complete sketch and save as a 2D profile").clicked() {
                        log::info!("Finishing sketch — saving it as a 2D object.");
                        // Bake any still-staged fillet/chamfer corners so finishing
                        // the sketch never silently drops them.
                        self.commit_pending_corners();
                        self.recompute_sketch_regions();
                        if !self.sketch_curves.is_empty() {
                            let sketch_id = format!("sketch_{}", self.next_id());
                            let sketch_name = self.next_sketch_name();
                            log::info!(
                                "Saving sketch {} ({}) ({} curves, {} faces).",
                                sketch_id,
                                sketch_name,
                                self.sketch_curves.segments.len() + self.sketch_curves.circles.len(),
                                self.detected_regions.len(),
                            );

                            let sketch_node = FeatureNode {
                                id: sketch_id.clone(),
                                name: sketch_name,
                                feature: FeatureType::Sketch {
                                    cs: self.active_sketch_cs,
                                    curves: self.sketch_curves.clone(),
                                    shapes: self.sketch_shapes.clone(),
                                    corner_mods: self.sketch_corner_mods.clone(),
                                    on_face: self.active_sketch_on_face,
                                },
                            };

                            self.push_undo();
                            self.graph.add_feature(sketch_node);
                            self.selected_node_id = Some(sketch_id);
                            self.reset_sketch_state();
                            self.status_msg = "Sketch saved as a 2D object. Use the Extrude tool to make a body.".to_string();
                        } else {
                            self.status_msg = "Empty sketch discarded.".to_string();
                            log::warn!("Sketch discarded: nothing drawn.");
                            self.reset_sketch_state();
                        }

                        // Animate camera BACK to previous 3D state
                        log::info!("Restoring previous 3D camera state: pitch: {:.2}, yaw: {:.2}", self.pre_sketch_pitch, self.pre_sketch_yaw);
                        self.restore_camera(ctx);

                        self.is_sketch_mode = false;
                        self.is_plane_selection_mode = false;
                    }
                } else {
                    let draw_btn = icons::Icon::Sketch.labeled_button(
                        ui,
                        "Draw Sketch",
                        egui::Color32::from_rgb(241, 245, 249), // Clean slate grey
                        egui::Color32::from_rgb(226, 232, 240), // Hover
                        self.pal().text_strong,
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225)),
                    );
                    if draw_btn.on_hover_text("Enter sketch mode — sketches on the selected body face if one is selected, else pick an origin plane").clicked() {
                        // Context-aware: if exactly one body FACE is selected,
                        // sketch directly on it; otherwise open the plane picker.
                        let face_sel = if self.selected_body.len() == 1 {
                            self.selected_body.iter().next().and_then(|(nid, p)| match p {
                                BodyPick::Face(f) => Some((nid.clone(), *f)),
                                _ => None,
                            })
                        } else {
                            None
                        };

                        match face_sel.and_then(|(nid, fid)| self.face_cs(&nid, fid)) {
                            Some(cs) => {
                                log::info!("Sketching on a selected body face.");
                                let now = ui.input(|i| i.time);
                                self.active_sketch_on_face = true;
                                self.begin_sketch_on(cs, now);
                                self.status_msg =
                                    "Sketching on the selected face. Draw a profile, then Finish Sketch.".to_string();
                            }
                            None => {
                                log::info!("Entering sketch plane selection mode. Viewport remains in 3D.");
                                self.active_sketch_on_face = false;
                                self.is_plane_selection_mode = true;
                                self.is_sketch_mode = false;
                                self.reset_sketch_state();
                                self.status_msg = "Click on one of the origin planes (XY Red, XZ Green, YZ Blue) in the viewport to sketch on it.".to_string();
                            }
                        }
                    }
                }

                // EXTRUDE: select faces in the 3D viewport, then start the tool.
                if !active_sketching && self.extrude_op.is_none() {
                    ui.separator();
                    let sel = self.selected_faces.len();
                    let extrude_enabled = sel > 0;

                    if extrude_enabled {
                        let extrude_btn = icons::Icon::Extrude.labeled_button(
                            ui,
                            &format!("Extrude ({})", sel),
                            egui::Color32::from_rgb(37, 99, 235), // vibrant active blue
                            egui::Color32::from_rgb(29, 78, 216),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        );
                        if extrude_btn
                            .on_hover_text("Extrude the selected 3D face(s) into a solid body")
                            .clicked()
                        {
                            self.begin_extrude_from_selection();
                        }
                    } else {
                        // Inert (no selection): same fill on hover so it reads disabled.
                        icons::Icon::Extrude.labeled_button(
                            ui,
                            "Extrude",
                            egui::Color32::from_rgb(241, 245, 249),
                            egui::Color32::from_rgb(241, 245, 249),
                            self.pal().text_faint,
                            egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)),
                        )
                        .on_hover_text("Select one or more 3D faces first");
                    }

                    if sel > 0 {
                        let clear_sel_btn = ui.add(
                            egui::Button::new(
                                egui::RichText::new("Clear Selection")
                                    .color(egui::Color32::from_rgb(220, 38, 38)) // red text
                                    .size(12.0)
                            )
                            .fill(egui::Color32::from_rgb(254, 226, 226)) // soft red wash
                            .rounding(egui::Rounding::same(6.0))
                            .min_size(egui::vec2(90.0, 26.0))
                        );
                        if clear_sel_btn.clicked() {
                            self.selected_faces.clear();
                            self.selected_edges.clear();
                            self.selected_body.clear();
                        }
                    }
                }

                // 3D edge fillet / chamfer: shown when one or more body EDGES are
                // selected (and we're not sketching/extruding). Rounds or bevels the
                // real solid; several edges (Shift/Ctrl-click) fillet at once.
                let edge_sel = (!active_sketching
                    && self.extrude_op.is_none()
                    && self.edge_mod_op.is_none())
                .then(|| self.selected_body_edges())
                .flatten();
                if let Some((_, edge_ids)) = edge_sel {
                    let n_edges = edge_ids.len();
                    ui.separator();
                    ui.label(
                        egui::RichText::new(if n_edges > 1 {
                            format!("Modify {n_edges} Edges")
                        } else {
                            "Modify Edge".to_string()
                        })
                        .strong()
                        .size(12.0)
                        .color(self.pal().text_strong),
                    );
                    ui.add_space(4.0);
                    // One button: a left-click starts a Fillet (the default); a
                    // right-click (or the ▾) opens a flyout to pick Fillet or
                    // Chamfer — the same convention the Rectangle/Circle tools use.
                    // Either way it begins a live preview; the size is set in the
                    // floating box, and the inline dialog can still toggle the kind.
                    let popup_id = ui.make_persistent_id("edgemod_flyout");
                    ui.horizontal(|ui| {
                        let btn = icons::Icon::Fillet.labeled_button(
                            ui,
                            "Fillet  ▾",
                            egui::Color32::from_rgb(37, 99, 235),
                            egui::Color32::from_rgb(29, 78, 216),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        );
                        let btn = btn.on_hover_text(
                            "Round the edge (live preview). Right-click or ▾ to choose Chamfer.",
                        );
                        if btn.clicked() {
                            self.begin_edge_mod(CornerKind::Fillet);
                        }
                        if btn.secondary_clicked() {
                            ui.memory_mut(|m| m.toggle_popup(popup_id));
                        }
                        egui::popup_below_widget(ui, popup_id, &btn, |ui| {
                            ui.set_min_width(140.0);
                            if icons::Icon::Fillet.menu_button(ui, "Fillet").clicked() {
                                self.begin_edge_mod(CornerKind::Fillet);
                                ui.memory_mut(|m| m.close_popup());
                            }
                            if icons::Icon::Chamfer.menu_button(ui, "Chamfer").clicked() {
                                self.begin_edge_mod(CornerKind::Chamfer);
                                ui.memory_mut(|m| m.close_popup());
                            }
                        });
                    });
                    ui.label(
                        egui::RichText::new("Works best on a convex edge of a plain box/extrude.")
                            .size(10.0)
                            .color(self.pal().text_faint),
                    );
                }

                if self.is_plane_selection_mode {
                    ui.separator();
                    ui.label(
                        egui::RichText::new("🖱️ Hover & Click a 3D Plane sheet in the viewport to select your sketching plane.")
                            .color(egui::Color32::from_rgb(217, 119, 6)) // elegant warm amber text
                            .strong()
                    );
                }

                if self.is_sketch_mode {
                    ui.separator();

                    // Premium control tabs with custom vector graphics for Sketch Tools
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

                        // Helper to draw a beautiful tool button
                        let pal = self.pal();
                        let draw_tool_btn = |ui: &mut egui::Ui, is_active: bool, text: &str, icon: Option<icons::Icon>| -> egui::Response {
                            let button_color = if is_active {
                                egui::Color32::from_rgb(219, 234, 254) // Active light blue
                            } else {
                                egui::Color32::from_rgb(241, 245, 249) // Slate grey
                            };
                            let text_color = if is_active {
                                egui::Color32::from_rgb(29, 78, 216) // Solid active blue
                            } else {
                                pal.text_body // Muted slate text
                            };
                            let stroke = if is_active {
                                egui::Stroke::new(1.5, egui::Color32::from_rgb(59, 130, 246))
                            } else {
                                egui::Stroke::new(1.0, egui::Color32::from_rgb(203, 213, 225))
                            };

                            let desired_size = egui::vec2(95.0, 28.0);
                            let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click());

                            if ui.is_rect_visible(rect) {
                                let bg_fill = if response.hovered() {
                                    if is_active { egui::Color32::from_rgb(191, 219, 254) } else { egui::Color32::from_rgb(226, 232, 240) }
                                } else {
                                    button_color
                                };

                                ui.painter().rect(rect, 4.0, bg_fill, stroke);

                                // Position and paint icon
                                let icon_rect = egui::Rect::from_min_size(rect.left_top() + egui::vec2(8.0, 6.0), egui::vec2(16.0, 16.0));
                                if let Some(ic) = icon {
                                    ic.draw(ui.painter(), icon_rect, text_color);
                                } else {
                                    // Custom pointer polygon for "Select" tool
                                    let p = icon_rect.left_top();
                                    let points = vec![
                                        p + egui::vec2(3.0, 2.0),
                                        p + egui::vec2(3.0, 13.0),
                                        p + egui::vec2(6.0, 10.0),
                                        p + egui::vec2(10.0, 10.0),
                                    ];
                                    ui.painter().add(egui::Shape::convex_polygon(points, text_color, egui::Stroke::new(1.0, text_color)));
                                }

                                // Position and paint text
                                let text_pos = rect.left_top() + egui::vec2(28.0, 6.0);
                                ui.painter().text(
                                    text_pos,
                                    egui::Align2::LEFT_TOP,
                                    text,
                                    egui::FontId::proportional(12.0),
                                    text_color
                                );
                            }

                            response
                        };

                        // No explicit "Select" button: pressing Esc returns to the
                        // neutral Select state (`active_tool = None`), which lets the
                        // user pick body faces/edges/vertices without leaving the
                        // sketch — see the global Escape handler below.

                        // Line Tool (single mode, no flyout).
                        {
                            let is_active = self.active_tool == Some(SketchTool::Line);
                            let btn = draw_tool_btn(ui, is_active, "Line", Some(icons::Icon::Line));
                            if btn.on_hover_text("Draw individual line segments (L)").clicked() {
                                self.active_tool = Some(SketchTool::Line);
                                self.cancel_in_progress_shape();
                                self.clear_pending_corners();
                                log::info!("Switched to Line tool");
                            }
                        }

                        // Rectangle, Circle and the corner tool each expose a mode
                        // flyout: click the active button again (or right-click it)
                        // to choose corner/center/3-point, ellipse, or Fillet ↔
                        // Chamfer. The corner button is a single button (like the 3D
                        // edge fillet/chamfer) whose flyout switches the two kinds.
                        for (family, key, hover) in [
                            (
                                ToolFamily::Rectangle,
                                "Rectangle",
                                "Rectangle (R) — click again or right-click for modes",
                            ),
                            (
                                ToolFamily::Circle,
                                "Circle",
                                "Circle (C) — click again or right-click for ellipse / 3-point modes",
                            ),
                            (
                                ToolFamily::Corner,
                                "Fillet",
                                "Fillet / Chamfer — set the radius, then click a corner. Click again or right-click to switch kind",
                            ),
                        ] {
                            let active = self.active_tool.map_or(false, |t| t.family() == family);
                            // The button shows the icon of the active sub-mode so
                            // the user sees which variant is armed.
                            let icon = if active {
                                self.active_tool.unwrap().icon()
                            } else {
                                family.default_mode().icon()
                            };
                            // Fixed label per button, except the corner button shows
                            // the armed kind (Fillet vs Chamfer) so it's visible which
                            // is set — the two labels are short enough to fit.
                            let label = if family == ToolFamily::Corner && active {
                                self.active_tool.unwrap().label()
                            } else {
                                key
                            };
                            let btn = draw_tool_btn(ui, active, label, Some(icon))
                                .on_hover_text(hover);
                            let popup_id = ui.make_persistent_id(("tool_flyout", key));

                            if btn.clicked() {
                                if active {
                                    // Re-clicking the armed tool opens the flyout.
                                    ui.memory_mut(|m| m.toggle_popup(popup_id));
                                } else {
                                    self.active_tool = Some(family.default_mode());
                                    self.cancel_in_progress_shape();
                                    self.clear_pending_corners();
                                    ui.memory_mut(|m| m.close_popup());
                                }
                            }
                            if btn.secondary_clicked() {
                                ui.memory_mut(|m| m.open_popup(popup_id));
                            }

                            egui::popup_below_widget(ui, popup_id, &btn, |ui| {
                                ui.set_min_width(180.0);
                                for &mode in family.modes() {
                                    let selected = self.active_tool == Some(mode);
                                    let prefix = if selected { "● " } else { "   " };
                                    let row = mode.icon().menu_button(
                                        ui,
                                        &format!("{}{}", prefix, mode.label()),
                                    );
                                    if row.clicked() {
                                        self.active_tool = Some(mode);
                                        self.cancel_in_progress_shape();
                                        self.clear_pending_corners();
                                        ui.memory_mut(|m| m.close_popup());
                                        log::info!("Switched to {:?}", mode);
                                    }
                                }
                            });
                        }

                        // Radius/distance input for the active corner tool, with
                        // a unit suffix. Editing it re-previews the staged corners
                        // live. An OK button (and Enter) commits the pending set.
                        if let Some(kind) = self.active_tool.and_then(|t| t.corner_kind()) {
                            let label = match kind {
                                CornerKind::Fillet => "R:",
                                CornerKind::Chamfer => "D:",
                            };
                            ui.label(egui::RichText::new(label).size(12.0).color(self.pal().text_body));
                            let changed = ui
                                .add(
                                    egui::TextEdit::singleline(&mut self.corner_radius_text)
                                        .desired_width(46.0)
                                        .hint_text("5"),
                                )
                                .changed();
                            let unit_suffix = match self.current_unit {
                                Unit::Millimeter => "mm",
                                Unit::Inch => "in",
                                Unit::Meter => "m",
                            };
                            ui.label(
                                egui::RichText::new(unit_suffix)
                                    .size(11.0)
                                    .color(self.pal().text_faint),
                            );
                            // Live: changing the radius re-previews the staged corners.
                            if changed && !self.pending_corners.is_empty() {
                                self.rebuild_active_sketch_curves();
                            }
                            if !self.pending_corners.is_empty() {
                                let ok = ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(format!(
                                            "✓ OK ({})",
                                            self.pending_corners.len()
                                        ))
                                        .size(12.0)
                                        .color(egui::Color32::WHITE),
                                    )
                                    .fill(egui::Color32::from_rgb(34, 139, 84))
                                    .rounding(egui::Rounding::same(4.0)),
                                );
                                if ok.on_hover_text("Apply the staged corners (Enter)").clicked() {
                                    self.commit_pending_corners();
                                }
                            }
                        }
                    });

                    // Curve statistics, Undo / Clear Sketch row
                    let curve_count = self.sketch_curves.segments.len() + self.sketch_curves.circles.len();
                    if curve_count > 0 {
                        ui.separator();
                        ui.label(
                            egui::RichText::new(format!("Curves: {} · Faces: {}", curve_count, self.detected_regions.len()))
                                .color(self.pal().text_body)
                                .size(12.0)
                        );

                        let undo_btn = ui.add(
                            egui::Button::new(egui::RichText::new("↩ Undo").size(12.0))
                                .fill(egui::Color32::from_rgb(241, 245, 249))
                                .rounding(egui::Rounding::same(4.0))
                        );
                        if undo_btn.on_hover_text("Undo last drawn shape").clicked() {
                            // One undo removes a whole shape (not a single
                            // segment), then the live curves are rebuilt.
                            self.sketch_shapes.pop();
                            self.rebuild_active_sketch_curves();
                        }

                        let reset_btn = icons::Icon::Trash.labeled_button(
                            ui,
                            "Clear",
                            egui::Color32::from_rgb(254, 242, 242),
                            egui::Color32::from_rgb(254, 226, 226),
                            egui::Color32::from_rgb(185, 28, 28),
                            egui::Stroke::NONE,
                        );
                        if reset_btn.on_hover_text("Clear all curves in current sketch").clicked() {
                            self.reset_sketch_state();
                        }
                    }
                }
            });
        });

        // LEFT PANEL: History Tree & Feature Properties
        egui::SidePanel::left("history_sidebar")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |ui| {
                ui.vertical(|ui| {
                    ui.add_space(8.0);
                    ui.label(
                        egui::RichText::new("Document Browser")
                            .font(egui::FontId::proportional(14.0))
                            .strong()
                            .color(self.pal().text_strong) // Slate-900
                    );
                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(6.0);

                    // Bucket features into Sketches (2D objects) and Bodies
                    // (solids: boxes, cylinders, extrudes). The Origin is shown
                    // on its own so it can't be mistaken for either group.
                    let mut origin: Option<(String, String)> = None;
                    let mut sketches: Vec<(String, String)> = Vec::new();
                    let mut bodies: Vec<(String, String)> = Vec::new();
                    let mut operations: Vec<(String, String)> = Vec::new();
                    let mut variable_sets: Vec<(String, String)> = Vec::new();
                    for idx in self.graph.graph.node_indices() {
                        let node = &self.graph.graph[idx];
                        let entry = (node.id.clone(), node.name.clone());
                        match node.feature {
                            FeatureType::Origin => origin = Some(entry),
                            FeatureType::Sketch { .. } => sketches.push(entry),
                            FeatureType::Box { .. }
                            | FeatureType::Cylinder { .. }
                            | FeatureType::Extrude {
                                mode: ExtrudeMode::NewBody,
                                ..
                            } => bodies.push(entry),
                            FeatureType::Extrude { .. } | FeatureType::EdgeMod { .. } => {
                                operations.push(entry)
                            }
                            FeatureType::VariableSet { .. } => variable_sets.push(entry),
                        }
                    }

                    let mut id_to_delete: Option<String> = None;
                    let mut id_to_toggle: Option<String> = None;
                    let mut id_to_add_var: Option<String> = None;
                    let mut create_var_set = false;

                    egui::ScrollArea::vertical().id_source("tree_scroll").max_height(300.0).show(ui, |ui| {
                        if let Some((id, name)) = &origin {
                            let hidden = self.hidden_nodes.contains(id);
                            match self.feature_tree_row(ui, id, name, hidden, false) {
                                RowAction::Delete => id_to_delete = Some(id.clone()),
                                RowAction::ToggleVisibility => id_to_toggle = Some(id.clone()),
                                RowAction::None => {}
                                        RowAction::AddVariable => {}
                            }
                        }

                        egui::CollapsingHeader::new(
                            egui::RichText::new(format!("Sketches ({})", sketches.len()))
                                .font(egui::FontId::proportional(12.5))
                                .strong()
                                .color(self.pal().text_body) // Slate-600
                        )
                            .default_open(true)
                            .show(ui, |ui| {
                                if sketches.is_empty() {
                                    ui.weak("No sketches yet — use Draw Sketch.");
                                }
                                for (id, name) in &sketches {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => id_to_toggle = Some(id.clone()),
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                        egui::CollapsingHeader::new(
                            egui::RichText::new(format!("Bodies ({})", bodies.len()))
                                .font(egui::FontId::proportional(12.5))
                                .strong()
                                .color(self.pal().text_body) // Slate-600
                        )
                            .default_open(true)
                            .show(ui, |ui| {
                                if bodies.is_empty() {
                                    ui.weak("No bodies yet — add a primitive or Extrude a sketch.");
                                }
                                for (id, name) in &bodies {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => id_to_toggle = Some(id.clone()),
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                        egui::CollapsingHeader::new(
                            egui::RichText::new(format!("Operations ({})", operations.len()))
                                .font(egui::FontId::proportional(12.5))
                                .strong()
                                .color(self.pal().text_body) // Slate-600
                        )
                            .default_open(true)
                            .show(ui, |ui| {
                                if operations.is_empty() {
                                    ui.weak("No body operations yet.");
                                }
                                for (id, name) in &operations {
                                    let hidden = self.hidden_nodes.contains(id);
                                    match self.feature_tree_row(ui, id, name, hidden, false) {
                                        RowAction::Delete => id_to_delete = Some(id.clone()),
                                        RowAction::ToggleVisibility => id_to_toggle = Some(id.clone()),
                                        RowAction::None => {}
                                        RowAction::AddVariable => {}
                                    }
                                }
                            });

                        // Variable Sets: a section title with a "+" on the right
                        // to create a new set. Each set lives below as a row whose
                        // right-click menu can add variables.
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(format!("Variable Sets ({})", variable_sets.len()))
                                    .font(egui::FontId::proportional(12.5))
                                    .strong()
                                    .color(self.pal().text_body), // Slate-600
                            );
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui
                                    .add(egui::Button::new(egui::RichText::new("➕").size(12.0)).small())
                                    .on_hover_text("New variable set")
                                    .clicked()
                                {
                                    create_var_set = true;
                                }
                            });
                        });
                        if variable_sets.is_empty() {
                            ui.weak("No variable sets yet — click ➕ to add one.");
                        }
                        for (id, name) in &variable_sets {
                            match self.feature_tree_row(ui, id, name, false, true) {
                                RowAction::Delete => id_to_delete = Some(id.clone()),
                                RowAction::AddVariable => id_to_add_var = Some(id.clone()),
                                RowAction::ToggleVisibility => {}
                                RowAction::None => {}
                            }
                        }
                    });

                    if let Some(toggle_id) = id_to_toggle {
                        if !self.hidden_nodes.remove(&toggle_id) {
                            self.hidden_nodes.insert(toggle_id);
                        }
                        // Bodies are baked into the mesh, so re-evaluate to reflect
                        // the change; sketches just toggle in the draw pass.
                        self.reevaluate_geometry();
                    }

                    if let Some(del_id) = id_to_delete {
                        // Resolve the id to its current graph index, then remove.
                        let mut target = None;
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == del_id {
                                target = Some(idx);
                                break;
                            }
                        }
                        if let Some(idx) = target {
                            self.push_undo();
                            self.graph.graph.remove_node(idx);
                            if self.selected_node_id.as_deref() == Some(del_id.as_str()) {
                                self.selected_node_id = None;
                            }
                            self.selected_faces.retain(|(sid, _)| sid != &del_id);
                            self.selected_edges.retain(|(sid, _)| sid != &del_id);
                            self.selected_body.retain(|(nid, _)| nid != &del_id);
                            self.hidden_nodes.remove(&del_id);
                            self.reevaluate_geometry();
                        }
                    }

                    // Create a new, empty variable set and select it so the user
                    // can rename it (Label field) and start adding variables.
                    if create_var_set {
                        let id = format!("varset_{}", self.next_id());
                        let name = self.next_variable_set_name();
                        self.graph.add_feature(FeatureNode {
                            id: id.clone(),
                            name,
                            feature: FeatureType::VariableSet { variables: Vec::new() },
                        });
                        self.selected_node_id = Some(id);
                    }

                    // Append a fresh variable to the targeted set (from a row's
                    // right-click "Add Variable").
                    if let Some(set_id) = id_to_add_var {
                        let unit = self.current_unit;
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == set_id {
                                if let FeatureType::VariableSet { variables } =
                                    &mut self.graph.graph[idx].feature
                                {
                                    let n = variables.len() + 1;
                                    variables.push(Variable::new(format!("var{}", n), unit));
                                }
                                break;
                            }
                        }
                        self.selected_node_id = Some(set_id);
                    }

                    ui.add_space(15.0);
                    ui.label(
                        egui::RichText::new("Properties")
                            .font(egui::FontId::proportional(14.0))
                            .strong()
                            .color(self.pal().text_strong) // Slate-900
                    );
                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(8.0);

                    // Render dynamic sliders based on selected node's feature type
                    if let Some(ref selected_id) = self.selected_node_id {
                        let mut node_idx = None;
                        for idx in self.graph.graph.node_indices() {
                            if self.graph.graph[idx].id == *selected_id {
                                node_idx = Some(idx);
                                break;
                            }
                        }

                        if let Some(idx) = node_idx {
                            // Deferred action: extruding needs `&mut self`, but
                            // `node` holds a mutable borrow of the graph below.
                            let mut extrude_request: Option<String> = None;
                            let mut modified = false;

                            // Capture palette + unit + the variable map before
                            // borrowing the graph mutably (so the Extrude panel can
                            // show what an expression-driven depth resolves to).
                            let pal = self.pal();
                            let current_unit = self.current_unit;
                            let var_map = self.graph.variable_map();
                            let node = &mut self.graph.graph[idx];

                            // Render inside a highly visual white inspector card
                            egui::Frame::none()
                                .fill(egui::Color32::WHITE)
                                .rounding(8.0)
                                .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)))
                                .inner_margin(12.0)
                                .show(ui, |ui| {
                                    ui.vertical(|ui| {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("ID:").weak().size(11.5));
                                            ui.label(egui::RichText::new(&node.id).strong().size(12.0).color(pal.text_body));
                                        });
                                        ui.add_space(6.0);
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new("Label:").size(12.0));
                                            ui.text_edit_singleline(&mut node.name);
                                        });
                                        ui.add_space(8.0);
                                        ui.separator();
                                        ui.add_space(8.0);

                                        match &mut node.feature {
                                            FeatureType::Origin => {
                                                ui.label(
                                                    egui::RichText::new("📍 Base Origin coordinate planes space (0,0,0).")
                                                        .size(11.5)
                                                        .color(pal.text_muted)
                                                );
                                            }
                                            FeatureType::Box { w, h, d } => {
                                                ui.label(egui::RichText::new("Dimensions:").strong().size(12.0).color(pal.text_strong));
                                                ui.add_space(6.0);
                                                egui::Grid::new("box_grid").spacing(egui::vec2(10.0, 10.0)).show(ui, |ui| {
                                                    ui.label(egui::RichText::new("Width").size(12.0));
                                                    let w_resp = ui.add(egui::Slider::new(w, 5.0..=150.0).suffix(self.current_unit.suffix()));
                                                    if w_resp.changed() { modified = true; }
                                                    ui.end_row();

                                                    ui.label(egui::RichText::new("Height").size(12.0));
                                                    let h_resp = ui.add(egui::Slider::new(h, 5.0..=150.0).suffix(self.current_unit.suffix()));
                                                    if h_resp.changed() { modified = true; }
                                                    ui.end_row();

                                                    ui.label(egui::RichText::new("Depth").size(12.0));
                                                    let d_resp = ui.add(egui::Slider::new(d, 5.0..=150.0).suffix(self.current_unit.suffix()));
                                                    if d_resp.changed() { modified = true; }
                                                    ui.end_row();
                                                });
                                            }
                                            FeatureType::Cylinder { r, h } => {
                                                ui.label(egui::RichText::new("Dimensions:").strong().size(12.0).color(pal.text_strong));
                                                ui.add_space(6.0);
                                                egui::Grid::new("cyl_grid").spacing(egui::vec2(10.0, 10.0)).show(ui, |ui| {
                                                    ui.label(egui::RichText::new("Radius").size(12.0));
                                                    let r_resp = ui.add(egui::Slider::new(r, 2.0..=80.0).suffix(self.current_unit.suffix()));
                                                    if r_resp.changed() { modified = true; }
                                                    ui.end_row();

                                                    ui.label(egui::RichText::new("Height").size(12.0));
                                                    let h_resp = ui.add(egui::Slider::new(h, 5.0..=200.0).suffix(self.current_unit.suffix()));
                                                    if h_resp.changed() { modified = true; }
                                                    ui.end_row();
                                                });
                                            }
                                            FeatureType::Sketch { cs, curves, shapes, corner_mods, .. } => {
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("Plane:").size(12.0));
                                                    ui.label(egui::RichText::new(Self::cs_label(cs)).strong().size(12.0).color(egui::Color32::from_rgb(37, 99, 235)));
                                                });
                                                ui.add_space(4.0);
                                                // Resolve against the current variables so the counts
                                                // (and any extrude below) reflect variable-driven dims.
                                                let eff = zerocad_core::effective_curves(curves, shapes, corner_mods, &var_map);
                                                ui.label(egui::RichText::new(format!("Curves: {} segments, {} circles", eff.segments.len(), eff.circles.len())).size(11.5).weak());
                                                let regions = detect_regions(&eff);
                                                ui.label(egui::RichText::new(format!("Faces: {}", regions.len())).size(11.5).weak());
                                                // Surface any variable-bound dimensions so the user knows
                                                // the sketch is parametric (editing is done by redrawing).
                                                let bound = sketch_variable_dims(shapes);
                                                if !bound.is_empty() {
                                                    ui.add_space(2.0);
                                                    ui.label(egui::RichText::new(format!("🔗 Variable dims: {}", bound.join(", "))).size(11.0).color(egui::Color32::from_rgb(37, 99, 235)));
                                                }
                                                ui.add_space(8.0);

                                                let has_faces = !regions.is_empty();
                                                let extrude_btn = icons::Icon::Extrude.labeled_button(
                                                    ui,
                                                    "Extrude whole Sketch",
                                                    if has_faces { egui::Color32::from_rgb(37, 99, 235) } else { egui::Color32::from_rgb(241, 245, 249) },
                                                    if has_faces { egui::Color32::from_rgb(29, 78, 216) } else { egui::Color32::from_rgb(241, 245, 249) },
                                                    if has_faces { egui::Color32::WHITE } else { pal.text_faint },
                                                    if has_faces { egui::Stroke::NONE } else { egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)) },
                                                );
                                                if has_faces && extrude_btn.clicked() {
                                                    extrude_request = Some(node.id.clone());
                                                }
                                            }
                                            FeatureType::Extrude { depth, region_indices, mode, depth_expr } => {
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("Extrusion Depth:").size(12.0));
                                                    let d_resp = ui.add(
                                                        egui::Slider::new(depth, 1.0..=150.0)
                                                            .suffix(current_unit.suffix()),
                                                    );
                                                    if d_resp.changed() {
                                                        // Dragging sets a literal depth — drop the binding.
                                                        *depth_expr = None;
                                                        modified = true;
                                                    }
                                                });
                                                // Variable/expression binding: a depth like `width / 2`
                                                // re-evaluates whenever the variable changes. Empty clears it.
                                                ui.add_space(4.0);
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("=").size(13.0).color(pal.text_muted));
                                                    let mut buf = depth_expr.clone().unwrap_or_default();
                                                    let r = ui.add(
                                                        egui::TextEdit::singleline(&mut buf)
                                                            .hint_text("expression, e.g. width / 2")
                                                            .desired_width(150.0),
                                                    );
                                                    if r.changed() {
                                                        let t = buf.trim();
                                                        *depth_expr = if t.is_empty() { None } else { Some(t.to_string()) };
                                                        modified = true;
                                                    }
                                                });
                                                if let Some(e) = depth_expr.as_ref() {
                                                    let txt = match zerocad_core::expr::eval(e, &var_map) {
                                                        Ok(v) => format!("→ {:.2} {}", v, current_unit.suffix()),
                                                        Err(_) => "→ unresolved (check variable names)".to_string(),
                                                    };
                                                    ui.label(egui::RichText::new(txt).size(11.0).weak());
                                                }
                                                ui.add_space(6.0);
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("Operation:").size(12.0));
                                                    for (m, label) in [
                                                        (ExtrudeMode::NewBody, "New Body"),
                                                        (ExtrudeMode::Join, "Join"),
                                                        (ExtrudeMode::Cut, "Cut"),
                                                    ] {
                                                        if ui.selectable_label(*mode == m, label).clicked() && *mode != m {
                                                            *mode = m;
                                                            modified = true;
                                                        }
                                                    }
                                                });
                                                ui.add_space(6.0);
                                                if region_indices.is_empty() {
                                                    ui.label(egui::RichText::new("Regions: all detected").size(11.5).weak());
                                                } else {
                                                    ui.label(egui::RichText::new(format!("Regions: {:?}", region_indices)).size(11.5).weak());
                                                }
                                            }
                                            FeatureType::EdgeMod { dist, dist_expr, kind, .. } => {
                                                let noun = match kind {
                                                    CornerKind::Fillet => "Fillet radius:",
                                                    CornerKind::Chamfer => "Chamfer distance:",
                                                };
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new(noun).size(12.0));
                                                    let d_resp = ui.add(
                                                        egui::Slider::new(dist, 0.2..=40.0)
                                                            .suffix(current_unit.suffix()),
                                                    );
                                                    if d_resp.changed() {
                                                        *dist_expr = None; // a literal drag drops the binding
                                                        modified = true;
                                                    }
                                                });
                                                // Variable/expression binding, mirroring the extrude depth.
                                                ui.add_space(4.0);
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("=").size(13.0).color(pal.text_muted));
                                                    let mut buf = dist_expr.clone().unwrap_or_default();
                                                    let r = ui.add(
                                                        egui::TextEdit::singleline(&mut buf)
                                                            .hint_text("expression, e.g. fillet_r")
                                                            .desired_width(150.0),
                                                    );
                                                    if r.changed() {
                                                        let t = buf.trim();
                                                        *dist_expr = if t.is_empty() { None } else { Some(t.to_string()) };
                                                        modified = true;
                                                    }
                                                });
                                                if let Some(e) = dist_expr.as_ref() {
                                                    let txt = match zerocad_core::expr::eval(e, &var_map) {
                                                        Ok(v) => format!("→ {:.2} {}", v, current_unit.suffix()),
                                                        Err(_) => "→ unresolved (check variable names)".to_string(),
                                                    };
                                                    ui.label(egui::RichText::new(txt).size(11.0).weak());
                                                }
                                                ui.add_space(6.0);
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("Type:").size(12.0));
                                                    for (k, label) in [
                                                        (CornerKind::Fillet, "Fillet"),
                                                        (CornerKind::Chamfer, "Chamfer"),
                                                    ] {
                                                        if ui.selectable_label(*kind == k, label).clicked() && *kind != k {
                                                            *kind = k;
                                                            modified = true;
                                                        }
                                                    }
                                                });
                                                ui.add_space(4.0);
                                                ui.label(
                                                    egui::RichText::new("Edge captured in 3D; edits re-cut the body.")
                                                        .size(10.5)
                                                        .color(pal.text_faint),
                                                );
                                            }
                                            FeatureType::VariableSet { variables } => {
                                                // Section header: "Variables" + count.
                                                ui.horizontal(|ui| {
                                                    ui.label(egui::RichText::new("Variables").strong().size(12.0).color(pal.text_strong));
                                                    ui.label(
                                                        egui::RichText::new(format!("({})", variables.len()))
                                                            .size(11.5)
                                                            .color(pal.text_faint),
                                                    );
                                                });
                                                ui.add_space(8.0);

                                                if variables.is_empty() {
                                                    egui::Frame::none()
                                                        .fill(egui::Color32::from_rgb(248, 250, 252))
                                                        .rounding(6.0)
                                                        .inner_margin(10.0)
                                                        .show(ui, |ui| {
                                                            ui.label(
                                                                egui::RichText::new("No variables yet. Add one below.")
                                                                    .size(11.5)
                                                                    .color(pal.text_muted),
                                                            );
                                                        });
                                                }

                                                // Each variable is its own soft card: a full-width
                                                // name field on top, then value + unit + delete.
                                                let mut remove_idx: Option<usize> = None;
                                                for (i, var) in variables.iter_mut().enumerate() {
                                                    egui::Frame::none()
                                                        .fill(egui::Color32::from_rgb(248, 250, 252)) // slate-50
                                                        .rounding(6.0)
                                                        .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)))
                                                        .inner_margin(8.0)
                                                        .show(ui, |ui| {
                                                            ui.add(
                                                                egui::TextEdit::singleline(&mut var.name)
                                                                    .desired_width(f32::INFINITY)
                                                                    .hint_text("name")
                                                                    .font(egui::FontId::proportional(12.5)),
                                                            );
                                                            ui.add_space(6.0);
                                                            ui.horizontal(|ui| {
                                                                ui.add(
                                                                    egui::DragValue::new(&mut var.value)
                                                                        .speed(0.1)
                                                                        .min_decimals(0)
                                                                        .max_decimals(3),
                                                                );
                                                                egui::ComboBox::from_id_source(("var_unit", i))
                                                                    .selected_text(var.unit.suffix())
                                                                    .width(50.0)
                                                                    .show_ui(ui, |ui| {
                                                                        ui.selectable_value(&mut var.unit, Unit::Millimeter, "mm");
                                                                        ui.selectable_value(&mut var.unit, Unit::Inch, "in");
                                                                        ui.selectable_value(&mut var.unit, Unit::Meter, "m");
                                                                    });
                                                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                                    let del = icons::Icon::Trash.icon_button(
                                                                        ui,
                                                                        egui::Color32::TRANSPARENT,
                                                                        egui::Color32::from_rgb(254, 226, 226),
                                                                        egui::Color32::from_rgb(185, 28, 28),
                                                                    );
                                                                    if del.on_hover_text("Delete variable").clicked() {
                                                                        remove_idx = Some(i);
                                                                    }
                                                                });
                                                            });
                                                        });
                                                    ui.add_space(6.0);
                                                }
                                                if let Some(i) = remove_idx {
                                                    variables.remove(i);
                                                }

                                                ui.add_space(2.0);
                                                let add = icons::Icon::Sketch.labeled_button(
                                                    ui,
                                                    "Add Variable",
                                                    egui::Color32::from_rgb(37, 99, 235),
                                                    egui::Color32::from_rgb(29, 78, 216),
                                                    egui::Color32::WHITE,
                                                    egui::Stroke::NONE,
                                                );
                                                if add.clicked() {
                                                    let n = variables.len() + 1;
                                                    variables.push(Variable::new(format!("var{}", n), current_unit));
                                                }
                                            }
                                        }
                                    });
                                });

                            if modified {
                                self.reevaluate_geometry();
                            }

                            if let Some(sketch_id) = extrude_request {
                                self.begin_extrude_whole_sketch(&sketch_id);
                            }
                        }
                    } else {
                        // Render a clean fallback banner
                        egui::Frame::none()
                            .fill(egui::Color32::from_rgb(248, 250, 252))
                            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(226, 232, 240)))
                            .rounding(6.0)
                            .inner_margin(12.0)
                            .show(ui, |ui| {
                                ui.centered_and_justified(|ui| {
                                    ui.label(
                                        egui::RichText::new("ℹ️ Select a feature from the tree to edit properties.")
                                            .weak()
                                            .size(12.0)
                                    );
                                });
                            });
                    }
                });
            });

        // BOTTOM PANEL: Status Bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(6.0, 0.0);

                // Show an elegant status icon based on contents
                let status_icon = if self.status_msg.starts_with("Error") {
                    "❌"
                } else if self.status_msg.starts_with("Click")
                    || self.status_msg.starts_with("Hover")
                    || self.status_msg.starts_with("Select")
                {
                    "🖱️"
                } else if self.status_msg.starts_with("Toggled") {
                    "📐"
                } else {
                    "ℹ️"
                };

                ui.label(egui::RichText::new(status_icon).size(11.0));
                ui.label(
                    egui::RichText::new(&self.status_msg)
                        .size(11.5)
                        .color(self.pal().text_body), // Slate-600
                );

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(ref err) = self.error_msg {
                        ui.label(
                            egui::RichText::new(format!("⚠️ {}", err))
                                .size(11.5)
                                .color(egui::Color32::from_rgb(220, 38, 38)) // Red-600
                                .strong(),
                        );
                    } else {
                        let (verts, tris) = self.mesh_stats;
                        ui.label(
                            egui::RichText::new(format!(
                                "Vertices: {}  |  Triangles: {}",
                                verts, tris
                            ))
                            .size(11.0)
                            .color(self.pal().text_faint), // Slate-400
                        );
                    }
                });
            });
        });

        // RIGHT PANEL: Extrude tool window, shown alongside the inline distance
        // box (the floating "n.nn mm" field, drawn after the viewport in
        // `show_extrude_dialog`). Both edit the same `op.depth`.
        if self.extrude_op.is_some() {
            egui::SidePanel::right("extrude_tool_window")
                .resizable(false)
                .default_width(220.0)
                .show(ctx, |ui| {
                    ui.add_space(8.0);
                    ui.heading("⬆️ Extrude");
                    ui.separator();

                    let unit_suffix = self.current_unit.suffix();
                    let mut commit = false;
                    let mut cancel = false;
                    if let Some(op) = self.extrude_op.as_mut() {
                        let faces: usize = op.targets.iter().map(|t| t.indices.len()).sum();
                        ui.label(format!("Faces: {}", faces));
                        ui.add_space(6.0);

                        ui.label("Distance");
                        let mut changed = ui
                            .add(
                                egui::DragValue::new(&mut op.depth)
                                    .speed(0.5)
                                    .suffix(unit_suffix),
                            )
                            .changed();
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut op.depth, -150.0..=150.0)
                                    .suffix(unit_suffix),
                            )
                            .changed();

                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            if ui
                                .button("Flip")
                                .on_hover_text("Reverse direction")
                                .clicked()
                            {
                                op.depth = -op.depth;
                                changed = true;
                            }
                            ui.weak("· drag in view to push/pull");
                        });

                        // Keep the inline box text in sync with slider/drag edits.
                        if changed {
                            op.depth_text = format!("{:.2}", op.depth);
                        }

                        ui.add_space(10.0);
                        ui.label("Operation");
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                            for (mode, label) in [
                                (ExtrudeMode::NewBody, "New Body"),
                                (ExtrudeMode::Join, "Join"),
                                (ExtrudeMode::Cut, "Cut"),
                            ] {
                                let selected = op.mode == mode;
                                let (fill, text) = if selected {
                                    (egui::Color32::from_rgb(0, 120, 215), egui::Color32::WHITE)
                                } else {
                                    (
                                        egui::Color32::from_rgb(238, 241, 245),
                                        egui::Color32::from_rgb(70, 75, 82),
                                    )
                                };
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new(label).color(text).size(11.5),
                                        )
                                        .fill(fill)
                                        .rounding(3.0)
                                        .min_size(egui::vec2(58.0, 22.0)),
                                    )
                                    .clicked()
                                {
                                    op.mode = mode;
                                }
                            }
                        });
                    }

                    ui.add_space(12.0);
                    ui.separator();
                    ui.horizontal(|ui| {
                        let ok_btn = icons::Icon::Check.labeled_button(
                            ui,
                            "OK",
                            egui::Color32::from_rgb(16, 185, 129),
                            egui::Color32::from_rgb(5, 150, 105),
                            egui::Color32::WHITE,
                            egui::Stroke::NONE,
                        );
                        if ok_btn.clicked() {
                            commit = true;
                        }
                        if ui
                            .add(egui::Button::new("Cancel").min_size(egui::vec2(70.0, 28.0)))
                            .clicked()
                        {
                            cancel = true;
                        }
                    });
                    ui.add_space(4.0);
                    ui.weak("Enter = OK · Esc = Cancel");

                    if commit {
                        self.commit_extrude_op();
                    } else if cancel {
                        self.cancel_extrude_op();
                    }
                });
        }

        // CENTRAL PANEL: 3D CAD Viewport
        egui::CentralPanel::default().show(ctx, |ui| {
            // Draw a nice border and frame around the viewport
            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                let (rect, response) = ui.allocate_exact_size(
                    ui.available_size() - egui::vec2(0.0, 4.0),
                    egui::Sense::click() | egui::Sense::drag(),
                );

                let center_x = rect.center().x + self.camera_pan.x;
                let center_y = rect.center().y + self.camera_pan.y;
                let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);

                let cos_p = self.camera_pitch.cos();
                let sin_p = self.camera_pitch.sin();
                let cos_y = self.camera_yaw.cos();
                let sin_y = self.camera_yaw.sin();

                // 3D coordinate projection mapping function
                let project_3d = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
                    let rx = cos_y * x - sin_y * z;
                    let rz = sin_y * x + cos_y * z;
                    let ry = cos_p * y - sin_p * rz;
                    let final_z = sin_p * y + cos_p * rz;

                    if self.is_perspective {
                        let dist = 1200.0;
                        let factor = dist / (dist - final_z.min(dist * 0.85));
                        (
                            center_x + rx * view_scale * factor,
                            center_y - ry * view_scale * factor,
                            final_z,
                        )
                    } else {
                        (
                            center_x + rx * view_scale,
                            center_y - ry * view_scale,
                            final_z,
                        )
                    }
                };

                // Define coordinates of the 3 origin sheets in 3D for frame-perfect click & hover hit-tests.
                // Quadrant origin planes meet at (0.0, 0.0, 0.0) with size 18.0 units (similar to Fusion 360).
                let size = 18.0;

                // XY Plane corners (positive X, positive Y)
                let xy_c = [
                    project_3d(0.0, 0.0, 0.0),
                    project_3d(size, 0.0, 0.0),
                    project_3d(size, size, 0.0),
                    project_3d(0.0, size, 0.0),
                ];
                let xy_pts = [
                    egui::pos2(xy_c[0].0, xy_c[0].1),
                    egui::pos2(xy_c[1].0, xy_c[1].1),
                    egui::pos2(xy_c[2].0, xy_c[2].1),
                    egui::pos2(xy_c[3].0, xy_c[3].1),
                ];

                // XZ Plane corners (positive X, positive Z)
                let xz_c = [
                    project_3d(0.0, 0.0, 0.0),
                    project_3d(size, 0.0, 0.0),
                    project_3d(size, 0.0, size),
                    project_3d(0.0, 0.0, size),
                ];
                let xz_pts = [
                    egui::pos2(xz_c[0].0, xz_c[0].1),
                    egui::pos2(xz_c[1].0, xz_c[1].1),
                    egui::pos2(xz_c[2].0, xz_c[2].1),
                    egui::pos2(xz_c[3].0, xz_c[3].1),
                ];

                // YZ Plane corners (positive Y, positive Z)
                let yz_c = [
                    project_3d(0.0, 0.0, 0.0),
                    project_3d(0.0, size, 0.0),
                    project_3d(0.0, size, size),
                    project_3d(0.0, 0.0, size),
                ];
                let yz_pts = [
                    egui::pos2(yz_c[0].0, yz_c[0].1),
                    egui::pos2(yz_c[1].0, yz_c[1].1),
                    egui::pos2(yz_c[2].0, yz_c[2].1),
                    egui::pos2(yz_c[3].0, yz_c[3].1),
                ];

                // Perform frame-perfect hover checking immediately
                let hover_pos = response.hover_pos();
                self.hovered_plane = None;
                if self.is_plane_selection_mode {
                    if let Some(pos) = hover_pos {
                        if is_point_in_quad(pos, &xy_pts) {
                            self.hovered_plane = Some(SketchPlane::XY);
                        } else if is_point_in_quad(pos, &xz_pts) {
                            self.hovered_plane = Some(SketchPlane::XZ);
                        } else if is_point_in_quad(pos, &yz_pts) {
                            self.hovered_plane = Some(SketchPlane::YZ);
                        }
                    }

                    if self.hovered_plane.is_some() {
                        egui::show_tooltip_at_pointer(ctx, egui::Id::new("plane_select_tooltip"), |ui| {
                            ui.style_mut().visuals.window_fill = egui::Color32::from_rgb(255, 255, 255);
                            ui.style_mut().visuals.window_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(200, 200, 200));
                            ui.label(
                                egui::RichText::new("Select a plane or planar face")
                                    .color(egui::Color32::from_rgb(45, 45, 45))
                                    .size(12.0)
                            );
                        });
                    }
                }

                // Viewport navigation. Button mapping:
                //   • Middle-drag  → orbit (3D) / pan (sketch, where orbit is locked)
                //   • Shift + drag → pan (3D)
                //   • Left-drag    → selecting faces·edges / drawing shapes
                //   • Shift (sketch) → suppress snapping while drawing
                let pointer_delta = ctx.input(|i| i.pointer.delta());
                let shift = ctx.input(|i| i.modifiers.shift);

                // Middle-drag orbit/pan is latched rather than read from egui's
                // per-frame `dragged_by`, which can momentarily report false
                // mid-motion (drag-threshold / id churn) and make the orbit stall
                // at random points. We start on a middle-press over the viewport
                // and hold until the button is physically released.
                let middle_down = ctx.input(|i| i.pointer.middle_down());
                let middle_pressed =
                    ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Middle));
                if middle_pressed && response.hovered() {
                    self.orbiting = true;
                }
                if !middle_down {
                    self.orbiting = false;
                }
                let middle_drag = self.orbiting;
                let primary_drag = response.dragged_by(egui::PointerButton::Primary);
                let any_drag = middle_drag || primary_drag;

                if !self.camera_anim_active {
                    if self.is_planar_view() {
                        // Lock the camera perpendicular to the active plane,
                        // looking straight down its outward normal.
                        let (p, y) = Self::camera_look_at_normal(self.active_sketch_cs.n);
                        self.camera_pitch = p;
                        self.camera_yaw = y;
                        // Sketch mode: pan with middle-drag (camera can't orbit
                        // here). Shift is reserved for suppressing snapping while
                        // drawing, so it must NOT pan here.
                        if middle_drag {
                            self.camera_pan += pointer_delta;
                        }
                    } else if self.extrude_op.is_some() {
                        // Push/pull with left-drag; Shift+drag pans; middle orbits.
                        if shift && any_drag {
                            self.camera_pan += pointer_delta;
                        } else if middle_drag {
                            // Drag right → model turns right, drag down → tilt
                            // down (grab feel on both axes).
                            self.camera_yaw -= pointer_delta.x * 0.008;
                            self.camera_pitch = (self.camera_pitch + pointer_delta.y * 0.008)
                                .clamp(-std::f32::consts::FRAC_PI_2 + 0.05, std::f32::consts::FRAC_PI_2 - 0.05);
                        } else if primary_drag {
                            // Push/pull ALONG the extrude axis (the sketch-plane
                            // normal) projected into screen space, so the drag
                            // tracks the cursor for any plane orientation. The old
                            // code drove depth straight off vertical mouse motion,
                            // which only lined up when the normal pointed up the
                            // screen (XY/XZ) — on YZ and tilted face planes it ran
                            // backwards and lagged the cursor.
                            let axis = self
                                .extrude_op
                                .as_ref()
                                .and_then(|op| op.targets.first())
                                .map(|t| t.cs.n);
                            if let Some(n) = axis {
                                let view_scale = rect.width().min(rect.height())
                                    / (self.camera_zoom * 5.0).max(1e-3);
                                // World depth change for this drag, mapped onto the
                                // screen projection of the axis. `None` when the
                                // axis is too edge-on to track → vertical fallback.
                                let delta = extrude_depth_delta(
                                    n,
                                    self.camera_pitch,
                                    self.camera_yaw,
                                    view_scale,
                                    pointer_delta,
                                )
                                .unwrap_or(-pointer_delta.y / view_scale.max(1e-6));
                                if let Some(op) = self.extrude_op.as_mut() {
                                    op.depth = (op.depth + delta).clamp(-300.0, 300.0);
                                    // Mirror the dragged value into the inline box.
                                    op.depth_text = format!("{:.2}", op.depth);
                                    // Live ghost mode: while pushing/pulling we show
                                    // only the cheap tool volume, deferring the truck
                                    // boolean until the button is released.
                                    self.extrude_depth_dragging = true;
                                }
                            }
                        }
                    } else {
                        // Free 3D view: middle orbits, Shift+drag pans, left selects.
                        if shift && any_drag {
                            self.camera_pan += pointer_delta;
                        } else if middle_drag {
                            // Drag right → model turns right, drag down → tilt
                            // down (grab feel on both axes).
                            self.camera_yaw -= pointer_delta.x * 0.008;
                            self.camera_pitch = (self.camera_pitch + pointer_delta.y * 0.008)
                                .clamp(-std::f32::consts::FRAC_PI_2 + 0.05, std::f32::consts::FRAC_PI_2 - 0.05);
                        }
                    }
                }

                // End of a push/pull: once the primary button is released, drop
                // the live-ghost flag so this frame's draw_viewport (below) runs
                // the deferred truck boolean once and shows the real result.
                if self.extrude_depth_dragging && !ctx.input(|i| i.pointer.primary_down()) {
                    self.extrude_depth_dragging = false;
                }

                // Zoom: Mouse scroll
                let scroll_delta = ctx.input(|i| i.smooth_scroll_delta.y);
                if scroll_delta != 0.0 {
                    self.camera_zoom = (self.camera_zoom * (1.0 + scroll_delta * 0.002)).clamp(1.0, 50.0);
                }

                // Plane selection click interaction
                if self.is_plane_selection_mode && response.clicked() {
                    if let Some(plane) = self.hovered_plane {
                        log::info!("User selected plane sheet: {:?}", plane);

                        // Save current camera state before pivoting
                        self.pre_sketch_pitch = self.camera_pitch;
                        self.pre_sketch_yaw = self.camera_yaw;
                        self.pre_sketch_perspective = self.is_perspective;

                        // The origin plane becomes the active sketch coordinate
                        // system; animate the camera to look straight at it.
                        let cs = match plane {
                            SketchPlane::XY => CoordinateSystem::XY,
                            SketchPlane::XZ => CoordinateSystem::XZ,
                            SketchPlane::YZ => CoordinateSystem::YZ,
                        };
                        let (target_pitch, target_yaw) = Self::camera_look_at_normal(cs.n);

                        log::info!("Initiating camera animation to pitch: {:.2}, yaw: {:.2}", target_pitch, target_yaw);
                        self.camera_anim_active = true;
                        self.camera_anim_start_pitch = self.camera_pitch;
                        self.camera_anim_start_yaw = self.camera_yaw;
                        self.camera_anim_target_pitch = target_pitch;
                        self.camera_anim_target_yaw = target_yaw;
                        self.camera_anim_start_time = ctx.input(|i| i.time);

                        self.active_sketch_cs = cs;
                        self.active_sketch_on_face = false;
                        self.is_plane_selection_mode = false;
                        self.is_sketch_mode = true;
                        self.reset_sketch_state();

                        // Set topographic mode: orthographic (parallel) projection
                        self.is_perspective = false;
                        self.status_msg = format!("Selected {:?}. Camera locked perpendicular. Active Tool: {:?}", plane, self.active_tool.map_or("Select".to_string(), |t| format!("{:?}", t)));
                    }
                }

                // Sketching interaction: Click inside viewport in Sketch Mode
                // A left-drag with a shape tool sets the first point on press
                // (so press-drag-release begins the shape); the shape is only
                // finalized on the next click — never on the drag itself.
                let begin_draw = response.drag_started_by(egui::PointerButton::Primary);
                if self.is_sketch_mode
                    && self.active_tool.is_some()
                    && (response.clicked() || begin_draw)
                    && !self.camera_anim_active
                {
                    if let Some(hover_pos) = response.interact_pointer_pos() {
                        let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                        // Map the click onto the active sketch plane via a ray /
                        // plane intersection — WYSIWYG on any plane orientation.
                        let raw = self.screen_to_sketch(hover_pos, rect, &self.active_sketch_cs);
                        let (sketch_x, sketch_y) = self.snap_sketch_point(raw, scale, shift);

                        let tool = self.active_tool.unwrap();
                        let pt = (sketch_x, sketch_y);

                        if let Some(kind) = tool.corner_kind() {
                            // Fillet/Chamfer: a click STAGES the nearest corner
                            // (live preview); it isn't committed until Enter / OK.
                            // The user can stack several corners and tune R first.
                            if response.clicked() {
                                self.stage_corner_at(pt, kind);
                            }
                        } else {
                            let point_count = tool.point_count();
                            if self.sketch_points.is_empty() {
                                // First point (click or press-drag). 2-point tools open
                                // the inline dimension dialog here; multi-point tools
                                // (rotated rect, 3-point circle, ellipses) draw by
                                // clicking each point with a live preview.
                                self.sketch_points.push(pt);
                                self.sketch_temp_start = Some(pt);
                                if point_count == 2 {
                                    self.dim_anchor = Some(hover_pos);
                                    self.dim_input = Some(DimInput {
                                        fields: dim_fields_for(tool),
                                        focus_request: Some(0),
                                        active_field: 0,
                                        select_all: true,
                                    });
                                    self.status_msg =
                                        "First point set — move and click, or type dimensions (Tab/Enter)."
                                            .to_string();
                                } else {
                                    self.status_msg = format!(
                                        "Point 1 of {} set — click to place the next point.",
                                        point_count
                                    );
                                }
                            } else if response.clicked() {
                                // A subsequent explicit click. Finalize once enough
                                // points exist, otherwise record an intermediate point.
                                if self.sketch_points.len() + 1 >= point_count {
                                    self.finalize_shape(pt);
                                } else {
                                    self.sketch_points.push(pt);
                                    self.status_msg = format!(
                                        "Point {} of {} set — click to place the next point.",
                                        self.sketch_points.len(),
                                        point_count
                                    );
                                }
                            }
                        }
                    }
                }

                // 3D selection: click picks a body face/edge/vertex (or a finished
                // sketch's face/edge); double-click selects the whole body/sketch.
                // Works in normal 3D view, and while sketching when no drawing
                // tool is armed (the Select state) — so body geometry can be
                // selected without leaving the sketch.
                if (response.clicked() || response.double_clicked())
                    && (!self.is_sketch_mode || self.active_tool.is_none())
                    && !self.is_plane_selection_mode
                    && self.extrude_op.is_none()
                    && self.edge_mod_op.is_none()
                    && !self.camera_anim_active
                {
                    let is_double = response.double_clicked();
                    // Shift / Ctrl (⌘ on macOS) extend the selection: each modified
                    // click adds the picked face/edge/point to the set, or removes it
                    // if already selected, instead of replacing the whole selection.
                    let multi_select =
                        ctx.input(|i| i.modifiers.shift || i.modifiers.ctrl || i.modifiers.command);
                    if let Some(click_pos) = response.interact_pointer_pos() {
                        // Local projection that captures only Copy values (not
                        // `self`), so it doesn't extend a borrow across the
                        // mutations elsewhere in this scope.
                        let is_persp = self.is_perspective;
                        let proj = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
                            let rx = cos_y * x - sin_y * z;
                            let rz = sin_y * x + cos_y * z;
                            let ry = cos_p * y - sin_p * rz;
                            let final_z = sin_p * y + cos_p * rz;
                            if is_persp {
                                let dist = 1200.0;
                                let factor = dist / (dist - final_z.min(dist * 0.85));
                                (center_x + rx * view_scale * factor, center_y - ry * view_scale * factor, final_z)
                            } else {
                                (center_x + rx * view_scale, center_y - ry * view_scale, final_z)
                            }
                        };

                        // Sketches take priority over bodies — a sketch drawn
                        // on a face sits visually on top, so clicking it should
                        // select the sketch element, not the body face behind it.
                        // We try sketch picking first and fall through to body
                        // picking only when no sketch element is under the cursor.

                        let mut best: Option<(String, usize, f32)> = None; // (sketch, region, depth)
                        let mut best_edge: Option<(String, usize, f32)> = None; // (sketch, edge, px dist)
                        const EDGE_TOL_PX: f32 = 6.0;
                        let var_map = self.graph.variable_map();
                        for idx in self.graph.graph.node_indices() {
                            let node = &self.graph.graph[idx];
                            if self.hidden_nodes.contains(&node.id) {
                                continue; // can't pick a hidden sketch
                            }
                            if let FeatureType::Sketch { cs, curves, shapes, corner_mods, .. } = &node.feature {
                                let cs = *cs;
                                // Pick against the variable-resolved geometry.
                                let eff = zerocad_core::effective_curves(curves, shapes, corner_mods, &var_map);
                                let curves = &eff;
                                let to_scr = |u: f32, v: f32| -> egui::Pos2 {
                                    let w = cs.unproject(u, v);
                                    let pr = proj(w.x, w.y, w.z);
                                    egui::pos2(pr.0, pr.1)
                                };
                                // Project a sketch loop to screen coordinates.
                                let project_loop = |loop_pts: &[(f32, f32)]| -> Vec<(f32, f32)> {
                                    loop_pts
                                        .iter()
                                        .map(|&(u, v)| {
                                            let s = to_scr(u, v);
                                            (s.x, s.y)
                                        })
                                        .collect()
                                };

                                // Edge candidates: drawn segments, then circles.
                                let seg_count = curves.segments.len();
                                for (i, s) in curves.segments.iter().enumerate() {
                                    let d = dist_point_to_segment(
                                        click_pos,
                                        to_scr(s.a.0, s.a.1),
                                        to_scr(s.b.0, s.b.1),
                                    );
                                    if d < EDGE_TOL_PX
                                        && best_edge.as_ref().map_or(true, |b| d < b.2)
                                    {
                                        best_edge = Some((node.id.clone(), i, d));
                                    }
                                }
                                for (j, c) in curves.circles.iter().enumerate() {
                                    let mut prev: Option<egui::Pos2> = None;
                                    let mut mind = f32::INFINITY;
                                    for k in 0..=48 {
                                        let th = (k as f32 / 48.0) * std::f32::consts::TAU;
                                        let p = to_scr(
                                            c.center.0 + c.radius * th.cos(),
                                            c.center.1 + c.radius * th.sin(),
                                        );
                                        if let Some(pp) = prev {
                                            mind = mind.min(dist_point_to_segment(click_pos, pp, p));
                                        }
                                        prev = Some(p);
                                    }
                                    if mind < EDGE_TOL_PX
                                        && best_edge.as_ref().map_or(true, |b| mind < b.2)
                                    {
                                        best_edge = Some((node.id.clone(), seg_count + j, mind));
                                    }
                                }

                                for (ri, region) in detect_regions(curves).iter().enumerate() {
                                    let screen = project_loop(&region.boundary);
                                    if screen.len() < 3 {
                                        continue;
                                    }
                                    let click = (click_pos.x, click_pos.y);
                                    // Inside the outer boundary but not in a hole.
                                    let in_outer =
                                        zerocad_core::sketch::point_in_polygon(click, &screen);
                                    let in_hole = region.holes.iter().any(|h| {
                                        let hs = project_loop(h);
                                        hs.len() >= 3
                                            && zerocad_core::sketch::point_in_polygon(click, &hs)
                                    });
                                    if in_outer && !in_hole {
                                        // Average projected depth of the boundary,
                                        // for nearest-face selection.
                                        let depth = region
                                            .boundary
                                            .iter()
                                            .map(|&(u, v)| {
                                                let w = cs.unproject(u, v);
                                                proj(w.x, w.y, w.z).2
                                            })
                                            .sum::<f32>()
                                            / region.boundary.len().max(1) as f32;
                                        if best.as_ref().map_or(true, |b| depth > b.2) {
                                            best = Some((node.id.clone(), ri, depth));
                                        }
                                    }
                                }
                            }
                        }

                        // Did we hit any sketch element?
                        let hit_sketch = best
                            .as_ref()
                            .map(|b| b.0.clone())
                            .or_else(|| best_edge.as_ref().map(|b| b.0.clone()));

                        if hit_sketch.is_some() {
                            // A sketch is under the cursor — select it, clearing
                            // any body selection.
                            self.selected_body.clear();

                            // A PLAIN click selects exactly the one element under
                            // the cursor, replacing any prior sketch selection — so
                            // "click a face, click Extrude" pulls only that face,
                            // never the whole sketch. Shift/Ctrl EXTENDS the
                            // selection (toggling the clicked element), which is the
                            // multi-face extrude workflow. Edges take priority over
                            // faces. (A double-click no longer selects every region —
                            // that silently turned a one-face extrude into a whole-
                            // sketch one; the sketch property panel's "Extrude whole
                            // Sketch" button is the explicit way to get all regions.)
                            if !multi_select {
                                self.selected_faces.clear();
                                self.selected_edges.clear();
                            }
                            if let Some((sid, ei, _)) = best_edge {
                                let key = (sid, ei);
                                if multi_select && !self.selected_edges.insert(key.clone()) {
                                    self.selected_edges.remove(&key);
                                } else {
                                    self.selected_edges.insert(key.clone());
                                }
                                self.status_msg = format!(
                                    "Edge {} of {} selected. Edges: {}.",
                                    key.1,
                                    key.0,
                                    self.selected_edges.len(),
                                );
                            } else if let Some((sid, ri, _)) = best {
                                let key = (sid, ri);
                                if multi_select && !self.selected_faces.insert(key.clone()) {
                                    self.selected_faces.remove(&key);
                                } else {
                                    self.selected_faces.insert(key.clone());
                                }
                                self.status_msg = format!(
                                    "Face {} of {} selected. Faces: {} (click Extrude to build).",
                                    key.1,
                                    key.0,
                                    self.selected_faces.len(),
                                );
                            }
                        } else {
                            // No sketch hit — try body picking instead.
                            let body_hit =
                                self.pick_body_element(click_pos, &proj, sin_p, cos_p, sin_y, cos_y);
                            if let Some((node, pick)) = body_hit {
                                // Sketch-region/edge selections are a separate
                                // concept; a body pick always supersedes them.
                                self.selected_faces.clear();
                                self.selected_edges.clear();
                                if is_double {
                                    // Double-click always selects the whole body.
                                    self.selected_body.clear();
                                    self.selected_body.insert((node.clone(), BodyPick::Whole));
                                    self.status_msg = format!("Selected whole body {}.", node);
                                } else if multi_select {
                                    // Add to the selection, or remove it if already
                                    // selected, so multiple faces/edges/points can be
                                    // picked together (e.g. to fillet several edges).
                                    let key = (node.clone(), pick);
                                    if !self.selected_body.insert(key.clone()) {
                                        self.selected_body.remove(&key);
                                    }
                                    self.status_msg =
                                        format!("{} element(s) selected.", self.selected_body.len());
                                } else {
                                    // Plain click replaces the selection.
                                    self.selected_body.clear();
                                    self.selected_body.insert((node.clone(), pick));
                                    self.status_msg = match pick {
                                        BodyPick::Whole => format!("Selected whole body {}.", node),
                                        BodyPick::Face(f) => {
                                            format!("Selected face {} of {} (Draw Sketch to sketch on it).", f, node)
                                        }
                                        BodyPick::Edge(e) => format!("Selected edge {} of {}.", e, node),
                                        BodyPick::Vertex(v) => format!("Selected point {} of {}.", v, node),
                                    };
                                }
                            } else if !multi_select {
                                // Nothing hit and no modifier held — clear everything
                                // (body AND sketch face/edge selections), so an empty
                                // click is a reliable "deselect all". With a modifier
                                // down, keep the in-progress multi-selection intact.
                                self.selected_body.clear();
                                self.selected_faces.clear();
                                self.selected_edges.clear();
                            }
                        } // end: sketch-first picking
                    }
                }

                // Compute cursor snap preview coordinates
                let current_cursor_snap = if let Some(pos) = hover_pos {
                    let scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);
                    let raw = self.screen_to_sketch(pos, rect, &self.active_sketch_cs);
                    Some(self.snap_sketch_point(raw, scale, shift))
                } else {
                    None
                };

                // Track cursor and refresh live dimension fields.
                self.last_cursor = current_cursor_snap;
                if let (Some(start), Some(cursor)) =
                    (self.sketch_temp_start, current_cursor_snap)
                {
                    self.update_dim_live(start, cursor);

                    // Compute Fusion 360-style screen positions for inline dim inputs.
                    let s_scr = egui::pos2(
                        center_x + start.0 * view_scale,
                        center_y - start.1 * view_scale,
                    );
                    let c_scr = egui::pos2(
                        center_x + cursor.0 * view_scale,
                        center_y - cursor.1 * view_scale,
                    );
                    self.dim_screen_positions = match self.active_tool.unwrap_or(SketchTool::Line) {
                        SketchTool::Rectangle | SketchTool::RectangleCenter => {
                            // Width: midpoint of the edge furthest from shape center (bottom or top)
                            let outer_y = if c_scr.y > s_scr.y { c_scr.y } else { s_scr.y };
                            let mid_w = egui::pos2((s_scr.x + c_scr.x) / 2.0, outer_y + 22.0);
                            // Height: midpoint of the edge furthest from shape center (right or left)
                            let outer_x = if c_scr.x > s_scr.x { c_scr.x } else { s_scr.x };
                            let mid_h = egui::pos2(outer_x + 22.0, (s_scr.y + c_scr.y) / 2.0);
                            vec![mid_w, mid_h]
                        }
                        SketchTool::Circle => {
                            let dx = cursor.0 - start.0;
                            let dy = cursor.1 - start.1;
                            let r = (dx * dx + dy * dy).sqrt();
                            let r_scr = egui::pos2(
                                center_x + (start.0 + r) * view_scale,
                                center_y - start.1 * view_scale,
                            );
                            vec![egui::pos2(r_scr.x + 18.0, r_scr.y)]
                        }
                        SketchTool::Line => {
                            // Length: midpoint of line, offset perpendicular (upward in screen)
                            let mid = egui::pos2(
                                (s_scr.x + c_scr.x) / 2.0,
                                (s_scr.y + c_scr.y) / 2.0 - 22.0,
                            );
                            // Angle: near start point
                            let ang_pos = egui::pos2(s_scr.x + 40.0, s_scr.y + 22.0);
                            vec![mid, ang_pos]
                        }
                        // 3-point tools have no inline dimensions.
                        _ => Vec::new(),
                    };
                } else {
                    self.dim_screen_positions.clear();
                }

                // Draw the 3D projected CAD viewport
                let painter = ui.painter_at(rect);
                self.draw_viewport(painter.clone(), rect, hover_pos, current_cursor_snap);
                painter
            });
        });

        // Dimension dialog overlay (drawn after the viewport, on top).
        self.show_dimension_dialog(ctx);

        // Inline extrude distance box overlay (Fusion-style, mirrors the sketch
        // dimension dialog). Drawn on top of the viewport while extruding.
        self.show_extrude_dialog(ctx);

        // 3D fillet/chamfer: the drag manipulator on the edge, the inline size
        // box, and the inline 2D corner-radius box (anchored on the staged
        // corner/cursor). The handle is drawn first so the box layers over it.
        self.drag_edge_mod_handle(ctx);
        self.show_edge_mod_dialog(ctx);
        self.drag_corner_radius_handle(ctx);
        self.show_corner_radius_box(ctx);

        // Enter commits the staged Fillet/Chamfer corners (the dimension dialog,
        // when open, owns Enter for its own fields — so only act when it's not).
        if self.is_sketch_mode
            && self.dim_input.is_none()
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
            && ctx.input(|i| i.key_pressed(egui::Key::Escape))
        {
            if self.clear_pending_corners() {
                // Discarded the staged corners, keep the tool armed.
                self.status_msg = "Staged corners discarded.".to_string();
            } else if !self.sketch_points.is_empty() {
                // Abort the in-progress (multi-point) shape, keep the tool armed.
                self.cancel_in_progress_shape();
                self.status_msg = "Shape cancelled.".to_string();
            } else if self.active_tool.is_some() {
                self.active_tool = None;
                self.status_msg =
                    "Tool deselected — select faces, edges, or points.".to_string();
                log::info!("Escape: switched to Select mode");
            }
        }

        // In the plain 3D view (no sketch, no live op or dialog), Escape returns to
        // the neutral Select state by clearing the current selection.
        if !self.is_sketch_mode
            && self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
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

        // Persist preferences whenever the unit, theme, or onboarding toggle
        // changed this frame. One diff against the last-saved snapshot covers
        // every edit site (Settings window, Ctrl+D theme toggle, the Welcome
        // footer checkbox) without threading a save into each.
        let current = settings::AppSettings {
            show_onboarding: self.show_onboarding,
            dark_mode: self.dark_mode,
            unit: self.current_unit,
        };
        if current != self.settings_baseline {
            current.save();
            self.settings_baseline = current;
        }
    }
}

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
