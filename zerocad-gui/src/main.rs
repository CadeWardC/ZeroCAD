#![cfg_attr(
    all(target_os = "windows", not(debug_assertions)),
    windows_subsystem = "windows"
)]

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use eframe::egui;
use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{
    detect_regions, CoordinateSystem, CornerKind, CornerMod, Dimension, Document, EdgeRef,
    EvaluatedScene, ExtrudeMode, FeatureNode, FeatureType, LineSegment, MockMesh, Region,
    SceneStats, SketchCurves, SketchPlane, SketchShape, Unit, Variable, Vec3,
};

mod body_ops_ui;
mod bug_report;
mod combine_ui;
mod direct_edit_ui;
mod document_worker;
mod draft_ui;
mod dxf_ui;
mod edgemod;
mod evaluation_worker;
mod expr;
mod extrude;
mod geom2d;
mod gpu_viewport;
mod hole_ui;
mod icons;
mod inspection_ui;
mod loft_sweep_ui;
mod move_ui;
mod parameters_ui;
mod pattern_ui;
mod recovery;
mod render;
mod revolve_ui;
mod settings;
mod shell_ui;
mod shortcuts;
mod sketch_ui;
mod theme;
mod thread_ui;
mod thumbnail;
use body_ops_ui::{ScaleBodyOp, SplitBodyOp};
use combine_ui::CombineOp;
use direct_edit_ui::DirectFaceCommand;
use draft_ui::DraftOp;
use edgemod::EdgeModOp;
use expr::Autocomplete;
use extrude::ExtrudeOp;
use geom2d::{circumcircle, dist_point_to_segment, is_point_in_quad, project_point_on_segment};
use hole_ui::HoleOp;
use inspection_ui::{InspectionDialog, SectionView};
use loft_sweep_ui::SweepOp;
use move_ui::{BodyClipboard, MoveOp};
use parameters_ui::ParametersDialog;
use pattern_ui::PatternOp;
use revolve_ui::RevolveOp;
use shell_ui::ShellOp;
use shortcuts::{Keymap, ShortcutAction};
use sketch_ui::{dim_fields_for, DimInput, SketchDimensionEditor};
use theme::{apply_premium_dark_theme, apply_premium_light_theme, Palette};
use thread_ui::ThreadOp;
use zerocad_core::parametric::FaceRef;

fn automatic_wgpu_backends() -> wgpu::Backends {
    #[cfg(target_os = "windows")]
    {
        return wgpu::Backends::DX12;
    }
    #[cfg(target_os = "linux")]
    {
        return wgpu::Backends::VULKAN;
    }
    #[cfg(target_os = "macos")]
    {
        return wgpu::Backends::METAL;
    }
    #[allow(unreachable_code)]
    wgpu::Backends::PRIMARY
}

fn main() -> eframe::Result<()> {
    recovery::install_panic_hook();
    let mut builder = env_logger::Builder::from_default_env();
    if std::env::var("RUST_LOG").is_err() {
        builder.filter_level(log::LevelFilter::Info);
        // Keep ZeroCAD's useful application messages while hiding routine GPU
        // synchronization chatter. Graphics warnings and errors remain visible.
        builder.filter_module("wgpu", log::LevelFilter::Warn);
        builder.filter_module("wgpu_core", log::LevelFilter::Warn);
        builder.filter_module("wgpu_hal", log::LevelFilter::Warn);
        builder.filter_module("naga", log::LevelFilter::Warn);
    }
    builder.target(env_logger::Target::Pipe(Box::new(
        recovery::SessionLogWriter::new(),
    )));
    builder.init();

    log::info!("========================================================");
    log::info!("Starting ZeroCAD - Premium 3D Parametric CAD Designer...");
    log::info!("Application logger initialized");
    log::info!("========================================================");

    // The user's persisted backend preference (Settings → Viewport) narrows
    // which wgpu backends the adapter search may pick. The WGPU_BACKEND env var
    // still overrides everything, as a debugging escape hatch.
    let app_settings = settings::AppSettings::load();
    let backends = match app_settings.backend {
        settings::GraphicsBackend::Auto => automatic_wgpu_backends(),
        settings::GraphicsBackend::Vulkan => wgpu::Backends::VULKAN,
        settings::GraphicsBackend::Dx12 => wgpu::Backends::DX12,
        settings::GraphicsBackend::OpenGl => wgpu::Backends::GL,
    };

    let run = |supported_backends: wgpu::Backends| -> eframe::Result<()> {
        let app_settings = app_settings.clone();
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_title("ZeroCAD - 3D Parametric CAD Designer")
                .with_inner_size([1200.0, 800.0])
                .with_min_inner_size([960.0, 640.0]),
            // The workspace viewport renders its 3D scene on the GPU via
            // openrcad-render, embedded as an egui texture. That requires eframe
            // to run on the wgpu backend so `frame.wgpu_render_state()` yields
            // the device/queue the render core draws with.
            renderer: eframe::Renderer::Wgpu,
            wgpu_options: egui_wgpu::WgpuConfiguration {
                supported_backends: wgpu::util::backend_bits_from_env()
                    .unwrap_or(supported_backends),
                ..Default::default()
            },
            ..Default::default()
        };
        eframe::run_native(
            "ZeroCAD",
            options,
            Box::new(move |_cc| Ok(Box::new(ZeroCadApp::new_with_settings(app_settings)))),
        )
    };

    // Wgpu owns the embedded GPU viewport. If the selected native backend has
    // no usable adapter, retry through wgpu's OpenGL backend; this preserves
    // the OpenGL escape hatch without shipping eframe's second renderer stack.
    match run(backends) {
        Err(eframe::Error::Wgpu(err)) if backends != wgpu::Backends::GL => {
            log::warn!("native wgpu backend unavailable ({err}); retrying through OpenGL");
            run(wgpu::Backends::GL)
        }
        other => other,
    }
}

/// A drawing tool *mode*. Each toolbar button (a [`ToolFamily`]) exposes one or
/// more of these via its flyout; the first listed is that button's default.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchTool {
    Line,
    /// Cubic B-spline defined by editable control points. Double-click or press
    /// Enter to finish the current curve.
    ControlPointSpline,
    /// Interpolating spline through editable fit points. Double-click or press
    /// Enter to finish the current curve.
    FitPointSpline,
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
    /// Regular N-gon with its vertices on the drag circle (center → vertex sets
    /// the circumradius and the rotation). Side count comes from the toolbar.
    PolygonInscribed,
    /// Regular N-gon with its edge midpoints on the drag circle (center → flat
    /// sets the apothem and the rotation). Side count comes from the toolbar.
    PolygonCircumscribed,
    /// Center-to-center slot: click both end centers, then a third point to set
    /// the full width. Coincident centers produce a circle.
    Slot,
    /// Associative offset of selected line/arc/circle entities. Click sources,
    /// then click the creation side/distance.
    Offset,
    /// Remove the exact line/arc/circle span under the cursor. The hover result
    /// is the immutable replacement plan committed by the click.
    Trim,
    /// Add a driving sketch dimension inferred from selected geometry, then
    /// place and edit its value directly in the viewport.
    Dimension,
    /// Reflect the whole sketch across a 2-point axis (center line). Not a draw
    /// tool in the shape sense — its two clicks define the mirror line.
    Mirror,
    /// Round a sketch corner (click the corner). Not a draw tool.
    Fillet,
    /// Bevel a sketch corner (click the corner). Not a draw tool.
    Chamfer,
}

/// What kind of existing geometry the live cursor snapped onto while sketching.
/// Drives the on-screen snap glyph (an endpoint ring, a midpoint/centre cross)
/// so the user can see *why* the point locked where it did. `OnLine`/`Grid`
/// snaps are silent — they get no glyph.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SnapKind {
    /// A segment endpoint / shared corner.
    Endpoint,
    /// The midpoint of a straight segment.
    Midpoint,
    /// A circle (or arc) centre.
    Center,
    /// The nearest point along a segment.
    OnLine,
    /// The active sketch plane's origin (sketch coords `(0, 0)`).
    Origin,
    /// A point lying on a woken midline inference guide (perpendicular through a
    /// segment midpoint). No point glyph — the dashed guide line is the cue.
    Midline,
    /// The intersection of two midlines — e.g. the centre of a rectangle. Drawn
    /// with the same orange X as [`SnapKind::Midpoint`]/[`SnapKind::Center`].
    MidlineCenter,
    /// The background placement grid.
    Grid,
}

/// A "woken" midline inference guide (Fusion 360 style): the perpendicular line
/// through a straight segment's midpoint. Waking one lets the cursor snap along
/// the midline and onto its intersections with other midlines (rectangle centre).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MidlineGuide {
    /// The segment midpoint the guide passes through.
    pub origin: (f32, f32),
    /// Unit direction of the guide line (perpendicular to the owning segment).
    pub dir: (f32, f32),
}

/// The toolbar button a [`SketchTool`] lives under. Switching buttons is by
/// family; the flyout picks the exact mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolFamily {
    Line,
    Spline,
    Rectangle,
    Circle,
    /// The regular-polygon button, holding the inscribed and circumscribed modes.
    Polygon,
    Slot,
    Offset,
    Trim,
    Dimension,
    /// The sketch mirror button (single mode, no flyout).
    Mirror,
    /// The corner-modifier button, holding both Fillet and Chamfer (its flyout
    /// switches between them) — mirroring the single 3D edge fillet/chamfer button.
    Corner,
}

impl SketchTool {
    /// Which toolbar button this mode belongs to.
    pub fn family(self) -> ToolFamily {
        match self {
            SketchTool::Line => ToolFamily::Line,
            SketchTool::ControlPointSpline | SketchTool::FitPointSpline => ToolFamily::Spline,
            SketchTool::Rectangle
            | SketchTool::RectangleCenter
            | SketchTool::RectangleThreePoint => ToolFamily::Rectangle,
            SketchTool::Circle
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse => ToolFamily::Circle,
            SketchTool::PolygonInscribed | SketchTool::PolygonCircumscribed => ToolFamily::Polygon,
            SketchTool::Slot => ToolFamily::Slot,
            SketchTool::Offset => ToolFamily::Offset,
            SketchTool::Trim => ToolFamily::Trim,
            SketchTool::Dimension => ToolFamily::Dimension,
            SketchTool::Mirror => ToolFamily::Mirror,
            SketchTool::Fillet => ToolFamily::Corner,
            SketchTool::Chamfer => ToolFamily::Corner,
        }
    }

    /// How many points (clicks) the *drawing* tool places before the shape
    /// finalizes. The corner tools (Fillet/Chamfer) aren't drawn this way.
    pub fn point_count(self) -> usize {
        match self {
            SketchTool::Line
            | SketchTool::ControlPointSpline
            | SketchTool::FitPointSpline
            | SketchTool::Rectangle
            | SketchTool::RectangleCenter
            | SketchTool::Circle
            | SketchTool::PolygonInscribed
            | SketchTool::PolygonCircumscribed
            | SketchTool::Mirror => 2,
            SketchTool::RectangleThreePoint
            | SketchTool::ThreePointCircle
            | SketchTool::Ellipse
            | SketchTool::ThreePointEllipse => 3,
            SketchTool::Slot => 3,
            SketchTool::Offset
            | SketchTool::Trim
            | SketchTool::Dimension
            | SketchTool::Fillet
            | SketchTool::Chamfer => 1,
        }
    }

    /// Whether this tool is defined entirely by point clicks and therefore has
    /// no inline dimension dialog. Polygons are excluded because their second
    /// point drives an editable construction-circle diameter.
    pub fn is_point_drawn(self) -> bool {
        matches!(
            self,
            SketchTool::RectangleThreePoint
                | SketchTool::ThreePointCircle
                | SketchTool::Ellipse
                | SketchTool::ThreePointEllipse
                | SketchTool::Slot
                | SketchTool::Offset
                | SketchTool::Trim
                | SketchTool::Dimension
                | SketchTool::Mirror
        )
    }

    pub fn is_spline(self) -> bool {
        matches!(
            self,
            SketchTool::ControlPointSpline | SketchTool::FitPointSpline
        )
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
    pub(crate) fn icon(self) -> icons::Icon {
        match self {
            SketchTool::Line => icons::Icon::Line,
            SketchTool::ControlPointSpline | SketchTool::FitPointSpline => icons::Icon::Line,
            SketchTool::Rectangle => icons::Icon::Rectangle,
            SketchTool::RectangleCenter => icons::Icon::RectangleFromCenter,
            SketchTool::RectangleThreePoint => icons::Icon::RectangleThreePoints,
            SketchTool::Circle => icons::Icon::Circle,
            SketchTool::ThreePointCircle => icons::Icon::ThreePointCircle,
            SketchTool::Ellipse => icons::Icon::Ellipse,
            SketchTool::ThreePointEllipse => icons::Icon::ThreePointEllipse,
            SketchTool::PolygonInscribed | SketchTool::PolygonCircumscribed => icons::Icon::Polygon,
            SketchTool::Slot => icons::Icon::Slot,
            SketchTool::Offset => icons::Icon::Offset,
            SketchTool::Trim => icons::Icon::Trim,
            SketchTool::Dimension => icons::Icon::Dimension,
            SketchTool::Mirror => icons::Icon::Mirror,
            SketchTool::Fillet => icons::Icon::Fillet,
            SketchTool::Chamfer => icons::Icon::Chamfer,
        }
    }

    /// Short menu label for the flyout.
    pub fn label(self) -> &'static str {
        match self {
            SketchTool::Line => "Line",
            SketchTool::ControlPointSpline => "Control Point Spline",
            SketchTool::FitPointSpline => "Fit Point Spline",
            SketchTool::Rectangle => "Rectangle",
            SketchTool::RectangleCenter => "Center Rectangle",
            SketchTool::RectangleThreePoint => "3-Point Rectangle",
            SketchTool::Circle => "Center Circle",
            SketchTool::ThreePointCircle => "3-Point Circle",
            SketchTool::Ellipse => "Ellipse",
            SketchTool::ThreePointEllipse => "3-Point Ellipse",
            SketchTool::PolygonInscribed => "Inscribed Polygon",
            SketchTool::PolygonCircumscribed => "Circumscribed Polygon",
            SketchTool::Slot => "Center-to-Center Slot",
            SketchTool::Offset => "Offset",
            SketchTool::Trim => "Trim",
            SketchTool::Dimension => "Dimension",
            SketchTool::Mirror => "Mirror",
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
            ToolFamily::Spline => SketchTool::ControlPointSpline,
            ToolFamily::Rectangle => SketchTool::Rectangle,
            ToolFamily::Circle => SketchTool::Circle,
            ToolFamily::Polygon => SketchTool::PolygonInscribed,
            ToolFamily::Slot => SketchTool::Slot,
            ToolFamily::Offset => SketchTool::Offset,
            ToolFamily::Trim => SketchTool::Trim,
            ToolFamily::Dimension => SketchTool::Dimension,
            ToolFamily::Mirror => SketchTool::Mirror,
            ToolFamily::Corner => SketchTool::Fillet,
        }
    }

    /// The modes offered in this button's flyout, in order (first = default).
    pub fn modes(self) -> &'static [SketchTool] {
        match self {
            ToolFamily::Line => &[SketchTool::Line],
            ToolFamily::Spline => &[SketchTool::ControlPointSpline, SketchTool::FitPointSpline],
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
            ToolFamily::Polygon => &[
                SketchTool::PolygonInscribed,
                SketchTool::PolygonCircumscribed,
            ],
            ToolFamily::Slot => &[SketchTool::Slot],
            ToolFamily::Offset => &[SketchTool::Offset],
            ToolFamily::Trim => &[SketchTool::Trim],
            ToolFamily::Dimension => &[SketchTool::Dimension],
            ToolFamily::Mirror => &[SketchTool::Mirror],
            ToolFamily::Corner => &[SketchTool::Fillet, SketchTool::Chamfer],
        }
    }
}

/// The expressions bound to a sketch's dimensions, for display in the property
/// panel (so the editable source remains visible after it evaluates).
fn sketch_variable_dims(shapes: &[SketchShape]) -> Vec<String> {
    let mut out = Vec::new();
    for s in shapes {
        let dims: Vec<&Dimension> = match s {
            SketchShape::Rectangle { w, h, .. } => vec![w, h],
            SketchShape::Circle { diameter, .. } => vec![diameter],
            SketchShape::Line {
                length, angle_deg, ..
            } => vec![length, angle_deg],
            SketchShape::RegularPolygon { diameter, .. } => vec![diameter],
            SketchShape::Slot { width, .. } => vec![width],
            SketchShape::Spline { .. } | SketchShape::Imported { .. } | SketchShape::Raw { .. } => {
                vec![]
            }
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

/// Browser label for one runtime body emitted by a feature. The first output
/// keeps the feature's label; numbered `Body_N` labels continue naturally for
/// later disconnected outputs (Body_1, Body_2, ...).
pub(crate) fn body_output_label(feature_label: &str, output_index: usize) -> String {
    if output_index == 0 {
        return feature_label.to_string();
    }

    if let Some((prefix, number)) = feature_label.rsplit_once('_') {
        if let Ok(number) = number.parse::<usize>() {
            return format!("{prefix}_{}", number + output_index);
        }
    }

    format!("{feature_label} ({})", output_index + 1)
}

#[cfg(test)]
mod body_output_label_tests {
    use super::body_output_label;

    #[test]
    fn disconnected_outputs_receive_separate_body_numbers() {
        assert_eq!(body_output_label("Body_1", 0), "Body_1");
        assert_eq!(body_output_label("Body_1", 1), "Body_2");
        assert_eq!(body_output_label("Body_1", 2), "Body_3");
    }
}

/// What the Datum toolbar menu creates (see `app::datum::create_datum`).
#[derive(Debug, Clone, PartialEq)]
pub enum DatumKind {
    OffsetPlane(zerocad_core::PlaneBase),
    AnglePlane,
    ThreePointPlane,
    Axis,
    Point,
    PlaneFromFace(zerocad_core::parametric::FaceRef),
    AxisFromEdge(zerocad_core::parametric::EdgeRef),
    AxisFromFace(zerocad_core::parametric::FaceRef),
    AxisFromVertices(
        zerocad_core::parametric::VertexRef,
        zerocad_core::parametric::VertexRef,
    ),
    PointFromVertex(zerocad_core::parametric::VertexRef),
    PointBetweenVertices(
        zerocad_core::parametric::VertexRef,
        zerocad_core::parametric::VertexRef,
    ),
    PointOnEdge(zerocad_core::parametric::EdgeRef),
    PointAtCircleCenter(zerocad_core::parametric::EdgeRef),
}

/// What the user did on a feature-tree row this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RowAction {
    None,
    Delete,
    ToggleVisibility,
    /// Toggle whether the feature participates in parametric rebuilds.
    ToggleSuppression,
    MoveUp,
    MoveDown,
    /// Right-clicked "Add Variable" on a variable-set row.
    AddVariable,
    /// Right-clicked "Edit Sketch" on a sketch row.
    EditSketch,
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

    fn profile(self, hydrated_cache_mb: u32) -> zerocad_core::SaveProfile {
        match self {
            SaveFormat::ZcadLightweight => zerocad_core::SaveProfile::Compact,
            SaveFormat::ZcadFull => zerocad_core::SaveProfile::Hydrated {
                total_accelerator_budget: hydrated_cache_mb as u64 * 1024 * 1024,
            },
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

type SharedBodyMeshes = std::sync::Arc<Vec<(String, MockMesh)>>;

#[derive(Debug, Clone)]
struct UndoSnapshot {
    document: Document,
}

/// Authoritative state of a live sketch before one user-visible transaction.
///
/// This is deliberately independent of the whole-document undo stack. A
/// future Trim, Offset, or sketch Pattern may touch many entities but still
/// pushes exactly one of these snapshots and therefore undoes atomically.
#[derive(Debug, Clone)]
struct WorkingSketchSnapshot {
    shapes: Vec<SketchShape>,
    corner_mods: Vec<CornerMod>,
    mirrors: Vec<zerocad_core::SketchMirror>,
    solver_model: Option<zerocad_core::sketch::SketchSolverModel>,
    entity_ids: Vec<zerocad_core::sketch::EntityId>,
    next_entity_id: u32,
}

struct PendingSave {
    path: PathBuf,
    profile: zerocad_core::SaveProfile,
    started: std::time::Instant,
    dispatched: bool,
    revision: Option<u64>,
}

struct ExportCompletion {
    message: String,
    error: bool,
}

pub(crate) enum PendingVisualMode {
    Extrude(ExtrudeMode),
    EdgeMod,
}

pub(crate) struct PendingCommitVisual {
    pub(crate) bodies: Vec<(String, MockMesh)>,
    pub(crate) mesh: Option<MockMesh>,
    pub(crate) mode: PendingVisualMode,
    /// Whether `bodies` already represents the exact result for the committed
    /// inputs. False means the current tool ghost must remain visible while the
    /// final evaluator catches up.
    pub(crate) exact_bodies: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PendingMirrorJoinFeedback {
    feature_id: String,
    source_body_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum MirrorJoinOutcome {
    Evaluating,
    Joined,
    Separate,
    Unresolved(String),
}

struct ZeroCadApp {
    pending_visual: Option<PendingCommitVisual>,
    /// A just-created Mirror+Join whose actual joined/separate outcome is
    /// waiting on the committed evaluator. This is UI-only derived state.
    pending_mirror_join_feedback: Option<PendingMirrorJoinFeedback>,
    /// Authoritative editable project. The runtime graph is an evaluator
    /// projection owned by this document rather than the application root.
    document: Document,
    selected_node_id: Option<String>,
    /// Part-evaluator and hydrated-cache compatibility storage. Viewport,
    /// picking, bounds, sectioning, and thumbnail consumers use
    /// `evaluated_scene`, which shares this allocation.
    body_meshes: SharedBodyMeshes,
    /// Immutable renderer-facing view of `body_meshes`, coupled to the GPU
    /// invalidation epoch. Part documents currently use identity placements.
    evaluated_scene: app::ViewportSceneState,
    /// Construction geometry resolved alongside `body_meshes` at the same
    /// document revision. This includes face/edge/vertex-derived datums.
    datum_values: std::collections::HashMap<String, zerocad_core::DatumValue>,
    /// Explicit geometry-pool, referenced, and instance-expanded complexity
    /// measures. A future status bar can choose a labeled meaning rather than
    /// overloading one ambiguous vertex/triangle tuple.
    scene_stats: SceneStats,
    /// GPU-accelerated 3D viewport: renders the committed bodies via
    /// `openrcad-render` into an offscreen texture composited under the CPU
    /// overlays. Falls back to the CPU rasterizer when `gpu_render` is off or the
    /// wgpu backend is unavailable.
    gpu: gpu_viewport::GpuViewport,
    /// Master toggle for the GPU viewport (settings-controlled, default on). When
    /// off, the legacy CPU software renderer draws solids and edges.
    gpu_render: bool,
    /// Requested wgpu backend (Auto/Vulkan/DX12/OpenGL). Persisted; applied at
    /// startup in `main`, so edits here take effect on the next launch.
    graphics_backend: settings::GraphicsBackend,
    /// GPU viewport anti-aliasing quality (persisted; applied live).
    msaa_level: settings::MsaaLevel,
    hydrated_cache_mb: u32,
    /// This frame's composited GPU scene texture, painted by `draw_viewport`.
    gpu_texture_id: Option<egui::TextureId>,
    /// The preview plan `render_gpu_scene` resolved this frame, handed to
    /// `draw_viewport` so the plan (and its cloned preview body set) is built
    /// once per frame, not once per renderer. `None` when the GPU path didn't
    /// run this frame (CPU mode resolves its own).
    frame_preview_plan: Option<render::PreviewPlan>,
    /// Background "refine" evaluation for slow arc fillets. `reevaluate_geometry`
    /// shows the fast faceted draft instantly, then — when the model has a fillet
    /// — spawns the arc-cutter evaluation on a worker thread and swaps the result
    /// in here, so committing a fillet never stalls the UI (the arc boolean is
    /// ~1s; see `ParametricGraph::has_arc_fillet`). Stale jobs are ignored by
    /// generation.
    evaluator: evaluation_worker::ModelEvaluator,
    document_worker: document_worker::DocumentWorker,
    recovery: recovery::RecoveryManager,
    pending_save: Option<PendingSave>,
    last_slow_frame_log: Option<std::time::Instant>,
    export_completions: std::sync::Arc<std::sync::Mutex<Vec<ExportCompletion>>>,
    eval_generation: u64,
    /// Monotonic committed-document revision used to avoid clearing a newer
    /// autosave when an older background Save finishes.
    document_revision: u64,
    /// True while a background refine is in flight (drives a "Refining…" hint).
    eval_pending: bool,
    eval_started: Option<std::time::Instant>,
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
    /// When sketching on a body face, that face's boundary loops projected into
    /// the sketch's 2D plane — reference geometry the user never drew. It joins
    /// region detection (drawn shapes split against the face outline, and the
    /// outline itself is an extrudable region), renders in a distinct color,
    /// and is a snap target. Persisted onto `graph.sketch_face_boundaries` at
    /// commit. Empty for origin-plane / datum sketches.
    active_face_boundary: SketchCurves,
    /// When sketching on a datum plane, that datum's node id, carried from
    /// plane-pick to sketch-commit so the finished sketch records the
    /// attachment (and follows the datum when it's edited). `None` otherwise.
    active_sketch_datum_ref: Option<String>,
    /// Creation timestamp (Unix seconds) of the document currently open, carried
    /// from the loaded `.zcad` so re-saving preserves "created" rather than
    /// stamping it anew. `None` for a fresh/never-saved or legacy document.
    doc_created_unix: Option<u64>,
    /// Path of the open/saved document, used only for the project title in the
    /// application chrome. A fresh part is shown as "Untitled Project".
    current_document_path: Option<PathBuf>,

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
    /// Associative mirror operations of the in-progress sketch (see
    /// [`zerocad_core::SketchMirror`]). Each reflects the live geometry across
    /// its axis; persisted on the node at Finish Sketch.
    sketch_mirrors: Vec<zerocad_core::SketchMirror>,
    /// Corners the user has clicked with the Fillet/Chamfer tool but not yet
    /// committed. They preview live with the current radius and are only folded
    /// into `sketch_corner_mods` when the user presses Enter / clicks OK.
    pending_corners: Vec<(f32, f32)>,
    /// Editable radius/setback for the Fillet/Chamfer tools (a number or a
    /// variable expression).
    corner_radius_text: String,
    /// Optional expression for the Offset distance. Empty means the placement
    /// click supplies the exact signed distance.
    offset_distance_text: String,
    /// Associative sketch-pattern inputs. X/Y are a direction for Linear and a
    /// center for Circular; source expressions are persisted where supported.
    sketch_pattern_x_text: String,
    sketch_pattern_y_text: String,
    sketch_pattern_spacing_text: String,
    sketch_pattern_count_text: String,
    sketch_pattern_angle_text: String,
    /// Immutable exact trim plan currently under the cursor. A successful
    /// click consumes this same plan so hover and commit cannot diverge.
    sketch_trim_preview: Option<zerocad_core::sketch::TrimPreview>,
    /// Side count for the regular-polygon tools (inscribed/circumscribed),
    /// editable in the sketch toolbar. Clamped to a sane 3..=64 range.
    polygon_sides: u32,
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
    /// When the active sketch session is EDITING an existing committed sketch
    /// (via "Edit Sketch"), the node id to update in place at Finish. `None`
    /// for a fresh sketch, which commits as a new node. In-place update keeps
    /// the node id, so `sketch_face_refs` and dependency edges — and every
    /// captured reference downstream — survive the edit.
    editing_sketch_id: Option<String>,
    /// The constraint-solver model of the active sketch (shared points +
    /// entities + constraints). `Some` when editing a committed sketch (legacy
    /// shapes are promoted on entry) — the source of truth for the live
    /// geometry; drag-to-solve mutates it and Finish persists it.
    sketch_solver_model: Option<zerocad_core::sketch::SketchSolverModel>,
    /// Durable per-shape ids of the active sketch (parallel to
    /// `sketch_shapes`). Loaded from the node on Edit Sketch; new shapes drawn
    /// during the session get fresh ids at commit. NEVER resequenced — a
    /// surviving shape keeps its id through the edit.
    sketch_entity_ids: Vec<zerocad_core::sketch::EntityId>,
    /// Next unallocated entity id for the active sketch's model.
    sketch_next_entity_id: u32,
    /// The solver point currently being dragged (latched on press near a
    /// point, cleared on release) — drag-to-solve state.
    sketch_drag_point: Option<zerocad_core::sketch::EntityId>,
    /// Selected solver points/entities in an Edit Sketch session (click to
    /// select, Shift-click to extend). Drives which constraint-palette buttons
    /// are applicable.
    sketch_selected_ids: Vec<zerocad_core::sketch::EntityId>,
    /// Constraint selected in the constraints panel (for delete / highlight).
    sketch_selected_constraint: Option<zerocad_core::sketch::EntityId>,
    /// Active value editor for a dimension just placed with the Dimension tool.
    sketch_dimension_editor: Option<SketchDimensionEditor>,
    /// User-placed dimension-label anchors in sketch-plane coordinates. This is
    /// presentation state for the current edit session; constraints themselves
    /// remain fully durable in the solver model and project file.
    sketch_dimension_positions: HashMap<zerocad_core::sketch::EntityId, (f64, f64)>,
    /// The conflicting constraint reported by the last live solve (None when
    /// the model solves clean) — drives the red badge + list highlight without
    /// re-solving in the render path.
    sketch_conflict_constraint: Option<zerocad_core::sketch::EntityId>,
    /// First click of any 2-click tool (line/rect/circle). When `None`, the
    /// next click sets the starting point; when `Some(pt)` it completes the shape.
    /// Mirrors `sketch_points[0]` (kept for the dim dialog + preview anchor).
    sketch_temp_start: Option<(f32, f32)>,
    /// All points placed so far for the in-progress shape *except* the final one
    /// (which the next click / cursor supplies). 2-point tools hold one entry;
    /// 3-point tools hold up to two. Cleared when the shape finalizes or cancels.
    sketch_points: Vec<(f32, f32)>,
    /// Start vertex of the current continuous-Line chain. `Some` while a Line
    /// chain is being drawn (≥1 segment committed), so the next segment re-seeds
    /// from each endpoint and a click back on the start closes the loop into a
    /// face. `None` for every other tool and between chains.
    line_chain_start: Option<(f32, f32)>,
    hovered_plane: Option<SketchPlane>,
    /// The datum plane under the cursor during plane selection (node id).
    hovered_datum_plane: Option<String>,
    /// Material density (g/cm³) for the Measure panel's mass line. A display
    /// preference, not part of the document.
    measure_density: f32,
    /// Read-only measure/section/interference workspace.
    inspection_dialog: Option<InspectionDialog>,
    /// Active visual section plane; presentation-only and never serialized.
    section_view: Option<SectionView>,
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
    /// Internal body clipboard used by Ctrl+C / Ctrl+V. The source remains a
    /// parametric reference; each paste creates a translated copy feature.
    body_clipboard: Option<BodyClipboard>,
    /// Active body move dialog and manipulator.
    move_op: Option<MoveOp>,
    /// Active two-body Join/Cut dialog.
    combine_op: Option<CombineOp>,
    /// Active planar split dialog.
    split_body_op: Option<SplitBodyOp>,
    /// Active positive uniform scale dialog.
    scale_body_op: Option<ScaleBodyOp>,
    /// Cheap translated body-set preview shared by CPU and GPU renderers.
    move_preview_bodies: Option<SharedBodyMeshes>,
    /// Depth used by the Extrude action.
    extrude_depth: f32,
    /// Last-used extrude mode (new body / join / cut), seeded into each new op.
    extrude_mode: ExtrudeMode,
    /// Active (uncommitted) extrude operation with its live preview, if any.
    extrude_op: Option<ExtrudeOp>,
    /// The in-progress Revolve tool (axis/angle/mode dialog), `None` when idle.
    revolve_op: Option<RevolveOp>,
    /// The in-progress Pattern/Mirror tool dialog, `None` when idle.
    pattern_op: Option<PatternOp>,
    /// The in-progress Hole tool dialog, `None` when idle.
    hole_op: Option<HoleOp>,
    /// The in-progress Shell tool dialog, `None` when idle.
    shell_op: Option<ShellOp>,
    /// The in-progress Thread tool dialog, `None` when idle.
    thread_op: Option<ThreadOp>,
    /// The in-progress Sweep tool dialog (profile chosen, picking path).
    sweep_op: Option<SweepOp>,
    /// The in-progress standalone planar-face Draft command.
    draft_op: Option<DraftOp>,
    /// Memoized live Cut/Join preview: `(input hash, evaluated bodies)`. The
    /// preview re-runs the whole parametric model (truck booleans), which is far
    /// too slow to redo every frame, so it's cached and only recomputed when the
    /// extrude's depth / mode / targets actually change. Cleared when the op ends.
    extrude_preview_cache: Option<(u64, SharedBodyMeshes)>,
    /// Memoized live extrude *tool* ghost (the orange New-Body volume / red Cut
    /// volume): `(input hash, mesh)`. Like `extrude_preview_cache`, it's rebuilt
    /// only when the depth/targets change, not on every repaint (e.g. mouse moves
    /// over the viewport while the dialog is open).
    extrude_preview_mesh_cache: Option<(u64, MockMesh)>,
    /// Tessellated ghost base: `(targets+sign key, build depth, per-target
    /// (plane origin, extrusion axis, mesh))`. Depth changes rescale these
    /// along the axis instead of re-tessellating — curved profiles run the
    /// kernel prism per build, far too slow per drag step. Rebuilt only when
    /// the targets or the depth's sign change.
    extrude_ghost_base: Option<(
        u64,
        f32,
        Vec<(zerocad_core::Vec3, zerocad_core::Vec3, MockMesh)>,
    )>,
    /// In-flight exact extrude preview job. The lightweight tool mesh is shown
    /// until this worker returns the real Cut/Join/overlap-NewBody result.
    extrude_preview_inflight: Option<u64>,
    /// Debounce for exact Cut/Join previews. The lightweight ghost updates every
    /// frame; B-Rep work starts only after this key stays unchanged for 100 ms.
    extrude_preview_settle: Option<(u64, std::time::Instant)>,
    /// Memoized live edge fillet/chamfer preview: `(input hash, bodies)`. The
    /// shared evaluator resolves the temporary edge-mod node in the background;
    /// this cache avoids repeating that solve on every repaint while the size box
    /// is open or the handle is dragged. Recomputed only when the
    /// size/kind/target actually change. Cleared when the op ends.
    edge_mod_preview_cache: Option<(u64, SharedBodyMeshes)>,
    /// Memoized lightweight edge fillet/chamfer overlay mesh shown immediately
    /// while the exact worker-computed preview bodies are still pending.
    edge_mod_preview_mesh_cache: Option<(u64, MockMesh)>,
    /// **Speculative** exact precompute for the live edge mod. Every new size or
    /// kind is submitted immediately on the worker and cached here, keyed by the
    /// same hash `commit_edge_mod` recomputes. If the user commits at that input,
    /// an already-computed exact result can be applied instantly.
    /// Holds `(key, bodies, warnings)`; cleared when the op ends.
    edge_mod_arc_cache: Option<(u64, SharedBodyMeshes, Vec<String>)>,
    /// A small most-recent-first cache of completed speculative solves, keyed by
    /// the same size hash. Scrubbing the size back to a value solved earlier in
    /// this edit finds it here and shows the exact preview instantly instead of
    /// re-solving. Bounded (`EDGE_MOD_ARC_LRU_CAP`); cleared when the op ends.
    edge_mod_arc_lru: Vec<(u64, SharedBodyMeshes, Vec<String>)>,
    /// In-flight speculative arc job key. At most one preview request is active;
    /// a newer request cooperatively cancels the obsolete evaluation.
    edge_mod_arc_inflight: Option<u64>,
    /// Size key whose exact edge-mod solve came back failed (warnings / error).
    /// Stops the per-frame tick from endlessly re-solving a failing blend while
    /// the op stays alive (error in the status bar, ribbon + unchanged body on
    /// screen). A size change makes a new key and retries; cleared with the rest
    /// of the speculation state.
    edge_mod_arc_failed: Option<u64>,
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
    undo_stack: Vec<UndoSnapshot>,
    /// Snapshot stack for Redo (Ctrl+Y / Ctrl+Shift+Z). Cleared whenever a new
    /// destructive change is committed.
    redo_stack: Vec<UndoSnapshot>,
    /// Transaction snapshots for edits made before Finish Sketch. Kept
    /// separately from document undo and capped at the same 50-entry policy.
    working_sketch_undo: Vec<WorkingSketchSnapshot>,

    /// Active shape-dimension dialog (after the first click of a shape).
    dim_input: Option<DimInput>,
    /// Screen anchor (last cursor position) for placing the dimension dialog.
    dim_anchor: Option<egui::Pos2>,
    /// Last cursor position in sketch-plane coordinates (for live dims / finalize).
    last_cursor: Option<(f32, f32)>,
    /// The geometry feature the live cursor snapped onto this frame — drives the
    /// snap glyph drawn over the viewport. `None` when nothing snapped or when
    /// snapping is suppressed (Shift held).
    cursor_snap_kind: Option<SnapKind>,
    /// Woken midline inference guides (≤2). A guide wakes when the cursor snaps
    /// onto a segment midpoint and expires once the cursor drifts off its line.
    snap_guides: Vec<MidlineGuide>,
    /// Dashed guide segments (sketch-plane coords, anchor→snapped point) that the
    /// viewport should draw this frame. Recomputed each hover; empty under Shift.
    cursor_snap_guides: Vec<((f32, f32), (f32, f32))>,
    /// Screen-space positions for inline dimension labels (Fusion 360 style).
    dim_screen_positions: Vec<egui::Pos2>,

    /// Monotonic counter for generating unique feature ids (survives deletes).
    id_counter: usize,

    // Unit settings
    current_unit: Unit,
    /// User-facing viewport controls shared by the sketch inspector and status
    /// bar. These are application preferences, not document data.
    snap_enabled: bool,
    grid_visible: bool,

    /// Whether the Settings window is open.
    show_preferences: bool,
    /// Whether the build/version and offline-report information window is open.
    show_about: bool,
    /// Centralized parameter table, staged until Apply for atomic validation.
    parameters_dialog: Option<ParametersDialog>,
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
    /// Thumbnail textures that became stale during the current frame. They stay
    /// alive until the next frame because egui-wgpu 0.29 destroys freed textures
    /// before submitting the command buffer that may still reference them.
    pending_onboarding_texture_evictions: HashSet<PathBuf>,
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
