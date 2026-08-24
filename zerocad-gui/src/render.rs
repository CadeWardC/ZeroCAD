//! The CPU-projected 3D viewport renderer: depth-sorted painter's-algorithm
//! drawing of solids (with back-face culling), wireframe edges (with hidden-line
//! removal), origin planes, grids and the orientation triad.

use std::collections::HashSet;

use eframe::egui;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureType, MockMesh, ScenePlacement, SketchPlane,
};

use crate::geom2d::{draw_sketch_geometry, fill_nested_loops};
use crate::{
    BodyPick, PendingVisualMode, SectionView, SharedBodyMeshes, SketchPick, SnapKind, ZeroCadApp,
};

const NORMAL_BODY_BASE: (f32, f32, f32) = (190.0, 196.0, 210.0);
// Match the GPU renderer's warm selected-face tint: yellow body fill with the
// stronger orange outline drawn by the selection overlay.
const SELECTED_BODY_BASE: (f32, f32, f32) = (255.0, 199.0, 71.0);
const HOVER_BODY_TINT: (f32, f32, f32) = (85.0, 155.0, 225.0);

fn blend_base(base: (f32, f32, f32), tint: (f32, f32, f32), amount: f32) -> (f32, f32, f32) {
    let keep = 1.0 - amount;
    (
        base.0 * keep + tint.0 * amount,
        base.1 * keep + tint.1 * amount,
        base.2 * keep + tint.2 * amount,
    )
}

/// Weak-perspective camera distance used by every `project_3d` implementation
/// (this file, the viewport pickers, and the GPU `build_view_proj`). One
/// constant, so the CPU and GPU projections can never drift apart.
pub(crate) const PERSP_DIST: f32 = 1200.0;

// Unified render list structures for depth-sorted transparent overlays
#[derive(Debug, Clone)]
struct RenderItem {
    depth: f32,
    content: RenderItemContent,
}

fn section_mesh(mesh: &MockMesh, section: &SectionView) -> (MockMesh, Vec<Vec<[f32; 3]>>) {
    let (origin, normal) = section.plane();
    match zerocad_core::mock_kernel::clip_mesh_by_plane(mesh, origin, normal, section.keep_positive)
    {
        Ok(section) => (section.mesh, section.contours),
        Err(_) => (mesh.clone(), Vec::new()),
    }
}

fn draw_construction_curves(
    painter: &egui::Painter,
    curves: &zerocad_core::SketchCurves,
    to_screen: &dyn Fn((f32, f32)) -> egui::Pos2,
) {
    let stroke = egui::Stroke::new(1.2, egui::Color32::from_rgb(55, 125, 205));
    let draw_polyline = |points: Vec<(f32, f32)>| {
        if points.len() >= 2 {
            let screen: Vec<egui::Pos2> = points.into_iter().map(to_screen).collect();
            painter.add(egui::Shape::dashed_line(&screen, stroke, 5.0, 3.0));
        }
    };
    for segment in &curves.segments {
        draw_polyline(vec![segment.a, segment.b]);
    }
    for circle in &curves.circles {
        draw_polyline(
            (0..=48)
                .map(|index| {
                    let angle = index as f32 / 48.0 * std::f32::consts::TAU;
                    (
                        circle.center.0 + circle.radius * angle.cos(),
                        circle.center.1 + circle.radius * angle.sin(),
                    )
                })
                .collect(),
        );
    }
    for arc in &curves.arcs {
        let start = (arc.start.1 - arc.center.1).atan2(arc.start.0 - arc.center.0);
        let mut end = (arc.end.1 - arc.center.1).atan2(arc.end.0 - arc.center.0);
        while end - start > std::f32::consts::PI {
            end -= std::f32::consts::TAU;
        }
        while end - start < -std::f32::consts::PI {
            end += std::f32::consts::TAU;
        }
        draw_polyline(
            (0..=32)
                .map(|index| {
                    let angle = start + (end - start) * index as f32 / 32.0;
                    (
                        arc.center.0 + arc.radius * angle.cos(),
                        arc.center.1 + arc.radius * angle.sin(),
                    )
                })
                .collect(),
        );
    }
    for spline in &curves.splines {
        draw_polyline(spline.sampled_points(0.01));
    }
}

fn draw_hovered_solver_element(
    painter: &egui::Painter,
    model: &zerocad_core::sketch::SketchSolverModel,
    hovered: zerocad_core::sketch::EntityId,
    to_screen: &dyn Fn((f32, f32)) -> egui::Pos2,
) {
    use zerocad_core::sketch::SketchEntity;

    let edge_stroke = egui::Stroke::new(
        2.35,
        egui::Color32::from_rgba_unmultiplied(65, 145, 210, 145),
    );
    if let Some(point) = model.points.iter().find(|point| point.id == hovered) {
        let center = to_screen((point.pos.0 as f32, point.pos.1 as f32));
        painter.circle_filled(
            center,
            4.0,
            egui::Color32::from_rgba_unmultiplied(125, 180, 225, 90),
        );
        painter.circle_stroke(
            center,
            4.0,
            egui::Stroke::new(
                1.35,
                egui::Color32::from_rgba_unmultiplied(60, 135, 195, 165),
            ),
        );
        return;
    }

    let point = |id| {
        model
            .point(id)
            .map(|point| (point.pos.0 as f32, point.pos.1 as f32))
    };
    let draw_polyline = |points: Vec<(f32, f32)>| {
        for pair in points.windows(2) {
            painter.line_segment([to_screen(pair[0]), to_screen(pair[1])], edge_stroke);
        }
    };
    let Some(entity) = model.entities.iter().find(|entity| entity.id() == hovered) else {
        return;
    };
    match entity {
        SketchEntity::Line { p0, p1, .. } => {
            if let (Some(a), Some(b)) = (point(*p0), point(*p1)) {
                draw_polyline(vec![a, b]);
            }
        }
        SketchEntity::Circle { center, radius, .. } => {
            let Some(center) = point(*center) else {
                return;
            };
            draw_polyline(
                (0..=64)
                    .map(|index| {
                        let angle = index as f32 / 64.0 * std::f32::consts::TAU;
                        (
                            center.0 + *radius as f32 * angle.cos(),
                            center.1 + *radius as f32 * angle.sin(),
                        )
                    })
                    .collect(),
            );
        }
        SketchEntity::Arc {
            center,
            start,
            end,
            radius,
            clockwise,
            ..
        } => {
            let (Some(center), Some(start), Some(end)) =
                (point(*center), point(*start), point(*end))
            else {
                return;
            };
            let arc = zerocad_core::sketch::Arc {
                center,
                radius: *radius as f32,
                start,
                end,
                clockwise: *clockwise,
            };
            draw_polyline(crate::geom2d::sample_arc_points(&arc, 48));
        }
        SketchEntity::Ellipse {
            center,
            major_axis,
            minor_axis,
            start_parameter,
            end_parameter,
            closed,
            ..
        } => {
            let Some(center) = point(*center) else {
                return;
            };
            let sweep = if *closed {
                std::f64::consts::TAU
            } else {
                end_parameter - start_parameter
            };
            draw_polyline(
                (0..=64)
                    .map(|index| {
                        let parameter = start_parameter + sweep * index as f64 / 64.0;
                        let (sin, cos) = parameter.sin_cos();
                        (
                            center.0 + (major_axis[0] * cos + minor_axis[0] * sin) as f32,
                            center.1 + (major_axis[1] * cos + minor_axis[1] * sin) as f32,
                        )
                    })
                    .collect(),
            );
        }
        SketchEntity::Spline {
            points,
            kind,
            degree,
            knots,
            weights,
            closed,
            periodic,
            continuity,
            trim,
            ..
        } => {
            let points: Option<Vec<(f32, f32)>> = points.iter().map(|id| point(*id)).collect();
            let Some(points) = points else {
                return;
            };
            let spline = zerocad_core::Spline {
                kind: *kind,
                points,
                degree: *degree,
                knots: knots.clone(),
                weights: weights.clone(),
                closed: *closed,
                periodic: *periodic,
                continuity: *continuity,
                trim: *trim,
            };
            draw_polyline(spline.sampled_points(0.01));
        }
    }
}

#[derive(Debug, Clone)]
enum RenderItemContent {
    Triangle {
        points: [egui::Pos2; 3],
        /// One color per vertex — egui interpolates them across the triangle
        /// (Gouraud shading), so a smoothly-varying normal across a fillet's
        /// facets reads as one continuous curved surface instead of flat bands.
        colors: [egui::Color32; 3],
    },
    PlaneSheet {
        /// The four quad corners in world space (CCW). The sheet is subdivided and
        /// each cell depth-tested against the solid occlusion buffer, so a sheet
        /// that dips behind a body is hidden there. The screen `points` are no
        /// longer stored — every vertex is re-projected from these world corners.
        world_corners: [[f32; 3]; 4],
        fill_color: egui::Color32,
        border_color: egui::Color32,
        label: String,
        /// Whether this sheet is the hover target (label drawn darker).
        hovered: bool,
    },
}

fn face_uses_selected_material(
    node_id: Option<&str>,
    face_id: u32,
    selected_whole_bodies: &HashSet<&str>,
    selected_faces: &HashSet<(&str, u32)>,
) -> bool {
    let Some(node_id) = node_id else {
        return false;
    };
    selected_whole_bodies.contains(node_id) || selected_faces.contains(&(node_id, face_id))
}

fn selected_material_sets(
    selected_body: &HashSet<(String, BodyPick)>,
) -> (HashSet<&str>, HashSet<(&str, u32)>) {
    let selected_whole_bodies = selected_body
        .iter()
        .filter_map(|(id, pick)| match pick {
            BodyPick::Whole => Some(id.as_str()),
            _ => None,
        })
        .collect();
    let selected_faces = selected_body
        .iter()
        .filter_map(|(id, pick)| match pick {
            BodyPick::Face(fid) => Some((id.as_str(), *fid)),
            _ => None,
        })
        .collect();
    (selected_whole_bodies, selected_faces)
}

/// Whether the CPU hidden-surface buffer is needed for this frame.
///
/// The CPU renderer always needs it for hidden-line removal.  The GPU renderer
/// already owns solid depth, so duplicating every triangle into a software depth
/// grid is useful only for the few CPU overlays that intersect the model.  While
/// the camera is moving those overlays use the same coarse visual LOD as the rest
/// of the viewport and settle to depth-correct rendering on release.
fn needs_cpu_occlusion(
    gpu_active: bool,
    interacting: bool,
    plane_pick_active: bool,
    has_datum_sheets: bool,
    has_depth_clipped_body_highlight: bool,
) -> bool {
    !gpu_active
        || (!interacting
            && (plane_pick_active || has_datum_sheets || has_depth_clipped_body_highlight))
}

#[cfg(test)]
mod selected_material_tests {
    use super::*;

    fn picks(
        items: impl IntoIterator<Item = (&'static str, BodyPick)>,
    ) -> HashSet<(String, BodyPick)> {
        items
            .into_iter()
            .map(|(id, pick)| (id.to_string(), pick))
            .collect()
    }

    #[test]
    fn stale_exact_preview_never_hides_live_additive_tool() {
        assert!(show_live_extrude_tool(Some(ExtrudeMode::Join), false));
        assert!(show_live_extrude_tool(Some(ExtrudeMode::NewBody), false));
        assert!(!show_exact_extrude_result(Some(ExtrudeMode::Join), false));
        assert!(!show_exact_extrude_result(
            Some(ExtrudeMode::NewBody),
            false
        ));
    }

    #[test]
    fn matching_exact_preview_replaces_additive_tool() {
        assert!(!show_live_extrude_tool(Some(ExtrudeMode::Join), true));
        assert!(!show_live_extrude_tool(Some(ExtrudeMode::NewBody), true));
        assert!(show_exact_extrude_result(Some(ExtrudeMode::Join), true));
        assert!(show_exact_extrude_result(Some(ExtrudeMode::NewBody), true));
    }

    #[test]
    fn cut_tool_remains_visible_with_matching_exact_preview() {
        assert!(show_live_extrude_tool(Some(ExtrudeMode::Cut), true));
        assert!(show_exact_extrude_result(Some(ExtrudeMode::Cut), false));
    }

    #[test]
    fn selected_face_tints_only_matching_face_id() {
        let selected = picks([("body-a", BodyPick::Face(7))]);
        let (whole, faces) = selected_material_sets(&selected);

        assert!(face_uses_selected_material(
            Some("body-a"),
            7,
            &whole,
            &faces
        ));
        assert!(!face_uses_selected_material(
            Some("body-a"),
            8,
            &whole,
            &faces
        ));
        assert!(!face_uses_selected_material(
            Some("body-b"),
            7,
            &whole,
            &faces
        ));
    }

    #[test]
    fn selected_whole_body_tints_every_face_for_that_body() {
        let selected = picks([("body-a", BodyPick::Whole)]);
        let (whole, faces) = selected_material_sets(&selected);

        assert!(face_uses_selected_material(
            Some("body-a"),
            1,
            &whole,
            &faces
        ));
        assert!(face_uses_selected_material(
            Some("body-a"),
            42,
            &whole,
            &faces
        ));
        assert!(!face_uses_selected_material(
            Some("body-b"),
            1,
            &whole,
            &faces
        ));
    }

    #[test]
    fn edge_and_vertex_selections_do_not_tint_faces() {
        let selected = picks([
            ("body-a", BodyPick::Edge(2)),
            ("body-a", BodyPick::Vertex(3)),
        ]);
        let (whole, faces) = selected_material_sets(&selected);

        assert!(!face_uses_selected_material(
            Some("body-a"),
            2,
            &whole,
            &faces
        ));
        assert!(!face_uses_selected_material(None, 2, &whole, &faces));
    }

    #[test]
    fn gpu_plain_view_skips_the_software_depth_buffer() {
        assert!(!needs_cpu_occlusion(true, false, false, false, false));
        assert!(!needs_cpu_occlusion(true, true, true, true, true));
    }

    #[test]
    fn overlays_request_software_depth_only_when_camera_is_still() {
        assert!(needs_cpu_occlusion(true, false, true, false, false));
        assert!(needs_cpu_occlusion(true, false, false, true, false));
        assert!(needs_cpu_occlusion(true, false, false, false, true));
        assert!(needs_cpu_occlusion(false, true, false, false, false));
    }

    #[test]
    fn body_highlight_uses_the_moved_preview_geometry() {
        let mut app = ZeroCadApp::new();
        let mesh = MockMesh::make_box(2.0, 2.0, 2.0);
        let original_edge_x = mesh.edge_vertices[0];
        app.set_body_meshes(vec![("body".to_string(), mesh)]);
        app.begin_move_body("body".to_string());
        app.move_op.as_mut().unwrap().translation = [12.0, 3.0, -4.0];
        app.refresh_move_preview();

        let preview = app.move_preview_bodies.as_ref().unwrap();
        let (highlight_mesh, placement) = app
            .body_highlight_geometry("body", Some(preview))
            .expect("preview highlight geometry");

        assert!(placement.is_identity());
        assert!((highlight_mesh.edge_vertices[0] - (original_edge_x + 12.0)).abs() < 1.0e-6);
        let (_, committed_mesh) = app.evaluated_scene.find("body").unwrap();
        assert!((committed_mesh.edge_vertices[0] - original_edge_x).abs() < 1.0e-6);
    }

    #[test]
    fn section_plane_clips_a_body_and_returns_a_closed_cut_contour() {
        let mut graph = zerocad_core::ParametricGraph::new();
        graph.add_feature(zerocad_core::FeatureNode {
            id: "box".to_string(),
            name: "Box".to_string(),
            feature: zerocad_core::FeatureType::Box {
                w: 10.0,
                h: 8.0,
                d: 6.0,
            },
        });
        let mesh = graph.evaluate().expect("box mesh");
        let section = SectionView {
            origin: [0.0; 3],
            normal: [1.0, 0.0, 0.0],
            offset: 5.0,
            keep_positive: true,
            capped: true,
        };
        let (clipped, contours) = section_mesh(&mesh, &section);

        assert!(!clipped.indices.is_empty());
        assert_eq!(contours.len(), 1, "a box section has one closed contour");
        assert!(contours[0].len() >= 4);
        for vertex in clipped.vertices.chunks_exact(6) {
            assert!(vertex[0] >= 5.0 - 1.0e-5, "clipped x={}", vertex[0]);
        }
        for point in &contours[0] {
            assert!((point[0] - 5.0).abs() < 1.0e-5);
        }
    }
}

/// Rasterize one projected triangle's depth into the coarse occlusion buffer,
/// keeping the NEAREST depth per cell (larger `final_z` = nearer the camera).
/// Used only to hide wireframe edges that fall behind solid faces — the fill is
/// still drawn by egui as vector polygons. `origin` is the viewport's top-left in
/// screen space and `cell` the buffer's pixel size.
#[allow(clippy::too_many_arguments)]
fn rasterize_depth(
    zbuf: &mut [f32],
    w: usize,
    h: usize,
    cell: f32,
    origin: egui::Pos2,
    p0: (f32, f32, f32),
    p1: (f32, f32, f32),
    p2: (f32, f32, f32),
) {
    let (ax, ay) = ((p0.0 - origin.x) / cell, (p0.1 - origin.y) / cell);
    let (bx, by) = ((p1.0 - origin.x) / cell, (p1.1 - origin.y) / cell);
    let (gx, gy) = ((p2.0 - origin.x) / cell, (p2.1 - origin.y) / cell);
    let den = (by - gy) * (ax - gx) + (gx - bx) * (ay - gy);
    if den.abs() < 1e-9 {
        return;
    }
    let minx = ax.min(bx).min(gx).floor().max(0.0) as usize;
    let maxx = (ax.max(bx).max(gx).ceil() as i32).min(w as i32 - 1);
    let miny = ay.min(by).min(gy).floor().max(0.0) as usize;
    let maxy = (ay.max(by).max(gy).ceil() as i32).min(h as i32 - 1);
    if maxx < minx as i32 || maxy < miny as i32 {
        return;
    }
    for y in miny..=maxy as usize {
        for x in minx..=maxx as usize {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let wa = ((by - gy) * (fx - gx) + (gx - bx) * (fy - gy)) / den;
            let wb = ((gy - ay) * (fx - gx) + (ax - gx) * (fy - gy)) / den;
            let wc = 1.0 - wa - wb;
            if wa < -0.01 || wb < -0.01 || wc < -0.01 {
                continue;
            }
            let dz = wa * p0.2 + wb * p1.2 + wc * p2.2;
            let idx = y * w + x;
            if dz > zbuf[idx] {
                zbuf[idx] = dz;
            }
        }
    }
}

/// What this frame's live-operation preview looks like, resolved once per frame
/// and shared by BOTH renderers: the CPU painter consumes it as `Drawable`s and
/// the GPU viewport as translucent/opaque scene layers — so the two paths can
/// never disagree about what a preview shows.
pub(crate) struct PreviewPlan {
    /// The extrude mode being previewed, if any.
    pub preview_mode: Option<ExtrudeMode>,
    /// Full replacement body set (the evaluated Join/Cut/edge-mod result);
    /// when `Some`, the committed bodies are NOT drawn.
    pub preview_bodies: Option<SharedBodyMeshes>,
    /// The extrude tool volume (red cut volume / warm additive ghost).
    pub extrude_preview_mesh: Option<MockMesh>,
    /// The lightweight edge-mod overlay ribbon (until exact bodies arrive).
    pub edge_mod_preview_mesh: Option<MockMesh>,
    /// Translucent reflected body shown after a Mirror plane is picked.
    pub mirror_preview_mesh: Option<MockMesh>,
    /// Cosmetic helical ribbon on the selected cylindrical face. The committed
    /// body remains untouched until the Thread dialog is accepted.
    pub thread_preview_mesh: Option<MockMesh>,
    /// Alpha for `preview_bodies` (Cut ghosts the result at 120).
    pub body_alpha: u8,
}

fn show_live_extrude_tool(mode: Option<ExtrudeMode>, exact_preview_is_current: bool) -> bool {
    match mode {
        // Keep the cutter visible through the translucent exact result so the
        // removed volume remains legible.
        Some(ExtrudeMode::Cut) => true,
        // For additive modes, the lightweight tool is the immediate feedback.
        // Only the exact result for these exact inputs may replace it.
        Some(ExtrudeMode::Join | ExtrudeMode::NewBody) => !exact_preview_is_current,
        None => true,
    }
}

/// Whether an evaluated extrude result is allowed into the frame. Additive
/// previews are opaque replacement body sets, so showing an older depth beside
/// the current lightweight tool looks exactly like a second body lagging behind
/// the drag. Withhold it until its input key matches. Cut keeps its translucent
/// last result as context while the current red cutter remains explicit.
fn show_exact_extrude_result(mode: Option<ExtrudeMode>, exact_preview_is_current: bool) -> bool {
    match mode {
        Some(ExtrudeMode::Join | ExtrudeMode::NewBody) => exact_preview_is_current,
        Some(ExtrudeMode::Cut) | None => true,
    }
}

impl ZeroCadApp {
    /// Resolve this frame's operation preview (see [`PreviewPlan`]). Memoized
    /// caches back every branch, so calling it once per renderer path is cheap.
    pub(crate) fn resolve_preview(&mut self) -> PreviewPlan {
        // Meshes to draw: New Body previews keep the warm additive tool volume;
        // Join/Cut previews evaluate a temporary feature so the user sees the
        // actual merged/cut result before committing.
        // A live 3D edge fillet/chamfer previews the resulting (cut) body, the
        // same way a Cut/Join extrude does — and suppresses the extrude preview.
        let mut edge_mod_active = self.edge_mod_op.is_some();
        let mut preview_mode = self.extrude_op.as_ref().map(|op| op.mode);

        // If there is no active operation but we have a pending visual, we can use the mode from the pending visual!
        if !edge_mod_active && preview_mode.is_none() {
            if let Some(pending) = &self.pending_visual {
                match pending.mode {
                    PendingVisualMode::Extrude(mode) => {
                        preview_mode = Some(mode);
                    }
                    PendingVisualMode::EdgeMod => {
                        edge_mod_active = true;
                    }
                }
            }
        }

        // A New Body extrude of genuinely crossing shapes is a boolean
        // (box straddled by a cylinder, two crossing rectangles): the warm ghost
        // would draw the un-fused split regions, so suppress it and show the real
        // evaluated body. A contained circle (a hole) is NOT a crossing — the
        // ghost renders that prism-with-hole exactly and instantly.
        let newbody_overlap = !edge_mod_active
            && preview_mode == Some(ExtrudeMode::NewBody)
            && self.op_has_crossing_shapes();
        // Dense text plates are also exact-only previews. Their sampled ghost
        // makes one lateral face per display chord and previously blocked the
        // UI thread for tens of seconds. The same evaluator used for boolean
        // previews builds the analytic result off-thread instead.
        let newbody_dense_profile = !edge_mod_active
            && preview_mode == Some(ExtrudeMode::NewBody)
            && self
                .extrude_op
                .as_ref()
                .is_some_and(crate::extrude::ExtrudeOp::requires_background_preview);
        let needs_exact_extrude = !edge_mod_active
            && (matches!(preview_mode, Some(ExtrudeMode::Join | ExtrudeMode::Cut))
                || newbody_overlap
                || newbody_dense_profile);
        // The cache may return the last completed body while the worker evaluates
        // the current input. Do not put that stale opaque body into an additive
        // frame: it visibly trails the current lightweight tool as a second solid.
        let cached_exact_extrude_bodies = needs_exact_extrude
            .then(|| self.cached_preview_extrude_bodies())
            .flatten();
        let pending_exact_bodies = self.extrude_op.is_none()
            && self.pending_visual.as_ref().is_some_and(|pending| {
                matches!(pending.mode, PendingVisualMode::Extrude(_)) && pending.exact_bodies
            });
        let exact_preview_is_current = self.has_current_extrude_preview() || pending_exact_bodies;
        let exact_extrude_bodies = cached_exact_extrude_bodies
            .filter(|_| show_exact_extrude_result(preview_mode, exact_preview_is_current));
        let extrude_preview_mesh =
            if edge_mod_active || !show_live_extrude_tool(preview_mode, exact_preview_is_current) {
                None
            } else {
                // Memoized: only re-tessellated when the depth/targets change.
                self.cached_preview_mesh().or_else(|| {
                    if let Some(pending) = &self.pending_visual {
                        if matches!(pending.mode, PendingVisualMode::Extrude(_)) {
                            return pending.mesh.clone();
                        }
                    }
                    None
                })
            };
        let mut preview_bodies = if edge_mod_active {
            // Exact edge-mod bodies arrive from the worker cache. Until then the
            // lightweight overlay mesh below gives immediate visual feedback.
            self.cached_preview_edge_mod_bodies()
        } else {
            // This also runs during push/pull. Additive results enter only when
            // their key matches the current depth; until then the committed scene
            // plus the current lightweight tool is the complete frame.
            exact_extrude_bodies
        };

        // A pending visual bridges an operation that has already committed and
        // closed. Never let one leak into a newly active operation as stale body
        // geometry.
        if preview_bodies.is_none() && self.extrude_op.is_none() && self.edge_mod_op.is_none() {
            if let Some(pending) = &self.pending_visual {
                preview_bodies = Some(pending.bodies.clone().into());
            }
        }

        let edge_mod_preview_mesh =
            if edge_mod_active && self.cached_preview_edge_mod_bodies().is_none() {
                self.cached_preview_edge_mod_mesh().or_else(|| {
                    if let Some(pending) = &self.pending_visual {
                        if matches!(pending.mode, PendingVisualMode::EdgeMod) {
                            return pending.mesh.clone();
                        }
                    }
                    None
                })
            } else {
                None
            };

        // Body movement is a rigid mesh translation, so its preview is exact
        // and immediate. It replaces the committed body set in both renderers.
        if let Some(moved) = self.move_preview_bodies.as_ref() {
            preview_mode = None;
            preview_bodies = Some(moved.clone());
        }

        // A Cut preview ghosts the resulting body (alpha < 255) so the pocket /
        // hole being formed shows through, like Fusion's cut preview. Everything
        // else stays opaque.
        let body_alpha: u8 = if preview_mode == Some(ExtrudeMode::Cut) {
            120
        } else {
            255
        };

        PreviewPlan {
            preview_mode,
            preview_bodies,
            extrude_preview_mesh,
            edge_mod_preview_mesh,
            mirror_preview_mesh: self.mirror_preview_mesh(),
            thread_preview_mesh: self.thread_preview_mesh(),
            body_alpha,
        }
    }

    /// Geometry used by a body selection/preselection overlay.
    ///
    /// Full-body previews replace the committed scene for the current frame, so
    /// their highlights must use the same replacement mesh. Otherwise a moved
    /// body's fill advances while its orange outline remains at the old position.
    fn body_highlight_geometry<'a>(
        &'a self,
        node_id: &str,
        preview_bodies: Option<&'a SharedBodyMeshes>,
    ) -> Option<(&'a MockMesh, ScenePlacement)> {
        if let Some((_, mesh)) = preview_bodies
            .and_then(|bodies| bodies.iter().find(|(preview_id, _)| preview_id == node_id))
        {
            return Some((mesh, ScenePlacement::IDENTITY));
        }

        let (instance, mesh) = self.evaluated_scene.find(node_id)?;
        Some((mesh, instance.placement()))
    }

    /// A high-performance, robust, and clean CPU-projected vector viewport drawing engine
    /// utilizing egui's native vector Painter.
    pub(crate) fn draw_viewport(
        &mut self,
        painter: egui::Painter,
        rect: egui::Rect,
        _hover_pos: Option<egui::Pos2>,
        current_cursor_snap: Option<(f32, f32)>,
    ) {
        let center_x = rect.center().x + self.camera_pan.x;
        let center_y = rect.center().y + self.camera_pan.y;
        let view_scale = rect.width().min(rect.height()) / (self.camera_zoom * 5.0);

        // Level-of-detail while the camera is in motion (orbit/pan drag or an
        // animated view transition). During motion the eye can't resolve fine
        // detail, so we drop the cosmetic per-triangle seam strokes and march the
        // hidden-line wireframe in coarser steps — this is what keeps orbiting a
        // big model smooth. When motion stops, the next repaint draws full detail.
        let interacting = self.orbiting || self.camera_anim_active;

        // When the GPU viewport produced a scene texture this frame, it owns the
        // shaded solids + wireframe; the CPU pipeline below then skips drawing the
        // committed bodies and only paints the 2D overlays (planes, sketches,
        // dimensions, gizmos) on top.
        let gpu_active = self.gpu_texture_id.is_some();

        // Compute Orthographic or Perspective Projection matrices
        let cos_p = self.camera_pitch.cos();
        let sin_p = self.camera_pitch.sin();
        let cos_y = self.camera_yaw.cos();
        let sin_y = self.camera_yaw.sin();

        // 3D coordinate projection mapping function. Captures `is_perspective` by
        // value (not via `self`) so the closure borrows nothing from `self` — that
        // lets the live-preview cache be refreshed (&mut self) mid-frame.
        let is_perspective = self.is_perspective;
        let project_3d = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
            let rx = cos_y * x - sin_y * z;
            let rz = sin_y * x + cos_y * z;
            let ry = cos_p * y - sin_p * rz;
            let final_z = sin_p * y + cos_p * rz; // depth for z-sorting

            if is_perspective {
                let dist = PERSP_DIST;
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

        // --- 1. DRAW CAD BACKGROUND ---
        // Keep the modeling canvas in the same semantic surface family as the
        // surrounding application chrome in both themes.
        let viewport_background = if self.dark_mode {
            egui::Color32::from_rgb(15, 23, 42)
        } else {
            egui::Color32::from_rgb(245, 246, 248)
        };
        painter.rect_filled(rect, 0.0, viewport_background);

        // Define coordinates of the 3 origin sheets in 3D
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
        let xy_depth = (xy_c[0].2 + xy_c[1].2 + xy_c[2].2 + xy_c[3].2) / 4.0;

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
        let xz_depth = (xz_c[0].2 + xz_c[1].2 + xz_c[2].2 + xz_c[3].2) / 4.0;

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
        let yz_depth = (yz_c[0].2 + yz_c[1].2 + yz_c[2].2 + yz_c[3].2) / 4.0;

        // Gather sheets for depth sorting.
        // Hovered color: semi-transparent grayish-blue.
        // Unhovered color: transparent orange/yellow.
        let fill_xy = if self.hovered_plane == Some(SketchPlane::XY) {
            egui::Color32::from_rgba_unmultiplied(100, 150, 240, 120) // grayish blue
        } else {
            egui::Color32::from_rgba_unmultiplied(240, 180, 100, 35) // transparent orange
        };
        let stroke_xy = if self.hovered_plane == Some(SketchPlane::XY) {
            egui::Color32::from_rgb(0, 120, 215) // active blue
        } else {
            egui::Color32::from_rgb(230, 150, 50) // orange border
        };

        let fill_xz = if self.hovered_plane == Some(SketchPlane::XZ) {
            egui::Color32::from_rgba_unmultiplied(100, 150, 240, 120)
        } else {
            egui::Color32::from_rgba_unmultiplied(240, 180, 100, 35)
        };
        let stroke_xz = if self.hovered_plane == Some(SketchPlane::XZ) {
            egui::Color32::from_rgb(0, 120, 215)
        } else {
            egui::Color32::from_rgb(230, 150, 50)
        };

        let fill_yz = if self.hovered_plane == Some(SketchPlane::YZ) {
            egui::Color32::from_rgba_unmultiplied(100, 150, 240, 120)
        } else {
            egui::Color32::from_rgba_unmultiplied(240, 180, 100, 35)
        };
        let stroke_yz = if self.hovered_plane == Some(SketchPlane::YZ) {
            egui::Color32::from_rgb(0, 120, 215)
        } else {
            egui::Color32::from_rgb(230, 150, 50)
        };

        // World-space corners of each sheet, in the same winding as the projected
        // `*_pts`, so each cell can be re-projected and depth-tested.
        let xy_world = [
            [0.0, 0.0, 0.0],
            [size, 0.0, 0.0],
            [size, size, 0.0],
            [0.0, size, 0.0],
        ];
        let xz_world = [
            [0.0, 0.0, 0.0],
            [size, 0.0, 0.0],
            [size, 0.0, size],
            [0.0, 0.0, size],
        ];
        let yz_world = [
            [0.0, 0.0, 0.0],
            [0.0, size, 0.0],
            [0.0, size, size],
            [0.0, 0.0, size],
        ];

        let planes_to_draw = vec![
            (
                xy_pts,
                xy_world,
                xy_depth,
                SketchPlane::XY,
                fill_xy,
                stroke_xy,
                "", // Clean: no text label printed inside the viewport
            ),
            (
                xz_pts,
                xz_world,
                xz_depth,
                SketchPlane::XZ,
                fill_xz,
                stroke_xz,
                "",
            ),
            (
                yz_pts,
                yz_world,
                yz_depth,
                SketchPlane::YZ,
                fill_yz,
                stroke_yz,
                "",
            ),
        ];

        // Datum plane sheets: always visible reference geometry (violet), lit up
        // like the origin sheets while picking a sketch plane. World corners come
        // from the shared helper so hover hit-testing (viewport) and drawing agree.
        let datum_sheets: Vec<(String, String, [[f32; 3]; 4], f32)> = self
            .resolved_datum_planes()
            .into_iter()
            .map(|(id, name, cs)| {
                let w = Self::datum_plane_world_corners(&cs);
                let depth = w
                    .iter()
                    .map(|c| project_3d(c[0], c[1], c[2]).2)
                    .sum::<f32>()
                    / 4.0;
                (id, name, w, depth)
            })
            .collect();

        // --- 2. GROUND REFERENCE GRID & CENTRAL AXES (when not in a planar mode) ---
        if self.grid_visible && !self.is_planar_view() {
            // Floor grid lies flat on the XZ plane (y = 0), drawn as a disc that
            // FADES OUT toward its rim rather than as full-length lines running to
            // the horizon. The old version drew every line across a 600mm span, so
            // in perspective they all converged toward the vanishing points and
            // read as bright rays fanning out from behind the body. Clipping each
            // line to a radius and fading its alpha by distance keeps the ground
            // readable near the model and lets it dissolve into the background.
            let r_grid = 150.0f32;
            let step = 2.5f32;
            // Fade a line span (on the floor, y=0) by each sub-segment's distance
            // from the origin, so the grid melts away at the rim with no hard edge
            // and no converging rays.
            let faded_floor_line =
                |ax: f32, az: f32, bx: f32, bz: f32, width: f32, base_alpha: f32| {
                    const SUBS: usize = 10;
                    for s in 0..SUBS {
                        let t0 = s as f32 / SUBS as f32;
                        let t1 = (s + 1) as f32 / SUBS as f32;
                        let (x0, z0) = (ax + (bx - ax) * t0, az + (bz - az) * t0);
                        let (x1, z1) = (ax + (bx - ax) * t1, az + (bz - az) * t1);
                        let mr = (((x0 + x1) * 0.5).powi(2) + ((z0 + z1) * 0.5).powi(2)).sqrt();
                        let fade = (1.0 - mr / r_grid).clamp(0.0, 1.0).powf(1.5);
                        let a = (base_alpha * fade) as u8;
                        if a <= 3 {
                            continue;
                        }
                        let p0 = project_3d(x0, 0.0, z0);
                        let p1 = project_3d(x1, 0.0, z1);
                        painter.line_segment(
                            [egui::pos2(p0.0, p0.1), egui::pos2(p1.0, p1.1)],
                            egui::Stroke::new(
                                width,
                                egui::Color32::from_rgba_unmultiplied(95, 100, 122, a),
                            ),
                        );
                    }
                };
            let mut i = -r_grid;
            while i <= r_grid + 0.001 {
                // Clip each line to the grid disc so it never reaches the horizon.
                let inside = r_grid * r_grid - i * i;
                if inside > 0.0 {
                    let half = inside.sqrt();
                    let is_major = (i / 50.0).round() * 50.0 == i || i.abs() < 0.001;
                    let (w, base) = if is_major { (1.2, 150.0) } else { (1.0, 120.0) };
                    faded_floor_line(i, -half, i, half, w, base);
                    faded_floor_line(-half, i, half, i, w, base);
                }
                i += step;
            }

            // Draw central 3D coordinate axis lines
            let axis_length = 90.0;
            let px1 = project_3d(-axis_length, 0.0, 0.0);
            let px2 = project_3d(axis_length, 0.0, 0.0);
            painter.line_segment(
                [egui::pos2(px1.0, px1.1), egui::pos2(px2.0, px2.1)],
                egui::Stroke::new(1.8, egui::Color32::from_rgba_unmultiplied(220, 50, 50, 190)),
            ); // X-Red

            // (Y-Green vertical axis intentionally not drawn — it ran straight
            // through bodies sitting on the ground and cluttered the view.)

            let pz1 = project_3d(0.0, 0.0, -axis_length);
            let pz2 = project_3d(0.0, 0.0, axis_length);
            painter.line_segment(
                [egui::pos2(pz1.0, pz1.1), egui::pos2(pz2.0, pz2.1)],
                egui::Stroke::new(1.8, egui::Color32::from_rgba_unmultiplied(50, 50, 220, 190)),
            ); // Z-Blue
        }

        // Datum axes and points — persistent reference geometry, drawn as a
        // violet line (axis) / ring marker (point) with a name label.
        for (id, name, value) in self.resolved_datum_axes_points() {
            let hidden = self.hidden_nodes.contains(&id);
            if hidden {
                continue;
            }
            let violet = egui::Color32::from_rgba_unmultiplied(140, 90, 230, 200);
            match value {
                zerocad_core::DatumValue::Axis { origin, dir } => {
                    let half = 40.0;
                    let a = origin.sub(dir.mul(half));
                    let b = origin.add(dir.mul(half));
                    let pa = project_3d(a.x, a.y, a.z);
                    let pb = project_3d(b.x, b.y, b.z);
                    // Dash the line manually (egui has no dashed stroke here).
                    const DASHES: usize = 24;
                    for s in 0..DASHES {
                        if s % 2 == 1 {
                            continue;
                        }
                        let t0 = s as f32 / DASHES as f32;
                        let t1 = (s + 1) as f32 / DASHES as f32;
                        let lerp =
                            |t: f32| egui::pos2(pa.0 + (pb.0 - pa.0) * t, pa.1 + (pb.1 - pa.1) * t);
                        painter.line_segment([lerp(t0), lerp(t1)], egui::Stroke::new(1.6, violet));
                    }
                    painter.text(
                        egui::pos2(pb.0, pb.1),
                        egui::Align2::LEFT_BOTTOM,
                        name,
                        egui::FontId::proportional(10.5),
                        violet,
                    );
                }
                zerocad_core::DatumValue::Point(p) => {
                    let pp = project_3d(p.x, p.y, p.z);
                    painter.circle_stroke(
                        egui::pos2(pp.0, pp.1),
                        4.0,
                        egui::Stroke::new(1.6, violet),
                    );
                    painter.text(
                        egui::pos2(pp.0 + 6.0, pp.1),
                        egui::Align2::LEFT_CENTER,
                        name,
                        egui::FontId::proportional(10.5),
                        violet,
                    );
                }
                zerocad_core::DatumValue::Plane(_) => {}
            }
        }

        // --- 3. 3D PROJECTED ACTIVE PLANE GRID (drawn behind solid parts) ---
        // The grid is drawn in the active plane's own (u, v) coordinates and
        // unprojected to 3D, so it aligns to an origin plane OR a body face.
        let grid_cs: Option<CoordinateSystem> =
            if self.extrude_op.is_some() || self.edge_mod_op.is_some() {
                // While pushing/pulling an extrude or edge mod the sketch grid isn't
                // needed — and viewed at the oblique angle those ops are done from, its
                // far lines fan into long rays across the model. Hide it.
                None
            } else if self.is_sketch_mode {
                Some(self.active_sketch_cs)
            } else if self.plane_pick_active() {
                self.hovered_plane.map(|p| match p {
                    SketchPlane::XY => CoordinateSystem::XY,
                    SketchPlane::XZ => CoordinateSystem::XZ,
                    SketchPlane::YZ => CoordinateSystem::YZ,
                })
            } else {
                None
            };

        if self.grid_visible {
            if let Some(cs) = grid_cs {
                let r_grid = 150.0f32;
                let step = 5.0f32;

                // Fade the whole grid out as its plane turns edge-on to the camera.
                // A grid seen at a grazing angle is unreadable and its lines pile into
                // converging rays; `facing` is |n · view| (1 = looking straight at the
                // plane, 0 = edge-on). Full grid within ~50° of head-on, gone by ~80°.
                let facing = (sin_p * cs.n.y + cos_p * (sin_y * cs.n.x + cos_y * cs.n.z)).abs();
                let facing_mul = ((facing - 0.18) / 0.45).clamp(0.0, 1.0);
                if facing_mul <= 0.01 {
                    // Effectively edge-on — skip the grid entirely this frame.
                } else {
                    // Fade + clip each plane-grid line the same way the floor grid does:
                    // clip it to a disc of radius `r_grid` and fade each sub-segment by its
                    // distance from the plane origin. Drawing the grid full-span instead
                    // made every line run to ±150 and, on a VERTICAL sketch plane viewed
                    // in perspective, converge into bright rays fanning far above and
                    // below the model (visible straight through a fresh cut, too) — the
                    // exact artifact the floor grid was already fixed for.
                    let faded_plane_line =
                        |u0: f32, v0: f32, u1: f32, v1: f32, width: f32, base_alpha: f32| {
                            const SUBS: usize = 10;
                            for s in 0..SUBS {
                                let t0 = s as f32 / SUBS as f32;
                                let t1 = (s + 1) as f32 / SUBS as f32;
                                let (au, av) = (u0 + (u1 - u0) * t0, v0 + (v1 - v0) * t0);
                                let (bu, bv) = (u0 + (u1 - u0) * t1, v0 + (v1 - v0) * t1);
                                let mr =
                                    (((au + bu) * 0.5).powi(2) + ((av + bv) * 0.5).powi(2)).sqrt();
                                let fade = (1.0 - mr / r_grid).clamp(0.0, 1.0).powf(1.5);
                                let a = (base_alpha * fade) as u8;
                                if a <= 3 {
                                    continue;
                                }
                                let wa = cs.unproject(au, av);
                                let wb = cs.unproject(bu, bv);
                                let pa = project_3d(wa.x, wa.y, wa.z);
                                let pb = project_3d(wb.x, wb.y, wb.z);
                                painter.line_segment(
                                    [egui::pos2(pa.0, pa.1), egui::pos2(pb.0, pb.1)],
                                    egui::Stroke::new(
                                        width,
                                        egui::Color32::from_rgba_unmultiplied(110, 110, 124, a),
                                    ),
                                );
                            }
                        };

                    let mut i = -r_grid;
                    while i <= r_grid + 0.001 {
                        // Clip each line to the grid disc so it never reaches the horizon.
                        let inside = r_grid * r_grid - i * i;
                        if inside > 0.0 {
                            let half = inside.sqrt();
                            // The lines through the plane origin read as its major axes.
                            let is_major = i.abs() < 0.001;
                            let (w, base) = if is_major { (1.2, 90.0) } else { (1.0, 40.0) };
                            let base = base * facing_mul;
                            faded_plane_line(i, -half, i, half, w, base);
                            faded_plane_line(-half, i, half, i, w, base);
                        }
                        i += step;
                    }
                }
            }
        }

        // Composite the GPU-rendered 3D scene here — after the floor/plane grids,
        // axes, and datum marks (which sit BEHIND solids, exactly as when the CPU
        // paints the bodies at this point), and before the overlays that sit on
        // top. The texture's background is transparent, so everything painted
        // above shows through where there is no geometry — bodies read as solid,
        // never see-through.
        if let Some(tex) = self.gpu_texture_id {
            let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
            painter.image(tex, rect, uv, egui::Color32::WHITE);
        }

        // --- 4. DEPTH-SORTED RENDER LIST PIPELINE ---
        let mut render_items = Vec::new();

        // This frame's operation preview, resolved by the shared planner (the
        // GPU path feeds the same plan into its scene layers, so both renderers
        // always agree on what a preview shows). When the GPU path ran this
        // frame it already resolved the plan — reuse it instead of re-cloning
        // the preview body set a second time.
        let PreviewPlan {
            preview_mode,
            preview_bodies,
            extrude_preview_mesh,
            edge_mod_preview_mesh,
            mirror_preview_mesh,
            thread_preview_mesh,
            body_alpha,
        } = self
            .frame_preview_plan
            .take()
            .unwrap_or_else(|| self.resolve_preview());
        // Each drawable records HOW to render it. Translucent body ghosts keep
        // their back faces (so the far walls of a pocket show through) and draw
        // their wireframe. The red cut-tool VOLUME is different: it culls its
        // back faces and draws no wireframe, so it reads as one clean translucent
        // solid instead of a crisscross of overlapping front/back triangles
        // (the X-shaped "faint triangles") fringed by dark outline spikes poking
        // out of the body.
        struct Drawable<'a> {
            node_id: Option<&'a str>,
            mesh: &'a MockMesh,
            placement: ScenePlacement,
            base: (f32, f32, f32),
            alpha: u8,
            cull_back: bool,
            draw_edges: bool,
            /// Sort after everything else (x-ray): for previews of surfaces
            /// that lie INSIDE the body (the fillet/chamfer band), which the
            /// depth sort would otherwise bury behind the walls around them.
            on_top: bool,
        }
        let (sectioned_bodies, section_contours): (
            Option<Vec<(String, MockMesh)>>,
            Vec<Vec<[f32; 3]>>,
        ) = if let Some(section) = &self.section_view {
            let mut bodies = Vec::new();
            let mut contours = Vec::new();
            if let Some(source) = preview_bodies.as_ref() {
                bodies.reserve(source.len());
                for (id, mesh) in source.iter() {
                    let (clipped, mut loops) = section_mesh(mesh, section);
                    bodies.push((id.clone(), clipped));
                    contours.append(&mut loops);
                }
            } else {
                bodies.reserve(self.evaluated_scene.instances().len());
                for instance in self.evaluated_scene.instances() {
                    let mesh = self.evaluated_scene.mesh(instance);
                    let placement = instance.placement();
                    let transformed;
                    let world_mesh = if placement.is_identity() {
                        mesh
                    } else {
                        transformed = placement.transform_mesh(mesh);
                        &transformed
                    };
                    let (clipped, mut loops) = section_mesh(world_mesh, section);
                    bodies.push((instance.entity_id().to_string(), clipped));
                    contours.append(&mut loops);
                }
            }
            (Some(bodies), contours)
        } else {
            (None, Vec::new())
        };
        let mut meshes: Vec<Drawable> = if gpu_active {
            // The GPU composited this frame's full scene — committed bodies AND
            // live preview layers — so the CPU paints no solids at all.
            Vec::new()
        } else if let Some(bodies) = sectioned_bodies.as_ref() {
            let alpha = if preview_bodies.is_some() {
                body_alpha
            } else {
                255
            };
            bodies
                .iter()
                .map(|(id, m)| Drawable {
                    node_id: Some(id.as_str()),
                    mesh: m,
                    placement: ScenePlacement::IDENTITY,
                    base: NORMAL_BODY_BASE,
                    alpha,
                    cull_back: alpha == 255,
                    draw_edges: true,
                    on_top: false,
                })
                .collect()
        } else if let Some(bodies) = preview_bodies.as_ref() {
            bodies
                .iter()
                .map(|(id, m)| Drawable {
                    node_id: Some(id.as_str()),
                    mesh: m,
                    placement: ScenePlacement::IDENTITY,
                    base: NORMAL_BODY_BASE,
                    alpha: body_alpha,
                    // A translucent ghost (Cut result) keeps its back faces.
                    cull_back: body_alpha == 255,
                    draw_edges: true,
                    on_top: false,
                })
                .collect()
        } else {
            self.evaluated_scene
                .instances()
                .iter()
                .map(|instance| Drawable {
                    node_id: Some(instance.entity_id()),
                    mesh: self.evaluated_scene.mesh(instance),
                    placement: instance.placement(),
                    base: NORMAL_BODY_BASE,
                    alpha: 255,
                    cull_back: true,
                    draw_edges: true,
                    on_top: false,
                })
                .collect()
        };
        // The preview tool volumes / overlay ribbon join the CPU draw list only
        // when the CPU is painting solids; with the GPU active they are drawn
        // as GPU scene layers instead (see `render_gpu_scene`).
        match preview_mode {
            _ if gpu_active => {}
            // Cut: lay the FULL cut volume over the ghosted result in translucent
            // red, so the user sees exactly how far the cut reaches — including
            // where it punches out the far side of a body. Back faces culled +
            // no wireframe so it stays a clean translucent red solid.
            Some(ExtrudeMode::Cut) => {
                if let Some(pm) = extrude_preview_mesh.as_ref() {
                    meshes.push(Drawable {
                        node_id: None,
                        mesh: pm,
                        placement: ScenePlacement::IDENTITY,
                        base: (232.0, 66.0, 66.0),
                        alpha: 90,
                        cull_back: true,
                        draw_edges: false,
                        on_top: false,
                    });
                }
            }
            // The plan supplies this tool only while the matching exact Join
            // result is unavailable. Keep its real preview outline visible;
            // the exact fused body replaces this layer when it arrives.
            Some(ExtrudeMode::Join) => {
                if let Some(pm) = extrude_preview_mesh.as_ref() {
                    meshes.push(Drawable {
                        node_id: None,
                        mesh: pm,
                        placement: ScenePlacement::IDENTITY,
                        base: (255.0, 178.0, 96.0),
                        alpha: 255,
                        cull_back: true,
                        draw_edges: true,
                        on_top: false,
                    });
                }
            }
            // New Body: the warm additive tool volume floats over the live model.
            Some(ExtrudeMode::NewBody) | None => {
                if let Some(pm) = extrude_preview_mesh.as_ref() {
                    meshes.push(Drawable {
                        node_id: None,
                        mesh: pm,
                        placement: ScenePlacement::IDENTITY,
                        base: (255.0, 178.0, 96.0),
                        alpha: 255,
                        cull_back: true,
                        draw_edges: true,
                        on_top: false,
                    });
                }
            }
        }
        if !gpu_active {
            if let Some(mesh) = mirror_preview_mesh.as_ref() {
                meshes.push(Drawable {
                    node_id: None,
                    mesh,
                    placement: ScenePlacement::IDENTITY,
                    base: (70.0, 170.0, 245.0),
                    alpha: 115,
                    cull_back: false,
                    draw_edges: true,
                    on_top: false,
                });
            }
        }
        if !gpu_active {
            if let Some(mesh) = thread_preview_mesh.as_ref() {
                meshes.push(Drawable {
                    node_id: None,
                    mesh,
                    placement: ScenePlacement::IDENTITY,
                    base: (70.0, 145.0, 245.0),
                    alpha: 220,
                    cull_back: false,
                    draw_edges: true,
                    on_top: true,
                });
            }
        }
        if let Some(pm) = edge_mod_preview_mesh.as_ref() {
            if !gpu_active {
                meshes.push(Drawable {
                    node_id: None,
                    mesh: pm,
                    placement: ScenePlacement::IDENTITY,
                    base: (255.0, 178.0, 96.0),
                    alpha: 200,
                    cull_back: false,
                    draw_edges: true,
                    // The blend band lies inside the body — depth-sorted it
                    // would be buried behind the walls around it.
                    on_top: true,
                });
            }
        }

        // Screen anchor for the inline extrude distance box: the projected
        // centroid of the live preview, nudged up-right so it floats clear of
        // the body. Applied to `self.extrude_dim_pos` at the end of the frame
        // (the `meshes`/`project_3d` borrows of `self` are still live here).
        let mut extrude_anchor: Option<egui::Pos2> = None;
        if let Some(pm) = extrude_preview_mesh.as_ref() {
            let vcount = pm.vertices.len() / 6;
            if vcount > 0 {
                let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
                for v in 0..vcount {
                    sx += pm.vertices[v * 6];
                    sy += pm.vertices[v * 6 + 1];
                    sz += pm.vertices[v * 6 + 2];
                }
                let n = vcount as f32;
                let c = project_3d(sx / n, sy / n, sz / n);
                extrude_anchor = Some(egui::pos2(c.0 + 55.0, c.1 - 18.0));
            }
        }

        // Screen anchor for the inline edge fillet/chamfer size box: the projected
        // midpoint of the selected edge, nudged clear of the body. Also the drag
        // manipulator: a handle offset along the edge's outward bisector (away
        // from the body), whose screen axis carries px-per-mm so the drag handler
        // can convert pixels back to millimetres.
        let mut edge_mod_anchor: Option<egui::Pos2> = None;
        let mut edge_mod_handle: Option<(egui::Pos2, egui::Pos2, egui::Vec2)> = None;
        if let Some(op) = self.edge_mod_op.as_ref() {
            let m = op.edge_midpoint();
            let c = project_3d(m[0], m[1], m[2]);
            let mid_s = egui::pos2(c.0, c.1);
            edge_mod_anchor = Some(egui::pos2(c.0 + 40.0, c.1 - 14.0));

            // Outward bisector of the two adjacent faces (points away from the
            // body for a convex edge) → the direction that grows the radius. Anchors
            // on the primary (first) edge when several are being filleted at once.
            let (n1, n2) = (op.primary().n1, op.primary().n2);
            let mut o = [n1[0] + n2[0], n1[1] + n2[1], n1[2] + n2[2]];
            let ol = (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt();
            if ol > 1.0e-6 {
                o = [o[0] / ol, o[1] / ol, o[2] / ol];
            }
            let b = project_3d(m[0] + o[0], m[1] + o[1], m[2] + o[2]);
            let out_vec = egui::vec2(b.0 - c.0, b.1 - c.1); // px per 1mm outward
            let ppm = out_vec.length();
            // Foreshortened edge-on view → fall back to a vertical screen axis.
            let (unit, axis) = if ppm > 1.0 {
                (out_vec / ppm, out_vec)
            } else {
                (egui::vec2(0.0, -1.0), egui::vec2(0.0, -view_scale.max(1.0)))
            };
            let hpos = mid_s + unit * 34.0;
            edge_mod_handle = Some((mid_s, hpos, axis));
        }

        // Screen anchor for the inline 2D corner-radius box (set in section 6,
        // where the sketch-plane projection is available): the last staged
        // corner, else the live cursor. `corner_handle` carries the matching drag
        // manipulator (corner, handle, px-per-mm bisector axis).
        let mut corner_anchor: Option<egui::Pos2> = None;
        let mut corner_handle: Option<(egui::Pos2, egui::Pos2, egui::Vec2)> = None;

        // Depth buffer for wireframe hidden-line removal. Painter's per-triangle
        // depth sort can't reliably hide an edge that lies behind a large tilted
        // face — the face's centroid depth misrepresents its nearness at the edge —
        // so back edges x-ray through the solid. We rasterize the OPAQUE surface
        // into a coarse depth grid (section A), then in section F draw only the
        // edge spans that aren't behind a nearer face. Larger `final_z` = nearer.
        let has_depth_clipped_body_highlight = self
            .selected_body
            .iter()
            .any(|(_, pick)| matches!(pick, BodyPick::Whole | BodyPick::Edge(_)));
        let build_occlusion = needs_cpu_occlusion(
            gpu_active,
            interacting,
            self.plane_pick_active(),
            !datum_sheets.is_empty(),
            has_depth_clipped_body_highlight,
        );
        let (occ_cell, occ_w, occ_h) = if build_occlusion {
            let bw_full = rect.width().ceil().max(1.0) as usize;
            let bh_full = rect.height().ceil().max(1.0) as usize;
            // ~2px cells, scaled up on hi-dpi / fullscreen so the buffer stays small.
            let cell = (((bw_full.max(bh_full) + 1023) / 1024).max(1) * 2) as f32;
            (
                cell,
                (bw_full as f32 / cell) as usize + 2,
                (bh_full as f32 / cell) as usize + 2,
            )
        } else {
            // Keep the shared `occluded` closure branch-free.  A one-cell empty
            // buffer answers "visible" for every point at negligible cost.
            (f32::MAX, 1, 1)
        };
        let mut zbuf = vec![f32::NEG_INFINITY; occ_w * occ_h];
        let (mut depth_min, mut depth_max) = (f32::INFINITY, f32::NEG_INFINITY);

        let (selected_whole_bodies, mut selected_faces) =
            selected_material_sets(&self.selected_body);
        // While picking a sketch plane, preview the hovered planar body face as
        // selected so the user sees which face a click will sketch on.
        if self.plane_pick_active() {
            if let Some((node, fid)) = self.hovered_sketch_face.as_ref() {
                selected_faces.insert((node.as_str(), *fid));
            }
        }

        // A0. GPU-composited mode: the solids are painted by the GPU texture
        // (`meshes` is empty), but the plane sheets/grids and any remaining CPU
        // strokes still consult this occlusion buffer to hide whatever dips
        // behind a solid. Rasterize the front-facing depth of exactly what the
        // GPU drew opaquely — the preview result set when one replaces the
        // committed model (a translucent Cut ghost doesn't occlude, matching
        // the CPU path), else the committed bodies — with the identical
        // projection, so overlays keep the same hidden-surface behavior in
        // both render modes.
        if gpu_active && build_occlusion {
            let mut occluder_meshes: Vec<(&MockMesh, ScenePlacement)> = Vec::new();
            if let Some(bodies) = preview_bodies.as_ref() {
                if body_alpha == 255 {
                    occluder_meshes.extend(
                        bodies
                            .iter()
                            .map(|(_, mesh)| (mesh, ScenePlacement::IDENTITY)),
                    );
                }
            } else {
                occluder_meshes.extend(
                    self.evaluated_scene.instances().iter().map(|instance| {
                        (self.evaluated_scene.mesh(instance), instance.placement())
                    }),
                );
            }
            // The warm extrude tool ghost is drawn OPAQUE (depth-written) by the
            // GPU — New Body always, Join while push/pull dragging — so it must
            // occlude overlays exactly like a committed body. The translucent
            // red Cut volume and the settled-Join case (no ghost layer) don't.
            let warm_ghost_opaque = match preview_mode {
                Some(ExtrudeMode::Cut) => false,
                Some(ExtrudeMode::Join) => self.extrude_depth_dragging,
                Some(ExtrudeMode::NewBody) | None => true,
            };
            if warm_ghost_opaque {
                if let Some(m) = extrude_preview_mesh.as_ref() {
                    occluder_meshes.push((m, ScenePlacement::IDENTITY));
                }
            }
            for (mesh, placement) in occluder_meshes {
                let num_tris = mesh.indices.len() / 3;
                for i in 0..num_tris {
                    let i0 = mesh.indices[i * 3] as usize * 6;
                    let i1 = mesh.indices[i * 3 + 1] as usize * 6;
                    let i2 = mesh.indices[i * 3 + 2] as usize * 6;
                    // Average vertex normal decides front/back, as in section A.
                    let navg = placement.transform_vector([
                        (mesh.vertices[i0 + 3] + mesh.vertices[i1 + 3] + mesh.vertices[i2 + 3])
                            / 3.0,
                        (mesh.vertices[i0 + 4] + mesh.vertices[i1 + 4] + mesh.vertices[i2 + 4])
                            / 3.0,
                        (mesh.vertices[i0 + 5] + mesh.vertices[i1 + 5] + mesh.vertices[i2 + 5])
                            / 3.0,
                    ]);
                    let rz_n = sin_y * navg[0] + cos_y * navg[2];
                    if sin_p * navg[1] + cos_p * rz_n <= 0.0 {
                        continue;
                    }
                    let point0 = placement.transform_point([
                        mesh.vertices[i0],
                        mesh.vertices[i0 + 1],
                        mesh.vertices[i0 + 2],
                    ]);
                    let point1 = placement.transform_point([
                        mesh.vertices[i1],
                        mesh.vertices[i1 + 1],
                        mesh.vertices[i1 + 2],
                    ]);
                    let point2 = placement.transform_point([
                        mesh.vertices[i2],
                        mesh.vertices[i2 + 1],
                        mesh.vertices[i2 + 2],
                    ]);
                    let p0 = project_3d(point0[0], point0[1], point0[2]);
                    let p1 = project_3d(point1[0], point1[1], point1[2]);
                    let p2 = project_3d(point2[0], point2[1], point2[2]);
                    for p in [p0, p1, p2] {
                        depth_min = depth_min.min(p.2);
                        depth_max = depth_max.max(p.2);
                    }
                    rasterize_depth(&mut zbuf, occ_w, occ_h, occ_cell, rect.min, p0, p1, p2);
                }
            }
        }

        // A. Gather Solid Mesh Triangles (committed model + extrude preview)
        for d in &meshes {
            let (mesh, alpha, cull_back) = (d.mesh, &d.alpha, d.cull_back);
            let placement = d.placement;
            let num_tris = mesh.indices.len() / 3;
            for i in 0..num_tris {
                let i0 = mesh.indices[i * 3] as usize * 6;
                let i1 = mesh.indices[i * 3 + 1] as usize * 6;
                let i2 = mesh.indices[i * 3 + 2] as usize * 6;

                let vertex = |offset: usize| {
                    let point = placement.transform_point([
                        mesh.vertices[offset],
                        mesh.vertices[offset + 1],
                        mesh.vertices[offset + 2],
                    ]);
                    (point[0], point[1], point[2])
                };
                let v0 = vertex(i0);
                let v1 = vertex(i1);
                let v2 = vertex(i2);

                // Per-vertex normals (smoothed across shallow creases at mesh
                // build time, so a fillet's facets share a continuous normal
                // field). Each drives its own vertex shade for Gouraud; their
                // average decides back-face culling for the whole triangle.
                let vnorm = |o: usize| {
                    let normal = placement.transform_vector([
                        mesh.vertices[o + 3],
                        mesh.vertices[o + 4],
                        mesh.vertices[o + 5],
                    ]);
                    (normal[0], normal[1], normal[2])
                };
                let n0 = vnorm(i0);
                let n1 = vnorm(i1);
                let n2 = vnorm(i2);
                let navg = (
                    (n0.0 + n1.0 + n2.0) / 3.0,
                    (n0.1 + n1.1 + n2.1) / 3.0,
                    (n0.2 + n1.2 + n2.2) / 3.0,
                );

                // Back-face culling. The outward face normal is rotated through
                // the same view transform as the points; its depth component
                // (toward the camera = positive) tells us if the face points at
                // the viewer. Dropping back faces makes the solid read as opaque
                // (no see-through to the far side) and removes the front/back
                // depth-sort ties that caused shimmering while orbiting.
                // The ghosted Cut RESULT body keeps its back faces so the far
                // walls of the pocket / through-hole show through it; the red
                // cut-tool VOLUME culls them so it stays a clean translucent
                // solid (see `Drawable::cull_back`).
                let rz_n = sin_y * navg.0 + cos_y * navg.2;
                let n_depth = sin_p * navg.1 + cos_p * rz_n;
                if cull_back && n_depth <= 0.0 {
                    continue;
                }

                let p0 = project_3d(v0.0, v0.1, v0.2);
                let p1 = project_3d(v1.0, v1.1, v1.2);
                let p2 = project_3d(v2.0, v2.1, v2.2);

                // Feed the opaque solid surface into the occlusion buffer (it's
                // what hides back edges). Translucent ghosts/tools are see-through
                // and must not occlude, so they're skipped.
                if *alpha == 255 {
                    for p in [p0, p1, p2] {
                        depth_min = depth_min.min(p.2);
                        depth_max = depth_max.max(p.2);
                    }
                    rasterize_depth(&mut zbuf, occ_w, occ_h, occ_cell, rect.min, p0, p1, p2);
                }

                // On-top drawables sort after the whole scene (still depth-
                // ordered among themselves) so the depth span stays finite.
                let avg_depth = (p0.2 + p1.2 + p2.2) / 3.0 + if d.on_top { 1.0e6 } else { 0.0 };
                let face_id = mesh.face_ids.get(i).copied().unwrap_or(0);
                let base = if face_uses_selected_material(
                    d.node_id,
                    face_id,
                    &selected_whole_bodies,
                    &selected_faces,
                ) {
                    SELECTED_BODY_BASE
                } else if matches!(
                    self.hovered_body_element.as_ref(),
                    Some((node_id, BodyPick::Face(hovered_face)))
                        if Some(node_id.as_str()) == d.node_id && *hovered_face == face_id
                ) {
                    blend_base(d.base, HOVER_BODY_TINT, 0.14)
                } else {
                    d.base
                };

                // Gouraud shading: one shade per vertex from its own normal, so a
                // smoothly-varying normal across a fillet's facets blends into one
                // continuous curved surface instead of flat-shaded bands.
                let light = (0.40, 0.82, 0.40); // soft top-ish key light
                let shade_vertex = |n: (f32, f32, f32)| -> egui::Color32 {
                    let d = (n.0 * light.0 + n.1 * light.1 + n.2 * light.2).abs();
                    let intensity = (0.55 + 0.45 * d).clamp(0.0, 1.0);
                    let s = |b: f32| (b * intensity).clamp(0.0, 255.0) as u8;
                    egui::Color32::from_rgba_unmultiplied(s(base.0), s(base.1), s(base.2), *alpha)
                };
                let colors = [shade_vertex(n0), shade_vertex(n1), shade_vertex(n2)];

                render_items.push(RenderItem {
                    depth: avg_depth,
                    content: RenderItemContent::Triangle {
                        points: [
                            egui::pos2(p0.0, p0.1),
                            egui::pos2(p1.0, p1.1),
                            egui::pos2(p2.0, p2.1),
                        ],
                        colors,
                    },
                });
            }
        }

        // B. Gather Origin Plane Sheets
        if self.plane_pick_active() {
            for (_pts, world, depth, plane, fill_col, border_col, label) in planes_to_draw {
                render_items.push(RenderItem {
                    depth,
                    content: RenderItemContent::PlaneSheet {
                        world_corners: world,
                        fill_color: fill_col,
                        border_color: border_col,
                        label: label.to_string(),
                        hovered: self.hovered_plane == Some(plane),
                    },
                });
            }
        }
        // B2. Datum plane sheets — always drawn (they're persistent reference
        // geometry the user created), brighter while hovered in plane selection.
        for (id, name, world, depth) in &datum_sheets {
            let hovered = self.hovered_datum_plane.as_deref() == Some(id.as_str());
            let (fill, border) = if hovered {
                (
                    egui::Color32::from_rgba_unmultiplied(150, 120, 240, 120),
                    egui::Color32::from_rgb(110, 70, 220),
                )
            } else {
                (
                    egui::Color32::from_rgba_unmultiplied(170, 130, 240, 28),
                    egui::Color32::from_rgba_unmultiplied(150, 100, 230, 150),
                )
            };
            render_items.push(RenderItem {
                depth: *depth,
                content: RenderItemContent::PlaneSheet {
                    world_corners: *world,
                    fill_color: fill,
                    border_color: border,
                    label: name.clone(),
                    hovered,
                },
            });
        }
        // Outside plane-selection we no longer draw the big origin sheets; the
        // minimal origin (axis lines + corner triad) is enough and keeps bodies
        // from looking like they sit behind a translucent pane.

        // C. (Wireframe edges are drawn in section F, AFTER the solids, using the
        // depth buffer for hidden-line removal — see below.)

        // D. Sort back-to-front by depth (Painter's Algorithm)
        render_items.sort_by(|a, b| {
            a.depth
                .partial_cmp(&b.depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // E. Draw Sorted Items — triangles are accumulated into one batched
        // egui::Mesh (depth-sorted order is preserved by vertex insertion order,
        // which egui renders sequentially). One painter.add per frame instead of
        // one per triangle eliminates the per-triangle vertex-buffer overhead.
        // PlaneSheets flush the current batch to maintain their painter's position
        // in the depth-sorted sequence. Real model edges are drawn in section F;
        // drawing per-triangle seam outlines here exposes boolean tessellation
        // diagonals as stray construction-like lines after Join/Cut operations.

        // The solid occlusion buffer is complete after section A, so define the
        // hidden-surface test here: both the plane sheets (E) and the wireframe
        // edges (F) use it to hide the parts that dip behind a nearer solid face.
        // Self-occlusion guard: geometry lies ON its own faces, so only a face
        // nearer by more than this hides it. Scaled to the model's depth span.
        let occ_bias = ((depth_max - depth_min) * 0.01).max(0.02);
        // Does this one cell hold a face nearer than the tested point?
        let cell_occludes = |cx: i32, cy: i32, sd: f32| -> bool {
            if cx < 0 || cy < 0 || cx as usize >= occ_w || cy as usize >= occ_h {
                return false; // off-buffer ⇒ background ⇒ not occluding
            }
            zbuf[cy as usize * occ_w + cx as usize] > sd + occ_bias
        };
        // A point is hidden only if its cell AND its four orthogonal neighbours are
        // all covered by a nearer face. The 5-cell rule keeps curved silhouettes and
        // sheet-vs-body seams stable rather than dashing them (see section F).
        let occluded = |sx: f32, sy: f32, sd: f32| -> bool {
            let cx = ((sx - rect.min.x) / occ_cell) as i32;
            let cy = ((sy - rect.min.y) / occ_cell) as i32;
            cell_occludes(cx, cy, sd)
                && cell_occludes(cx - 1, cy, sd)
                && cell_occludes(cx + 1, cy, sd)
                && cell_occludes(cx, cy - 1, sd)
                && cell_occludes(cx, cy + 1, sd)
        };

        let mut batched_mesh = egui::Mesh::default();

        let flush_batch = |mesh: &mut egui::Mesh, painter: &egui::Painter| {
            if !mesh.vertices.is_empty() {
                painter.add(egui::Shape::mesh(std::mem::take(mesh)));
            }
        };

        for item in render_items {
            match item.content {
                RenderItemContent::Triangle { points, colors } => {
                    // Append this triangle into the running batch. The base index
                    // advances by 3 per triangle so each triangle's local [0,1,2]
                    // offsets map to the correct absolute vertex slot.
                    let base = batched_mesh.vertices.len() as u32;
                    for (p, c) in points.iter().zip(colors.iter()) {
                        batched_mesh.colored_vertex(*p, *c);
                    }
                    batched_mesh.add_triangle(base, base + 1, base + 2);
                }
                RenderItemContent::PlaneSheet {
                    world_corners,
                    fill_color,
                    border_color,
                    label,
                    hovered,
                } => {
                    // Flush accumulated triangles before the sheet so their
                    // painter's-algorithm position in the sorted list is respected.
                    flush_batch(&mut batched_mesh, &painter);

                    // Subdivide the sheet into a cell grid, re-project each cell in
                    // world space, and skip cells whose centre is hidden by a nearer
                    // solid — so a sheet that interpenetrates a body is drawn only
                    // where it's actually in front. A single averaged sheet depth
                    // (the old convex polygon) could not do this.
                    const N: usize = 12;
                    let w = world_corners;
                    // Bilinear world position at (u, v) in [0,1]², matching the
                    // corner winding: u runs w0→w1 (and w3→w2), v runs w0→w3.
                    let world_at = |u: f32, v: f32| -> (f32, f32, f32) {
                        let lerp = |a: [f32; 3], b: [f32; 3], t: f32| {
                            [
                                a[0] + (b[0] - a[0]) * t,
                                a[1] + (b[1] - a[1]) * t,
                                a[2] + (b[2] - a[2]) * t,
                            ]
                        };
                        let bottom = lerp(w[0], w[1], u);
                        let top = lerp(w[3], w[2], u);
                        let p = lerp(bottom, top, v);
                        (p[0], p[1], p[2])
                    };
                    let proj_uv = |u: f32, v: f32| -> (f32, f32, f32) {
                        let (x, y, z) = world_at(u, v);
                        project_3d(x, y, z)
                    };

                    // Precompute the (N+1)² projected grid vertices.
                    let mut grid: Vec<(f32, f32, f32)> = Vec::with_capacity((N + 1) * (N + 1));
                    for j in 0..=N {
                        for i in 0..=N {
                            grid.push(proj_uv(i as f32 / N as f32, j as f32 / N as f32));
                        }
                    }
                    let at = |i: usize, j: usize| grid[j * (N + 1) + i];

                    // Visible cells → one mesh.
                    let mut sheet_mesh = egui::Mesh::default();
                    for j in 0..N {
                        for i in 0..N {
                            let cu = (i as f32 + 0.5) / N as f32;
                            let cv = (j as f32 + 0.5) / N as f32;
                            let center = proj_uv(cu, cv);
                            if occluded(center.0, center.1, center.2) {
                                continue;
                            }
                            let (a, b, c, e) =
                                (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                            let base = sheet_mesh.vertices.len() as u32;
                            for p in [a, b, c, e] {
                                sheet_mesh.colored_vertex(egui::pos2(p.0, p.1), fill_color);
                            }
                            sheet_mesh.add_triangle(base, base + 1, base + 2);
                            sheet_mesh.add_triangle(base, base + 2, base + 3);
                        }
                    }
                    if !sheet_mesh.vertices.is_empty() {
                        painter.add(egui::Shape::mesh(sheet_mesh));
                    }

                    // Border: stroke only the unoccluded spans of the four edges.
                    let stroke = egui::Stroke::new(1.8, border_color);
                    let edges = [
                        ((0.0, 0.0), (1.0, 0.0)),
                        ((1.0, 0.0), (1.0, 1.0)),
                        ((1.0, 1.0), (0.0, 1.0)),
                        ((0.0, 1.0), (0.0, 0.0)),
                    ];
                    for ((u0, v0), (u1, v1)) in edges {
                        let mut prev: Option<(f32, f32, f32)> = None;
                        for s in 0..=N {
                            let t = s as f32 / N as f32;
                            let p = proj_uv(u0 + (u1 - u0) * t, v0 + (v1 - v0) * t);
                            if let Some(pp) = prev {
                                let mid =
                                    ((pp.0 + p.0) * 0.5, (pp.1 + p.1) * 0.5, (pp.2 + p.2) * 0.5);
                                if !occluded(mid.0, mid.1, mid.2) {
                                    painter.line_segment(
                                        [egui::pos2(pp.0, pp.1), egui::pos2(p.0, p.1)],
                                        stroke,
                                    );
                                }
                            }
                            prev = Some(p);
                        }
                    }

                    if !label.is_empty() {
                        let c = proj_uv(0.5, 0.5);
                        painter.text(
                            egui::pos2(c.0, c.1),
                            egui::Align2::CENTER_CENTER,
                            label,
                            egui::FontId::proportional(11.0),
                            if hovered {
                                egui::Color32::BLACK
                            } else {
                                egui::Color32::from_rgb(80, 80, 80)
                            },
                        );
                    }
                }
            }
        }
        // Flush any remaining triangles.
        flush_batch(&mut batched_mesh, &painter);

        if !section_contours.is_empty() {
            let projected: Vec<Vec<egui::Pos2>> = section_contours
                .iter()
                .map(|loop_| {
                    loop_
                        .iter()
                        .map(|point| {
                            let projected = project_3d(point[0], point[1], point[2]);
                            egui::pos2(projected.0, projected.1)
                        })
                        .collect()
                })
                .collect();
            if !gpu_active
                && self
                    .section_view
                    .as_ref()
                    .is_some_and(|section| section.capped)
            {
                fill_nested_loops(
                    &painter,
                    &projected,
                    egui::Color32::from_rgba_unmultiplied(245, 120, 70, 210),
                );
            }
            for loop_ in &projected {
                for index in 0..loop_.len() {
                    painter.line_segment(
                        [loop_[index], loop_[(index + 1) % loop_.len()]],
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(190, 65, 25)),
                    );
                }
            }
        }

        // F. Wireframe edges, drawn ON TOP of the solids with depth-buffer
        // hidden-line removal. Each edge is walked in screen space and only the
        // spans that aren't behind a nearer solid face (per the occlusion buffer
        // from section A) are stroked — so back edges and the far walls of a
        // pocket no longer x-ray through the body. An edge whose both faces point
        // away is dropped up front (cheap, and the only filter for translucent
        // previews, which put nothing in the buffer).
        let edge_stroke = egui::Stroke::new(1.5, egui::Color32::from_rgb(45, 50, 60));
        // `occ_bias`, `cell_occludes` and `occluded` are defined above (before
        // section E) so the plane sheets can share this hidden-surface test.
        let faces_camera = |n: (f32, f32, f32)| -> bool {
            let rz_n = sin_y * n.0 + cos_y * n.2;
            sin_p * n.1 + cos_p * rz_n > 0.0
        };
        for d in &meshes {
            if !d.draw_edges {
                continue;
            }
            let mesh = d.mesh;
            let placement = d.placement;
            let num_edges = mesh.edge_indices.len() / 2;
            let has_normals = mesh.edge_face_normals.len() >= num_edges * 6;
            for i in 0..num_edges {
                if has_normals {
                    let o = i * 6;
                    let na = placement.transform_vector([
                        mesh.edge_face_normals[o],
                        mesh.edge_face_normals[o + 1],
                        mesh.edge_face_normals[o + 2],
                    ]);
                    let nb = placement.transform_vector([
                        mesh.edge_face_normals[o + 3],
                        mesh.edge_face_normals[o + 4],
                        mesh.edge_face_normals[o + 5],
                    ]);
                    if !faces_camera((na[0], na[1], na[2])) && !faces_camera((nb[0], nb[1], nb[2]))
                    {
                        continue;
                    }
                }

                let i0 = mesh.edge_indices[i * 2] as usize * 3;
                let i1 = mesh.edge_indices[i * 2 + 1] as usize * 3;
                let point0 = placement.transform_point([
                    mesh.edge_vertices[i0],
                    mesh.edge_vertices[i0 + 1],
                    mesh.edge_vertices[i0 + 2],
                ]);
                let point1 = placement.transform_point([
                    mesh.edge_vertices[i1],
                    mesh.edge_vertices[i1 + 1],
                    mesh.edge_vertices[i1 + 2],
                ]);
                let p0 = project_3d(point0[0], point0[1], point0[2]);
                let p1 = project_3d(point1[0], point1[1], point1[2]);

                // Walk the edge in screen space, stroking the contiguous visible
                // runs. ~2px steps when still (crisp hidden-line cuts); coarser
                // ~6px steps while orbiting, where the extra precision can't be
                // seen but the per-step occlusion lookups dominate edge-heavy models.
                let len = (p1.0 - p0.0).hypot(p1.1 - p0.1);
                let step_px = if interacting { 6.0 } else { 2.0 };
                let steps = (len / step_px).ceil().max(1.0) as usize;
                let mut run_start: Option<egui::Pos2> = None;
                let mut last_vis = egui::pos2(p0.0, p0.1);
                for s in 0..=steps {
                    let t = s as f32 / steps as f32;
                    let x = p0.0 + (p1.0 - p0.0) * t;
                    let y = p0.1 + (p1.1 - p0.1) * t;
                    let dz = p0.2 + (p1.2 - p0.2) * t;
                    if occluded(x, y, dz) {
                        if let Some(start) = run_start.take() {
                            painter.line_segment([start, last_vis], edge_stroke);
                        }
                    } else {
                        let pt = egui::pos2(x, y);
                        if run_start.is_none() {
                            run_start = Some(pt);
                        }
                        last_vis = pt;
                    }
                }
                if let Some(start) = run_start.take() {
                    painter.line_segment([start, last_vis], edge_stroke);
                }
            }
        }

        // --- 5a. DRAW BODY PRESELECTION + SELECTION HIGHLIGHTS ---
        // Hover is painted first and skipped when the same element (or its whole
        // body) is selected, so the stronger orange selected state always wins.
        let mut body_highlights: Vec<(String, BodyPick, bool)> = Vec::new();
        if let Some((node_id, pick)) = self.hovered_body_element.as_ref() {
            let hover_is_visible = matches!(pick, BodyPick::Edge(_) | BodyPick::Vertex(_));
            let hover_is_selected = self.selected_body.iter().any(|(selected_node, selected)| {
                selected_node == node_id && (*selected == *pick || *selected == BodyPick::Whole)
            });
            if hover_is_visible && !hover_is_selected {
                body_highlights.push((node_id.clone(), *pick, false));
            }
        }
        body_highlights.extend(
            self.selected_body
                .iter()
                .map(|(node_id, pick)| (node_id.clone(), *pick, true)),
        );

        if !body_highlights.is_empty() {
            for (node_id, pick, is_selected) in body_highlights {
                let edge_stroke = if is_selected {
                    egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 140, 0))
                } else {
                    egui::Stroke::new(
                        2.35,
                        egui::Color32::from_rgba_unmultiplied(65, 145, 210, 145),
                    )
                };
                let vertex_fill = if is_selected {
                    egui::Color32::from_rgb(255, 140, 0)
                } else {
                    egui::Color32::from_rgba_unmultiplied(125, 180, 225, 95)
                };
                let vertex_stroke = if is_selected {
                    egui::Stroke::new(1.5, egui::Color32::WHITE)
                } else {
                    egui::Stroke::new(
                        1.35,
                        egui::Color32::from_rgba_unmultiplied(60, 135, 195, 170),
                    )
                };
                let vertex_radius = if is_selected { 5.0 } else { 4.25 };

                let Some((mesh, placement)) =
                    self.body_highlight_geometry(&node_id, preview_bodies.as_ref())
                else {
                    continue;
                };

                // Highlight one edge. Whole-body and explicit edge selections
                // use the same front-face and depth-buffer clipping as the
                // normal hidden-line pass, so selected rear edges never x-ray.
                let highlight_edge = |painter: &egui::Painter, e: usize, visible_only: bool| {
                    if visible_only && mesh.edge_face_normals.len() >= (e + 1) * 6 {
                        let o = e * 6;
                        let na = placement.transform_vector([
                            mesh.edge_face_normals[o],
                            mesh.edge_face_normals[o + 1],
                            mesh.edge_face_normals[o + 2],
                        ]);
                        let nb = placement.transform_vector([
                            mesh.edge_face_normals[o + 3],
                            mesh.edge_face_normals[o + 4],
                            mesh.edge_face_normals[o + 5],
                        ]);
                        if !faces_camera((na[0], na[1], na[2]))
                            && !faces_camera((nb[0], nb[1], nb[2]))
                        {
                            return;
                        }
                    }
                    let i0 = mesh.edge_indices[e * 2] as usize * 3;
                    let i1 = mesh.edge_indices[e * 2 + 1] as usize * 3;
                    let point_a = placement.transform_point([
                        mesh.edge_vertices[i0],
                        mesh.edge_vertices[i0 + 1],
                        mesh.edge_vertices[i0 + 2],
                    ]);
                    let point_b = placement.transform_point([
                        mesh.edge_vertices[i1],
                        mesh.edge_vertices[i1 + 1],
                        mesh.edge_vertices[i1 + 2],
                    ]);
                    let a = project_3d(point_a[0], point_a[1], point_a[2]);
                    let b = project_3d(point_b[0], point_b[1], point_b[2]);
                    if !visible_only {
                        painter.line_segment(
                            [egui::pos2(a.0, a.1), egui::pos2(b.0, b.1)],
                            edge_stroke,
                        );
                        return;
                    }
                    let len = (b.0 - a.0).hypot(b.1 - a.1);
                    let steps = (len / 2.0).ceil().max(1.0) as usize;
                    let mut run_start: Option<egui::Pos2> = None;
                    let mut last_visible = egui::pos2(a.0, a.1);
                    for step in 0..=steps {
                        let t = step as f32 / steps as f32;
                        let x = a.0 + (b.0 - a.0) * t;
                        let y = a.1 + (b.1 - a.1) * t;
                        let depth = a.2 + (b.2 - a.2) * t;
                        if occluded(x, y, depth) {
                            if let Some(start) = run_start.take() {
                                painter.line_segment([start, last_visible], edge_stroke);
                            }
                        } else {
                            let point = egui::pos2(x, y);
                            run_start.get_or_insert(point);
                            last_visible = point;
                        }
                    }
                    if let Some(start) = run_start {
                        painter.line_segment([start, last_visible], edge_stroke);
                    }
                };

                match pick {
                    BodyPick::Face(_) => {}
                    BodyPick::Edge(g) => {
                        // `g` is a topological edge group: light up every chord that
                        // belongs to it so a whole fillet arc / rim draws as one
                        // curve. A legacy mesh without grouping treats `g` as the
                        // raw segment index (one chord).
                        let ecount = mesh.edge_indices.len() / 2;
                        if mesh.edge_groups.is_empty() {
                            if (g as usize) < ecount {
                                highlight_edge(&painter, g as usize, true);
                            }
                        } else {
                            for seg in 0..ecount {
                                if mesh.edge_groups.get(seg).copied() == Some(g) {
                                    highlight_edge(&painter, seg, true);
                                }
                            }
                        }
                    }
                    BodyPick::Vertex(v) => {
                        let i = v as usize * 3;
                        if i + 2 < mesh.edge_vertices.len() {
                            let point = placement.transform_point([
                                mesh.edge_vertices[i],
                                mesh.edge_vertices[i + 1],
                                mesh.edge_vertices[i + 2],
                            ]);
                            let p = project_3d(point[0], point[1], point[2]);
                            painter.circle_filled(egui::pos2(p.0, p.1), vertex_radius, vertex_fill);
                            painter.circle_stroke(
                                egui::pos2(p.0, p.1),
                                vertex_radius,
                                vertex_stroke,
                            );
                        }
                    }
                    BodyPick::Whole => {
                        let ecount = mesh.edge_indices.len() / 2;
                        for e in 0..ecount {
                            highlight_edge(&painter, e, true);
                        }
                    }
                }
            }
        }

        // --- 5b. DRAW FINISHED SKETCHES AS 2D OBJECTS ---
        // Every saved Sketch node is shown on its plane so the user can see and
        // pick it later; selected faces are highlighted for extrusion.
        let empty_sel: HashSet<usize> = HashSet::new();
        let active_extrude_sources: HashSet<String> = self
            .extrude_op
            .as_ref()
            .map(|op| op.targets.iter().map(|t| t.sketch_id.clone()).collect())
            .unwrap_or_default();
        let var_map = self.document.variable_map();
        for idx in self.document.graph.node_indices() {
            let node = &self.document.graph[idx];
            if self.hidden_nodes.contains(&node.id) {
                continue; // hidden sketch — don't draw
            }
            // While editing an existing sketch, the live sketch renderer below
            // owns its display. Drawing the committed copy on the same plane as
            // the editable copy caused doubled lines/fills and z-fighting over
            // bodies that intersect the origin plane.
            if self.editing_sketch_id.as_deref() == Some(node.id.as_str()) {
                continue;
            }
            if active_extrude_sources.contains(&node.id) {
                continue;
            }
            if let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver,
                ..
            } = &node.feature
            {
                let cs = *cs;
                let to_screen = |p: (f32, f32)| -> egui::Pos2 {
                    let w = cs.unproject(p.0, p.1);
                    let proj = project_3d(w.x, w.y, w.z);
                    egui::pos2(proj.0, proj.1)
                };

                // Draw the variable-resolved geometry of the sketch, with the
                // projected face boundary folded in (sketch-on-face) so the
                // fills/regions match what an extrude of this sketch builds.
                let mut eff = zerocad_core::effective_curves_solved(
                    curves,
                    shapes,
                    corner_mods,
                    mirrors,
                    solver.as_ref(),
                    &var_map,
                );
                if let Some(b) = self.document.sketch_face_boundaries.get(node.id.as_str()) {
                    eff.extend_curves(b);
                }
                let resolved_curves = &eff;
                let has_text = shapes
                    .iter()
                    .any(|shape| matches!(shape, zerocad_core::SketchShape::Text { .. }));
                let cached =
                    self.cached_finished_regions(&node.id, resolved_curves, has_text, |regions| {
                        zerocad_core::text::sketch_region_ink_mask(
                            curves,
                            shapes,
                            corner_mods,
                            mirrors,
                            solver.as_ref(),
                            &var_map,
                            regions,
                        )
                    });
                let regions = cached.regions.as_slice();
                let selected = self.selected_regions_for(&node.id);
                let sel_edges = self.selected_edges_for(&node.id);
                let sel_points = self.selected_sketch_points_for(&node.id);
                let hovered = self
                    .hovered_sketch_element
                    .as_ref()
                    .filter(|(sketch_id, _)| sketch_id == &node.id)
                    .map(|(_, pick)| *pick);
                let hovered_face = match hovered {
                    Some(SketchPick::Face(index)) => Some(index),
                    _ => None,
                };
                let hovered_edge = match hovered {
                    Some(SketchPick::Edge(index)) => Some(index),
                    _ => None,
                };
                let hovered_point = match hovered {
                    Some(SketchPick::Point(index)) => Some(index),
                    _ => None,
                };
                // Finished sketches always draw "passive": unselected faces stay
                // faint/neutral and only picked faces/edges/points are highlighted,
                // instead of the whole sketch lighting up.
                draw_sketch_geometry(
                    &painter,
                    resolved_curves,
                    regions,
                    Some(cached.fill.as_slice()),
                    Some(cached.ink_mask.as_slice()),
                    &selected,
                    &sel_edges,
                    &sel_points,
                    hovered_face,
                    hovered_edge,
                    hovered_point,
                    &to_screen,
                    false,
                );
                if let Some(solver) = solver.as_ref() {
                    let construction = zerocad_core::sketch::bake_construction_curves(solver);
                    draw_construction_curves(&painter, &construction, &to_screen);
                }
            }
        }

        // --- 6. DRAW ACTIVE (IN-PROGRESS) SKETCH CURVES + DETECTED REGIONS ---
        if self.is_sketch_mode {
            let cs = self.active_sketch_cs;
            let to_screen = |p: (f32, f32)| -> egui::Pos2 {
                let w = cs.unproject(p.0, p.1);
                let proj = project_3d(w.x, w.y, w.z);
                egui::pos2(proj.0, proj.1)
            };

            // Projected face boundary (sketch-on-face) first, as reference
            // geometry in Fusion's projected-edge violet — under the drawn
            // curves, over the region fills drawn next.
            draw_sketch_geometry(
                &painter,
                &self.sketch_curves,
                &self.detected_regions,
                Some(&self.sketch_region_fill_cache),
                Some(&self.sketch_region_ink_mask),
                &self.selected_region_indices,
                &empty_sel,
                &empty_sel,
                self.hovered_active_sketch_region,
                None,
                None,
                &to_screen,
                true,
            );
            self.draw_text_preview(&painter, &to_screen);
            if let Some(solver) = self.sketch_solver_model.as_ref() {
                let construction = zerocad_core::sketch::bake_construction_curves(solver);
                draw_construction_curves(&painter, &construction, &to_screen);
                if let Some(hovered) = self.hovered_active_sketch_element {
                    draw_hovered_solver_element(&painter, solver, hovered, &to_screen);
                }
            }
            if !self.active_face_boundary.is_empty() {
                let stroke = egui::Stroke::new(1.6, egui::Color32::from_rgb(150, 80, 200));
                for s in &self.active_face_boundary.segments {
                    painter.line_segment([to_screen(s.a), to_screen(s.b)], stroke);
                }
                for c in &self.active_face_boundary.circles {
                    let mut prev: Option<egui::Pos2> = None;
                    for k in 0..=48 {
                        let th = (k as f32 / 48.0) * std::f32::consts::TAU;
                        let p = to_screen((
                            c.center.0 + c.radius * th.cos(),
                            c.center.1 + c.radius * th.sin(),
                        ));
                        if let Some(pp) = prev {
                            painter.line_segment([pp, p], stroke);
                        }
                        prev = Some(p);
                    }
                }
            }

            // Constraint badges + selection markers for an Edit Sketch session:
            // each constraint draws its glyph at its anchor (line midpoint /
            // point / between the two points), red when it is the reported
            // conflict; selected points/entities get a highlight ring so the
            // palette's applicability is legible.
            if let Some(model) = &self.sketch_solver_model {
                let conflict = self.sketch_conflict_constraint;
                self.draw_constraint_badges(&painter, model, conflict, &|p: (f64, f64)| {
                    to_screen((p.0 as f32, p.1 as f32))
                });
            }

            // Markers on the corners staged (but not yet committed) for the
            // Fillet/Chamfer tool. The geometry already previews rounded/beveled;
            // these dots confirm which corners are selected and how many.
            for &at in &self.pending_corners {
                let p = to_screen(at);
                painter.circle_filled(p, 4.5, egui::Color32::from_rgb(255, 140, 0));
                painter.circle_stroke(p, 4.5, egui::Stroke::new(1.5, egui::Color32::WHITE));
            }

            // Anchor the inline radius box at the last staged corner, or — before
            // any corner is staged — at the live cursor, so it tracks like Fusion.
            // Once a corner is staged, also place the drag manipulator on it: a
            // handle along the corner's bisector whose screen axis carries
            // px-per-mm, so dragging maps 1:1 to radius.
            if self
                .active_tool
                .map_or(false, |t| t.corner_kind().is_some())
            {
                if let Some(&last) = self.pending_corners.last() {
                    let p = to_screen(last);
                    corner_anchor = Some(egui::pos2(p.x + 14.0, p.y - 30.0));

                    if let Some((v, bis)) = self.corner_bisector(last) {
                        let radius = self
                            .eval_dim(&self.corner_radius_text)
                            .unwrap_or(5.0)
                            .max(0.1);
                        let corner_s = to_screen(v);
                        let along = to_screen((v.0 + bis.0, v.1 + bis.1));
                        // Screen vector for 1mm along the (interior) bisector.
                        let axis = egui::vec2(along.x - corner_s.x, along.y - corner_s.y);
                        let handle_s = to_screen((v.0 + bis.0 * radius, v.1 + bis.1 * radius));
                        if axis.length() > 0.5 {
                            corner_handle = Some((corner_s, handle_s, axis));
                        }
                    }
                } else if let Some(cur) = current_cursor_snap {
                    let p = to_screen(cur);
                    corner_anchor = Some(egui::pos2(p.x + 14.0, p.y - 30.0));
                }
            }

            if self.active_tool == Some(crate::SketchTool::Trim) {
                if let Some(preview) = &self.sketch_trim_preview {
                    let removable_stroke =
                        egui::Stroke::new(3.2, egui::Color32::from_rgb(255, 82, 82));
                    for segment in &preview.removable.segments {
                        painter.line_segment(
                            [to_screen(segment.a), to_screen(segment.b)],
                            removable_stroke,
                        );
                    }
                    for circle in &preview.removable.circles {
                        let mut previous = None;
                        for index in 0..=64 {
                            let angle = index as f32 / 64.0 * std::f32::consts::TAU;
                            let point = to_screen((
                                circle.center.0 + circle.radius * angle.cos(),
                                circle.center.1 + circle.radius * angle.sin(),
                            ));
                            if let Some(previous) = previous {
                                painter.line_segment([previous, point], removable_stroke);
                            }
                            previous = Some(point);
                        }
                    }
                    for arc in &preview.removable.arcs {
                        let first = (arc.start.1 - arc.center.1).atan2(arc.start.0 - arc.center.0);
                        let mut last = (arc.end.1 - arc.center.1).atan2(arc.end.0 - arc.center.0);
                        if arc.clockwise {
                            while last >= first {
                                last -= std::f32::consts::TAU;
                            }
                        } else {
                            while last <= first {
                                last += std::f32::consts::TAU;
                            }
                        }
                        let steps = (((last - first).abs() / std::f32::consts::TAU) * 64.0)
                            .ceil()
                            .max(8.0) as usize;
                        let mut previous = None;
                        for index in 0..=steps {
                            let angle = first + (last - first) * index as f32 / steps as f32;
                            let point = to_screen((
                                arc.center.0 + arc.radius * angle.cos(),
                                arc.center.1 + arc.radius * angle.sin(),
                            ));
                            if let Some(previous) = previous {
                                painter.line_segment([previous, point], removable_stroke);
                            }
                            previous = Some(point);
                        }
                    }
                    for spline in &preview.removable.splines {
                        for segment in spline.sampled_points(0.01).windows(2) {
                            painter.line_segment(
                                [to_screen(segment[0]), to_screen(segment[1])],
                                removable_stroke,
                            );
                        }
                    }
                }
            }

            if self.active_tool == Some(crate::SketchTool::Offset) {
                if let Some(cursor) = current_cursor_snap {
                    if let Some(preview) = self.preview_sketch_offset(cursor) {
                        let preview_stroke =
                            egui::Stroke::new(1.8, egui::Color32::from_rgb(255, 140, 0));
                        for segment in &preview.segments {
                            painter.line_segment(
                                [to_screen(segment.a), to_screen(segment.b)],
                                preview_stroke,
                            );
                        }
                        for circle in &preview.circles {
                            let mut previous = None;
                            for index in 0..=48 {
                                let angle = index as f32 / 48.0 * std::f32::consts::TAU;
                                let point = to_screen((
                                    circle.center.0 + circle.radius * angle.cos(),
                                    circle.center.1 + circle.radius * angle.sin(),
                                ));
                                if let Some(previous) = previous {
                                    painter.line_segment([previous, point], preview_stroke);
                                }
                                previous = Some(point);
                            }
                        }
                        for arc in &preview.arcs {
                            let first =
                                (arc.start.1 - arc.center.1).atan2(arc.start.0 - arc.center.0);
                            let mut last =
                                (arc.end.1 - arc.center.1).atan2(arc.end.0 - arc.center.0);
                            while last - first > std::f32::consts::PI {
                                last -= std::f32::consts::TAU;
                            }
                            while last - first < -std::f32::consts::PI {
                                last += std::f32::consts::TAU;
                            }
                            if arc.clockwise && last > first {
                                last -= std::f32::consts::TAU;
                            }
                            let mut previous = None;
                            for index in 0..=24 {
                                let angle = first + (last - first) * index as f32 / 24.0;
                                let point = to_screen((
                                    arc.center.0 + arc.radius * angle.cos(),
                                    arc.center.1 + arc.radius * angle.sin(),
                                ));
                                if let Some(previous) = previous {
                                    painter.line_segment([previous, point], preview_stroke);
                                }
                                previous = Some(point);
                            }
                        }
                    }
                }
            }

            // 6e. Live preview of the in-progress shape. It is built with the
            // exact same `shape_from_points` used to commit, so the preview can
            // never diverge from the result — and it folds in typed dimensions
            // (2-point tools) and multi-point geometry (3-point tools) for free.
            if !self.sketch_points.is_empty() {
                if let Some(cursor) = current_cursor_snap {
                    let preview_stroke =
                        egui::Stroke::new(1.8, egui::Color32::from_rgb(255, 140, 0));

                    // Guide lines chaining placed points to the cursor — clarifies
                    // the multi-click tools (base edge / axes being defined).
                    if self.active_tool.map_or(false, |t| t.point_count() == 3) {
                        let guide_stroke =
                            egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 190, 110));
                        let mut chain = self.sketch_points.clone();
                        chain.push(cursor);
                        for w in chain.windows(2) {
                            painter.line_segment([to_screen(w[0]), to_screen(w[1])], guide_stroke);
                        }
                    }

                    // Mirror preview: the reflection axis (a thin guide) plus a
                    // live reflected copy of the whole sketch across it.
                    if self.active_tool == Some(crate::SketchTool::Mirror) {
                        let p0 = self.sketch_points[0];
                        let axis_stroke =
                            egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 180, 255));
                        painter.line_segment([to_screen(p0), to_screen(cursor)], axis_stroke);
                        if let Some(m) = self.mirror_preview_curves(p0, cursor) {
                            for seg in &m.segments {
                                painter.line_segment(
                                    [to_screen(seg.a), to_screen(seg.b)],
                                    preview_stroke,
                                );
                            }
                            for c in &m.circles {
                                let mut prev: Option<egui::Pos2> = None;
                                for i in 0..=48 {
                                    let t = (i as f32 / 48.0) * std::f32::consts::TAU;
                                    let p = (
                                        c.center.0 + c.radius * t.cos(),
                                        c.center.1 + c.radius * t.sin(),
                                    );
                                    let ps = to_screen(p);
                                    if let Some(last) = prev {
                                        painter.line_segment([last, ps], preview_stroke);
                                    }
                                    prev = Some(ps);
                                }
                            }
                            for arc in &m.arcs {
                                let a0 =
                                    (arc.start.1 - arc.center.1).atan2(arc.start.0 - arc.center.0);
                                let mut a1 =
                                    (arc.end.1 - arc.center.1).atan2(arc.end.0 - arc.center.0);
                                while a1 - a0 > std::f32::consts::PI {
                                    a1 -= std::f32::consts::TAU;
                                }
                                while a1 - a0 < -std::f32::consts::PI {
                                    a1 += std::f32::consts::TAU;
                                }
                                let mut prev: Option<egui::Pos2> = None;
                                for i in 0..=16 {
                                    let t = a0 + (a1 - a0) * (i as f32 / 16.0);
                                    let p = (
                                        arc.center.0 + arc.radius * t.cos(),
                                        arc.center.1 + arc.radius * t.sin(),
                                    );
                                    let ps = to_screen(p);
                                    if let Some(last) = prev {
                                        painter.line_segment([last, ps], preview_stroke);
                                    }
                                    prev = Some(ps);
                                }
                            }
                        }
                    }

                    let shape = self.shape_from_points(cursor);
                    for seg in &shape.segments {
                        painter.line_segment([to_screen(seg.a), to_screen(seg.b)], preview_stroke);
                    }
                    for c in &shape.circles {
                        let mut prev_pt: Option<egui::Pos2> = None;
                        for i in 0..=48 {
                            let theta = (i as f32 / 48.0) * std::f32::consts::TAU;
                            let p = (
                                c.center.0 + c.radius * theta.cos(),
                                c.center.1 + c.radius * theta.sin(),
                            );
                            let pt_screen = to_screen(p);
                            if let Some(last) = prev_pt {
                                painter.line_segment([last, pt_screen], preview_stroke);
                            }
                            prev_pt = Some(pt_screen);
                        }
                    }
                }
            }

            // Snap glyph: mark what the live cursor locked onto so the snap is
            // visible and trustworthy. A corner/endpoint gets a thin orange
            // ring; a midpoint or a centre gets a thin orange X. On-line and
            // grid snaps stay unmarked. Only shown while a draw/corner tool is
            // armed — i.e. the cursor is actively placing a point.
            if self.active_tool.is_some() {
                let orange = egui::Color32::from_rgb(255, 140, 0);
                // Midline inference guides: dashed orange lines from a woken
                // midpoint toward the snapped point (Fusion 360 style).
                for (a, b) in &self.cursor_snap_guides {
                    painter.add(egui::Shape::dashed_line(
                        &[to_screen(*a), to_screen(*b)],
                        egui::Stroke::new(1.0, orange),
                        5.0,
                        4.0,
                    ));
                }
                // Close-loop cue: when a continuous-Line chain is open and the
                // cursor has snapped back onto its start, emphasize that point
                // (a filled disc + white ring) — a click there closes the loop
                // and creates a face. This overrides the plain ring at that spot.
                let close_cue = match (self.line_chain_start, current_cursor_snap) {
                    (Some(cs), Some(cur)) => {
                        let d2 = (cur.0 - cs.0).powi(2) + (cur.1 - cs.1).powi(2);
                        (d2 < 1.0e-6).then(|| to_screen(cs))
                    }
                    _ => None,
                };
                if let Some(cp) = close_cue {
                    painter.circle_filled(cp, 6.0, orange);
                    painter.circle_stroke(cp, 6.0, egui::Stroke::new(1.5, egui::Color32::WHITE));
                } else if let (Some(cur), Some(kind)) = (current_cursor_snap, self.cursor_snap_kind)
                {
                    let p = to_screen(cur);
                    let stroke = egui::Stroke::new(1.0, orange);
                    match kind {
                        SnapKind::Endpoint => {
                            painter.circle_stroke(p, 5.0, stroke);
                        }
                        SnapKind::Origin => {
                            // Ring plus a filled centre dot to distinguish it.
                            painter.circle_stroke(p, 5.0, stroke);
                            painter.circle_filled(p, 1.5, orange);
                        }
                        SnapKind::Midpoint | SnapKind::Center | SnapKind::MidlineCenter => {
                            let r = 4.5;
                            painter.line_segment(
                                [egui::pos2(p.x - r, p.y - r), egui::pos2(p.x + r, p.y + r)],
                                stroke,
                            );
                            painter.line_segment(
                                [egui::pos2(p.x - r, p.y + r), egui::pos2(p.x + r, p.y - r)],
                                stroke,
                            );
                        }
                        SnapKind::Midline | SnapKind::OnLine | SnapKind::Grid => {}
                    }
                }
            }
        }

        // --- 6f. Draw Central Origin Sphere (when selecting planes) ---
        if self.plane_pick_active() {
            let orig_3d = project_3d(0.0, 0.0, 0.0);
            let orig_pt = egui::pos2(orig_3d.0, orig_3d.1);
            painter.circle_filled(orig_pt, 5.0, egui::Color32::WHITE);
            painter.circle_stroke(
                orig_pt,
                5.0,
                egui::Stroke::new(1.5, egui::Color32::from_rgb(120, 120, 120)),
            );
        }

        // --- 7. DRAW VIEWPORT CORNER AXIS TRIAD SYSTEM ---
        let triad_origin = egui::pos2(rect.left() + 45.0, rect.bottom() - 45.0);
        let triad_length = 20.0;

        let t_cos_p = self.camera_pitch.cos();
        let t_sin_p = self.camera_pitch.sin();
        let t_cos_y = self.camera_yaw.cos();
        let t_sin_y = self.camera_yaw.sin();

        let project_triad = |x: f32, y: f32, z: f32| -> egui::Pos2 {
            let rx = t_cos_y * x - t_sin_y * z;
            let rz = t_sin_y * x + t_cos_y * z;
            let ry = t_cos_p * y - t_sin_p * rz;
            egui::pos2(
                triad_origin.x + rx * triad_length,
                triad_origin.y - ry * triad_length,
            )
        };

        let tx = project_triad(1.0, 0.0, 0.0);
        let ty = project_triad(0.0, 1.0, 0.0);
        let tz = project_triad(0.0, 0.0, 1.0);

        painter.line_segment(
            [triad_origin, tx],
            egui::Stroke::new(2.5, egui::Color32::from_rgb(220, 50, 50)),
        ); // X - Red
        painter.line_segment(
            [triad_origin, ty],
            egui::Stroke::new(2.5, egui::Color32::from_rgb(50, 180, 50)),
        ); // Y - Green
        painter.line_segment(
            [triad_origin, tz],
            egui::Stroke::new(2.5, egui::Color32::from_rgb(50, 50, 220)),
        ); // Z - Blue

        painter.text(
            tx,
            egui::Align2::CENTER_CENTER,
            "X",
            egui::FontId::proportional(9.0),
            egui::Color32::from_rgb(180, 40, 40),
        );
        painter.text(
            ty,
            egui::Align2::CENTER_CENTER,
            "Y",
            egui::FontId::proportional(9.0),
            egui::Color32::from_rgb(40, 140, 40),
        );
        painter.text(
            tz,
            egui::Align2::CENTER_CENTER,
            "Z",
            egui::FontId::proportional(9.0),
            egui::Color32::from_rgb(40, 40, 180),
        );

        // Commit the inline box anchors (see where they were computed). All
        // earlier `self` borrows from `project_3d`/`meshes` have ended by now.
        self.extrude_dim_pos = extrude_anchor;
        self.edge_mod_dim_pos = edge_mod_anchor;
        self.edge_mod_handle = edge_mod_handle;
        self.corner_dim_pos = corner_anchor;
        self.corner_handle = corner_handle;
    }
}
