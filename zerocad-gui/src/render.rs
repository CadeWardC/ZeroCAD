//! The CPU-projected 3D viewport renderer: depth-sorted painter's-algorithm
//! drawing of solids (with back-face culling), wireframe edges (with hidden-line
//! removal), origin planes, grids and the orientation triad.

use std::collections::HashSet;

use eframe::egui;
use zerocad_core::{
    detect_regions, CoordinateSystem, ExtrudeMode, FeatureType, MockMesh, SketchPlane,
};

use crate::geom2d::draw_sketch_geometry;
use crate::{BodyPick, ZeroCadApp};

// Unified render list structures for depth-sorted transparent overlays
#[derive(Debug, Clone)]
struct RenderItem {
    depth: f32,
    content: RenderItemContent,
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
        points: [egui::Pos2; 4],
        fill_color: egui::Color32,
        border_color: egui::Color32,
        label: &'static str,
        plane: SketchPlane,
    },
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

impl ZeroCadApp {
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

        // --- 1. DRAW CAD BACKGROUND ---
        // Light, elegant CAD viewport canvas color
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(245, 246, 248));

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

        let planes_to_draw = vec![
            (
                xy_pts,
                xy_depth,
                SketchPlane::XY,
                fill_xy,
                stroke_xy,
                "", // Clean: no text label printed inside the viewport
            ),
            (xz_pts, xz_depth, SketchPlane::XZ, fill_xz, stroke_xz, ""),
            (yz_pts, yz_depth, SketchPlane::YZ, fill_yz, stroke_yz, ""),
        ];

        // --- 2. GROUND REFERENCE GRID & CENTRAL AXES (when not in a planar mode) ---
        if !self.is_planar_view() {
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

        // --- 3. 3D PROJECTED ACTIVE PLANE GRID (drawn behind solid parts) ---
        // The grid is drawn in the active plane's own (u, v) coordinates and
        // unprojected to 3D, so it aligns to an origin plane OR a body face.
        let grid_cs: Option<CoordinateSystem> = if self.extrude_op.is_some()
            || self.edge_mod_op.is_some()
        {
            // While pushing/pulling an extrude or edge mod the sketch grid isn't
            // needed — and viewed at the oblique angle those ops are done from, its
            // far lines fan into long rays across the model. Hide it.
            None
        } else if self.is_sketch_mode {
            Some(self.active_sketch_cs)
        } else if self.is_plane_selection_mode {
            self.hovered_plane.map(|p| match p {
                SketchPlane::XY => CoordinateSystem::XY,
                SketchPlane::XZ => CoordinateSystem::XZ,
                SketchPlane::YZ => CoordinateSystem::YZ,
            })
        } else {
            None
        };

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
                        let mr = (((au + bu) * 0.5).powi(2) + ((av + bv) * 0.5).powi(2)).sqrt();
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

        // --- 4. DEPTH-SORTED RENDER LIST PIPELINE ---
        let mut render_items = Vec::new();

        // Meshes to draw: New Body previews keep the warm additive tool volume;
        // Join/Cut previews evaluate a temporary feature so the user sees the
        // actual merged/cut result before committing.
        // A live 3D edge fillet/chamfer previews the resulting (cut) body, the
        // same way a Cut/Join extrude does — and suppresses the extrude preview.
        let edge_mod_active = self.edge_mod_op.is_some();
        let preview_mesh = if edge_mod_active {
            None
        } else {
            // Memoized: only re-tessellated when the depth/targets change.
            self.cached_preview_mesh()
        };
        let preview_mode = self.extrude_op.as_ref().map(|op| op.mode);
        let preview_bodies = if edge_mod_active {
            // Memoized: the full-model re-evaluation (truck boolean) only reruns
            // when the fillet/chamfer size/kind/target change, not every frame.
            self.cached_preview_edge_mod_bodies()
        } else if self.extrude_depth_dragging {
            // Live ghost drag: skip the (expensive) truck boolean entirely. We
            // render the un-booleaned model plus the cheap ghost tool volume
            // (added below) so the preview tracks the cursor at full frame rate.
            // The real merged/cut result is computed once on release.
            None
        } else {
            match preview_mode {
                // Memoized: the full-model re-evaluation (truck booleans) only
                // reruns when the depth/mode/targets change, not every frame.
                Some(ExtrudeMode::Join | ExtrudeMode::Cut) => self.cached_preview_extrude_bodies(),
                _ => None,
            }
        };

        // A Cut preview ghosts the resulting body (alpha < 255) so the pocket /
        // hole being formed shows through, like Fusion's cut preview. Everything
        // else stays opaque.
        let body_alpha: u8 = if preview_mode == Some(ExtrudeMode::Cut) {
            120
        } else {
            255
        };
        // Each drawable records HOW to render it. Translucent body ghosts keep
        // their back faces (so the far walls of a pocket show through) and draw
        // their wireframe. The red cut-tool VOLUME is different: it culls its
        // back faces and draws no wireframe, so it reads as one clean translucent
        // solid instead of a crisscross of overlapping front/back triangles
        // (the X-shaped "faint triangles") fringed by dark outline spikes poking
        // out of the body.
        struct Drawable<'a> {
            mesh: &'a MockMesh,
            base: (f32, f32, f32),
            alpha: u8,
            cull_back: bool,
            draw_edges: bool,
        }
        let mut meshes: Vec<Drawable> = if let Some(bodies) = preview_bodies.as_ref() {
            bodies
                .iter()
                .map(|(_, m)| Drawable {
                    mesh: m,
                    base: (190.0, 196.0, 210.0),
                    alpha: body_alpha,
                    // A translucent ghost (Cut result) keeps its back faces.
                    cull_back: body_alpha == 255,
                    draw_edges: true,
                })
                .collect()
        } else {
            self.body_meshes
                .iter()
                .map(|(_, m)| Drawable {
                    mesh: m,
                    base: (190.0, 196.0, 210.0),
                    alpha: 255,
                    cull_back: true,
                    draw_edges: true,
                })
                .collect()
        };
        match preview_mode {
            // Cut: lay the FULL cut volume over the ghosted result in translucent
            // red, so the user sees exactly how far the cut reaches — including
            // where it punches out the far side of a body. Back faces culled +
            // no wireframe so it stays a clean translucent red solid.
            Some(ExtrudeMode::Cut) => {
                if let Some(pm) = preview_mesh.as_ref() {
                    meshes.push(Drawable {
                        mesh: pm,
                        base: (232.0, 66.0, 66.0),
                        alpha: 90,
                        cull_back: true,
                        draw_edges: false,
                    });
                }
            }
            // Join: a settled preview already shows the merged result (the added
            // material is part of the booleaned body), so no separate tool volume
            // is needed. But while push/pull dragging, that boolean is deferred —
            // so float the warm additive ghost over the live body for instant,
            // full-rate feedback. On release the merge runs and replaces it.
            Some(ExtrudeMode::Join) => {
                if self.extrude_depth_dragging {
                    if let Some(pm) = preview_mesh.as_ref() {
                        meshes.push(Drawable {
                            mesh: pm,
                            base: (255.0, 178.0, 96.0),
                            alpha: 255,
                            cull_back: true,
                            draw_edges: true,
                        });
                    }
                }
            }
            // New Body: the warm additive tool volume floats over the live model.
            Some(ExtrudeMode::NewBody) | None => {
                if let Some(pm) = preview_mesh.as_ref() {
                    meshes.push(Drawable {
                        mesh: pm,
                        base: (255.0, 178.0, 96.0),
                        alpha: 255,
                        cull_back: true,
                        draw_edges: true,
                    });
                }
            }
        }

        // Screen anchor for the inline extrude distance box: the projected
        // centroid of the live preview, nudged up-right so it floats clear of
        // the body. Applied to `self.extrude_dim_pos` at the end of the frame
        // (the `meshes`/`project_3d` borrows of `self` are still live here).
        let mut extrude_anchor: Option<egui::Pos2> = None;
        if let Some(pm) = preview_mesh.as_ref() {
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
            // body for a convex edge) → the direction that grows the radius.
            let (n1, n2) = (op.edge.n1, op.edge.n2);
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
        let bw_full = rect.width().ceil().max(1.0) as usize;
        let bh_full = rect.height().ceil().max(1.0) as usize;
        // ~2px cells, scaled up on hi-dpi / fullscreen so the buffer stays small.
        let occ_cell = (((bw_full.max(bh_full) + 1023) / 1024).max(1) * 2) as f32;
        let occ_w = (bw_full as f32 / occ_cell) as usize + 2;
        let occ_h = (bh_full as f32 / occ_cell) as usize + 2;
        let mut zbuf = vec![f32::NEG_INFINITY; occ_w * occ_h];
        let (mut depth_min, mut depth_max) = (f32::INFINITY, f32::NEG_INFINITY);

        // A. Gather Solid Mesh Triangles (committed model + extrude preview)
        for d in &meshes {
            let (mesh, base, alpha, cull_back) = (d.mesh, &d.base, &d.alpha, d.cull_back);
            let num_tris = mesh.indices.len() / 3;
            for i in 0..num_tris {
                let i0 = mesh.indices[i * 3] as usize * 6;
                let i1 = mesh.indices[i * 3 + 1] as usize * 6;
                let i2 = mesh.indices[i * 3 + 2] as usize * 6;

                let v0 = (
                    mesh.vertices[i0],
                    mesh.vertices[i0 + 1],
                    mesh.vertices[i0 + 2],
                );
                let v1 = (
                    mesh.vertices[i1],
                    mesh.vertices[i1 + 1],
                    mesh.vertices[i1 + 2],
                );
                let v2 = (
                    mesh.vertices[i2],
                    mesh.vertices[i2 + 1],
                    mesh.vertices[i2 + 2],
                );

                // Per-vertex normals (smoothed across shallow creases at mesh
                // build time, so a fillet's facets share a continuous normal
                // field). Each drives its own vertex shade for Gouraud; their
                // average decides back-face culling for the whole triangle.
                let vnorm = |o: usize| {
                    (mesh.vertices[o + 3], mesh.vertices[o + 4], mesh.vertices[o + 5])
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

                let avg_depth = (p0.2 + p1.2 + p2.2) / 3.0;

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
        if self.is_plane_selection_mode {
            for (pts, depth, plane, fill_col, border_col, label) in planes_to_draw {
                render_items.push(RenderItem {
                    depth,
                    content: RenderItemContent::PlaneSheet {
                        points: pts,
                        fill_color: fill_col,
                        border_color: border_col,
                        label,
                        plane,
                    },
                });
            }
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
        // in the depth-sorted sequence. Hairline seam strokes (flat faces only)
        // are collected and emitted after the mesh so they draw on top of all
        // opaque geometry without interrupting the batch.
        let mut batched_mesh = egui::Mesh::default();
        let mut seam_strokes: Vec<egui::Shape> = Vec::new();

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

                    // Hairline seam stroke for flat-shaded facets only (see the
                    // longer comment in the single-triangle version above). Skipped
                    // while the camera is moving — it's one closed_line (3 strokes)
                    // per flat triangle, the heaviest per-triangle cost, and its
                    // absence is imperceptible mid-orbit.
                    let flat = colors[0] == colors[1] && colors[1] == colors[2];
                    if !interacting && flat && colors[0].a() == 255 {
                        seam_strokes.push(egui::Shape::closed_line(
                            points.to_vec(),
                            egui::Stroke::new(1.0, colors[0]),
                        ));
                    }
                }
                RenderItemContent::PlaneSheet {
                    points,
                    fill_color,
                    border_color,
                    label,
                    plane,
                } => {
                    // Flush accumulated triangles before the sheet so their
                    // painter's-algorithm position in the sorted list is respected.
                    flush_batch(&mut batched_mesh, &painter);
                    painter.add(egui::Shape::convex_polygon(
                        points.to_vec(),
                        fill_color,
                        egui::Stroke::new(1.8, border_color),
                    ));
                    if !label.is_empty() {
                        let cx = (points[0].x + points[1].x + points[2].x + points[3].x) / 4.0;
                        let cy = (points[0].y + points[1].y + points[2].y + points[3].y) / 4.0;
                        painter.text(
                            egui::pos2(cx, cy),
                            egui::Align2::CENTER_CENTER,
                            label,
                            egui::FontId::proportional(11.0),
                            if self.hovered_plane == Some(plane) {
                                egui::Color32::BLACK
                            } else {
                                egui::Color32::from_rgb(80, 80, 80)
                            },
                        );
                    }
                }
            }
        }
        // Flush any remaining triangles, then draw seam strokes on top.
        flush_batch(&mut batched_mesh, &painter);
        painter.extend(seam_strokes);

        // F. Wireframe edges, drawn ON TOP of the solids with depth-buffer
        // hidden-line removal. Each edge is walked in screen space and only the
        // spans that aren't behind a nearer solid face (per the occlusion buffer
        // from section A) are stroked — so back edges and the far walls of a
        // pocket no longer x-ray through the body. An edge whose both faces point
        // away is dropped up front (cheap, and the only filter for translucent
        // previews, which put nothing in the buffer).
        let edge_stroke = egui::Stroke::new(1.5, egui::Color32::from_rgb(45, 50, 60));
        // Self-occlusion guard: an edge lies ON its own faces, so only a face
        // nearer by more than this hides it. Scaled to the model's depth span.
        let occ_bias = ((depth_max - depth_min) * 0.01).max(0.02);
        let occluded = |sx: f32, sy: f32, sd: f32| -> bool {
            let cx = ((sx - rect.min.x) / occ_cell) as i32;
            let cy = ((sy - rect.min.y) / occ_cell) as i32;
            if cx < 0 || cy < 0 || cx as usize >= occ_w || cy as usize >= occ_h {
                return false;
            }
            zbuf[cy as usize * occ_w + cx as usize] > sd + occ_bias
        };
        let faces_camera = |n: (f32, f32, f32)| -> bool {
            let rz_n = sin_y * n.0 + cos_y * n.2;
            sin_p * n.1 + cos_p * rz_n > 0.0
        };
        for d in &meshes {
            if !d.draw_edges {
                continue;
            }
            let mesh = d.mesh;
            let num_edges = mesh.edge_indices.len() / 2;
            let has_normals = mesh.edge_face_normals.len() >= num_edges * 6;
            for i in 0..num_edges {
                if has_normals {
                    let o = i * 6;
                    let na = (
                        mesh.edge_face_normals[o],
                        mesh.edge_face_normals[o + 1],
                        mesh.edge_face_normals[o + 2],
                    );
                    let nb = (
                        mesh.edge_face_normals[o + 3],
                        mesh.edge_face_normals[o + 4],
                        mesh.edge_face_normals[o + 5],
                    );
                    if !faces_camera(na) && !faces_camera(nb) {
                        continue;
                    }
                }

                let i0 = mesh.edge_indices[i * 2] as usize * 3;
                let i1 = mesh.edge_indices[i * 2 + 1] as usize * 3;
                let p0 = project_3d(
                    mesh.edge_vertices[i0],
                    mesh.edge_vertices[i0 + 1],
                    mesh.edge_vertices[i0 + 2],
                );
                let p1 = project_3d(
                    mesh.edge_vertices[i1],
                    mesh.edge_vertices[i1 + 1],
                    mesh.edge_vertices[i1 + 2],
                );

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

        // --- 5a. DRAW BODY SELECTION HIGHLIGHTS (on top of the solids) ---
        if !self.selected_body.is_empty() {
            let sel_fill = egui::Color32::from_rgba_unmultiplied(0, 140, 255, 90);
            let sel_edge = egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 140, 0));
            let sel_vert = egui::Color32::from_rgb(255, 140, 0);
            let faces_camera = |n: (f32, f32, f32)| -> bool {
                let rz_n = sin_y * n.0 + cos_y * n.2;
                sin_p * n.1 + cos_p * rz_n > 0.0
            };
            for (node_id, pick) in &self.selected_body {
                let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| id == node_id) else {
                    continue;
                };

                // Translucent overlay over the selected face(s) — front-facing
                // triangles only, emitted as one mesh (no feathering seams).
                let fill_face = |want: Option<u32>| {
                    let mut m = egui::Mesh::default();
                    let ntris = mesh.indices.len() / 3;
                    for t in 0..ntris {
                        if let Some(w) = want {
                            if mesh.face_ids.get(t).copied() != Some(w) {
                                continue;
                            }
                        }
                        let i0 = mesh.indices[t * 3] as usize * 6;
                        let i1 = mesh.indices[t * 3 + 1] as usize * 6;
                        let i2 = mesh.indices[t * 3 + 2] as usize * 6;
                        let n = (
                            mesh.vertices[i0 + 3],
                            mesh.vertices[i0 + 4],
                            mesh.vertices[i0 + 5],
                        );
                        if !faces_camera(n) {
                            continue;
                        }
                        let pr = |i: usize| {
                            let p = project_3d(
                                mesh.vertices[i],
                                mesh.vertices[i + 1],
                                mesh.vertices[i + 2],
                            );
                            egui::pos2(p.0, p.1)
                        };
                        let base = m.vertices.len() as u32;
                        m.colored_vertex(pr(i0), sel_fill);
                        m.colored_vertex(pr(i1), sel_fill);
                        m.colored_vertex(pr(i2), sel_fill);
                        m.add_triangle(base, base + 1, base + 2);
                    }
                    if !m.is_empty() {
                        painter.add(egui::Shape::mesh(m));
                    }
                };

                // Highlight the visible (front-facing) edges of the body.
                let highlight_edge = |painter: &egui::Painter, e: usize| {
                    let i0 = mesh.edge_indices[e * 2] as usize * 3;
                    let i1 = mesh.edge_indices[e * 2 + 1] as usize * 3;
                    let a = project_3d(
                        mesh.edge_vertices[i0],
                        mesh.edge_vertices[i0 + 1],
                        mesh.edge_vertices[i0 + 2],
                    );
                    let b = project_3d(
                        mesh.edge_vertices[i1],
                        mesh.edge_vertices[i1 + 1],
                        mesh.edge_vertices[i1 + 2],
                    );
                    painter.line_segment([egui::pos2(a.0, a.1), egui::pos2(b.0, b.1)], sel_edge);
                };

                match *pick {
                    BodyPick::Face(fid) => fill_face(Some(fid)),
                    BodyPick::Edge(e) => highlight_edge(&painter, e as usize),
                    BodyPick::Vertex(v) => {
                        let i = v as usize * 3;
                        if i + 2 < mesh.edge_vertices.len() {
                            let p = project_3d(
                                mesh.edge_vertices[i],
                                mesh.edge_vertices[i + 1],
                                mesh.edge_vertices[i + 2],
                            );
                            painter.circle_filled(egui::pos2(p.0, p.1), 5.0, sel_vert);
                            painter.circle_stroke(
                                egui::pos2(p.0, p.1),
                                5.0,
                                egui::Stroke::new(1.5, egui::Color32::WHITE),
                            );
                        }
                    }
                    BodyPick::Whole => {
                        fill_face(None);
                        let ecount = mesh.edge_indices.len() / 2;
                        for e in 0..ecount {
                            highlight_edge(&painter, e);
                        }
                    }
                }
            }
        }

        // --- 5b. DRAW FINISHED SKETCHES AS 2D OBJECTS ---
        // Every saved Sketch node is shown on its plane so the user can see and
        // pick it later; selected faces are highlighted for extrusion.
        let empty_sel: HashSet<usize> = HashSet::new();
        let cut_preview_sources: HashSet<String> = self
            .extrude_op
            .as_ref()
            .filter(|op| op.mode == ExtrudeMode::Cut)
            .map(|op| op.targets.iter().map(|t| t.sketch_id.clone()).collect())
            .unwrap_or_default();
        let var_map = self.graph.variable_map();
        for idx in self.graph.graph.node_indices() {
            let node = &self.graph.graph[idx];
            if self.hidden_nodes.contains(&node.id) {
                continue; // hidden sketch — don't draw
            }
            if let FeatureType::Sketch { cs, curves, shapes, corner_mods, .. } = &node.feature {
                let cs = *cs;
                let to_screen = |p: (f32, f32)| -> egui::Pos2 {
                    let w = cs.unproject(p.0, p.1);
                    let proj = project_3d(w.x, w.y, w.z);
                    egui::pos2(proj.0, proj.1)
                };

                // Draw the variable-resolved geometry of the sketch.
                let eff = zerocad_core::effective_curves(curves, shapes, corner_mods, &var_map);
                let curves = &eff;
                let regions = detect_regions(curves);
                let selected = if cut_preview_sources.contains(&node.id) {
                    HashSet::new()
                } else {
                    self.selected_regions_for(&node.id)
                };
                let sel_edges = self.selected_edges_for(&node.id);
                // Finished sketches always draw "passive": unselected faces stay
                // faint/neutral and only picked faces/edges are highlighted,
                // instead of the whole sketch lighting up.
                draw_sketch_geometry(
                    &painter, curves, &regions, &selected, &sel_edges, &to_screen, false,
                );
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

            draw_sketch_geometry(
                &painter,
                &self.sketch_curves,
                &self.detected_regions,
                &empty_sel,
                &empty_sel,
                &to_screen,
                true,
            );

            // Markers on the corners staged (but not yet committed) for the
            // Fillet/Chamfer tool. The geometry already previews rounded/beveled;
            // these dots confirm which corners are selected and how many.
            for &at in &self.pending_corners {
                let p = to_screen(at);
                painter.circle_filled(p, 4.5, egui::Color32::from_rgb(255, 140, 0));
                painter.circle_stroke(
                    p,
                    4.5,
                    egui::Stroke::new(1.5, egui::Color32::WHITE),
                );
            }

            // Anchor the inline radius box at the last staged corner, or — before
            // any corner is staged — at the live cursor, so it tracks like Fusion.
            // Once a corner is staged, also place the drag manipulator on it: a
            // handle along the corner's bisector whose screen axis carries
            // px-per-mm, so dragging maps 1:1 to radius.
            if self.active_tool.map_or(false, |t| t.corner_kind().is_some()) {
                if let Some(&last) = self.pending_corners.last() {
                    let p = to_screen(last);
                    corner_anchor = Some(egui::pos2(p.x + 14.0, p.y - 30.0));

                    if let Some((v, bis)) = self.corner_bisector(last) {
                        let radius = self.eval_dim(&self.corner_radius_text).unwrap_or(5.0).max(0.1);
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
                            painter.line_segment(
                                [to_screen(w[0]), to_screen(w[1])],
                                guide_stroke,
                            );
                        }
                    }

                    let shape = self.shape_from_points(cursor);
                    for seg in &shape.segments {
                        painter.line_segment(
                            [to_screen(seg.a), to_screen(seg.b)],
                            preview_stroke,
                        );
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
        }

        // --- 6f. Draw Central Origin Sphere (when selecting planes) ---
        if self.is_plane_selection_mode {
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
