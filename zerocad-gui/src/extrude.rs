//! The extrude tool: the live (uncommitted) extrude operation and its preview,
//! plus committing it into the parametric graph and the inline distance dialog.

use std::collections::{HashMap, HashSet};

use eframe::egui;
use zerocad_core::{
    detect_regions, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, Region,
};

use crate::{PendingCommitVisual, PendingVisualMode, SharedBodyMeshes, ZeroCadApp};

const EXTRUDE_PREVIEW_SETTLE: std::time::Duration = std::time::Duration::from_millis(100);

/// The default extrude mode for a freshly started op: a sketch on a body face
/// pulled **outward** (depth ≥ 0, along the outward face normal) adds material
/// (Join); pushed **inward** (depth < 0) removes it (Cut). A sketch on an origin
/// plane has no body to combine with, so it makes a new body.
pub(crate) fn default_extrude_mode(on_face: bool, depth: f32) -> ExtrudeMode {
    if !on_face {
        ExtrudeMode::NewBody
    } else if depth >= 0.0 {
        ExtrudeMode::Join
    } else {
        ExtrudeMode::Cut
    }
}

#[cfg(test)]
mod preview_tests {
    use super::*;

    #[test]
    fn split_circle_preview_is_clean_in_every_extrude_mode() {
        let circle = zerocad_core::Circle {
            center: (0.0, 0.0),
            radius: 2.0,
        };
        let mut curves = zerocad_core::SketchCurves::new();
        curves.add_circle(circle.center, circle.radius);
        curves.add_line((0.0, -3.0), (0.0, 3.0));
        let regions = detect_regions(&curves);
        assert!(regions.len() >= 2, "line should split the circle");

        for mode in [ExtrudeMode::NewBody, ExtrudeMode::Join, ExtrudeMode::Cut] {
            let op = ExtrudeOp {
                targets: vec![ExtrudeTarget {
                    sketch_id: "sketch_1".into(),
                    cs: CoordinateSystem::XY,
                    indices: (0..regions.len()).collect(),
                    regions: regions.clone(),
                    loops: Vec::new(),
                    circles: vec![circle],
                    draft_supported: false,
                    on_face: mode != ExtrudeMode::NewBody,
                }],
                depth: 4.0,
                depth_text: "4".into(),
                draft_angle_deg: 0.0,
                draft_angle_text: "0".into(),
                focus_request: false,
                mode,
                on_face: mode != ExtrudeMode::NewBody,
                mode_user_set: true,
                pending_face_sketch: None,
            };
            let parts = op.preview_part_meshes(4.0);
            assert_eq!(parts.len(), 1);
            let seams: Vec<_> = parts[0]
                .2
                .edge_refs
                .iter()
                .filter(|edge| {
                    matches!(
                        edge.curve,
                        Some(zerocad_core::mock_kernel::EdgeCurveHint::Line)
                    ) && (edge.p0[2] - edge.p1[2]).abs() > 3.9
                })
                .collect();
            assert!(seams.is_empty(), "{mode:?} preview seams: {seams:#?}");
        }
    }
}

/// Faces from one sketch that participate in an extrude, plus the geometry
/// needed to build the preview mesh without touching the parametric graph.
#[derive(Debug, Clone)]
pub(crate) struct ExtrudeTarget {
    pub(crate) sketch_id: String,
    pub(crate) cs: CoordinateSystem,
    pub(crate) regions: Vec<Region>,
    pub(crate) indices: Vec<usize>,
    /// The sketch's shape outlines, so the ghost can resolve overlapping-shapes-
    /// as-boolean exactly like the evaluator (dropping the tool-lens region of a
    /// circle-in-rectangle so it reads as a hole). Empty for a direct face
    /// push/pull (no drawn shapes).
    pub(crate) loops: Vec<zerocad_core::ShapeLoop>,
    /// Variable-resolved analytic circles from the sketch. Kept separately from
    /// the split regions so the orange ghost can rebuild a clean cylinder when a
    /// construction/projected line divides the circle into multiple faces.
    pub(crate) circles: Vec<zerocad_core::Circle>,
    /// False when the selected sketch contains analytic curved boundaries that
    /// Extrude Draft v1 deliberately leaves unresolved.
    pub(crate) draft_supported: bool,
    /// True when the source sketch sits on an existing body face (vs an origin
    /// plane). Drives the auto-chosen extrude mode (face → Join/Cut, plane →
    /// New Body).
    pub(crate) on_face: bool,
}

/// A helper sketch that will be created at COMMIT time for a direct push/pull
/// on a body face: an empty (nothing drawn) on-face sketch whose projected
/// face boundary is its only geometry, so its single region IS the face. Kept
/// out of the parametric graph until the user confirms — cancelling the op
/// leaves no trace.
#[derive(Debug, Clone)]
pub(crate) struct PendingFaceSketch {
    pub(crate) sketch_id: String,
    pub(crate) name: String,
    pub(crate) cs: CoordinateSystem,
    pub(crate) fref: Option<zerocad_core::parametric::FaceRef>,
    pub(crate) boundary: zerocad_core::SketchCurves,
    /// The body the face belongs to (dependency edge + boolean target).
    pub(crate) body_id: String,
}

/// A live, uncommitted extrude. Drives the right-hand tool window and the
/// real-time push/pull preview (Fusion-style) until the user confirms.
#[derive(Debug, Clone)]
pub(crate) struct ExtrudeOp {
    pub(crate) targets: Vec<ExtrudeTarget>,
    pub(crate) depth: f32,
    /// Editable text buffer for the inline distance box. Kept in sync with
    /// `depth` (typing parses into `depth`; dragging reformats this).
    pub(crate) depth_text: String,
    /// Signed far-section wall draft in degrees and its editable expression.
    pub(crate) draft_angle_deg: f32,
    pub(crate) draft_angle_text: String,
    /// True until the inline box has grabbed keyboard focus once.
    pub(crate) focus_request: bool,
    /// Whether this extrude makes a new body, joins existing bodies, or cuts.
    pub(crate) mode: ExtrudeMode,
    /// True when at least one source sketch is on a body face — enables the
    /// direction-driven default mode (pull out → Join, push in → Cut).
    pub(crate) on_face: bool,
    /// Set once the user clicks a mode button, which freezes the mode so the
    /// direction-driven default stops overriding their choice.
    pub(crate) mode_user_set: bool,
    /// `Some` for a direct push/pull started from a body face: the helper
    /// on-face sketch to materialize at commit (see [`PendingFaceSketch`]).
    pub(crate) pending_face_sketch: Option<PendingFaceSketch>,
}

impl ExtrudeOp {
    /// Build the per-target ghost meshes at `depth`, each paired with its
    /// extrusion plane origin and axis so a cached build can later be rescaled
    /// along the axis instead of re-tessellated (curved profiles run the
    /// kernel prism per build — see [`ZeroCadApp::cached_preview_mesh`]).
    pub(crate) fn preview_part_meshes(
        &self,
        depth: f32,
    ) -> Vec<(zerocad_core::Vec3, zerocad_core::Vec3, MockMesh)> {
        let mut parts = Vec::new();
        if depth.abs() < f32::EPSILON {
            return parts;
        }
        for t in &self.targets {
            // Keep exactly the regions a commit would: for overlapping shapes the
            // boolean plan drops the tool-lens region (the inner disc of a
            // circle-in-rectangle), so the annulus — which already carries that
            // disc as a hole — renders as a clean prism-with-hole.
            let plan = zerocad_core::boolean_region_plan(&t.loops, &t.regions, &t.indices);
            let mut mesh = MockMesh::empty();
            if self.draft_angle_deg.abs() > f32::EPSILON {
                if !t.draft_supported {
                    return Vec::new();
                }
                for (region_index, region) in t.regions.iter().enumerate() {
                    if !plan.process.get(region_index).copied().unwrap_or(false) {
                        continue;
                    }
                    let Ok(solid) = zerocad_core::drafted_region_solid(
                        &region.boundary,
                        &region.holes,
                        depth,
                        &t.cs,
                        self.draft_angle_deg,
                    ) else {
                        return Vec::new();
                    };
                    mesh.append(MockMesh::from_solid(&solid));
                }
                let axis = t.cs.u.cross(t.cs.v).normalize();
                parts.push((t.cs.origin, axis, mesh));
                continue;
            }
            let complete_circles =
                zerocad_core::complete_selected_circles(&t.circles, &t.regions, &plan.process);
            let mut collapsed_regions = HashSet::new();
            for (circle, indices) in complete_circles {
                let boundary: Vec<(f32, f32)> = (0..zerocad_core::CIRCLE_SEGS)
                    .map(|i| {
                        let angle =
                            i as f32 / zerocad_core::CIRCLE_SEGS as f32 * std::f32::consts::TAU;
                        (
                            circle.center.0 + circle.radius * angle.cos(),
                            circle.center.1 + circle.radius * angle.sin(),
                        )
                    })
                    .collect();
                mesh.append(zerocad_core::mock_kernel::extruded_region_display_mesh(
                    &boundary,
                    &[],
                    depth,
                    &t.cs,
                ));
                collapsed_regions.extend(indices);
            }
            for (ri, r) in t.regions.iter().enumerate() {
                if collapsed_regions.contains(&ri) {
                    continue;
                }
                if !plan.process.get(ri).copied().unwrap_or(false) {
                    continue;
                }
                mesh.append(zerocad_core::mock_kernel::extruded_region_display_mesh(
                    &r.boundary,
                    &r.holes,
                    depth,
                    &t.cs,
                ));
            }
            // The true extrusion axis is u×v (cs.n can be flipped on
            // left-handed planes); its sign doesn't matter for rescaling.
            let axis = t.cs.u.cross(t.cs.v).normalize();
            parts.push((t.cs.origin, axis, mesh));
        }
        parts
    }
}

/// Rescale an extrusion ghost built at `ref_depth` to a new depth by scaling
/// every point's offset along the extrusion axis about the sketch plane
/// (`scale` = depth / ref_depth, same sign). Valid because an extrusion's
/// cross-section is constant along the axis: wall normals stay perpendicular
/// to it and cap normals stay along it, so only positions move.
fn rescale_extrusion_mesh(
    mesh: &MockMesh,
    origin: zerocad_core::Vec3,
    axis: zerocad_core::Vec3,
    scale: f32,
) -> MockMesh {
    let mut out = mesh.clone();
    let k = scale - 1.0;
    let shift = |p: &mut [f32]| {
        let d =
            (p[0] - origin.x) * axis.x + (p[1] - origin.y) * axis.y + (p[2] - origin.z) * axis.z;
        p[0] += axis.x * d * k;
        p[1] += axis.y * d * k;
        p[2] += axis.z * d * k;
    };
    for v in out.vertices.chunks_exact_mut(6) {
        shift(&mut v[0..3]);
    }
    for v in out.edge_vertices.chunks_exact_mut(3) {
        shift(v);
    }
    for e in &mut out.edge_refs {
        shift(&mut e.p0);
        shift(&mut e.p1);
    }
    for f in &mut out.face_refs {
        shift(&mut f.centroid);
    }
    out
}

/// Materialize a pending face sketch into `graph`: the (empty-curves) on-face
/// sketch node, its face ref, its projected-boundary reference curves, and the
/// dependency edge from the body. Shared by the preview graph clone and the
/// real commit so both see identical regions.
fn insert_pending_face_sketch(graph: &mut zerocad_core::ParametricGraph, p: &PendingFaceSketch) {
    graph.add_feature(FeatureNode {
        id: p.sketch_id.clone(),
        name: p.name.clone(),
        feature: FeatureType::Sketch {
            cs: p.cs,
            curves: zerocad_core::SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: true,
            entity_ids: Vec::new(),
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_dependency(&p.body_id, &p.sketch_id);
    if let Some(fref) = &p.fref {
        graph
            .sketch_face_refs
            .insert(p.sketch_id.as_str().into(), fref.clone());
    }
    graph
        .sketch_face_boundaries
        .insert(p.sketch_id.as_str().into(), p.boundary.clone());
}

impl ZeroCadApp {
    fn extrude_preview_key(&self) -> Option<u64> {
        use std::hash::{Hash, Hasher};

        let op = self.extrude_op.as_ref()?;
        let mut h = std::collections::hash_map::DefaultHasher::new();
        ((op.depth / 0.05).round() as i64).hash(&mut h);
        ((op.draft_angle_deg / 0.02).round() as i64).hash(&mut h);
        let mode_id: u8 = match op.mode {
            ExtrudeMode::NewBody => 0,
            ExtrudeMode::Join => 1,
            ExtrudeMode::Cut => 2,
        };
        mode_id.hash(&mut h);
        for target in &op.targets {
            target.sketch_id.hash(&mut h);
            target.indices.hash(&mut h);
        }
        self.id_counter.hash(&mut h);
        let mut hidden: Vec<&String> = self.hidden_nodes.iter().collect();
        hidden.sort();
        for id in hidden {
            id.hash(&mut h);
        }
        let vars = self.document.variable_map();
        let mut var_keys: Vec<&String> = vars.keys().collect();
        var_keys.sort();
        for key in var_keys {
            key.hash(&mut h);
            ((vars[key] * 1000.0) as i64).hash(&mut h);
        }
        Some(h.finish())
    }

    /// Whether the exact worker result corresponds to the values on screen
    /// now. Older results remain useful context, but must not hide the live
    /// ghost or be promoted while committing a newer value.
    pub(crate) fn has_current_extrude_preview(&self) -> bool {
        let Some(key) = self.extrude_preview_key() else {
            return false;
        };
        matches!(self.extrude_preview_cache.as_ref(), Some((cached, _)) if *cached == key)
    }

    fn clear_extrude_preview_eval(&mut self) {
        self.extrude_preview_cache = None;
        self.extrude_preview_mesh_cache = None;
        self.extrude_ghost_base = None;
        self.extrude_preview_inflight = None;
        self.extrude_preview_settle = None;
    }

    fn build_preview_extrude_document(&self) -> Option<zerocad_core::Document> {
        let op = self.extrude_op.as_ref()?;
        if op.depth.abs() < 0.01 {
            return None;
        }

        let mut graph = self.document.clone();
        // Direct face push/pull: the helper sketch only exists in this preview
        // clone (and later at commit) — never in the working graph.
        if let Some(p) = &op.pending_face_sketch {
            insert_pending_face_sketch(&mut graph, p);
        }
        for (i, target) in op.targets.iter().enumerate() {
            if target.indices.is_empty() {
                continue;
            }

            let mut region_indices = target.indices.clone();
            region_indices.sort();
            region_indices.dedup();

            let extrude_id = format!("extrude_{}", self.id_counter + i);
            graph.add_feature(FeatureNode {
                id: extrude_id.clone(),
                name: format!("Preview Extrude {}", i + 1),
                feature: FeatureType::Extrude {
                    target: None,
                    depth: op.depth,
                    region_indices,
                    mode: op.mode,
                    depth_expr: None,
                    draft_angle_deg: op.draft_angle_deg,
                    draft_angle_expr: None,
                },
            });
            graph.add_dependency(&target.sketch_id, &extrude_id);
        }

        Some(graph)
    }

    /// Exact Cut/Join preview evaluated by the background worker. The result is
    /// cached and recomputed only when the depth (quantized to a sub-visible
    /// step), mode, or targets change, so idle frames and slow drags are cheap.
    pub(crate) fn cached_preview_extrude_bodies(&mut self) -> Option<SharedBodyMeshes> {
        let Some(depth) = self.extrude_op.as_ref().map(|op| op.depth) else {
            self.clear_extrude_preview_eval();
            return None;
        };
        if depth.abs() < 0.01 {
            return Some(self.body_meshes.clone());
        }
        let key = self.extrude_preview_key()?;

        if let Some((cached_key, bodies)) = self.extrude_preview_cache.as_ref() {
            if *cached_key == key {
                return Some(bodies.clone());
            }
        }

        // An authoritative commit always wins the single evaluator. Keep the
        // lightweight ghost visible until that final result lands, then allow
        // settled preview work to use the worker.
        if self.eval_pending {
            return self
                .extrude_preview_cache
                .as_ref()
                .map(|(_, bodies)| bodies.clone());
        }

        // The ghost volume already follows the input at frame rate. Delay the
        // exact boolean until the value settles, and cancel an obsolete solve as
        // soon as a different key arrives.
        let settled_at = match self.extrude_preview_settle {
            Some((settled_key, at)) if settled_key == key => at,
            _ => {
                if self.extrude_preview_inflight.take().is_some() {
                    self.evaluator.cancel();
                }
                let now = std::time::Instant::now();
                self.extrude_preview_settle = Some((key, now));
                if let Some(ctx) = &self.egui_ctx {
                    ctx.request_repaint_after(EXTRUDE_PREVIEW_SETTLE);
                }
                return self
                    .extrude_preview_cache
                    .as_ref()
                    .map(|(_, bodies)| bodies.clone());
            }
        };
        let waited = settled_at.elapsed();
        if waited < EXTRUDE_PREVIEW_SETTLE {
            if let Some(ctx) = &self.egui_ctx {
                ctx.request_repaint_after(EXTRUDE_PREVIEW_SETTLE - waited);
            }
            return self
                .extrude_preview_cache
                .as_ref()
                .map(|(_, bodies)| bodies.clone());
        }

        // The persistent evaluator owns the single worker and applies latest-wins
        // cancellation, so no per-preview thread or result channel is needed.
        if self.extrude_preview_inflight.is_none() {
            if let Some(document) = self.build_preview_extrude_document() {
                self.evaluator.submit(
                    crate::evaluation_worker::EvaluationPurpose::ExtrudePreview(key),
                    document,
                    self.hidden_nodes.clone(),
                    zerocad_core::EvaluationQuality::Interactive,
                    self.egui_ctx.clone(),
                );
                self.extrude_preview_inflight = Some(key);
            }
        }

        // Keep the last result cached while the worker chases the newest depth.
        // The frame planner may use it for Cut context, but deliberately withholds
        // stale opaque Join/New Body results so they cannot trail the live tool as
        // an apparent second body.
        self.extrude_preview_cache
            .as_ref()
            .map(|(_, bodies)| bodies.clone())
    }

    /// Memoized ghost tool volume for the live extrude. The
    /// tessellated base is built ONCE per target set + depth sign (curved
    /// profiles run the kernel prism, far too slow per drag step) and every
    /// depth change just rescales it along the extrusion axis — O(vertices),
    /// no kernel calls — so the ghost tracks the cursor at full frame rate.
    pub(crate) fn cached_preview_mesh(&mut self) -> Option<MockMesh> {
        use std::hash::{Hash, Hasher};
        let Some(op) = self.extrude_op.as_ref() else {
            self.extrude_preview_mesh_cache = None;
            self.extrude_ghost_base = None;
            return None;
        };
        let depth = op.depth;
        if depth.abs() < f32::EPSILON {
            return Some(MockMesh::empty());
        }
        let base_key = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            (depth >= 0.0).hash(&mut h);
            for t in &op.targets {
                t.sketch_id.hash(&mut h);
                t.indices.hash(&mut h);
            }
            h.finish()
        };
        let full_key = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            base_key.hash(&mut h);
            depth.to_bits().hash(&mut h);
            op.draft_angle_deg.to_bits().hash(&mut h);
            h.finish()
        };
        if let Some((cached_key, mesh)) = self.extrude_preview_mesh_cache.as_ref() {
            if *cached_key == full_key {
                return Some(mesh.clone());
            }
        }
        if op.draft_angle_deg.abs() > f32::EPSILON {
            let mut mesh = MockMesh::empty();
            for (_, _, part) in op.preview_part_meshes(depth) {
                mesh.append(part);
            }
            self.extrude_preview_mesh_cache = Some((full_key, mesh.clone()));
            self.extrude_ghost_base = None;
            return Some(mesh);
        }
        let base_hit =
            matches!(self.extrude_ghost_base.as_ref(), Some((k, _, _)) if *k == base_key);
        if !base_hit {
            // Build the base away from zero: a near-zero build tessellates a
            // paper-thin (possibly degenerate) prism that rescales badly.
            let build_depth = if depth.abs() < 0.05 {
                1.0f32.copysign(depth)
            } else {
                depth
            };
            let parts = op.preview_part_meshes(build_depth);
            self.extrude_ghost_base = Some((base_key, build_depth, parts));
        }
        let (_, ref_depth, parts) = self.extrude_ghost_base.as_ref().unwrap();
        let scale = depth / ref_depth;
        let mut mesh = MockMesh::empty();
        for (origin, axis, part) in parts {
            mesh.append(rescale_extrusion_mesh(part, *origin, *axis, scale));
        }
        self.extrude_preview_mesh_cache = Some((full_key, mesh.clone()));
        Some(mesh)
    }

    /// Inline extrude distance box — the same Fusion-style floating input used
    /// for sketch dimensions. Shows the live distance near the previewed body;
    /// typing a value or dragging in the viewport both update it. Enter / the OK
    /// button commit, Escape / Cancel abort.
    pub(crate) fn show_extrude_dialog(&mut self, ctx: &egui::Context) {
        if self.extrude_op.is_none() {
            return;
        }

        let unit_suffix = self.current_unit.suffix();
        // Pulled out of `self` so the field widget can borrow the variable list
        // and the shared autocomplete state while the closure holds `&mut self`.
        let var_names = self.visible_variable_names();
        let varmap = self.visible_variable_map();
        let mut ac = self.autocomplete.take();
        // Set when the autocomplete swallows an Enter (to accept a suggestion),
        // so we don't also treat that Enter as "commit the extrude".
        let mut suppress_commit = false;

        let pos = self
            .extrude_dim_pos
            .unwrap_or_else(|| ctx.screen_rect().center());

        // Fusion-style floating value field: just the editable distance, a unit
        // suffix, and a "⋮" affordance. Commit/cancel live on the right panel
        // and on Enter/Esc; this box only edits the value.
        egui::Area::new(egui::Id::new("extrude_inline"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::WHITE)
                    .rounding(3.0)
                    .stroke(egui::Stroke::new(
                        1.0,
                        egui::Color32::from_rgb(170, 180, 190),
                    ))
                    .shadow(egui::epaint::Shadow {
                        offset: egui::vec2(0.0, 2.0),
                        blur: 8.0,
                        spread: 0.0,
                        color: egui::Color32::from_black_alpha(35),
                    })
                    .inner_margin(egui::Margin::symmetric(8.0, 5.0))
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(4.0, 0.0);
                        ui.horizontal(|ui| {
                            if let Some(op) = self.extrude_op.as_mut() {
                                ui.style_mut().visuals.extreme_bg_color = egui::Color32::WHITE;
                                ui.style_mut().visuals.widgets.inactive.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.widgets.hovered.bg_stroke =
                                    egui::Stroke::NONE;
                                ui.style_mut().visuals.selection.bg_fill =
                                    egui::Color32::from_rgb(0, 120, 215).linear_multiply(0.35);

                                let field_id = egui::Id::new("extrude_depth_field");
                                let outcome = crate::expr::autocomplete_field(
                                    ui,
                                    field_id,
                                    &mut op.depth_text,
                                    56.0,
                                    true,
                                    op.focus_request,
                                    op.focus_request,
                                    &var_names,
                                    &mut ac,
                                );
                                let resp = outcome.response;
                                op.focus_request = false;
                                if outcome.accepted_via_key {
                                    suppress_commit = true;
                                }

                                if resp.has_focus() || resp.changed() {
                                    // User is typing → evaluate the expression
                                    // (number, variable, or arithmetic) into depth.
                                    if let Ok(v) = crate::expr::eval(&op.depth_text, &varmap) {
                                        op.depth = (v as f32).clamp(-300.0, 300.0);
                                    }
                                } else if !zerocad_core::expr::preserves_source(&op.depth_text) {
                                    // Not focused → reflect drag/slider changes.
                                    op.depth_text = format!("{:.2}", op.depth);
                                }

                                // Blue active underline beneath the value.
                                if resp.has_focus() {
                                    let r = resp.rect;
                                    ui.painter().line_segment(
                                        [r.left_bottom(), r.right_bottom()],
                                        egui::Stroke::new(
                                            1.5,
                                            egui::Color32::from_rgb(0, 120, 215),
                                        ),
                                    );
                                }

                                ui.label(
                                    egui::RichText::new(unit_suffix)
                                        .color(egui::Color32::from_rgb(110, 110, 110))
                                        .size(12.0),
                                );
                            }

                            // Vertical "more" affordance, like the Fusion field.
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new("⋮")
                                    .color(egui::Color32::from_rgb(150, 150, 150))
                                    .size(14.0),
                            );
                        });

                        // Operation mode: New Body / Join / Cut — a compact
                        // segmented control beneath the distance, Fusion-style.
                        if let Some(op) = self.extrude_op.as_ref() {
                            if zerocad_core::expr::preserves_source(&op.depth_text) {
                                if let Ok(value) = crate::expr::eval(&op.depth_text, &varmap) {
                                    ui.label(
                                        egui::RichText::new(format!("= {value:.2} {unit_suffix}"))
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(70, 120, 70)),
                                    );
                                }
                            }
                        }
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new("Draft")
                                    .size(11.5)
                                    .color(egui::Color32::from_rgb(90, 95, 102)),
                            );
                            if let Some(op) = self.extrude_op.as_mut() {
                                let outcome = crate::expr::autocomplete_field(
                                    ui,
                                    egui::Id::new("extrude_draft_angle_field"),
                                    &mut op.draft_angle_text,
                                    56.0,
                                    true,
                                    false,
                                    false,
                                    &var_names,
                                    &mut ac,
                                );
                                if outcome.accepted_via_key {
                                    suppress_commit = true;
                                }
                                if outcome.response.changed() || outcome.response.has_focus() {
                                    if let Ok(value) =
                                        crate::expr::eval(&op.draft_angle_text, &varmap)
                                    {
                                        op.draft_angle_deg = value as f32;
                                    }
                                }
                                ui.label(
                                    egui::RichText::new("°")
                                        .size(12.0)
                                        .color(egui::Color32::from_rgb(110, 110, 110)),
                                );
                            }
                        });
                        ui.add_space(5.0);
                        if let Some(op) = self.extrude_op.as_mut() {
                            // Until the user picks a mode, follow the sketch's
                            // context: plane → New Body, face pulled out → Join,
                            // face pushed in → Cut (re-evaluated as they drag).
                            if !op.mode_user_set {
                                op.mode = default_extrude_mode(op.on_face, op.depth);
                            }
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(3.0, 0.0);
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
                                    let btn = ui.add(
                                        egui::Button::new(
                                            egui::RichText::new(label).color(text).size(11.5),
                                        )
                                        .fill(fill)
                                        .rounding(3.0)
                                        .min_size(egui::vec2(52.0, 20.0)),
                                    );
                                    if btn.clicked() {
                                        op.mode = mode;
                                        op.mode_user_set = true;
                                    }
                                }
                            });
                        }
                    });
            });

        // Read commit/cancel keys *after* the field has rendered, so an Enter the
        // autocomplete consumed (to accept a suggestion) never leaks into commit.
        let enter = ctx.input(|i| i.key_pressed(egui::Key::Enter));
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        self.autocomplete = ac;

        if enter && !suppress_commit {
            self.commit_extrude_op();
        } else if escape {
            self.cancel_extrude_op();
        }
    }

    /// Create one Extrude body from `region_indices` of `sketch_id` and return
    /// its new id. Does not clear selection or re-evaluate — callers do that.
    fn build_extrude_body(
        &mut self,
        sketch_id: &str,
        mut region_indices: Vec<usize>,
        depth: f32,
        mode: ExtrudeMode,
        depth_expr: Option<String>,
        draft_angle_deg: f32,
        draft_angle_expr: Option<String>,
    ) -> Option<String> {
        if region_indices.is_empty() {
            return None;
        }
        region_indices.sort();
        region_indices.dedup();

        let feature_name = self.next_operation_name(mode);
        let extrude_id = format!("extrude_{}", self.next_id());
        log::info!(
            "Creating {:?} extrude {} ({}) from sketch {} ({} faces, depth {}, expr {:?})",
            mode,
            extrude_id,
            feature_name,
            sketch_id,
            region_indices.len(),
            depth,
            depth_expr,
        );

        // Cut/Join from a face-attached sketch defaults to targeting THAT body
        // (the one the sketch sits on) — the boolean then can't leak onto a
        // bystander body that later drifts into the tool's path, and a deleted
        // target fails loud instead of cutting something else.
        let target = if matches!(mode, ExtrudeMode::Cut | ExtrudeMode::Join) {
            self.document
                .sketch_face_refs
                .get(sketch_id)
                .and_then(|fref| fref.topology.as_ref())
                .and_then(|t| t.body_id.clone())
        } else {
            None
        };

        let extrude_node = FeatureNode {
            id: extrude_id.clone(),
            name: feature_name,
            feature: FeatureType::Extrude {
                target,
                depth,
                region_indices,
                mode,
                depth_expr,
                draft_angle_deg,
                draft_angle_expr,
            },
        };
        self.document.add_feature(extrude_node);
        self.document.add_dependency(sketch_id, &extrude_id);
        Some(extrude_id)
    }

    /// Find a sketch node by id and return its coordinate system, detected
    /// faces, and whether it sits on a body face.
    fn lookup_sketch(
        &self,
        sketch_id: &str,
    ) -> Option<(
        CoordinateSystem,
        Vec<Region>,
        bool,
        Vec<zerocad_core::ShapeLoop>,
        Vec<zerocad_core::Circle>,
        bool,
    )> {
        let var_map = self.document.variable_map();
        self.document.graph.node_indices().find_map(|idx| {
            let node = &self.document.graph[idx];
            if node.id == sketch_id {
                if let FeatureType::Sketch {
                    cs,
                    curves,
                    shapes,
                    corner_mods,
                    mirrors,
                    on_face,
                    solver,
                    ..
                } = &node.feature
                {
                    // Resolve variable-driven dimensions before detecting faces.
                    let mut eff = zerocad_core::effective_curves_solved(
                        curves,
                        shapes,
                        corner_mods,
                        mirrors,
                        solver.as_ref(),
                        &var_map,
                    );
                    // Same merge order as the evaluator: drawn curves first,
                    // then the projected face boundary (sketch-on-face), so
                    // the region indices agree with what eval will build.
                    if let Some(b) = self.document.sketch_face_boundaries.get(sketch_id) {
                        eff.extend_curves(b);
                    }
                    // Shape outlines (skipped when the sketch uses corner-mods,
                    // which take the legacy per-region path and are never boolean).
                    let loops = if corner_mods.is_empty() {
                        zerocad_core::shape_loops(shapes, &var_map)
                    } else {
                        Vec::new()
                    };
                    let circles = eff.circles.clone();
                    let draft_supported =
                        eff.circles.is_empty() && eff.arcs.is_empty() && eff.splines.is_empty();
                    return Some((
                        *cs,
                        detect_regions(&eff),
                        *on_face,
                        loops,
                        circles,
                        draft_supported,
                    ));
                }
            }
            None
        })
    }

    /// True when any sketch feeding the live extrude has drawn shapes that
    /// genuinely **cross** (a partial/edge-crossing overlap that resolves as a
    /// boolean split or fusion). The warm ghost can't render those un-booleaned,
    /// so the caller switches to the real evaluated body preview instead. Pure
    /// containment (a circle inside a rectangle) is NOT a crossing — it is a
    /// hole the ghost draws exactly and instantly (see [`shapes_cross`]).
    /// Sketches with sketch fillets/chamfers (`corner_mods`) take the legacy
    /// per-region path and are not boolean.
    pub(crate) fn op_has_crossing_shapes(&self) -> bool {
        let Some(op) = self.extrude_op.as_ref() else {
            return false;
        };
        let var_map = self.document.variable_map();
        op.targets.iter().any(|t| {
            self.document.graph.node_indices().any(|idx| {
                let node = &self.document.graph[idx];
                if node.id != t.sketch_id {
                    return false;
                }
                if let FeatureType::Sketch {
                    shapes,
                    corner_mods,
                    ..
                } = &node.feature
                {
                    if !corner_mods.is_empty() {
                        return false;
                    }
                    let loops = zerocad_core::shape_loops(shapes, &var_map);
                    loops.iter().enumerate().any(|(i, a)| {
                        loops
                            .iter()
                            .skip(i + 1)
                            .any(|b| zerocad_core::shapes_cross(a, b))
                    })
                } else {
                    false
                }
            })
        })
    }

    /// Begin a live extrude operation from a set of `(sketch_id, region_index)`
    /// faces. Opens the right-hand tool window and starts the push/pull preview
    /// instead of committing immediately.
    fn begin_extrude_op(&mut self, faces: Vec<(String, usize)>) {
        if self.extrude_op.is_some() {
            return; // already extruding
        }
        let mut by_sketch: HashMap<String, Vec<usize>> = HashMap::new();
        for (sid, ri) in faces {
            by_sketch.entry(sid).or_default().push(ri);
        }

        let mut targets = Vec::new();
        for (sid, mut idxs) in by_sketch {
            idxs.sort();
            idxs.dedup();
            if let Some((cs, regions, on_face, loops, circles, draft_supported)) =
                self.lookup_sketch(&sid)
            {
                idxs.retain(|&i| i < regions.len());
                if !idxs.is_empty() {
                    targets.push(ExtrudeTarget {
                        sketch_id: sid,
                        cs,
                        regions,
                        indices: idxs,
                        on_face,
                        loops,
                        circles,
                        draft_supported,
                    });
                }
            }
        }

        if targets.is_empty() {
            self.status_msg = "No valid faces to extrude.".to_string();
            return;
        }

        let depth = self.extrude_depth.max(1.0);
        // A sketch on a body face defaults to a direction-driven Join/Cut; a
        // sketch on an origin plane defaults to a new body. The user can still
        // override with the mode buttons (which set `mode_user_set`).
        let on_face = targets.iter().any(|t| t.on_face);
        let mode = default_extrude_mode(on_face, depth);
        self.extrude_op = Some(ExtrudeOp {
            targets,
            depth,
            depth_text: format!("{:.2}", depth),
            draft_angle_deg: 0.0,
            draft_angle_text: "0".to_string(),
            focus_request: true,
            mode,
            on_face,
            mode_user_set: false,
            pending_face_sketch: None,
        });
        // Fresh op — drop any preview cached for a previous one.
        self.extrude_preview_cache = None;
        self.extrude_preview_mesh_cache = None;
        self.extrude_ghost_base = None;
        self.status_msg =
            "Extrude: drag up/down in the viewport to push/pull, or type a distance, then OK."
                .to_string();
    }

    /// Begin a direct push/pull on a planar **body face** — no pre-drawn
    /// sketch needed. Builds a pending on-face helper sketch whose only
    /// geometry is the projected face outline, so its region(s) are exactly
    /// the face; pulling out joins material, pushing in cuts. Nothing touches
    /// the parametric graph until the user confirms.
    pub(crate) fn begin_extrude_on_body_face(&mut self, node: String, fid: u32) {
        if self.extrude_op.is_some() {
            return;
        }
        if !self.face_is_planar(&node, fid) {
            self.status_msg =
                "Extrude needs a flat body face — curved faces aren't supported yet.".to_string();
            return;
        }
        let Some(cs) = self.face_cs(&node, fid) else {
            self.status_msg = "Couldn't resolve the selected face.".to_string();
            return;
        };
        let fref = self.face_ref(&node, fid);
        let boundary = self.face_boundary_curves(&node, fid, &cs);
        let regions = detect_regions(&boundary);

        // Keep only the regions holding actual face material: project this
        // face's triangle centroids into the sketch plane and keep regions
        // containing one. (A face with a hole projects the hole rim as its own
        // disc region — no centroid lands in it, so it drops out and the hole
        // survives the push/pull.)
        let mut indices: Vec<usize> = Vec::new();
        if let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| *id == node) {
            let ntris = mesh.indices.len() / 3;
            let mut centroids: Vec<(f32, f32)> = Vec::new();
            for t in 0..ntris {
                if mesh.face_ids.get(t).copied() != Some(fid) {
                    continue;
                }
                let mut c = zerocad_core::Vec3::ZERO;
                for k in 0..3 {
                    let i = mesh.indices[t * 3 + k] as usize * 6;
                    c = c.add(zerocad_core::Vec3::new(
                        mesh.vertices[i],
                        mesh.vertices[i + 1],
                        mesh.vertices[i + 2],
                    ));
                }
                let c = c.mul(1.0 / 3.0);
                let rel = c.sub(cs.origin);
                centroids.push((rel.dot(cs.u), rel.dot(cs.v)));
            }
            indices = regions
                .iter()
                .enumerate()
                .filter(|(_, r)| centroids.iter().any(|&p| r.contains(p)))
                .map(|(i, _)| i)
                .collect();
        }
        if indices.is_empty() {
            self.status_msg = "Couldn't derive a closed outline from that face.".to_string();
            return;
        }

        let sketch_id = format!("sketch_{}", self.next_id());
        let name = self.next_sketch_name();
        let depth = self.extrude_depth.max(1.0);
        let mode = default_extrude_mode(true, depth);
        self.extrude_op = Some(ExtrudeOp {
            targets: vec![ExtrudeTarget {
                sketch_id: sketch_id.clone(),
                cs,
                regions,
                indices,
                on_face: true,
                // A direct face push/pull has no drawn overlapping shapes.
                loops: Vec::new(),
                circles: Vec::new(),
                draft_supported: boundary.circles.is_empty()
                    && boundary.arcs.is_empty()
                    && boundary.splines.is_empty(),
            }],
            depth,
            depth_text: format!("{:.2}", depth),
            draft_angle_deg: 0.0,
            draft_angle_text: "0".to_string(),
            focus_request: true,
            mode,
            on_face: true,
            mode_user_set: false,
            pending_face_sketch: Some(PendingFaceSketch {
                sketch_id,
                name,
                cs,
                fref,
                boundary,
                body_id: node,
            }),
        });
        self.selected_body.clear();
        self.extrude_preview_cache = None;
        self.extrude_preview_mesh_cache = None;
        self.extrude_ghost_base = None;
        self.status_msg =
            "Extrude face: drag along the normal to push/pull (out = Join, in = Cut), or type a distance, then OK."
                .to_string();
    }

    /// Start an extrude from the currently selected faces.
    pub(crate) fn begin_extrude_from_selection(&mut self) {
        if self.selected_faces.is_empty() {
            self.status_msg =
                "Select one or more sketch faces in the viewport first, then click Extrude."
                    .to_string();
            return;
        }
        let faces: Vec<(String, usize)> = self.selected_faces.iter().cloned().collect();
        self.begin_extrude_op(faces);
    }

    /// Start an extrude from every face of one sketch (property-panel shortcut).
    pub(crate) fn begin_extrude_whole_sketch(&mut self, sketch_id: &str) {
        let regions = self
            .lookup_sketch(sketch_id)
            .map(|(_, r, _, _, _, _)| r)
            .unwrap_or_default();
        if regions.is_empty() {
            self.status_msg =
                "That sketch has no closed faces to extrude (close your shapes).".to_string();
            return;
        }
        let faces: Vec<(String, usize)> = (0..regions.len())
            .map(|i| (sketch_id.to_string(), i))
            .collect();
        self.begin_extrude_op(faces);
    }

    /// Commit the in-progress extrude: build one body per source sketch.
    pub(crate) fn commit_extrude_op(&mut self) {
        // Resolve preview state first to avoid borrow-check conflicts
        let cached_bodies = self.cached_preview_extrude_bodies();
        let exact_preview_is_current = self.has_current_extrude_preview();
        let current_cached_bodies = cached_bodies.filter(|_| exact_preview_is_current);
        let bodies = current_cached_bodies
            .as_ref()
            .map(|bodies| (**bodies).clone())
            .unwrap_or_else(|| (*self.body_meshes).clone());
        let mesh = self.cached_preview_mesh();

        let Some(op) = self.extrude_op.take() else {
            return;
        };

        if !op.draft_angle_deg.is_finite() || op.draft_angle_deg.abs() >= 89.0 {
            self.status_msg = format!(
                "Extrude Draft must be finite and strictly between -89° and 89° (got {:.3}°).",
                op.draft_angle_deg
            );
            self.extrude_op = Some(op);
            return;
        }

        // Capture pending visual
        self.pending_visual = Some(PendingCommitVisual {
            bodies: bodies.clone(),
            mesh,
            mode: PendingVisualMode::Extrude(op.mode),
            exact_bodies: exact_preview_is_current,
        });

        // Copy preview cache if ready immediately to avoid any flash/refine delay
        if let Some(cb) = current_cached_bodies {
            self.set_body_meshes((*cb).clone());
        }

        self.push_undo();
        self.extrude_depth = op.depth; // remember for next time
        self.extrude_mode = op.mode; // remember the mode too

        if op.depth.abs() < 0.01 {
            self.clear_extrude_preview_eval();
            self.status_msg = "Extrude distance is zero — nothing created.".to_string();
            return;
        }

        // If the distance box held a variable/expression (not a bare number),
        // persist it so the extrude re-evaluates when the variable changes.
        let depth_expr = if zerocad_core::expr::preserves_source(&op.depth_text) {
            Some(op.depth_text.trim().to_string())
        } else {
            None
        };
        let draft_angle_expr = if zerocad_core::expr::preserves_source(&op.draft_angle_text) {
            Some(op.draft_angle_text.trim().to_string())
        } else {
            None
        };

        // Direct face push/pull: materialize the helper on-face sketch now
        // (inside this undo unit), so the extrude below has a real parent.
        if let Some(p) = &op.pending_face_sketch {
            insert_pending_face_sketch(&mut self.document, p);
        }

        let mut last_id = None;
        let mut count = 0;
        for t in &op.targets {
            if let Some(id) = self.build_extrude_body(
                &t.sketch_id,
                t.indices.clone(),
                op.depth,
                op.mode,
                depth_expr.clone(),
                op.draft_angle_deg,
                draft_angle_expr.clone(),
            ) {
                last_id = Some(id);
                count += 1;
                // The source sketch is consumed into the body — hide it (like
                // Fusion). The user can unhide it from the browser.
                self.hidden_nodes.insert(t.sketch_id.clone());
            }
        }

        self.selected_faces.clear();
        if let Some(id) = last_id {
            self.selected_node_id = Some(id);
        }
        self.clear_extrude_preview_eval();
        self.spawn_refine_eval();
        self.status_msg = match op.mode {
            ExtrudeMode::NewBody => {
                format!("Extruded {} new body(ies). Source sketch hidden.", count)
            }
            ExtrudeMode::Join => {
                format!(
                    "Joined {} extrude(s) into the model. Source sketch hidden.",
                    count
                )
            }
            ExtrudeMode::Cut => {
                format!(
                    "Cut {} profile(s) out of the model. Source sketch hidden.",
                    count
                )
            }
        };
    }

    /// Discard the in-progress extrude, keeping the face selection.
    pub(crate) fn cancel_extrude_op(&mut self) {
        self.extrude_op = None;
        self.clear_extrude_preview_eval();
        self.status_msg = "Extrude cancelled.".to_string();
    }
}
