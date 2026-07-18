//! GPU viewport: renders the committed body meshes on the GPU (via
//! `openrcad-render`'s `RenderCore`) into an off-screen texture that is then
//! composited into the egui viewport as an image, underneath the CPU-drawn 2D
//! overlays (planes, sketches, dimensions, gizmos).
//!
//! # Why an off-screen texture, not a paint callback
//!
//! egui's own wgpu render pass has no depth buffer, so 3D solids drawn directly
//! into it would not depth-test. Rendering into our own MSAA + depth target and
//! handing egui the resolved color texture (via
//! [`Renderer::register_native_texture`]) gives full depth/MSAA control and keeps
//! the compositing trivially correct.
//!
//! # Camera parity
//!
//! The 3D scene must line up pixel-for-pixel with the CPU-drawn overlays, which
//! use the hand-rolled `project_3d` in `render.rs`. [`build_view_proj`] therefore
//! reconstructs that exact mapping as a 4×4 matrix (exact in orthographic mode;
//! in perspective mode the screen-space x/y match exactly and depth stays
//! monotonic, see the derivation there).

use std::collections::HashMap;

use egui_wgpu::RenderState;
use openrcad_render::{
    mesh_bounds, FaceHighlight, GpuMesh, LayerStyle, OffscreenTarget, PickTarget, RenderCore,
    SceneGlobals,
};

use crate::*;

/// The subset of the app's camera state needed to reproduce `project_3d`.
pub(crate) struct CameraParams {
    pub pitch: f32,
    pub yaw: f32,
    pub zoom: f32,
    pub pan: egui::Vec2,
    pub is_perspective: bool,
    /// Viewport size in logical points (egui's rect size).
    pub rect_w: f32,
    pub rect_h: f32,
}

/// Weak-perspective distance used by `project_3d` (the one shared constant, so
/// the GPU matrix can never drift from the CPU projection).
use crate::render::PERSP_DIST;

/// Map from a selectable `(body node id, MockMesh face id)` to its merged GPU
/// face id (the id space shared by the highlight texture and the pick buffer).
pub(crate) type FaceIdMap = HashMap<(String, u32), u32>;

/// Everything one frame hands the GPU viewport: the committed scene, this
/// frame's preview layers, selection/hover inputs, camera, and quality knobs.
pub(crate) struct SceneFrame<'a> {
    pub body_meshes: &'a [(String, MockMesh)],
    pub epoch: u64,
    /// Draw the committed bodies (false while a preview result set replaces them).
    pub draw_bodies: bool,
    pub layers: &'a [(GpuMesh, LayerStyle)],
    /// When a preview-result layer replaces the base scene, its own dense face
    /// map + count drive the highlight texture instead of the base map (that's
    /// how a selection stays tinted while its body is previewed).
    pub preview_faces: Option<(&'a FaceIdMap, u32)>,
    pub selected_faces: &'a [(String, u32)],
    pub whole_bodies: &'a [String],
    /// Physical pixel under the cursor for GPU hover picking (`None` = no hover).
    pub hover_px: Option<(u32, u32)>,
    pub cam: CameraParams,
    pub px_w: u32,
    pub px_h: u32,
    /// MSAA sample count (already clamped to what the adapter supports).
    pub samples: u32,
    /// Wireframe line width in physical pixels.
    pub edge_px: f32,
    /// World-space section half-space passed directly to the GPU shader.
    pub clip_plane: Option<[f32; 4]>,
}

/// Per-body upload cache entry: geometry fingerprint + the derived face-id
/// shape, so an unchanged body is never re-interleaved/re-uploaded.
struct BodyCache {
    node: String,
    fingerprint: u64,
    /// Distinct faces in this body; the body owns merged ids `base..base+count`.
    face_count: u32,
    base: u32,
    /// MockMesh face id → dense local id (0..face_count).
    local: HashMap<u32, u32>,
    min: [f32; 3],
    max: [f32; 3],
}

/// All GPU state for the embedded workspace viewport.
pub(crate) struct GpuViewport {
    /// eframe's shared device/queue/renderer, refreshed each frame in `update`.
    render_state: Option<RenderState>,
    core: Option<RenderCore>,
    target: Option<OffscreenTarget>,
    texture_id: Option<egui::TextureId>,
    /// The mesh epoch last uploaded to the GPU (`u64::MAX` = nothing uploaded).
    uploaded_epoch: u64,
    /// Per-body upload cache — an epoch bump re-uploads only the bodies whose
    /// fingerprint (or merged-id base) actually changed.
    body_cache: Vec<BodyCache>,
    /// Map from a selectable `(body node id, MockMesh face id)` to the merged
    /// GpuMesh face id, so selection/hover can be turned into highlight states.
    face_map: FaceIdMap,
    /// Merged GpuMesh face id → its `(node, MockMesh face id)`, for hover picks.
    face_rev: HashMap<u32, (String, u32)>,
    /// Number of distinct face ids in the merged mesh (highlight-texture size).
    face_count: u32,
    /// World-space AABB of the uploaded mesh, for per-frame depth range.
    world_min: [f32; 3],
    world_max: [f32; 3],
    /// Fingerprint of the last-uploaded preview layers (`0` = none uploaded).
    /// An unchanged fingerprint skips the whole per-frame layer re-upload
    /// (wgpu buffer creation + feature-edge extraction) while a preview idles.
    layers_fp: u64,
    /// Union AABB of the uploaded layers (valid while `layers_fp != 0`).
    layers_min: [f32; 3],
    layers_max: [f32; 3],
    /// Face-id pick buffer for exact hover/selection under the cursor.
    pick: Option<PickTarget>,
    /// What the pick buffer currently holds: (epoch, view-proj hash, w, h).
    /// Re-rendered only when any of these change; readbacks are cheap.
    pick_key: (u64, u64, u32, u32),
    /// The face under the cursor resolved by the last hover pick — consumed as
    /// this frame's hover tint (one frame of latency, invisible in practice).
    last_hover: Option<(String, u32)>,
    /// Async hover readback bookkeeping. Context is retained per request so a
    /// result from an older camera/scene can never tint the current frame.
    pick_request_seq: u64,
    last_applied_pick_request: u64,
    pick_context: HashMap<u64, ((u64, u64, u32, u32), u32, u32)>,
    last_hover_sample: Option<((u64, u64, u32, u32), u32, u32, Option<(String, u32)>)>,
    last_hover_requested: Option<((u64, u64, u32, u32), u32, u32)>,
    /// The globals/epoch/size of the last rendered frame, so a click can probe
    /// the pick buffer on demand against exactly what was displayed.
    last_pick_ctx: Option<(SceneGlobals, u64, u32, u32)>,
    /// Full visual-state key for reusing the already-rendered offscreen texture.
    last_render_key: u64,
}

impl Default for GpuViewport {
    fn default() -> Self {
        Self {
            render_state: None,
            core: None,
            target: None,
            texture_id: None,
            uploaded_epoch: u64::MAX,
            body_cache: Vec::new(),
            face_map: HashMap::new(),
            face_rev: HashMap::new(),
            face_count: 0,
            world_min: [-0.5; 3],
            world_max: [0.5; 3],
            layers_fp: 0,
            layers_min: [0.0; 3],
            layers_max: [0.0; 3],
            pick: None,
            pick_key: (u64::MAX, 0, 0, 0),
            last_hover: None,
            pick_request_seq: 0,
            last_applied_pick_request: 0,
            pick_context: HashMap::new(),
            last_hover_sample: None,
            last_hover_requested: None,
            last_pick_ctx: None,
            last_render_key: 0,
        }
    }
}

impl GpuViewport {
    /// Refresh the eframe render state (called once per frame in `update`).
    /// Logs the adapter the first time one appears, so the chosen backend
    /// (Vulkan / DX12 / GL) is visible in the console.
    pub(crate) fn set_render_state(&mut self, rs: Option<RenderState>) {
        if self.render_state.is_none() {
            if let Some(new) = rs.as_ref() {
                let info = new.adapter.get_info();
                log::info!(
                    "GPU viewport adapter: {} — backend {:?}, driver {} {}",
                    info.name,
                    info.backend,
                    info.driver,
                    info.driver_info
                );
            }
        }
        self.render_state = rs;
    }

    /// True when eframe handed us a wgpu render state (i.e. the wgpu backend is
    /// active). If false, the GPU viewport can't run and the CPU path is used.
    pub(crate) fn is_available(&self) -> bool {
        self.render_state.is_some()
    }

    /// Human-readable description of the adapter actually in use, e.g.
    /// `NVIDIA GeForce RTX 3070 (Vulkan)`. `None` when wgpu isn't active.
    pub(crate) fn adapter_info(&self) -> Option<String> {
        let rs = self.render_state.as_ref()?;
        let info = rs.adapter.get_info();
        Some(format!("{} ({:?})", info.name, info.backend))
    }

    /// Clamp a requested MSAA sample count to what the adapter supports for
    /// both the color surface format and the depth format (8× is optional in
    /// wgpu; 1 and 4 are universal).
    pub(crate) fn clamp_samples(&self, want: u32) -> u32 {
        if want <= 1 {
            return 1;
        }
        let Some(rs) = self.render_state.as_ref() else {
            return want.min(4);
        };
        let supported = |count: u32| {
            let color = rs.adapter.get_texture_format_features(rs.target_format);
            let depth = rs
                .adapter
                .get_texture_format_features(openrcad_render::DEPTH_FORMAT);
            color.flags.sample_count_supported(count) && depth.flags.sample_count_supported(count)
        };
        if supported(want) {
            want
        } else {
            4
        }
    }

    /// Render this frame's [`SceneFrame`] and return the egui texture id to
    /// paint at the viewport rect. Returns `None` if the wgpu backend is
    /// unavailable or the target is degenerate. Also resolves the hover pick
    /// for `frame.hover_px` (consumed as the NEXT frame's hover tint).
    pub(crate) fn render(&mut self, frame: &SceneFrame<'_>) -> Option<egui::TextureId> {
        let rs = self.render_state.clone()?;
        let device = rs.device.clone();
        let queue = rs.queue.clone();
        let px_w = frame.px_w.max(1);
        let px_h = frame.px_h.max(1);
        let samples = frame.samples.max(1);

        // A changed MSAA setting rebuilds the pipelines (and forces the body
        // slots to re-upload into the fresh core).
        if self
            .core
            .as_ref()
            .is_some_and(|c| c.sample_count() != samples)
        {
            self.core = None;
            self.uploaded_epoch = u64::MAX;
            self.body_cache.clear();
            self.layers_fp = 0;
        }
        // Lazily build the render core against eframe's surface format.
        if self.core.is_none() {
            let mut core = RenderCore::new(&device, rs.target_format, samples);
            // Transparent background: the texture is composited into the egui
            // viewport AFTER the canvas, floor grid, axes, and datum marks, so
            // those show through wherever there's no geometry — and solids fully
            // cover them, exactly like the CPU painter's draw order.
            core.clear_color = [0.0, 0.0, 0.0, 0.0];
            self.core = Some(core);
        }

        // (Re)upload the committed scene when the epoch changed — per body:
        // only slots whose geometry fingerprint (or merged-face-id base) moved
        // are re-interleaved and re-uploaded, so editing one body of a large
        // assembly no longer costs a whole-scene upload.
        if frame.epoch != self.uploaded_epoch {
            let bodies = frame.body_meshes;
            let mut new_cache: Vec<BodyCache> = Vec::with_capacity(bodies.len());
            let mut rebuilt: Vec<Option<GpuMesh>> = Vec::with_capacity(bodies.len());
            let mut next_base: u32 = 0;
            for (slot, (node, mesh)) in bodies.iter().enumerate() {
                let fingerprint = mock_mesh_fingerprint(mesh);
                let cached = self
                    .body_cache
                    .get(slot)
                    .filter(|c| c.node == *node && c.fingerprint == fingerprint);
                let (local, face_count, min, max) = match cached {
                    Some(c) => (c.local.clone(), c.face_count, c.min, c.max),
                    None => body_shape(mesh),
                };
                let base = next_base;
                next_base = next_base.saturating_add(face_count);
                // Same geometry AND same id base ⇒ the uploaded buffers are
                // still exact; keep them.
                let keep = cached.is_some_and(|c| c.base == base);
                rebuilt.push((!keep).then(|| body_to_gpu(mesh, base, &local)));
                new_cache.push(BodyCache {
                    node: node.clone(),
                    fingerprint,
                    face_count,
                    base,
                    local,
                    min,
                    max,
                });
            }
            if let Some(core) = self.core.as_mut() {
                let refs: Vec<Option<&GpuMesh>> = rebuilt.iter().map(|m| m.as_ref()).collect();
                core.update_bodies(&device, &refs);
            }
            // Merged face maps + world bounds from the (partly reused) cache.
            self.face_map.clear();
            self.face_rev.clear();
            let mut wmin = [f32::INFINITY; 3];
            let mut wmax = [f32::NEG_INFINITY; 3];
            for cache in &new_cache {
                for (&mock_fid, &local_id) in &cache.local {
                    let merged = cache.base + local_id;
                    self.face_map.insert((cache.node.clone(), mock_fid), merged);
                    self.face_rev.insert(merged, (cache.node.clone(), mock_fid));
                }
                for k in 0..3 {
                    wmin[k] = wmin[k].min(cache.min[k]);
                    wmax[k] = wmax[k].max(cache.max[k]);
                }
            }
            if !wmin[0].is_finite() {
                (wmin, wmax) = ([-0.5; 3], [0.5; 3]);
            }
            self.face_count = next_base;
            self.world_min = wmin;
            self.world_max = wmax;
            self.body_cache = new_cache;
            self.uploaded_epoch = frame.epoch;
        }

        // Preview layers: re-uploaded only when their geometry/style actually
        // changed (fingerprint over styles + vertex bits). A settled preview
        // idling under a dialog costs nothing per frame — matching the epoch
        // gate the base scene gets.
        if let Some(core) = self.core.as_mut() {
            core.base_visible = frame.draw_bodies;
            if frame.layers.is_empty() {
                if core.layer_count() > 0 {
                    core.clear_layers();
                }
                self.layers_fp = 0;
            } else {
                let fp = layers_fingerprint(frame.layers);
                if fp != self.layers_fp {
                    core.set_layers(&device, frame.layers);
                    let mut lmin = [f32::INFINITY; 3];
                    let mut lmax = [f32::NEG_INFINITY; 3];
                    for (mesh, _) in frame.layers {
                        let (mn, mx) = mesh_bounds(mesh);
                        for k in 0..3 {
                            if mn[k].is_finite() && mx[k].is_finite() {
                                lmin[k] = lmin[k].min(mn[k]);
                                lmax[k] = lmax[k].max(mx[k]);
                            }
                        }
                    }
                    self.layers_min = lmin;
                    self.layers_max = lmax;
                    self.layers_fp = fp;
                }
            }
        }

        // Highlight states from the current selection / hover. While a preview
        // result set replaces the base scene, the states are built against ITS
        // dense face-id space (frame.preview_faces) so selections stay tinted
        // through the preview; otherwise against the committed map. Hover first
        // so a selection always wins the texel. (set_face_states skips the GPU
        // upload when nothing changed, so doing this every frame is cheap.)
        let (state_map, state_count): (&FaceIdMap, u32) = match frame.preview_faces {
            Some((map, count)) if !frame.draw_bodies => (map, count),
            _ => (&self.face_map, self.face_count),
        };
        if state_count > 0 {
            let mut states = vec![FaceHighlight::None; state_count as usize];
            if frame.draw_bodies {
                if let Some((node, fid)) = self.last_hover.as_ref() {
                    if let Some(&g) = state_map.get(&(node.clone(), *fid)) {
                        if let Some(s) = states.get_mut(g as usize) {
                            *s = FaceHighlight::Hover;
                        }
                    }
                }
            }
            // A whole-body selection tints every face of that body: one pass over
            // the (per-distinct-face) map, not over the mesh triangles.
            if !frame.whole_bodies.is_empty() {
                for ((node, _), &g) in state_map {
                    if frame.whole_bodies.iter().any(|w| w == node) {
                        if let Some(s) = states.get_mut(g as usize) {
                            *s = FaceHighlight::Selected;
                        }
                    }
                }
            }
            for (node, fid) in frame.selected_faces {
                if let Some(&g) = state_map.get(&(node.clone(), *fid)) {
                    if let Some(s) = states.get_mut(g as usize) {
                        *s = FaceHighlight::Selected;
                    }
                }
            }
            if let Some(core) = self.core.as_mut() {
                core.set_face_states(&device, &queue, &states);
            }
        }

        // Ensure the offscreen target matches the viewport pixel size; register
        // (or re-point) the egui texture whenever the target is (re)created.
        let need_new = self
            .target
            .as_ref()
            .map(|t| !t.matches(rs.target_format, samples, px_w, px_h))
            .unwrap_or(true);
        if need_new {
            let target = OffscreenTarget::new(&device, rs.target_format, samples, px_w, px_h);
            let mut renderer = rs.renderer.write();
            match self.texture_id {
                Some(id) => renderer.update_egui_texture_from_wgpu_texture(
                    &device,
                    target.color_view(),
                    wgpu::FilterMode::Linear,
                    id,
                ),
                None => {
                    self.texture_id = Some(renderer.register_native_texture(
                        &device,
                        target.color_view(),
                        wgpu::FilterMode::Linear,
                    ));
                }
            }
            self.target = Some(target);
        }

        // Depth range must span the preview layers too — a drag ghost pulled
        // past the committed model's AABB would otherwise leave the reserved
        // NDC z band and get clipped. Uses the AABB cached at layer upload.
        let (mut world_min, mut world_max) = (self.world_min, self.world_max);
        if self.layers_fp != 0 {
            for k in 0..3 {
                if self.layers_min[k].is_finite() && self.layers_max[k].is_finite() {
                    world_min[k] = world_min[k].min(self.layers_min[k]);
                    world_max[k] = world_max[k].max(self.layers_max[k]);
                }
            }
        }
        let view_proj = build_view_proj(&frame.cam, world_min, world_max);
        let globals = SceneGlobals {
            view_proj,
            // A soft headlight from the upper-front, matching the CPU shading's
            // general feel; direction is in world space.
            light_dir: [0.35, 0.6, 0.72],
            ambient: 0.34,
            color: BODY_COLOR,
            viewport_px: [px_w as f32, px_h as f32],
            edge_px: frame.edge_px,
            clip_plane: frame.clip_plane,
        };

        let mut render_key = view_proj_hash(&globals.view_proj);
        let mut mix = |value: u64| {
            render_key ^= value;
            render_key = render_key.wrapping_mul(0x0000_0100_0000_01b3);
        };
        mix(frame.epoch);
        mix(self.layers_fp);
        mix(((px_w as u64) << 32) | px_h as u64);
        mix(samples as u64);
        mix(frame.draw_bodies as u64);
        for component in frame.clip_plane.unwrap_or([0.0, 0.0, 0.0, 1.0]) {
            mix(component.to_bits() as u64);
        }
        for (node, face) in frame.selected_faces {
            for byte in node.as_bytes() {
                mix(*byte as u64);
            }
            mix(*face as u64);
        }
        for node in frame.whole_bodies {
            for byte in node.as_bytes() {
                mix(*byte as u64);
            }
        }
        if let Some((node, face)) = &self.last_hover {
            for byte in node.as_bytes() {
                mix(*byte as u64);
            }
            mix(*face as u64);
        }
        if need_new || render_key != self.last_render_key {
            let (core, target) = (self.core.as_ref()?, self.target.as_ref()?);
            let render_started = std::time::Instant::now();
            core.render_to(&device, &queue, target, &globals);
            let elapsed = render_started.elapsed();
            if elapsed >= std::time::Duration::from_millis(8) {
                log::debug!("GPU viewport submission: {elapsed:?}");
            }
            self.last_render_key = render_key;
        }

        // Remember this frame's pick context so a click can probe the id
        // buffer on demand (`pick_face_at`) — against exactly the frame the
        // user saw when they clicked.
        self.last_pick_ctx = Some((globals, frame.epoch, px_w, px_h));

        // Hover pick: resolve the face under the cursor for the NEXT frame's
        // tint. The id buffer re-renders only when the scene/camera/viewport
        // changed; a moving cursor over a still scene costs one 4-byte readback.
        if let Some((hx, hy)) = frame.hover_px.filter(|_| frame.draw_bodies) {
            self.poll_and_request_hover(&device, &queue, &globals, frame.epoch, px_w, px_h, hx, hy);
        } else {
            self.last_hover = None;
            self.last_hover_requested = None;
        }

        self.texture_id
    }

    /// Probe the pick buffer at pixel `(x, y)` and resolve the merged face id
    /// back to its `(node, MockMesh face id)`. Re-renders the id buffer only
    /// when (epoch, view-proj, size) moved since it was last drawn.
    #[allow(clippy::too_many_arguments)]
    fn resolve_pick(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        globals: &SceneGlobals,
        epoch: u64,
        px_w: u32,
        px_h: u32,
        x: u32,
        y: u32,
    ) -> Option<(String, u32)> {
        let stale = self
            .pick
            .as_ref()
            .map(|p| !p.matches(px_w, px_h))
            .unwrap_or(true);
        if stale {
            self.pick = Some(PickTarget::new(device, px_w, px_h));
            self.pick_key = (u64::MAX, 0, 0, 0);
        }
        let key = (epoch, view_proj_hash(&globals.view_proj), px_w, px_h);
        let (core, pick) = (self.core.as_ref()?, self.pick.as_ref()?);
        if key != self.pick_key {
            core.render_pick(device, queue, pick, globals);
            self.pick_key = key;
        }
        core.read_pick_at(device, queue, pick, x, y)
            .and_then(|id| self.face_rev.get(&id).cloned())
    }

    #[allow(clippy::too_many_arguments)]
    fn poll_and_request_hover(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        globals: &SceneGlobals,
        epoch: u64,
        px_w: u32,
        px_h: u32,
        x: u32,
        y: u32,
    ) {
        let stale = self
            .pick
            .as_ref()
            .map(|p| !p.matches(px_w, px_h))
            .unwrap_or(true);
        if stale {
            self.pick = Some(PickTarget::new(device, px_w, px_h));
            self.pick_key = (u64::MAX, 0, 0, 0);
            self.pick_context.clear();
            self.last_applied_pick_request = 0;
            self.last_hover_sample = None;
            self.last_hover_requested = None;
        }
        let key = (epoch, view_proj_hash(&globals.view_proj), px_w, px_h);
        let (Some(core), Some(pick)) = (self.core.as_ref(), self.pick.as_mut()) else {
            return;
        };
        if key != self.pick_key {
            core.render_pick(device, queue, pick, globals);
            self.pick_key = key;
        }

        for result in core.poll_pick_results(device, pick) {
            let Some((request_key, rx, ry)) = self.pick_context.remove(&result.request_id) else {
                continue;
            };
            if request_key != key {
                continue;
            }
            if result.request_id < self.last_applied_pick_request {
                continue;
            }
            self.last_applied_pick_request = result.request_id;
            let resolved = result
                .face_id
                .and_then(|id| self.face_rev.get(&id).cloned());
            self.last_hover = resolved.clone();
            self.last_hover_sample = Some((request_key, rx, ry, resolved));
        }

        let wanted = (key, x.min(px_w - 1), y.min(px_h - 1));
        if self.last_hover_requested == Some(wanted) {
            return;
        }
        self.pick_request_seq = self.pick_request_seq.wrapping_add(1);
        let request_id = self.pick_request_seq;
        if core.request_pick_at(device, queue, pick, request_id, wanted.1, wanted.2) {
            self.pick_context
                .insert(request_id, (wanted.0, wanted.1, wanted.2));
            self.last_hover_requested = Some(wanted);
        }
    }

    /// Exact face under the physical pixel `(x, y)`, from the id buffer of the
    /// last rendered frame (what the user actually saw). `Unavailable` until a
    /// GPU frame has been rendered — callers then run their CPU fallback.
    pub(crate) fn pick_face_at(&mut self, x: u32, y: u32) -> GpuFacePick {
        let Some(rs) = self.render_state.clone() else {
            return GpuFacePick::Unavailable;
        };
        let Some((globals, epoch, px_w, px_h)) = self.last_pick_ctx else {
            return GpuFacePick::Unavailable;
        };
        let device = rs.device.clone();
        let queue = rs.queue.clone();
        let key = (epoch, view_proj_hash(&globals.view_proj), px_w, px_h);
        if let Some((sample_key, sx, sy, hit)) = &self.last_hover_sample {
            if *sample_key == key && sx.abs_diff(x) <= 2 && sy.abs_diff(y) <= 2 {
                return GpuFacePick::Hit(hit.clone());
            }
        }
        GpuFacePick::Hit(self.resolve_pick(&device, &queue, &globals, epoch, px_w, px_h, x, y))
    }
}

/// Outcome of asking the GPU for the face under a pixel.
pub(crate) enum GpuFacePick {
    /// The GPU pick path can't answer (no wgpu, nothing rendered yet) — the
    /// caller should fall back to the CPU triangle scan.
    Unavailable,
    /// The pick buffer answered authoritatively: the face under the pixel, or
    /// `None` for background. No CPU fallback — the buffer IS what's on screen.
    Hit(Option<(String, u32)>),
}

/// FNV-1a over a view-projection matrix's bit pattern (pick-buffer staleness).
fn view_proj_hash(m: &[[f32; 4]; 4]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for col in m {
        for &v in col {
            h ^= v.to_bits() as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    h
}

/// FNV-1a fingerprint of a body mesh's render-relevant bits (positions/normals,
/// triangle indices, face ids). Two meshes with equal fingerprints produce the
/// same uploaded buffers, so the cached GPU slot can be kept.
fn mock_mesh_fingerprint(mesh: &MockMesh) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |v: u64| {
        h ^= v;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    mix(mesh.vertices.len() as u64);
    for &v in &mesh.vertices {
        mix(v.to_bits() as u64);
    }
    mix(mesh.indices.len() as u64);
    for &i in &mesh.indices {
        mix(i as u64);
    }
    mix(mesh.face_ids.len() as u64);
    for &f in &mesh.face_ids {
        mix(f as u64);
    }
    h
}

/// One body's face-id shape + bounds: MockMesh face id → dense local id
/// (0..count, first-appearance order, so it's deterministic), and the vertex
/// AABB.
fn body_shape(mesh: &MockMesh) -> (HashMap<u32, u32>, u32, [f32; 3], [f32; 3]) {
    let mut local: HashMap<u32, u32> = HashMap::new();
    let mut next: u32 = 0;
    let tri_count = mesh.indices.len() / 3;
    for t in 0..tri_count {
        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
        local.entry(fid).or_insert_with(|| {
            let id = next;
            next += 1;
            id
        });
    }
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for v in mesh.vertices.chunks_exact(6) {
        for k in 0..3 {
            min[k] = min[k].min(v[k]);
            max[k] = max[k].max(v[k]);
        }
    }
    if !min[0].is_finite() {
        (min, max) = ([-0.5; 3], [0.5; 3]);
    }
    (local, next, min, max)
}

/// Unweld one body into a triangle-soup [`GpuMesh`] whose face ids are the
/// body's dense local ids offset by `base` — the merged id space shared with
/// the face-state texture and the pick buffer.
fn body_to_gpu(mesh: &MockMesh, base: u32, local: &HashMap<u32, u32>) -> GpuMesh {
    let tri_count = mesh.indices.len() / 3;
    let mut positions = Vec::with_capacity(tri_count * 9);
    let mut normals = Vec::with_capacity(tri_count * 9);
    let mut face_ids = Vec::with_capacity(tri_count);
    for t in 0..tri_count {
        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
        let merged = base + local.get(&fid).copied().unwrap_or(0);
        for k in 0..3 {
            let vi = mesh.indices[t * 3 + k] as usize;
            let b = vi * 6;
            if b + 6 <= mesh.vertices.len() {
                positions.extend_from_slice(&mesh.vertices[b..b + 3]);
                normals.extend_from_slice(&mesh.vertices[b + 3..b + 6]);
            } else {
                positions.extend_from_slice(&[0.0, 0.0, 0.0]);
                normals.extend_from_slice(&[0.0, 0.0, 1.0]);
            }
        }
        face_ids.push(merged);
    }
    let vert_count = positions.len() / 3;
    GpuMesh {
        positions,
        normals,
        indices: (0..vert_count as u32).collect(),
        face_ids,
    }
}

/// Bridge the app's indexed [`MockMesh`] bodies into one unwelded triangle-soup
/// [`GpuMesh`] (as `openrcad-render`'s scene upload expects: vertex `i`'s face is
/// `face_ids[i / 3]`). Every distinct `(node id, MockMesh face id)` pair gets the
/// next sequential GpuMesh face id, so the merged id space is dense no matter
/// how sparse a body's own face ids are — the face-state highlight texture is
/// then sized by the number of *actual* faces. The returned map resolves a
/// selection to its merged id; the `u32` is the total distinct-face count.
/// Order-independent fingerprint of the preview layers (styles + vertex bit
/// patterns), used to skip the GPU re-upload when nothing changed. `0` is
/// reserved for "no layers".
fn layers_fingerprint(layers: &[(GpuMesh, LayerStyle)]) -> u64 {
    // FNV-1a over the discriminating bits.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |v: u64| {
        h ^= v;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    mix(layers.len() as u64);
    for (mesh, style) in layers {
        mix(mesh.positions.len() as u64);
        for c in style.color {
            mix(c.to_bits() as u64);
        }
        mix(style.alpha.to_bits() as u64);
        mix(style.cull_back as u64 | ((style.draw_edges as u64) << 1) | ((style.xray as u64) << 2));
        for &p in &mesh.positions {
            mix(p.to_bits() as u64);
        }
    }
    h.max(1)
}

fn mock_meshes_to_gpu(bodies: &[(String, MockMesh)]) -> (GpuMesh, FaceIdMap, u32) {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut face_ids = Vec::new();
    let mut map: FaceIdMap = HashMap::new();
    let mut next: u32 = 0;

    let total_indices: usize = bodies.iter().map(|(_, m)| m.indices.len()).sum();
    positions.reserve(total_indices * 3);
    normals.reserve(total_indices * 3);
    face_ids.reserve(total_indices / 3);

    for (node, mesh) in bodies {
        // Per-body id map keyed by the mock face id alone, so the node-id
        // String is cloned once per distinct face, not once per triangle.
        let mut local: HashMap<u32, u32> = HashMap::new();
        let tri_count = mesh.indices.len() / 3;
        for t in 0..tri_count {
            let mock_fid = mesh.face_ids.get(t).copied().unwrap_or(0);
            let gpu_fid = *local.entry(mock_fid).or_insert_with(|| {
                let id = next;
                next += 1;
                map.insert((node.clone(), mock_fid), id);
                id
            });
            for k in 0..3 {
                let vi = mesh.indices[t * 3 + k] as usize;
                let b = vi * 6;
                if b + 6 <= mesh.vertices.len() {
                    positions.extend_from_slice(&mesh.vertices[b..b + 3]);
                    normals.extend_from_slice(&mesh.vertices[b + 3..b + 6]);
                } else {
                    positions.extend_from_slice(&[0.0, 0.0, 0.0]);
                    normals.extend_from_slice(&[0.0, 0.0, 1.0]);
                }
            }
            face_ids.push(gpu_fid);
        }
    }

    let vert_count = positions.len() / 3;
    let indices: Vec<u32> = (0..vert_count as u32).collect();
    let mesh = GpuMesh {
        positions,
        normals,
        indices,
        face_ids,
    };
    (mesh, map, next)
}

/// Build the column-major view-projection matrix that reproduces `render.rs`'s
/// `project_3d`, so GPU solids align with the CPU-drawn 2D overlays.
///
/// `project_3d` maps world `(x, y, z)` to screen points via a yaw-then-pitch
/// rotation and a uniform scale `view_scale`, with an optional weak-perspective
/// factor `dist / (dist − final_z)`. Working through the egui-rect → NDC change
/// of variables, every clip component (`x`, `y`, `z`, `w`) turns out to be an
/// *affine* function of world `(x, y, z)` — so the whole mapping is one 4×4
/// matrix. Depth is remapped so the model's AABB spans `[0.05, 0.95]` in NDC z
/// (nearer = smaller), which keeps occlusion correct without a real near/far
/// frustum. In perspective mode the derivative of `ndc_z` w.r.t. `final_z` has
/// constant sign, so depth stays monotonic there too.
pub(crate) fn build_view_proj(
    cam: &CameraParams,
    world_min: [f32; 3],
    world_max: [f32; 3],
) -> [[f32; 4]; 4] {
    let (sp, cp) = (cam.pitch.sin(), cam.pitch.cos());
    let (sy, cy) = (cam.yaw.sin(), cam.yaw.cos());
    let w = cam.rect_w.max(1.0);
    let h = cam.rect_h.max(1.0);
    let view_scale = w.min(h) / (cam.zoom * 5.0).max(1e-3);

    // Linear coefficients (over x, y, z, 1) of the intermediate quantities.
    // rx = cy*x - sy*z
    let rx = [cy, 0.0, -sy, 0.0];
    // ry = -sp*sy*x + cp*y - sp*cy*z
    let ry = [-sp * sy, cp, -sp * cy, 0.0];
    // final_z = cp*sy*x + sp*y + cp*cy*z
    let fz = [cp * sy, sp, cp * cy, 0.0];

    // clip_w: 1 (ortho) or (1 - final_z/dist) (perspective).
    let cw = if cam.is_perspective {
        [
            -fz[0] / PERSP_DIST,
            -fz[1] / PERSP_DIST,
            -fz[2] / PERSP_DIST,
            1.0,
        ]
    } else {
        [0.0, 0.0, 0.0, 1.0]
    };

    // Depth range: final_z over the 8 AABB corners (final_z is affine, so its
    // extremes are at the corners).
    let (mut fz_min, mut fz_max) = (f32::INFINITY, f32::NEG_INFINITY);
    for &xi in &[world_min[0], world_max[0]] {
        for &yi in &[world_min[1], world_max[1]] {
            for &zi in &[world_min[2], world_max[2]] {
                let v = fz[0] * xi + fz[1] * yi + fz[2] * zi;
                fz_min = fz_min.min(v);
                fz_max = fz_max.max(v);
            }
        }
    }
    if !(fz_max - fz_min).is_finite() || (fz_max - fz_min) < 1e-4 {
        fz_min -= 1.0;
        fz_max += 1.0;
    }
    // Depth-band solve uses the same far clamp `project_3d` applies (final_z
    // capped at 0.85·dist ⇒ w floored at 0.15): a scene whose AABB reaches past
    // PERSP_DIST would otherwise hand the k/c solve a zero/negative w and wreck
    // the depth mapping for the *whole* scene. (Per-vertex clip_w in the matrix
    // stays unclamped — a min() isn't linear — so geometry beyond the clamp is
    // a documented divergence from the CPU projection, not a solver blow-up.)
    let w_of = |f: f32| {
        if cam.is_perspective {
            (1.0 - f / PERSP_DIST).max(0.15)
        } else {
            1.0
        }
    };
    let (w_min, w_max) = (w_of(fz_min), w_of(fz_max));
    // Solve clip_z = k*final_z + c so that ndc_z(fz_max)=0.05 (near) and
    // ndc_z(fz_min)=0.95 (far), where ndc_z = clip_z / clip_w.
    let denom = fz_max - fz_min;
    let k = (0.05 * w_max - 0.95 * w_min) / denom;
    let c = 0.05 * w_max - k * fz_max;

    // Assemble each clip row as a linear combination over (x, y, z, 1).
    let ax = 2.0 * view_scale / w; // rx scale
    let ay = 2.0 * view_scale / h; // ry scale
    let panx = 2.0 * cam.pan.x / w;
    let pany = 2.0 * cam.pan.y / h;

    let mut row_x = [0.0f32; 4];
    let mut row_y = [0.0f32; 4];
    let mut row_z = [0.0f32; 4];
    let mut row_w = [0.0f32; 4];
    for j in 0..4 {
        // clip_x = ax*rx + panx*clip_w
        row_x[j] = ax * rx[j] + panx * cw[j];
        // clip_y = ay*ry - pany*clip_w
        row_y[j] = ay * ry[j] - pany * cw[j];
        // clip_z = k*final_z + c
        row_z[j] = k * fz[j] + if j == 3 { c } else { 0.0 };
        // clip_w
        row_w[j] = cw[j];
    }

    // Column-major output: vp[col][row] = coeff[row][col], so the WGSL
    // `view_proj * vec4(pos, 1)` yields the clip rows above.
    let mut vp = [[0.0f32; 4]; 4];
    for col in 0..4 {
        vp[col][0] = row_x[col];
        vp[col][1] = row_y[col];
        vp[col][2] = row_z[col];
        vp[col][3] = row_w[col];
    }
    vp
}

/// Linear color from 8-bit sRGB components — GPU layer colors are linear (egui
/// gamma-encodes sampled native textures), while the CPU painter's palette is
/// sRGB, so preview volumes convert to keep both renderers the same hue.
/// Delegates to egui's own transfer function so the conversion can never drift
/// from what the rest of the UI uses.
fn srgb8_to_linear(r: u8, g: u8, b: u8) -> [f32; 3] {
    let f = egui::ecolor::linear_f32_from_gamma_u8;
    [f(r), f(g), f(b)]
}

/// The GPU base-scene body color (linear), shared by preview result sets so a
/// settled Join/Cut preview matches the committed look.
const BODY_COLOR: [f32; 3] = [0.78, 0.80, 0.84];

/// Flatten meshes into one unwelded triangle soup for a preview layer. Layers
/// don't tint by face state, but their wireframe IS derived from face-id
/// boundaries (`feature_edge_lines`) — zeroing the ids would leave a closed
/// preview solid with no edges at all — so each mesh's own face ids are kept,
/// offset per mesh so ids never collide across bodies.
fn mock_meshes_to_layer_soup<'a>(meshes: impl Iterator<Item = &'a MockMesh>) -> GpuMesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut face_ids = Vec::new();
    let mut fid_offset: u32 = 0;
    for mesh in meshes {
        let tri_count = mesh.indices.len() / 3;
        positions.reserve(tri_count * 9);
        normals.reserve(tri_count * 9);
        face_ids.reserve(tri_count);
        let mut max_fid: u32 = 0;
        for t in 0..tri_count {
            for k in 0..3 {
                let vi = mesh.indices[t * 3 + k] as usize;
                let b = vi * 6;
                if b + 6 <= mesh.vertices.len() {
                    positions.extend_from_slice(&mesh.vertices[b..b + 3]);
                    normals.extend_from_slice(&mesh.vertices[b + 3..b + 6]);
                } else {
                    positions.extend_from_slice(&[0.0, 0.0, 0.0]);
                    normals.extend_from_slice(&[0.0, 0.0, 1.0]);
                }
            }
            let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
            max_fid = max_fid.max(fid);
            face_ids.push(fid_offset.saturating_add(fid));
        }
        fid_offset = fid_offset.saturating_add(max_fid).saturating_add(1);
    }
    let vert_count = positions.len() / 3;
    GpuMesh {
        positions,
        normals,
        indices: (0..vert_count as u32).collect(),
        face_ids,
    }
}

/// Build the filled surface that closes a clipped viewport body. The contour
/// extractor is shared with the CPU section path and loop triangulation is
/// shared through `geom2d`, leaving only projection/unprojection backend-local.
fn section_cap_mesh<'a>(
    meshes: impl Iterator<Item = &'a MockMesh>,
    section: &SectionView,
) -> Option<GpuMesh> {
    let (origin, normal) = section.plane();
    let mut contours = Vec::new();
    for mesh in meshes {
        let clipped = zerocad_core::mock_kernel::clip_mesh_by_plane(
            mesh,
            origin,
            normal,
            section.keep_positive,
        )
        .ok()?;
        contours.extend(clipped.contours);
    }
    if contours.is_empty() {
        return None;
    }

    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let normalize = |value: [f32; 3]| {
        let length = (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt();
        [value[0] / length, value[1] / length, value[2] / length]
    };
    let reference = if normal[2].abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = normalize(cross(reference, normal));
    let v = cross(normal, u);
    let project = |point: [f32; 3]| {
        let relative = [
            point[0] - origin[0],
            point[1] - origin[1],
            point[2] - origin[2],
        ];
        egui::pos2(
            relative[0] * u[0] + relative[1] * u[1] + relative[2] * u[2],
            relative[0] * v[0] + relative[1] * v[1] + relative[2] * v[2],
        )
    };
    let loops: Vec<Vec<egui::Pos2>> = contours
        .iter()
        .map(|contour| contour.iter().copied().map(project).collect())
        .collect();
    let triangles = crate::geom2d::triangulate_nested_loops(&loops);
    if triangles.is_empty() {
        return None;
    }

    let mut span: f32 = 0.0;
    for contour in &contours {
        for point in contour {
            span = span.max((point[0] - origin[0]).abs());
            span = span.max((point[1] - origin[1]).abs());
            span = span.max((point[2] - origin[2]).abs());
        }
    }
    // Bias the cap a microscopic distance into the retained half-space. This
    // prevents a round-off-negative plane dot product from clipping the cap
    // itself without creating a visibly separate sheet.
    let kept_normal = if section.keep_positive {
        normal
    } else {
        [-normal[0], -normal[1], -normal[2]]
    };
    let bias = (span * 1.0e-6).max(1.0e-7);
    let outward = [-kept_normal[0], -kept_normal[1], -kept_normal[2]];
    let unproject = |point: egui::Pos2| {
        [
            origin[0] + u[0] * point.x + v[0] * point.y + kept_normal[0] * bias,
            origin[1] + u[1] * point.x + v[1] * point.y + kept_normal[1] * bias,
            origin[2] + u[2] * point.x + v[2] * point.y + kept_normal[2] * bias,
        ]
    };

    let mut positions = Vec::with_capacity(triangles.len() * 9);
    let mut normals = Vec::with_capacity(triangles.len() * 9);
    let mut face_ids = Vec::with_capacity(triangles.len());
    for triangle in triangles {
        for point in triangle {
            positions.extend_from_slice(&unproject(point));
            normals.extend_from_slice(&outward);
        }
        face_ids.push(0);
    }
    let vertex_count = positions.len() / 3;
    Some(GpuMesh {
        positions,
        normals,
        indices: (0..vertex_count as u32).collect(),
        face_ids,
    })
}

impl ZeroCadApp {
    /// Render the scene (committed bodies + this frame's operation preview) to
    /// the GPU offscreen texture and stash the resulting egui texture id in
    /// [`Self::gpu_texture_id`], to be composited by `draw_viewport`. No-op
    /// (clears the id) when the GPU path is unavailable.
    pub(crate) fn render_gpu_scene(&mut self, rect: egui::Rect, ctx: &egui::Context) {
        let prepare_started = std::time::Instant::now();
        if !self.gpu.is_available() {
            self.gpu_texture_id = None;
            return;
        }
        let ppp = ctx.pixels_per_point();
        let px_w = (rect.width() * ppp).round().max(1.0) as u32;
        let px_h = (rect.height() * ppp).round().max(1.0) as u32;

        // This frame's preview, from the same planner the CPU path uses, turned
        // into GPU scene layers. When a result set replaces the committed model
        // the base scene is hidden; translucent volumes come last so they blend
        // over everything opaque.
        let plan = self.resolve_preview();
        let mut layers: Vec<(GpuMesh, LayerStyle)> = Vec::new();
        let draw_bodies = plan.preview_bodies.is_none();
        // The result set's own dense face map: its layer is face-tinted against
        // it, so face/body selections stay highlighted through the preview.
        let mut preview_map: Option<(FaceIdMap, u32)> = None;
        if let Some(bodies) = plan.preview_bodies.as_ref() {
            let (mesh, map, count) = mock_meshes_to_gpu(bodies);
            layers.push((
                mesh,
                LayerStyle {
                    color: BODY_COLOR,
                    alpha: plan.body_alpha as f32 / 255.0,
                    cull_back: false,
                    draw_edges: true,
                    face_tint: true,
                    xray: false,
                },
            ));
            preview_map = Some((map, count));
        }
        let warm = srgb8_to_linear(255, 178, 96);
        match plan.preview_mode {
            // Translucent red cut volume over the ghosted result (last = on top).
            Some(ExtrudeMode::Cut) => {
                if let Some(pm) = plan.extrude_preview_mesh.as_ref() {
                    layers.push((
                        mock_meshes_to_layer_soup(std::iter::once(pm)),
                        LayerStyle {
                            color: srgb8_to_linear(232, 66, 66),
                            alpha: 90.0 / 255.0,
                            cull_back: true,
                            draw_edges: false,
                            face_tint: false,
                            xray: false,
                        },
                    ));
                }
            }
            // Until the exact fused result arrives, keep Join's additive tool
            // visually explicit. Its real outline edges must remain visible in
            // the live preview; the exact boolean result replaces this layer
            // once it is current.
            Some(ExtrudeMode::Join) => {
                if let Some(pm) = plan.extrude_preview_mesh.as_ref() {
                    layers.push((
                        mock_meshes_to_layer_soup(std::iter::once(pm)),
                        LayerStyle {
                            color: warm,
                            alpha: 1.0,
                            cull_back: true,
                            draw_edges: true,
                            face_tint: false,
                            xray: false,
                        },
                    ));
                }
            }
            // New Body stays visually distinct because it intentionally creates
            // a separate part rather than joining the current model.
            Some(ExtrudeMode::NewBody) | None => {
                if let Some(pm) = plan.extrude_preview_mesh.as_ref() {
                    layers.push((
                        mock_meshes_to_layer_soup(std::iter::once(pm)),
                        LayerStyle {
                            color: warm,
                            alpha: 1.0,
                            cull_back: true,
                            draw_edges: true,
                            face_tint: false,
                            xray: false,
                        },
                    ));
                }
            }
        }
        if let Some(mesh) = plan.mirror_preview_mesh.as_ref() {
            layers.push((
                mock_meshes_to_layer_soup(std::iter::once(mesh)),
                LayerStyle {
                    color: srgb8_to_linear(70, 170, 245),
                    alpha: 115.0 / 255.0,
                    cull_back: false,
                    draw_edges: true,
                    face_tint: false,
                    xray: false,
                },
            ));
        }
        if let Some(mesh) = plan.thread_preview_mesh.as_ref() {
            layers.push((
                mock_meshes_to_layer_soup(std::iter::once(mesh)),
                LayerStyle {
                    color: srgb8_to_linear(70, 145, 245),
                    alpha: 220.0 / 255.0,
                    cull_back: false,
                    draw_edges: true,
                    face_tint: false,
                    // The ribbon sits directly on the selected wall. X-ray it
                    // so both outside and inside thread previews remain legible.
                    xray: true,
                },
            ));
        }
        // Fillet/chamfer overlay ribbon (until the exact result bodies arrive).
        // The band is the surface LEFT AFTER the corner material is removed, so
        // it lies inside the body — depth-tested it would be invisible (only
        // its rail wires peeked through). X-ray it over the body instead.
        if let Some(pm) = plan.edge_mod_preview_mesh.as_ref() {
            layers.push((
                mock_meshes_to_layer_soup(std::iter::once(pm)),
                LayerStyle {
                    color: warm,
                    alpha: 200.0 / 255.0,
                    cull_back: false,
                    draw_edges: true,
                    face_tint: false,
                    xray: true,
                },
            ));
        }
        if let Some(section) = self.section_view.as_ref().filter(|section| section.capped) {
            let source = plan.preview_bodies.as_ref().unwrap_or(&self.body_meshes);
            if let Some(cap) = section_cap_mesh(source.iter().map(|(_, mesh)| mesh), section) {
                layers.push((
                    cap,
                    LayerStyle {
                        color: srgb8_to_linear(245, 120, 70),
                        alpha: 210.0 / 255.0,
                        cull_back: false,
                        draw_edges: false,
                        face_tint: false,
                        xray: false,
                    },
                ));
            }
        }

        let cam = CameraParams {
            pitch: self.camera_pitch,
            yaw: self.camera_yaw,
            zoom: self.camera_zoom,
            pan: self.camera_pan,
            is_perspective: self.is_perspective,
            rect_w: rect.width(),
            rect_h: rect.height(),
        };

        // Selected body faces → highlight states. A whole-body selection is
        // passed as the node id; the GPU viewport expands it over its per-face
        // map (one entry per distinct face, not per triangle).
        let mut selected_faces: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(id, pick)| match pick {
                BodyPick::Face(f) => Some((id.clone(), *f)),
                _ => None,
            })
            .collect();
        selected_faces.sort();
        // While picking a sketch plane, preview the hovered planar body face as
        // selected so the user sees which face a click will sketch on — the
        // same tint the CPU painter applies (render.rs section A).
        if self.plane_pick_active() {
            if let Some((node, fid)) = self.hovered_sketch_face.as_ref() {
                selected_faces.push((node.clone(), *fid));
            }
        }
        selected_faces.sort();
        let mut whole_bodies: Vec<String> = self
            .selected_body
            .iter()
            .filter(|(_, pick)| matches!(pick, BodyPick::Whole))
            .map(|(id, _)| id.clone())
            .collect();
        whole_bodies.sort();

        // Hover pick: only in plain select mode (no live operation, sketch, or
        // plane picking, and not mid-orbit), with the pointer inside the
        // viewport. The GPU resolves the exact face under this pixel and tints
        // it next frame.
        let hover_allowed = self.extrude_op.is_none()
            && self.edge_mod_op.is_none()
            && self.pending_visual.is_none()
            && !self.is_sketch_mode
            && !self.plane_pick_active()
            && !self.orbiting;
        let hover_px = if hover_allowed {
            ctx.pointer_latest_pos()
                .filter(|p| rect.contains(*p))
                .map(|p| {
                    (
                        ((p.x - rect.min.x) * ppp).round().max(0.0) as u32,
                        ((p.y - rect.min.y) * ppp).round().max(0.0) as u32,
                    )
                })
        } else {
            None
        };

        // GPU viewport quality knobs (persisted settings, applied live).
        let samples = self.gpu.clamp_samples(self.msaa_level.samples());
        // Match the CPU wireframe's 1.5-logical-point stroke on any DPI.
        let edge_px = 1.5 * ppp;

        let scene = SceneFrame {
            body_meshes: &self.body_meshes,
            epoch: self.mesh_epoch,
            draw_bodies,
            layers: &layers,
            preview_faces: preview_map.as_ref().map(|(m, c)| (m, *c)),
            selected_faces: &selected_faces,
            whole_bodies: &whole_bodies,
            hover_px,
            cam,
            px_w,
            px_h,
            samples,
            edge_px,
            clip_plane: self.section_view.as_ref().map(|section| {
                let (origin, mut normal) = section.plane();
                if !section.keep_positive {
                    normal = normal.map(|component| -component);
                }
                [
                    normal[0],
                    normal[1],
                    normal[2],
                    -(normal[0] * origin[0] + normal[1] * origin[1] + normal[2] * origin[2]),
                ]
            }),
        };
        let prepare_elapsed = prepare_started.elapsed();
        if prepare_elapsed >= std::time::Duration::from_millis(4) {
            log::debug!("GPU scene/preview preparation: {prepare_elapsed:?}");
        }
        self.gpu_texture_id = self.gpu.render(&scene);

        // Hand the resolved plan to this frame's `draw_viewport` so it doesn't
        // resolve (and deep-clone the preview body set) a second time.
        self.frame_preview_plan = Some(plan);
    }

    /// Ask the GPU pick buffer which face sits under the logical viewport
    /// point `pos` (from the frame the user is looking at). `Unavailable` when
    /// GPU rendering is off/not composited this frame — callers fall back to
    /// the CPU triangle scan so behavior is identical on the CPU path.
    pub(crate) fn gpu_pick_face(
        &mut self,
        pos: egui::Pos2,
        rect: egui::Rect,
        ppp: f32,
    ) -> gpu_viewport::GpuFacePick {
        if !self.gpu_render || self.gpu_texture_id.is_none() {
            return gpu_viewport::GpuFacePick::Unavailable;
        }
        let x = ((pos.x - rect.min.x) * ppp).round().max(0.0) as u32;
        let y = ((pos.y - rect.min.y) * ppp).round().max(0.0) as u32;
        self.gpu.pick_face_at(x, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merged_face_ids_are_dense_across_bodies() {
        // Two bodies with sparse/overlapping MockMesh face ids must merge into
        // dense GpuMesh ids (0..count), one per distinct (body, face) pair — the
        // highlight texture is sized from `count`, so sparse ids must not
        // inflate it.
        let mut a = MockMesh::empty();
        a.vertices = vec![0.0; 18]; // 3 vertices × [x,y,z,nx,ny,nz]
        a.indices = vec![0, 1, 2, 0, 1, 2];
        a.face_ids = vec![7, 3000];
        let mut b = MockMesh::empty();
        b.vertices = vec![0.0; 18];
        b.indices = vec![0, 1, 2];
        b.face_ids = vec![7];
        let bodies = vec![("a".to_string(), a), ("b".to_string(), b)];

        let (mesh, map, count) = mock_meshes_to_gpu(&bodies);
        assert_eq!(count, 3);
        assert_eq!(mesh.face_ids.len(), 3);
        let mut ids: Vec<u32> = map.values().copied().collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![0, 1, 2]);
        // The same MockMesh face id in different bodies stays a distinct face.
        assert_ne!(map[&("a".to_string(), 7)], map[&("b".to_string(), 7)]);
    }

    #[test]
    fn arbitrary_section_cap_is_gpu_geometry_on_the_retained_plane() {
        let body = MockMesh::make_box(10.0, 8.0, 6.0);
        let section = SectionView {
            origin: [5.0, 4.0, 3.0],
            normal: [0.3, 0.4, 0.5],
            offset: 0.0,
            keep_positive: true,
            capped: true,
        };
        let cap = section_cap_mesh(std::iter::once(&body), &section)
            .expect("an oblique box section should produce a GPU cap");
        assert!(!cap.indices.is_empty());
        assert_eq!(cap.positions.len(), cap.normals.len());
        assert_eq!(cap.indices.len() / 3, cap.face_ids.len());

        let (origin, normal) = section.plane();
        let mut area = 0.0_f32;
        for triangle in cap.positions.chunks_exact(9) {
            for point in triangle.chunks_exact(3) {
                let signed = (point[0] - origin[0]) * normal[0]
                    + (point[1] - origin[1]) * normal[1]
                    + (point[2] - origin[2]) * normal[2];
                assert!(signed >= 0.0, "cap vertex escaped the retained half-space");
                assert!(signed < 1.0e-3, "cap bias must remain visually negligible");
            }
            let a = [triangle[0], triangle[1], triangle[2]];
            let ab = [triangle[3] - a[0], triangle[4] - a[1], triangle[5] - a[2]];
            let ac = [triangle[6] - a[0], triangle[7] - a[1], triangle[8] - a[2]];
            let cross = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            area += 0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
        }
        assert!(area > 1.0, "section cap must contain non-degenerate area");
    }

    /// Reference reimplementation of `render.rs`'s `project_3d`, returning the
    /// screen offset from the rect centre plus the raw view depth `final_z`.
    /// [`build_view_proj`] must reproduce this mapping exactly (ortho) / in x-y
    /// (perspective, below the far clamp `project_3d` applies).
    fn reference_project(cam: &CameraParams, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        let (sp, cp) = (cam.pitch.sin(), cam.pitch.cos());
        let (sy, cy) = (cam.yaw.sin(), cam.yaw.cos());
        let view_scale = cam.rect_w.min(cam.rect_h) / (cam.zoom * 5.0);
        let rx = cy * x - sy * z;
        let rz = sy * x + cy * z;
        let ry = cp * y - sp * rz;
        let final_z = sp * y + cp * rz;
        let factor = if cam.is_perspective {
            PERSP_DIST / (PERSP_DIST - final_z.min(PERSP_DIST * 0.85))
        } else {
            1.0
        };
        (
            cam.pan.x + rx * view_scale * factor,
            cam.pan.y - ry * view_scale * factor,
            final_z,
        )
    }

    /// Push a world point through the matrix and map NDC back to the same
    /// centre-relative screen offset (egui y grows downward).
    fn matrix_project(
        m: &[[f32; 4]; 4],
        cam: &CameraParams,
        x: f32,
        y: f32,
        z: f32,
    ) -> (f32, f32, f32) {
        let v = [x, y, z, 1.0];
        let mut clip = [0.0f32; 4];
        for (r, c) in clip.iter_mut().enumerate() {
            for (col, vi) in v.iter().enumerate() {
                *c += m[col][r] * vi;
            }
        }
        (
            clip[0] / clip[3] * cam.rect_w / 2.0,
            -clip[1] / clip[3] * cam.rect_h / 2.0,
            clip[2] / clip[3],
        )
    }

    fn test_cam(is_perspective: bool) -> CameraParams {
        CameraParams {
            pitch: 0.5,
            yaw: 0.8,
            zoom: 10.0,
            pan: egui::vec2(30.0, -20.0),
            is_perspective,
            rect_w: 800.0,
            rect_h: 600.0,
        }
    }

    const MIN: [f32; 3] = [-40.0, -10.0, -25.0];
    const MAX: [f32; 3] = [60.0, 35.0, 45.0];

    fn sample_points() -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        for &x in &[MIN[0], 12.5, MAX[0]] {
            for &y in &[MIN[1], 7.0, MAX[1]] {
                for &z in &[MIN[2], -3.0, MAX[2]] {
                    pts.push([x, y, z]);
                }
            }
        }
        pts
    }

    #[test]
    fn view_proj_matches_project_3d() {
        for persp in [false, true] {
            let cam = test_cam(persp);
            let m = build_view_proj(&cam, MIN, MAX);
            for [x, y, z] in sample_points() {
                let (rx, ry, _rfz) = reference_project(&cam, x, y, z);
                let (mx, my, mz) = matrix_project(&m, &cam, x, y, z);
                assert!(
                    (rx - mx).abs() < 0.02 && (ry - my).abs() < 0.02,
                    "screen mismatch (persp={persp}) at ({x},{y},{z}): \
                     reference ({rx},{ry}) vs matrix ({mx},{my})"
                );
                // Depth must land inside the reserved NDC band…
                assert!(
                    (0.04..=0.96).contains(&mz),
                    "ndc depth {mz} out of band (persp={persp}) at ({x},{y},{z})"
                );
            }
            // …and preserve nearer-in-front ordering: larger final_z (nearer to
            // the camera in project_3d terms) must give a smaller NDC depth.
            let mut by_depth: Vec<(f32, f32)> = sample_points()
                .into_iter()
                .map(|[x, y, z]| {
                    let (_, _, rfz) = reference_project(&cam, x, y, z);
                    let (_, _, mz) = matrix_project(&m, &cam, x, y, z);
                    (rfz, mz)
                })
                .collect();
            by_depth.sort_by(|a, b| a.0.total_cmp(&b.0));
            for pair in by_depth.windows(2) {
                assert!(
                    pair[0].1 >= pair[1].1 - 1e-6,
                    "depth ordering flipped (persp={persp}): \
                     final_z {} → ndc {}, final_z {} → ndc {}",
                    pair[0].0,
                    pair[0].1,
                    pair[1].0,
                    pair[1].1
                );
            }
        }
    }
}
