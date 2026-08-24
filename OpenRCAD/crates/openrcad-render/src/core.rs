//! Device-injected render core — the reusable heart of the viewer.
//!
//! [`RenderCore`] owns every GPU resource that does **not** depend on a window
//! surface: the shaded-surface and wireframe pipelines, the globals uniform, the
//! per-face highlight-state texture, and the uploaded [`SceneMesh`]. It is
//! constructed from a borrowed `&wgpu::Device` and driven by borrowed
//! `&Device`/`&Queue` handles, so it works equally well behind the standalone
//! winit [`State`](crate::state::State) (which owns its own device) and inside a
//! host application such as ZeroCAD, which hands over eframe's shared
//! device/queue and composites the result as an egui texture.
//!
//! Rendering targets an [`OffscreenTarget`] — a multisampled color buffer that
//! resolves into a single-sampled texture plus a matching depth buffer. The host
//! samples the resolved color texture; nothing here touches a swapchain.

use openrcad_mesh::GpuMesh;
use wgpu::util::DeviceExt;

use crate::scene::{self, SceneMesh};

/// Depth format used by every offscreen target.
pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Format of the pick (face-id) target: one u32 face id + 1 per pixel.
pub const PICK_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Uint;

/// MSAA sample count. 4× is universally supported and removes the jagged edges
/// the single-sample pipeline produced on silhouettes and the wireframe overlay.
pub const SAMPLE_COUNT: u32 = 4;

/// Row width at which the face-state texture wraps to 2D. 2048 sits within
/// every backend's guaranteed maximum texture dimension (the downlevel limit),
/// so face counts are bounded by 2048² ≈ 4.2M rather than one row's width.
const FACE_STATE_ROW: u32 = 2048;

/// Everything the shader needs for one frame that is *not* geometry: the
/// combined view-projection matrix, the light, and the base surface color.
///
/// Per-face highlighting (hover / selection) is driven separately through
/// [`RenderCore::set_face_states`], not this struct.
#[derive(Clone, Copy, Debug)]
pub struct SceneGlobals {
    /// Column-major view-projection matrix (`m[col][row]`), ready for upload.
    pub view_proj: [[f32; 4]; 4],
    /// World-space light direction (need not be normalized).
    pub light_dir: [f32; 3],
    /// Ambient strength in `[0, 1]`.
    pub ambient: f32,
    /// Base surface color (linear rgb).
    pub color: [f32; 3],
    /// Render-target size in pixels — the wireframe quads expand in screen
    /// space, so the shader needs the pixel grid.
    pub viewport_px: [f32; 2],
    /// Wireframe line width in pixels (before the 1px anti-alias feather).
    pub edge_px: f32,
    /// Optional world-space clipping half-space `(nx, ny, nz, d)`, keeping
    /// points where `dot(n, point) + d >= 0`. `None` disables clipping.
    pub clip_plane: Option<[f32; 4]>,
}

impl Default for SceneGlobals {
    fn default() -> Self {
        Self {
            view_proj: identity4(),
            light_dir: [0.0, 0.0, 1.0],
            ambient: 0.25,
            color: [0.72, 0.74, 0.78],
            viewport_px: [1.0, 1.0],
            edge_px: 1.5,
            clip_plane: None,
        }
    }
}

impl SceneGlobals {
    /// Pack into the 32-float `struct Globals` layout `shader.wgsl` expects,
    /// overriding the surface color/alpha and the face-tint switch.
    fn pack(&self, color: [f32; 3], alpha: f32, face_tint: bool) -> [f32; 32] {
        let mut g = [0.0f32; 32];
        for (c, col) in self.view_proj.iter().enumerate() {
            for (r, &v) in col.iter().enumerate() {
                g[c * 4 + r] = v;
            }
        }
        g[16] = self.light_dir[0];
        g[17] = self.light_dir[1];
        g[18] = self.light_dir[2];
        g[19] = self.ambient;
        g[20] = color[0];
        g[21] = color[1];
        g[22] = color[2];
        g[23] = alpha;
        g[24] = if face_tint { 1.0 } else { 0.0 };
        g[25] = self.viewport_px[0].max(1.0);
        g[26] = self.viewport_px[1].max(1.0);
        g[27] = self.edge_px.max(0.1);
        let clip = self.clip_plane.unwrap_or([0.0, 0.0, 0.0, 1.0]);
        g[28..32].copy_from_slice(&clip);
        g
    }

    /// The base-scene packing: the globals' own color, opaque, face tint on.
    fn to_bytes(self) -> [f32; 32] {
        self.pack(self.color, 1.0, true)
    }
}

/// One occurrence of a shared uploaded geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshInstance {
    pub geometry_index: usize,
    /// Column-major local-to-world matrix.
    pub model: [[f32; 4]; 4],
    /// Added to each local face id in the highlight and pick id space.
    pub face_id_base: u32,
}

impl MeshInstance {
    pub fn identity(geometry_index: usize) -> Self {
        Self {
            geometry_index,
            model: identity4(),
            face_id_base: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InstanceUpdateStats {
    pub records_written: usize,
    pub bytes_written: u64,
    pub buffer_reallocated: bool,
}

#[derive(Clone, Copy, Debug)]
struct GpuMeshInstance {
    geometry_index: usize,
    uniform_offset: u32,
}

/// Style of one auxiliary scene layer (live-preview ghosts, tool volumes).
/// Layers draw over the base scene: opaque ones depth-test/write normally,
/// translucent ones (`alpha < 1`) blend after all opaque geometry without
/// writing depth.
#[derive(Clone, Copy, Debug)]
pub struct LayerStyle {
    /// Base surface color (linear rgb).
    pub color: [f32; 3],
    /// Coverage in `[0, 1]`; `1.0` renders through the opaque pipeline.
    pub alpha: f32,
    /// Cull back faces (a clean translucent volume) instead of drawing both
    /// sides (an x-ray ghost).
    pub cull_back: bool,
    /// Draw the dark wireframe edge overlay for this layer.
    pub draw_edges: bool,
    /// Tint this layer's faces by the face-state texture. Only meaningful when
    /// the layer's face ids were built against the same id space the host used
    /// for `set_face_states` (e.g. a preview result set shown while the base
    /// scene — the texture's usual owner — is hidden).
    pub face_tint: bool,
    /// Skip the depth test so the layer shows through opaque geometry (an
    /// x-ray overlay). Used for previews of surfaces that lie *inside* the
    /// body, e.g. the fillet/chamfer band left after material is removed —
    /// depth-tested it would be entirely occluded. Only meaningful for
    /// translucent layers (`alpha < 1`); opaque layers ignore it.
    pub xray: bool,
}

impl LayerStyle {
    fn is_opaque(&self) -> bool {
        self.alpha >= 0.999
    }
}

/// A GPU-resident auxiliary layer: uploaded geometry + style + its own globals
/// uniform (color/alpha differ per layer; view/light are shared at render time).
struct GpuLayer {
    mesh: SceneMesh,
    style: LayerStyle,
    uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// Per-face highlight state, matching the branches in `fs_main`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceHighlight {
    /// No tint.
    None,
    /// Light hover tint.
    Hover,
    /// Strong selection tint.
    Selected,
}

impl FaceHighlight {
    fn as_f32(self) -> f32 {
        match self {
            FaceHighlight::None => 0.0,
            FaceHighlight::Hover => 1.0,
            FaceHighlight::Selected => 2.0,
        }
    }
}

/// A multisampled color target that resolves into a sampleable single-sample
/// texture, paired with a matching depth buffer. The host reads
/// [`color_view`](Self::color_view) (e.g. to register it as an egui texture).
pub struct OffscreenTarget {
    width: u32,
    height: u32,
    /// Single-sample resolved color the host samples.
    color: wgpu::Texture,
    color_view: wgpu::TextureView,
    /// Multisampled color the pipeline renders into; `None` when
    /// `sample_count == 1` (the pass then draws straight into `color`).
    msaa_view: Option<wgpu::TextureView>,
    depth_view: wgpu::TextureView,
    format: wgpu::TextureFormat,
    sample_count: u32,
}

impl OffscreenTarget {
    /// Allocate an offscreen target of `width × height` pixels.
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        sample_count: u32,
        width: u32,
        height: u32,
    ) -> Self {
        let width = width.max(1);
        let height = height.max(1);
        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };

        let color = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("openrcad-render offscreen color"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            // RENDER_ATTACHMENT so it can be a resolve target; TEXTURE_BINDING so
            // the host (egui) can sample it; COPY_SRC so it can be read back (tests
            // and future viewport screenshots).
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color.create_view(&wgpu::TextureViewDescriptor::default());

        let msaa_view = (sample_count > 1).then(|| {
            let msaa = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("openrcad-render offscreen msaa"),
                size,
                mip_level_count: 1,
                sample_count,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            msaa.create_view(&wgpu::TextureViewDescriptor::default())
        });

        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("openrcad-render offscreen depth"),
            size,
            mip_level_count: 1,
            sample_count,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth.create_view(&wgpu::TextureViewDescriptor::default());

        Self {
            width,
            height,
            color,
            color_view,
            msaa_view,
            depth_view,
            format,
            sample_count,
        }
    }

    /// The single-sample resolved color texture view the host samples.
    pub fn color_view(&self) -> &wgpu::TextureView {
        &self.color_view
    }

    /// The underlying resolved color texture.
    pub fn color_texture(&self) -> &wgpu::Texture {
        &self.color
    }

    /// Current size in pixels.
    pub fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// True when this target already matches `(width, height)` and the given
    /// format/sample count — i.e. it can be reused without reallocation.
    pub fn matches(&self, format: wgpu::TextureFormat, sample_count: u32, w: u32, h: u32) -> bool {
        self.format == format
            && self.sample_count == sample_count
            && self.width == w.max(1)
            && self.height == h.max(1)
    }
}

/// A single-sample face-id target for exact cursor picking: the base scene is
/// rendered into it with [`RenderCore::render_pick`] (each pixel = face id + 1,
/// 0 = background), then individual pixels are read back with
/// [`RenderCore::read_pick_at`]. Re-render only when the scene or camera
/// changed; reads against an unchanged target are cheap.
pub struct PickTarget {
    width: u32,
    height: u32,
    tex: wgpu::Texture,
    view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    /// Three staging buffers keep hover picking asynchronous. While one buffer
    /// is waiting for the GPU, newer pointer samples can use another slot.
    readbacks: Vec<PickReadbackSlot>,
    /// Reserved for the uncommon uncached click fallback.
    blocking_readback: wgpu::Buffer,
}

struct PickReadbackSlot {
    buffer: wgpu::Buffer,
    pending: Option<PendingPickReadback>,
}

struct PendingPickReadback {
    request_id: u64,
    x: u32,
    y: u32,
    ready: std::sync::mpsc::Receiver<Result<(), wgpu::BufferAsyncError>>,
}

/// One completed asynchronous face-id sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AsyncPickResult {
    pub request_id: u64,
    pub x: u32,
    pub y: u32,
    /// `None` is the background or a failed readback.
    pub face_id: Option<u32>,
}

impl PickTarget {
    /// Allocate a pick target of `width × height` pixels.
    pub fn new(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let width = width.max(1);
        let height = height.max(1);
        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("openrcad-render pick ids"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: PICK_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("openrcad-render pick depth"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth.create_view(&wgpu::TextureViewDescriptor::default());
        let readbacks = (0..3)
            .map(|_| PickReadbackSlot {
                buffer: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("openrcad-render async pick readback"),
                    size: 256,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                }),
                pending: None,
            })
            .collect();
        let blocking_readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("openrcad-render blocking click readback"),
            size: 256,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        Self {
            width,
            height,
            tex,
            view,
            depth_view,
            readbacks,
            blocking_readback,
        }
    }

    /// True when this target already matches `(width, height)`.
    pub fn matches(&self, w: u32, h: u32) -> bool {
        self.width == w.max(1) && self.height == h.max(1)
    }
}

/// All surface-independent GPU state: pipelines, uniform, highlight texture, and
/// the uploaded scene (base mesh + optional auxiliary layers).
pub struct RenderCore {
    /// Opaque fill, both sides drawn (CAD shells carry mixed orientation).
    pipeline: wgpu::RenderPipeline,
    /// Opaque fill with back faces culled (clean opaque preview volumes).
    pipeline_cull: wgpu::RenderPipeline,
    /// Alpha-blended fill, both sides, depth-tested but not depth-written.
    pipeline_blend: wgpu::RenderPipeline,
    /// Alpha-blended fill with back faces culled.
    pipeline_blend_cull: wgpu::RenderPipeline,
    /// Alpha-blended fill that skips the depth test (x-ray overlays).
    pipeline_blend_xray: wgpu::RenderPipeline,
    /// X-ray fill with back faces culled.
    pipeline_blend_xray_cull: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    /// Face ids → R32Uint at one sample, for exact cursor picking/hover.
    pick_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group_layout: wgpu::BindGroupLayout,
    bind_group: wgpu::BindGroup,
    instance_bind_group_layout: wgpu::BindGroupLayout,
    instance_buffer: wgpu::Buffer,
    instance_bind_group: wgpu::BindGroup,
    instance_stride: u32,
    instance_buffer_capacity_bytes: u64,
    identity_instance_offset: u32,
    instances: Vec<GpuMeshInstance>,
    instance_records: Vec<MeshInstance>,
    last_instance_update: InstanceUpdateStats,
    /// One texel per face id, holding its [`FaceHighlight`] state. Laid out in
    /// rows of [`FACE_STATE_ROW`] texels (`texel(i) = (i % row, i / row)`).
    face_state_tex: wgpu::Texture,
    face_state_view: wgpu::TextureView,
    /// Total texel capacity of `face_state_tex` (width × height).
    face_state_capacity: u32,
    /// The states of the last upload, so an unchanged per-frame call is free.
    last_face_states: Vec<FaceHighlight>,
    /// Unique base-scene geometry slots. Occurrences are stored separately, so
    /// repeated components never duplicate these uploads.
    bodies: Vec<SceneMesh>,
    /// Auxiliary layers drawn over (or instead of) the base scene.
    layers: Vec<GpuLayer>,
    color_format: wgpu::TextureFormat,
    sample_count: u32,
    /// Background clear color (linear rgba). An embedding host that composites
    /// the texture over its own background wants transparent `[0,0,0,0]`.
    pub clear_color: [f64; 4],
    /// Draw the base scene this frame. Hosts flip this off while a preview
    /// layer fully replaces the committed model.
    pub base_visible: bool,
}

impl RenderCore {
    /// Build the pipelines and uniform/texture bind group for a target of
    /// `color_format` at `sample_count`.
    pub fn new(
        device: &wgpu::Device,
        color_format: wgpu::TextureFormat,
        sample_count: u32,
    ) -> Self {
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("openrcad-render globals"),
            size: std::mem::size_of::<[f32; 32]>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // A 1×1 default highlight texture; grown on demand by set_face_states.
        let (face_state_tex, face_state_capacity) = create_face_state_texture(device, 1);
        let face_state_view = face_state_tex.create_view(&wgpu::TextureViewDescriptor::default());

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("openrcad-render globals layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        // Read with textureLoad (no sampling), so filterable can
                        // be false — portable across all backends.
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
            ],
        });
        let bind_group = make_bind_group(
            device,
            &bind_group_layout,
            &uniform_buffer,
            &face_state_view,
        );
        let instance_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("openrcad-render instance layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let instance_stride = align_up(
            std::mem::size_of::<[f32; 20]>() as u32,
            device.limits().min_uniform_buffer_offset_alignment,
        );
        let identity_data = pack_instance(identity4(), 0, instance_stride as usize);
        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("openrcad-render instances"),
            contents: &identity_data,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let instance_bind_group =
            make_instance_bind_group(device, &instance_bind_group_layout, &instance_buffer);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("openrcad-render shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("openrcad-render pipeline layout"),
            bind_group_layouts: &[&bind_group_layout, &instance_bind_group_layout],
            push_constant_ranges: &[],
        });

        // Fill-pipeline variants. Opaque ones depth-write with no blending;
        // translucent ones alpha-blend into the (premultiplied-alpha) target
        // without writing depth, so they never occlude anything. The blend
        // factors produce premultiplied output over a possibly-transparent
        // background: color (SrcAlpha, OneMinusSrcAlpha), alpha (One,
        // OneMinusSrcAlpha).
        let make_fill = |label: &str, blend: bool, cull_back: bool, xray: bool| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: "vs_main",
                    buffers: &[scene::vertex_layout()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format: color_format,
                        blend: Some(if blend {
                            wgpu::BlendState {
                                color: wgpu::BlendComponent {
                                    src_factor: wgpu::BlendFactor::SrcAlpha,
                                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                                    operation: wgpu::BlendOperation::Add,
                                },
                                alpha: wgpu::BlendComponent {
                                    src_factor: wgpu::BlendFactor::One,
                                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                                    operation: wgpu::BlendOperation::Add,
                                },
                            }
                        } else {
                            wgpu::BlendState::REPLACE
                        }),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    // CAD shells can carry mixed face orientation; the base scene
                    // draws both sides. Preview volumes opt into culling.
                    cull_mode: cull_back.then_some(wgpu::Face::Back),
                    unclipped_depth: false,
                    polygon_mode: wgpu::PolygonMode::Fill,
                    conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: !blend,
                    // X-ray layers blend over everything regardless of depth
                    // (previews of surfaces that lie inside the body).
                    depth_compare: if xray {
                        wgpu::CompareFunction::Always
                    } else {
                        wgpu::CompareFunction::Less
                    },
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: sample_count,
                    ..Default::default()
                },
                multiview: None,
                cache: None,
            })
        };
        let pipeline = make_fill("openrcad-render pipeline", false, false, false);
        let pipeline_cull = make_fill("openrcad-render pipeline (cull)", false, true, false);
        let pipeline_blend = make_fill("openrcad-render pipeline (blend)", true, false, false);
        let pipeline_blend_cull =
            make_fill("openrcad-render pipeline (blend+cull)", true, true, false);
        let pipeline_blend_xray =
            make_fill("openrcad-render pipeline (blend+xray)", true, false, true);
        let pipeline_blend_xray_cull = make_fill(
            "openrcad-render pipeline (blend+xray+cull)",
            true,
            true,
            true,
        );

        // Wireframe overlay pipeline: line-list, sharing the globals bind group.
        // A negative depth bias pulls edges slightly toward the camera so they
        // sit cleanly on the shaded surface instead of z-fighting it.
        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("openrcad-render edge pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_edge",
                buffers: &[scene::edge_vertex_layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_edge",
                targets: &[Some(wgpu::ColorTargetState {
                    format: color_format,
                    // Alpha-blended so a translucent layer's wireframe fades
                    // with its fill (fs_edge emits the layer alpha) instead of
                    // stroking every far-side edge as a solid x-ray line.
                    // Opaque geometry emits alpha 1, which lands identical to
                    // the previous REPLACE state.
                    blend: Some(wgpu::BlendState {
                        color: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::SrcAlpha,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                        alpha: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::One,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                // Segments are instance-expanded into screen-aligned quads in
                // vs_edge — thick, anti-aliased, hi-dpi-correct lines that
                // hardware LineList rasterization can't provide.
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState {
                    constant: -2,
                    slope_scale: -1.0,
                    clamp: 0.0,
                },
            }),
            multisample: wgpu::MultisampleState {
                count: sample_count,
                ..Default::default()
            },
            multiview: None,
            cache: None,
        });

        // Pick pipeline: the same vertex path, but face ids written into a
        // single-sample R32Uint target with its own depth buffer, so the host
        // can read the exact face under a pixel (no blend/MSAA — ids must not
        // be filtered).
        let pick_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("openrcad-render pick pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[scene::vertex_layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_pick",
                targets: &[Some(wgpu::ColorTargetState {
                    format: PICK_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            pipeline_cull,
            pipeline_blend,
            pipeline_blend_cull,
            pipeline_blend_xray,
            pipeline_blend_xray_cull,
            edge_pipeline,
            pick_pipeline,
            uniform_buffer,
            bind_group_layout,
            bind_group,
            instance_bind_group_layout,
            instance_buffer,
            instance_bind_group,
            instance_stride,
            instance_buffer_capacity_bytes: identity_data.len() as u64,
            identity_instance_offset: 0,
            instances: Vec::new(),
            instance_records: Vec::new(),
            last_instance_update: InstanceUpdateStats::default(),
            face_state_tex,
            face_state_view,
            face_state_capacity,
            last_face_states: Vec::new(),
            bodies: Vec::new(),
            layers: Vec::new(),
            color_format,
            sample_count,
            clear_color: [0.05, 0.06, 0.08, 1.0],
            base_visible: true,
        }
    }

    /// The color format the pipelines were built for.
    pub fn color_format(&self) -> wgpu::TextureFormat {
        self.color_format
    }

    /// The MSAA sample count the pipelines were built for.
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// True once base-scene geometry has been uploaded.
    pub fn has_scene(&self) -> bool {
        !self.bodies.is_empty()
    }

    /// Replace the entire base scene with one mesh (single-body hosts).
    pub fn set_mesh(&mut self, device: &wgpu::Device, mesh: &GpuMesh) {
        self.bodies = vec![SceneMesh::upload(device, mesh)];
        self.set_instances(device, &[MeshInstance::identity(0)]);
    }

    /// Incrementally update the base scene's per-body slots. `updates[i]`
    /// carries `Some(mesh)` to (re)upload slot `i` and `None` to keep its
    /// current geometry; the base scene is truncated/grown to `updates.len()`.
    /// A host that fingerprints its bodies re-uploads only what changed —
    /// editing one body of a large assembly stops costing a whole-scene
    /// re-upload. A NEW slot must come with `Some(mesh)`.
    pub fn update_bodies(&mut self, device: &wgpu::Device, updates: &[Option<&GpuMesh>]) {
        self.update_bodies_preserving_instances(device, updates);
        let identities: Vec<MeshInstance> =
            (0..self.bodies.len()).map(MeshInstance::identity).collect();
        self.set_instances(device, &identities);
    }

    /// Update geometry slots without disturbing the occurrence buffer. Hosts
    /// with an explicit instanced scene use this to prevent a placement-only
    /// epoch from rebuilding the instance allocation as identity records.
    pub fn update_bodies_preserving_instances(
        &mut self,
        device: &wgpu::Device,
        updates: &[Option<&GpuMesh>],
    ) {
        self.bodies.truncate(updates.len());
        for (slot, update) in updates.iter().enumerate() {
            match (update, slot < self.bodies.len()) {
                (Some(mesh), true) => self.bodies[slot] = SceneMesh::upload(device, mesh),
                (Some(mesh), false) => self.bodies.push(SceneMesh::upload(device, mesh)),
                (None, true) => {}
                (None, false) => {
                    debug_assert!(false, "new body slot {slot} must carry a mesh");
                    // Keep slots aligned even in release: an empty placeholder.
                    self.bodies
                        .push(SceneMesh::upload(device, &GpuMesh::default()));
                }
            }
        }
    }

    /// Replace only the occurrence records. Geometry uploads are untouched.
    ///
    /// The records are packed into one dynamic uniform buffer, and every draw
    /// selects its transform by offset. Invalid geometry indices are ignored in
    /// release builds and asserted in debug builds.
    pub fn set_instances(&mut self, device: &wgpu::Device, instances: &[MeshInstance]) {
        let (packed, gpu_instances, instance_records, identity_offset) =
            self.pack_instances(instances);
        self.identity_instance_offset = identity_offset;
        self.instance_buffer_capacity_bytes = packed.len() as u64;
        self.instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("openrcad-render instances"),
            contents: &packed,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        self.instance_bind_group = make_instance_bind_group(
            device,
            &self.instance_bind_group_layout,
            &self.instance_buffer,
        );
        self.instances = gpu_instances;
        self.instance_records = instance_records;
        self.last_instance_update = InstanceUpdateStats {
            records_written: self.instances.len() + 1,
            bytes_written: packed.len() as u64,
            buffer_reallocated: true,
        };
    }

    /// Replace occurrence records while retaining the existing GPU buffer and
    /// bind group whenever its geometrically grown capacity is sufficient.
    pub fn set_instances_incremental(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        instances: &[MeshInstance],
    ) {
        let (packed, gpu_instances, instance_records, identity_offset) =
            self.pack_instances(instances);
        let required_bytes = packed.len() as u64;
        let same_layout = required_bytes <= self.instance_buffer_capacity_bytes
            && identity_offset == self.identity_instance_offset
            && instance_records.len() == self.instance_records.len();
        if same_layout {
            let mut records_written = 0usize;
            for (slot, (previous, next)) in self
                .instance_records
                .iter()
                .zip(&instance_records)
                .enumerate()
            {
                if previous == next {
                    continue;
                }
                let bytes =
                    pack_instance(next.model, next.face_id_base, self.instance_stride as usize);
                queue.write_buffer(
                    &self.instance_buffer,
                    slot as u64 * self.instance_stride as u64,
                    &bytes,
                );
                records_written += 1;
            }
            self.instances = gpu_instances;
            self.instance_records = instance_records;
            self.last_instance_update = InstanceUpdateStats {
                records_written,
                bytes_written: records_written as u64 * self.instance_stride as u64,
                buffer_reallocated: false,
            };
            return;
        }

        let mut buffer_reallocated = false;
        if required_bytes > self.instance_buffer_capacity_bytes {
            let capacity = required_bytes.next_power_of_two();
            self.instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("openrcad-render instances"),
                size: capacity,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.instance_bind_group = make_instance_bind_group(
                device,
                &self.instance_bind_group_layout,
                &self.instance_buffer,
            );
            self.instance_buffer_capacity_bytes = capacity;
            buffer_reallocated = true;
        }
        queue.write_buffer(&self.instance_buffer, 0, &packed);
        self.identity_instance_offset = identity_offset;
        self.instances = gpu_instances;
        self.instance_records = instance_records;
        self.last_instance_update = InstanceUpdateStats {
            records_written: self.instances.len() + 1,
            bytes_written: required_bytes,
            buffer_reallocated,
        };
    }

    pub fn instance_buffer_capacity_bytes(&self) -> u64 {
        self.instance_buffer_capacity_bytes
    }

    pub fn last_instance_update(&self) -> InstanceUpdateStats {
        self.last_instance_update
    }

    fn pack_instances(
        &self,
        instances: &[MeshInstance],
    ) -> (Vec<u8>, Vec<GpuMeshInstance>, Vec<MeshInstance>, u32) {
        let mut packed = Vec::with_capacity((instances.len() + 1) * self.instance_stride as usize);
        let mut gpu_instances = Vec::with_capacity(instances.len());
        let mut instance_records = Vec::with_capacity(instances.len());
        for instance in instances {
            if instance.geometry_index >= self.bodies.len() {
                debug_assert!(
                    false,
                    "instance references missing geometry {}",
                    instance.geometry_index
                );
                continue;
            }
            let offset = packed.len() as u32;
            packed.extend_from_slice(&pack_instance(
                instance.model,
                instance.face_id_base,
                self.instance_stride as usize,
            ));
            gpu_instances.push(GpuMeshInstance {
                geometry_index: instance.geometry_index,
                uniform_offset: offset,
            });
            instance_records.push(*instance);
        }
        let identity_offset = packed.len() as u32;
        packed.extend_from_slice(&pack_instance(
            identity4(),
            0,
            self.instance_stride as usize,
        ));
        (packed, gpu_instances, instance_records, identity_offset)
    }

    /// Drop any uploaded geometry (renders an empty scene).
    pub fn clear_mesh(&mut self) {
        self.bodies.clear();
        self.instances.clear();
        self.instance_records.clear();
    }

    /// Replace all auxiliary layers (live-preview ghosts / tool volumes). Each
    /// call re-uploads the given meshes; pass an empty slice to clear. Layers
    /// draw in slice order after the base scene — put translucent volumes last.
    pub fn set_layers(&mut self, device: &wgpu::Device, layers: &[(GpuMesh, LayerStyle)]) {
        self.layers = layers
            .iter()
            .map(|(mesh, style)| {
                let uniform = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("openrcad-render layer globals"),
                    size: std::mem::size_of::<[f32; 32]>() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let bind_group = make_bind_group(
                    device,
                    &self.bind_group_layout,
                    &uniform,
                    &self.face_state_view,
                );
                GpuLayer {
                    mesh: SceneMesh::upload(device, mesh),
                    style: *style,
                    uniform,
                    bind_group,
                }
            })
            .collect();
    }

    /// Drop all auxiliary layers.
    pub fn clear_layers(&mut self) {
        self.layers.clear();
    }

    /// Number of auxiliary layers currently uploaded.
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Upload per-face highlight states. `states[i]` is the state of face id `i`;
    /// any face id beyond the slice is treated as [`FaceHighlight::None`]. Grows
    /// the highlight texture (and rebuilds the bind group) when needed. Hosts may
    /// call this every frame: an upload with unchanged states is skipped.
    pub fn set_face_states(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        states: &[FaceHighlight],
    ) {
        let needed = states.len().max(1) as u32;
        let mut recreated = false;
        if needed > self.face_state_capacity {
            // Grow to the next power of two to avoid churning on incremental
            // selection changes.
            let mut cap = self.face_state_capacity.max(1);
            while cap < needed {
                cap = cap.saturating_mul(2);
            }
            let (tex, capacity) = create_face_state_texture(device, cap);
            self.face_state_tex = tex;
            self.face_state_capacity = capacity;
            recreated = true;
            self.face_state_view = self
                .face_state_tex
                .create_view(&wgpu::TextureViewDescriptor::default());
            // Every bind group references the texture view — rebuild them all.
            self.bind_group = make_bind_group(
                device,
                &self.bind_group_layout,
                &self.uniform_buffer,
                &self.face_state_view,
            );
            for layer in &mut self.layers {
                layer.bind_group = make_bind_group(
                    device,
                    &self.bind_group_layout,
                    &layer.uniform,
                    &self.face_state_view,
                );
            }
        }
        if !recreated && states == self.last_face_states.as_slice() {
            return;
        }
        self.last_face_states = states.to_vec();

        let width = self.face_state_capacity.min(FACE_STATE_ROW);
        let height = self.face_state_capacity.div_ceil(width);
        let mut data = vec![0.0f32; (width * height) as usize];
        for (i, s) in states.iter().enumerate().take(data.len()) {
            data[i] = s.as_f32();
        }
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &self.face_state_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            bytemuck::cast_slice(&data),
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(width * 4),
                rows_per_image: Some(height),
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
    }

    /// Render the current scene into `target`, clearing it first. Writes the
    /// globals uniform (base + one per layer) via `queue`, records a pass into
    /// a fresh encoder, and submits. After this returns, `target.color_view()`
    /// holds the frame.
    pub fn render_to(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &OffscreenTarget,
        globals: &SceneGlobals,
    ) {
        debug_assert_eq!(target.format, self.color_format);
        debug_assert_eq!(target.sample_count, self.sample_count);

        queue.write_buffer(
            &self.uniform_buffer,
            0,
            bytemuck::bytes_of(&globals.to_bytes()),
        );
        for layer in &self.layers {
            queue.write_buffer(
                &layer.uniform,
                0,
                bytemuck::bytes_of(&globals.pack(
                    layer.style.color,
                    layer.style.alpha,
                    layer.style.face_tint,
                )),
            );
        }

        // With MSAA the pass draws into the multisampled buffer and resolves
        // into the sampleable color texture; at 1 sample it draws straight in.
        let (attach_view, resolve) = match target.msaa_view.as_ref() {
            Some(msaa) => (msaa, Some(&target.color_view)),
            None => (&target.color_view, None),
        };
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("openrcad-render offscreen encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("openrcad-render offscreen pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: attach_view,
                    resolve_target: resolve,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: self.clear_color[0],
                            g: self.clear_color[1],
                            b: self.clear_color[2],
                            a: self.clear_color[3],
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            self.draw(&mut pass);
        }
        queue.submit(std::iter::once(encoder.finish()));
    }

    /// Record the scene draw calls into an existing render pass. The pass must
    /// already be configured with a color (and resolve) attachment of
    /// [`color_format`](Self::color_format) and a depth attachment, both at
    /// [`sample_count`](Self::sample_count). The globals uniforms must have been
    /// written beforehand (see [`render_to`](Self::render_to), which does both).
    ///
    /// Order: opaque fills (base + opaque layers, depth-written), their edge
    /// overlays, then translucent layers in slice order (blended, no depth
    /// write) with their edges — so ghosts never occlude solids and edges stay
    /// on top of their own fill.
    pub fn draw<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>) {
        let fill = |pass: &mut wgpu::RenderPass<'a>,
                    pipeline: &'a wgpu::RenderPipeline,
                    bind_group: &'a wgpu::BindGroup,
                    instance_bind_group: &'a wgpu::BindGroup,
                    instance_offset: u32,
                    mesh: &'a SceneMesh| {
            if mesh.index_count == 0 {
                return;
            }
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, bind_group, &[]);
            pass.set_bind_group(1, instance_bind_group, &[instance_offset]);
            pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
            pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..mesh.index_count, 0, 0..1);
        };
        // Edge quads: 6 vertices per segment instance (see vs_edge).
        let edges = |pass: &mut wgpu::RenderPass<'a>,
                     bind_group: &'a wgpu::BindGroup,
                     instance_bind_group: &'a wgpu::BindGroup,
                     instance_offset: u32,
                     mesh: &'a SceneMesh| {
            if mesh.edge_segment_count == 0 {
                return;
            }
            pass.set_pipeline(&self.edge_pipeline);
            pass.set_bind_group(0, bind_group, &[]);
            pass.set_bind_group(1, instance_bind_group, &[instance_offset]);
            pass.set_vertex_buffer(0, mesh.edge_buffer.slice(..));
            pass.draw(0..6, 0..mesh.edge_segment_count);
        };

        // 1. Opaque fills.
        if self.base_visible {
            for instance in &self.instances {
                fill(
                    pass,
                    &self.pipeline,
                    &self.bind_group,
                    &self.instance_bind_group,
                    instance.uniform_offset,
                    &self.bodies[instance.geometry_index],
                );
            }
        }
        for layer in self.layers.iter().filter(|l| l.style.is_opaque()) {
            let pipeline = if layer.style.cull_back {
                &self.pipeline_cull
            } else {
                &self.pipeline
            };
            fill(
                pass,
                pipeline,
                &layer.bind_group,
                &self.instance_bind_group,
                self.identity_instance_offset,
                &layer.mesh,
            );
        }

        // 2. Edge overlays of the opaque geometry.
        if self.base_visible {
            for instance in &self.instances {
                edges(
                    pass,
                    &self.bind_group,
                    &self.instance_bind_group,
                    instance.uniform_offset,
                    &self.bodies[instance.geometry_index],
                );
            }
        }
        for layer in self.layers.iter().filter(|l| l.style.is_opaque()) {
            if layer.style.draw_edges {
                edges(
                    pass,
                    &layer.bind_group,
                    &self.instance_bind_group,
                    self.identity_instance_offset,
                    &layer.mesh,
                );
            }
        }

        // 3. Translucent layers (and their edges), in slice order.
        for layer in self.layers.iter().filter(|l| !l.style.is_opaque()) {
            let pipeline = match (layer.style.xray, layer.style.cull_back) {
                (true, true) => &self.pipeline_blend_xray_cull,
                (true, false) => &self.pipeline_blend_xray,
                (false, true) => &self.pipeline_blend_cull,
                (false, false) => &self.pipeline_blend,
            };
            fill(
                pass,
                pipeline,
                &layer.bind_group,
                &self.instance_bind_group,
                self.identity_instance_offset,
                &layer.mesh,
            );
            if layer.style.draw_edges {
                edges(
                    pass,
                    &layer.bind_group,
                    &self.instance_bind_group,
                    self.identity_instance_offset,
                    &layer.mesh,
                );
            }
        }
    }

    /// Render the base scene's face ids into `target` (0 = background,
    /// face id + 1 otherwise). Preview layers are deliberately excluded —
    /// picking targets the committed model. Call when the scene or camera
    /// changed; then probe pixels with [`read_pick_at`](Self::read_pick_at).
    pub fn render_pick(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &PickTarget,
        globals: &SceneGlobals,
    ) {
        queue.write_buffer(
            &self.uniform_buffer,
            0,
            bytemuck::bytes_of(&globals.to_bytes()),
        );
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("openrcad-render pick encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("openrcad-render pick pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT), // id 0
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pick_pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            for instance in &self.instances {
                let body = &self.bodies[instance.geometry_index];
                if body.index_count == 0 {
                    continue;
                }
                pass.set_bind_group(1, &self.instance_bind_group, &[instance.uniform_offset]);
                pass.set_vertex_buffer(0, body.vertex_buffer.slice(..));
                pass.set_index_buffer(body.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..body.index_count, 0, 0..1);
            }
        }
        queue.submit(std::iter::once(encoder.finish()));
    }

    /// Read the face id at pixel `(x, y)` of a previously rendered `target`.
    /// Returns `None` for the background. Blocks briefly on the one-pixel
    /// GPU→CPU copy (device poll).
    pub fn read_pick_at(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &PickTarget,
        x: u32,
        y: u32,
    ) -> Option<u32> {
        if x >= target.width || y >= target.height {
            return None;
        }
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("openrcad-render pick readback encoder"),
        });
        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &target.tex,
                mip_level: 0,
                origin: wgpu::Origin3d { x, y, z: 0 },
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &target.blocking_readback,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: None, // single row
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(std::iter::once(encoder.finish()));

        let slice = target.blocking_readback.slice(0..4);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::Maintain::Wait);
        let id = {
            let data = slice.get_mapped_range();
            u32::from_le_bytes([data[0], data[1], data[2], data[3]])
        };
        target.blocking_readback.unmap();
        id.checked_sub(1)
    }

    /// Queue a non-blocking one-pixel pick readback. Returns `false` when all
    /// three slots are busy; callers should retain only their newest sample and
    /// retry next frame rather than building an unbounded queue.
    pub fn request_pick_at(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &mut PickTarget,
        request_id: u64,
        x: u32,
        y: u32,
    ) -> bool {
        if x >= target.width || y >= target.height {
            return false;
        }
        let Some(slot) = target
            .readbacks
            .iter_mut()
            .find(|slot| slot.pending.is_none())
        else {
            return false;
        };
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("openrcad-render async pick encoder"),
        });
        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &target.tex,
                mip_level: 0,
                origin: wgpu::Origin3d { x, y, z: 0 },
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &slot.buffer,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: None,
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(std::iter::once(encoder.finish()));
        let (tx, rx) = std::sync::mpsc::channel();
        slot.buffer
            .slice(0..4)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        slot.pending = Some(PendingPickReadback {
            request_id,
            x,
            y,
            ready: rx,
        });
        true
    }

    /// Collect ready asynchronous samples without waiting for the GPU.
    pub fn poll_pick_results(
        &self,
        device: &wgpu::Device,
        target: &mut PickTarget,
    ) -> Vec<AsyncPickResult> {
        let _ = device.poll(wgpu::Maintain::Poll);
        let mut completed = Vec::new();
        for slot in &mut target.readbacks {
            let ready = slot
                .pending
                .as_ref()
                .and_then(|pending| pending.ready.try_recv().ok());
            let Some(mapped) = ready else { continue };
            let pending = slot.pending.take().expect("pending pick disappeared");
            let face_id = if mapped.is_ok() {
                let slice = slot.buffer.slice(0..4);
                let data = slice.get_mapped_range();
                let raw = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                drop(data);
                raw.checked_sub(1)
            } else {
                None
            };
            slot.buffer.unmap();
            completed.push(AsyncPickResult {
                request_id: pending.request_id,
                x: pending.x,
                y: pending.y,
                face_id,
            });
        }
        completed
    }

    /// Only for the standalone [`State`](crate::state::State): expose the uniform
    /// buffer so it can pack globals with a per-frame [`SceneGlobals`].
    #[cfg(feature = "viewer-app")]
    pub(crate) fn write_globals(&self, queue: &wgpu::Queue, globals: &SceneGlobals) {
        queue.write_buffer(
            &self.uniform_buffer,
            0,
            bytemuck::bytes_of(&globals.to_bytes()),
        );
    }
}

/// Allocate a face-state texture holding at least `len` texels, wrapped into
/// rows of [`FACE_STATE_ROW`] so no dimension exceeds backend limits. Returns
/// the texture and its actual texel capacity (width × height).
fn create_face_state_texture(device: &wgpu::Device, len: u32) -> (wgpu::Texture, u32) {
    let len = len.clamp(1, FACE_STATE_ROW * FACE_STATE_ROW);
    let width = len.min(FACE_STATE_ROW);
    let height = len.div_ceil(width);
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("openrcad-render face states"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R32Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    (tex, width * height)
}

fn make_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    uniform: &wgpu::Buffer,
    face_state_view: &wgpu::TextureView,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("openrcad-render globals bind group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(face_state_view),
            },
        ],
    })
}

fn make_instance_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffer: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("openrcad-render instance bind group"),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer,
                offset: 0,
                size: std::num::NonZeroU64::new(std::mem::size_of::<[f32; 20]>() as u64),
            }),
        }],
    })
}

fn pack_instance(model: [[f32; 4]; 4], face_id_base: u32, stride: usize) -> Vec<u8> {
    let mut values = [0.0f32; 20];
    for (column, entries) in model.iter().enumerate() {
        values[column * 4..column * 4 + 4].copy_from_slice(entries);
    }
    values[16] = face_id_base as f32;
    let bytes = bytemuck::bytes_of(&values);
    let mut packed = vec![0u8; stride];
    packed[..bytes.len()].copy_from_slice(bytes);
    packed
}

fn align_up(value: u32, alignment: u32) -> u32 {
    let alignment = alignment.max(1);
    value.div_ceil(alignment) * alignment
}

fn identity4() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}
