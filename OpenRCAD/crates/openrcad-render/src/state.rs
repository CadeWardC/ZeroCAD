//! The wgpu platform layer: device/queue/surface/depth plus the render pass.
//!
//! [`State`] owns every GPU resource for one window and exposes `resize` and
//! `render`. It is intentionally small — orbit camera in [`crate::camera`],
//! geometry upload in [`crate::scene`] — so the milestone (a spinning,
//! flat-shaded box) is the whole story.

use std::sync::Arc;

use openrcad_mesh::GpuMesh;
use winit::window::Window;

use crate::camera::OrbitCamera;
use crate::pick::Picker;
use crate::scene::{self, SceneMesh};

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// MSAA sample count. 4× is universally supported and removes the jagged edges
/// the single-sample pipeline produced on silhouettes and the wireframe overlay.
const SAMPLE_COUNT: u32 = 4;

/// Pack the shader `Globals` uniform (112 bytes) into a flat float array.
///
/// Layout mirrors `struct Globals` in `shader.wgsl`: a column-major view-proj
/// matrix, then `light.xyz + ambient`, `color.rgb + pad`, and selection data.
fn globals_bytes(
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 3],
    ambient: f32,
    color: [f32; 3],
    selected_face: Option<u32>,
) -> [f32; 28] {
    let mut g = [0.0f32; 28];
    for (c, col) in view_proj.iter().enumerate() {
        for (r, &v) in col.iter().enumerate() {
            g[c * 4 + r] = v;
        }
    }
    g[16] = light_dir[0];
    g[17] = light_dir[1];
    g[18] = light_dir[2];
    g[19] = ambient;
    g[20] = color[0];
    g[21] = color[1];
    g[22] = color[2];
    if let Some(face) = selected_face {
        g[24] = face as f32;
        g[25] = 1.0;
        g[26] = 0.75;
    }
    g
}

/// All GPU state for a single viewer window.
pub struct State {
    // `window` must outlive `surface` (which borrows it as `'static` via the Arc).
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    size: winit::dpi::PhysicalSize<u32>,
    depth_view: wgpu::TextureView,
    /// Multisampled color target; resolved into the swapchain frame each draw.
    msaa_view: wgpu::TextureView,
    pipeline: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    scene: SceneMesh,
    picker: Picker,
    selected_face: Option<u32>,
    auto_spin: bool,
    /// The orbit camera; the viewer animates `yaw` to spin the model.
    pub camera: OrbitCamera,
}

impl State {
    /// Create the device/surface/pipeline and upload `mesh`.
    pub async fn new(window: Arc<Window>, mesh: &GpuMesh) -> Self {
        let size = window.inner_size();
        let size = winit::dpi::PhysicalSize::new(size.width.max(1), size.height.max(1));

        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        let surface = instance
            .create_surface(window.clone())
            .expect("create surface");

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("no suitable GPU adapter found");

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("openrcad-render device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::downlevel_defaults(),
                    memory_hints: wgpu::MemoryHints::default(),
                },
                None,
            )
            .await
            .expect("request device");

        let caps = surface.get_capabilities(&adapter);
        let format = caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(caps.formats[0]);
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        let depth_view = create_depth_view(&device, &config);
        let msaa_view = create_msaa_view(&device, &config);

        // Uniform buffer + bind group.
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("openrcad-render globals"),
            size: std::mem::size_of::<[f32; 28]>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("openrcad-render globals layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("openrcad-render globals bind group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("openrcad-render shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("openrcad-render pipeline layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("openrcad-render pipeline"),
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
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                // CAD shells can carry mixed face orientation; draw both sides.
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
            multisample: wgpu::MultisampleState {
                count: SAMPLE_COUNT,
                ..Default::default()
            },
            multiview: None,
            cache: None,
        });

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
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
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
                count: SAMPLE_COUNT,
                ..Default::default()
            },
            multiview: None,
            cache: None,
        });

        let scene = SceneMesh::upload(&device, mesh);
        let picker = Picker::from_gpu_mesh(mesh);

        Self {
            window,
            surface,
            device,
            queue,
            config,
            size,
            depth_view,
            msaa_view,
            pipeline,
            edge_pipeline,
            uniform_buffer,
            bind_group,
            scene,
            picker,
            selected_face: None,
            auto_spin: true,
            camera: OrbitCamera::default(),
        }
    }

    /// Resolve the source face id under a cursor position (pixels), if any.
    ///
    /// Shoots a ray through the cursor with the current camera and intersects
    /// the mesh; see [`crate::pick`].
    pub fn pick(&self, cursor: winit::dpi::PhysicalPosition<f64>) -> Option<u32> {
        let w = self.config.width.max(1) as f32;
        let h = self.config.height.max(1) as f32;
        let ndc_x = (cursor.x as f32 / w) * 2.0 - 1.0;
        let ndc_y = 1.0 - (cursor.y as f32 / h) * 2.0;
        let (origin, dir) = self.camera.ray(ndc_x, ndc_y, w / h);
        self.picker.pick(origin, dir)
    }

    /// Pick under the cursor and store the selected face for shader highlight.
    pub fn select_at(&mut self, cursor: winit::dpi::PhysicalPosition<f64>) -> Option<u32> {
        self.selected_face = self.pick(cursor);
        self.selected_face
    }

    /// Orbit the camera and stop the initial auto-spin.
    pub fn orbit_pixels(&mut self, dx: f32, dy: f32) {
        self.auto_spin = false;
        self.camera.orbit_pixels(dx, dy);
    }

    /// Pan the camera target and stop the initial auto-spin.
    pub fn pan_pixels(&mut self, dx: f32, dy: f32) {
        self.auto_spin = false;
        self.camera.pan_pixels(dx, dy);
    }

    /// Zoom the camera and stop the initial auto-spin.
    pub fn zoom_steps(&mut self, steps: f32) {
        self.auto_spin = false;
        self.camera.zoom_steps(steps);
    }

    /// The window this state renders into.
    pub fn window(&self) -> &Arc<Window> {
        &self.window
    }

    /// Reconfigure the surface and depth buffer after a resize.
    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width == 0 || new_size.height == 0 {
            return;
        }
        self.size = new_size;
        self.config.width = new_size.width;
        self.config.height = new_size.height;
        self.surface.configure(&self.device, &self.config);
        self.depth_view = create_depth_view(&self.device, &self.config);
        self.msaa_view = create_msaa_view(&self.device, &self.config);
    }

    /// Advance the spin animation by `dt` seconds.
    pub fn update(&mut self, dt: f32) {
        if self.auto_spin {
            self.camera.yaw += dt * 0.6;
        }
    }

    /// Draw one frame. Returns `Err` on a lost/outdated surface so the caller
    /// can reconfigure.
    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let aspect = self.config.width as f32 / self.config.height as f32;
        let view_proj = self.camera.view_proj(aspect);
        // Headlight pointing from the eye toward the target.
        let eye = self.camera.eye();
        let t = self.camera.target;
        let light_dir = [t[0] - eye[0], t[1] - eye[1], t[2] - eye[2]];
        let globals = globals_bytes(
            view_proj,
            light_dir,
            0.25,
            [0.72, 0.74, 0.78],
            self.selected_face,
        );
        self.queue
            .write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&globals));

        let frame = self.surface.get_current_texture()?;
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("openrcad-render encoder"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("openrcad-render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    // Render into the multisampled target and resolve into the
                    // swapchain frame on store.
                    view: &self.msaa_view,
                    resolve_target: Some(&view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.05,
                            g: 0.06,
                            b: 0.08,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_vertex_buffer(0, self.scene.vertex_buffer.slice(..));
            pass.set_index_buffer(self.scene.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.scene.index_count, 0, 0..1);

            // Wireframe overlay on top of the shaded surface.
            if self.scene.edge_vertex_count > 0 {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_bind_group(0, &self.bind_group, &[]);
                pass.set_vertex_buffer(0, self.scene.edge_buffer.slice(..));
                pass.draw(0..self.scene.edge_vertex_count, 0..1);
            }
        }
        self.queue.submit(std::iter::once(encoder.finish()));
        frame.present();
        Ok(())
    }
}

fn create_depth_view(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("openrcad-render depth"),
        size: wgpu::Extent3d {
            width: config.width,
            height: config.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: SAMPLE_COUNT,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}

/// The multisampled color target the pipeline draws into before resolving.
fn create_msaa_view(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("openrcad-render msaa color"),
        size: wgpu::Extent3d {
            width: config.width,
            height: config.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: SAMPLE_COUNT,
        dimension: wgpu::TextureDimension::D2,
        format: config.format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}
