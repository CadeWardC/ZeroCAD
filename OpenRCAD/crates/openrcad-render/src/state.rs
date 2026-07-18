//! The winit swapchain platform layer for the standalone viewer.
//!
//! [`State`] owns the window's surface, device/queue, and swapchain-sized MSAA +
//! depth targets, and drives a shared [`RenderCore`](crate::core::RenderCore) for
//! the actual drawing. It exists only for the standalone `openrcad_render::run_*`
//! viewer; a host application (e.g. ZeroCAD) skips this entirely and drives a
//! `RenderCore` against an [`OffscreenTarget`](crate::core::OffscreenTarget)
//! instead. Compiled only with the `viewer-app` feature.

use std::sync::Arc;

use openrcad_mesh::GpuMesh;
use winit::window::Window;

use crate::camera::OrbitCamera;
use crate::core::{FaceHighlight, RenderCore, SceneGlobals, DEPTH_FORMAT, SAMPLE_COUNT};
use crate::pick::Picker;

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
    core: RenderCore,
    picker: Picker,
    /// Number of distinct face ids in the uploaded mesh (highlight texture size).
    face_count: usize,
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

        let mut core = RenderCore::new(&device, format, SAMPLE_COUNT);
        core.set_mesh(&device, mesh);
        let picker = Picker::from_gpu_mesh(mesh);
        let face_count = mesh
            .face_ids
            .iter()
            .copied()
            .max()
            .map(|m| m as usize + 1)
            .unwrap_or(0);

        Self {
            window,
            surface,
            device,
            queue,
            config,
            size,
            depth_view,
            msaa_view,
            core,
            picker,
            face_count,
            selected_face: None,
            auto_spin: true,
            camera: OrbitCamera::default(),
        }
    }

    /// Resolve the source face id under a cursor position (pixels), if any.
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
        self.update_face_states();
        self.selected_face
    }

    fn update_face_states(&mut self) {
        if self.face_count == 0 {
            return;
        }
        let mut states = vec![FaceHighlight::None; self.face_count];
        if let Some(f) = self.selected_face {
            if (f as usize) < states.len() {
                states[f as usize] = FaceHighlight::Selected;
            }
        }
        self.core
            .set_face_states(&self.device, &self.queue, &states);
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
        let globals = SceneGlobals {
            view_proj,
            light_dir: [t[0] - eye[0], t[1] - eye[1], t[2] - eye[2]],
            ambient: 0.25,
            color: [0.72, 0.74, 0.78],
            viewport_px: [self.config.width as f32, self.config.height as f32],
            edge_px: 1.5,
            clip_plane: None,
        };
        self.core.write_globals(&self.queue, &globals);

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
                            r: self.core.clear_color[0],
                            g: self.core.clear_color[1],
                            b: self.core.clear_color[2],
                            a: self.core.clear_color[3],
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
            self.core.draw(&mut pass);
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
