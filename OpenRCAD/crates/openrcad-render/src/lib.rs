#![forbid(unsafe_code)]
//! Real-time viewer for OpenRCAD models, built on wgpu.
//!
//! This crate is **not** part of the kernel and is deliberately excluded from
//! the `openrcad` facade: GPU dependencies (`wgpu`, and — for the standalone
//! window — `winit`) must never reach a core crate. It consumes the pure,
//! flat-shaded [`openrcad_mesh::GpuMesh`] buffers and turns them into pixels.
//!
//! # Two ways to use it
//!
//! * **Standalone window** (`viewer-app` feature, on by default): [`run_solid`]
//!   tessellates a [`Solid`](openrcad_topo::Solid) and opens an interactive,
//!   flat-shaded window (orbit / pan / zoom / click-to-select). This path owns a
//!   winit window and a wgpu surface via [`state::State`].
//!
//! * **Embedded** (any host): drive a [`RenderCore`] against an
//!   [`OffscreenTarget`] using a device/queue the host already owns (e.g.
//!   eframe's), then composite the resolved color texture however the host
//!   likes. No winit, no surface — build with `default-features = false` +
//!   `native-backends` to drop the window/event-loop dependencies entirely.
//!
//! ```no_run
//! # #[cfg(feature = "viewer-app")]
//! # {
//! use openrcad_foundation::Pnt;
//! use openrcad_primitives::make_box;
//!
//! let solid = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);
//! openrcad_render::run_solid(&solid, 0.01); // blocks until the window closes
//! # }
//! ```

pub mod camera;
pub mod core;
pub mod edges;
pub mod pick;
pub mod scene;

#[cfg(feature = "viewer-app")]
pub mod state;

pub use crate::core::{
    FaceHighlight, LayerStyle, OffscreenTarget, PickTarget, RenderCore, SceneGlobals, DEPTH_FORMAT,
    PICK_FORMAT, SAMPLE_COUNT,
};
pub use openrcad_mesh::GpuMesh;

/// Axis-aligned bounds `[min, max]` of a [`GpuMesh`]'s positions.
///
/// Empty meshes fall back to a unit box so a camera always has something to
/// frame.
pub fn mesh_bounds(mesh: &GpuMesh) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for v in mesh.positions.chunks_exact(3) {
        for k in 0..3 {
            min[k] = min[k].min(v[k]);
            max[k] = max[k].max(v[k]);
        }
    }
    if !min[0].is_finite() {
        return ([-0.5; 3], [0.5; 3]);
    }
    (min, max)
}

#[cfg(feature = "viewer-app")]
pub use viewer_app::{run_gpu_mesh, run_solid};

#[cfg(feature = "viewer-app")]
mod viewer_app {
    use std::sync::Arc;
    use std::time::Instant;

    use openrcad_mesh::GpuMesh;
    use winit::application::ApplicationHandler;
    use winit::dpi::PhysicalPosition;
    use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
    use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
    use winit::window::{Window, WindowId};

    use crate::mesh_bounds;
    use crate::state::State;

    /// Tessellate `solid` within `chord_err` and open an interactive, flat-shaded
    /// viewer (orbit / pan / zoom / click-to-select).
    ///
    /// Blocks the calling thread until the window is closed.
    pub fn run_solid(solid: &openrcad_topo::Solid, chord_err: f64) {
        let mesh = openrcad_mesh::tessellate(solid, chord_err, 0.5).gpu_mesh();
        run_gpu_mesh(mesh);
    }

    /// Open an interactive, flat-shaded viewer for already-tessellated GPU
    /// buffers (orbit / pan / zoom / click-to-select).
    ///
    /// Blocks the calling thread until the window is closed.
    pub fn run_gpu_mesh(mesh: GpuMesh) {
        let event_loop = EventLoop::new().expect("create event loop");
        event_loop.set_control_flow(ControlFlow::Poll);
        let mut app = App::new(mesh);
        event_loop.run_app(&mut app).expect("run viewer");
    }

    struct App {
        mesh: GpuMesh,
        bounds: ([f32; 3], [f32; 3]),
        state: Option<State>,
        last_frame: Instant,
        cursor: PhysicalPosition<f64>,
        last_cursor: PhysicalPosition<f64>,
        left_down: bool,
        pan_down: bool,
        dragged: bool,
    }

    impl App {
        fn new(mesh: GpuMesh) -> Self {
            let bounds = mesh_bounds(&mesh);
            Self {
                mesh,
                bounds,
                state: None,
                last_frame: Instant::now(),
                cursor: PhysicalPosition::new(0.0, 0.0),
                last_cursor: PhysicalPosition::new(0.0, 0.0),
                left_down: false,
                pan_down: false,
                dragged: false,
            }
        }
    }

    impl ApplicationHandler for App {
        fn resumed(&mut self, event_loop: &ActiveEventLoop) {
            if self.state.is_some() {
                return;
            }
            let attrs = Window::default_attributes().with_title("OpenRCAD Viewer");
            let window = Arc::new(event_loop.create_window(attrs).expect("create window"));
            let mut state = pollster::block_on(State::new(window, &self.mesh));
            state.camera.frame_bounds(self.bounds.0, self.bounds.1);
            self.last_frame = Instant::now();
            self.state = Some(state);
        }

        fn window_event(
            &mut self,
            event_loop: &ActiveEventLoop,
            _window_id: WindowId,
            event: WindowEvent,
        ) {
            let Some(state) = self.state.as_mut() else {
                return;
            };
            match event {
                WindowEvent::CloseRequested => event_loop.exit(),
                WindowEvent::Resized(size) => state.resize(size),
                WindowEvent::CursorMoved { position, .. } => {
                    let dx = position.x - self.last_cursor.x;
                    let dy = position.y - self.last_cursor.y;
                    self.cursor = position;
                    self.last_cursor = position;
                    if self.left_down || self.pan_down {
                        if dx.abs() + dy.abs() > 0.5 {
                            self.dragged = true;
                        }
                        if self.left_down {
                            state.orbit_pixels(dx as f32, dy as f32);
                        } else if self.pan_down {
                            state.pan_pixels(dx as f32, dy as f32);
                        }
                    }
                }
                WindowEvent::MouseInput {
                    state: button_state,
                    button,
                    ..
                } => match (button_state, button) {
                    (ElementState::Pressed, MouseButton::Left) => {
                        self.left_down = true;
                        self.dragged = false;
                    }
                    (ElementState::Released, MouseButton::Left) => {
                        self.left_down = false;
                        if !self.dragged {
                            match state.select_at(self.cursor) {
                                Some(face) => println!("Selected face {face}"),
                                None => println!("Selected nothing (background)"),
                            }
                        }
                        self.dragged = false;
                    }
                    (ElementState::Pressed, MouseButton::Middle | MouseButton::Right) => {
                        self.pan_down = true;
                        self.dragged = false;
                    }
                    (ElementState::Released, MouseButton::Middle | MouseButton::Right) => {
                        self.pan_down = false;
                        self.dragged = false;
                    }
                    _ => {}
                },
                WindowEvent::MouseWheel { delta, .. } => {
                    let steps = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(p) => (p.y as f32) / 60.0,
                    };
                    state.zoom_steps(steps);
                }
                WindowEvent::RedrawRequested => {
                    let now = Instant::now();
                    let dt = (now - self.last_frame).as_secs_f32();
                    self.last_frame = now;
                    state.update(dt);
                    match state.render() {
                        Ok(()) => {}
                        Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                            let size = state.window().inner_size();
                            state.resize(size);
                        }
                        Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                        Err(wgpu::SurfaceError::Timeout) => {}
                    }
                }
                _ => {}
            }
        }

        fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
            if let Some(state) = self.state.as_ref() {
                state.window().request_redraw();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounds_of_unit_triangle() {
        let mesh = GpuMesh {
            positions: vec![0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 1.0],
            normals: vec![0.0; 9],
            indices: vec![0, 1, 2],
            face_ids: vec![0],
        };
        let (min, max) = mesh_bounds(&mesh);
        assert_eq!(min, [0.0, 0.0, 0.0]);
        assert_eq!(max, [2.0, 3.0, 1.0]);
    }

    #[test]
    fn empty_mesh_bounds_fall_back_to_unit_box() {
        let (min, max) = mesh_bounds(&GpuMesh::default());
        assert_eq!(min, [-0.5; 3]);
        assert_eq!(max, [0.5; 3]);
    }
}
