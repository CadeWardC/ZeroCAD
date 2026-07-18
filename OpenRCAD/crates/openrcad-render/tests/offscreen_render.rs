//! Headless runtime validation of the reusable [`RenderCore`] + [`OffscreenTarget`].
//!
//! This is what ZeroCAD's embedded GPU viewport relies on: build a render core
//! against a device we own, upload a mesh, render into an off-screen MSAA/depth
//! target, resolve, and read the pixels back. It exercises the shader compile,
//! the pipelines, the face-state highlight texture, and the MSAA resolve — none
//! of which a pure compile check can reach.
//!
//! If no GPU adapter is available (headless CI without a software fallback), the
//! test skips rather than fails.

use openrcad_render::{
    FaceHighlight, GpuMesh, LayerStyle, OffscreenTarget, PickTarget, RenderCore, SceneGlobals,
    SAMPLE_COUNT,
};

const SIZE: u32 = 64;

fn identity() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[test]
fn render_core_draws_a_triangle_offscreen() {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let Some(adapter) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
    else {
        eprintln!("no GPU adapter available; skipping offscreen render test");
        return;
    };
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("openrcad-render test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("request device");

    let format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let mut core = RenderCore::new(&device, format, SAMPLE_COUNT);
    // Black background so a drawn (lit, white) triangle is unambiguous.
    core.clear_color = [0.0, 0.0, 0.0, 1.0];

    // A big triangle straight in clip space (identity view-projection): apex at
    // top-center, base across the bottom, so the image center is inside it and
    // the top-left corner is outside.
    let mesh = GpuMesh {
        positions: vec![
            -0.9, -0.9, 0.5, // bottom-left
            0.9, -0.9, 0.5, // bottom-right
            0.0, 0.9, 0.5, // top-center
        ],
        normals: vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
        indices: vec![0, 1, 2],
        face_ids: vec![0],
    };
    core.set_mesh(&device, &mesh);
    core.set_face_states(&device, &queue, &[FaceHighlight::Selected]);

    let target = OffscreenTarget::new(&device, format, SAMPLE_COUNT, SIZE, SIZE);
    let globals = SceneGlobals {
        view_proj: identity(),
        light_dir: [0.0, 0.0, 1.0],
        ambient: 1.0,
        color: [1.0, 1.0, 1.0],
        viewport_px: [SIZE as f32, SIZE as f32],
        edge_px: 1.5,
        clip_plane: None,
    };
    core.render_to(&device, &queue, &target, &globals);

    let pixels = read_back(&device, &queue, target.color_texture(), SIZE, SIZE);

    let px = |x: u32, y: u32| -> [u8; 4] {
        let o = ((y * SIZE + x) * 4) as usize;
        [pixels[o], pixels[o + 1], pixels[o + 2], pixels[o + 3]]
    };

    // Center is inside the triangle → a bright, lit fragment.
    let center = px(SIZE / 2, SIZE / 2);
    assert!(
        center[0] > 60 && center[1] > 40,
        "expected the triangle to shade the center bright, got {center:?}"
    );
    // Top-left corner is outside the triangle → the black clear color.
    let corner = px(1, 1);
    assert!(
        corner[0] < 30 && corner[1] < 30 && corner[2] < 30,
        "expected the corner to keep the clear color, got {corner:?}"
    );

    // The same geometry clipped by the arbitrary world plane x >= 0 must
    // retain only the right half. This exercises real fragment clipping rather
    // than merely checking the uniform packing contract.
    let clipped_target = OffscreenTarget::new(&device, format, SAMPLE_COUNT, SIZE, SIZE);
    let clipped_globals = SceneGlobals {
        clip_plane: Some([1.0, 0.0, 0.0, 0.0]),
        ..globals
    };
    core.render_to(&device, &queue, &clipped_target, &clipped_globals);
    let clipped = read_back(&device, &queue, clipped_target.color_texture(), SIZE, SIZE);
    let mut lit_left = 0usize;
    let mut lit_right = 0usize;
    for y in 0..SIZE {
        for x in 0..SIZE {
            let offset = ((y * SIZE + x) * 4) as usize;
            if clipped[offset] > 30 || clipped[offset + 1] > 30 || clipped[offset + 2] > 30 {
                if x < SIZE / 2 {
                    lit_left += 1;
                } else {
                    lit_right += 1;
                }
            }
        }
    }
    assert!(lit_right > 200, "clipped triangle disappeared: {lit_right}");
    assert!(
        lit_left * 20 < lit_right,
        "clip leaked across the plane: left={lit_left}, right={lit_right}"
    );
}

#[test]
fn face_state_texture_wraps_beyond_one_row() {
    // Face ids past the state texture's row width (2048) land on later rows.
    // Select a high face id and verify its triangle actually renders tinted —
    // this exercises the grow-to-2D upload path and the shader's row-major
    // lookup end to end.
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let Some(adapter) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
    else {
        eprintln!("no GPU adapter available; skipping face-state wrap test");
        return;
    };
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("openrcad-render wrap test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("request device");

    let format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let mut core = RenderCore::new(&device, format, SAMPLE_COUNT);
    core.clear_color = [0.0, 0.0, 0.0, 1.0];

    const FACE_ID: usize = 2500; // > 2048 → second texture row
    let mesh = GpuMesh {
        positions: vec![
            -0.9, -0.9, 0.5, //
            0.9, -0.9, 0.5, //
            0.0, 0.9, 0.5,
        ],
        normals: vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
        indices: vec![0, 1, 2],
        face_ids: vec![FACE_ID as u32],
    };
    core.set_mesh(&device, &mesh);
    let mut states = vec![FaceHighlight::None; FACE_ID + 1];
    states[FACE_ID] = FaceHighlight::Selected;
    core.set_face_states(&device, &queue, &states);

    let target = OffscreenTarget::new(&device, format, SAMPLE_COUNT, SIZE, SIZE);
    let globals = SceneGlobals {
        view_proj: identity(),
        light_dir: [0.0, 0.0, 1.0],
        ambient: 1.0,
        color: [1.0, 1.0, 1.0],
        viewport_px: [SIZE as f32, SIZE as f32],
        edge_px: 1.5,
        clip_plane: None,
    };
    core.render_to(&device, &queue, &target, &globals);

    let pixels = read_back(&device, &queue, target.color_texture(), SIZE, SIZE);
    let o = (((SIZE / 2) * SIZE + SIZE / 2) * 4) as usize;
    let center = [pixels[o], pixels[o + 1], pixels[o + 2]];
    // A selected face mixes toward the warm select color: red stays high while
    // blue drops well below an untinted white surface.
    assert!(
        center[0] > 200 && center[2] < 230 && center[0] > center[2],
        "expected the high face id to render selection-tinted, got {center:?}"
    );
}

#[test]
fn translucent_layer_blends_over_base_scene() {
    // A red half-transparent preview layer in front of a white base triangle
    // must blend (not replace) at the center, and the clear color must stay
    // transparent so the host can composite the texture over its own overlays.
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let Some(adapter) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
    else {
        eprintln!("no GPU adapter available; skipping layer blend test");
        return;
    };
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("openrcad-render layer test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("request device");

    // Non-sRGB format: values are stored raw, so readback compares in linear.
    let format = wgpu::TextureFormat::Rgba8Unorm;
    let mut core = RenderCore::new(&device, format, SAMPLE_COUNT);
    core.clear_color = [0.0, 0.0, 0.0, 0.0]; // transparent background

    let tri = |z: f32| GpuMesh {
        positions: vec![-0.9, -0.9, z, 0.9, -0.9, z, 0.0, 0.9, z],
        normals: vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
        indices: vec![0, 1, 2],
        face_ids: vec![0],
    };
    core.set_mesh(&device, &tri(0.5));
    core.set_layers(
        &device,
        &[(
            tri(0.3), // nearer than the base triangle
            LayerStyle {
                color: [1.0, 0.0, 0.0],
                alpha: 0.5,
                cull_back: false,
                draw_edges: false,
                face_tint: false,
                xray: false,
            },
        )],
    );

    let target = OffscreenTarget::new(&device, format, SAMPLE_COUNT, SIZE, SIZE);
    let globals = SceneGlobals {
        view_proj: identity(),
        light_dir: [0.0, 0.0, 1.0],
        ambient: 1.0, // shade = 1 → colors pass through unlit
        color: [1.0, 1.0, 1.0],
        viewport_px: [SIZE as f32, SIZE as f32],
        edge_px: 1.5,
        clip_plane: None,
    };
    core.render_to(&device, &queue, &target, &globals);

    let pixels = read_back(&device, &queue, target.color_texture(), SIZE, SIZE);
    let px = |x: u32, y: u32| -> [u8; 4] {
        let o = ((y * SIZE + x) * 4) as usize;
        [pixels[o], pixels[o + 1], pixels[o + 2], pixels[o + 3]]
    };

    // Center: 0.5·red over white = (1.0, 0.5, 0.5), alpha stays 1.
    let center = px(SIZE / 2, SIZE / 2);
    assert!(
        center[0] > 240 && (100..=160).contains(&center[1]) && (100..=160).contains(&center[2]),
        "expected a 50/50 red-over-white blend at center, got {center:?}"
    );
    assert!(
        center[3] > 240,
        "blended pixel must stay opaque, got {center:?}"
    );
    // Corner: fully transparent clear (host background shows through).
    let corner = px(1, 1);
    assert!(
        corner[3] < 10,
        "expected transparent clear at the corner, got {corner:?}"
    );

    // Hiding the base scene leaves only the (translucent) layer: alpha ≈ 0.5.
    core.base_visible = false;
    core.render_to(&device, &queue, &target, &globals);
    let pixels = read_back(&device, &queue, target.color_texture(), SIZE, SIZE);
    let o = (((SIZE / 2) * SIZE + SIZE / 2) * 4) as usize;
    let alpha = pixels[o + 3];
    assert!(
        (100..=160).contains(&alpha),
        "layer-only render should leave half-coverage alpha, got {alpha}"
    );
}

#[test]
fn pick_pass_reads_back_the_nearer_face_id() {
    // Two overlapping triangles at different depths and face ids: the pick
    // readback at the center must return the NEARER face's id, and a corner
    // outside all geometry must return background (None).
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let Some(adapter) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
    else {
        eprintln!("no GPU adapter available; skipping pick test");
        return;
    };
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("openrcad-render pick test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("request device");

    let mut core = RenderCore::new(&device, wgpu::TextureFormat::Rgba8UnormSrgb, SAMPLE_COUNT);
    // face 5 far (z=0.8), face 9 near (z=0.2), both covering the center.
    let mesh = GpuMesh {
        positions: vec![
            -0.9, -0.9, 0.8, 0.9, -0.9, 0.8, 0.0, 0.9, 0.8, // far triangle
            -0.5, -0.5, 0.2, 0.5, -0.5, 0.2, 0.0, 0.5, 0.2, // near triangle
        ],
        normals: [0.0f32, 0.0, 1.0].repeat(6),
        indices: vec![0, 1, 2, 3, 4, 5],
        face_ids: vec![5, 9],
    };
    core.set_mesh(&device, &mesh);

    let pick = PickTarget::new(&device, SIZE, SIZE);
    let globals = SceneGlobals {
        view_proj: identity(),
        light_dir: [0.0, 0.0, 1.0],
        ambient: 1.0,
        color: [1.0, 1.0, 1.0],
        viewport_px: [SIZE as f32, SIZE as f32],
        edge_px: 1.5,
        clip_plane: None,
    };
    core.render_pick(&device, &queue, &pick, &globals);

    let center = core.read_pick_at(&device, &queue, &pick, SIZE / 2, SIZE / 2);
    assert_eq!(center, Some(9), "expected the nearer face id at center");
    let corner = core.read_pick_at(&device, &queue, &pick, 1, 1);
    assert_eq!(corner, None, "expected background at the corner");
}

#[test]
fn msaa_off_renders_with_thick_edges() {
    // sample_count = 1 must render without a resolve target (the MSAA-off
    // setting), and the instanced edge quads must stroke a visibly dark, thick
    // boundary line over the fill.
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let Some(adapter) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
    else {
        eprintln!("no GPU adapter available; skipping msaa-off test");
        return;
    };
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("openrcad-render msaa-off test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
        },
        None,
    ))
    .expect("request device");

    let format = wgpu::TextureFormat::Rgba8Unorm;
    let mut core = RenderCore::new(&device, format, 1);
    core.clear_color = [0.0, 0.0, 0.0, 1.0];

    // A quad (two triangles, one face id) filling most of the view: its open
    // boundary edges are feature edges, so the bottom edge runs horizontally
    // near the bottom of the image.
    let mesh = GpuMesh {
        positions: vec![
            -0.8, -0.8, 0.5, 0.8, -0.8, 0.5, 0.8, 0.8, 0.5, //
            -0.8, -0.8, 0.5, 0.8, 0.8, 0.5, -0.8, 0.8, 0.5,
        ],
        normals: [0.0f32, 0.0, 1.0].repeat(6),
        indices: vec![0, 1, 2, 3, 4, 5],
        face_ids: vec![0, 0],
    };
    core.set_mesh(&device, &mesh);

    let target = OffscreenTarget::new(&device, format, 1, SIZE, SIZE);
    let globals = SceneGlobals {
        view_proj: identity(),
        light_dir: [0.0, 0.0, 1.0],
        ambient: 1.0,
        color: [1.0, 1.0, 1.0],
        viewport_px: [SIZE as f32, SIZE as f32],
        edge_px: 6.0, // deliberately wide so the stroke is unmistakable
        clip_plane: None,
    };
    core.render_to(&device, &queue, &target, &globals);

    let pixels = read_back(&device, &queue, target.color_texture(), SIZE, SIZE);
    let px = |x: u32, y: u32| -> [u8; 4] {
        let o = ((y * SIZE + x) * 4) as usize;
        [pixels[o], pixels[o + 1], pixels[o + 2], pixels[o + 3]]
    };

    // Interior: bright white fill.
    let interior = px(SIZE / 2, SIZE / 2);
    assert!(
        interior[0] > 200 && interior[1] > 200,
        "expected bright fill at center, got {interior:?}"
    );
    // On the bottom boundary edge (ndc y = -0.8 → pixel row ≈ 0.9 * SIZE): a
    // thick dark stroke.
    let edge_y = ((1.0 + 0.8) / 2.0 * SIZE as f32) as u32; // ≈ 57 for SIZE 64
    let on_edge = px(SIZE / 2, edge_y);
    assert!(
        on_edge[0] < 100 && on_edge[1] < 100,
        "expected a dark thick edge stroke at row {edge_y}, got {on_edge:?}"
    );
}

/// Copy an RGBA8 texture back to CPU memory (tight, un-padded rows). Only valid
/// when `width * 4` is already 256-byte aligned (true for `SIZE = 64`).
fn read_back(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let bytes_per_row = width * 4;
    assert_eq!(
        bytes_per_row % 256,
        0,
        "test size must give 256-aligned rows"
    );
    let size = (bytes_per_row * height) as u64;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder =
        device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    encoder.copy_texture_to_buffer(
        wgpu::ImageCopyTexture {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::ImageCopyBuffer {
            buffer: &buffer,
            layout: wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(bytes_per_row),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    queue.submit(std::iter::once(encoder.finish()));

    buffer.slice(..).map_async(wgpu::MapMode::Read, |_| {});
    device.poll(wgpu::Maintain::Wait);
    let data = buffer.slice(..).get_mapped_range().to_vec();
    buffer.unmap();
    data
}
