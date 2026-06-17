//! A tiny, self-contained offscreen software rasterizer used only to produce
//! Recent-project thumbnails for the onboarding screen.
//!
//! The live viewport ([`crate::render`]) is CPU-projected too, but it is welded
//! to the app's camera/selection state and draws into an `egui::Painter`, so it
//! can't be reused to bake a small standalone image. This module instead takes
//! the already-evaluated solid meshes and rasterizes them into an RGBA buffer
//! with a fixed 3/4 view, a z-buffer, and simple flat shading — no egui, no GPU,
//! and no new dependencies. The result is cached to disk by `settings::save_thumb`.

use zerocad_core::MockMesh;

/// Fixed camera angles for the preview — the app's own default 3/4 orientation
/// (`ZeroCadApp::new`), so a thumbnail reads the same way the model first
/// appears in the viewport.
const PITCH: f32 = 0.7;
const YAW: f32 = 0.7;

/// Render `meshes` into a `size`×`size` RGBA (unmultiplied) image. Returns
/// `(width, height, rgba)`, always opaque, suitable for
/// `egui::ColorImage::from_rgba_unmultiplied`.
///
/// Empty / triangle-less input yields a plain background tile (the caller skips
/// caching in that case, but this never panics).
pub fn render_thumbnail(meshes: &[(String, MockMesh)], size: usize) -> (usize, usize, Vec<u8>) {
    const BG: [u8; 3] = [243, 245, 248]; // matches the onboarding card surface
    const BASE: [f32; 3] = [168.0, 180.0, 198.0]; // slate-blue body color
    const MARGIN: f32 = 0.12; // fraction of the frame left as padding

    let mut rgba = vec![0u8; size * size * 4];
    for px in rgba.chunks_exact_mut(4) {
        px[0] = BG[0];
        px[1] = BG[1];
        px[2] = BG[2];
        px[3] = 255;
    }

    let (sp, cp) = (PITCH.sin(), PITCH.cos());
    let (sy, cy) = (YAW.sin(), YAW.cos());
    // Rotate a world point/direction into camera space (x right, y up, z toward
    // the viewer). Mirrors the orthographic branch of `render.rs`'s `project_3d`.
    let rotate = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
        let rx = cy * x - sy * z;
        let rz = sy * x + cy * z;
        let ry = cp * y - sp * rz;
        let fz = sp * y + cp * rz;
        (rx, ry, fz)
    };

    // Camera-space 2D bounds of every vertex, to center and scale the model.
    let mut min = (f32::MAX, f32::MAX);
    let mut max = (f32::MIN, f32::MIN);
    let mut any = false;
    for (_, m) in meshes {
        for v in m.vertices.chunks_exact(6) {
            let (rx, ry, _) = rotate(v[0], v[1], v[2]);
            min.0 = min.0.min(rx);
            min.1 = min.1.min(ry);
            max.0 = max.0.max(rx);
            max.1 = max.1.max(ry);
            any = true;
        }
    }
    if !any {
        return (size, size, rgba);
    }

    let (cx, cy2) = ((min.0 + max.0) * 0.5, (min.1 + max.1) * 0.5);
    let extent = (max.0 - min.0).max(max.1 - min.1).max(1e-4);
    let scale = size as f32 * (1.0 - 2.0 * MARGIN) / extent;
    let half = size as f32 * 0.5;
    // World → screen (pixel) + depth. Larger depth = nearer the camera.
    let project = |x: f32, y: f32, z: f32| -> (f32, f32, f32) {
        let (rx, ry, fz) = rotate(x, y, z);
        ((rx - cx) * scale + half, half - (ry - cy2) * scale, fz)
    };

    // Light from the upper-front in camera space, so shading tracks the view
    // rather than the model's world orientation.
    let light = normalize3([-0.35, 0.55, 1.0]);

    let mut zbuf = vec![f32::MIN; size * size];
    for (_, m) in meshes {
        for tri in m.indices.chunks_exact(3) {
            let p: Vec<(f32, f32, f32)> = tri
                .iter()
                .map(|&i| {
                    let b = i as usize * 6;
                    project(m.vertices[b], m.vertices[b + 1], m.vertices[b + 2])
                })
                .collect();
            // Flat shade from the first vertex's normal (rotated into camera space).
            let nb = tri[0] as usize * 6;
            let (nx, ny, nz) = rotate(
                m.vertices[nb + 3],
                m.vertices[nb + 4],
                m.vertices[nb + 5],
            );
            let n = normalize3([nx, ny, nz]);
            // Two-sided: meshes are outward-facing, but a stray inverted normal
            // shouldn't render a face pure black.
            let diff = (n[0] * light[0] + n[1] * light[1] + n[2] * light[2]).abs();
            let shade = (0.35 + 0.65 * diff).clamp(0.0, 1.0);
            let color = [
                (BASE[0] * shade) as u8,
                (BASE[1] * shade) as u8,
                (BASE[2] * shade) as u8,
            ];
            fill_triangle(&mut rgba, &mut zbuf, size, p[0], p[1], p[2], color);
        }
    }

    (size, size, rgba)
}

fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l < 1e-6 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / l, v[1] / l, v[2] / l]
    }
}

/// Barycentric filled-triangle rasterizer with a per-pixel depth test.
#[allow(clippy::too_many_arguments)]
fn fill_triangle(
    rgba: &mut [u8],
    zbuf: &mut [f32],
    size: usize,
    a: (f32, f32, f32),
    b: (f32, f32, f32),
    c: (f32, f32, f32),
    color: [u8; 3],
) {
    let den = (b.1 - c.1) * (a.0 - c.0) + (c.0 - b.0) * (a.1 - c.1);
    if den.abs() < 1e-9 {
        return;
    }
    let minx = a.0.min(b.0).min(c.0).floor().max(0.0) as i32;
    let maxx = (a.0.max(b.0).max(c.0).ceil() as i32).min(size as i32 - 1);
    let miny = a.1.min(b.1).min(c.1).floor().max(0.0) as i32;
    let maxy = (a.1.max(b.1).max(c.1).ceil() as i32).min(size as i32 - 1);
    for y in miny..=maxy {
        for x in minx..=maxx {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let wa = ((b.1 - c.1) * (fx - c.0) + (c.0 - b.0) * (fy - c.1)) / den;
            let wb = ((c.1 - a.1) * (fx - c.0) + (a.0 - c.0) * (fy - c.1)) / den;
            let wc = 1.0 - wa - wb;
            if wa < 0.0 || wb < 0.0 || wc < 0.0 {
                continue;
            }
            let depth = wa * a.2 + wb * b.2 + wc * c.2;
            let idx = y as usize * size + x as usize;
            if depth > zbuf[idx] {
                zbuf[idx] = depth;
                let o = idx * 4;
                rgba[o] = color[0];
                rgba[o + 1] = color[1];
                rgba[o + 2] = color[2];
                rgba[o + 3] = 255;
            }
        }
    }
}
