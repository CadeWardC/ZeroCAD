//! A minimal orbit camera with hand-rolled `f32` matrix math.
//!
//! No external linear-algebra crate is pulled in: the viewer only needs a
//! right-handed `look_at` and a `[0, 1]`-depth perspective (the wgpu/Direct3D
//! convention), so both are implemented directly. All matrices are stored
//! **column-major** as `[[f32; 4]; 4]` (`m[col][row]`), matching the memory
//! layout WGSL expects for a `mat4x4<f32>`, so `m * v` in the shader is correct
//! without any transpose.

/// A camera that orbits a target point on a sphere of a given radius.
#[derive(Clone, Copy, Debug)]
pub struct OrbitCamera {
    /// Point the camera looks at.
    pub target: [f32; 3],
    /// Distance from the target (orbit radius).
    pub distance: f32,
    /// Azimuth angle around the up axis, radians.
    pub yaw: f32,
    /// Elevation angle above the horizon, radians.
    pub pitch: f32,
    /// Vertical field of view, radians.
    pub fovy: f32,
    /// Near clip plane.
    pub znear: f32,
    /// Far clip plane.
    pub zfar: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: [0.0, 0.0, 0.0],
            distance: 4.0,
            yaw: 0.7,
            pitch: 0.5,
            fovy: 45.0_f32.to_radians(),
            znear: 0.05,
            zfar: 1000.0,
        }
    }
}

impl OrbitCamera {
    /// Frame the camera so a bounding box `[min, max]` fits comfortably in view.
    pub fn frame_bounds(&mut self, min: [f32; 3], max: [f32; 3]) {
        self.target = [
            0.5 * (min[0] + max[0]),
            0.5 * (min[1] + max[1]),
            0.5 * (min[2] + max[2]),
        ];
        let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
        let radius =
            0.5 * (extent[0] * extent[0] + extent[1] * extent[1] + extent[2] * extent[2]).sqrt();
        // Pull back far enough that the bounding sphere fits in the vertical FOV,
        // with a little margin.
        self.distance = (radius / (0.5 * self.fovy).sin()).max(radius * 2.0) * 1.2;
        self.znear = (self.distance * 0.01).max(1e-3);
        self.zfar = self.distance * 10.0 + radius * 10.0;
    }

    /// Rotate the camera around the target by screen-space pixel deltas.
    pub fn orbit_pixels(&mut self, dx: f32, dy: f32) {
        const SENSITIVITY: f32 = 0.01;
        const LIMIT: f32 = 1.54;
        self.yaw -= dx * SENSITIVITY;
        self.pitch = (self.pitch + dy * SENSITIVITY).clamp(-LIMIT, LIMIT);
    }

    /// Move the target parallel to the view plane by screen-space pixel deltas.
    pub fn pan_pixels(&mut self, dx: f32, dy: f32) {
        let (_forward, right, up) = self.basis();
        let scale = (self.distance * 0.0015).max(1e-4);
        self.target = add(self.target, scale_vec(right, -dx * scale));
        self.target = add(self.target, scale_vec(up, dy * scale));
    }

    /// Dolly toward or away from the target. Positive values zoom in.
    pub fn zoom_steps(&mut self, steps: f32) {
        let factor = (1.0 - steps * 0.12).clamp(0.15, 4.0);
        self.distance = (self.distance * factor).max(1e-4);
        self.znear = (self.distance * 0.01).max(1e-3);
        self.zfar = self.zfar.max(self.distance * 20.0);
    }

    /// Current eye position in world space.
    pub fn eye(&self) -> [f32; 3] {
        let cp = self.pitch.cos();
        let dir = [cp * self.yaw.cos(), self.pitch.sin(), cp * self.yaw.sin()];
        [
            self.target[0] + self.distance * dir[0],
            self.target[1] + self.distance * dir[1],
            self.target[2] + self.distance * dir[2],
        ]
    }

    /// A world-space picking ray `(origin, dir)` through a normalized device
    /// coordinate `(ndc_x, ndc_y)` in `[-1, 1]` (x right, y up).
    ///
    /// `dir` is normalized. Built directly from the camera basis and FOV so no
    /// matrix inversion is needed.
    pub fn ray(&self, ndc_x: f32, ndc_y: f32, aspect: f32) -> ([f32; 3], [f32; 3]) {
        let eye = self.eye();
        let (forward, right, up) = self.basis();
        let tan_half = (0.5 * self.fovy).tan();
        let sx = ndc_x * aspect * tan_half;
        let sy = ndc_y * tan_half;
        let dir = normalize([
            forward[0] + right[0] * sx + up[0] * sy,
            forward[1] + right[1] * sx + up[1] * sy,
            forward[2] + right[2] * sx + up[2] * sy,
        ]);
        (eye, dir)
    }

    /// Combined view-projection matrix, column-major, ready for upload.
    pub fn view_proj(&self, aspect: f32) -> [[f32; 4]; 4] {
        let view = look_at_rh(self.eye(), self.target, [0.0, 1.0, 0.0]);
        let proj = perspective_rh_zo(self.fovy, aspect.max(1e-4), self.znear, self.zfar);
        mul(proj, view)
    }

    fn basis(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        let eye = self.eye();
        let forward = normalize(sub(self.target, eye));
        let world_up = if forward[1].abs() > 0.98 {
            [0.0, 0.0, 1.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let right = normalize(cross(forward, world_up));
        let up = cross(right, forward);
        (forward, right, up)
    }
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale_vec(a: [f32; 3], s: f32) -> [f32; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(a: [f32; 3]) -> [f32; 3] {
    let len = dot(a, a).sqrt();
    if len > 0.0 {
        [a[0] / len, a[1] / len, a[2] / len]
    } else {
        a
    }
}

/// Right-handed `look_at`, column-major (glm `lookAtRH`).
fn look_at_rh(eye: [f32; 3], center: [f32; 3], up: [f32; 3]) -> [[f32; 4]; 4] {
    let f = normalize(sub(center, eye));
    let s = normalize(cross(f, up));
    let u = cross(s, f);
    [
        [s[0], u[0], -f[0], 0.0],
        [s[1], u[1], -f[1], 0.0],
        [s[2], u[2], -f[2], 0.0],
        [-dot(s, eye), -dot(u, eye), dot(f, eye), 1.0],
    ]
}

/// Right-handed perspective with a `[0, 1]` depth range (wgpu/Direct3D),
/// column-major (glm `perspectiveRH_ZO`).
fn perspective_rh_zo(fovy: f32, aspect: f32, znear: f32, zfar: f32) -> [[f32; 4]; 4] {
    let f = 1.0 / (0.5 * fovy).tan();
    [
        [f / aspect, 0.0, 0.0, 0.0],
        [0.0, f, 0.0, 0.0],
        [0.0, 0.0, zfar / (znear - zfar), -1.0],
        [0.0, 0.0, (znear * zfar) / (znear - zfar), 0.0],
    ]
}

/// Column-major 4×4 multiply: `a * b`.
fn mul(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for (c, out_col) in out.iter_mut().enumerate() {
        for (r, slot) in out_col.iter_mut().enumerate() {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[k][r] * b[c][k];
            }
            *slot = sum;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_multiply() {
        let id = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let m = perspective_rh_zo(1.0, 1.5, 0.1, 100.0);
        assert_eq!(mul(id, m), m);
        assert_eq!(mul(m, id), m);
    }

    #[test]
    fn eye_orbits_target_at_distance() {
        let cam = OrbitCamera {
            target: [1.0, 2.0, 3.0],
            distance: 5.0,
            ..Default::default()
        };
        let e = cam.eye();
        let d = sub(e, cam.target);
        assert!((dot(d, d).sqrt() - 5.0).abs() < 1e-4);
    }

    #[test]
    fn center_ray_points_at_target() {
        let cam = OrbitCamera::default();
        let (origin, dir) = cam.ray(0.0, 0.0, 1.5);
        assert_eq!(origin, cam.eye());
        // The central ray should aim from the eye toward the target.
        let to_target = normalize(sub(cam.target, cam.eye()));
        assert!((dir[0] - to_target[0]).abs() < 1e-5);
        assert!((dir[1] - to_target[1]).abs() < 1e-5);
        assert!((dir[2] - to_target[2]).abs() < 1e-5);
    }

    #[test]
    fn view_proj_is_finite() {
        let cam = OrbitCamera::default();
        let m = cam.view_proj(1.777);
        assert!(m.iter().flatten().all(|x| x.is_finite()));
    }

    #[test]
    fn orbit_and_zoom_remain_finite() {
        let mut cam = OrbitCamera::default();
        cam.orbit_pixels(120.0, -80.0);
        cam.zoom_steps(2.0);
        assert!(cam.eye().iter().all(|x| x.is_finite()));
        assert!(cam.distance > 0.0);
    }

    #[test]
    fn pan_moves_target() {
        let mut cam = OrbitCamera::default();
        let before = cam.target;
        cam.pan_pixels(40.0, 20.0);
        assert_ne!(cam.target, before);
    }
}
