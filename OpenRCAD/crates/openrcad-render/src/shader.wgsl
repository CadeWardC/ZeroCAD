// Flat-shaded surface shader for the OpenRCAD viewer.
//
// Normals are already constant across each triangle (computed per-face on the
// CPU), so straightforward per-fragment lighting yields a crisp faceted CAD
// look. Lighting is a simple hemisphere ambient plus a camera-relative
// directional term, kept deterministic and dependency-free.

struct Globals {
    view_proj: mat4x4<f32>,
    // Light direction in world space (xyz) + ambient strength (w).
    light: vec4<f32>,
    // Base surface color (rgb) + unused (a).
    color: vec4<f32>,
    // selected face id (x), enabled flag (y), highlight strength (z), unused (w).
    selection: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> globals: Globals;

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) face_id: f32,
};

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) face_id: f32,
) -> VsOut {
    var out: VsOut;
    out.clip_pos = globals.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.face_id = face_id;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let l = normalize(globals.light.xyz);
    // Two-sided so back faces of an open shell still read.
    let diffuse = max(abs(dot(n, l)), 0.0);
    let ambient = globals.light.w;
    let shade = clamp(ambient + (1.0 - ambient) * diffuse, 0.0, 1.0);
    var color = globals.color.rgb;
    if globals.selection.y > 0.5 && abs(in.face_id - globals.selection.x) < 0.5 {
        color = mix(color, vec3<f32>(1.0, 0.78, 0.28), globals.selection.z);
    }
    return vec4<f32>(color * shade, 1.0);
}

// Wireframe overlay: model edges drawn as dark line segments on top of the
// shaded surface (a small depth bias on the pipeline keeps them from z-fighting).
@vertex
fn vs_edge(@location(0) position: vec3<f32>) -> @builtin(position) vec4<f32> {
    return globals.view_proj * vec4<f32>(position, 1.0);
}

@fragment
fn fs_edge() -> @location(0) vec4<f32> {
    return vec4<f32>(0.03, 0.03, 0.05, 1.0);
}
