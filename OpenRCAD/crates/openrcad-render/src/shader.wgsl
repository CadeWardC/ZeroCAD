// Flat-shaded surface shader for the OpenRCAD viewer.
//
// Normals are already constant across each triangle (computed per-face on the
// CPU), so straightforward per-fragment lighting yields a crisp faceted CAD
// look. Lighting is a simple hemisphere ambient plus a camera-relative
// directional term, kept deterministic and dependency-free.
//
// Per-face highlighting (hover / selection) is read from a state texture:
// texel `i` (row-major over the texture's width) holds the highlight state of
// face id `i` (0 none, 1 hover, 2 selected). The texture wraps to multiple rows
// for large face counts, so no dimension exceeds backend limits. This supports
// arbitrary multi-selection without a second pass.

struct Globals {
    view_proj: mat4x4<f32>,
    // Light direction in world space (xyz) + ambient strength (w).
    light: vec4<f32>,
    // Base surface color (rgb) + coverage alpha (a). Opaque pipelines write
    // a = 1; translucent layer pipelines blend by this alpha.
    color: vec4<f32>,
    // x: face-state tint enable (the tinted scene's face ids index the
    // highlight texture; others disable it). y, z: viewport size in pixels.
    // w: wireframe line width in pixels.
    params: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> globals: Globals;

// One texel per face id (row-major): its highlight state
// (0 = none, 1 = hover, 2 = selected).
@group(0) @binding(1)
var face_states: texture_2d<f32>;

const HOVER_COLOR: vec3<f32> = vec3<f32>(0.35, 0.62, 1.0);
const SELECT_COLOR: vec3<f32> = vec3<f32>(1.0, 0.78, 0.28);

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
    // Face id → row-major texel. The state texture always covers every face id
    // the uploaded base mesh uses (the host sizes it from the same mesh), so
    // the lookup stays in bounds. Auxiliary layers disable the tint.
    if globals.params.x > 0.5 {
        let sid = i32(round(in.face_id));
        let row = i32(textureDimensions(face_states).x);
        let state = textureLoad(face_states, vec2<i32>(sid % row, sid / row), 0).r;
        if state > 1.5 {
            color = mix(color, SELECT_COLOR, 0.6);
        } else if state > 0.5 {
            color = mix(color, HOVER_COLOR, 0.45);
        }
    }
    return vec4<f32>(color * shade, globals.color.a);
}

// Wireframe overlay: each edge segment is one instance, expanded here into a
// screen-aligned quad of `params.w` pixels width plus a 1px anti-aliasing
// feather on each side (hardware lines are fixed 1px and alias). A small depth
// bias on the pipeline keeps the quads from z-fighting the shaded surface.

struct EdgeOut {
    @builtin(position) clip_pos: vec4<f32>,
    // Signed distance from the segment centreline, in pixels.
    @location(0) across_px: f32,
};

@vertex
fn vs_edge(
    @builtin(vertex_index) vi: u32,
    @location(0) a: vec3<f32>,
    @location(1) b: vec3<f32>,
) -> EdgeOut {
    var out: EdgeOut;
    let ca = globals.view_proj * vec4<f32>(a, 1.0);
    let cb = globals.view_proj * vec4<f32>(b, 1.0);
    // An endpoint at/behind the projection plane can't be expanded in screen
    // space — collapse the quad to a clipped point.
    if ca.w <= 0.0 || cb.w <= 0.0 {
        out.clip_pos = vec4<f32>(0.0, 0.0, 2.0, 1.0);
        out.across_px = 0.0;
        return out;
    }

    // Two triangles over (t, side): t runs along the segment, side across it.
    var T = array<f32, 6>(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
    var S = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let t = T[vi];
    let s = S[vi];

    let half_px = 0.5 * vec2<f32>(globals.params.y, globals.params.z);
    // NDC → pixel coordinates (y flip is irrelevant for a symmetric expansion).
    let sa = ca.xy / ca.w * half_px;
    let sb = cb.xy / cb.w * half_px;
    var dir = sb - sa;
    let len = length(dir);
    if len < 1.0e-4 {
        dir = vec2<f32>(1.0, 0.0);
    } else {
        dir = dir / len;
    }
    let n = vec2<f32>(-dir.y, dir.x);
    let ext = 0.5 * globals.params.w + 1.0; // half line width + AA feather
    let sp = mix(sa, sb, t) + n * s * ext;

    // Back to clip space with the endpoint-interpolated depth/w so the quad
    // sits at the segment's own depth.
    let w = mix(ca.w, cb.w, t);
    let z = mix(ca.z, cb.z, t);
    out.clip_pos = vec4<f32>(sp / half_px * w, z, w);
    out.across_px = s * ext;
    return out;
}

@fragment
fn fs_edge(in: EdgeOut) -> @location(0) vec4<f32> {
    // Anti-aliased coverage: full inside the line body, fading over the 1px
    // feather; multiplied by the owning geometry's alpha so a translucent
    // ghost's wireframe fades with its fill.
    let half_w = 0.5 * globals.params.w;
    let cov = clamp(half_w + 0.5 - abs(in.across_px), 0.0, 1.0);
    let alpha = cov * clamp(globals.color.a, 0.0, 1.0);
    return vec4<f32>(0.03, 0.03, 0.05, alpha);
}

// Pick pass: face ids rendered into an R32Uint target (0 = background,
// id + 1 otherwise) at one sample, read back by the host for exact
// pixel-under-cursor face picking and hover.
@fragment
fn fs_pick(in: VsOut) -> @location(0) u32 {
    return u32(round(in.face_id)) + 1u;
}
