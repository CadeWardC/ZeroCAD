//! Kernel-level geometry check for modeled threads: the helical thread tooth
//! (an isoceles V-profile swept along a helix) must build a valid, closable
//! solid whose radial extent matches the requested crest, for both external and
//! internal threads and both handedness. This exercises the helix + profile +
//! rotation-minimizing sweep pipeline independently of the feature/boolean layer.

use zerocad_core::mock_kernel::{helical_thread_solid, try_display_mesh_from_part, ThreadSpec};
use zerocad_core::Vec3;

/// Max distance of any mesh vertex from the axis (here +Y through the origin).
fn max_radius_about_y(vertices: &[f32]) -> f32 {
    vertices
        .chunks(6)
        .map(|v| (v[0] * v[0] + v[2] * v[2]).sqrt())
        .fold(0.0_f32, f32::max)
}

/// Min distance of any mesh vertex from the axis (here +Y through the origin).
fn min_radius_about_y(vertices: &[f32]) -> f32 {
    vertices
        .chunks(6)
        .map(|v| (v[0] * v[0] + v[2] * v[2]).sqrt())
        .fold(f32::INFINITY, f32::min)
}

#[test]
fn external_thread_tooth_builds_and_reaches_crest() {
    // An M6-ish external thread: pitch radius 2.8, depth 0.4 → crest at 3.0.
    let spec = ThreadSpec {
        mean_radius: 2.8,
        pitch: 1.0,
        length: 6.0,
        depth: 0.4,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: false,
        segments_per_turn: 32,
        starts: 1,
    };
    let solid = helical_thread_solid(Vec3::ZERO, Vec3::Y, &spec)
        .expect("helical thread tooth should build");
    let mesh = try_display_mesh_from_part(&solid).expect("thread tooth should tessellate");
    assert!(
        mesh.indices.len() >= 3,
        "thread tooth mesh has triangles: {} indices",
        mesh.indices.len()
    );
    let r = max_radius_about_y(&mesh.vertices);
    // Crest = mean + depth/2 = 3.0; allow faceting/frame slack.
    assert!(
        (r - 3.0).abs() < 0.25,
        "external crest reaches ~3.0mm, got {r}"
    );
}

#[test]
fn internal_thread_tooth_points_inward() {
    // A tapped-hole thread: pitch radius 2.8, depth 0.4 → crest at 2.6 (inward).
    let spec = ThreadSpec {
        mean_radius: 2.8,
        pitch: 1.0,
        length: 4.0,
        depth: 0.4,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: true,
        segments_per_turn: 32,
        starts: 1,
    };
    let solid = helical_thread_solid(Vec3::ZERO, Vec3::Y, &spec)
        .expect("internal thread tooth should build");
    let mesh = try_display_mesh_from_part(&solid).expect("internal tooth should tessellate");
    // The crest points inward: some vertex reaches ~mean − depth/2 = 2.6, well
    // inside the pitch radius (2.8). The outer root stays near 3.0.
    let rmin = min_radius_about_y(&mesh.vertices);
    let rmax = max_radius_about_y(&mesh.vertices);
    assert!(
        rmin <= 2.7,
        "internal crest points inward toward ~2.6mm, got {rmin}"
    );
    assert!(
        rmax < 3.2,
        "internal root stays near the pitch cylinder, got {rmax}"
    );
}

#[test]
fn left_handed_thread_also_builds() {
    let spec = ThreadSpec {
        mean_radius: 5.0,
        pitch: 2.0,
        length: 10.0,
        depth: 0.8,
        half_angle_deg: 30.0,
        right_handed: false,
        internal: false,
        segments_per_turn: 24,
        starts: 1,
    };
    assert!(
        helical_thread_solid(Vec3::ZERO, Vec3::Y, &spec).is_some(),
        "left-handed thread tooth should build"
    );
}

#[test]
fn threaded_cylinder_probe_timing() {
    use zerocad_core::mock_kernel::threaded_cylinder_solid;
    // Same shape as the feature-level test: r=4 h=5, pitch 1.5, depth 0.5.
    let spec = ThreadSpec {
        mean_radius: 4.0,
        pitch: 1.5,
        length: 5.0,
        depth: 0.5,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: false,
        segments_per_turn: 16,
        starts: 1,
    };
    let t0 = std::time::Instant::now();
    let solid = threaded_cylinder_solid(Vec3::ZERO, Vec3::Y, 4.0, 0.0, 5.0, &spec)
        .expect("threaded cylinder should build");
    let build = t0.elapsed();
    let t1 = std::time::Instant::now();
    let mesh = try_display_mesh_from_part(&solid).expect("threaded cylinder should tessellate");
    let tess = t1.elapsed();
    eprintln!(
        "threaded_cylinder: build={build:?} tessellate={tess:?} tris={}",
        mesh.indices.len() / 3
    );
    let rmax = max_radius_about_y(&mesh.vertices);
    let rmin_wall = mesh
        .vertices
        .chunks(6)
        .filter(|v| v[1] > 1.0 && v[1] < 4.0) // away from the runout/caps
        .map(|v| (v[0] * v[0] + v[2] * v[2]).sqrt())
        .fold(f32::INFINITY, f32::min);
    assert!(
        (rmax - 4.0).abs() < 0.1,
        "crest stays at nominal radius, got {rmax}"
    );
    assert!(
        rmin_wall < 3.6,
        "thread root cuts ~0.5 deep, got {rmin_wall}"
    );
}

#[test]
fn degenerate_thread_params_return_none() {
    let base = ThreadSpec {
        mean_radius: 3.0,
        pitch: 1.0,
        length: 5.0,
        depth: 0.4,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: false,
        segments_per_turn: 16,
        starts: 1,
    };
    assert!(
        helical_thread_solid(Vec3::ZERO, Vec3::Y, &ThreadSpec { pitch: 0.0, ..base }).is_none()
    );
    assert!(
        helical_thread_solid(Vec3::ZERO, Vec3::Y, &ThreadSpec { depth: 0.0, ..base }).is_none()
    );
    assert!(helical_thread_solid(
        Vec3::ZERO,
        Vec3::Y,
        &ThreadSpec {
            length: 0.0,
            ..base
        }
    )
    .is_none());
}
