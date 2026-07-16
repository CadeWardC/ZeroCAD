//! Repro: chamfer/fillet of a cylinder extruded from a circle sketched on the
//! GROUND plane (CoordinateSystem::XZ, the left-handed one), off-origin.
//! GUI reports:
//!   - "native result rejected: candidate expands outside the original part bounds"
//!   - "candidate removed unselected circular-bite side span [...]"
//! and the preview band flares OUTWARD past the wall.

use openrcad::topo::Orientation;
use std::collections::HashSet;
use std::f32::consts::TAU;
use zerocad_core::mock_kernel::EdgeCurveHint;
use zerocad_core::{
    CoordinateSystem, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves,
};

const CX: f32 = 2.2;
const CZ: f32 = 9.085;
const R: f32 = 5.0;
const H: f32 = 14.56;

fn ground_cylinder_edge_mod(
    kind: zerocad_core::CornerKind,
    axis: [f32; 3],
    label: &str,
) -> (Vec<(String, zerocad_core::MockMesh)>, Vec<String>, usize) {
    let mut curves = SketchCurves::new();
    curves.add_circle((CX, CZ), R);
    let regions = zerocad_core::detect_regions(&curves);
    let region = regions
        .iter()
        .position(|r| r.contains((CX, CZ)))
        .expect("circle region");

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "s".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::XZ,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "e".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: H,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("s", "e");
    let unmodified = g.evaluate_bodies(&HashSet::new()).unwrap();
    let base_tris = unmodified[0].1.indices.len();

    // GUI-shaped closed rim hint on the TOP rim.
    let rim = EdgeRef {
        p0: [CX + R, H, CZ],
        p1: [CX - R, H, CZ],
        n1: [0.0, 1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [CX, H, CZ],
            axis,
            x_dir: [1.0, 0.0, 0.0],
            radius: R,
            start: 0.0,
            end: TAU,
            closed: true,
        }),
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: "em".into(),
        name: "Edge Mod".into(),
        feature: FeatureType::EdgeMod {
            target: "e".into(),
            edge: rim,
            dist: 3.0,
            dist_expr: None,
            kind,
        },
    });
    g.add_dependency("e", "em");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    println!("{label}: warnings = {warnings:?}");
    (bodies, warnings, base_tris)
}

fn assert_committed(
    label: &str,
    bodies: &[(String, zerocad_core::MockMesh)],
    warnings: &[String],
    base_tris: usize,
) {
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: edge-mod should commit, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "{label}: keeps one body");
    assert_ne!(
        bodies[0].1.indices.len(),
        base_tris,
        "{label}: the edge-mod must change the display mesh"
    );
}

/// Kernel-level probe: build the same ground-plane extruded circle and chamfer
/// it directly, printing the before/after AABBs so we can see whether the cone
/// band flares outward (the acceptance gate's complaint).
#[test]
fn ground_cylinder_kernel_chamfer_aabb_probe() {
    use zerocad_core::mock_kernel::{chamfer_edge_with_hint, solid_aabb};

    // Same boundary polyline a drawn circle produces.
    let mut curves = SketchCurves::new();
    curves.add_circle((CX, CZ), R);
    let regions = zerocad_core::detect_regions(&curves);
    let region = regions
        .iter()
        .find(|r| r.contains((CX, CZ)))
        .expect("circle region");
    let solid = zerocad_core::mock_kernel::extruded_region_solid(
        &region.boundary,
        &[],
        H,
        &CoordinateSystem::XZ,
    )
    .expect("extruded solid");
    let before = solid_aabb(&solid).unwrap();
    println!("before AABB: {:?}", before);
    for face in solid.shell().faces() {
        println!(
            "face surface: {:?} orientation {:?}",
            face.surface().map(|s| match s {
                openrcad::geom::GeomSurface::Plane(_) => "Plane",
                openrcad::geom::GeomSurface::Cylinder(_) => "Cylinder",
                _ => "Other",
            }),
            face.orientation()
        );
    }

    let hint = EdgeCurveHint::Circle {
        center: [CX, H, CZ],
        axis: [0.0, 1.0, 0.0],
        x_dir: [1.0, 0.0, 0.0],
        radius: R,
        start: 0.0,
        end: TAU,
        closed: true,
    };
    let out = chamfer_edge_with_hint(&solid, [CX + R, H, CZ], [CX - R, H, CZ], Some(&hint), 3.0)
        .expect("kernel chamfer should succeed");
    let after = solid_aabb(&out).unwrap();
    println!("after AABB:  {:?}", after);
    let grew = (0..3).any(|k| after.0[k] < before.0[k] - 0.05 || after.1[k] > before.1[k] + 0.05);
    assert!(
        !grew,
        "chamfer band flares OUTWARD: before {before:?} after {after:?}"
    );
}

/// Same probe but with the solid the graph actually builds for a whole-circle
/// profile: `circular_cylinder_tool` (native make_cylinder along cs.n).
#[test]
fn ground_cylinder_native_tool_chamfer_aabb_probe() {
    use zerocad_core::mock_kernel::{chamfer_edge_with_hint, circular_cylinder_tool, solid_aabb};

    let mut curves = SketchCurves::new();
    curves.add_circle((CX, CZ), R);
    let regions = zerocad_core::detect_regions(&curves);
    let region = regions
        .iter()
        .find(|r| r.contains((CX, CZ)))
        .expect("circle region");
    let solid = circular_cylinder_tool(&region.boundary, &region.holes, H, &CoordinateSystem::XZ)
        .expect("native cylinder tool");
    let before = solid_aabb(&solid).unwrap();
    println!("before AABB: {:?}", before);

    let hint = EdgeCurveHint::Circle {
        center: [CX, H, CZ],
        axis: [0.0, 1.0, 0.0],
        x_dir: [1.0, 0.0, 0.0],
        radius: R,
        start: 0.0,
        end: TAU,
        closed: true,
    };
    let out = chamfer_edge_with_hint(&solid, [CX + R, H, CZ], [CX - R, H, CZ], Some(&hint), 3.0)
        .expect("kernel chamfer should succeed");
    let after = solid_aabb(&out).unwrap();
    println!("after AABB:  {:?}", after);
    let grew = (0..3).any(|k| after.0[k] < before.0[k] - 0.05 || after.1[k] > before.1[k] + 0.05);

    // Mirror edge_mod_keeps_body's second condition: enclosed-volume ratio from
    // a coarse tessellation (divergence theorem).
    let vol = |s: &zerocad_core::mock_kernel::KernelSolid| -> f64 {
        let mesh = openrcad::mesh::tessellate(s, 0.5, std::f64::consts::PI);
        let mut vol6 = 0.0f64;
        for tri in &mesh.triangles {
            let a = mesh.vertices[tri[0] as usize];
            let b = mesh.vertices[tri[1] as usize];
            let c = mesh.vertices[tri[2] as usize];
            vol6 += a.x() * (b.y() * c.z() - b.z() * c.y())
                + a.y() * (b.z() * c.x() - b.x() * c.z())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
        }
        (vol6 / 6.0).abs()
    };
    let pv = vol(&solid);
    let rv = vol(&out);
    println!("volume before {pv:.2} after {rv:.2} ratio {:.3}", rv / pv);

    // Per-face signed volume contribution at the coarse volume-gate tolerance:
    // a consistently-outward-wound closed mesh sums to the true volume.
    let coarse = openrcad::mesh::tessellate(&out, 0.5, std::f64::consts::PI);
    let mut per_face = std::collections::HashMap::<u32, (usize, f64, f64)>::new();
    for (t, tri) in coarse.triangles.iter().enumerate() {
        let a = coarse.vertices[tri[0] as usize];
        let b = coarse.vertices[tri[1] as usize];
        let c = coarse.vertices[tri[2] as usize];
        let v6 = a.x() * (b.y() * c.z() - b.z() * c.y())
            + a.y() * (b.z() * c.x() - b.x() * c.z())
            + a.z() * (b.x() * c.y() - b.y() * c.x());
        let ab = (b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
        let ac = (c.x() - a.x(), c.y() - a.y(), c.z() - a.z());
        let cx = ab.1 * ac.2 - ab.2 * ac.1;
        let cy = ab.2 * ac.0 - ab.0 * ac.2;
        let cz = ab.0 * ac.1 - ab.1 * ac.0;
        let area = 0.5 * (cx * cx + cy * cy + cz * cz).sqrt();
        let e = per_face.entry(coarse.face_ids[t]).or_default();
        e.0 += 1;
        e.1 += v6 / 6.0;
        e.2 += area;
    }
    for (i, face) in out.shell().faces().iter().enumerate() {
        let kind = face.surface().map(|s| match s {
            openrcad::geom::GeomSurface::Plane(_) => "Plane",
            openrcad::geom::GeomSurface::Cylinder(_) => "Cylinder",
            openrcad::geom::GeomSurface::Cone(_) => "Cone",
            openrcad::geom::GeomSurface::Torus(_) => "Torus",
            _ => "Other",
        });
        let (tris, sv, area) = per_face.get(&(i as u32)).copied().unwrap_or((0, 0.0, 0.0));
        // Effective B-Rep normal (natural du×dv, sign-flipped when Reversed) at
        // a boundary-edge midpoint, dotted with the known outward direction for
        // a top-rim chamfer band (radial+axial, 45° up-and-out). Says whether
        // the stored orientation flag is correct independent of the mesh.
        let mut brep_dot = f64::NAN;
        if let Some(surf @ openrcad::geom::GeomSurface::Cone(_)) = face.surface() {
            // Sample the surface normal field anywhere (a cone's radial sense is
            // uniform) and compare the ORIENTED normal with the true outward
            // direction (radial from the rim axis + up, the 45° chamfer normal).
            let (p, du, dv) = surf.d1(0.3, 1.0);
            let mut n = du.cross(&dv);
            if face.orientation() == Orientation::Reversed {
                n = -n;
            }
            let rad = openrcad::foundation::Vec::new(p.x() - CX as f64, 0.0, p.z() - CZ as f64);
            if let (Some(r), Some(nn)) = (rad.normalized(), n.normalized()) {
                let up = openrcad::foundation::Vec::new(0.0, 1.0, 0.0);
                let outward = openrcad::foundation::Vec::from_dir(r) + up;
                brep_dot = openrcad::foundation::Vec::from_dir(nn).dot(&outward);
            }
        }
        println!(
            "face {i}: {:?} orient {:?} tris {tris} signed_vol {sv:.2} area {area:.2} brep_out_dot {brep_dot:.3}",
            kind,
            face.orientation(),
        );
    }
    assert!(
        !grew,
        "chamfer band flares OUTWARD: before {before:?} after {after:?}"
    );
    assert!(
        rv >= pv * 0.5,
        "volume gate would reject: before {pv:.2} after {rv:.2}"
    );
}

#[test]
fn ground_cylinder_rim_chamfer_commits_axis_up() {
    let (bodies, warnings, base_tris) = ground_cylinder_edge_mod(
        zerocad_core::CornerKind::Chamfer,
        [0.0, 1.0, 0.0],
        "chamfer axis +Y",
    );
    assert_committed("ground chamfer +Y", &bodies, &warnings, base_tris);
}

#[test]
fn ground_cylinder_rim_chamfer_commits_axis_down() {
    // The GUI's circle fit can hand back either axis direction.
    let (bodies, warnings, base_tris) = ground_cylinder_edge_mod(
        zerocad_core::CornerKind::Chamfer,
        [0.0, -1.0, 0.0],
        "chamfer axis -Y",
    );
    assert_committed("ground chamfer -Y", &bodies, &warnings, base_tris);
}

#[test]
fn ground_cylinder_rim_fillet_commits_axis_up() {
    let (bodies, warnings, base_tris) = ground_cylinder_edge_mod(
        zerocad_core::CornerKind::Fillet,
        [0.0, 1.0, 0.0],
        "fillet axis +Y",
    );
    assert_committed("ground fillet +Y", &bodies, &warnings, base_tris);
}
