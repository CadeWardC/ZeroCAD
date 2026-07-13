//! Concave (inner-corner) fillet & chamfer: blending the vertical edge of a
//! rectangular pocket must ADD a rounded/beveled wedge of material in the
//! corner void — the mirror of the ordinary subtractive edge blend.

use openrcad_algo::boolean::point_in_solid;
use openrcad_algo::{boolean, chamfer_edges, fillet_edges, BooleanOp};
use openrcad_foundation::Pnt;
use openrcad_geom::{GeomSurface, Surface};
use openrcad_mesh::tessellate;
use openrcad_primitives::make_box;
use openrcad_topo::{Edge, Solid};
use std::collections::HashMap;

/// 20×20×10 block with a 10×10 pocket sunk 6 deep from the top (floor z=4).
/// The pocket's inner corners are at (5,5), (15,5), (15,15), (5,15).
fn pocketed_block() -> Solid {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let tool = make_box(&Pnt::new(5.0, 5.0, 4.0), 10.0, 10.0, 7.0);
    let cut = boolean(&block, &tool, BooleanOp::Cut);
    assert!(cut.is_watertight(), "pocket fixture must be watertight");
    cut
}

/// The pocket's inner vertical edge at (5,5), z 4..10.
fn pocket_corner_edge() -> Edge {
    Edge::between_points(Pnt::new(5.0, 5.0, 4.0), Pnt::new(5.0, 5.0, 10.0))
}

/// One edge of the pocket opening. On the block's top face this edge belongs to
/// an inner wire (the pocket hole), not the face's outer box boundary.
fn pocket_top_rim_edge() -> Edge {
    Edge::between_points(Pnt::new(5.0, 5.0, 10.0), Pnt::new(15.0, 5.0, 10.0))
}

type MeshKey = (i64, i64, i64);

fn mesh_edge_health(solid: &Solid) -> (usize, usize) {
    let mesh = tessellate(solid, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> MeshKey {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1.0e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    let mut edges: HashMap<(MeshKey, MeshKey), u32> = HashMap::new();
    for triangle in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(triangle[a] as usize), q(triangle[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&count| count == 1).count(),
        edges.values().filter(|&&count| count > 2).count(),
    )
}

fn sequential_pocket_miter(vertical_first: bool, radius: f64) -> Solid {
    let body = pocketed_block();
    let (first, second) = if vertical_first {
        (pocket_corner_edge(), pocket_top_rim_edge())
    } else {
        (pocket_top_rim_edge(), pocket_corner_edge())
    };
    let once = fillet_edges(&body, &[first], radius).expect("first concave fillet");
    fillet_edges(&once, &[second], radius).expect("adjacent concave fillet must miter")
}

fn assert_concave_miter(label: &str, solid: &Solid) {
    assert!(solid.is_watertight(), "{label} must be watertight");
    assert!(
        solid.health_report().is_healthy(),
        "{label} must be topologically healthy"
    );
    let miter_patches = solid
        .shell()
        .faces()
        .iter()
        .filter(|face| matches!(face.surface(), Some(GeomSurface::Ruled(_))))
        .count();
    assert_eq!(
        miter_patches, 1,
        "{label} must replace the flat junction with one concave ruled miter"
    );
    let (cracks, nonmanifold) = mesh_edge_health(solid);
    assert_eq!(cracks, 0, "{label} display mesh must be crack-free");
    assert_eq!(
        nonmanifold, 0,
        "{label} display mesh must not contain coincident membranes"
    );
    assert_eq!(
        solid
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Sphere(_))))
            .count(),
        0,
        "{label} is a two-edge miter, not a spherical three-edge corner"
    );

    let (lo, hi) = solid
        .bounding_box()
        .corners()
        .expect("mitered pocket must have bounds");
    assert!(
        lo.x() >= -1.0e-6
            && lo.y() >= -1.0e-6
            && lo.z() >= -1.0e-6
            && hi.x() <= 20.0 + 1.0e-6
            && hi.y() <= 20.0 + 1.0e-6
            && hi.z() <= 10.0 + 1.0e-6,
        "{label} must add material only into the pocket, never outside the block"
    );
    assert!(
        point_in_solid(&Pnt::new(5.2, 5.4, 8.2), solid),
        "{label} must fill the inward corner next to the miter"
    );
    assert!(
        !point_in_solid(&Pnt::new(8.0, 8.0, 8.0), solid),
        "{label} must leave the interior of the pocket empty"
    );
}

#[test]
fn equal_radius_concave_fillets_miter_in_both_orders() {
    let vertical_then_top = sequential_pocket_miter(true, 2.0);
    let top_then_vertical = sequential_pocket_miter(false, 2.0);
    assert_concave_miter("vertical->top", &vertical_then_top);
    assert_concave_miter("top->vertical", &top_then_vertical);

    let signature = |solid: &Solid| {
        let cylinders = solid
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        (
            solid.vertex_count(),
            solid.edge_count(),
            solid.face_count(),
            cylinders,
        )
    };
    assert_eq!(
        signature(&vertical_then_top),
        signature(&top_then_vertical),
        "concave miter topology must not depend on edge application order"
    );

    let ruled = |solid: &Solid| {
        solid
            .shell()
            .faces()
            .iter()
            .find_map(|face| match face.surface() {
                Some(GeomSurface::Ruled(surface)) => Some(surface.clone()),
                _ => None,
            })
    };
    let forward_seam = ruled(&vertical_then_top).expect("forward miter surface");
    let reverse_seam = ruled(&top_then_vertical).expect("reverse miter surface");
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert!(
                forward_seam
                    .point(u, v)
                    .distance(&reverse_seam.point(u, 1.0 - v))
                    <= 1.0e-8,
                "miter surface geometry must not depend on application order"
            );
        }
    }
    for sample in [
        Pnt::new(5.2, 5.2, 9.0),
        Pnt::new(5.8, 5.4, 9.4),
        Pnt::new(6.4, 5.8, 8.6),
    ] {
        assert_eq!(
            point_in_solid(&sample, &vertical_then_top),
            point_in_solid(&sample, &top_then_vertical),
            "concave miter material differs by application order at {sample:?}"
        );
    }
}

#[test]
fn pocket_top_inner_wire_edge_fillet_succeeds() {
    let body = pocketed_block();
    let result = fillet_edges(&body, &[pocket_top_rim_edge()], 1.5)
        .expect("filleting a pocket rim stored in the top face's inner wire must succeed");
    assert!(result.is_watertight());
    assert!(result.health_report().is_healthy());
}

#[test]
fn pocket_top_inner_wire_edge_chamfer_succeeds() {
    let body = pocketed_block();
    let result = chamfer_edges(&body, &[pocket_top_rim_edge()], 1.5)
        .expect("chamfering a pocket rim stored in the top face's inner wire must succeed");
    assert!(result.is_watertight());
    assert!(result.health_report().is_healthy());
}

#[test]
fn concave_fillet_adds_rounded_corner_material() {
    let body = pocketed_block();
    let edge = pocket_corner_edge();

    // Sanity: the corner void is empty before the blend.
    let near_corner = Pnt::new(5.5, 5.5, 7.0);
    assert!(!point_in_solid(&near_corner, &body));

    let r = fillet_edges(&body, std::slice::from_ref(&edge), 2.0);
    let s = r.expect("concave pocket-edge fillet should succeed");
    assert!(s.is_watertight(), "concave fillet must stay watertight");
    assert!(
        s.health_report().is_healthy(),
        "concave fillet must stay healthy"
    );

    // Ball center line at (7,7): the wedge between the corner and the r=2
    // band (dist to (7,7) > 2 within the corner square) is now material.
    assert!(
        point_in_solid(&near_corner, &s),
        "corner wedge must be filled ({near_corner:?})"
    );
    assert!(
        point_in_solid(&Pnt::new(5.2, 5.2, 4.5), &s),
        "wedge must be filled down to the pocket floor"
    );
    // On/inside the band cylinder stays void (the rolling ball itself).
    assert!(
        !point_in_solid(&Pnt::new(7.0, 7.0, 7.0), &s),
        "ball-center line must stay void"
    );
    assert!(
        !point_in_solid(&Pnt::new(6.0, 6.0, 7.0), &s),
        "inside the band tube must stay void (dist to axis ~1.41 < 2)"
    );
    // Rest of the pocket is untouched.
    assert!(!point_in_solid(&Pnt::new(10.0, 10.0, 7.0), &s));
    // And the fillet added nothing outside the original bounds.
    let (lo, hi) = (Pnt::new(-0.1, -0.1, -0.1), Pnt::new(20.1, 20.1, 10.1));
    let (bmin, bmax) = s.bounding_box().corners().expect("result must have bounds");
    assert!(
        bmin.x() >= lo.x()
            && bmin.y() >= lo.y()
            && bmin.z() >= lo.z()
            && bmax.x() <= hi.x()
            && bmax.y() <= hi.y()
            && bmax.z() <= hi.z(),
        "result must stay inside the input bounds"
    );
}

#[test]
fn concave_chamfer_adds_beveled_corner_material() {
    let body = pocketed_block();
    let edge = pocket_corner_edge();

    let near_corner = Pnt::new(5.4, 5.4, 7.0);
    assert!(!point_in_solid(&near_corner, &body));

    let r = chamfer_edges(&body, std::slice::from_ref(&edge), 1.5);
    let s = r.expect("concave pocket-edge chamfer should succeed");
    assert!(s.is_watertight(), "concave chamfer must stay watertight");
    assert!(
        s.health_report().is_healthy(),
        "concave chamfer must stay healthy"
    );

    // Bevel plane runs from (5, 6.5) to (6.5, 5): x + y = 11.5. The material
    // side is toward the corner.
    assert!(
        point_in_solid(&near_corner, &s),
        "bevel wedge must be filled (x+y=10.8 < 11.5)"
    );
    assert!(
        !point_in_solid(&Pnt::new(6.2, 6.2, 7.0), &s),
        "beyond the bevel plane must stay void (x+y=12.4 > 11.5)"
    );
    assert!(!point_in_solid(&Pnt::new(10.0, 10.0, 7.0), &s));
}

/// Same pocket but cut with a FLUSH-topped tool (exactly z 4..10, no
/// overshoot) — the shape ZeroCAD's auto-directed cut produces. The rim
/// topology differs from the overshoot cut and must still blend.
#[test]
fn concave_fillet_on_flush_cut_pocket() {
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let tool = make_box(&Pnt::new(5.0, 5.0, 4.0), 10.0, 10.0, 6.0);
    let body = boolean(&block, &tool, BooleanOp::Cut);
    assert!(
        body.is_watertight(),
        "flush pocket fixture must be watertight"
    );

    let edge = pocket_corner_edge();
    let r = fillet_edges(&body, std::slice::from_ref(&edge), 2.0);
    let s = r.expect("flush-cut concave fillet should succeed");
    assert!(s.is_watertight() && s.health_report().is_healthy());
    assert!(point_in_solid(&Pnt::new(5.5, 5.5, 7.0), &s));
}

/// A convex edge on the same body must still take the subtractive path — the
/// concavity probe must not misclassify ordinary edges.
#[test]
fn convex_edge_on_pocketed_block_still_subtracts() {
    let body = pocketed_block();
    // Outer top edge along X at y=0, z=10.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 10.0), Pnt::new(20.0, 0.0, 10.0));
    let s = fillet_edges(&body, std::slice::from_ref(&edge), 2.0)
        .expect("convex fillet should still succeed");
    assert!(s.is_watertight() && s.health_report().is_healthy());
    // Material removed at the rim...
    assert!(!point_in_solid(&Pnt::new(10.0, 0.05, 9.95), &s));
    // ...and nothing added in the pocket corner.
    assert!(!point_in_solid(&Pnt::new(5.5, 5.5, 7.0), &s));
}
