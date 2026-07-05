//! Concave (inner-corner) fillet & chamfer: blending the vertical edge of a
//! rectangular pocket must ADD a rounded/beveled wedge of material in the
//! corner void — the mirror of the ordinary subtractive edge blend.

use openrcad_algo::boolean::point_in_solid;
use openrcad_algo::{boolean, chamfer_edges, fillet_edges, BooleanOp};
use openrcad_foundation::Pnt;
use openrcad_primitives::make_box;
use openrcad_topo::{Edge, Solid};

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
    let (bmin, bmax) = s
        .bounding_box()
        .corners()
        .expect("result must have bounds");
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
    assert!(body.is_watertight(), "flush pocket fixture must be watertight");

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
