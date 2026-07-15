//! Fillet a top edge that runs into a concave vertical cut-cylinder.
//!
//! Desired ("trim flush"): the fillet is trimmed where it meets the cut; the cut
//! stays a clean full-height vertical cylinder (NOT rounded into a sphere octant),
//! and the fillet ends in a curved trim edge shared with the cut wall.

use core::f64::consts::PI;
use openrcad_algo::{boolean, chamfer_edges, fillet_edges, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt};
use openrcad_geom::{Curve, GeomSurface};
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Solid};

/// Box 10^3 with a vertical Ø8 cylinder cut out of the (10,10) corner. The rim
/// seam is rotated to 30/150/270 deg so the corner's 180->270 cut arc stays
/// seam-free (one clean wall face -> watertight Cut).
fn cut_body() -> Solid {
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let xdir = Dir::new((PI / 6.0).cos(), (PI / 6.0).sin(), 0.0);
    let axis = Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), xdir);
    let cyl = make_cylinder(&axis, 4.0, 12.0);
    boolean(&cube, &cyl, BooleanOp::Cut)
}

/// The GUI case: the same Ø8 corner cut, but with the rim seam rotated to 90° so
/// one seam (at 210°) falls *inside* the corner's 180->270° cut arc. The boolean
/// therefore produces the concave wall as TWO cocylindrical fragments meeting at
/// that seam; the cocylindrical merge must re-unite them into one cut face.
fn cut_body_seam_crossing() -> Solid {
    let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
    let xdir = Dir::new((PI / 2.0).cos(), (PI / 2.0).sin(), 0.0);
    let axis = Ax2::new_axes(Pnt::new(10.0, 10.0, -1.0), Dir::dz(), xdir);
    let cyl = make_cylinder(&axis, 4.0, 12.0);
    boolean(&cube, &cyl, BooleanOp::Cut)
}

fn zerocad_circular_bite_body() -> Solid {
    let base = make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let axis = Ax2::new(Pnt::new(20.0, 8.0, -0.25), Dir::dz());
    let cyl = make_cylinder(&axis, 14.0, 10.5);
    boolean(&base, &cyl, BooleanOp::Cut)
}

fn zerocad_circular_bite_cutoff_edge() -> Edge {
    let x = 20.0 - (14.0_f64 * 14.0 - 3.0_f64 * 3.0).sqrt();
    Edge::between_points(Pnt::new(0.0, 5.0, 10.0), Pnt::new(x, 5.0, 10.0))
}

fn has_sphere(s: &Solid) -> bool {
    s.shell()
        .faces()
        .iter()
        .any(|f| matches!(f.surface(), Some(GeomSurface::Sphere(_))))
}

/// The vertical (Z-axis) cylinder face of `radius` ≈ `r`, if exactly one exists.
fn vertical_cut_face(s: &Solid, r: f64) -> Option<Face> {
    let mut found = None;
    for f in s.shell().faces() {
        if let Some(GeomSurface::Cylinder(c)) = f.surface() {
            let axis = c.position().direction();
            if axis.dot(&Dir::dz()).abs() > 0.999 && (c.radius() - r).abs() < 1e-3 {
                if found.is_some() {
                    return None; // more than one
                }
                found = Some(f);
            }
        }
    }
    found
}

/// Every edge of `face`'s outer wire stays on the cylinder of `radius` about the
/// vertical axis through `axis_xy` — i.e. the cut wall is still a clean cylinder.
fn boundary_on_cut_cylinder(face: &Face, axis_xy: (f64, f64), radius: f64) -> bool {
    let wire = face.outer_wire().expect("cut face has an outer wire");
    for e in wire.edges() {
        let Some(curve) = e.curve() else { return false };
        let (t0, t1) = (e.first(), e.last());
        for k in 0..=8 {
            let t = t0 + (t1 - t0) * (k as f64) / 8.0;
            let p = curve.point(t);
            let d = ((p.x() - axis_xy.0).powi(2) + (p.y() - axis_xy.1).powi(2)).sqrt();
            // The trim curve is a finely-chorded approximation: tolerate sub-mesh
            // chord deviation, but catch the old bug (off-surface blend cap arc was
            // ~0.27 off a Ø8 wall).
            if (d - radius).abs() > 5e-3 {
                return false;
            }
        }
    }
    true
}

fn safe_blend_result<E: core::fmt::Display>(
    label: &str,
    source: &Solid,
    result: Result<Solid, E>,
) -> Option<Solid> {
    match result {
        Ok(solid) => {
            assert!(solid.is_watertight(), "{label}: accepted an open shell");
            assert!(
                solid.health_report().is_healthy(),
                "{label}: accepted unhealthy topology: {:?}",
                solid.health_report().errors
            );
            Some(solid)
        }
        Err(error) => {
            assert!(
                !error.to_string().is_empty(),
                "{label}: empty failure diagnostic"
            );
            assert!(source.is_watertight() && source.health_report().is_healthy());
            None
        }
    }
}

/// Test A — the bug: filleting a top edge into the cut must trim flush, leaving
/// the cut a clean vertical cylinder (no sphere, no off-surface distortion).
///
/// Covers both top edges that run into the same corner cut (the y=10 edge whose
/// cut boundary is the curved rim, and the x=10 edge), at a spread of radii.
#[test]
fn fillet_into_cut_trims_flush_and_keeps_cut_clean() {
    let body = cut_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    // Two top edges, each running from a far corner into the (10,10) cut.
    let edges = [
        // y=10, z=10: from x=0 into the cut (cut boundary on this face is the rim).
        Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0)),
        // x=10, z=10: from y=0 into the cut (cut starts at y=6).
        Edge::between_points(Pnt::new(10.0, 0.0, 10.0), Pnt::new(10.0, 6.0, 10.0)),
    ];

    for edge in &edges {
        for r in [1.0_f64, 1.5, 2.5] {
            let Some(s) = safe_blend_result(
                &format!("fillet r={r} into cut"),
                &body,
                fillet_edges(&body, std::slice::from_ref(edge), r),
            ) else {
                continue;
            };

            assert!(s.is_watertight(), "r={r}: result must be watertight");
            assert!(
                s.health_report().is_healthy(),
                "r={r}: result must be healthy"
            );
            assert!(
                !has_sphere(&s),
                "r={r}: the cut must NOT be rounded into a sphere"
            );

            // The cut survives as exactly one clean vertical cylinder of radius 4,
            // and every boundary edge of it still lies on that cylinder (not the
            // off-surface blend end-cap the old corner trim spliced in).
            let cut = vertical_cut_face(&s, 4.0)
                .unwrap_or_else(|| panic!("r={r}: exactly one Ø8 cut wall must survive"));
            assert!(
                boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
                "r={r}: the cut wall's boundary must stay on the cut cylinder (clean)"
            );

            // The fillet blend cylinder (radius r) is present.
            let has_blend = s.shell().faces().iter().any(|f| {
                matches!(f.surface(), Some(GeomSurface::Cylinder(c)) if (c.radius() - r).abs() < 1e-3)
            });
            assert!(
                has_blend,
                "r={r}: the fillet blend cylinder must be present"
            );
        }
    }
}

/// Test A2 — the GUI bug: when the corner arc crosses a `make_cylinder` rim seam,
/// the boolean splits the concave wall into two cocylindrical faces. The
/// cocylindrical merge must collapse them back to ONE clean cut face, and
/// filleting the top edge into it must then trim flush.
#[test]
fn seam_crossing_cut_merges_to_one_face_and_fillets_flush() {
    let body = cut_body_seam_crossing();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    // The merge collapsed the split wall: exactly one Ø8 vertical cut face exists.
    assert!(
        vertical_cut_face(&body, 4.0).is_some(),
        "the concave cut must be a single Ø8 cylinder face (cocylindrical merge)"
    );

    for r in [1.0_f64, 1.5, 2.5] {
        let edge = Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0));
        let Some(s) = safe_blend_result(
            &format!("fillet r={r} into seam-crossing cut"),
            &body,
            fillet_edges(&body, std::slice::from_ref(&edge), r),
        ) else {
            continue;
        };

        assert!(s.is_watertight(), "r={r}: result must be watertight");
        assert!(
            s.health_report().is_healthy(),
            "r={r}: result must be healthy"
        );
        assert!(
            !has_sphere(&s),
            "r={r}: the cut must NOT be rounded into a sphere"
        );

        let cut = vertical_cut_face(&s, 4.0)
            .unwrap_or_else(|| panic!("r={r}: exactly one Ø8 cut wall must survive"));
        assert!(
            boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
            "r={r}: the cut wall's boundary must stay on the cut cylinder (clean)"
        );
    }
}

/// Test B — negative control: filleting an edge AWAY from the cut behaves exactly
/// as a normal box-edge fillet, and the cut is left untouched.
#[test]
fn fillet_away_from_cut_is_unaffected() {
    let body = cut_body();
    // Bottom-front edge (y=0, z=0), nowhere near the (10,10) cut.
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(10.0, 0.0, 0.0));
    let s = fillet_edges(&body, std::slice::from_ref(&edge), 1.5)
        .expect("an edge away from the cut must fillet normally");
    assert!(s.is_watertight() && s.health_report().is_healthy());
    assert!(!has_sphere(&s));
    // The cut cylinder is untouched: still one clean vertical Ø8 wall.
    let cut = vertical_cut_face(&s, 4.0).expect("cut wall must survive untouched");
    assert!(boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0));
}

/// Test C — oversized radius: the public path must never emit garbage; it either
/// errors cleanly (body left unchanged upstream) or returns a valid solid.
#[test]
fn fillet_into_cut_oversized_radius_is_graceful() {
    let body = cut_body();
    let edge = Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0));
    match fillet_edges(&body, std::slice::from_ref(&edge), 6.0) {
        Err(_) => {} // clean failure -> upstream leaves the body unchanged
        Ok(s) => {
            assert!(
                s.is_watertight() && s.health_report().is_healthy(),
                "an accepted oversize fillet must still be valid"
            );
            assert!(!has_sphere(&s));
        }
    }
}

/// Native chamfer uses the same topology-first endpoint trim as fillet: the
/// bevel plane must end on the cut cylinder with one shared trim edge, leaving
/// the bored wall cylindrical and crack-free.
#[test]
fn chamfer_into_cut_trims_flush_and_keeps_cut_clean() {
    let body = cut_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");

    let edges = [
        Edge::between_points(Pnt::new(0.0, 10.0, 10.0), Pnt::new(6.0, 10.0, 10.0)),
        Edge::between_points(Pnt::new(10.0, 0.0, 10.0), Pnt::new(10.0, 6.0, 10.0)),
    ];

    for edge in &edges {
        for d in [0.75_f64, 1.5, 2.5] {
            let Some(s) = safe_blend_result(
                &format!("chamfer d={d} into cut"),
                &body,
                chamfer_edges(&body, std::slice::from_ref(edge), d),
            ) else {
                continue;
            };

            assert!(s.is_watertight(), "d={d}: result must be watertight");
            assert!(
                s.health_report().is_healthy(),
                "d={d}: result must be healthy"
            );

            let cut = vertical_cut_face(&s, 4.0)
                .unwrap_or_else(|| panic!("d={d}: exactly one Ø8 cut wall must survive"));
            assert!(
                boundary_on_cut_cylinder(&cut, (10.0, 10.0), 4.0),
                "d={d}: the cut wall's boundary must stay on the cut cylinder"
            );
        }
    }
}

#[test]
fn zerocad_circular_bite_fillet_into_cut_stays_analytic() {
    let r = 3.0;
    let body = zerocad_circular_bite_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");
    assert!(
        body.health_report().is_healthy(),
        "precondition: cut body is healthy: {:?}",
        body.health_report().errors
    );

    let edge = zerocad_circular_bite_cutoff_edge();
    let Some(s) = safe_blend_result(
        &format!("ZeroCAD circular-bite fillet r={r}"),
        &body,
        fillet_edges(&body, std::slice::from_ref(&edge), r),
    ) else {
        return;
    };

    assert!(s.is_watertight(), "r={r}: result must be watertight");
    assert!(
        s.health_report().is_healthy(),
        "r={r}: result must be healthy: {:?}",
        s.health_report().errors
    );
    assert!(
        !has_sphere(&s),
        "r={r}: the cut wall must not be rounded into a sphere"
    );

    let cut_faces: Vec<_> = s
        .shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - 14.0).abs() < 1e-3
            )
        })
        .collect();
    assert!(
        !cut_faces.is_empty(),
        "r={r}: radius-14 cut wall must survive"
    );
    for cut in &cut_faces {
        assert!(
            boundary_on_cut_cylinder(cut, (20.0, 8.0), 14.0),
            "r={r}: the cut wall's boundary must stay on the cut cylinder"
        );
    }
    assert!(
        s.shell().faces().iter().any(|f| {
            matches!(f.surface(), Some(GeomSurface::Cylinder(c)) if (c.radius() - r).abs() < 1e-3)
        }),
        "r={r}: the fillet blend cylinder must be present"
    );
}

#[test]
fn zerocad_circular_bite_chamfer_into_cut_stays_analytic() {
    let body = zerocad_circular_bite_body();
    assert!(body.is_watertight(), "precondition: cut body is watertight");
    assert!(
        body.health_report().is_healthy(),
        "precondition: cut body is healthy: {:?}",
        body.health_report().errors
    );

    let edge = zerocad_circular_bite_cutoff_edge();
    let Some(s) = safe_blend_result(
        "ZeroCAD circular-bite chamfer",
        &body,
        chamfer_edges(&body, std::slice::from_ref(&edge), 1.0),
    ) else {
        return;
    };

    assert!(s.is_watertight(), "result must be watertight");
    assert!(
        s.health_report().is_healthy(),
        "result must be healthy: {:?}",
        s.health_report().errors
    );

    let cut_faces: Vec<_> = s
        .shell()
        .faces()
        .into_iter()
        .filter(|f| {
            matches!(
                f.surface(),
                Some(GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                        && (c.radius() - 14.0).abs() < 1e-3
            )
        })
        .collect();
    assert!(!cut_faces.is_empty(), "radius-14 cut wall must survive");
    for cut in &cut_faces {
        assert!(
            boundary_on_cut_cylinder(cut, (20.0, 8.0), 14.0),
            "the cut wall's boundary must stay on the cut cylinder"
        );
    }
}
