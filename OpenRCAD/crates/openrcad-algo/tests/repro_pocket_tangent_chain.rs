//! A rounded pocket opening stores its rim in the top face's inner wire. This is
//! the topology produced by a sketch fillet tangent to straight pocket edges.

use openrcad_algo::{
    apply_blend_contour, boolean, prism, BlendContour, BlendCurveHint, BlendKind, BooleanOp,
};
use openrcad_foundation::{Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_primitives::make_box;
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};

fn rounded_pocket_body() -> Solid {
    let block = make_box(&Pnt::origin(), 30.0, 20.0, 10.0);
    let circle = Circle::new(
        Ax3::new_axes(Pnt::new(15.0, 10.0, 4.0), Dir::dz(), Dir::dx()),
        5.0,
    );
    let arc = Edge::new(
        Some(GeomCurve::circle(circle)),
        -core::f64::consts::FRAC_PI_2,
        core::f64::consts::FRAC_PI_2,
        Vertex::new(circle.point(-core::f64::consts::FRAC_PI_2)),
        Vertex::new(circle.point(core::f64::consts::FRAC_PI_2)),
    );
    let profile = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            Pnt::new(0.0, 0.0, 4.0),
            Dir::dz(),
        ))),
        Wire::from_edges([
            Edge::between_points(Pnt::new(10.0, 5.0, 4.0), Pnt::new(15.0, 5.0, 4.0)),
            arc,
            Edge::between_points(Pnt::new(15.0, 15.0, 4.0), Pnt::new(10.0, 15.0, 4.0)),
            Edge::between_points(Pnt::new(10.0, 15.0, 4.0), Pnt::new(10.0, 5.0, 4.0)),
        ]),
    );
    let tool = prism(&profile, GeomVec::new(0.0, 0.0, 6.0)).expect("pocket tool");
    let body = boolean(&block, &tool, BooleanOp::Cut);
    assert!(
        body.is_watertight() && body.health_report().is_healthy(),
        "rounded-pocket fixture invalid: {:?}",
        body.health_report().errors
    );
    body
}

fn rounded_top_rim(body: &Solid) -> Vec<Edge> {
    body.edges()
        .into_iter()
        .filter(|edge| {
            let Some(GeomCurve::Circle(circle)) = edge.curve() else {
                return false;
            };
            (circle.radius() - 5.0).abs() < 1.0e-4
                && (circle.center().x() - 15.0).abs() < 1.0e-4
                && (circle.center().y() - 10.0).abs() < 1.0e-4
                && edge.source().point().z() > 9.999
                && edge.target().point().z() > 9.999
        })
        .collect()
}

fn tangent_top_rim(body: &Solid) -> Vec<Edge> {
    let near = |p: Pnt, q: Pnt| p.distance(&q) < 1.0e-4;
    let find = |a: Pnt, b: Pnt| {
        body.edges()
            .into_iter()
            .find(|edge| {
                let (p, q) = (edge.source().point(), edge.target().point());
                (near(p, a) && near(q, b)) || (near(p, b) && near(q, a))
            })
            .expect("tangent rim edge")
    };
    let mut result = vec![find(Pnt::new(10.0, 5.0, 10.0), Pnt::new(15.0, 5.0, 10.0))];
    result.extend(rounded_top_rim(body));
    result.push(find(Pnt::new(15.0, 15.0, 10.0), Pnt::new(10.0, 15.0, 10.0)));
    result
}

fn blend_rounded_pocket_rim(kind: BlendKind) {
    let body = rounded_pocket_body();
    let rim = rounded_top_rim(&body);
    assert!(!rim.is_empty(), "rounded top rim must stay analytic");
    let contour = BlendContour::constant(rim, kind, 1.0, Some(BlendCurveHint::Circle));
    let result = apply_blend_contour(&body, &contour)
        .unwrap_or_else(|error| panic!("rounded pocket rim {kind:?} failed: {error}"));
    assert!(result.is_watertight());
    assert!(result.health_report().is_healthy());
}

#[test]
fn rounded_pocket_inner_rim_fillet_succeeds() {
    blend_rounded_pocket_rim(BlendKind::Fillet);
}

#[test]
fn rounded_pocket_inner_rim_chamfer_succeeds() {
    blend_rounded_pocket_rim(BlendKind::Chamfer);
}

#[test]
fn tangent_pocket_inner_rim_fillet_succeeds() {
    let body = rounded_pocket_body();
    let contour = BlendContour::constant(
        tangent_top_rim(&body),
        BlendKind::Fillet,
        1.0,
        Some(BlendCurveHint::Circle),
    );
    let result = apply_blend_contour(&body, &contour)
        .unwrap_or_else(|error| panic!("tangent pocket rim fillet failed: {error}"));
    assert!(result.is_watertight());
    assert!(result.health_report().is_healthy());
}

#[test]
fn tangent_pocket_inner_rim_chamfer_succeeds() {
    let body = rounded_pocket_body();
    let contour = BlendContour::constant(
        tangent_top_rim(&body),
        BlendKind::Chamfer,
        1.0,
        Some(BlendCurveHint::Circle),
    );
    let result = apply_blend_contour(&body, &contour)
        .unwrap_or_else(|error| panic!("tangent pocket rim chamfer failed: {error}"));
    assert!(result.is_watertight());
    assert!(result.health_report().is_healthy());
}
