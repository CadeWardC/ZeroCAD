//! A rounded pocket opening stores its rim in the top face's inner wire. This is
//! the topology produced by a sketch fillet tangent to straight pocket edges.

use openrcad_algo::{
    apply_blend_contour, boolean_operation, prism_operation, BlendContour, BlendContourError,
    BlendCurveHint, BlendKind, BooleanOp,
};
use openrcad_foundation::{Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Plane};
use openrcad_primitives::make_box;
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};

fn rounded_pocket_body() -> Result<Solid, String> {
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
    let tool = prism_operation(&profile, GeomVec::new(0.0, 0.0, 6.0))
        .map_err(|error| error.to_string())?
        .value;
    let body = boolean_operation(&block, &tool, BooleanOp::Cut)
        .map_err(|error| error.to_string())?
        .value;
    assert!(
        body.is_watertight() && body.health_report().is_healthy(),
        "rounded-pocket fixture invalid: {:?}",
        body.health_report().errors
    );
    Ok(body)
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

fn assert_safe_blend_outcome(body: &Solid, result: Result<Solid, BlendContourError>, label: &str) {
    match result {
        Ok(result) => {
            assert!(result.is_watertight(), "{label} returned an open shell");
            assert!(
                result.health_report().is_healthy(),
                "{label} returned an unhealthy solid"
            );
        }
        Err(error) => {
            assert!(
                !error.to_string().is_empty(),
                "{label} returned an empty diagnostic"
            );
            assert!(body.is_watertight() && body.health_report().is_healthy());
        }
    }
}

fn blend_rounded_pocket_rim(kind: BlendKind) {
    let body = match rounded_pocket_body() {
        Ok(body) => body,
        Err(error) => {
            assert!(!error.is_empty(), "pocket rejection needs a diagnostic");
            return;
        }
    };
    let rim = rounded_top_rim(&body);
    assert!(!rim.is_empty(), "rounded top rim must stay analytic");
    let contour = BlendContour::constant(rim, kind, 1.0, Some(BlendCurveHint::Circle));
    let result = apply_blend_contour(&body, &contour);
    assert_safe_blend_outcome(&body, result, &format!("rounded pocket rim {kind:?}"));
}

#[test]
fn rounded_pocket_inner_rim_fillet_is_fail_safe() {
    blend_rounded_pocket_rim(BlendKind::Fillet);
}

#[test]
fn rounded_pocket_inner_rim_chamfer_is_fail_safe() {
    blend_rounded_pocket_rim(BlendKind::Chamfer);
}

#[test]
fn tangent_pocket_inner_rim_fillet_is_fail_safe() {
    let body = match rounded_pocket_body() {
        Ok(body) => body,
        Err(error) => {
            assert!(!error.is_empty(), "pocket rejection needs a diagnostic");
            return;
        }
    };
    let contour = BlendContour::constant(
        tangent_top_rim(&body),
        BlendKind::Fillet,
        1.0,
        Some(BlendCurveHint::Circle),
    );
    let result = apply_blend_contour(&body, &contour);
    assert_safe_blend_outcome(&body, result, "tangent pocket rim fillet");
}

#[test]
fn tangent_pocket_inner_rim_chamfer_is_fail_safe() {
    let body = match rounded_pocket_body() {
        Ok(body) => body,
        Err(error) => {
            assert!(!error.is_empty(), "pocket rejection needs a diagnostic");
            return;
        }
    };
    let contour = BlendContour::constant(
        tangent_top_rim(&body),
        BlendKind::Chamfer,
        1.0,
        Some(BlendCurveHint::Circle),
    );
    let result = apply_blend_contour(&body, &contour);
    assert_safe_blend_outcome(&body, result, "tangent pocket rim chamfer");
}
