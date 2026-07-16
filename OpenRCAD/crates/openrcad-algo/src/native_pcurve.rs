//! Exact construction helpers for operation-owned trimming curves.
//!
//! These helpers deliberately do not project sampled 3D boundaries back onto a
//! surface.  Callers supply coordinates that are already known from the
//! construction, while planar curves are transformed analytically into the
//! plane's parameter frame.

use openrcad_foundation::{Ax22d, Dir2d, Pnt, Pnt2d, Vec as GeomVec};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Plane};
use openrcad_geom2d::{BSplineCurve2d, Circle2d, Ellipse2d, GeomCurve2d, Line2d};
use openrcad_topo::{
    Edge, Face, FaceBuildError, Orientation, PcurveData, SurfacePeriodicity, Wire,
};

/// A straight face-local curve between two known UV coordinates.
pub(crate) fn uv_line(start: Pnt2d, end: Pnt2d, periodicity: SurfacePeriodicity) -> PcurveData {
    let delta_x = end.x() - start.x();
    let delta_y = end.y() - start.y();
    let length = delta_x.hypot(delta_y);
    let direction = Dir2d::try_new(delta_x, delta_y).unwrap_or_else(Dir2d::dx);
    PcurveData::new(
        GeomCurve2d::line(Line2d::from_point_dir(start, direction)),
        0.0,
        length.max(openrcad_foundation::tolerance::CONFUSION),
    )
    .with_periodicity(periodicity)
}

/// Periodic directions of the analytic surface families used by native sweeps.
pub(crate) fn surface_periodicity(surface: &GeomSurface) -> SurfacePeriodicity {
    use core::f64::consts::TAU;

    match surface {
        GeomSurface::Cylinder(_) | GeomSurface::Cone(_) | GeomSurface::Sphere(_) => {
            SurfacePeriodicity::u_periodic(TAU)
        }
        GeomSurface::Torus(_) => SurfacePeriodicity {
            u_period: Some(TAU),
            v_period: Some(TAU),
        },
        _ => SurfacePeriodicity::NONE,
    }
}

/// Move `value` onto the periodic branch closest to `reference`.
pub(crate) fn unwrap_near(value: f64, reference: f64, period: Option<f64>) -> f64 {
    period.map_or(value, |period| {
        value + period * ((reference - value) / period).round()
    })
}

/// Build a planar face with exact pcurves on the outer and inner wires.
pub(crate) fn planar_face_with_pcurves(
    plane: Plane,
    outer: Option<Wire>,
    inners: Vec<Wire>,
    orientation: Orientation,
) -> Result<Face, FaceBuildError> {
    let outer = outer.map(|wire| {
        let pcurves = wire
            .edges()
            .iter()
            .map(|edge| planar_edge_pcurve(&plane, edge))
            .collect();
        (wire, pcurves)
    });
    let inners = inners
        .into_iter()
        .map(|wire| {
            let pcurves = wire
                .edges()
                .iter()
                .map(|edge| planar_edge_pcurve(&plane, edge))
                .collect();
            (wire, pcurves)
        })
        .collect();
    Face::with_wires_and_pcurves(Some(GeomSurface::plane(plane)), outer, inners, orientation)
}

/// Build a face on an analytic surface whose constructor guarantees that each
/// boundary edge follows a surface coordinate line. Pcurves are bound in the
/// same transaction as the face, so the face never exists in a surface-backed
/// state with missing coedge data.
pub(crate) fn analytic_face_with_pcurves(
    surface: GeomSurface,
    outer: Wire,
    orientation: Orientation,
) -> Result<Face, FaceBuildError> {
    let pcurves = outer
        .edges()
        .iter()
        .map(|edge| analytic_line_pcurve(&surface, edge))
        .collect();
    Face::with_wires_and_pcurves(
        Some(surface),
        Some((outer, pcurves)),
        Vec::new(),
        orientation,
    )
}

/// Exact image of a planar 3D edge in the plane's Cartesian UV frame.
fn planar_edge_pcurve(plane: &Plane, edge: &Edge) -> PcurveData {
    let project_point = |point: Pnt| {
        let offset = point - plane.location();
        Pnt2d::new(
            offset.dot(&GeomVec::from_dir(plane.position().x_direction())),
            offset.dot(&GeomVec::from_dir(plane.position().y_direction())),
        )
    };
    let project_direction = |direction: openrcad_foundation::Dir| {
        Dir2d::new(
            direction.dot(&plane.position().x_direction()),
            direction.dot(&plane.position().y_direction()),
        )
    };

    match edge.curve() {
        Some(GeomCurve::Circle(circle)) => {
            let frame = Ax22d::new_axes(
                project_point(circle.center()),
                project_direction(circle.position().x_direction()),
                project_direction(circle.position().y_direction()),
            );
            PcurveData::new(
                GeomCurve2d::circle(Circle2d::new(frame, circle.radius())),
                edge.first(),
                edge.last(),
            )
        }
        Some(GeomCurve::Ellipse(ellipse)) => {
            let frame = Ax22d::new_axes(
                project_point(ellipse.center()),
                project_direction(ellipse.position().x_direction()),
                project_direction(ellipse.position().y_direction()),
            );
            PcurveData::new(
                GeomCurve2d::ellipse(Ellipse2d::new(
                    frame,
                    ellipse.major_radius(),
                    ellipse.minor_radius(),
                )),
                edge.first(),
                edge.last(),
            )
        }
        Some(GeomCurve::BSpline(curve)) => {
            let poles = curve.poles().iter().copied().map(project_point).collect();
            PcurveData::new(
                GeomCurve2d::bspline(BSplineCurve2d::new(
                    curve.degree(),
                    poles,
                    curve.weights().map(<[f64]>::to_vec),
                    curve.knots().to_vec(),
                    curve.multiplicities().to_vec(),
                )),
                edge.first(),
                edge.last(),
            )
        }
        _ => uv_line(
            project_point(edge.start().point()),
            project_point(edge.end().point()),
            SurfacePeriodicity::NONE,
        ),
    }
}

/// Exact UV line for an edge known by its constructor to follow one coordinate
/// line on an analytic surface. The midpoint selects the correct unwrapped
/// branch at periodic seams; no iterative projection or curve fitting occurs.
pub(crate) fn analytic_line_pcurve(surface: &GeomSurface, edge: &Edge) -> PcurveData {
    if let GeomSurface::Plane(plane) = surface {
        return planar_edge_pcurve(plane, edge);
    }

    let edge_point = |fraction: f64| {
        edge.curve().map_or_else(
            || {
                let start = edge.start().point();
                start + (edge.end().point() - start) * fraction
            },
            |curve| curve.point(edge.first() + (edge.last() - edge.first()) * fraction),
        )
    };
    let periodicity = surface_periodicity(surface);
    let start = crate::intersect::uv_of(surface, &edge_point(0.0));
    let middle = crate::intersect::uv_of(surface, &edge_point(0.5));
    let end = crate::intersect::uv_of(surface, &edge_point(1.0));
    let middle = (
        unwrap_near(middle.0, start.0, periodicity.u_period),
        unwrap_near(middle.1, start.1, periodicity.v_period),
    );
    let end = (
        unwrap_near(end.0, middle.0, periodicity.u_period),
        unwrap_near(end.1, middle.1, periodicity.v_period),
    );
    uv_line(
        Pnt2d::new(start.0, start.1),
        Pnt2d::new(end.0, end.1),
        periodicity,
    )
}
