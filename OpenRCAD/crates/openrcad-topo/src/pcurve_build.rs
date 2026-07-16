//! Construction of validated face-local pcurves for operation-created edges.

use std::{collections::HashSet, sync::Arc};

use openrcad_foundation::{
    Ax22d, Dir2d, Pnt, Pnt2d, TolerancePolicy, TolerancePolicyError, Vec as GeomVec,
};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Plane, Surface};
use openrcad_geom2d::{BSplineCurve2d, Circle2d, Ellipse2d, GeomCurve2d, Line2d};

use crate::arena::{BRep, EdgeData, EdgeId, FaceId, LoopId};
use crate::{PcurveData, Solid, SurfacePeriodicity};

/// Failure while constructing a missing pcurve.
#[derive(Clone, Debug, PartialEq)]
pub enum PcurveBuildError {
    InvalidTolerancePolicy(TolerancePolicyError),
    MissingTopology,
    ProjectionFailed {
        face: FaceId,
        loop_id: LoopId,
        edge: EdgeId,
    },
    Inconsistent {
        face: FaceId,
        loop_id: LoopId,
        edge: EdgeId,
        max_deviation: f64,
        tolerance: f64,
    },
}

impl core::fmt::Display for PcurveBuildError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => write!(f, "invalid tolerance policy: {error}"),
            Self::MissingTopology => write!(f, "solid contains dangling topology"),
            Self::ProjectionFailed {
                face,
                loop_id,
                edge,
            } => write!(
                f,
                "could not project face {face:?} loop {loop_id:?} edge {edge:?}"
            ),
            Self::Inconsistent {
                face,
                loop_id,
                edge,
                max_deviation,
                tolerance,
            } => write!(
                f,
                "generated pcurve for face {face:?} loop {loop_id:?} edge {edge:?} deviates by {max_deviation:e} (tolerance {tolerance:e})"
            ),
        }
    }
}

impl std::error::Error for PcurveBuildError {}

impl Solid {
    /// Return a copy with every missing surface-backed coedge pcurve generated
    /// and verified against its 3D edge. Existing pcurves remain authoritative.
    pub fn complete_missing_pcurves(
        &self,
        policy: &TolerancePolicy,
    ) -> Result<(Self, usize), PcurveBuildError> {
        self.complete_pcurves_impl(policy, false, None)
            .map(|(solid, rebuilt, _, _)| (solid, rebuilt))
    }

    /// Return a copy whose missing or stale operation-created pcurves are
    /// rebuilt. Valid existing pcurves are preserved.
    pub fn repair_pcurves(
        &self,
        policy: &TolerancePolicy,
    ) -> Result<(Self, usize), PcurveBuildError> {
        self.complete_pcurves_impl(policy, true, None)
            .map(|(solid, rebuilt, _, _)| (solid, rebuilt))
    }

    /// Explicit legacy-import adapter. Missing/stale pcurves are rebuilt and
    /// approximate imported edges may have their tolerance promoted, never
    /// beyond `policy.snap_max`. The returned counts make both repairs visible
    /// to exchange-layer diagnostics and recovery metadata.
    pub fn repair_imported_pcurves_compatibility(
        &self,
        policy: &TolerancePolicy,
    ) -> Result<(Self, usize, usize, f64), PcurveBuildError> {
        self.complete_pcurves_impl(policy, true, Some(policy.snap_max))
    }

    fn complete_pcurves_impl(
        &self,
        policy: &TolerancePolicy,
        replace_inconsistent: bool,
        recovery_tolerance_cap: Option<f64>,
    ) -> Result<(Self, usize, usize, f64), PcurveBuildError> {
        policy
            .validate()
            .map_err(PcurveBuildError::InvalidTolerancePolicy)?;
        let mut brep = self.brep.as_ref().clone();
        let solid = brep
            .solids
            .get(self.id)
            .ok_or(PcurveBuildError::MissingTopology)?
            .clone();
        let mut tasks = Vec::new();
        for shell_id in solid.shells {
            let shell = brep
                .shells
                .get(shell_id)
                .ok_or(PcurveBuildError::MissingTopology)?;
            for &face_id in &shell.faces {
                let face = brep
                    .faces
                    .get(face_id)
                    .ok_or(PcurveBuildError::MissingTopology)?;
                let Some(surface) = face.surface.clone() else {
                    continue;
                };
                for loop_id in face
                    .outer_wire
                    .into_iter()
                    .chain(face.inner_wires.iter().copied())
                {
                    let wire = brep
                        .loops
                        .get(loop_id)
                        .ok_or(PcurveBuildError::MissingTopology)?;
                    for (coedge_index, coedge) in wire.edges.iter().enumerate() {
                        let edge = brep
                            .edges
                            .get(coedge.id)
                            .ok_or(PcurveBuildError::MissingTopology)?;
                        let needs_rebuild = match coedge.pcurve {
                            None => true,
                            Some(_) if !replace_inconsistent => false,
                            Some(id) => {
                                let tolerance = policy.pcurve_consistency.max(edge.tolerance);
                                !brep.pcurves.get(id).is_some_and(|pcurve| {
                                    pcurve.is_valid()
                                        && max_deviation(&brep, &surface, edge, pcurve, 96)
                                            <= tolerance
                                })
                            }
                        };
                        if needs_rebuild {
                            tasks.push((
                                face_id,
                                loop_id,
                                coedge_index,
                                coedge.id,
                                surface.clone(),
                            ));
                        }
                    }
                }
            }
        }

        let mut promoted_edges = HashSet::new();
        let mut maximum_promoted_tolerance = 0.0_f64;
        for (face_id, loop_id, coedge_index, edge_id, surface) in &tasks {
            let edge = brep
                .edges
                .get(*edge_id)
                .ok_or(PcurveBuildError::MissingTopology)?
                .clone();
            let tolerance = policy.pcurve_consistency.max(edge.tolerance);
            let allowed_tolerance =
                recovery_tolerance_cap.map_or(tolerance, |cap| tolerance.max(cap));
            let pcurve = build_pcurve(&brep, surface, &edge, policy, allowed_tolerance)
                .ok_or_else(|| PcurveBuildError::ProjectionFailed {
                    face: *face_id,
                    loop_id: *loop_id,
                    edge: *edge_id,
                })?;
            let deviation = max_deviation(&brep, surface, &edge, &pcurve, 96);
            if !deviation.is_finite() || deviation > allowed_tolerance {
                return Err(PcurveBuildError::Inconsistent {
                    face: *face_id,
                    loop_id: *loop_id,
                    edge: *edge_id,
                    max_deviation: deviation,
                    tolerance: allowed_tolerance,
                });
            }
            if deviation > tolerance {
                let promoted = (deviation + policy.resolution).min(allowed_tolerance);
                brep.edges[*edge_id].tolerance = brep.edges[*edge_id].tolerance.max(promoted);
                promoted_edges.insert(*edge_id);
                maximum_promoted_tolerance = maximum_promoted_tolerance.max(promoted);
            }
            let id = brep.pcurves.insert(pcurve);
            brep.loops[*loop_id].edges[*coedge_index].pcurve = Some(id);
        }

        Ok((
            Solid::from_id(Arc::new(brep), self.id),
            tasks.len(),
            promoted_edges.len(),
            maximum_promoted_tolerance,
        ))
    }
}

fn build_pcurve(
    brep: &BRep,
    surface: &GeomSurface,
    edge: &EdgeData,
    policy: &TolerancePolicy,
    allowed_tolerance: f64,
) -> Option<PcurveData> {
    if let Some(exact) = exact_planar_pcurve(brep, surface, edge) {
        return Some(exact);
    }
    if let Some(exact) = exact_ruled_boundary_pcurve(surface, edge) {
        return Some(exact);
    }
    let periodicity = surface_periodicity(surface);
    let tolerance = allowed_tolerance;

    // Analytic seams, rims, and generators become exact UV lines.
    let samples = project_samples(brep, surface, edge, 9, periodicity)?;
    if let Some(line) = line_pcurve(&samples, periodicity) {
        if max_deviation(brep, surface, edge, &line, 96) <= tolerance {
            return Some(line);
        }
    }

    // A truly collapsed pole use still needs an explicit, evaluable pcurve.
    // Coincident endpoints alone are insufficient: closed circles and closed
    // B-spline intersection boundaries also start and end at one vertex.
    let start = edge_point(brep, edge, 0.0)?;
    let collapsed = [0.25, 0.5, 0.75, 1.0].into_iter().all(|fraction| {
        edge_point(brep, edge, fraction)
            .is_some_and(|point| start.distance(&point) <= policy.linear)
    });
    if collapsed {
        let uv = samples[0];
        return Some(
            PcurveData::new(
                GeomCurve2d::circle(Circle2d::from_center(uv, 0.0)),
                0.0,
                1.0,
            )
            .with_periodicity(periodicity),
        );
    }

    // General curve-on-surface fallback: refine a UV polyline until lifting it
    // through the surface agrees with the 3D edge to policy tolerance.
    let mut count = 17;
    // Imported intersection curves may be long, closed cubic splines. Their
    // explicit compatibility reconstruction still has to meet the same strict
    // pcurve tolerance, so allow enough adaptive refinement to validate them
    // instead of accepting a coarse approximation.
    while count <= 32769 {
        let points = project_samples(brep, surface, edge, count, periodicity)?;
        let candidate = polyline_pcurve(points, periodicity);
        // Use a verification grid that is deliberately not an integer multiple
        // of the polyline's knot intervals. A sparse projection of a long helix
        // can alias by whole turns and agree exactly at every knot and midpoint
        // while deviating between them. The coprime interval count detects that
        // alias and forces refinement until adjacent samples unwrap correctly.
        let deviation = max_deviation(brep, surface, edge, &candidate, count * 2 + 1);
        if deviation <= tolerance {
            return Some(candidate);
        }
        count = (count - 1) * 2 + 1;
    }
    None
}

/// A ruled surface's two rail curves are exact constant-v boundaries. Handling
/// them before projection is important for tapered, zero-lead helices: adding a
/// whole turn changes their radius, so angle-only projection cannot recover the
/// absolute curve parameter at the first sample.
fn exact_ruled_boundary_pcurve(surface: &GeomSurface, edge: &EdgeData) -> Option<PcurveData> {
    let GeomSurface::Ruled(ruled) = surface else {
        return None;
    };
    let curve = edge.curve.as_ref()?;
    let v = if curve == &ruled.curve1 {
        0.0
    } else if curve == &ruled.curve2 {
        1.0
    } else {
        return None;
    };
    line_pcurve(
        &[Pnt2d::new(edge.first, v), Pnt2d::new(edge.last, v)],
        surface_periodicity(surface),
    )
}

fn exact_planar_pcurve(brep: &BRep, surface: &GeomSurface, edge: &EdgeData) -> Option<PcurveData> {
    let GeomSurface::Plane(plane) = surface else {
        return None;
    };
    let curve = edge.curve.as_ref()?;
    let project = |point: Pnt| plane_uv(plane, point);
    let direction = |dir: openrcad_foundation::Dir| {
        Dir2d::new(
            dir.dot(&plane.position().x_direction()),
            dir.dot(&plane.position().y_direction()),
        )
    };
    match curve {
        GeomCurve::Circle(circle) => Some(PcurveData::new(
            GeomCurve2d::circle(Circle2d::new(
                Ax22d::new_axes(
                    project(circle.center()),
                    direction(circle.position().x_direction()),
                    direction(circle.position().y_direction()),
                ),
                circle.radius(),
            )),
            edge.first,
            edge.last,
        )),
        GeomCurve::Ellipse(ellipse) => Some(PcurveData::new(
            GeomCurve2d::ellipse(Ellipse2d::new(
                Ax22d::new_axes(
                    project(ellipse.center()),
                    direction(ellipse.position().x_direction()),
                    direction(ellipse.position().y_direction()),
                ),
                ellipse.major_radius(),
                ellipse.minor_radius(),
            )),
            edge.first,
            edge.last,
        )),
        GeomCurve::BSpline(curve) => {
            let poles = curve.poles().iter().copied().map(project).collect();
            Some(PcurveData::new(
                GeomCurve2d::bspline(BSplineCurve2d::new(
                    curve.degree(),
                    poles,
                    curve.weights().map(<[f64]>::to_vec),
                    curve.knots().to_vec(),
                    curve.multiplicities().to_vec(),
                )),
                edge.first,
                edge.last,
            ))
        }
        GeomCurve::Line(_) => {
            let start = project(brep.vertices.get(edge.start)?.point);
            let end = project(brep.vertices.get(edge.end)?.point);
            line_pcurve(&[start, end], SurfacePeriodicity::NONE)
        }
        // Parabolas, hyperbolas, helices, and future curve families need the
        // adaptive projection path below. Replacing them with a chord would
        // only match their endpoints and could silently invalidate the face
        // interior.
        _ => None,
    }
}

fn line_pcurve(points: &[Pnt2d], periodicity: SurfacePeriodicity) -> Option<PcurveData> {
    let first = *points.first()?;
    let last = *points.last()?;
    let dx = last.x() - first.x();
    let dy = last.y() - first.y();
    let length = dx.hypot(dy);
    let direction = Dir2d::try_new(dx, dy)?;
    Some(
        PcurveData::new(
            GeomCurve2d::line(Line2d::from_point_dir(first, direction)),
            0.0,
            length,
        )
        .with_periodicity(periodicity),
    )
}

fn polyline_pcurve(points: Vec<Pnt2d>, periodicity: SurfacePeriodicity) -> PcurveData {
    let last = points.len() - 1;
    let knots = (0..=last).map(|index| index as f64).collect::<Vec<_>>();
    let mults = (0..=last)
        .map(|index| if index == 0 || index == last { 2 } else { 1 })
        .collect();
    PcurveData::new(
        GeomCurve2d::bspline(BSplineCurve2d::new(1, points, None, knots, mults)),
        0.0,
        last as f64,
    )
    .with_periodicity(periodicity)
}

fn project_samples(
    brep: &BRep,
    surface: &GeomSurface,
    edge: &EdgeData,
    count: usize,
    periodicity: SurfacePeriodicity,
) -> Option<Vec<Pnt2d>> {
    let mut out = Vec::with_capacity(count);
    // At a spherical pole longitude is undefined. Seed projection from the
    // first interior edge sample so a pole endpoint inherits the longitude of
    // the great-circle boundary instead of arbitrarily starting at zero.
    let mut hint = if matches!(surface, GeomSurface::Sphere(_)) && count > 2 {
        edge_point(brep, edge, 1.0 / (count - 1) as f64)
            .and_then(|point| project_point(surface, point, None))
    } else {
        None
    };
    for index in 0..count {
        let fraction = index as f64 / (count - 1) as f64;
        let point = edge_point(brep, edge, fraction)?;
        let mut uv = project_point(surface, point, hint)?;
        if let Some(previous) = hint {
            uv = Pnt2d::new(
                unwrap_near(uv.x(), previous.x(), periodicity.u_period),
                unwrap_near(uv.y(), previous.y(), periodicity.v_period),
            );
        }
        hint = Some(uv);
        out.push(uv);
    }
    Some(out)
}

fn edge_point(brep: &BRep, edge: &EdgeData, fraction: f64) -> Option<Pnt> {
    if let Some(curve) = &edge.curve {
        return Some(curve.point(edge.first + (edge.last - edge.first) * fraction));
    }
    let start = brep.vertices.get(edge.start)?.point;
    let end = brep.vertices.get(edge.end)?.point;
    Some(start + (end - start) * fraction)
}

fn max_deviation(
    brep: &BRep,
    surface: &GeomSurface,
    edge: &EdgeData,
    pcurve: &PcurveData,
    samples: usize,
) -> f64 {
    (0..=samples).fold(0.0_f64, |maximum, index| {
        let fraction = index as f64 / samples as f64;
        let Some(expected) = edge_point(brep, edge, fraction) else {
            return f64::INFINITY;
        };
        let uv = pcurve.point_at_fraction(fraction);
        maximum.max(surface.point(uv.x(), uv.y()).distance(&expected))
    })
}

fn surface_periodicity(surface: &GeomSurface) -> SurfacePeriodicity {
    let (u0, u1, v0, v1) = surface.bounds();
    let curve_period = |curve: &GeomCurve| match curve {
        GeomCurve::Helix(helix)
            if helix.taper().abs() <= 1.0e-12 && helix.lead().abs() <= 1.0e-12 =>
        {
            Some(core::f64::consts::TAU)
        }
        curve if curve.is_periodic() && curve.period().is_finite() && curve.period() > 0.0 => {
            Some(curve.period())
        }
        _ => None,
    };
    let ruled_u_period = match surface {
        GeomSurface::Ruled(ruled) => {
            match (curve_period(&ruled.curve1), curve_period(&ruled.curve2)) {
                (Some(first), Some(second)) if (first - second).abs() <= 1.0e-12 => Some(first),
                _ => None,
            }
        }
        _ => None,
    };
    SurfacePeriodicity {
        u_period: ruled_u_period.or_else(|| {
            (surface.is_uclosed() && u0.is_finite() && u1.is_finite()).then_some((u1 - u0).abs())
        }),
        v_period: (surface.is_vclosed() && v0.is_finite() && v1.is_finite())
            .then_some((v1 - v0).abs()),
    }
}

fn unwrap_near(value: f64, previous: f64, period: Option<f64>) -> f64 {
    let Some(period) = period.filter(|period| period.is_finite() && *period > 0.0) else {
        return value;
    };
    value + ((previous - value) / period).round() * period
}

fn plane_uv(plane: &Plane, point: Pnt) -> Pnt2d {
    let offset = point - plane.location();
    Pnt2d::new(
        offset.dot(&GeomVec::from_dir(plane.position().x_direction())),
        offset.dot(&GeomVec::from_dir(plane.position().y_direction())),
    )
}

fn project_point(surface: &GeomSurface, point: Pnt, hint: Option<Pnt2d>) -> Option<Pnt2d> {
    let uv = match surface {
        GeomSurface::Plane(plane) => plane_uv(plane, point),
        GeomSurface::Cylinder(cylinder) => axial_uv(cylinder.position(), point, hint),
        GeomSurface::Cone(cone) => axial_uv(cone.position(), point, hint),
        GeomSurface::Sphere(sphere) => {
            let frame = sphere.position();
            let offset = point - frame.location();
            let radius = sphere.radius();
            if radius <= 0.0 {
                return None;
            }
            let z = offset.dot(&GeomVec::from_dir(frame.direction()));
            let x = offset.dot(&GeomVec::from_dir(frame.x_direction()));
            let y = offset.dot(&GeomVec::from_dir(frame.y_direction()));
            let latitude = (z / radius).clamp(-1.0, 1.0).asin();
            let longitude = if x.hypot(y) <= openrcad_foundation::tolerance::CONFUSION {
                hint.map_or(0.0, |point| point.x())
            } else {
                y.atan2(x).rem_euclid(core::f64::consts::TAU)
            };
            Pnt2d::new(longitude, latitude)
        }
        GeomSurface::Torus(torus) => {
            let frame = torus.position();
            let offset = point - frame.location();
            let x = offset.dot(&GeomVec::from_dir(frame.x_direction()));
            let y = offset.dot(&GeomVec::from_dir(frame.y_direction()));
            let z = offset.dot(&GeomVec::from_dir(frame.direction()));
            let u = y.atan2(x).rem_euclid(core::f64::consts::TAU);
            let v = z
                .atan2(x.hypot(y) - torus.major_radius())
                .rem_euclid(core::f64::consts::TAU);
            Pnt2d::new(u, v)
        }
        GeomSurface::Ruled(ruled) => {
            if let Some((u, v)) = ruled.helical_uv_hinted(point, hint.map(|p| (p.x(), p.y()))) {
                Pnt2d::new(u, v)
            } else {
                newton_uv(surface, point, hint)?
            }
        }
        GeomSurface::BSpline(_) | GeomSurface::Gregory(_) | GeomSurface::Offset(_) => {
            newton_uv(surface, point, hint)?
        }
    };
    (uv.x().is_finite() && uv.y().is_finite()).then_some(uv)
}

fn axial_uv(frame: openrcad_foundation::Ax3, point: Pnt, hint: Option<Pnt2d>) -> Pnt2d {
    let offset = point - frame.location();
    let v = offset.dot(&GeomVec::from_dir(frame.direction()));
    let x = offset.dot(&GeomVec::from_dir(frame.x_direction()));
    let y = offset.dot(&GeomVec::from_dir(frame.y_direction()));
    let u = if x.hypot(y) <= openrcad_foundation::tolerance::CONFUSION {
        hint.map_or(0.0, |point| point.x())
    } else {
        y.atan2(x).rem_euclid(core::f64::consts::TAU)
    };
    Pnt2d::new(u, v)
}

fn newton_uv(surface: &GeomSurface, point: Pnt, hint: Option<Pnt2d>) -> Option<Pnt2d> {
    let (u0, u1, v0, v1) = surface.bounds();
    let midpoint = |a: f64, b: f64| match (a.is_finite(), b.is_finite()) {
        (true, true) => 0.5 * (a + b),
        _ => 0.0,
    };
    let mut u = hint.map_or_else(|| midpoint(u0, u1), |point| point.x());
    let mut v = hint.map_or_else(|| midpoint(v0, v1), |point| point.y());
    for _ in 0..24 {
        let (on_surface, su, sv) = surface.d1(u, v);
        let delta = on_surface - point;
        let a = su.dot(&su);
        let b = su.dot(&sv);
        let c = sv.dot(&sv);
        let det = a * c - b * b;
        if det.abs() <= 1e-20 {
            break;
        }
        let gu = delta.dot(&su);
        let gv = delta.dot(&sv);
        let du = (c * gu - b * gv) / det;
        let dv = (-b * gu + a * gv) / det;
        u = clamp_bound(u - du, u0, u1);
        v = clamp_bound(v - dv, v0, v1);
        if du.abs() <= 1e-11 && dv.abs() <= 1e-11 {
            break;
        }
    }
    let projected = surface.point(u, v);
    (projected.distance(&point).is_finite()).then_some(Pnt2d::new(u, v))
}

fn clamp_bound(value: f64, first: f64, last: f64) -> f64 {
    match (first.is_finite(), last.is_finite()) {
        (true, true) => value.clamp(first.min(last), first.max(last)),
        (true, false) => value.max(first),
        (false, true) => value.min(last),
        (false, false) => value,
    }
}
