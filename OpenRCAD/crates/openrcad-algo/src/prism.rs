//! Prism/extrusion sweeping for B-Rep faces.
//!
//! This is the OpenRCAD equivalent of the practical core of OCCT's
//! `BRepSweep_Prism`: duplicate the swept face for the far cap, generate one
//! lateral face per boundary edge, then sew the result into a watertight solid.

use core::fmt;

use openrcad_foundation::{
    tolerance, Ax3, Dir, Pnt, Pnt2d, TolerancePolicy, TolerancePolicyError, Trsf, Vec as GeomVec,
};
use openrcad_geom::{Curve, CylindricalSurface, GeomCurve, GeomSurface, Line, Plane, RuledSurface};
use openrcad_topo::{
    containment::point_in_polygon_2d, Edge, Face, FaceBuildError, HealthReport, OperationResult,
    Orientation, RecoveryReport, Solid, SurfacePeriodicity, TopologyHistory, ValidationReport,
    Wire,
};

use crate::native_pcurve::{analytic_line_pcurve, planar_face_with_pcurves, uv_line};
use crate::sew::sew_shell_with_policy as sew_with_policy;

/// Errors reported by prism/extrusion sweeping.
#[derive(Clone, Debug, PartialEq)]
pub enum SweepError {
    /// The supplied document tolerance policy is invalid.
    InvalidTolerancePolicy(TolerancePolicyError),
    /// A face and its exact construction-time pcurves could not be assembled.
    FaceBuild(FaceBuildError),
    /// The sweep vector has no usable length.
    DegenerateVector,
    /// The source face has no outer boundary.
    MissingOuterWire,
    /// A boundary wire is not closed.
    OpenWire,
    /// The sweep assembled a solid that failed the Phase 1 representation gate.
    InvalidOutput {
        report: HealthReport,
        watertight: bool,
        pcurves_complete: bool,
        /// The strict-validation failure, when that is what rejected the solid.
        /// The general `ValidationReport` can pass while the strict pcurve
        /// audit fails; without this field that rejection surfaced as an
        /// "invalid output" carrying an EMPTY health report — an error message
        /// that named no error, which made real tolerance failures (an f32
        /// input whose pcurve deviates by 1.4e-6 against a 1e-6 audit)
        /// undiagnosable from the log.
        strict: Option<openrcad_topo::ValidationError>,
    },
}

impl fmt::Display for SweepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "prism: invalid tolerance policy: {error}")
            }
            Self::FaceBuild(error) => write!(f, "prism: pcurve construction failed: {error}"),
            Self::DegenerateVector => f.write_str("prism: sweep vector must be non-zero"),
            Self::MissingOuterWire => f.write_str("prism: source face has no outer wire"),
            Self::OpenWire => f.write_str("prism: every swept wire must be closed"),
            Self::InvalidOutput {
                report,
                watertight,
                pcurves_complete,
                strict,
            } => {
                write!(
                    f,
                    "prism: invalid output (watertight={watertight}, pcurves_complete={pcurves_complete}): {report:?}"
                )?;
                if let Some(error) = strict {
                    write!(f, "; strict validation: {error}")?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for SweepError {}

/// Sweep `face` by `vector` and return a sewn solid.
///
/// Straight boundary edges generate planar lateral faces. Circular arcs whose
/// plane normal is parallel to the sweep vector generate cylindrical faces.
/// Other curves generate ruled lateral faces between the base and translated
/// edge, which covers NURBS/B-spline boundaries and skew circular sweeps.
fn build_prism(
    face: &Face,
    vector: GeomVec,
    policy: &TolerancePolicy,
    reconcile_laterals: bool,
) -> Result<Solid, SweepError> {
    if vector.magnitude() <= tolerance::CONFUSION {
        return Err(SweepError::DegenerateVector);
    }
    let Some(outer) = face.outer_wire() else {
        return Err(SweepError::MissingOuterWire);
    };
    if !outer.is_closed() {
        return Err(SweepError::OpenWire);
    }
    for wire in face.inner_wires() {
        if !wire.is_closed() {
            return Err(SweepError::OpenWire);
        }
    }

    let translation = Trsf::translation(vector);
    let mut faces = Vec::new();
    let face_normal = effective_face_normal(face);
    let sweep_points_along_normal = face_normal
        .map(|n| GeomVec::from_dir(n).dot(&vector) >= 0.0)
        .unwrap_or(true);

    faces.extend(cap_faces(
        face,
        &translation,
        sweep_points_along_normal,
        true,
    )?);

    for wire in face.wires() {
        for edge in wire.edges() {
            if edge.length() <= tolerance::CONFUSION {
                continue;
            }
            faces.push(lateral_face(
                &edge,
                &translation,
                vector,
                face,
                reconcile_laterals,
            )?);
        }
    }

    Ok(Solid::new(
        sew_with_policy(&faces, policy).map_err(SweepError::InvalidTolerancePolicy)?,
    ))
}

/// Signed loop area in a planar face's own UV frame, respecting coedge
/// traversal. Used only to canonicalize topological inner wires before the
/// sweep; containment itself remains orientation-independent.
fn planar_wire_signed_area(face: &Face, wire: &Wire) -> Option<f64> {
    let Some(GeomSurface::Plane(plane)) = face.surface() else {
        return None;
    };
    let frame = plane.position();
    let origin = frame.location();
    let to_uv = |point: Pnt| {
        let offset = point - origin;
        (
            offset.dot(&GeomVec::from_dir(frame.x_direction())),
            offset.dot(&GeomVec::from_dir(frame.y_direction())),
        )
    };
    let mut points = Vec::new();
    for edge in wire.edges() {
        if let Some(curve) = edge.curve() {
            let (first, last) = if edge.orientation() == Orientation::Reversed {
                (edge.last(), edge.first())
            } else {
                (edge.first(), edge.last())
            };
            let samples = match curve {
                GeomCurve::Line(_) => 1,
                _ => 24,
            };
            for step in 0..samples {
                let parameter = first + (last - first) * step as f64 / samples as f64;
                points.push(to_uv(curve.point(parameter)));
            }
        } else {
            points.push(to_uv(edge.source().point()));
        }
    }
    (points.len() >= 3).then(|| {
        points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
            .map(|(&(ax, ay), &(bx, by))| ax * by - bx * ay)
            .sum::<f64>()
            * 0.5
    })
}

fn wire_has_only_circular_edges(wire: &Wire) -> bool {
    !wire.edges().is_empty()
        && wire
            .edges()
            .iter()
            .all(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_))))
}

/// Sweep a face and return the solid together with validation, recovery, and
/// complete generated-topology history.
pub fn prism_operation(face: &Face, vector: GeomVec) -> Result<OperationResult<Solid>, SweepError> {
    prism_operation_with_policy(face, vector, &TolerancePolicy::STANDARD)
}

/// Policy-aware canonical prism operation.
pub fn prism_operation_with_policy(
    face: &Face,
    vector: GeomVec,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, SweepError> {
    policy
        .validate()
        .map_err(SweepError::InvalidTolerancePolicy)?;
    let solid = build_prism(face, vector, policy, true)?;
    let validation = ValidationReport::for_solid(&solid, policy);
    let strict = solid.validate_strict_with_policy(policy).err();
    if !validation.is_valid() || strict.is_some() {
        return Err(SweepError::InvalidOutput {
            report: validation.health,
            watertight: validation.watertight,
            pcurves_complete: validation.pcurves_complete,
            strict,
        });
    }
    let recovery = RecoveryReport::default();
    let history = TopologyHistory::generated_solid(&solid);
    Ok(OperationResult {
        value: solid,
        history,
        diagnostics: Vec::new(),
        recovery,
        validation,
    })
}

/// [`prism_operation_with_policy`] without lateral-orientation reconciliation.
///
/// The rolling-ball blend rebuilds profile slices and then re-sews them into a
/// larger shell together with faces prism never sees; final orientation is
/// decided by that global sew, and locally reconciled walls break its winding
/// consistency (proven by the fillet-overflow volume regression). Standalone
/// prisms — every other caller — keep reconciliation, which is what makes a
/// swept disc enclose +V instead of a mixed-orientation shell.
pub(crate) fn prism_operation_with_policy_unreconciled(
    face: &Face,
    vector: GeomVec,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, SweepError> {
    policy
        .validate()
        .map_err(SweepError::InvalidTolerancePolicy)?;
    let solid = build_prism(face, vector, policy, false)?;
    let validation = ValidationReport::for_solid(&solid, policy);
    let strict = solid.validate_strict_with_policy(policy).err();
    if !validation.is_valid() || strict.is_some() {
        return Err(SweepError::InvalidOutput {
            report: validation.health,
            watertight: validation.watertight,
            pcurves_complete: validation.pcurves_complete,
            strict,
        });
    }
    let recovery = RecoveryReport::default();
    let history = TopologyHistory::generated_solid(&solid);
    Ok(OperationResult {
        value: solid,
        history,
        diagnostics: Vec::new(),
        recovery,
        validation,
    })
}

/// Compatibility prism wrapper.
///
/// This delegates to [`prism_operation`] and discards validation, recovery,
/// diagnostics, and topology history.
#[deprecated(note = "use prism_operation; this wrapper discards operation metadata")]
pub fn prism(face: &Face, vector: GeomVec) -> Result<Solid, SweepError> {
    prism_operation(face, vector).map(|result| result.value)
}

/// Alias matching the OpenCASCADE class name in user-facing docs.
#[inline]
#[deprecated(note = "use prism_operation; this wrapper discards operation metadata")]
pub fn sweep_prism(face: &Face, vector: GeomVec) -> Result<Solid, SweepError> {
    prism_operation(face, vector).map(|result| result.value)
}

/// True when `point` (on the profile plane) lies inside the profile face —
/// inside its outer loop and outside every hole. Curved edges are sampled;
/// the callers probe well clear of the boundary, so sampling accuracy is not
/// load-bearing.
fn profile_contains(face: &Face, point: Pnt) -> bool {
    let Some(GeomSurface::Plane(plane)) = face.surface() else {
        return false;
    };
    let frame = plane.position();
    let origin = frame.location();
    let to_uv = |p: &Pnt| -> (f64, f64) {
        let d = *p - origin;
        (
            d.dot(&GeomVec::from_dir(frame.x_direction())),
            d.dot(&GeomVec::from_dir(frame.y_direction())),
        )
    };
    let sample_wire = |wire: &Wire| -> Vec<(f64, f64)> {
        let mut polygon = Vec::new();
        for edge in wire.edges() {
            match edge.curve() {
                Some(curve) => {
                    const SAMPLES: usize = 24;
                    let (first, last) = if edge.orientation() == Orientation::Reversed {
                        (edge.last(), edge.first())
                    } else {
                        (edge.first(), edge.last())
                    };
                    for step in 0..SAMPLES {
                        let t = first + (last - first) * step as f64 / SAMPLES as f64;
                        polygon.push(to_uv(&curve.point(t)));
                    }
                }
                None => polygon.push(to_uv(&edge.source().point())),
            }
        }
        polygon
    };
    let uv = to_uv(&point);
    let Some(outer) = face.outer_wire() else {
        return false;
    };
    if !point_in_polygon_2d(uv, &sample_wire(&outer)) {
        return false;
    }
    !face
        .inner_wires()
        .iter()
        .any(|hole| point_in_polygon_2d(uv, &sample_wire(hole)))
}

fn lateral_face(
    edge: &Edge,
    translation: &Trsf,
    vector: GeomVec,
    profile: &Face,
    reconcile: bool,
) -> Result<Face, SweepError> {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let q0 = translation.transform_point(&p0);
    let q1 = translation.transform_point(&p1);

    let (source_parameter, target_parameter) = if edge.orientation() == Orientation::Reversed {
        (edge.last(), edge.first())
    } else {
        (edge.first(), edge.last())
    };
    let (source_tolerance, target_tolerance) =
        edge.curve()
            .map_or((edge.tolerance(), edge.tolerance()), |curve| {
                (
                    edge.tolerance()
                        .max(p0.distance(&curve.point(source_parameter))),
                    edge.tolerance()
                        .max(p1.distance(&curve.point(target_parameter))),
                )
            });

    let top_edge = edge.transformed(translation);
    let edges = [
        edge.reversed(),
        line_edge_with_tolerance(p0, q0, source_tolerance),
        top_edge.clone(),
        line_edge_with_tolerance(q1, p1, target_tolerance),
    ];
    let surface = lateral_surface(edge, p1, p0, vector, translation);

    // Reconcile a cylindrical wall's effective normal with the MATERIAL side.
    //
    // Which way a wall must face is not decided by which wire its edge came
    // from: an annulus hole (inner wire) and a fillet-clipped corner (outer
    // wire) both bound concave material and want the wall looking toward the
    // axis, while a disc boundary wants it looking away. The discriminator is
    // whether the profile has material just OUTSIDE the arc — probed with a
    // point nudged radially outward from the arc midpoint.
    //
    // Only cylinders are touched. Planar laterals were provably correct before
    // reconciliation existed, and ruled laterals carry hand-built coordinate
    // pcurves whose correspondence with the wire the blend solver relies on.
    //
    // The wire is reversed rather than the surface: flipping a cylinder's axis
    // does not survive `sew`, which re-canonicalizes it straight back.
    let reverse_loop = reconcile
        && match (&surface, edge.curve()) {
            (GeomSurface::Cylinder(_), Some(GeomCurve::Circle(circle))) => {
                let mid = circle.point(edge.first() + (edge.last() - edge.first()) * 0.5);
                match ((mid - circle.center()).normalized(), vector.normalized()) {
                    (Some(radial), Some(sweep)) => {
                        let radial = GeomVec::from_dir(radial);
                        let step = (circle.radius() * 1.0e-3).max(tolerance::CONFUSION * 10.0);
                        let material_outside = profile_contains(profile, mid + radial * step);

                        // Which way the built wall FACES is decided by the wire's
                        // actual traversal, not by `loop_agrees_with_surface`: the
                        // cylinder is constructed on the SWEEP axis, and
                        // `is_parallel` accepts an ANTIPARALLEL circle axis, so an
                        // arc wound about -sweep traverses oppositely inside the
                        // built frame. Compute the winding normal directly.
                        let mut sense = if edge.last() >= edge.first() {
                            1.0
                        } else {
                            -1.0
                        };
                        if edge.orientation() == Orientation::Reversed {
                            sense = -sense;
                        }
                        let traversal =
                            GeomVec::from_dir(circle.position().direction()).cross(&radial) * sense;
                        // The wire's first edge is `edge.reversed()`, so the
                        // boundary runs against the profile traversal.
                        let winding_normal = (traversal * -1.0).cross(&GeomVec::from_dir(sweep));
                        let faces_away = winding_normal.dot(&radial) > 0.0;

                        // Convex boundary (material inside the cylinder) wants the
                        // wall looking away from the axis; concave wants it looking
                        // toward the axis.
                        let wants_away = !material_outside;
                        faces_away != wants_away
                    }
                    _ => false,
                }
            }
            // A ruled surface uses `(u = curve parameter, v = sweep fraction)`.
            // The constructed boundary starts with the base edge reversed, so
            // an increasing profile traversal makes that UV loop clockwise.
            // Reverse the whole loop (and its pcurves below) to keep the curved
            // face winding aligned with the surface's intrinsic `dU x dV`
            // normal before sewing propagates the profile's material side.
            (GeomSurface::Ruled(_), _) => target_parameter > source_parameter,
            _ => false,
        };
    // Pcurves describe each 3D edge in its NATURAL first..last direction, not
    // the direction in which a particular loop traverses that edge.  The
    // distinction matters here because the base rail is deliberately used as
    // a reversed coedge, and orientation reconciliation can reverse all four
    // coedges again.  Encoding the rectangle in traversal order attached the
    // v=0 base pcurve to a v=1 top edge for some glyph spans after sewing; the
    // resulting deviation was exactly the extrusion depth.
    let ruled_pcurves = matches!(&surface, GeomSurface::Ruled(_)).then(|| {
        [
            (Pnt2d::new(edge.first(), 0.0), Pnt2d::new(edge.last(), 0.0)),
            (
                Pnt2d::new(source_parameter, 0.0),
                Pnt2d::new(source_parameter, 1.0),
            ),
            (Pnt2d::new(edge.first(), 1.0), Pnt2d::new(edge.last(), 1.0)),
            (
                Pnt2d::new(target_parameter, 1.0),
                Pnt2d::new(target_parameter, 0.0),
            ),
        ]
        .map(|(from, to)| uv_line(from, to, SurfacePeriodicity::NONE))
    });

    let (edges, pcurves): (Vec<_>, Vec<_>) = if let Some(ruled_pcurves) = ruled_pcurves {
        let pairs = edges.into_iter().zip(ruled_pcurves);
        if reverse_loop {
            pairs
                .rev()
                .map(|(edge, pcurve)| (edge.reversed(), pcurve))
                .unzip()
        } else {
            pairs.unzip()
        }
    } else {
        let edges = if reverse_loop {
            edges
                .into_iter()
                .rev()
                .map(|edge| edge.reversed())
                .collect::<Vec<_>>()
        } else {
            edges.to_vec()
        };
        let pcurves = edges
            .iter()
            .map(|edge| analytic_line_pcurve(&surface, edge))
            .collect();
        (edges, pcurves)
    };

    Face::with_pcurves(surface, Wire::from_edges(edges), pcurves).map_err(SweepError::FaceBuild)
}

fn cap_faces(
    face: &Face,
    translation: &Trsf,
    sweep_points_along_normal: bool,
    normalize_compound_inners: bool,
) -> Result<[Face; 2], SweepError> {
    let top = face.transformed(translation);
    let Some(GeomSurface::Plane(_)) = face.surface() else {
        return Ok(if sweep_points_along_normal {
            [face.reversed(), top]
        } else {
            [face.clone(), top.reversed()]
        });
    };

    let Some(normal) = effective_face_normal(face) else {
        return Ok(if sweep_points_along_normal {
            [face.reversed(), top]
        } else {
            [face.clone(), top.reversed()]
        });
    };

    // The cap that takes `normal.reversed()` faces *opposite* the source face, so
    // its loop — a verbatim copy of the source winding — must be reversed to stay
    // wound CCW about its own outward normal. Without this its winding disagrees
    // with its plane normal, and `sew`'s winding-based orientation propagation
    // resolves the conflict by flipping the cap's orientation flag — leaving an
    // inward-pointing *effective* normal (`orientation × plane normal`). That
    // inverted cap is invisible to the watertight/health checks but breaks every
    // consumer that trusts the stored normal: the rolling-ball fillet (wrong
    // bisector side) and the renderer (back-face-culled / mis-shaded top, the
    // "the top disappears" artifact). Reversing the winding keeps winding and
    // normal consistent, so `sew` leaves the cap outward — matching `make_box`.
    let caps = if sweep_points_along_normal {
        [
            planar_cap(
                face,
                normal.reversed(),
                None,
                true,
                normalize_compound_inners,
            )?,
            planar_cap(
                face,
                normal,
                Some(translation),
                false,
                normalize_compound_inners,
            )?,
        ]
    } else {
        [
            planar_cap(face, normal, None, false, normalize_compound_inners)?,
            planar_cap(
                face,
                normal.reversed(),
                Some(translation),
                true,
                normalize_compound_inners,
            )?,
        ]
    };
    Ok(caps)
}

/// Reverse a loop's winding: reverse every edge and their order, so the chain
/// stays contiguous but runs the other way (flipping the implied CCW normal).
fn reversed_wire(wire: &Wire) -> Wire {
    let mut edges: Vec<Edge> = wire.edges().iter().map(|e| e.reversed()).collect();
    edges.reverse();
    Wire::from_edges(edges)
}

fn planar_cap(
    face: &Face,
    normal: Dir,
    translation: Option<&Trsf>,
    flip_winding: bool,
    normalize_compound_inners: bool,
) -> Result<Face, SweepError> {
    let transform_wire = |wire: Wire| {
        let w = match translation {
            Some(t) => wire.transformed(t),
            None => wire,
        };
        if flip_winding {
            reversed_wire(&w)
        } else {
            w
        }
    };

    let outer = face.outer_wire().map(transform_wire);
    let outer_winding = outer
        .as_ref()
        .and_then(|wire| planar_wire_signed_area(face, wire));
    let inners = face
        .inner_wires()
        .into_iter()
        .map(transform_wire)
        .map(|wire| {
            if normalize_compound_inners
                && !wire_has_only_circular_edges(&wire)
                && outer_winding
                    .zip(planar_wire_signed_area(face, &wire))
                    .is_some_and(|(outer, inner)| outer * inner > 0.0)
            {
                reversed_wire(&wire)
            } else {
                wire
            }
        })
        .collect::<Vec<_>>();
    let point = outer
        .as_ref()
        .and_then(|wire| wire.edges().first().map(|edge| edge.source().point()))
        .unwrap_or(Pnt::origin());

    planar_face_with_pcurves(
        Plane::from_point_normal(point, normal),
        outer,
        inners,
        Orientation::Forward,
    )
    .map_err(SweepError::FaceBuild)
}

fn lateral_surface(
    edge: &Edge,
    p0: Pnt,
    p1: Pnt,
    vector: GeomVec,
    translation: &Trsf,
) -> GeomSurface {
    if let Some(curve) = edge.curve() {
        match curve {
            GeomCurve::Line(_) => {
                if let Some(plane) = plane_for_sweep(p0, p1, vector) {
                    return GeomSurface::plane(plane);
                }
            }
            GeomCurve::Circle(circle) => {
                if let Some(axis) = vector.normalized() {
                    // A circular arc swept along an axis parallel to the circle's
                    // own axis generates a cylinder (we build it on the exact sweep
                    // `axis`, not the fitted one). The arc's axis is fitted from
                    // possibly-f32 sketch samples, so it carries ~1e-7 rad of noise —
                    // comparing at 1e-8 wrongly rejects a genuinely axis-aligned
                    // extruded arc, leaving a `Ruled` wall that the rolling-ball
                    // fillet/trim can't blend (it errors `NotPlaneOrAnalytic` the
                    // moment that wall is a fillet's end cap). 1e-6 rad (~6e-5°) is
                    // still far tighter than any real obliquity.
                    if circle.axis().is_parallel(&axis, 1e-6) {
                        return GeomSurface::cylinder(CylindricalSurface::new(
                            Ax3::new_axes(circle.center(), axis, circle.position().x_direction()),
                            circle.radius(),
                        ));
                    }
                }
            }
            GeomCurve::BSpline(bspline)
                if bspline_poles_are_collinear(
                    bspline.poles(),
                    p0,
                    p1,
                    edge.tolerance().max(tolerance::CONFUSION),
                ) =>
            {
                if let Some(plane) = plane_for_sweep(p0, p1, vector) {
                    return GeomSurface::plane(plane);
                }
            }
            _ => {}
        }

        let top_curve = curve.transformed(translation);
        return GeomSurface::ruled(RuledSurface::new(curve.clone(), top_curve));
    }

    if let Some(plane) = plane_for_sweep(p0, p1, vector) {
        return GeomSurface::plane(plane);
    }

    let base = line_curve(p0, p1);
    let top = base.transformed(translation);
    GeomSurface::ruled(RuledSurface::new(base, top))
}

fn bspline_poles_are_collinear(poles: &[Pnt], start: Pnt, end: Pnt, tolerance: f64) -> bool {
    let chord = end - start;
    let length = chord.magnitude();
    length > tolerance
        && poles
            .iter()
            .all(|pole| ((*pole - start).cross(&chord)).magnitude() / length <= tolerance)
}

fn plane_for_sweep(p0: Pnt, p1: Pnt, vector: GeomVec) -> Option<Plane> {
    let tangent = p1 - p0;
    let normal = tangent.cross(&vector).normalized()?;
    Some(Plane::from_point_normal(p0, normal))
}

fn line_curve(p0: Pnt, p1: Pnt) -> GeomCurve {
    let tangent = p1 - p0;
    let dir = tangent.normalized().unwrap_or(Dir::dx());
    GeomCurve::line(Line::from_point_dir(p0, dir))
}

fn line_edge_with_tolerance(p0: Pnt, p1: Pnt, edge_tolerance: f64) -> Edge {
    let length = p0.distance(&p1);
    let start = openrcad_topo::Vertex::new(p0);
    let end = openrcad_topo::Vertex::new(p1);
    if length <= tolerance::CONFUSION {
        return Edge::new_with_tolerance(
            None,
            0.0,
            0.0,
            start,
            end,
            edge_tolerance.max(tolerance::CONFUSION),
        );
    }
    Edge::new_with_tolerance(
        Some(line_curve(p0, p1)),
        0.0,
        length,
        start,
        end,
        edge_tolerance.max(tolerance::CONFUSION),
    )
}

fn effective_face_normal(face: &Face) -> Option<Dir> {
    let mut normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        _ => newell_normal(face)?,
    };
    if face.orientation() == Orientation::Reversed {
        normal = normal.reversed();
    }
    Some(normal)
}

fn newell_normal(face: &Face) -> Option<Dir> {
    let wire = face.outer_wire()?;
    let edges = wire.edges();
    if edges.len() < 3 {
        return None;
    }

    let points: Vec<Pnt> = edges.iter().map(|edge| edge.source().point()).collect();
    let mut n = GeomVec::ZERO;
    for i in 0..points.len() {
        let p = points[i];
        let q = points[(i + 1) % points.len()];
        n += GeomVec::new(
            (p.y() - q.y()) * (p.z() + q.z()),
            (p.z() - q.z()) * (p.x() + q.x()),
            (p.x() - q.x()) * (p.y() + q.y()),
        );
    }
    n.normalized()
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::TAU;
    use openrcad_foundation::Ax3;
    use openrcad_geom::{BSplineCurve, Circle, Surface};
    use openrcad_topo::Vertex;

    fn square_face_with_hole() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(4.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let inner = Wire::from_edges([
            Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(1.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 3.0, 0.0), Pnt::new(3.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 3.0, 0.0), Pnt::new(3.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 1.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
        ]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![inner],
            Orientation::Forward,
        )
    }

    fn square_face_with_same_winding_hole() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(4.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        // CCW, intentionally the same traversal as the outer loop. Being in
        // `inner_wires` still makes this a topological hole.
        let inner = Wire::from_edges([
            Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(3.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 1.0, 0.0), Pnt::new(3.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(3.0, 3.0, 0.0), Pnt::new(1.0, 3.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 3.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
        ]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![inner],
            Orientation::Forward,
        )
    }

    fn square_face_with_same_winding_bspline_hole() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(4.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let edge = |start: Pnt, end: Pnt| {
            quadratic_edge_with_tolerance(
                start,
                Pnt::new(
                    (start.x() + end.x()) * 0.5,
                    (start.y() + end.y()) * 0.5,
                    0.0,
                ),
                end,
                start,
                end,
                tolerance::CONFUSION,
            )
        };
        let p0 = Pnt::new(1.0, 1.0, 0.0);
        let p1 = Pnt::new(3.0, 1.0, 0.0);
        let p2 = Pnt::new(3.0, 3.0, 0.0);
        let p3 = Pnt::new(1.0, 3.0, 0.0);
        let inner = Wire::from_edges([edge(p0, p1), edge(p1, p2), edge(p2, p3), edge(p3, p0)]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![inner],
            Orientation::Forward,
        )
    }

    fn rectangle_face_with_two_same_winding_bspline_holes() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(6.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(6.0, 0.0, 0.0), Pnt::new(6.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(6.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let hole = |x0: f64, x1: f64| {
            let edge = |start: Pnt, end: Pnt| {
                quadratic_edge_with_tolerance(
                    start,
                    Pnt::new(
                        (start.x() + end.x()) * 0.5,
                        (start.y() + end.y()) * 0.5,
                        0.0,
                    ),
                    end,
                    start,
                    end,
                    tolerance::CONFUSION,
                )
            };
            let p0 = Pnt::new(x0, 1.0, 0.0);
            let p1 = Pnt::new(x1, 1.0, 0.0);
            let p2 = Pnt::new(x1, 2.0, 0.0);
            let p3 = Pnt::new(x0, 2.0, 0.0);
            Wire::from_edges([edge(p0, p1), edge(p1, p2), edge(p2, p3), edge(p3, p0)])
        };
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![hole(1.0, 2.0), hole(4.0, 5.0)],
            Orientation::Forward,
        )
    }

    fn square_face_with_mixed_same_winding_hole() -> Face {
        let outer = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(4.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(4.0, 4.0, 0.0), Pnt::new(0.0, 4.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 4.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let spline = |start: Pnt, end: Pnt| {
            quadratic_edge_with_tolerance(
                start,
                Pnt::new(
                    (start.x() + end.x()) * 0.5,
                    (start.y() + end.y()) * 0.5,
                    0.0,
                ),
                end,
                start,
                end,
                tolerance::CONFUSION,
            )
        };
        let p0 = Pnt::new(1.0, 1.0, 0.0);
        let p1 = Pnt::new(3.0, 1.0, 0.0);
        let p2 = Pnt::new(3.0, 3.0, 0.0);
        let p3 = Pnt::new(1.0, 3.0, 0.0);
        let inner = Wire::from_edges([
            Edge::between_points(p0, p1),
            spline(p1, p2),
            Edge::between_points(p2, p3),
            spline(p3, p0),
        ]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(outer),
            vec![inner],
            Orientation::Forward,
        )
    }

    fn quadratic_edge_with_tolerance(
        curve_start: Pnt,
        control: Pnt,
        curve_end: Pnt,
        vertex_start: Pnt,
        vertex_end: Pnt,
        tolerance: f64,
    ) -> Edge {
        let curve = BSplineCurve::new(
            2,
            vec![curve_start, control, curve_end],
            None,
            vec![0.0, 1.0],
            vec![3, 3],
        );
        Edge::new_with_tolerance(
            Some(GeomCurve::bspline(curve)),
            0.0,
            1.0,
            Vertex::new(vertex_start),
            Vertex::new(vertex_end),
            tolerance,
        )
    }

    fn assert_face_pcurves_follow_natural_edges(face: &Face, tolerance: f64) {
        let surface = face.surface().expect("lateral face has a surface");
        for wire in face.wires() {
            for (index, edge) in wire.edges().iter().enumerate() {
                let curve = edge.curve().expect("lateral edge has a curve");
                let pcurve = wire.pcurve(index).expect("lateral coedge has a pcurve");
                for sample in 0..=16 {
                    let fraction = f64::from(sample) / 16.0;
                    let uv = pcurve.point_at_fraction(fraction);
                    let lifted = surface.point(uv.x(), uv.y());
                    let parameter = edge.first() + (edge.last() - edge.first()) * fraction;
                    let deviation = lifted.distance(&curve.point(parameter));
                    assert!(
                        deviation <= tolerance,
                        "coedge {index} pcurve follows the wrong natural edge direction/rail: deviation {deviation:e}"
                    );
                }
            }
        }
    }

    #[test]
    fn ruled_lateral_pcurves_follow_natural_edges_after_loop_reversal() {
        let start = Pnt::new(0.0, 0.0, 0.0);
        let end = Pnt::new(4.0, 0.0, 0.0);
        let edge = quadratic_edge_with_tolerance(
            start,
            Pnt::new(2.0, -1.0, 0.0),
            end,
            start,
            end,
            tolerance::CONFUSION,
        );
        let profile = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
            ]),
        );
        let vector = GeomVec::new(0.0, 0.0, 25.0);
        let translation = Trsf::translation(vector);

        // The forward edge triggers ruled-loop reconciliation; the reversed
        // use covers glyph spans whose contour traversal opposes their stored
        // spline parameterization.
        for edge_use in [edge.clone(), edge.reversed()] {
            let lateral = lateral_face(&edge_use, &translation, vector, &profile, true)
                .expect("ruled lateral");
            assert_face_pcurves_follow_natural_edges(&lateral, 1.0e-9);
        }
    }

    #[test]
    fn tolerated_bspline_junctions_keep_prism_pcurves_consistent() {
        // Sketch coordinates arrive through f32 and adjacent glyph spans can
        // evaluate a shared endpoint a little differently. The topology welds
        // those evaluations to one vertex and records the residual as the edge
        // tolerance. Prism must carry that uncertainty onto its sweep
        // connectors; otherwise a valid analytic glyph is rejected because a
        // ruled surface lifts the connector pcurve to the curve endpoint while
        // the connector itself starts at the welded vertex.
        let delta = 1.25e-6;
        let edge_tolerance = 3.0e-6;
        let p0 = Pnt::new(0.0, 0.0, 0.0);
        let p1 = Pnt::new(4.0, 0.0, 0.0);
        let p2 = Pnt::new(4.0, 3.0, 0.0);
        let p3 = Pnt::new(0.0, 3.0, 0.0);
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                quadratic_edge_with_tolerance(
                    Pnt::new(0.0, delta, 0.0),
                    Pnt::new(2.0, -0.5, 0.0),
                    Pnt::new(4.0, delta, 0.0),
                    p0,
                    p1,
                    edge_tolerance,
                ),
                quadratic_edge_with_tolerance(
                    Pnt::new(4.0 - delta, 0.0, 0.0),
                    Pnt::new(4.5, 1.5, 0.0),
                    Pnt::new(4.0 - delta, 3.0, 0.0),
                    p1,
                    p2,
                    edge_tolerance,
                ),
                quadratic_edge_with_tolerance(
                    Pnt::new(4.0, 3.0 - delta, 0.0),
                    Pnt::new(2.0, 3.5, 0.0),
                    Pnt::new(0.0, 3.0 - delta, 0.0),
                    p2,
                    p3,
                    edge_tolerance,
                ),
                quadratic_edge_with_tolerance(
                    Pnt::new(delta, 3.0, 0.0),
                    Pnt::new(-0.5, 1.5, 0.0),
                    Pnt::new(delta, 0.0, 0.0),
                    p3,
                    p0,
                    edge_tolerance,
                ),
            ]),
        );

        let operation = prism_operation(&face, GeomVec::new(0.0, 0.0, 2.0))
            .expect("tolerated analytic glyph spans should extrude without a sampled fallback");
        assert!(operation.value.is_watertight());
        assert!(operation.value.has_complete_pcurves());
        assert!(operation
            .value
            .validate_strict_with_policy(&TolerancePolicy::STANDARD)
            .is_ok());
    }

    #[test]
    fn extrudes_triangle_to_watertight_prism() {
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(2.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(2.0, 0.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
            ]),
        );

        let operation = prism_operation(&face, GeomVec::new(0.0, 0.0, 3.0)).unwrap();
        assert!(
            operation.recovery.actions.is_empty(),
            "native prism construction must not reconstruct pcurves"
        );
        assert!(operation.value.has_complete_pcurves());
        let solid = operation.value;
        assert_eq!(solid.vertex_count(), 6);
        assert_eq!(solid.edge_count(), 9);
        assert_eq!(solid.face_count(), 5);
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
    }

    #[test]
    fn prism_caps_are_oriented_outward() {
        // Regression: a swept prism must come out with every face's *effective*
        // normal (orientation × plane normal) pointing away from the solid centre,
        // like `make_box`. A cap left winding-inconsistent gets its orientation
        // flag flipped by `sew`, yielding an inward effective normal — invisible to
        // the watertight check but fatal to the rolling-ball fillet and to
        // back-face culling (the "extruded box's top disappears" bug).
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(4.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(4.0, 0.0, 0.0), Pnt::new(4.0, 3.0, 0.0)),
                Edge::between_points(Pnt::new(4.0, 3.0, 0.0), Pnt::new(0.0, 3.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 3.0, 0.0), Pnt::origin()),
            ]),
        );
        let solid = prism(&face, GeomVec::new(0.0, 0.0, 2.0)).unwrap();

        // Centroid of the box (1,1.5,1) lies inside; every effective face normal
        // must point away from it.
        let centre = Pnt::new(2.0, 1.5, 1.0);
        for f in solid.shell().faces() {
            let n = effective_face_normal(&f).expect("planar cap/wall has a normal");
            // A representative point on the face: its outer-loop vertex average.
            let pts: Vec<Pnt> = f
                .outer_wire()
                .unwrap()
                .edges()
                .iter()
                .map(|e| e.source().point())
                .collect();
            let k = pts.len() as f64;
            let c = Pnt::new(
                pts.iter().map(|p| p.x()).sum::<f64>() / k,
                pts.iter().map(|p| p.y()).sum::<f64>() / k,
                pts.iter().map(|p| p.z()).sum::<f64>() / k,
            );
            let outward = (c - centre).dot(&GeomVec::from_dir(n));
            assert!(
                outward > 0.0,
                "face normal points inward (dot={outward}) — cap orientation regressed"
            );
        }
    }

    #[test]
    fn extrudes_face_with_hole() {
        let solid = prism(&square_face_with_hole(), GeomVec::new(0.0, 0.0, 2.0)).unwrap();
        assert_eq!(solid.face_count(), 10);
        assert!(solid.is_watertight());
        assert!(solid.health_report().is_healthy());
        let expected = (4.0 * 4.0 - 2.0 * 2.0) * 2.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= 1.0e-6,
            "a swept inner wire must remove volume: expected {expected}, got {volume}"
        );
    }

    #[test]
    fn inner_wire_role_removes_volume_regardless_of_input_winding() {
        let solid = prism(
            &square_face_with_same_winding_hole(),
            GeomVec::new(0.0, 0.0, 2.0),
        )
        .expect("same-winding inner wire");
        let expected = (4.0 * 4.0 - 2.0 * 2.0) * 2.0;
        let volume = signed_volume(&solid);
        assert!(solid.is_watertight());
        assert!(solid
            .validate_strict_with_policy(&TolerancePolicy::STANDARD)
            .is_ok());
        assert!(
            (volume - expected).abs() <= 1.0e-6,
            "inner-wire topology must remove volume: expected {expected}, got {volume}"
        );
    }

    #[test]
    fn bspline_inner_wire_removes_volume_from_a_line_bounded_profile() {
        let solid = prism(
            &square_face_with_same_winding_bspline_hole(),
            GeomVec::new(0.0, 0.0, 2.0),
        )
        .expect("same-winding B-spline inner wire");
        let expected = (4.0 * 4.0 - 2.0 * 2.0) * 2.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= 1.0e-6,
            "B-spline inner wire must remove volume: expected {expected}, got {volume}"
        );
    }

    #[test]
    fn multiple_bspline_inner_wires_all_remove_volume() {
        let solid = prism(
            &rectangle_face_with_two_same_winding_bspline_holes(),
            GeomVec::new(0.0, 0.0, 2.0),
        )
        .expect("two same-winding B-spline inner wires");
        let expected = (6.0 * 4.0 - 2.0) * 2.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= 1.0e-6,
            "every B-spline inner wire must remove volume: expected {expected}, got {volume}"
        );
    }

    #[test]
    fn mixed_line_bspline_inner_wire_removes_volume() {
        let solid = prism(
            &square_face_with_mixed_same_winding_hole(),
            GeomVec::new(0.0, 0.0, 2.0),
        )
        .expect("mixed same-winding inner wire");
        let expected = (4.0 * 4.0 - 2.0 * 2.0) * 2.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= 1.0e-6,
            "mixed inner wire must remove volume: expected {expected}, got {volume}"
        );
    }

    /// Signed volume straight off the tessellation. Deliberately NOT absolute:
    /// mass properties and every downstream helper normalize the sign, which is
    /// exactly what let inward-facing laterals go unnoticed.
    fn signed_volume(solid: &Solid) -> f64 {
        let mesh = openrcad_mesh::tessellate(solid, 0.02, 0.2);
        let mut six = 0.0;
        for [a, b, c] in &mesh.triangles {
            let (p, q, r) = (
                mesh.vertices[*a as usize],
                mesh.vertices[*b as usize],
                mesh.vertices[*c as usize],
            );
            six += p.x() * (q.y() * r.z() - q.z() * r.y())
                - p.y() * (q.x() * r.z() - q.z() * r.x())
                + p.z() * (q.x() * r.y() - q.y() * r.x());
        }
        six / 6.0
    }

    fn disc_face(radius: f64) -> Face {
        let circle = Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), radius);
        let edges: Vec<Edge> = [0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU]
            .windows(2)
            .map(|w| {
                Edge::new(
                    Some(GeomCurve::circle(circle)),
                    w[0],
                    w[1],
                    Vertex::new(circle.point(w[0])),
                    Vertex::new(circle.point(w[1])),
                )
            })
            .collect();
        Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges(edges),
        )
    }

    /// Cylindrical laterals must face outward, so a swept disc encloses a
    /// POSITIVE signed volume. Watertight and health checks pass either way —
    /// they cannot see an inverted effective normal.
    #[test]
    fn cylindrical_laterals_face_outward_for_a_positive_sweep() {
        let solid = prism(&disc_face(2.0), GeomVec::new(0.0, 0.0, 5.0)).unwrap();
        let expected = core::f64::consts::PI * 4.0 * 5.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= expected * 0.02,
            "swept disc should enclose +{expected:.2}, got {volume:.2}"
        );
    }

    /// Same solid swept the other way: the body sits below z=0 but its volume is
    /// still positive, so the sign test is about orientation, not sweep direction.
    #[test]
    fn cylindrical_laterals_face_outward_for_a_negative_sweep() {
        let solid = prism(&disc_face(2.0), GeomVec::new(0.0, 0.0, -5.0)).unwrap();
        let expected = core::f64::consts::PI * 4.0 * 5.0;
        let volume = signed_volume(&solid);
        assert!(
            (volume - expected).abs() <= expected * 0.02,
            "reverse-swept disc should enclose +{expected:.2}, got {volume:.2}"
        );
    }

    #[test]
    fn circular_boundary_generates_cylindrical_laterals() {
        let circle = Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        let edges: Vec<Edge> = [0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU]
            .windows(2)
            .map(|w| {
                Edge::new(
                    Some(GeomCurve::circle(circle)),
                    w[0],
                    w[1],
                    Vertex::new(circle.point(w[0])),
                    Vertex::new(circle.point(w[1])),
                )
            })
            .collect();
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges(edges),
        );

        let solid = prism(&face, GeomVec::new(0.0, 0.0, 5.0)).unwrap();
        assert_eq!(solid.vertex_count(), 6);
        assert_eq!(solid.edge_count(), 9);
        assert_eq!(solid.face_count(), 5);
        let cylinders = solid
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 3);
        assert!(solid.is_watertight());
    }

    /// An axis-aligned circular arc reconstructed from (f32) sketch samples carries
    /// a tiny axis tilt. The lateral it sweeps must still be classified as a
    /// CYLINDER, not a Ruled wall — otherwise a fillet whose end runs into that
    /// wall fails with `NotPlaneOrAnalytic`. Regression for `fillet_problem.zcad`.
    #[test]
    fn slightly_tilted_circle_axis_still_makes_a_cylinder() {
        // Axis tilted ~1e-7 rad off +z (well past the old 1e-8 gate, far under 1e-6).
        let tilted = Dir::new(1.0e-7, 0.0, 1.0);
        let circle = Circle::new(Ax3::new(Pnt::origin(), tilted), 2.0);
        let edges: Vec<Edge> = [0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU]
            .windows(2)
            .map(|w| {
                Edge::new(
                    Some(GeomCurve::circle(circle)),
                    w[0],
                    w[1],
                    Vertex::new(circle.point(w[0])),
                    Vertex::new(circle.point(w[1])),
                )
            })
            .collect();
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                tilted,
            ))),
            Wire::from_edges(edges),
        );
        let solid = prism(&face, GeomVec::new(0.0, 0.0, 5.0)).unwrap();
        let cylinders = solid
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(
            cylinders, 3,
            "a near-axis-aligned arc must sweep to cylinders, not Ruled walls"
        );
    }
}
