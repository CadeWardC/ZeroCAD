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
    Edge, Face, FaceBuildError, HealthReport, OperationResult, Orientation, RecoveryReport, Solid,
    SurfacePeriodicity, TopologyHistory, ValidationReport, Wire,
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

    faces.extend(cap_faces(face, &translation, sweep_points_along_normal)?);

    // Outer and inner walls need OPPOSITE effective normals: an outer wall faces
    // away from its own axis, a hole's wall faces toward it, into the void. Both
    // are "outward" with respect to the material, so the two cases cannot share
    // one reconciliation rule.
    for edge in outer.edges() {
        if edge.length() <= tolerance::CONFUSION {
            continue;
        }
        faces.push(lateral_face(&edge, &translation, vector, false)?);
    }
    for wire in face.inner_wires() {
        for edge in wire.edges() {
            if edge.length() <= tolerance::CONFUSION {
                continue;
            }
            faces.push(lateral_face(&edge, &translation, vector, true)?);
        }
    }

    Ok(Solid::new(
        sew_with_policy(&faces, policy).map_err(SweepError::InvalidTolerancePolicy)?,
    ))
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
    let solid = build_prism(face, vector, policy)?;
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

fn lateral_face(
    edge: &Edge,
    translation: &Trsf,
    vector: GeomVec,
    bounds_a_hole: bool,
) -> Result<Face, SweepError> {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let q0 = translation.transform_point(&p0);
    let q1 = translation.transform_point(&p1);

    let top_edge = edge.transformed(translation);
    let edges = [
        edge.reversed(),
        Edge::between_points(p0, q0),
        top_edge.clone(),
        Edge::between_points(q1, p1),
    ];
    let surface = lateral_surface(edge, p1, p0, vector, translation);

    // Reconcile the wall's intrinsic normal with the material side.
    //
    // `cap_faces` branches on whether the sweep runs along the profile normal;
    // the laterals had no equivalent, so a sweep in the +normal direction left
    // walls whose effective normal pointed INWARD. Watertight, health and
    // closed-mesh checks all pass on such a solid — only its signed volume
    // shows it, and mass properties normalize that sign away.
    //
    // The wire is reversed rather than the surface: flipping a cylinder's axis
    // does not survive `sew`, which re-canonicalizes it straight back.
    let mid_base = edge.curve().map_or_else(
        || p0 + (p1 - p0) * 0.5,
        |curve| curve.point(edge.first() + (edge.last() - edge.first()) * 0.5),
    );
    let center = mid_base + vector * 0.5;
    let agrees = crate::revolve::loop_agrees_with_surface(
        &Wire::from_edges(edges.clone()),
        &surface,
        center,
    );
    // A hole's wall wants the inverse: material lies outside that cylinder, so
    // its face must look toward the axis.
    let mut reverse_loop = if bounds_a_hole { agrees } else { !agrees };
    if matches!(&surface, GeomSurface::Ruled(_)) {
        reverse_loop = false;
    }
    let edges: Vec<Edge> = if reverse_loop {
        edges
            .into_iter()
            .rev()
            .map(|edge| edge.reversed())
            .collect()
    } else {
        edges.to_vec()
    };

    let pcurves = if matches!(&surface, GeomSurface::Ruled(_)) {
        // A ruled lateral's pcurves are the four known coordinate lines of its
        // (u = profile parameter, v = sweep fraction) rectangle.
        let corners = [
            (Pnt2d::new(edge.first(), 0.0), Pnt2d::new(edge.last(), 0.0)),
            (Pnt2d::new(edge.first(), 0.0), Pnt2d::new(edge.first(), 1.0)),
            (Pnt2d::new(edge.first(), 1.0), Pnt2d::new(edge.last(), 1.0)),
            (Pnt2d::new(edge.last(), 1.0), Pnt2d::new(edge.last(), 0.0)),
        ];
        corners
            .into_iter()
            .map(|(from, to)| uv_line(from, to, SurfacePeriodicity::NONE))
            .collect()
    } else {
        edges
            .iter()
            .map(|edge| analytic_line_pcurve(&surface, edge))
            .collect()
    };

    Face::with_pcurves(surface, Wire::from_edges(edges), pcurves).map_err(SweepError::FaceBuild)
}

fn cap_faces(
    face: &Face,
    translation: &Trsf,
    sweep_points_along_normal: bool,
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
            planar_cap(face, normal.reversed(), None, true)?,
            planar_cap(face, normal, Some(translation), false)?,
        ]
    } else {
        [
            planar_cap(face, normal, None, false)?,
            planar_cap(face, normal.reversed(), Some(translation), true)?,
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
    let inners = face
        .inner_wires()
        .into_iter()
        .map(transform_wire)
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
    use openrcad_geom::Circle;
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
