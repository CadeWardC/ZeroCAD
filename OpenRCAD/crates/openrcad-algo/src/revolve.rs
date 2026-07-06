//! Solids of revolution: sweep a planar profile face around an axis lying in
//! its plane.
//!
//! The structural sibling of [`prism`](crate::prism): one lateral face per
//! profile boundary edge, planar caps at the start/end angles (none for a full
//! turn), sewn watertight. Every lateral surface is ANALYTIC — the revolved
//! image of a profile line is a cylinder / plane / cone (parallel /
//! perpendicular / skew to the axis) and of a profile arc a torus or sphere —
//! so downstream booleans, fillets, and tessellation see first-class surfaces,
//! never sampled approximations.
//!
//! A full 2π revolution is emitted as TWO half-turn lateral faces per profile
//! edge (plus no caps): a single closed face would need a seam edge, and two
//! halves keep every loop an ordinary contractible quad, which `sew` and the
//! mesh layer already handle.

use core::f64::consts::{PI, TAU};
use core::fmt;

use openrcad_foundation::{tolerance, Ax1, Ax3, Dir, Pnt, Trsf, Vec as GeomVec};
use openrcad_geom::{
    Circle, ConicalSurface, Curve, CylindricalSurface, GeomCurve, GeomSurface, Plane,
    SphericalSurface, ToroidalSurface,
};
use openrcad_topo::{Edge, Face, Orientation, Solid, Vertex, Wire};

use crate::sew::sew;

/// Errors reported by [`revolve`].
#[derive(Clone, Debug, PartialEq)]
pub enum RevolveError {
    /// The angle is not in `(0, 2π]`.
    InvalidAngle,
    /// The source face has no outer boundary.
    MissingOuterWire,
    /// A boundary wire is not closed.
    OpenWire,
    /// The profile face is not planar (only planar profiles can revolve).
    ProfileNotPlanar,
    /// The revolution axis does not lie in the profile's plane.
    AxisNotInProfilePlane,
    /// The profile has material on both sides of the axis.
    ProfileCrossesAxis,
    /// A profile boundary curve has no analytic surface of revolution
    /// (ellipse/B-spline profiles are not supported yet).
    UnsupportedProfileCurve,
}

impl fmt::Display for RevolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAngle => f.write_str("revolve: angle must be in (0, 2π]"),
            Self::MissingOuterWire => f.write_str("revolve: source face has no outer wire"),
            Self::OpenWire => f.write_str("revolve: every profile wire must be closed"),
            Self::ProfileNotPlanar => f.write_str("revolve: profile face must be planar"),
            Self::AxisNotInProfilePlane => {
                f.write_str("revolve: the axis must lie in the profile's plane")
            }
            Self::ProfileCrossesAxis => {
                f.write_str("revolve: profile must lie entirely on one side of the axis")
            }
            Self::UnsupportedProfileCurve => f.write_str(
                "revolve: only line and circular-arc profile edges are supported (no splines yet)",
            ),
        }
    }
}

impl std::error::Error for RevolveError {}

/// Revolve `face` by `angle` radians about the axis through `axis_point`
/// along `axis_dir`, and return a sewn solid.
pub fn revolve(
    face: &Face,
    axis_point: Pnt,
    axis_dir: Dir,
    angle: f64,
) -> Result<Solid, RevolveError> {
    if !(angle > tolerance::CONFUSION && angle <= TAU + 1e-9) {
        return Err(RevolveError::InvalidAngle);
    }
    let full = (angle - TAU).abs() < 1e-9;
    let Some(outer) = face.outer_wire() else {
        return Err(RevolveError::MissingOuterWire);
    };
    if !outer.is_closed() {
        return Err(RevolveError::OpenWire);
    }
    for wire in face.inner_wires() {
        if !wire.is_closed() {
            return Err(RevolveError::OpenWire);
        }
    }
    let Some(GeomSurface::Plane(plane)) = face.surface() else {
        return Err(RevolveError::ProfileNotPlanar);
    };
    let n = plane.normal();
    // The axis must lie in the profile plane: perpendicular to the normal, and
    // through a point of the plane.
    if axis_dir.dot(&n).abs() > 1e-6 {
        return Err(RevolveError::AxisNotInProfilePlane);
    }
    let off = (axis_point - plane_point(plane)).dot(&GeomVec::from_dir(n));
    if off.abs() > tolerance::CONFUSION * 100.0 {
        return Err(RevolveError::AxisNotInProfilePlane);
    }

    // Signed radial coordinate in the profile plane: positive along
    // `axis × n`. Every profile vertex must sit on one side (touching allowed).
    let rad_ref = axis_dir.cross(&n);
    let signed_r = |p: Pnt| (p - axis_point).dot(&GeomVec::from_dir(rad_ref));
    let mut side = 0.0f64;
    for wire in face.wires() {
        for edge in wire.edges() {
            for p in [edge.source().point(), edge.target().point()] {
                let s = signed_r(p);
                if s.abs() <= tolerance::CONFUSION * 100.0 {
                    continue;
                }
                if side == 0.0 {
                    side = s.signum();
                } else if s.signum() != side {
                    return Err(RevolveError::ProfileCrossesAxis);
                }
            }
        }
    }
    if side == 0.0 {
        // Degenerate: the whole profile lies on the axis.
        return Err(RevolveError::ProfileCrossesAxis);
    }

    let axis = Ax1::new(axis_point, axis_dir);

    // Orientation is decided by the WINDING normal (Newell over the outer
    // wire), not the stored plane normal — a sketch-built profile can carry
    // either handedness, and the lateral loops inherit the wire's direction.
    // `along` = does the sweep velocity at the widest profile point agree with
    // the winding normal: exactly prism's `sweep_points_along_normal`, and it
    // decides both the lateral loop direction and which cap flips.
    let n_wind = newell_normal(&outer).unwrap_or(n);
    let probe = widest_point(face, axis_point, axis_dir);
    let radial = probe - closest_axis_point(probe, axis_point, axis_dir);
    let sweep_vel = GeomVec::from_dir(axis_dir).cross(&radial);
    let along = sweep_vel.dot(&GeomVec::from_dir(n_wind)) >= 0.0;

    // Angular stations: ≤ π per lateral face so no loop needs an internal seam.
    // A FULL turn uses THREE 120° segments — matching `make_cylinder`'s thirds
    // so a revolved cylinder tessellates identically to a primitive one. (Two
    // half-turn arcs would be topologically valid now that arena edge identity
    // includes the arc span — see `edge_midpoints_match` — but thirds keep the
    // mesh density uniform with the rest of the kernel's round geometry.)
    let stations: Vec<f64> = if full {
        vec![0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU]
    } else if angle > PI + 1e-9 {
        vec![0.0, angle / 2.0, angle]
    } else {
        vec![0.0, angle]
    };

    let mut faces = Vec::new();
    for wire in face.wires() {
        for edge in wire.edges() {
            if edge.length() <= tolerance::CONFUSION {
                continue;
            }
            // An edge lying ON the axis sweeps a degenerate (zero-radius)
            // surface — it contributes nothing; the neighbouring laterals
            // close the solid there (e.g. the axis side of a cylinder's
            // rectangle profile).
            let r_src = (edge.source().point()
                - closest_axis_point(edge.source().point(), axis_point, axis_dir))
            .magnitude();
            let r_tgt = (edge.target().point()
                - closest_axis_point(edge.target().point(), axis_point, axis_dir))
            .magnitude();
            let on_axis_tol = tolerance::CONFUSION * 100.0;
            if r_src <= on_axis_tol && r_tgt <= on_axis_tol {
                continue;
            }
            let surface = lateral_surface(&edge, axis_point, axis_dir)?;
            for w in stations.windows(2) {
                faces.push(lateral_face(&edge, &axis, w[0], w[1], surface.clone()));
            }
        }
    }

    if !full {
        // Caps at the start and end angles, oriented off the winding normal
        // (prism's cap-orientation pattern: the backward-facing cap reverses
        // BOTH its normal and its loop winding before sew).
        let rot_end = Trsf::rotation(&axis, angle);
        let n_end = rotate_dir(n_wind, axis_dir, angle);
        if along {
            faces.push(revolve_cap(face, n_wind.reversed(), None, true));
            faces.push(revolve_cap(face, n_end, Some(&rot_end), false));
        } else {
            faces.push(revolve_cap(face, n_wind, None, false));
            faces.push(revolve_cap(face, n_end.reversed(), Some(&rot_end), true));
        }
    }

    Ok(Solid::new(sew(&faces, tolerance::CONFUSION * 10.0)))
}

fn plane_point(plane: &Plane) -> Pnt {
    plane.position().location()
}

/// The profile vertex farthest from the axis — a safe probe for the sweep
/// direction (the first vertex might sit ON the axis, where the velocity is
/// zero and says nothing).
fn widest_point(face: &Face, axis_point: Pnt, axis_dir: Dir) -> Pnt {
    let mut best = Pnt::origin();
    let mut best_r = -1.0f64;
    for wire in face.wires() {
        for edge in wire.edges() {
            for p in [edge.source().point(), edge.target().point()] {
                let r = (p - closest_axis_point(p, axis_point, axis_dir)).magnitude();
                if r > best_r {
                    best_r = r;
                    best = p;
                }
            }
        }
    }
    best
}

/// Newell normal of a wire's polygon (the true WINDING normal — CCW about it).
fn newell_normal(wire: &Wire) -> Option<Dir> {
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

fn closest_axis_point(p: Pnt, axis_point: Pnt, axis_dir: Dir) -> Pnt {
    let d = GeomVec::from_dir(axis_dir);
    let t = (p - axis_point).dot(&d);
    axis_point + d * t
}

fn rotate_dir(d: Dir, axis: Dir, angle: f64) -> Dir {
    let v = GeomVec::from_dir(d);
    let k = GeomVec::from_dir(axis);
    let (s, c) = angle.sin_cos();
    let r = v * c + k.cross(&v) * s + k * (k.dot(&v) * (1.0 - c));
    r.normalized().unwrap_or(d)
}

/// The arc a profile point traces from station `t0` to `t1`, as a circle edge
/// about the axis. `None` for a point on the axis (no arc — a pole).
fn revolved_arc(p: Pnt, axis: &Ax1, axis_dir: Dir, t0: f64, t1: f64) -> Option<Edge> {
    let center = closest_axis_point(p, axis.location(), axis_dir);
    let radial = p - center;
    let r = radial.magnitude();
    if r <= tolerance::CONFUSION {
        return None;
    }
    let rot0 = Trsf::rotation(axis, t0);
    let p0 = rot0.transform_point(&p);
    let x0 = (p0 - center).normalized()?;
    let circle = Circle::new(Ax3::new_axes(center, axis_dir, x0), r);
    let p1 = Trsf::rotation(axis, t1).transform_point(&p);
    Some(Edge::new(
        Some(GeomCurve::circle(circle)),
        0.0,
        t1 - t0,
        Vertex::new(p0),
        Vertex::new(p1),
    ))
}

/// One lateral face: the profile edge at `t0`, the arcs its endpoints trace to
/// `t1`, and the profile edge at `t1`, on the shared analytic `surface`.
fn lateral_face(edge: &Edge, axis: &Ax1, t0: f64, t1: f64, surface: GeomSurface) -> Face {
    let axis_dir = axis.direction();
    let rot0 = Trsf::rotation(axis, t0);
    let rot1 = Trsf::rotation(axis, t1);
    let e0 = edge.transformed(&rot0);
    let e1 = edge.transformed(&rot1);
    let p_src = edge.source().point();
    let p_tgt = edge.target().point();

    // Mirror prism's lateral loop: base edge backward, arc forward at the
    // source, far edge forward, arc backward at the target. A pole (endpoint
    // on the axis) simply contributes no arc. Return arcs are built forward
    // and reversed so their parameter range stays positive.
    let mut edges: Vec<Edge> = Vec::with_capacity(4);
    edges.push(e0.reversed());
    if let Some(arc) = revolved_arc(p_src, axis, axis_dir, t0, t1) {
        edges.push(arc);
    }
    edges.push(e1);
    if let Some(arc) = revolved_arc(p_tgt, axis, axis_dir, t0, t1) {
        edges.push(arc.reversed());
    }
    // INVARIANT: the loop must wind CCW in the surface's own (u, v) — i.e. its
    // winding normal must agree with the intrinsic surface normal (∂u × ∂v).
    // The material side is then carried solely by the orientation flag, which
    // `sew`'s winding-consistency BFS + global outward pass assigns; the
    // tessellator winds triangles from `orientation ⊗ intrinsic normal` and
    // ignores the loop direction, so a CW-in-uv loop renders inside-out (the
    // washer's inner wall did exactly that).
    // The patch centre, exactly on the surface: the profile edge's curve
    // midpoint rotated to the segment's middle angle. Projecting an OFF-surface
    // average can land on the far side of a torus tube and flip the test.
    let t_mid = 0.5 * (edge.first() + edge.last());
    let p_mid = match edge.curve() {
        Some(c) => c.point(t_mid),
        None => {
            let a = edge.source().point();
            let b = edge.target().point();
            a + (b - a) * 0.5
        }
    };
    let center = Trsf::rotation(axis, 0.5 * (t0 + t1)).transform_point(&p_mid);

    let wire = Wire::from_edges(edges);
    let wire = if loop_agrees_with_surface(&wire, &surface, center) {
        wire
    } else {
        reversed_wire(&wire)
    };
    Face::new(Some(surface), wire)
}

/// Does `wire`'s winding normal agree with `surface`'s intrinsic normal
/// (∂u × ∂v)? Sampled: Newell over points along each edge's curve (in loop
/// order), against the surface normal at `center` (a point ON the surface in
/// the middle of the patch). Shared by revolve and the loft/sweep skinner.
///
/// INVARIANT (curved-face winding): every curved face handed to `sew` MUST
/// already wind CCW in its surface's own uv — i.e. this returns `true`, else
/// the wire is reversed at construction. This is a CONSTRUCTION-time
/// responsibility that CANNOT be moved into `sew`'s post-hoc
/// `canonicalize_shell_orientation`, for two independent reasons:
///
///  1. Tessellation winds triangles from `orientation ⊗ intrinsic_normal`
///     (`orient_triangle_to_surface`, it ignores the loop entirely), while
///     `signed_volume` and boolean face-classification use
///     `orientation ⊗ loop_winding`. These agree only when
///     `intrinsic == loop_winding`. Flipping a face's *orientation flag* flips
///     BOTH effective normals, so it can never reconcile a face where they
///     disagree — only reversing the *wire* (changing `loop_winding`) can, and
///     a plane's stored normal can't stand in because a curved surface's normal
///     sense is intrinsic to its parameterisation.
///  2. `sew`'s step-7 BFS establishes cross-edge winding consistency over the
///     assembled loops; reversing a wire *after* that would desync the very
///     consistency the BFS built. So the reversal has to happen before sew.
///
/// Any new curved-surface op (revolve, skin, future sweeps/blends) must call
/// this before sewing. Do not "centralise" it into sew.
pub(crate) fn loop_agrees_with_surface(wire: &Wire, surface: &GeomSurface, center: Pnt) -> bool {
    const SAMPLES: usize = 6;
    let mut pts: Vec<Pnt> = Vec::new();
    for edge in wire.edges() {
        let (t0, t1) = (edge.first(), edge.last());
        let reversed = edge.orientation() != Orientation::Forward;
        for k in 0..SAMPLES {
            let f = k as f64 / SAMPLES as f64;
            let f = if reversed { 1.0 - f } else { f };
            let t = t0 + (t1 - t0) * f;
            let p = match edge.curve() {
                Some(c) => c.point(t),
                None => {
                    let a = edge.source().point();
                    let b = edge.target().point();
                    a + (b - a) * (k as f64 / SAMPLES as f64)
                }
            };
            pts.push(p);
        }
    }
    if pts.len() < 3 {
        return true;
    }
    // Newell winding normal of the sampled loop.
    let mut nw = GeomVec::ZERO;
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        nw += GeomVec::new(
            (p.y() - q.y()) * (p.z() + q.z()),
            (p.z() - q.z()) * (p.x() + q.x()),
            (p.x() - q.x()) * (p.y() + q.y()),
        );
    }
    // Intrinsic surface normal at the patch centre.
    let (u, v) = crate::intersect::uv_of(surface, &center);
    let (_, du, dv) = surface.d1(u, v);
    let ns = du.cross(&dv);
    nw.dot(&ns) >= 0.0
}

/// The analytic surface swept by one profile edge (in its base position —
/// every surface of revolution covers all `u`, so the angular window doesn't
/// change the surface).
fn lateral_surface(edge: &Edge, axis_point: Pnt, axis_dir: Dir) -> Result<GeomSurface, RevolveError> {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let c0 = closest_axis_point(p0, axis_point, axis_dir);
    let r0 = (p0 - c0).magnitude();
    let r1 = (p1 - closest_axis_point(p1, axis_point, axis_dir)).magnitude();
    let radial_x = |p: Pnt, c: Pnt| (p - c).normalized();

    match edge.curve() {
        Some(GeomCurve::Line(_)) | None => {
            let d = (p1 - p0).normalized().ok_or(RevolveError::UnsupportedProfileCurve)?;
            let along = d.dot(&axis_dir).abs();
            if (along - 1.0).abs() < 1e-9 {
                // Parallel to the axis → cylinder.
                let x = radial_x(p0, c0).ok_or(RevolveError::ProfileCrossesAxis)?;
                Ok(GeomSurface::cylinder(CylindricalSurface::new(
                    Ax3::new_axes(c0, axis_dir, x),
                    r0,
                )))
            } else if along < 1e-9 {
                // Perpendicular (radial) → flat annulus in the plane through
                // the segment, normal = axis.
                Ok(GeomSurface::plane(Plane::from_point_normal(p0, axis_dir)))
            } else {
                // Skew → cone. Reference radius r0 at v = 0 (the source's
                // axial station); the radius grows by tan(semi-angle) per unit
                // of +axis travel.
                let v1 = (p1 - p0).dot(&GeomVec::from_dir(axis_dir));
                let semi = ((r1 - r0) / v1).atan();
                let x = if r0 > tolerance::CONFUSION {
                    radial_x(p0, c0)
                } else {
                    radial_x(p1, closest_axis_point(p1, axis_point, axis_dir))
                }
                .ok_or(RevolveError::ProfileCrossesAxis)?;
                Ok(GeomSurface::cone(ConicalSurface::new(
                    Ax3::new_axes(c0, axis_dir, x),
                    r0,
                    semi,
                )))
            }
        }
        Some(GeomCurve::Circle(circle)) => {
            let ccen = circle.center();
            let caxis = closest_axis_point(ccen, axis_point, axis_dir);
            let major = (ccen - caxis).magnitude();
            if major <= tolerance::CONFUSION * 100.0 {
                // Arc centred on the axis → sphere. The profile arc's axis is
                // the plane normal, always perpendicular to the revolution
                // axis, so the cross product is a valid X direction.
                let x = axis_dir.cross(&circle.axis());
                Ok(GeomSurface::sphere(SphericalSurface::new(
                    Ax3::new_axes(ccen, axis_dir, x),
                    circle.radius(),
                )))
            } else {
                // Off-axis arc → torus, X toward the tube centre.
                let x = (ccen - caxis)
                    .normalized()
                    .ok_or(RevolveError::UnsupportedProfileCurve)?;
                Ok(GeomSurface::torus(ToroidalSurface::new(
                    Ax3::new_axes(caxis, axis_dir, x),
                    major,
                    circle.radius(),
                )))
            }
        }
        _ => Err(RevolveError::UnsupportedProfileCurve),
    }
}

/// Reverse a loop's winding (see prism's cap-orientation lesson: the cap whose
/// normal is reversed must also reverse its winding BEFORE sew, or sew flips
/// its orientation flag and the effective normal ends up inward).
pub(crate) fn reversed_wire(wire: &Wire) -> Wire {
    let mut edges: Vec<Edge> = wire.edges().iter().map(|e| e.reversed()).collect();
    edges.reverse();
    Wire::from_edges(edges)
}

fn revolve_cap(face: &Face, normal: Dir, rotation: Option<&Trsf>, flip_winding: bool) -> Face {
    let transform_wire = |wire: Wire| {
        let w = match rotation {
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
    Face::with_wires(
        Some(GeomSurface::plane(Plane::from_point_normal(point, normal))),
        outer,
        inners,
        Orientation::Forward,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_mesh::mass_properties;

    fn rect_profile(x0: f64, z0: f64, x1: f64, z1: f64) -> Face {
        // A rectangle in the XZ plane (y = 0), CCW about +Y.
        let p = [
            Pnt::new(x0, 0.0, z0),
            Pnt::new(x1, 0.0, z0),
            Pnt::new(x1, 0.0, z1),
            Pnt::new(x0, 0.0, z1),
        ];
        Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                p[0],
                Dir::dy(),
            ))),
            Wire::from_edges([
                Edge::between_points(p[0], p[1]),
                Edge::between_points(p[1], p[2]),
                Edge::between_points(p[2], p[3]),
                Edge::between_points(p[3], p[0]),
            ]),
        )
    }

    fn volume(solid: &Solid) -> f64 {
        let mesh = openrcad_mesh::tessellate(solid, 0.002, 0.05);
        mass_properties(&mesh).expect("closed mesh").volume
    }

    #[test]
    fn full_revolve_of_axis_touching_rect_is_cylinder() {
        // Rect from the axis (x=0) out to r=2, height 5 → full cylinder.
        let face = rect_profile(0.0, 0.0, 2.0, 5.0);
        let solid = revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap();
        assert!(
            solid.is_watertight(),
            "cylinder not watertight: {} faces, report: {:?}",
            solid.face_count(),
            solid.manifold_report()
        );
        let exact = PI * 2.0 * 2.0 * 5.0;
        let v = volume(&solid);
        assert!(
            (v - exact).abs() / exact < 0.01,
            "cylinder volume {v} vs {exact}"
        );
        // Lateral wall must be an analytic cylinder.
        assert!(solid
            .shell()
            .faces()
            .iter()
            .any(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_)))));
    }

    #[test]
    fn full_revolve_of_offset_rect_is_washer() {
        // Rect from r=1 to r=2, height 3 → tube (washer): π(R²−r²)h.
        let face = rect_profile(1.0, 0.0, 2.0, 3.0);
        let solid = revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap();
        assert!(solid.is_watertight(), "washer not watertight");
        let exact = PI * (4.0 - 1.0) * 3.0;
        let v = volume(&solid);
        assert!(
            (v - exact).abs() / exact < 0.01,
            "washer volume {v} vs {exact}"
        );
    }

    #[test]
    fn quarter_revolve_has_caps_and_correct_volume() {
        let face = rect_profile(1.0, 0.0, 2.0, 3.0);
        let solid = revolve(&face, Pnt::origin(), Dir::dz(), TAU / 4.0).unwrap();
        assert!(solid.is_watertight(), "quarter washer not watertight");
        assert!(solid.health_report().is_healthy());
        let exact = PI * (4.0 - 1.0) * 3.0 / 4.0;
        let v = volume(&solid);
        assert!(
            (v - exact).abs() / exact < 0.01,
            "quarter volume {v} vs {exact}"
        );
        // Effective normals all point outward (the prism cap lesson).
        let mesh = openrcad_mesh::tessellate(&solid, 0.002, 0.05);
        let mp = mass_properties(&mesh).unwrap();
        assert!(mp.volume > 0.0);
    }

    #[test]
    fn skew_profile_line_makes_cone() {
        // Right triangle: (0,0)→(2,0)→(0,4): revolve → cone, V = πr²h/3.
        let p = [
            Pnt::new(0.0, 0.0, 0.0),
            Pnt::new(2.0, 0.0, 0.0),
            Pnt::new(0.0, 0.0, 4.0),
        ];
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                p[0],
                Dir::dy(),
            ))),
            Wire::from_edges([
                Edge::between_points(p[0], p[1]),
                Edge::between_points(p[1], p[2]),
                Edge::between_points(p[2], p[0]),
            ]),
        );
        let solid = revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap();
        assert!(solid.is_watertight(), "cone not watertight");
        assert!(solid
            .shell()
            .faces()
            .iter()
            .any(|f| matches!(f.surface(), Some(GeomSurface::Cone(_)))));
        // The cone lateral is an exact analytic ConicalSurface, and the cone
        // tessellator now gives it a full ruled grid + ruled-edge refinement
        // (was a sparse single mid-ring that under-filled ~13%), so the mesh
        // matches the analytic volume to <1%, like cylinders and tori.
        let exact = PI * 4.0 * 4.0 / 3.0;
        let v = volume(&solid);
        assert!((v - exact).abs() / exact < 0.01, "cone volume {v} vs {exact}");
    }

    #[test]
    fn circle_profile_makes_torus() {
        // Circle of radius 0.5 centred at x=2 in the XZ plane → torus,
        // V = 2π²·R·r².
        let center = Pnt::new(2.0, 0.0, 0.0);
        let circle = Circle::new(Ax3::new_axes(center, Dir::dy(), Dir::dx()), 0.5);
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
                center,
                Dir::dy(),
            ))),
            Wire::from_edges(edges),
        );
        let solid = revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap();
        assert!(solid.is_watertight(), "torus not watertight");
        assert!(solid
            .shell()
            .faces()
            .iter()
            .any(|f| matches!(f.surface(), Some(GeomSurface::Torus(_)))));
        let exact = 2.0 * PI * PI * 2.0 * 0.25;
        let v = volume(&solid);
        assert!(
            (v - exact).abs() / exact < 0.015,
            "torus volume {v} vs {exact}"
        );
    }

    #[test]
    fn crossing_profile_and_bad_axis_are_rejected() {
        // Rect straddling the axis.
        let face = rect_profile(-1.0, 0.0, 1.0, 2.0);
        assert_eq!(
            revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap_err(),
            RevolveError::ProfileCrossesAxis
        );
        // Axis not in the profile plane.
        let face = rect_profile(1.0, 0.0, 2.0, 3.0);
        assert_eq!(
            revolve(&face, Pnt::new(0.0, 5.0, 0.0), Dir::dx(), TAU).unwrap_err(),
            RevolveError::AxisNotInProfilePlane
        );
        // Zero angle.
        assert_eq!(
            revolve(&face, Pnt::origin(), Dir::dz(), 0.0).unwrap_err(),
            RevolveError::InvalidAngle
        );
    }

    #[test]
    fn revolved_solid_survives_boolean() {
        use crate::{boolean::boolean, BooleanOp};
        let face = rect_profile(0.0, 0.0, 2.0, 5.0);
        let cyl = revolve(&face, Pnt::origin(), Dir::dz(), TAU).unwrap();
        let cutter = openrcad_primitives::make_box(&Pnt::new(-3.0, -3.0, 4.0), 6.0, 6.0, 3.0);
        let cut = boolean(&cyl, &cutter, BooleanOp::Cut);
        assert!(
            cut.is_watertight(),
            "revolved cylinder minus box not watertight"
        );
        let exact = PI * 4.0 * 4.0; // height reduced 5 → 4
        let v = volume(&cut);
        assert!((v - exact).abs() / exact < 0.01, "cut volume {v} vs {exact}");
    }
}
