//! Rolling-ball edge blend construction.
//!
//! This module is the first local-blend slice: it solves the contact curves for
//! a constant-radius ball rolling along a selected edge shared by two planar
//! faces, then builds the cylindrical blend face bounded by those contacts.

use core::fmt;

use openrcad_foundation::{tolerance, Ax2, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{
    Circle, ConicalSurface, Curve, CylindricalSurface, Ellipse, GeomCurve, GeomSurface,
    GregorySurface, Plane, SphericalSurface, Surface, ToroidalSurface,
};
use openrcad_mesh::tessellate;
use openrcad_primitives::make_cylinder;
use openrcad_topo::{Edge, Face, FaceId, Orientation, Solid, Vertex, Wire};

use crate::sew::sew;

/// Reasons the rolling-ball solver could not resolve a face adjacency.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdjacencyReason {
    /// An adjacent face's surface is neither planar nor analytic-supported.
    NotPlaneOrAnalytic,
    /// Both adjacent faces are non-planar and no analytic path applies.
    BothFacesNonPlanar,
    /// The numeric tangency solver failed to converge.
    NewtonFailed,
    /// The requested radius exceeds what fits the adjacent geometry.
    RadiusTooLarge,
    /// The pair of surface types has no supported solver path.
    UnsupportedSurfacePair,
}

/// Errors reported by the rolling-ball solver.
#[derive(Clone, Debug, PartialEq)]
pub enum RollingBallError {
    /// Radius must be finite and positive.
    InvalidRadius { radius: f64 },
    /// The selected edge is degenerate.
    DegenerateSpine,
    /// The selected edge is not shared by exactly two faces in the solid.
    EdgeAdjacency { count: usize },
    /// The two adjacent faces could not be blended (analytic or numeric).
    UnsolvableAdjacency { reason: AdjacencyReason },
    /// The adjacent faces are parallel or otherwise do not define a convex
    /// rolling-ball wedge.
    InvalidDihedral,
    /// The selected edge was not found in a face that should be trimmed.
    SpineNotOnFace,
    /// This local trim pass only supports simple outer-loop faces.
    UnsupportedTrimTopology,
    /// The numeric tangency solver diverged.
    NewtonDiverged { iterations: usize },
    /// The blend surface could not be constructed.
    BlendSurfaceBuild(&'static str),
    /// The local edit built faces, but the sewn shell was not watertight/healthy.
    InvalidTopology,
}

impl fmt::Display for RollingBallError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRadius { radius } => {
                write!(f, "rolling ball: radius must be positive, got {radius}")
            }
            Self::DegenerateSpine => f.write_str("rolling ball: selected edge is degenerate"),
            Self::EdgeAdjacency { count } => write!(
                f,
                "rolling ball: selected edge must have exactly two adjacent faces, found {count}"
            ),
            Self::UnsolvableAdjacency { reason } => {
                write!(
                    f,
                    "rolling ball: adjacent faces could not be blended ({reason:?})"
                )
            }
            Self::InvalidDihedral => {
                f.write_str("rolling ball: adjacent faces do not form a blendable wedge")
            }
            Self::SpineNotOnFace => {
                f.write_str("rolling ball: selected edge was not found on an adjacent face")
            }
            Self::UnsupportedTrimTopology => f.write_str(
                "rolling ball: local trimming currently supports simple outer-loop faces only",
            ),
            Self::NewtonDiverged { iterations } => write!(
                f,
                "rolling ball: numeric tangency solver diverged after {iterations} iterations"
            ),
            Self::BlendSurfaceBuild(msg) => {
                write!(f, "rolling ball: blend surface could not be built ({msg})")
            }
            Self::InvalidTopology => {
                f.write_str("rolling ball: rebuilt body is not watertight and healthy")
            }
        }
    }
}

impl std::error::Error for RollingBallError {}

/// Contact-curve and blend-surface result for one planar rolling-ball edge.
#[derive(Clone, Debug)]
pub struct RollingBallBlend {
    /// The selected spine edge.
    pub spine: Edge,
    /// First adjacent face.
    pub face_a: Face,
    /// Second adjacent face.
    pub face_b: Face,
    /// Contact curve on `face_a`.
    pub contact_a: Edge,
    /// Contact curve on `face_b`.
    pub contact_b: Edge,
    /// Ball-center path.
    pub centerline: Edge,
    /// Cylindrical blend face bounded by both contact curves and endpoint arcs.
    pub blend_face: Face,
    /// Arc on the start endpoint cap, from contact B to contact A.
    pub start_arc: Edge,
    /// Arc on the end endpoint cap, from contact A to contact B.
    pub end_arc: Edge,
    /// Fillet radius.
    pub radius: f64,
    /// True when the spine's material wedge is reflex (an inner corner, e.g.
    /// the vertical edge of a rectangular pocket). A concave blend ADDS
    /// material — the ball rides in the void wedge — so the convex corner
    /// closures (miter/sphere/cut runout) don't apply; endpoints terminate
    /// with the flat cap trim only.
    pub concave: bool,
}

/// Apply a selected-edge rolling-ball fillet to a simple planar solid.
///
/// This performs the first surgical local B-Rep edit: the two adjacent planar
/// faces are trimmed to the contact curves, the two endpoint cap faces are
/// trimmed to the circular end arcs, the cylindrical blend face is inserted,
/// and the shell is sewn back into a watertight solid.
fn face_outward_normal_at(face: &Face, point: Pnt) -> Result<Dir, RollingBallError> {
    let normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        Some(GeomSurface::Cylinder(cyl)) => {
            let axis_pt = cyl.position().location();
            let axis_dir = cyl.position().direction();
            let v = point - axis_pt;
            let proj = axis_pt + GeomVec::from_dir(axis_dir) * v.dot(&GeomVec::from_dir(axis_dir));
            let radial =
                (point - proj)
                    .normalized()
                    .ok_or(RollingBallError::UnsolvableAdjacency {
                        reason: AdjacencyReason::NotPlaneOrAnalytic,
                    })?;
            Dir::new(radial.x(), radial.y(), radial.z())
        }
        Some(surf @ (GeomSurface::Cone(_) | GeomSurface::Sphere(_))) => {
            // General analytic surfaces: the normal is dU × dV at the nearest
            // parameter to `point`, computed from the surface's first derivatives.
            let (u, v) = surface_nearest_uv(surf, point);
            let (_, du, dv) = surf.d1(u, v);
            du.cross(&dv)
                .normalized()
                .map(|n| Dir::new(n.x(), n.y(), n.z()))
                .ok_or(RollingBallError::UnsolvableAdjacency {
                    reason: AdjacencyReason::NotPlaneOrAnalytic,
                })?
        }
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    Ok(if face.orientation() == Orientation::Reversed {
        normal.reversed()
    } else {
        normal
    })
}

/// Nearest `(u, v)` on a general surface to `point`, via a coarse parameter
/// sweep refined by a short bounded Gauss-Newton search using `d1`.
fn surface_nearest_uv(surface: &GeomSurface, point: Pnt) -> (f64, f64) {
    let (mut u0, mut u1, mut v0, mut v1) = surface.bounds();
    if !u0.is_finite() {
        u0 = -100.0;
    }
    if !u1.is_finite() {
        u1 = 100.0;
    }
    if !v0.is_finite() {
        v0 = -100.0;
    }
    if !v1.is_finite() {
        v1 = 100.0;
    }
    let (u0, u1) = ordered_bounds(u0, u1);
    let (v0, v1) = ordered_bounds(v0, v1);

    let n = 16;
    let mut best = (u0, v0);
    let mut best_d2 = f64::INFINITY;
    for i in 0..=n {
        let u = u0 + (u1 - u0) * (i as f64) / (n as f64);
        for j in 0..=n {
            let v = v0 + (v1 - v0) * (j as f64) / (n as f64);
            let d2 = surface.point(u, v).distance_squared(&point);
            if d2 < best_d2 {
                best_d2 = d2;
                best = (u, v);
            }
        }
    }

    let (mut u, mut v) = best;
    for _ in 0..16 {
        let (s, du, dv) = surface.d1(u, v);
        let r = s - point;
        let guu = du.dot(&du);
        let gvv = dv.dot(&dv);
        let guv = du.dot(&dv);
        let bu = r.dot(&du);
        let bv = r.dot(&dv);
        let det = guu * gvv - guv * guv;
        if det.abs() <= 1e-14 {
            break;
        }
        let step_u = (bu * gvv - bv * guv) / det;
        let step_v = (guu * bv - guv * bu) / det;
        u = clamp_ordered(u - step_u, u0, u1);
        v = clamp_ordered(v - step_v, v0, v1);
        if step_u.abs() + step_v.abs() <= 1e-12 {
            break;
        }
    }
    (u, v)
}

fn ordered_bounds(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    let (lo, hi) = ordered_bounds(min, max);
    if value.is_nan() {
        lo
    } else {
        value.max(lo).min(hi)
    }
}

fn radial(axis: Dir, xref: Dir, u: f64) -> GeomVec {
    let x = GeomVec::from_dir(xref);
    let y = GeomVec::from_dir(axis).cross(&GeomVec::from_dir(xref));
    x * u.cos() + y * u.sin()
}

fn make_gregory_corner_patch(corner: Pnt, p_a: Pnt, p_b: Pnt, _radius: f64) -> Face {
    let p01 = corner;
    let p02 = corner;

    let p10 = corner + (p_a - corner) * 0.33;
    let p20 = corner + (p_a - corner) * 0.66;

    let p31 = p_a + (p_b - p_a) * 0.33;
    let p32 = p_a + (p_b - p_a) * 0.66;

    let p13 = corner + (p_b - corner) * 0.33;
    let p23 = corner + (p_b - corner) * 0.66;

    let p11_u = p10 + (p31 - p10) * 0.5;
    let p11_v = p13 + (p32 - p13) * 0.5;
    let p21_u = p20 + (p31 - p20) * 0.5;
    let p21_v = p23 + (p32 - p23) * 0.5;

    let p12_u = p11_u;
    let p12_v = p11_v;
    let p22_u = p21_u;
    let p22_v = p21_v;

    let surf = GregorySurface::new(
        corner, p01, p02, corner, p10, p20, p_a, p31, p32, p_b, p13, p23, p11_u, p11_v, p21_u,
        p21_v, p12_u, p12_v, p22_u, p22_v,
    );

    let e1 = Edge::between_points(corner, p_a);
    let e2 = Edge::between_points(p_a, p_b);
    let e3 = Edge::between_points(p_b, corner);

    let wire = Wire::from_edges([e1, e2, e3]);
    Face::new(Some(GeomSurface::gregory(surf)), wire)
}

/// Apply a selected-edge rolling-ball fillet to a simple planar or curved solid.
///
/// When the filleted edge ends at a corner already rounded by an earlier fillet
/// (two blends meeting at a shared vertex), the corner is closed with a smooth
/// spherical patch — the rolling ball pivoting in the corner — instead of the
/// degenerate flat trim that collapses the prior cylinder to its own axis (the
/// "spike" artifact). If that smooth path leaves a non-watertight shell for an
/// unusual corner, the build retries with the legacy corner trim, which always
/// closes (it just creases).
pub fn fillet_planar_edge(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    match fillet_planar_edge_inner(solid, edge, radius, true) {
        Ok(solid) => Ok(solid),
        Err(_) => fillet_planar_edge_inner(solid, edge, radius, false),
    }
}

fn fillet_planar_edge_inner(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
    use_sphere: bool,
) -> Result<Solid, RollingBallError> {
    let mut blend = rolling_ball_fillet_edge(solid, edge, radius)?;
    let start = edge.source().point();
    let end = edge.target().point();

    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);
    if std::env::var("ORC_DEBUG_FILLET").is_ok() {
        let kind = |f: &Face| match f.surface() {
            Some(GeomSurface::Plane(p)) => {
                let n = p.normal();
                let l = p.position().location();
                format!(
                    "Plane(n=({:.2},{:.2},{:.2}) p=({:.1},{:.1},{:.1}))",
                    n.x(),
                    n.y(),
                    n.z(),
                    l.x(),
                    l.y(),
                    l.z()
                )
            }
            Some(GeomSurface::Cylinder(_)) => "Cyl".to_string(),
            Some(GeomSurface::Torus(_)) => "Torus".to_string(),
            _ => "Other".to_string(),
        };
        eprintln!(
            "fillet dbg: concave={} start={:?} end={:?} start_caps={:?} end_caps={:?} use_sphere={use_sphere}",
            blend.concave,
            (start.x(), start.y(), start.z()),
            (end.x(), end.y(), end.z()),
            start_caps.iter().map(kind).collect::<Vec<_>>(),
            end_caps.iter().map(kind).collect::<Vec<_>>()
        );
    }
    let cut_guards = cut_cylinder_guards(solid, &start_caps, &end_caps);

    let mut faces = Vec::new();
    let mut skipped_faces = std::collections::HashSet::new();

    // First pass: smart corner closure, before the spine trims read the contacts
    // (`try_corner_miter` extends the new blend toward the seam by mutating `blend`).
    //
    // - One earlier fillet meets this one and the corner's third edge is still
    //   sharp → the two fillets *miter*, meeting along their mutual seam and leaving
    //   the third edge sharp (`try_corner_miter`).
    // - Two earlier fillets already meet here (mitered) and this is the third edge
    //   at the corner → it closes into a spherical octant (`try_corner_sphere_two_caps`).
    //
    // The two-cap sphere is tried first (a two-cap corner is never a single-cap
    // miter). Both are gated on `use_sphere`, so the retry path (use_sphere = false)
    // falls back to the flat corner trim if either leaves a non-watertight shell.
    // A blend endpoint that runs into a concave cut cylinder is trimmed flush
    // against it (the cut stays a clean vertical cylinder). This is not a sphere
    // path, so it runs regardless of `use_sphere`, and takes priority over the
    // miter/sphere closures.
    //
    // A CONCAVE spine blend adds material in the void wedge; every smart corner
    // closure below encodes the subtractive geometry (ball center inside the
    // material), so a concave blend skips them all and terminates with the flat
    // cap trim, which is side-agnostic wire surgery.
    let convex = !blend.concave;
    let start_cut = convex
        && try_corner_cut(
            solid,
            &mut blend,
            start,
            &start_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?;
    let start_tangent_wall = if start_cut || !convex {
        false
    } else {
        try_tangent_curved_wall_runout(
            solid,
            &mut blend,
            start,
            &start_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?
    };
    let start_mitered = start_cut
        || start_tangent_wall
        || (convex
            && use_sphere
            && (try_corner_sphere_two_caps(
                solid,
                &blend,
                start,
                &start_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_miter(
                solid,
                &mut blend,
                start,
                &start_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_circular_band_miter(
                &mut blend,
                start,
                &start_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            )));
    // Oblique planar cap (any non-perpendicular angle): flush ellipse trim.
    // Side-agnostic wire surgery, so it serves convex and concave blends alike.
    let start_mitered = start_mitered
        || try_oblique_planar_cap(
            &mut blend,
            start,
            &start_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?;
    let end_cut = convex
        && try_corner_cut(
            solid,
            &mut blend,
            end,
            &end_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?;
    let end_tangent_wall = if end_cut || !convex {
        false
    } else {
        try_tangent_curved_wall_runout(
            solid,
            &mut blend,
            end,
            &end_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?
    };
    let end_mitered = end_cut
        || end_tangent_wall
        || (convex
            && use_sphere
            && (try_corner_sphere_two_caps(
                solid,
                &blend,
                end,
                &end_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_miter(
                solid,
                &mut blend,
                end,
                &end_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_circular_band_miter(
                &mut blend,
                end,
                &end_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            )));
    let end_mitered = end_mitered
        || try_oblique_planar_cap(
            &mut blend,
            end,
            &end_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?;

    if !start_mitered {
        handle_corner_endpoint(
            solid,
            &blend,
            start,
            &start_caps,
            &blend.start_arc,
            radius,
            use_sphere && convex,
            &mut faces,
            &mut skipped_faces,
        )?;
    }
    if !end_mitered {
        handle_corner_endpoint(
            solid,
            &blend,
            end,
            &end_caps,
            &blend.end_arc,
            radius,
            use_sphere && convex,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

    let trimmed_a = trim_face_along_spine(&blend.face_a, &blend.spine, &blend.contact_a)?;
    let trimmed_b = trim_face_along_spine(&blend.face_b, &blend.spine, &blend.contact_b)?;
    if std::env::var("ORC_DEBUG_FILLET").is_ok() {
        for (name, f) in [("trimmed_a", &trimmed_a), ("trimmed_b", &trimmed_b)] {
            let pts: Vec<String> = f
                .outer_wire()
                .map(|w| w.edges())
                .unwrap_or_default()
                .iter()
                .map(|e| {
                    let a = e.source().point();
                    let b = e.target().point();
                    format!(
                        "[{:.2},{:.2},{:.2}]->[{:.2},{:.2},{:.2}]",
                        a.x(), a.y(), a.z(), b.x(), b.y(), b.z()
                    )
                })
                .collect();
            eprintln!("trim dbg {name}: {}", pts.join(" "));
        }
    }

    for face in solid.shell().faces() {
        if same_face(&face, &blend.face_a)
            || same_face(&face, &blend.face_b)
            || skipped_faces.contains(&face.id())
        {
            continue;
        }
        faces.push(face);
    }

    faces.push(trimmed_a);
    faces.push(trimmed_b);
    faces.push(blend.blend_face);

    // Reject a blend that didn't close: a radius too large for the local geometry
    // (e.g. larger than half the part thickness) collapses trim edges and leaves a
    // non-watertight / degenerate shell. Surface that as an error rather than
    // returning a broken solid the application would cache.
    let result = Solid::new(sew(&faces, radius * 0.1));
    if std::env::var("ORC_DEBUG_FILLET").is_ok() {
        eprintln!(
            "fillet dbg: result watertight={} healthy={} errors={:?}",
            result.is_watertight(),
            result.health_report().is_healthy(),
            result.health_report().errors
        );
        for (fi, face) in result.shell().faces().iter().enumerate() {
            for wire in face.wires() {
                let edges = wire.edges();
                let n = edges.len();
                for i in 0..n {
                    let t = edges[i].target().point();
                    let s = edges[(i + 1) % n].source().point();
                    if t.distance(&s) > 1.0e-4 {
                        eprintln!(
                            "  gap dbg: face {fi} edge {i}->{}: [{:.3},{:.3},{:.3}] != [{:.3},{:.3},{:.3}]",
                            (i + 1) % n,
                            t.x(), t.y(), t.z(), s.x(), s.y(), s.z()
                        );
                    }
                }
            }
        }
    }
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if cut_guards.is_empty() {
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
    } else {
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
    }
    Err(RollingBallError::InvalidTopology)
}

#[derive(Clone)]
struct CutCylinderGuard {
    cyl: CylindricalSurface,
    v_min: f64,
    v_max: f64,
}

fn cut_cylinder_guards(
    solid: &Solid,
    start_caps: &[Face],
    end_caps: &[Face],
) -> Vec<CutCylinderGuard> {
    start_caps
        .iter()
        .chain(end_caps.iter())
        .filter(|cap| is_concave_cut_cylinder(solid, cap))
        .filter_map(|cap| {
            let Some(GeomSurface::Cylinder(cyl)) = cap.surface() else {
                return None;
            };
            let mut v_min = f64::INFINITY;
            let mut v_max = f64::NEG_INFINITY;
            let axis_loc = cyl.position().location();
            let axis = GeomVec::from_dir(cyl.position().direction());
            for wire in cap.wires() {
                for edge in wire.edges() {
                    let Some(curve) = edge.curve() else {
                        continue;
                    };
                    let (t0, t1) = (edge.first(), edge.last());
                    for k in 0..=8 {
                        let t = t0 + (t1 - t0) * (k as f64) / 8.0;
                        let p = curve.point(t);
                        let v = (p - axis_loc).dot(&axis);
                        v_min = v_min.min(v);
                        v_max = v_max.max(v);
                    }
                }
            }
            v_min.is_finite().then_some(CutCylinderGuard {
                cyl: *cyl,
                v_min,
                v_max,
            })
        })
        .collect()
}

fn accept_subtractive_blend_result(
    candidate: &Solid,
    cut_guards: &[CutCylinderGuard],
) -> Option<Solid> {
    if !candidate.is_watertight() || !candidate.health_report().is_healthy() {
        return None;
    }
    if cut_guards.is_empty() {
        return Some(candidate.clone());
    }
    let intrudes = solid_surface_intrudes_into_cut(candidate, cut_guards);
    if !intrudes {
        return Some(candidate.clone());
    }

    let mut clipped = candidate.clone();
    for guard in cut_guards {
        if !solid_surface_intrudes_into_cut(&clipped, std::slice::from_ref(guard)) {
            continue;
        }
        let axis = guard.cyl.position();
        let dir = axis.direction();
        let base = axis.location() + GeomVec::from_dir(dir) * (guard.v_min - 0.25);
        let cutter_axis = Ax2::new_axes(base, dir, axis.x_direction());
        let cutter = make_cylinder(
            &cutter_axis,
            guard.cyl.radius(),
            (guard.v_max - guard.v_min).abs() + 0.5,
        );
        clipped = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            crate::boolean_checked(&clipped, &cutter, crate::BooleanOp::Cut)
        }))
        .ok()
        .and_then(Result::ok)?;
        clipped =
            crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&clipped));
    }
    (clipped.is_watertight()
        && clipped.health_report().is_healthy()
        && !solid_surface_intrudes_into_cut(&clipped, cut_guards))
    .then_some(clipped)
}

fn solid_surface_intrudes_into_cut(candidate: &Solid, cut_guards: &[CutCylinderGuard]) -> bool {
    solid_surface_intrudes_into_cut_once(candidate, cut_guards)
}

fn solid_surface_intrudes_into_cut_once(
    candidate: &Solid,
    cut_guards: &[CutCylinderGuard],
) -> bool {
    let mesh = tessellate(candidate, 0.05, 0.5);
    let faces = candidate.shell().faces();
    for (i, tri) in mesh.triangles.iter().enumerate() {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        let centroid = Pnt::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        let face = mesh
            .face_ids
            .get(i)
            .and_then(|fid| faces.get(*fid as usize));
        for guard in cut_guards {
            if face
                .and_then(|face| face.surface())
                .is_some_and(|surface| match surface {
                    GeomSurface::Cylinder(cyl) => cylinders_same_surface(cyl, &guard.cyl),
                    _ => false,
                })
            {
                continue;
            }
            if point_inside_cut_guard(centroid, guard, 0.75) {
                return true;
            }
        }
    }
    false
}

fn cylinders_same_surface(a: &CylindricalSurface, b: &CylindricalSurface) -> bool {
    if (a.radius() - b.radius()).abs() > 1.0e-6 {
        return false;
    }
    let ad = GeomVec::from_dir(a.position().direction());
    let bd = GeomVec::from_dir(b.position().direction());
    if ad.dot(&bd).abs() < 0.999_999 {
        return false;
    }
    let delta = b.position().location() - a.position().location();
    (delta - ad * delta.dot(&ad)).magnitude() <= 1.0e-5
}

fn point_inside_cut_guard(p: Pnt, guard: &CutCylinderGuard, margin: f64) -> bool {
    let axis_loc = guard.cyl.position().location();
    let axis = GeomVec::from_dir(guard.cyl.position().direction());
    let v = (p - axis_loc).dot(&axis);
    if v < guard.v_min - 0.05 || v > guard.v_max + 0.05 {
        return false;
    }
    let radial = (p - axis_loc) - axis * v;
    radial.magnitude() < guard.cyl.radius() - margin
}

/// Faces produced for one shared corner: the new faces to add and the original
/// faces (the prior cylinder, the trimmed side face) those replace.
struct CornerSphere {
    new_faces: Vec<Face>,
    skip_ids: Vec<FaceId>,
}

/// Trim/close the faces around one spine endpoint of a blend.
///
/// Three cases:
/// - The endpoint cap is a single prior *blend* surface (a cylinder/torus) — two
///   fillets share this corner. Round it with a [`corner_sphere_blend`].
/// - The cap is a single planar face — trim that face for the blend's end arc.
/// - Several cap faces (an n-valent vertex) — drop in a Gregory corner patch.
#[allow(clippy::too_many_arguments)]
fn handle_corner_endpoint(
    solid: &Solid,
    blend: &RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    arc: &Edge,
    radius: f64,
    use_sphere: bool,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<(), RollingBallError> {
    // The blend's two contact points at this corner (nearest endpoint of each
    // contact curve), independent of spine orientation.
    let ca = nearest_endpoint(&blend.contact_a, corner);
    let cb = nearest_endpoint(&blend.contact_b, corner);

    if caps.len() == 1 {
        let cap = &caps[0];
        let is_prior_blend = matches!(
            cap.surface(),
            Some(GeomSurface::Cylinder(_)) | Some(GeomSurface::Torus(_))
        );
        if use_sphere && is_prior_blend {
            if let Some(cs) = corner_sphere_blend(solid, cap, corner, blend, ca, cb, radius) {
                faces.extend(cs.new_faces);
                skipped.extend(cs.skip_ids);
                return Ok(());
            }
        }
        // The flat trim splices the blend's end arc into the cap's wire, which
        // is only sound when the arc actually lies in the cap (true for a
        // straight spine meeting a perpendicular planar cap). A circular-spine
        // blend's end arc lives in the spine's RADIAL plane instead — trimming
        // a planar cap with it bulges the solid through the cap, and the result
        // can still sew watertight, so the acceptance gate won't catch it.
        if let Some(GeomSurface::Plane(pl)) = cap.surface() {
            let n = GeomVec::from_dir(pl.normal());
            let p0 = pl.position().location();
            let off = |p: Pnt| (p - p0).dot(&n).abs();
            let mid = arc
                .curve()
                .map(|c| c.point(0.5 * (arc.first() + arc.last())))
                .unwrap_or_else(|| arc.start().point());
            let tol = corner_tol(radius);
            if off(arc.start().point()) > tol || off(arc.end().point()) > tol || off(mid) > tol {
                return Err(RollingBallError::UnsupportedTrimTopology);
            }
        }
        let trimmed = trim_face_at_corner(cap, corner, ca, cb, arc)?;
        faces.push(trimmed);
        skipped.insert(cap.id());
        Ok(())
    } else {
        // Several caps at this endpoint. When they are pieces of ONE plane that
        // a boolean imprint left split (e.g. a fused boss's bottom disc sitting
        // coplanar with the base face, sharing a chord edge), merge them and run
        // the ordinary flat trim on the union — the corner vertex then owns both
        // of its loop edges in a single wire. The Gregory patch below is for
        // genuinely distinct cap planes at an n-valent vertex.
        let dbg = std::env::var("ORC_DEBUG_FILLET").is_ok();
        if let Some(merged) = merged_coplanar_cap(caps) {
            if dbg {
                eprintln!(
                    "corner dbg: merged cap with {} outer edges",
                    merged.outer_wire().map_or(0, |w| w.edges().len())
                );
            }
            let in_plane = match merged.surface() {
                Some(GeomSurface::Plane(pl)) => {
                    let n = GeomVec::from_dir(pl.normal());
                    let p0 = pl.position().location();
                    let off = |p: Pnt| (p - p0).dot(&n).abs();
                    let mid = arc
                        .curve()
                        .map(|c| c.point(0.5 * (arc.first() + arc.last())))
                        .unwrap_or_else(|| arc.start().point());
                    let tol = corner_tol(radius);
                    off(arc.start().point()) <= tol
                        && off(arc.end().point()) <= tol
                        && off(mid) <= tol
                }
                _ => false,
            };
            if in_plane {
                match trim_face_at_corner(&merged, corner, ca, cb, arc) {
                    Ok(trimmed) => {
                        faces.push(trimmed);
                        skipped.extend(caps.iter().map(|cap| cap.id()));
                        return Ok(());
                    }
                    Err(e) => {
                        if dbg {
                            eprintln!("corner dbg: merged-cap trim failed: {e}");
                        }
                    }
                }
            } else if dbg {
                eprintln!("corner dbg: merged cap rejected — arc not in cap plane");
            }
        } else if dbg {
            eprintln!("corner dbg: caps did not merge (n={})", caps.len());
        }
        let patch = make_gregory_corner_patch(corner, ca, cb, radius);
        faces.push(patch);
        for cap in caps {
            if let Ok(trimmed) = trim_face_at_corner(cap, corner, ca, cb, arc) {
                faces.push(trimmed);
                skipped.insert(cap.id());
            }
        }
        Ok(())
    }
}

/// Merge endpoint cap faces that are all pieces of ONE coplanar, same-facing
/// plane into a single face: their outer wires are unioned with the shared
/// (undirected-equal) seam edges cancelled and the remainder rethreaded into
/// one loop. Inner (hole) wires are carried through. `None` when the caps are
/// not one coplanar family or the union does not chain into a single loop.
fn merged_coplanar_cap(caps: &[Face]) -> Option<Face> {
    if caps.len() < 2 {
        return None;
    }
    let tol = 10.0 * tolerance::CONFUSION;
    let plane_of = |cap: &Face| -> Option<Plane> {
        match cap.surface() {
            Some(GeomSurface::Plane(p)) => Some(*p),
            _ => None,
        }
    };
    let effective_normal = |cap: &Face| -> Option<GeomVec> {
        let n = GeomVec::from_dir(plane_of(cap)?.normal());
        Some(if cap.orientation() == Orientation::Reversed {
            n * -1.0
        } else {
            n
        })
    };
    // Same geometric plane for every cap. The stored normals may disagree in
    // SIGN: a boolean can leave one coplanar sliver with a flipped effective
    // normal (e.g. a fused boss's under-disc plug), and such a piece is exactly
    // the one whose edges all cancel below. The consistently-facing MAJORITY
    // group decides the merged face's surface, orientation, and seed winding.
    let p0 = plane_of(&caps[0])?;
    let n0 = effective_normal(&caps[0])?;
    for cap in &caps[1..] {
        let p = plane_of(cap)?;
        let n = effective_normal(cap)?;
        if n.dot(&n0).abs() < 0.999_999 {
            return None;
        }
        if (p.position().location() - p0.position().location())
            .dot(&n0)
            .abs()
            > tol
        {
            return None;
        }
    }
    let agrees_with_first: Vec<bool> = caps
        .iter()
        .map(|cap| effective_normal(cap).is_some_and(|n| n.dot(&n0) > 0.0))
        .collect();
    let majority_agrees =
        2 * agrees_with_first.iter().filter(|&&a| a).count() >= caps.len();

    let mut pool: Vec<Edge> = Vec::new();
    let mut pool_cap: Vec<usize> = Vec::new();
    for (ci, cap) in caps.iter().enumerate() {
        let edges = cap.outer_wire()?.edges();
        pool_cap.extend(std::iter::repeat(ci).take(edges.len()));
        pool.extend(edges);
    }
    // Cancel the shared seam edges (each appears once per abutting face).
    let mut removed = vec![false; pool.len()];
    for i in 0..pool.len() {
        if removed[i] {
            continue;
        }
        for j in (i + 1)..pool.len() {
            if removed[j] {
                continue;
            }
            if same_undirected_edge(&pool[i], &pool[j]) {
                removed[i] = true;
                removed[j] = true;
                break;
            }
        }
    }
    let mut pool: Vec<(Edge, usize)> = pool
        .into_iter()
        .zip(pool_cap)
        .zip(removed)
        .filter_map(|(entry, r)| (!r).then_some(entry))
        .collect();
    if pool.len() < 3 {
        return None;
    }

    // Rethread into one closed loop. Each majority-facing cap's retained edges
    // are already wound for the union (a face keeps its own winding along its
    // surviving boundary), so seed with one of THOSE to fix the loop direction —
    // a flipped sliver's edges would seed the loop inside-out.
    let seed_idx = pool
        .iter()
        .position(|(_, ci)| agrees_with_first[*ci] == majority_agrees)?;
    let seed_cap = pool[seed_idx].1;
    let mut loop_edges = vec![pool.remove(seed_idx).0];
    let mut pool: Vec<Edge> = pool.into_iter().map(|(e, _)| e).collect();
    while !pool.is_empty() {
        let end = loop_edges.last().expect("seeded").target().point();
        let next = pool.iter().position(|e| {
            e.source().point().distance(&end) <= tol || e.target().point().distance(&end) <= tol
        })?;
        let edge = pool.remove(next);
        if edge.source().point().distance(&end) <= tol {
            loop_edges.push(edge);
        } else {
            loop_edges.push(edge.reversed());
        }
    }
    let first = loop_edges.first().expect("seeded").source().point();
    let last = loop_edges.last().expect("seeded").target().point();
    if first.distance(&last) > tol {
        return None;
    }

    let inners: Vec<Wire> = caps.iter().flat_map(|cap| cap.inner_wires()).collect();
    Some(Face::with_wires(
        caps[seed_cap].surface().cloned(),
        Some(Wire::from_edges(loop_edges)),
        inners,
        caps[seed_cap].orientation(),
    ))
}

/// Position tolerance for corner-blend cross-checks, scaled to the fillet radius.
///
/// The corner constructors compare points they derive analytically (sphere/miter
/// tangents `T`/`F`/`R`/`K`) against geometry the blend already built. When the
/// GUI drives a fillet, those "already built" points are re-read from a quantized
/// tessellated mesh, so they drift from the exact analytic values by a few parts
/// in `1e4`. A fixed `1e-4` literal rejects genuine equal-radius perpendicular
/// corners under that drift — they then fall to the flat-trim crease (the user's
/// "becomes flat near the miter"). Scale the tolerance to the radius instead.
fn corner_tol(r: f64) -> f64 {
    (1e-4 * r.max(1.0)).max(10.0 * tolerance::CONFUSION)
}

/// The endpoint of `edge` nearest to `point`.
pub(crate) fn nearest_endpoint(edge: &Edge, point: Pnt) -> Pnt {
    let s = edge.source().point();
    let t = edge.target().point();
    if s.distance(&point) <= t.distance(&point) {
        s
    } else {
        t
    }
}

/// Round a corner where the new blend meets a cylinder left by an earlier fillet.
///
/// Geometry (a box's top-front-right corner with the top-front and top-right
/// edges blended, radius `r`): the rolling ball, pivoting in the corner, sits at
/// the point `C` tangent to all three faces (top, and the two sides). The corner
/// is a spherical octant of radius `r` about `C`, meeting:
/// - the prior cylinder along the great-circle arc T→F (so the prior cylinder is
///   *retracted* to end on that arc instead of overrunning to its own axis),
/// - the new blend cylinder along arc T→R (already built by the normal path),
/// - a small planar closure face along arc F→R, capping the still-sharp third
///   edge's stub.
///
/// `T`, `F`, `R` are the sphere's tangent points on the common (top), prior-side
/// and new-side faces. Returns `None` (caller falls back to the flat trim) unless
/// the corner is the supported convex, mutually-perpendicular, equal-radius case.
fn corner_sphere_blend(
    solid: &Solid,
    cap: &Face,
    corner: Pnt,
    blend: &RollingBallBlend,
    contact_a_corner: Pnt,
    contact_b_corner: Pnt,
    radius: f64,
) -> Option<CornerSphere> {
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    // Sphere center: the rolling-ball center at this corner.
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    // The cap must be a cylinder whose axis passes through the sphere center
    // (true for an equal-radius perpendicular meeting).
    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return None,
    };
    if point_line_distance(
        center,
        cyl.position().location(),
        cyl.position().direction(),
    ) > 1e-5 * r.max(1.0) + 1e-6
    {
        return None;
    }

    // The cap's outer loop must be the simple "two straight contacts + two end
    // arcs" of a single straight-edge fillet.
    let cap_edges = cap.outer_wire()?.edges();
    if cap_edges.len() != 4 {
        return None;
    }
    let is_arc = |e: &Edge| matches!(e.curve(), Some(GeomCurve::Circle(_)));

    // The corner-end arc of the cap: a circular edge with an endpoint at `corner`.
    let corner_arc_idx = cap_edges.iter().position(|e| {
        is_arc(e) && (near(e.source().point(), corner) || near(e.target().point(), corner))
    })?;
    let (arc_s, arc_t) = (
        cap_edges[corner_arc_idx].source().point(),
        cap_edges[corner_arc_idx].target().point(),
    );
    // The arc's other end sits on the cap's "side1" contact (the prior fillet's
    // other planar face); it is also the sharp corner stub's top vertex `K`.
    let other_corner = if near(arc_s, corner) { arc_t } else { arc_s };

    // The two straight contact edges and the planar faces they ride on.
    let straights: Vec<&Edge> = cap_edges.iter().filter(|e| !is_arc(e)).collect();
    if straights.len() != 2 {
        return None;
    }
    let top_contact = *straights
        .iter()
        .find(|e| near(e.source().point(), corner) || near(e.target().point(), corner))?;
    let side1_contact = *straights.iter().find(|e| {
        near(e.source().point(), other_corner) || near(e.target().point(), other_corner)
    })?;
    let common = adjacent_planar_face(solid, top_contact, cap)?;
    let side1 = adjacent_planar_face(solid, side1_contact, cap)?;

    // The common face must be one of the new blend's two faces; the other new
    // blend face is "side2".
    let side2 = if same_face(&common, &blend.face_a) {
        blend.face_b.clone()
    } else if same_face(&common, &blend.face_b) {
        blend.face_a.clone()
    } else {
        return None;
    };

    let n_top = planar_outward_normal(&common).ok()?;
    let n_side1 = planar_outward_normal(&side1).ok()?;
    let n_side2 = planar_outward_normal(&side2).ok()?;

    // Supported only for a convex, mutually-perpendicular trihedral corner.
    let perp = |a: Dir, b: Dir| GeomVec::from_dir(a).dot(&GeomVec::from_dir(b)).abs() < 1e-3;
    if !(perp(n_top, n_side1) && perp(n_top, n_side2) && perp(n_side1, n_side2)) {
        return None;
    }

    // Sphere tangent points on each face, and the sharp corner stub vertex K.
    let tangent = |n: Dir| center + GeomVec::from_dir(n) * r;
    let t = tangent(n_top);
    let f = tangent(n_side1);
    let rr = tangent(n_side2);
    let k = f + (rr - center);

    // Cross-checks against geometry the rest of the build already produced:
    // T/R must coincide with the new blend's two corner contacts, and K with the
    // cap's other corner vertex. Otherwise the corner is not the case we model.
    let xtol = corner_tol(r);
    let matches_a_or_b =
        |x: Pnt| x.distance(&contact_a_corner) < xtol || x.distance(&contact_b_corner) < xtol;
    if !matches_a_or_b(t) || !matches_a_or_b(rr) || k.distance(&other_corner) > xtol {
        return None;
    }

    // Retract the prior cylinder: its corner-end arc becomes the sphere arc T→F,
    // and its two contacts are shortened to T and F.
    let mut new_cap_edges = Vec::with_capacity(4);
    for (i, e) in cap_edges.iter().enumerate() {
        if i == corner_arc_idx {
            let s = if near(e.source().point(), corner) {
                t
            } else {
                f
            };
            let d = if near(e.target().point(), corner) {
                t
            } else {
                f
            };
            let a = arc_on_sphere(center, s, d, r).ok()?;
            new_cap_edges.push(orient_edge_between(&a, s, d));
        } else if is_arc(e) {
            new_cap_edges.push(e.clone());
        } else {
            let remap = |p: Pnt| {
                if near(p, corner) {
                    t
                } else if near(p, other_corner) {
                    f
                } else {
                    p
                }
            };
            new_cap_edges.push(Edge::between_points(
                remap(e.source().point()),
                remap(e.target().point()),
            ));
        }
    }
    let retracted_cap = Face::with_wires(
        cap.surface().cloned(),
        Some(Wire::from_edges(new_cap_edges)),
        cap.inner_wires(),
        cap.orientation(),
    );

    // The spherical octant T→F→R.
    let sphere_face = Face::new(
        Some(GeomSurface::sphere(SphericalSurface::new(
            Ax3::new_axes(center, n_top, n_side2),
            r,
        ))),
        Wire::from_edges([
            arc_on_sphere(center, t, f, r).ok()?,
            arc_on_sphere(center, f, rr, r).ok()?,
            arc_on_sphere(center, rr, t, r).ok()?,
        ]),
    );

    // The planar closure capping the sharp corner stub, bounded by F→K, K→R and
    // the sphere's R→F arc.
    let closure_face = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(f, n_top))),
        Wire::from_edges([
            Edge::between_points(f, k),
            Edge::between_points(k, rr),
            orient_edge_between(&arc_on_sphere(center, rr, f, r).ok()?, rr, f),
        ]),
    );

    // Split the prior fillet's side face so its contact edge ends at F (the rest
    // of that edge, F→K, is shared with the closure face).
    let side1_edges = side1.outer_wire()?.edges();
    let mut new_side1 = Vec::with_capacity(side1_edges.len() + 1);
    let mut split = false;
    for e in &side1_edges {
        if !split && same_undirected_edge(e, side1_contact) {
            let (s, d) = (e.source().point(), e.target().point());
            new_side1.push(Edge::between_points(s, f));
            new_side1.push(Edge::between_points(f, d));
            split = true;
        } else {
            new_side1.push(e.clone());
        }
    }
    if !split {
        return None;
    }
    let modified_side1 = Face::with_wires(
        side1.surface().cloned(),
        Some(Wire::from_edges(new_side1)),
        side1.inner_wires(),
        side1.orientation(),
    );

    Some(CornerSphere {
        new_faces: vec![retracted_cap, sphere_face, closure_face, modified_side1],
        skip_ids: vec![cap.id(), side1.id()],
    })
}

/// True when `cap` is a *concave* cylindrical wall (a bored/cut cylinder) whose
/// material lies *outside* the cylinder, as opposed to a *convex* prior-fillet
/// blend cylinder (material inside).
///
/// Face orientation alone does not discriminate (a convex blend cylinder can be
/// stored with the same winding as a cut wall), so we probe the solid directly:
/// at a point in the *interior* of the cap face, step a little radially outward
/// and inward and ask which side is material. A concave cut has material on the
/// *outside* of the cylinder; a convex boss on the inside.
pub(crate) fn is_concave_cut_cylinder(solid: &Solid, cap: &Face) -> bool {
    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return false,
    };
    let Some(wire) = cap.outer_wire() else {
        return false;
    };
    let verts: std::vec::Vec<Pnt> = wire.edges().iter().map(|e| e.source().point()).collect();
    if verts.is_empty() {
        return false;
    }
    let mut sum = GeomVec::new(0.0, 0.0, 0.0);
    for v in &verts {
        sum += *v - Pnt::origin();
    }
    let centroid = Pnt::origin() + sum * (1.0 / verts.len() as f64);
    let axis_pt = cyl.position().location();
    let axis_dir = GeomVec::from_dir(cyl.position().direction());
    let v = centroid - axis_pt;
    let along = axis_dir * v.dot(&axis_dir);
    let radial = match (v - along).normalized() {
        Some(r) => GeomVec::from_dir(r),
        None => return false,
    };
    let on_wall = axis_pt + along + radial * cyl.radius();
    let eps = (cyl.radius() * 0.05).max(10.0 * tolerance::CONFUSION);
    let outside = on_wall + radial * eps;
    let inside = on_wall - radial * eps;
    // Concave cut: material outside, void inside.
    crate::boolean::point_in_solid(&outside, solid)
        && !crate::boolean::point_in_solid(&inside, solid)
}

/// Intersect the infinite line `p0 + t·dir` with `cyl`, returning the hit point
/// nearest `near` (or `None` if the line misses the cylinder).
pub(crate) fn line_meets_cylinder(
    p0: Pnt,
    dir: Dir,
    cyl: &CylindricalSurface,
    near: Pnt,
) -> Option<Pnt> {
    let hits = line_cylinder_intersections(p0, dir, cyl)?;
    Some(if hits[0].distance(&near) <= hits[1].distance(&near) {
        hits[0]
    } else {
        hits[1]
    })
}

fn line_cylinder_intersections(p0: Pnt, dir: Dir, cyl: &CylindricalSurface) -> Option<[Pnt; 2]> {
    let axis_pt = cyl.position().location();
    let w = GeomVec::from_dir(cyl.position().direction());
    let d = GeomVec::from_dir(dir);
    // Components perpendicular to the cylinder axis.
    let e = p0 - axis_pt;
    let e_perp = e - w * e.dot(&w);
    let d_perp = d - w * d.dot(&w);
    let a = d_perp.dot(&d_perp);
    if a <= tolerance::CONFUSION {
        return None; // line parallel to axis: no transverse crossing
    }
    let b = 2.0 * e_perp.dot(&d_perp);
    let c = e_perp.dot(&e_perp) - cyl.radius() * cyl.radius();
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.max(0.0).sqrt();
    let t0 = (-b - sq) / (2.0 * a);
    let t1 = (-b + sq) / (2.0 * a);
    let p_at = |t: f64| p0 + d * t;
    Some([p_at(t0), p_at(t1)])
}

fn line_meets_cylinder_on_edge(
    p0: Pnt,
    dir: Dir,
    cyl: &CylindricalSurface,
    near: Pnt,
    edge: &Edge,
) -> Option<Pnt> {
    let hits = line_cylinder_intersections(p0, dir, cyl)?;
    let score = |p: Pnt| point_on_edge_score(edge, p) + 1.0e-7 * p.distance(&near);
    Some(if score(hits[0]) <= score(hits[1]) {
        hits[0]
    } else {
        hits[1]
    })
}

fn point_on_edge_score(edge: &Edge, point: Pnt) -> f64 {
    match edge.curve() {
        Some(GeomCurve::Line(_)) | None => {
            point_segment_distance(point, edge.source().point(), edge.target().point())
        }
        Some(GeomCurve::Circle(circle)) => {
            let center = circle.center();
            let axis = GeomVec::from_dir(circle.axis());
            let x = GeomVec::from_dir(circle.position().x_direction());
            let y = GeomVec::from_dir(circle.position().y_direction());
            let v = point - center;
            let axial = v.dot(&axis).abs();
            let radial_vec = v - axis * v.dot(&axis);
            let radial = radial_vec.magnitude();
            let raw = radial_vec.dot(&y).atan2(radial_vec.dot(&x));
            let lo = edge.first().min(edge.last());
            let hi = edge.first().max(edge.last());
            let mut param_penalty = f64::INFINITY;
            for k in -3..=3 {
                let u = raw + (k as f64) * core::f64::consts::TAU;
                let penalty = if u < lo {
                    (lo - u) * circle.radius()
                } else if u > hi {
                    (u - hi) * circle.radius()
                } else {
                    0.0
                };
                param_penalty = param_penalty.min(penalty);
            }
            axial + (radial - circle.radius()).abs() + param_penalty
        }
        Some(curve) => {
            let mut best = f64::INFINITY;
            for k in 0..=24 {
                let t = edge.first() + (edge.last() - edge.first()) * (k as f64) / 24.0;
                best = best.min(point.distance(&curve.point(t)));
            }
            best
        }
    }
}

/// A degree-1 (chorded) B-spline edge through `points`, from the first to the
/// last. Used for the cut-cylinder ∩ blend-cylinder trim curve, which has no
/// closed-form conic representation.
pub(crate) fn polyline_edge(points: &[Pnt]) -> Edge {
    use openrcad_geom::BSplineCurve;
    let n = points.len();
    let mut knots = vec![0.0];
    let mut mults = vec![2usize];
    let mut t = 0.0;
    for i in 1..n {
        t += points[i].distance(&points[i - 1]).max(1e-5);
        knots.push(t);
        mults.push(if i < n - 1 { 1 } else { 2 });
    }
    let curve = GeomCurve::bspline(BSplineCurve::new(1, points.to_vec(), None, knots, mults));
    Edge::new(
        Some(curve),
        0.0,
        t,
        Vertex::new(points[0]),
        Vertex::new(points[n - 1]),
    )
}

/// Sample the intersection of the `blend` cylinder with the `cut` cylinder
/// between contact points `p_a` (on the blend's A-side contact) and `p_b` (B-side),
/// returning a chorded B-spline edge that lies on *both* surfaces.
///
/// Parametrised by the blend cylinder's angle about its own axis (monotonic along
/// this quarter-arc of contact); for each angle the axial position is solved so
/// the point also lands on the cut cylinder.
fn cyl_cyl_trim_edge(
    blend: &CylindricalSurface,
    cut: &CylindricalSurface,
    p_a: Pnt,
    p_b: Pnt,
) -> Option<Edge> {
    let bl_pt = blend.position().location();
    let bl_axis = GeomVec::from_dir(blend.position().direction());
    let bx = GeomVec::from_dir(blend.position().x_direction());
    let by = GeomVec::from_dir(blend.position().y_direction());
    let r_b = blend.radius();

    // Angle of a point about the blend axis.
    let angle_of = |p: Pnt| -> f64 {
        let v = p - bl_pt;
        let vp = v - bl_axis * v.dot(&bl_axis);
        vp.dot(&by).atan2(vp.dot(&bx))
    };
    let mut phi_a = angle_of(p_a);
    let phi_b = angle_of(p_b);
    // Sweep the short way between the two contacts.
    use core::f64::consts::PI;
    while phi_a - phi_b > PI {
        phi_a -= 2.0 * PI;
    }
    while phi_b - phi_a > PI {
        phi_a += 2.0 * PI;
    }

    let cut_pt = cut.position().location();
    let cut_axis = GeomVec::from_dir(cut.position().direction());
    // For a fixed blend angle phi, the blend-cylinder point is base(phi) + s·bl_axis;
    // solve s so its distance from the cut axis equals the cut radius.
    let solve = |phi: f64, prev: Pnt| -> Option<Pnt> {
        let base = bl_pt + (bx * phi.cos() + by * phi.sin()) * r_b;
        let u = base - cut_pt;
        let u_perp = u - cut_axis * u.dot(&cut_axis);
        let d_perp = bl_axis - cut_axis * bl_axis.dot(&cut_axis);
        let a = d_perp.dot(&d_perp);
        if a <= tolerance::CONFUSION {
            return None;
        }
        let b = 2.0 * u_perp.dot(&d_perp);
        let c = u_perp.dot(&u_perp) - cut.radius() * cut.radius();
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        let s0 = (-b - sq) / (2.0 * a);
        let s1 = (-b + sq) / (2.0 * a);
        let q0 = base + bl_axis * s0;
        let q1 = base + bl_axis * s1;
        Some(if q0.distance(&prev) <= q1.distance(&prev) {
            q0
        } else {
            q1
        })
    };

    // Sample densely enough that the chorded curve hugs both cylinders to well
    // under a render tolerance (≈0.05 mm chords), so the cut wall reads as a
    // clean cylinder rather than a faceted boundary.
    let steps = ((p_a.distance(&p_b) / 0.05).ceil() as usize).clamp(24, 160);
    let mut pts = Vec::with_capacity(steps + 1);
    pts.push(p_a);
    let mut prev = p_a;
    for k in 1..steps {
        let phi = phi_a + (phi_b - phi_a) * (k as f64) / (steps as f64);
        let p = solve(phi, prev)?;
        prev = p;
        pts.push(p);
    }
    pts.push(p_b);
    Some(polyline_edge(&pts))
}

/// Trim a selected-edge fillet into an extruded sketch arc that is tangent to
/// one of the selected edge's planar side faces.
///
/// This is distinct from [`try_corner_cut`]: the wall is convex material
/// boundary, not a void. One blend contact may therefore shorten back along the
/// selected edge until it reaches the wall, while the other contact can end at
/// the original tangent point. The shared endpoint edge is still the true
/// cylinder-cylinder intersection between the blend cylinder and the wall
/// cylinder, so both surfaces keep analytic support and sew without cracks.
#[allow(clippy::too_many_arguments)]
fn try_tangent_curved_wall_runout(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<bool, RollingBallError> {
    if caps.len() != 1 {
        return Ok(false);
    }
    let cap = &caps[0];
    if is_concave_cut_cylinder(solid, cap) {
        return Ok(false);
    }
    let wall_cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Ok(false),
    };
    let blend_cyl = match blend.blend_face.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Ok(false),
    };

    // Prior fillets are also cylindrical endpoint caps. They are handled by the
    // miter/sphere paths when the cap axis passes through the rolling-ball center
    // at this corner. A tangent sketch wall sits away from that center.
    let center = nearest_endpoint(&blend.centerline, corner);
    if point_line_distance(
        center,
        wall_cyl.position().location(),
        wall_cyl.position().direction(),
    ) <= 1.0e-5 * radius.max(1.0) + 1.0e-6
    {
        return Ok(false);
    }
    // Two supported wall relations:
    // - TANGENT: an extruded sketch arc tangent to one of the blend's side
    //   faces (the original runout case).
    // - CROSSING: the wall rises straight through the blend's path — e.g. a
    //   boss standing on the filleted edge's face; the blend dives into the
    //   wall and is trimmed flush against it. Both share the same analytic
    //   construction (contact lines extended to the wall, cyl∩cyl trim curve);
    //   they differ only in how the wall is recognized.
    if !wall_is_tangent_to_selected_side(cap, &wall_cyl, blend, corner)?
        && !wall_crosses_blend_at_corner(cap, &wall_cyl, corner)
    {
        return Ok(false);
    }

    let a_corner = nearest_endpoint(&blend.contact_a, corner);
    let b_corner = nearest_endpoint(&blend.contact_b, corner);
    let a_far = farthest_endpoint(&blend.contact_a, corner);
    let b_far = farthest_endpoint(&blend.contact_b, corner);
    let a_dir = match (a_corner - a_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };
    let b_dir = match (b_corner - b_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };

    let (prev_edge, next_edge) = cap_edges_at_corner(cap, corner)?;
    let da_prev = point_on_edge_score(&prev_edge, a_corner);
    let db_prev = point_on_edge_score(&prev_edge, b_corner);
    let (a_trim_edge, b_trim_edge) = if da_prev <= db_prev {
        (&prev_edge, &next_edge)
    } else {
        (&next_edge, &prev_edge)
    };

    let ca_real = match line_meets_cylinder_on_edge(a_far, a_dir, &wall_cyl, a_corner, a_trim_edge)
    {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let cb_real = match line_meets_cylinder_on_edge(b_far, b_dir, &wall_cyl, b_corner, b_trim_edge)
    {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    if ca_real.distance(&cb_real) <= tolerance::CONFUSION {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let trim = match cyl_cyl_trim_edge(&blend_cyl, &wall_cyl, ca_real, cb_real) {
        Some(e) => e,
        None => {
            return Err(RollingBallError::BlendSurfaceBuild(
                "tangent wall trim curve",
            ))
        }
    };
    let trimmed_cap = trim_face_at_corner(cap, corner, ca_real, cb_real, &trim)?;

    let new_a = rebuild_contact(&blend.contact_a, a_far, ca_real);
    let new_b = rebuild_contact(&blend.contact_b, b_far, cb_real);
    let is_end = new_a.end().point().distance(&ca_real) <= new_a.start().point().distance(&ca_real);
    let new_blend_face = build_blend_face_with_trim(blend, &new_a, &new_b, &trim, is_end);

    blend.contact_a = new_a;
    blend.contact_b = new_b;
    blend.blend_face = new_blend_face;
    // Persist the trim as this end's cap arc: when the OTHER endpoint is also
    // flush-trimmed, its rebuild re-reads this end's arc from the blend, and a
    // stale perpendicular arc would resurrect pre-extension contact endpoints
    // (a non-contiguous blend loop).
    if is_end {
        blend.end_arc = trim;
    } else {
        blend.start_arc = trim;
    }
    faces.push(trimmed_cap);
    skipped.insert(cap.id());
    Ok(true)
}

fn cap_edges_at_corner(face: &Face, corner: Pnt) -> Result<(Edge, Edge), RollingBallError> {
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let Some(next_idx) = edges
        .iter()
        .position(|edge| edge.source().point().distance(&corner) <= 10.0 * tolerance::CONFUSION)
    else {
        return Err(RollingBallError::UnsupportedTrimTopology);
    };
    let prev_idx = (next_idx + n - 1) % n;
    if edges[prev_idx].target().point().distance(&corner) > 10.0 * tolerance::CONFUSION {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    Ok((edges[prev_idx].clone(), edges[next_idx].clone()))
}

/// True when the endpoint `corner` sits ON the wall cylinder's surface and the
/// wall face actually owns it — the selected edge runs INTO the wall, so the
/// blend must terminate flush against it (the crossing counterpart of
/// [`wall_is_tangent_to_selected_side`]).
fn wall_crosses_blend_at_corner(cap: &Face, wall_cyl: &CylindricalSurface, corner: Pnt) -> bool {
    let axis_pt = wall_cyl.position().location();
    let axis = GeomVec::from_dir(wall_cyl.position().direction());
    let v = corner - axis_pt;
    let radial = v - axis * v.dot(&axis);
    if (radial.magnitude() - wall_cyl.radius()).abs() > 1.0e-4 * wall_cyl.radius().max(1.0) {
        return false;
    }
    face_contains_point(cap, corner)
}

fn wall_is_tangent_to_selected_side(
    cap: &Face,
    wall_cyl: &CylindricalSurface,
    blend: &RollingBallBlend,
    corner: Pnt,
) -> Result<bool, RollingBallError> {
    let axis_pt = wall_cyl.position().location();
    let axis = GeomVec::from_dir(wall_cyl.position().direction());
    let v = corner - axis_pt;
    let radial = v - axis * v.dot(&axis);
    let Some(radial_dir) = radial.normalized() else {
        return Ok(false);
    };
    let radial_vec = GeomVec::from_dir(radial_dir);
    if (radial.magnitude() - wall_cyl.radius()).abs() > 1.0e-4 * wall_cyl.radius().max(1.0) {
        return Ok(false);
    }

    let tangent_to = |face: &Face| -> Result<bool, RollingBallError> {
        let n = planar_outward_normal(face)?;
        Ok(radial_vec.dot(&GeomVec::from_dir(n)).abs() > 0.999)
    };

    let tangent = tangent_to(&blend.face_a)? || tangent_to(&blend.face_b)?;
    if !tangent {
        return Ok(false);
    }

    // Require the cap to actually own the endpoint; otherwise a same-support
    // cylinder elsewhere in the shell could be mistaken for the runout wall.
    Ok(face_contains_point(cap, corner))
}

/// Trim the new blend flush against a concave cut cylinder it runs into at
/// `corner`, leaving the cut a clean full-height vertical cylinder.
///
/// The fillet's quarter-circle end cap (which lies on the *blend* cylinder, not
/// the cut) is replaced by the true blend ∩ cut intersection curve. The blend's
/// two contact curves are extended to where they actually meet the cut wall, and
/// the cut face is re-trimmed to share that same intersection edge — so its
/// boundary stays on the cylinder and the seam is watertight by construction.
/// Mutates `blend` (extended contacts + rebuilt blend face) so the later spine
/// trims pick up the extension, mirroring [`try_corner_miter`]. Returns
/// `Ok(false)` before a concave cut cylinder is recognized. Once one is
/// recognized, failures are reported as clean errors instead of falling back to a
/// flat endpoint cap that cannot share a valid edge with the cut wall.
#[allow(clippy::too_many_arguments)]
fn try_corner_cut(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    _radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<bool, RollingBallError> {
    if caps.len() != 1 {
        return Ok(false);
    }
    let cap = &caps[0];
    if !is_concave_cut_cylinder(solid, cap) {
        return Ok(false);
    }
    let cut_cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let blend_cyl = match blend.blend_face.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        // A non-cylindrical blend (e.g. a torus rim fillet) has no analytic
        // cylinder∩cylinder flush-trim yet. Decline the flush-trim rather than
        // abort the whole fillet (`?` at the call sites): the caller then falls
        // back to `handle_corner_endpoint`'s planar-cap trim. Cylindrical blends
        // are unaffected.
        _ => return Ok(false),
    };

    // Which end of each contact runs into the cut.
    let a_corner = nearest_endpoint(&blend.contact_a, corner);
    let b_corner = nearest_endpoint(&blend.contact_b, corner);
    let a_far = farthest_endpoint(&blend.contact_a, corner);
    let b_far = farthest_endpoint(&blend.contact_b, corner);
    let a_dir = match (a_corner - a_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };
    let b_dir = match (b_corner - b_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };

    // Where each contact line actually meets the cut wall (may extend past the
    // spine endpoint when the cut boundary curves away).
    let ca_real = match line_meets_cylinder(a_far, a_dir, &cut_cyl, a_corner) {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let cb_real = match line_meets_cylinder(b_far, b_dir, &cut_cyl, b_corner) {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };

    // The shared trim curve, lying on both cylinders.
    let trim = match cyl_cyl_trim_edge(&blend_cyl, &cut_cyl, ca_real, cb_real) {
        Some(e) => e,
        None => return Err(RollingBallError::BlendSurfaceBuild("cut trim curve")),
    };

    // Re-trim the cut face: shorten its rim/wall edges at `corner` to the real
    // contacts and splice in the trim curve (reusing the corner trimmer).
    let trimmed_cap = trim_face_at_corner(cap, corner, ca_real, cb_real, &trim)?;

    // Extend the blend's contacts to the real meet points and rebuild the blend
    // face with the trim curve replacing this end's cap arc. Build everything as
    // locals first, then commit atomically so a fallback can't see a half-mutated
    // blend.
    let new_a = rebuild_contact(&blend.contact_a, a_far, ca_real);
    let new_b = rebuild_contact(&blend.contact_b, b_far, cb_real);
    // The cut is at the contacts' *end* if ca_real sits at their end vertex.
    let is_end = new_a.end().point().distance(&ca_real) <= new_a.start().point().distance(&ca_real);
    let new_blend_face = build_blend_face_with_trim(blend, &new_a, &new_b, &trim, is_end);

    blend.contact_a = new_a;
    blend.contact_b = new_b;
    blend.blend_face = new_blend_face;
    // Persist the trim as this end's cap arc (see try_tangent_curved_wall_runout).
    if is_end {
        blend.end_arc = trim;
    } else {
        blend.start_arc = trim;
    }
    faces.push(trimmed_cap);
    skipped.insert(cap.id());
    Ok(true)
}

/// Trim the blend flush against a single planar cap that meets the filleted
/// edge at an OBLIQUE angle (any angle other than perpendicular) — e.g. the
/// end face of a line-drawn triangle prism. The flat corner trim only handles
/// perpendicular caps (the blend's circular end arc lies in the cap plane);
/// here the true termination curve is the ELLIPSE where the blend cylinder
/// crosses the cap plane. The blend's straight contacts are extended/shortened
/// to their cap-plane crossings, the end arc is replaced by the ellipse arc
/// between them, and the cap face is re-trimmed to share that same edge.
/// Returns `Ok(false)` for any unsupported configuration (perpendicular caps,
/// non-cylindrical blends, a cap parallel to the spine).
#[allow(clippy::too_many_arguments)]
fn try_oblique_planar_cap(
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    _radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<bool, RollingBallError> {
    if caps.len() != 1 {
        return Ok(false);
    }
    let cap = &caps[0];
    let Some(GeomSurface::Plane(pl)) = cap.surface() else {
        return Ok(false);
    };
    let pl = *pl;
    let Some(GeomSurface::Cylinder(blend_cyl)) = blend.blend_face.surface() else {
        return Ok(false);
    };
    let blend_cyl = *blend_cyl;

    let m = GeomVec::from_dir(pl.normal());
    let q0 = pl.position().location();
    let d = GeomVec::from_dir(blend_cyl.position().direction());
    let cos_phi = d.dot(&m);
    // Perpendicular cap: the existing flat trim already handles it (the end
    // arc IS the plane section). Near-parallel cap: no bounded crossing.
    if cos_phi.abs() > 0.999 || cos_phi.abs() < 1.0e-3 {
        return Ok(false);
    }

    let a_corner = nearest_endpoint(&blend.contact_a, corner);
    let b_corner = nearest_endpoint(&blend.contact_b, corner);
    let a_far = farthest_endpoint(&blend.contact_a, corner);
    let b_far = farthest_endpoint(&blend.contact_b, corner);
    let toward = |far: Pnt, near: Pnt| (near - far).normalized();
    let (Some(a_dir), Some(b_dir)) = (toward(a_far, a_corner), toward(b_far, b_corner)) else {
        return Err(RollingBallError::DegenerateSpine);
    };
    let cross = |far: Pnt, dir: GeomVec| -> Option<Pnt> {
        let denom = dir.dot(&m);
        if denom.abs() < 1.0e-9 {
            return None;
        }
        let t = (q0 - far).dot(&m) / denom;
        (t > tolerance::CONFUSION).then(|| far + dir * t)
    };
    let (Some(pa), Some(pb)) = (
        cross(a_far, GeomVec::from_dir(a_dir)),
        cross(b_far, GeomVec::from_dir(b_dir)),
    ) else {
        return Ok(false);
    };
    if pa.distance(&pb) <= tolerance::CONFUSION {
        return Ok(false);
    }

    // Ellipse: the blend cylinder's section by the cap plane. Center at the
    // axis crossing, major axis along the spine's in-plane projection with
    // semi-major r/|cos φ|, semi-minor r along the mutual perpendicular.
    let c_far = farthest_endpoint(&blend.centerline, corner);
    let c_near = nearest_endpoint(&blend.centerline, corner);
    let Some(c_dir) = toward(c_far, c_near) else {
        return Err(RollingBallError::DegenerateSpine);
    };
    let Some(e_center) = cross(c_far, GeomVec::from_dir(c_dir)) else {
        return Ok(false);
    };
    let Some(x_dir) = (d - m * cos_phi).normalized() else {
        return Ok(false);
    };
    let x_vec = GeomVec::from_dir(x_dir);
    let y_vec = m.cross(&x_vec);
    let a_r = blend.radius / cos_phi.abs();
    let b_r = blend.radius;
    let param_of = |p: Pnt| -> f64 {
        let v = p - e_center;
        (v.dot(&y_vec) / b_r).atan2(v.dot(&x_vec) / a_r)
    };
    let ta = param_of(pa);
    let tb = param_of(pb);

    // Two candidate arcs join the crossings; keep the one on the blend band's
    // own side, referenced by the original (perpendicular) end arc's midpoint
    // direction about the blend axis.
    let old_arc = if corner.distance(&blend.spine.source().point())
        <= corner.distance(&blend.spine.target().point())
    {
        &blend.start_arc
    } else {
        &blend.end_arc
    };
    let ref_mid = old_arc
        .curve()
        .map(|c| c.point(0.5 * (old_arc.first() + old_arc.last())))
        .unwrap_or_else(|| old_arc.start().point());
    let axis_pt = blend_cyl.position().location();
    let radial_of = |p: Pnt| -> Option<GeomVec> {
        let v = p - axis_pt;
        (v - d * v.dot(&d)).normalized().map(GeomVec::from_dir)
    };
    let Some(ref_dir) = radial_of(ref_mid) else {
        return Ok(false);
    };
    use core::f64::consts::TAU;
    let ccw_span = {
        let mut s = tb - ta;
        while s <= 0.0 {
            s += TAU;
        }
        s
    };
    let m_dir = Dir::new(m.x(), m.y(), m.z());
    let ell = Ellipse::new(Ax3::new_axes(e_center, m_dir, x_dir), a_r, b_r);
    let score = |t_mid: f64| -> f64 {
        let p = ell.point(t_mid);
        radial_of(p).map_or(f64::NEG_INFINITY, |r| r.dot(&ref_dir))
    };
    let ccw_mid = ta + 0.5 * ccw_span;
    let cw_mid = ta - 0.5 * (TAU - ccw_span);
    let t_end = if score(ccw_mid) >= score(cw_mid) {
        ta + ccw_span
    } else {
        ta - (TAU - ccw_span)
    };
    let trim = Edge::new(
        Some(GeomCurve::ellipse(ell)),
        ta,
        t_end,
        Vertex::new(pa),
        Vertex::new(pb),
    );

    let Ok(trimmed_cap) = trim_face_at_corner(cap, corner, pa, pb, &trim) else {
        return Ok(false);
    };

    let new_a = rebuild_contact(&blend.contact_a, a_far, pa);
    let new_b = rebuild_contact(&blend.contact_b, b_far, pb);
    let is_end = new_a.end().point().distance(&pa) <= new_a.start().point().distance(&pa);
    let new_blend_face = build_blend_face_with_trim(blend, &new_a, &new_b, &trim, is_end);

    blend.contact_a = new_a;
    blend.contact_b = new_b;
    blend.blend_face = new_blend_face;
    // Persist the trim as this end's cap arc (see try_tangent_curved_wall_runout).
    if is_end {
        blend.end_arc = trim;
    } else {
        blend.start_arc = trim;
    }
    faces.push(trimmed_cap);
    skipped.insert(cap.id());
    Ok(true)
}

/// The endpoint of `edge` farthest from `point`.
pub(crate) fn farthest_endpoint(edge: &Edge, point: Pnt) -> Pnt {
    let s = edge.source().point();
    let t = edge.target().point();
    if s.distance(&point) >= t.distance(&point) {
        s
    } else {
        t
    }
}

/// Rebuild a straight contact edge keeping its `keep` endpoint and moving the
/// other to `moved`.
fn rebuild_contact(edge: &Edge, keep: Pnt, moved: Pnt) -> Edge {
    // `keep`/`moved` are passed as the far/real points; map them onto the edge's
    // current source→target order so the contact direction is preserved.
    let s = edge.source().point();
    if s.distance(&keep) <= s.distance(&moved) {
        Edge::between_points(keep, moved)
    } else {
        Edge::between_points(moved, keep)
    }
}

/// Build the blend face's wire from the given contacts, replacing the end
/// (`is_end`) or start cap arc with the `trim` curve. The blend cylinder surface
/// is unchanged. The blend wire is always
/// `[contact_a, end_arc, contact_b.reversed(), start_arc]`.
fn build_blend_face_with_trim(
    blend: &RollingBallBlend,
    ca: &Edge,
    cb: &Edge,
    trim: &Edge,
    is_end: bool,
) -> Face {
    let end_src = if is_end { trim } else { &blend.end_arc };
    let start_src = if is_end { &blend.start_arc } else { trim };
    let end_edge = orient_edge_between(end_src, ca.end().point(), cb.end().point());
    let start_edge = orient_edge_between(start_src, cb.start().point(), ca.start().point());
    let wire = Wire::from_edges([ca.clone(), end_edge, cb.clone().reversed(), start_edge]);
    rebuild_face(&blend.blend_face, wire).expect("blend face rebuild is infallible")
}

/// Round a corner where the new blend meets exactly *one* earlier fillet's
/// cylinder and the corner's third edge is still sharp — the Fusion-style miter.
///
/// Two equal-radius perpendicular fillets meeting at a corner whose third edge is
/// NOT filleted must meet along their mutual intersection *seam* (a quarter-ellipse
/// running from the shared top tangent point `T` down to the sharp corner stub
/// vertex `K`), leaving that third edge a crisp edge running down from `K`. This
/// is distinct from the spherical ball-corner, which only belongs when all three
/// edges at the corner are rounded.
///
/// On the supported convex, mutually-perpendicular, equal-radius case this:
/// - retracts the prior cylinder cap so its corner end follows the seam (its top
///   contact shortens to `T`; its side contact already ends at `K`), pushing the
///   retracted cap and skipping the original,
/// - extends the new blend toward `K` and swaps its corner end arc for the seam,
///   mutating `blend` so the later spine trims pick up the extension,
/// - leaves the third edge's two side faces untouched (no closure, no sphere),
///
/// and returns `true`. Returns `false` (caller falls back to the sphere / flat
/// corner trim) for any unsupported corner.
#[allow(clippy::too_many_arguments)]
fn try_corner_miter(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> bool {
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    if caps.len() != 1 {
        return false;
    }
    let cap = &caps[0];
    if !matches!(
        cap.surface(),
        Some(GeomSurface::Cylinder(_)) | Some(GeomSurface::Torus(_))
    ) {
        return false;
    }

    // The corner's two new-blend contacts (nearest endpoint of each contact curve).
    let contact_a_corner = nearest_endpoint(&blend.contact_a, corner);
    let contact_b_corner = nearest_endpoint(&blend.contact_b, corner);

    // --- Corner frame: the rolling-ball center at this corner and the cap's
    // contact/arc layout (the same analysis `corner_sphere_blend` performs). ---
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return false,
    };
    if point_line_distance(
        center,
        cyl.position().location(),
        cyl.position().direction(),
    ) > 1e-5 * r.max(1.0) + 1e-6
    {
        return false;
    }

    let Some(cap_wire) = cap.outer_wire() else {
        return false;
    };
    let cap_edges = cap_wire.edges();
    if cap_edges.len() != 4 {
        return false;
    }
    let is_arc = |e: &Edge| matches!(e.curve(), Some(GeomCurve::Circle(_)));

    let Some(corner_arc_idx) = cap_edges.iter().position(|e| {
        is_arc(e) && (near(e.source().point(), corner) || near(e.target().point(), corner))
    }) else {
        return false;
    };
    let (arc_s, arc_t) = (
        cap_edges[corner_arc_idx].source().point(),
        cap_edges[corner_arc_idx].target().point(),
    );
    // The arc's far end is the sharp corner stub vertex `K`.
    let other_corner = if near(arc_s, corner) { arc_t } else { arc_s };

    // The cap's two contacts are its straight (line) edges. Select Lines
    // explicitly: a cap already mitered at its *other* end carries an elliptical
    // seam edge, which `!is_arc` (= "not a circle") would wrongly count here.
    let straights: Vec<&Edge> = cap_edges
        .iter()
        .filter(|e| matches!(e.curve(), Some(GeomCurve::Line(_))))
        .collect();
    if straights.len() != 2 {
        return false;
    }
    let Some(&top_contact) = straights
        .iter()
        .find(|e| near(e.source().point(), corner) || near(e.target().point(), corner))
    else {
        return false;
    };
    let Some(&side1_contact) = straights
        .iter()
        .find(|e| near(e.source().point(), other_corner) || near(e.target().point(), other_corner))
    else {
        return false;
    };
    let Some(common) = adjacent_planar_face(solid, top_contact, cap) else {
        return false;
    };
    let Some(side1) = adjacent_planar_face(solid, side1_contact, cap) else {
        return false;
    };

    let side2 = if same_face(&common, &blend.face_a) {
        blend.face_b.clone()
    } else if same_face(&common, &blend.face_b) {
        blend.face_a.clone()
    } else {
        return false;
    };

    let (Ok(n_top), Ok(n_side1), Ok(n_side2)) = (
        planar_outward_normal(&common),
        planar_outward_normal(&side1),
        planar_outward_normal(&side2),
    ) else {
        return false;
    };

    let perp = |a: Dir, b: Dir| GeomVec::from_dir(a).dot(&GeomVec::from_dir(b)).abs() < 1e-3;
    if !(perp(n_top, n_side1) && perp(n_top, n_side2) && perp(n_side1, n_side2)) {
        return false;
    }

    // Seam endpoints: `T` (top tangent), `R` (new-blend side tangent), `K` (stub).
    let t = center + GeomVec::from_dir(n_top) * r;
    let rr = center + GeomVec::from_dir(n_side2) * r;
    let k = center + (GeomVec::from_dir(n_side1) + GeomVec::from_dir(n_side2)) * r;

    // Cross-check against the geometry the rest of the build already produced: `T`
    // and `R` are the new blend's two corner contacts, and `K` the cap stub vertex.
    let xtol = corner_tol(r);
    let matches_a_or_b =
        |x: Pnt| x.distance(&contact_a_corner) < xtol || x.distance(&contact_b_corner) < xtol;
    if !matches_a_or_b(t) || !matches_a_or_b(rr) || k.distance(&other_corner) > xtol {
        return false;
    }

    // The miter seam: cylinder cap ∩ new blend, a quarter-ellipse `K -> T`.
    let Some(seam) = miter_seam_edge(center, n_top, n_side1, n_side2, r) else {
        return false;
    };

    // Retract the prior cap: corner arc -> seam (T -> K), top contact -> T, side
    // contact already ends at K. Curve edges other than the corner arc (the far
    // end arc, or an elliptical seam from an earlier miter at the cap's other end)
    // are preserved verbatim — only the straight contacts are remapped.
    let mut new_cap_edges = Vec::with_capacity(4);
    for (i, e) in cap_edges.iter().enumerate() {
        if i == corner_arc_idx {
            let s = if near(e.source().point(), corner) {
                t
            } else {
                k
            };
            let d = if near(e.target().point(), corner) {
                t
            } else {
                k
            };
            new_cap_edges.push(orient_edge_between(&seam, s, d));
        } else if !matches!(e.curve(), Some(GeomCurve::Line(_))) {
            new_cap_edges.push(e.clone());
        } else {
            let remap = |p: Pnt| if near(p, corner) { t } else { p };
            new_cap_edges.push(Edge::between_points(
                remap(e.source().point()),
                remap(e.target().point()),
            ));
        }
    }
    let retracted_cap = Face::with_wires(
        cap.surface().cloned(),
        Some(Wire::from_edges(new_cap_edges)),
        cap.inner_wires(),
        cap.orientation(),
    );

    // Extend the new blend toward `K`: the contact on `side2` (corner point == R)
    // is lengthened so it reaches the stub vertex; the contact on `common` (corner
    // point == T) is unchanged.
    let a_is_side2 = contact_a_corner.distance(&rr) < 1e-4;
    if a_is_side2 {
        blend.contact_a = extend_contact_corner(&blend.contact_a, corner, k);
    } else {
        blend.contact_b = extend_contact_corner(&blend.contact_b, corner, k);
    }

    // Swap the new blend's corner-side end arc for the seam, keeping the blend_face
    // wire [contact_a, end_arc, contact_b.reversed(), start_arc] contiguous.
    let corner_is_source = corner.distance(&blend.spine.source().point())
        <= corner.distance(&blend.spine.target().point());
    if corner_is_source {
        let b0 = blend.contact_b.source().point();
        let a0 = blend.contact_a.source().point();
        blend.start_arc = orient_edge_between(&seam, b0, a0);
    } else {
        let a1 = blend.contact_a.target().point();
        let b1 = blend.contact_b.target().point();
        blend.end_arc = orient_edge_between(&seam, a1, b1);
    }
    blend.blend_face = Face::new(
        blend.blend_face.surface().cloned(),
        Wire::from_edges([
            blend.contact_a.clone(),
            blend.end_arc.clone(),
            blend.contact_b.clone().reversed(),
            blend.start_arc.clone(),
        ]),
    );

    faces.push(retracted_cap);
    skipped.insert(cap.id());
    true
}

/// The miter seam between two equal-radius perpendicular fillets meeting at a
/// corner with center `center`: the quarter-ellipse where their two cylinders
/// intersect, running from the stub vertex `K = center + r(n_side1 + n_side2)`
/// (parameter 0) to the shared top tangent `T = center + r·n_top` (parameter π/2).
///
/// The intersection of two equal-radius cylinders whose axes cross at right angles
/// is a planar ellipse with semi-axes `r√2` (along `n_side1 + n_side2`) and `r`
/// (along `n_top`).
fn miter_seam_edge(center: Pnt, n_top: Dir, n_side1: Dir, n_side2: Dir, r: f64) -> Option<Edge> {
    use core::f64::consts::{FRAC_PI_2, SQRT_2};
    let to_dir = |v: GeomVec| v.normalized().map(|d| Dir::new(d.x(), d.y(), d.z()));
    let major_vec = GeomVec::from_dir(n_side1) + GeomVec::from_dir(n_side2);
    let major_dir = to_dir(major_vec)?;
    let normal = to_dir(GeomVec::from_dir(major_dir).cross(&GeomVec::from_dir(n_top)))?;
    let pos = Ax3::new_axes(center, normal, major_dir);
    let ell = Ellipse::new(pos, r * SQRT_2, r);
    let k = center + major_vec * r;
    let t = center + GeomVec::from_dir(n_top) * r;
    Some(Edge::new(
        Some(GeomCurve::ellipse(ell)),
        0.0,
        FRAC_PI_2,
        Vertex::new(k),
        Vertex::new(t),
    ))
}

/// Rebuild straight contact `contact` with the endpoint nearest `corner` moved to
/// `new_pt` (lengthening the new blend's side contact to reach the stub vertex).
fn extend_contact_corner(contact: &Edge, corner: Pnt, new_pt: Pnt) -> Edge {
    let s = contact.source().point();
    let t = contact.target().point();
    if s.distance(&corner) <= t.distance(&corner) {
        Edge::between_points(new_pt, t)
    } else {
        Edge::between_points(s, new_pt)
    }
}

/// Miter a new STRAIGHT fillet into the toroidal band of an earlier same-radius
/// CIRCULAR-rim fillet ending at this corner — the reverse order of
/// `blend_open_circular_chain`'s band-cylinder miter, producing the identical
/// seam. The straight edge was shortened by the earlier fillet, so its corner
/// vertex IS the torus band's old flush-end vertex; the new blend's contacts
/// are extended past it (into the region the earlier fillet vacated) to the
/// band∩band intersection seam, the corner end arc is replaced by that seam,
/// and the torus band is retracted to it. Returns `false` (caller falls back)
/// for any unsupported configuration.
fn try_corner_circular_band_miter(
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> bool {
    if caps.len() != 1 {
        return false;
    }
    let cap = &caps[0];
    let Some(GeomSurface::Torus(tor)) = cap.surface() else {
        return false;
    };
    let tor = *tor;
    if (tor.minor_radius() - radius).abs() > corner_tol(radius) {
        return false;
    }
    let Some(GeomSurface::Cylinder(bcyl)) = blend.blend_face.surface() else {
        return false;
    };
    let bcyl = *bcyl;

    let (Ok(n_a), Ok(n_b)) = (
        planar_outward_normal(&blend.face_a),
        planar_outward_normal(&blend.face_b),
    ) else {
        return false;
    };

    // Which of the blend's two support planes is the one the torus band is
    // tangent to (its tube-centre circle sits `radius` below it, along its own
    // axis)? That plane is the shared "top"; the other is the side wall.
    let m = GeomVec::from_dir(tor.position().direction());
    let ctr = tor.position().location();
    let plane_origin = |f: &Face| match f.surface() {
        Some(GeomSurface::Plane(pl)) => Some(pl.position().location()),
        _ => None,
    };
    let is_top = |n: Dir, f: &Face| -> bool {
        if GeomVec::from_dir(n).dot(&m).abs() < 0.999 {
            return false;
        }
        let Some(p0) = plane_origin(f) else {
            return false;
        };
        ((ctr - p0).dot(&GeomVec::from_dir(n)) + radius).abs() <= corner_tol(radius)
    };
    let a_is_top = is_top(n_a, &blend.face_a);
    let b_is_top = is_top(n_b, &blend.face_b);
    if a_is_top == b_is_top {
        return false;
    }
    let (n_top, n_side) = if a_is_top { (n_a, n_b) } else { (n_b, n_a) };

    // Signed distance to the torus tube-centre circle, minus the tube radius:
    // zero exactly on the band surface.
    let rr = tor.major_radius();
    let g_tor = |p: Pnt| -> f64 {
        let v = p - ctr;
        let h = v.dot(&m);
        let rho = (v - m * h).magnitude();
        ((rho - rr).powi(2) + h * h).sqrt() - radius
    };

    // Profile rails of the new straight band: the blend cylinder's axis line,
    // offset `radius` along the arc of directions from `n_top` to `n_side`.
    let na = GeomVec::from_dir(n_top);
    let nb = GeomVec::from_dir(n_side);
    let th = na.dot(&nb).clamp(-1.0, 1.0).acos();
    if th < 0.3 || th.sin() < 1e-6 {
        return false;
    }
    let u_at = |f: f64| (na * ((1.0 - f) * th).sin() + nb * (f * th).sin()) * (1.0 / th.sin());
    let ap = bcyl.position().location();
    let ad = GeomVec::from_dir(bcyl.position().direction());
    let rail_pt = |f: f64, t: f64| ap + ad * t + u_at(f) * radius;

    // Marching direction: beyond the (shortened) edge's corner end.
    let far = farthest_endpoint(&blend.spine, corner);
    let t_corner = (corner - ap).dot(&ad);
    let t_far = (far - ap).dot(&ad);
    let side_sign = if t_corner >= t_far { 1.0 } else { -1.0 };

    // Station 0 lies in the shared top plane, where the two bands are TANGENT:
    // the crossing is the minimum of the distance, not a sign change.
    let (w_lo, w_hi) = (
        t_corner - side_sign * 0.6 * radius,
        t_corner + side_sign * 2.5 * radius,
    );
    let t_a = minimize_scalar(|t| g_tor(rail_pt(0.0, t)).abs(), w_lo.min(w_hi), w_lo.max(w_hi));
    if g_tor(rail_pt(0.0, t_a)).abs() > 1e-4 * radius.max(1.0) {
        return false;
    }

    let steps = ((radius * 2.0 / 0.05).ceil() as usize).clamp(24, 160);
    let mut pts = vec![rail_pt(0.0, t_a)];
    let mut prev_t = t_a;
    for k in 1..=steps {
        let f = k as f64 / steps as f64;
        let Some(t) = directional_root(
            |t| g_tor(rail_pt(f, t)),
            prev_t,
            side_sign,
            0.6 * radius,
        ) else {
            return false;
        };
        if (t - prev_t).abs() > 0.4 * radius {
            return false;
        }
        prev_t = t;
        pts.push(rail_pt(f, t));
    }
    let a_pt = pts[0];
    let b_pt = *pts.last().expect("seam has points");
    if a_pt.distance(&corner) > 3.0 * radius || b_pt.distance(&corner) > 3.0 * radius {
        return false;
    }
    let seam = polyline_edge(&pts);

    // Retract the torus band to the seam: its old flush-end section (running
    // from this corner to the wall-contact point, which the seam also ends at)
    // is dropped, its top contact circle shortened to the seam's top endpoint.
    let Ok(retracted_cap) = miter_retract_band_at_end(cap, corner, a_pt, b_pt, &seam) else {
        return false;
    };

    // Extend the new blend's contacts out to the seam endpoints.
    if a_is_top {
        blend.contact_a = extend_contact_corner(&blend.contact_a, corner, a_pt);
        blend.contact_b = extend_contact_corner(&blend.contact_b, corner, b_pt);
    } else {
        blend.contact_a = extend_contact_corner(&blend.contact_a, corner, b_pt);
        blend.contact_b = extend_contact_corner(&blend.contact_b, corner, a_pt);
    }

    // Swap the corner-side end arc for the seam, keeping the blend_face wire
    // [contact_a, end_arc, contact_b.reversed(), start_arc] contiguous.
    let corner_is_source = corner.distance(&blend.spine.source().point())
        <= corner.distance(&blend.spine.target().point());
    if corner_is_source {
        let b0 = blend.contact_b.source().point();
        let a0 = blend.contact_a.source().point();
        blend.start_arc = orient_edge_between(&seam, b0, a0);
    } else {
        let a1 = blend.contact_a.target().point();
        let b1 = blend.contact_b.target().point();
        blend.end_arc = orient_edge_between(&seam, a1, b1);
    }
    blend.blend_face = Face::new(
        blend.blend_face.surface().cloned(),
        Wire::from_edges([
            blend.contact_a.clone(),
            blend.end_arc.clone(),
            blend.contact_b.clone().reversed(),
            blend.start_arc.clone(),
        ]),
    );

    faces.push(retracted_cap);
    skipped.insert(cap.id());
    true
}

/// Whether `a` and `b` share at least one (undirected) boundary edge.
fn faces_share_edge(a: &Face, b: &Face) -> bool {
    let b_edges: Vec<Edge> = b.wires().into_iter().flat_map(|w| w.edges()).collect();
    a.wires()
        .into_iter()
        .flat_map(|w| w.edges())
        .any(|ea| b_edges.iter().any(|eb| same_undirected_edge(&ea, eb)))
}

/// Close a corner where the new blend is the *third* edge rounded at a corner —
/// the two earlier fillets already meet there (two prior cylinder caps mitered
/// along their seam). With all three edges now rounded the corner becomes a
/// spherical octant (the rolling ball pivoting in the corner), tangent to all
/// three faces.
///
/// This retracts BOTH prior caps so each ends on a great-circle arc of the sphere
/// (`T -> F` for the side-1 cap, `T -> R` for the side-2 cap, replacing the miter
/// seam they shared), inserts the spherical octant `T-F-R`, and skips the two
/// originals. The new blend's own corner end arc (`F -> R`, in the plane through
/// the two side tangents) already lies on the sphere, so the new blend face is
/// left untouched. Returns `true` on the supported convex, mutually-perpendicular,
/// equal-radius corner; `false` otherwise (caller falls back).
#[allow(clippy::too_many_arguments)]
fn try_corner_sphere_two_caps(
    solid: &Solid,
    blend: &RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> bool {
    let _ = solid;
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    if caps.len() != 2 {
        return false;
    }
    if !caps
        .iter()
        .all(|c| matches!(c.surface(), Some(GeomSurface::Cylinder(_))))
    {
        return false;
    }

    // Sphere center: the new blend's rolling-ball center at this corner.
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    // The new blend's two corner tangents (on its two side faces).
    let side_a = nearest_endpoint(&blend.contact_a, corner);
    let side_b = nearest_endpoint(&blend.contact_b, corner);

    // The shared top tangent `T` is the far end of each cap's miter seam (the
    // ellipse edge meeting this corner). Both caps must agree on it.
    let seam_far = |cap: &Face| -> Option<Pnt> {
        let w = cap.outer_wire()?;
        for e in w.edges() {
            if matches!(e.curve(), Some(GeomCurve::Ellipse(_))) {
                let (s, t) = (e.source().point(), e.target().point());
                if near(s, corner) {
                    return Some(t);
                }
                if near(t, corner) {
                    return Some(s);
                }
            }
        }
        None
    };
    let (Some(t0), Some(t1)) = (seam_far(&caps[0]), seam_far(&caps[1])) else {
        return false;
    };
    let xtol = corner_tol(r);
    if t0.distance(&t1) > xtol {
        return false;
    }
    let t = t0;

    // All three tangent points must sit on the sphere of radius r about C.
    let on_sphere = |p: Pnt| (p.distance(&center) - r).abs() < xtol;
    if !(on_sphere(t) && on_sphere(side_a) && on_sphere(side_b)) {
        return false;
    }

    // Map each cap to the side tangent it must retract to (the side it borders).
    let side_pt_for = |cap: &Face| -> Option<Pnt> {
        if faces_share_edge(cap, &blend.face_a) {
            Some(side_a)
        } else if faces_share_edge(cap, &blend.face_b) {
            Some(side_b)
        } else {
            None
        }
    };
    let (Some(side0), Some(side1)) = (side_pt_for(&caps[0]), side_pt_for(&caps[1])) else {
        return false;
    };
    // The two caps must border different sides.
    if side0.distance(&side1) < 1e-6 {
        return false;
    }

    // Spherical octant T-F-R (F = side_a, R = side_b).
    let n_top = match (t - center).normalized() {
        Some(d) => Dir::new(d.x(), d.y(), d.z()),
        None => return false,
    };
    let n_x = match (side_b - center).normalized() {
        Some(d) => Dir::new(d.x(), d.y(), d.z()),
        None => return false,
    };
    let (Ok(arc_tf), Ok(arc_fr), Ok(arc_rt)) = (
        arc_on_sphere(center, t, side_a, r),
        arc_on_sphere(center, side_a, side_b, r),
        arc_on_sphere(center, side_b, t, r),
    ) else {
        return false;
    };
    let sphere_face = Face::new(
        Some(GeomSurface::sphere(SphericalSurface::new(
            Ax3::new_axes(center, n_top, n_x),
            r,
        ))),
        Wire::from_edges([arc_tf, arc_fr, arc_rt]),
    );

    // Retract each cap: its miter seam becomes the sphere arc `T <-> side`, and its
    // side contact shortens from the stub vertex (corner) to that side tangent.
    for cap in caps {
        let Some(side_pt) = side_pt_for(cap) else {
            return false;
        };
        let Ok(sphere_arc) = arc_on_sphere(center, t, side_pt, r) else {
            return false;
        };
        let Some(w) = cap.outer_wire() else {
            return false;
        };
        let touches_corner =
            |e: &Edge| near(e.source().point(), corner) || near(e.target().point(), corner);
        let mut new_edges = Vec::with_capacity(4);
        for e in w.edges() {
            if matches!(e.curve(), Some(GeomCurve::Ellipse(_))) && touches_corner(&e) {
                // This cap's seam at *this* corner becomes the sphere arc T <-> side.
                let s = if near(e.source().point(), corner) {
                    side_pt
                } else {
                    t
                };
                let d = if near(e.target().point(), corner) {
                    side_pt
                } else {
                    t
                };
                new_edges.push(orient_edge_between(&sphere_arc, s, d));
            } else if !matches!(e.curve(), Some(GeomCurve::Line(_))) {
                // Far-end arcs and any seam from another corner are preserved.
                new_edges.push(e.clone());
            } else {
                let remap = |p: Pnt| if near(p, corner) { side_pt } else { p };
                new_edges.push(Edge::between_points(
                    remap(e.source().point()),
                    remap(e.target().point()),
                ));
            }
        }
        faces.push(Face::with_wires(
            cap.surface().cloned(),
            Some(Wire::from_edges(new_edges)),
            cap.inner_wires(),
            cap.orientation(),
        ));
        skipped.insert(cap.id());
    }

    faces.push(sphere_face);
    true
}

/// A minor great-circle arc of the sphere centered at `center`, radius `r`, from
/// `start` to `end` (both must lie on the sphere).
fn arc_on_sphere(center: Pnt, start: Pnt, end: Pnt, r: f64) -> Result<Edge, RollingBallError> {
    let axis_v = (start - center).cross(&(end - center));
    let axis = axis_v
        .normalized()
        .map(|d| Dir::new(d.x(), d.y(), d.z()))
        .ok_or(RollingBallError::InvalidDihedral)?;
    let xref = (start - center)
        .normalized()
        .map(|d| Dir::new(d.x(), d.y(), d.z()))
        .ok_or(RollingBallError::InvalidDihedral)?;
    contact_arc(center, axis, xref, r, start, end)
}

/// The planar face of `solid`, other than `exclude`, whose boundary contains
/// `edge` (matched undirected by endpoints).
fn adjacent_planar_face(solid: &Solid, edge: &Edge, exclude: &Face) -> Option<Face> {
    solid.shell().faces().into_iter().find(|f| {
        !same_face(f, exclude)
            && matches!(f.surface(), Some(GeomSurface::Plane(_)))
            && f.wires()
                .into_iter()
                .flat_map(|w| w.edges())
                .any(|c| same_undirected_edge(&c, edge))
    })
}

/// Apply a constant-`radius` rolling-ball fillet to several selected `edges`.
///
/// Edges are filleted sequentially: after each blend the solid is rebuilt, so
/// the next selected edge is re-located in the evolving body by matching its
/// endpoint positions (within tolerance). Independent edges — and edges that
/// share a corner, where [`fillet_planar_edge`] inserts a corner cap or Gregory
/// patch — are supported; the order of `edges` does not need to be sorted.
///
/// Returns the first [`RollingBallError`] encountered, or
/// [`RollingBallError::SpineNotOnFace`] if a requested edge can no longer be
/// located after earlier blends consumed it.
pub fn fillet_edges(solid: &Solid, edges: &[Edge], radius: f64) -> Result<Solid, RollingBallError> {
    let mut current = solid.clone();
    for edge in edges {
        // Re-locate the edge in the evolving body. `relocate_edge` tolerates the
        // endpoint drift an earlier blend leaves when two requested edges share a
        // corner (the shared vertex is consumed, shortening the survivor) — an
        // exact endpoint match alone would fail there with `SpineNotOnFace`.
        let target = relocate_edge(&current, edge).ok_or(RollingBallError::SpineNotOnFace)?;
        current = fillet_planar_edge(&current, &target, radius)?;
    }
    Ok(current)
}

/// Fillet a logical circular edge that is represented in the B-Rep as several
/// co-circular edge fragments.
///
/// Cylinder booleans often split a circular rim at construction seams. Those
/// split points are not design corners, so sequentially filleting each fragment
/// asks the corner code to cap fake endpoints and produces invalid topology.
/// This treats the fragments as one selected edge: contacts and adjacent faces
/// are trimmed across the whole chain, while only the requested spine endpoints
/// are closed against real cap faces.
pub fn fillet_circular_edge_chain(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    if chain_edges.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }

    // A rim that wraps a full circle (a plain cylinder top, a bored hole) has no
    // free endpoints, so the open-chain endpoint closers below don't apply. Build
    // it as a seamless torus band of trimmed supports instead. On failure the
    // caller's routing falls back to the per-edge fillet.
    if spine_wraps_full_circle(spine, chain_edges) {
        return fillet_closed_circular_rim(solid, chain_edges, spine, radius);
    }

    // Open chain (the "bite arc"): prefer the analytic torus band with flush
    // end trims; fall back to the legacy rolling-ball open path for whatever
    // configuration it declines.
    if let Ok(result) = blend_open_circular_chain(solid, chain_edges, spine, radius, false) {
        return Ok(result);
    }

    let (plane_face, cyl_faces) = circular_chain_support_faces(solid, chain_edges)?;
    let mut blend =
        rolling_ball_between_curved_faces(solid, spine, &plane_face, &cyl_faces[0], radius)?;
    split_blend_face_for_spine_chain(&mut blend, spine, chain_edges)?;
    let start = spine.source().point();
    let end = spine.target().point();

    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);
    let cut_guards = cut_cylinder_guards(solid, &start_caps, &end_caps);

    let mut faces = Vec::new();
    let mut skipped_faces = std::collections::HashSet::new();

    let start_cut = try_corner_cut(
        solid,
        &mut blend,
        start,
        &start_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    if !start_cut {
        handle_corner_endpoint(
            solid,
            &blend,
            start,
            &start_caps,
            &blend.start_arc,
            radius,
            false,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

    let end_cut = try_corner_cut(
        solid,
        &mut blend,
        end,
        &end_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    if !end_cut {
        handle_corner_endpoint(
            solid,
            &blend,
            end,
            &end_caps,
            &blend.end_arc,
            radius,
            false,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

    let trimmed_plane =
        trim_face_along_spine_segments(&plane_face, chain_edges, spine, &blend.contact_a)?;
    let mut trimmed_cyls = Vec::new();
    for cyl in &cyl_faces {
        trimmed_cyls.push(trim_face_along_spine_segments(
            cyl,
            chain_edges,
            spine,
            &blend.contact_b,
        )?);
    }

    for face in solid.shell().faces() {
        if same_face(&face, &plane_face)
            || cyl_faces.iter().any(|cyl| same_face(&face, cyl))
            || skipped_faces.contains(&face.id())
        {
            continue;
        }
        faces.push(face);
    }

    faces.push(trimmed_plane);
    faces.extend(trimmed_cyls);
    faces.push(blend.blend_face);

    let result = Solid::new(sew(&faces, radius * 0.1));
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if cut_guards.is_empty() {
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
    } else {
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
    }
    Err(RollingBallError::InvalidTopology)
}

/// Whether the spine/chain closes a full circle (a 360° rim, no free endpoints).
fn spine_wraps_full_circle(spine: &Edge, chain_edges: &[Edge]) -> bool {
    // Every fragment must be circular for this to be a rim.
    if !chain_edges
        .iter()
        .all(|e| matches!(e.curve(), Some(GeomCurve::Circle(_))))
    {
        return false;
    }
    let closed_endpoints = spine.source().point().distance(&spine.target().point())
        <= 100.0 * tolerance::CONFUSION;
    // A single full-circle edge (source == target) also wraps.
    let full_span = (spine.last() - spine.first()).abs() >= core::f64::consts::TAU - 1e-4;
    closed_endpoints || full_span
}

/// Recovered geometry shared by the closed-rim fillet and chamfer builders.
struct ClosedRimGeom {
    plane_face: Face,
    cyl_faces: Vec<Face>,
    spine_r: f64,
    cax: Dir,
    cxr: Dir,
    n_plane: Dir,
    concave: bool,
    /// Radius of the plane contact ring: `spine_r ± dist`.
    major_radius: f64,
    /// Plane contact ring (on the cap) and wall contact ring (offset into solid).
    c_plane: Circle,
    c_cyl: Circle,
    /// Common center of the wall-contact ring (and the torus tube axis): the cap
    /// center pushed `dist` into the solid along `-n_plane`.
    contact_center: Pnt,
}

/// Recover the geometry of a closed circular rim for a cap-fillet/chamfer of size
/// `dist`, validating the cap-fillet configuration (plane normal ∥ wall axis) and
/// rejecting an oversize value.
fn recover_closed_rim_geom(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    dist: f64,
) -> Result<ClosedRimGeom, RollingBallError> {
    if !dist.is_finite() || dist <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius: dist });
    }
    let Some(GeomCurve::Circle(circle)) = spine.curve() else {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::NotPlaneOrAnalytic,
        });
    };
    let circle = *circle;

    let (plane_face, cyl_faces) = circular_chain_support_faces(solid, chain_edges)?;

    let center = circle.center();
    let spine_r = circle.radius();
    let cax = circle.axis();
    let cxr = circle.position().x_direction();
    let n_plane = face_outward_normal_at(&plane_face, spine.source().point())?;

    // Cap configuration only: the plane normal must be parallel to the wall axis
    // (the ball/bevel rolls around the rim → a torus/cone of revolution).
    let axis_dir = match cyl_faces[0].surface() {
        Some(GeomSurface::Cylinder(c)) => c.position().direction(),
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    if !n_plane.is_parallel(&axis_dir, 1e-4) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    }

    let concave = is_concave_cut_cylinder(solid, &cyl_faces[0]);
    let major_radius = if concave {
        spine_r + dist
    } else {
        spine_r - dist
    };
    if major_radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius: dist });
    }

    let contact_center = center - GeomVec::from_dir(n_plane) * dist;
    let c_plane = Circle::new(Ax3::new_axes(center, cax, cxr), major_radius);
    let c_cyl = Circle::new(Ax3::new_axes(contact_center, cax, cxr), spine_r);

    Ok(ClosedRimGeom {
        plane_face,
        cyl_faces,
        spine_r,
        cax,
        cxr,
        n_plane,
        concave,
        major_radius,
        c_plane,
        c_cyl,
        contact_center,
    })
}

/// Assemble a closed-rim blend: keep every non-support face, splice in the band
/// faces, and replace the plane cap + wall supports with versions trimmed back to
/// the contact rings. Safety-gated by [`accept_subtractive_blend_result`].
fn assemble_closed_rim(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    geom: &ClosedRimGeom,
    dist: f64,
    mut band_faces: Vec<Face>,
) -> Result<Solid, RollingBallError> {
    use core::f64::consts::TAU;

    // Full-circle contact edges drive the spine→contact param mapping.
    let c_plane_edge = Edge::new(
        Some(GeomCurve::circle(geom.c_plane)),
        0.0,
        TAU,
        Vertex::new(geom.c_plane.point(0.0)),
        Vertex::new(geom.c_plane.point(TAU)),
    );
    let c_cyl_edge = Edge::new(
        Some(GeomCurve::circle(geom.c_cyl)),
        0.0,
        TAU,
        Vertex::new(geom.c_cyl.point(0.0)),
        Vertex::new(geom.c_cyl.point(TAU)),
    );

    // Trim the plane cap: replace the rim loop with the plane-contact ring.
    let trimmed_cap = trim_cap_closed_loop(&geom.plane_face, spine, &c_plane_edge, chain_edges)?;
    // Trim each wall face: shorten its top arc to the wall-contact ring.
    let mut trimmed_walls = Vec::new();
    for cyl in &geom.cyl_faces {
        trimmed_walls.push(trim_face_along_spine_segments(
            cyl,
            chain_edges,
            spine,
            &c_cyl_edge,
        )?);
    }

    // Keep every other face untouched.
    for face in solid.shell().faces() {
        if same_face(&face, &geom.plane_face)
            || geom.cyl_faces.iter().any(|cyl| same_face(&face, cyl))
        {
            continue;
        }
        band_faces.push(face);
    }
    band_faces.push(trimmed_cap);
    band_faces.extend(trimmed_walls);

    let result = Solid::new(sew(&band_faces, dist * 0.1));
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if let Some(accepted) = accept_subtractive_blend_result(&merged, &[]) {
        return Ok(accepted);
    }
    if let Some(accepted) = accept_subtractive_blend_result(&result, &[]) {
        return Ok(accepted);
    }
    Err(RollingBallError::InvalidTopology)
}

/// Fillet a closed circular rim (a plain cylinder top, a bored hole): a seamless
/// toroidal band between the trimmed cap and wall supports.
fn fillet_closed_circular_rim(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    // The band construction needs the rim split into ≥2 fragments so each band
    // face has distinct start/end seams (a single full-circle edge would make a
    // degenerate wire). The make_cylinder rims that reach here are already 3 arcs.
    if chain_edges.len() < 2 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let geom = recover_closed_rim_geom(solid, chain_edges, spine, radius)?;
    let torus_surf = GeomSurface::torus(ToroidalSurface::new(
        Ax3::new_axes(geom.contact_center, geom.cax, geom.cxr),
        geom.major_radius,
        radius,
    ));

    // One toroidal band face per rim fragment. Adjacent fragments share an
    // identical quarter-arc seam at their common param, so `sew` welds them.
    let mut faces: Vec<Face> = Vec::new();
    for arc in chain_edges {
        let (u0, u1) = spine_param_span(spine, arc)?;
        let plane_arc = closed_rim_arc(&geom.c_plane, u0, u1);
        let cyl_arc = closed_rim_arc(&geom.c_cyl, u0, u1);
        let tube0 = geom.contact_center + radial(geom.cax, geom.cxr, u0) * geom.major_radius;
        let tube1 = geom.contact_center + radial(geom.cax, geom.cxr, u1) * geom.major_radius;
        let seam0 = crate::blend::quarter_arc(
            tube0,
            radius,
            geom.c_plane.point(u0),
            geom.c_cyl.point(u0),
        );
        let seam1 = crate::blend::quarter_arc(
            tube1,
            radius,
            geom.c_plane.point(u1),
            geom.c_cyl.point(u1),
        );
        let wire = Wire::from_edges([plane_arc, seam1, cyl_arc.reversed(), seam0.reversed()]);
        faces.push(Face::new(Some(torus_surf.clone()), wire));
    }

    assemble_closed_rim(solid, chain_edges, spine, &geom, radius, faces)
}

/// Chamfer a closed circular rim: a seamless conical (45°) frustum band between
/// the trimmed cap and wall supports, with straight seams.
fn chamfer_closed_circular_rim(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    dist: f64,
) -> Result<Solid, RollingBallError> {
    use core::f64::consts::FRAC_PI_2;

    // Same ≥2-fragments constraint as the closed-rim fillet (distinct seams).
    if chain_edges.len() < 2 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let geom = recover_closed_rim_geom(solid, chain_edges, spine, dist)?;

    // The cone passes through both contact rings: reference radius `spine_r` at the
    // wall contact, growing/shrinking to `major_radius` at the cap over axial
    // distance `dist` → a ±45° half-angle. The sign accounts for both the cut
    // concavity and whether the spine-circle axis agrees with the outward normal.
    let sign = if geom.n_plane.dot(&geom.cax) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let concave_sign = if geom.concave { 1.0 } else { -1.0 };
    let semi_angle = concave_sign * sign * (FRAC_PI_2 / 2.0);
    let cone_surf = GeomSurface::cone(ConicalSurface::new(
        Ax3::new_axes(geom.contact_center, geom.cax, geom.cxr),
        geom.spine_r,
        semi_angle,
    ));

    let mut faces: Vec<Face> = Vec::new();
    for arc in chain_edges {
        let (u0, u1) = spine_param_span(spine, arc)?;
        let plane_arc = closed_rim_arc(&geom.c_plane, u0, u1);
        let cyl_arc = closed_rim_arc(&geom.c_cyl, u0, u1);
        // Straight seams (a chamfer bevel is a ruled cone, not a rolled torus).
        let seam0 = Edge::between_points(geom.c_plane.point(u0), geom.c_cyl.point(u0));
        let seam1 = Edge::between_points(geom.c_plane.point(u1), geom.c_cyl.point(u1));
        let wire = Wire::from_edges([plane_arc, seam1, cyl_arc.reversed(), seam0.reversed()]);
        faces.push(Face::new(Some(cone_surf.clone()), wire));
    }

    assemble_closed_rim(solid, chain_edges, spine, &geom, dist, faces)
}

/// An arc edge of `circle` between spine params `u0`..`u1` (source → target).
fn closed_rim_arc(circle: &Circle, u0: f64, u1: f64) -> Edge {
    Edge::new(
        Some(GeomCurve::circle(*circle)),
        u0,
        u1,
        Vertex::new(circle.point(u0)),
        Vertex::new(circle.point(u1)),
    )
}

/// Sample points along an edge's curve (plus its vertices) — enough to bound a
/// circular arc's bulge for a bounds check.
fn edge_sample_points(edge: &Edge) -> Vec<Pnt> {
    let mut pts = vec![edge.start().point(), edge.end().point()];
    if let Some(curve) = edge.curve() {
        let (t0, t1) = (edge.first(), edge.last());
        for k in 1..8 {
            let t = t0 + (t1 - t0) * (k as f64) / 8.0;
            pts.push(curve.point(t));
        }
    }
    pts
}

/// Approximate axis-aligned bounds of a solid from its edges' sampled points.
fn approx_solid_bounds(solid: &Solid) -> (Pnt, Pnt) {
    let mut lo = Pnt::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Pnt::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for face in solid.shell().faces() {
        for wire in face.wires() {
            for edge in wire.edges() {
                for p in edge_sample_points(&edge) {
                    lo = Pnt::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z()));
                    hi = Pnt::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z()));
                }
            }
        }
    }
    (lo, hi)
}

/// Golden-section minimum of `g` over `[lo, hi]`.
fn minimize_scalar(g: impl Fn(f64) -> f64, lo: f64, hi: f64) -> f64 {
    let phi = (5.0f64.sqrt() - 1.0) / 2.0;
    let (mut a, mut b) = (lo.min(hi), lo.max(hi));
    let mut c = b - phi * (b - a);
    let mut d = a + phi * (b - a);
    let (mut gc, mut gd) = (g(c), g(d));
    for _ in 0..90 {
        if gc <= gd {
            b = d;
            d = c;
            gd = gc;
            c = b - phi * (b - a);
            gc = g(c);
        } else {
            a = c;
            c = d;
            gc = gd;
            d = a + phi * (b - a);
            gd = g(d);
        }
        if b - a <= 1e-12 {
            break;
        }
    }
    0.5 * (a + b)
}

/// Nearest root of `g` starting at `start`, preferring the `dir` (+1/-1)
/// direction: scan up to `ahead` forward (and a small `0.05` backtrack window)
/// for a sign change, then bisect it. Used to march a band∩band miter seam,
/// whose stations each have TWO crossings near the tangent end — the seam
/// branch continues in a consistent direction, the other walks back under the
/// chain.
fn directional_root(g: impl Fn(f64) -> f64, start: f64, dir: f64, ahead: f64) -> Option<f64> {
    let scan = |x0: f64, x1: f64, n: usize| -> Option<(f64, f64)> {
        let mut px = x0;
        let mut pg = g(x0);
        for i in 1..=n {
            let x = x0 + (x1 - x0) * (i as f64) / (n as f64);
            let gx = g(x);
            if pg == 0.0 {
                return Some((px, px));
            }
            if gx == 0.0 {
                return Some((x, x));
            }
            if gx.signum() != pg.signum() {
                return Some((px, x));
            }
            px = x;
            pg = gx;
        }
        None
    };
    let bracket = scan(start, start + dir * ahead, 240)
        .or_else(|| scan(start, start - dir * 0.05, 40))?;
    let (mut a, mut b) = bracket;
    if a == b {
        return Some(a);
    }
    let (mut ga, _) = (g(a), g(b));
    for _ in 0..80 {
        let m = 0.5 * (a + b);
        let gm = g(m);
        if gm == 0.0 || (b - a).abs() <= 1e-13 {
            return Some(m);
        }
        if gm.signum() == ga.signum() {
            a = m;
            ga = gm;
        } else {
            b = m;
        }
    }
    Some(0.5 * (a + b))
}

/// Retract an existing fillet-band face whose end at `old_end` is being mitered
/// against a newly built band: the boundary edge running from `old_end` to (≈)
/// `b_shared` — the band's old end-trim section, now entirely inside the strip
/// the new blend removes — is dropped; the band's other `old_end`-incident
/// boundary edge (its contact curve) is shortened along its own curve to
/// `a_new`; and the mutual `seam` (from `a_new` to `b_shared`) is spliced in.
fn miter_retract_band_at_end(
    face: &Face,
    old_end: Pnt,
    a_new: Pnt,
    b_shared: Pnt,
    seam: &Edge,
) -> Result<Face, RollingBallError> {
    let tol = 10.0 * tolerance::CONFUSION;
    // The old end trim spans the full blend cross-section; its far endpoint must
    // land on `b_shared` for the retraction to be sound. Allow the small drift a
    // chorded flush section accumulates.
    let end_tol = 1e-3 * (old_end.distance(&b_shared)).max(1.0);
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let Some(next_idx) = edges
        .iter()
        .position(|e| e.source().point().distance(&old_end) <= tol)
    else {
        return Err(RollingBallError::UnsupportedTrimTopology);
    };
    let prev_idx = (next_idx + n - 1) % n;
    if edges[prev_idx].target().point().distance(&old_end) > tol {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    // Which incident edge is the old end trim (drop), which the contact (retract)?
    let next_is_drop = edges[next_idx].target().point().distance(&b_shared) <= end_tol;
    let prev_is_drop = edges[prev_idx].source().point().distance(&b_shared) <= end_tol;
    if next_is_drop == prev_is_drop {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let mut new_edges = Vec::with_capacity(n + 1);
    for offset in 0..n {
        let i = (prev_idx + offset) % n;
        if i == prev_idx {
            if prev_is_drop {
                // [.., drop, contact ..] → [.., seam(b→a), contact' ..]
                new_edges.push(orient_edge_between(seam, b_shared, a_new));
            } else {
                new_edges.push(move_edge_endpoint_keep_curve(
                    &edges[prev_idx],
                    old_end,
                    a_new,
                ));
                new_edges.push(orient_edge_between(seam, a_new, b_shared));
            }
        } else if i == next_idx {
            if next_is_drop {
                // The drop edge is replaced by the seam already pushed above.
            } else {
                new_edges.push(move_edge_endpoint_keep_curve(
                    &edges[next_idx],
                    old_end,
                    a_new,
                ));
            }
        } else {
            new_edges.push(edges[i].clone());
        }
    }
    new_edges.retain(|e| e.source().point().distance(&e.target().point()) > tolerance::CONFUSION);
    rebuild_face(face, Wire::from_edges(new_edges))
}

/// Blend an OPEN co-circular chain — the "bite arc": rim fragments covering
/// part of a circle whose free ends run out onto planar faces (e.g. the front
/// face of a box a cylindrical bite was cut out of). Built exactly like the
/// closed rim — an analytic band (torus for a fillet, 45° cone for a chamfer)
/// between the contact rings, with cap/wall supports trimmed to them — except
/// at the two free ends, where the band is trimmed FLUSH against the end-cap
/// plane: each contact ring stops where it crosses that plane, the band is
/// closed by the band∩plane section (a chorded polyline lying on both
/// surfaces), and the cap face is re-trimmed to the same section. Nothing
/// bulges through the cap, and every neighbour pair shares identical edge
/// geometry, so the sewn result is watertight and tessellates crack-free.
fn blend_open_circular_chain(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    dist: f64,
    chamfer: bool,
) -> Result<Solid, RollingBallError> {
    use core::f64::consts::{FRAC_PI_2, PI, TAU};

    if chain_edges.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }
    let geom = recover_closed_rim_geom(solid, chain_edges, spine, dist)?;

    let (t_lo, t_hi) = (
        spine.first().min(spine.last()),
        spine.first().max(spine.last()),
    );
    // Free-end corners at the spine's param extremes (the canonical spine from
    // `circular_spine_from_chain` is ascending: source at `first`).
    let (corner_lo, corner_hi) = if spine.first() <= spine.last() {
        (spine.source().point(), spine.target().point())
    } else {
        (spine.target().point(), spine.source().point())
    };

    // Exactly one cap face at each free end: a planar wall (→ flush trim), or —
    // fillets only — the cylindrical band of an earlier same-radius straight
    // fillet tangent to the same support plane (→ miter: the two bands flow
    // into each other along their intersection seam).
    let n_plane_v = GeomVec::from_dir(geom.n_plane);
    let plane_p0 = match geom.plane_face.surface() {
        Some(GeomSurface::Plane(pl)) => pl.position().location(),
        _ => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let cap_at = |corner: Pnt| -> Result<(Face, Option<CylindricalSurface>), RollingBallError> {
        let caps: Vec<Face> = solid
            .shell()
            .faces()
            .into_iter()
            .filter(|f| {
                !same_face(f, &geom.plane_face)
                    && !geom.cyl_faces.iter().any(|c| same_face(f, c))
                    && face_contains_point(f, corner)
            })
            .collect();
        if caps.len() != 1 {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
        match caps[0].surface() {
            Some(GeomSurface::Plane(_)) => Ok((caps[0].clone(), None)),
            Some(GeomSurface::Cylinder(c)) if !chamfer => {
                // An earlier fillet's band: same radius, axis parallel to the
                // support plane at offset `dist` below it.
                let axis_off = (c.position().location() - plane_p0).dot(&n_plane_v);
                let axis_tilt = GeomVec::from_dir(c.position().direction())
                    .dot(&n_plane_v)
                    .abs();
                if (c.radius() - dist).abs() <= corner_tol(dist)
                    && axis_tilt < 1e-3
                    && (axis_off + dist).abs() <= corner_tol(dist)
                {
                    Ok((caps[0].clone(), Some(*c)))
                } else {
                    Err(RollingBallError::UnsupportedTrimTopology)
                }
            }
            _ => Err(RollingBallError::UnsupportedTrimTopology),
        }
    };
    let (cap_lo, miter_lo) = cap_at(corner_lo)?;
    let (cap_hi, miter_hi) = cap_at(corner_hi)?;

    // Ring frame (shared by both contact rings) and the cap-ring axial offset.
    let xv = GeomVec::from_dir(geom.c_plane.position().x_direction());
    let yv = GeomVec::from_dir(geom.c_plane.position().y_direction());
    let caxv = GeomVec::from_dir(geom.cax);
    let s_plane = if geom.n_plane.dot(&geom.cax) >= 0.0 {
        1.0
    } else {
        -1.0
    };

    struct FlushEnd {
        cap: Face,
        corner: Pnt,
        /// Cap-ring param where the blend stops (flush with the cap plane).
        ua: f64,
        /// Wall-ring param where the blend stops.
        ub: f64,
        /// The band∩cap section from `c_plane(ua)` to `c_cyl(ub)`.
        trim: Edge,
        /// Set when the "cap" is an earlier fillet's band cylinder: the end is a
        /// MITER (the trim is the band∩band seam, and the old band is retracted
        /// to it) rather than a flush planar cut.
        miter: bool,
    }

    let resolve_end = |cap: &Face, corner: Pnt, near: f64| -> Result<FlushEnd, RollingBallError> {
        let Some(GeomSurface::Plane(pl)) = cap.surface() else {
            return Err(RollingBallError::UnsupportedTrimTopology);
        };
        let n = GeomVec::from_dir(pl.normal());
        let p0 = pl.position().location();

        // Where the horizontal circle of radius `rho` at axial offset `h` (from
        // `contact_center`) crosses the cap plane: `a·cos u + b·sin u + c = 0`,
        // taking the branch nearest `seed`.
        let solve_u = |rho: f64, h: f64, seed: f64| -> Option<f64> {
            let a = rho * xv.dot(&n);
            let b = rho * yv.dot(&n);
            let c = (geom.contact_center + caxv * h - p0).dot(&n);
            let hyp = a.hypot(b);
            if hyp <= tolerance::CONFUSION || (c / hyp).abs() > 1.0 + 1.0e-9 {
                return None;
            }
            let phi = b.atan2(a);
            let d = (-c / hyp).clamp(-1.0, 1.0).acos();
            let mut best: Option<f64> = None;
            for cand in [phi + d, phi - d] {
                for k in -2..=2 {
                    let t = cand + f64::from(k) * TAU;
                    if best.map_or(true, |bb| (t - seed).abs() < (bb - seed).abs()) {
                        best = Some(t);
                    }
                }
            }
            best
        };

        let h_plane = s_plane * dist;
        let ua = solve_u(geom.major_radius, h_plane, near)
            .ok_or(RollingBallError::UnsupportedTrimTopology)?;
        let ub =
            solve_u(geom.spine_r, 0.0, near).ok_or(RollingBallError::UnsupportedTrimTopology)?;
        // Both crossings must sit inside the chain's own span: the blend runs
        // OUT of the solid at this free end, never past the far one.
        if ua < t_lo - 1.0e-6 || ua > t_hi + 1.0e-6 || ub < t_lo - 1.0e-6 || ub > t_hi + 1.0e-6 {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }

        // Chord the band∩plane section from the cap-ring crossing down to the
        // wall-ring crossing, densely enough to read as smooth (like the
        // cylinder∩cylinder trim, ≈0.05 chords).
        let steps = ((dist * 2.0 / 0.05).ceil() as usize).clamp(24, 160);
        let mut pts = Vec::with_capacity(steps + 1);
        pts.push(geom.c_plane.point(ua));
        let mut seed = ua;
        let v_p = s_plane * FRAC_PI_2;
        let v_c = if geom.concave { s_plane * PI } else { 0.0 };
        for k in 1..steps {
            let f = k as f64 / steps as f64;
            let (rho, h) = if chamfer {
                (
                    geom.major_radius + f * (geom.spine_r - geom.major_radius),
                    (1.0 - f) * h_plane,
                )
            } else {
                let v = v_p + f * (v_c - v_p);
                (geom.major_radius + dist * v.cos(), dist * v.sin())
            };
            let u = solve_u(rho, h, seed).ok_or(RollingBallError::UnsupportedTrimTopology)?;
            seed = u;
            pts.push(geom.contact_center + caxv * h + (xv * u.cos() + yv * u.sin()) * rho);
        }
        // A wrong branch anywhere would land the section far from the wall-ring
        // crossing; require the sampled sweep to arrive there.
        if (seed - ub).abs() > 0.5 {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
        pts.push(geom.c_cyl.point(ub));
        Ok(FlushEnd {
            cap: cap.clone(),
            corner,
            ua,
            ub,
            trim: polyline_edge(&pts),
            miter: false,
        })
    };

    // Miter this free end against an earlier same-radius straight fillet's band
    // cylinder: march the band∩band intersection seam from the shared support
    // plane (where the two bands are TANGENT — the crossing there is the
    // minimum of the distance, not a sign change) down to the wall contact.
    // `side` is +1 at the chain's high-param end, -1 at the low end: the seam
    // extends BEYOND the chain into the region the earlier fillet vacated, and
    // each station's root is taken on that side (the complementary root walks
    // back under the chain).
    let resolve_end_miter = |cap: &Face,
                             cyl: &CylindricalSurface,
                             corner: Pnt,
                             near: f64,
                             side: f64|
     -> Result<FlushEnd, RollingBallError> {
        let axis_pt = cyl.position().location();
        let axis_dir = cyl.position().direction();
        let r = cyl.radius();
        let ring_pt = |rho: f64, h: f64, u: f64| {
            geom.contact_center + caxv * h + (xv * u.cos() + yv * u.sin()) * rho
        };
        let g = |rho: f64, h: f64, u: f64| point_line_distance(ring_pt(rho, h, u), axis_pt, axis_dir) - r;

        let steps = ((dist * 2.0 / 0.05).ceil() as usize).clamp(24, 160);
        let h_plane = s_plane * dist;
        let v_p = s_plane * FRAC_PI_2;
        let v_c = if geom.concave { s_plane * PI } else { 0.0 };
        let station = |k: usize| -> (f64, f64) {
            let f = k as f64 / steps as f64;
            let v = v_p + f * (v_c - v_p);
            (geom.major_radius + dist * v.cos(), dist * v.sin())
        };

        let (rho0, h0) = (geom.major_radius, h_plane);
        let ua = minimize_scalar(|u| g(rho0, h0, u).abs(), near - 0.7, near + 0.7);
        if g(rho0, h0, ua).abs() > 1e-4 * dist.max(1.0) {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
        let mut pts = vec![ring_pt(rho0, h0, ua)];
        let mut prev_u = ua;
        for k in 1..=steps {
            let (rho, h) = station(k);
            let u = directional_root(|u| g(rho, h, u), prev_u, side, 0.35)
                .ok_or(RollingBallError::UnsupportedTrimTopology)?;
            if (u - prev_u).abs() > 0.3 {
                return Err(RollingBallError::UnsupportedTrimTopology);
            }
            prev_u = u;
            pts.push(ring_pt(rho, h, u));
        }
        let ub = prev_u;
        if (ua - near).abs() > 0.7 || (ub - near).abs() > 0.7 {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
        Ok(FlushEnd {
            cap: cap.clone(),
            corner,
            ua,
            ub,
            trim: polyline_edge(&pts),
            miter: true,
        })
    };

    let end_lo = match &miter_lo {
        Some(cyl) => resolve_end_miter(&cap_lo, cyl, corner_lo, t_lo, -1.0)?,
        None => resolve_end(&cap_lo, corner_lo, t_lo)?,
    };
    let end_hi = match &miter_hi {
        Some(cyl) => resolve_end_miter(&cap_hi, cyl, corner_hi, t_hi, 1.0)?,
        None => resolve_end(&cap_hi, corner_hi, t_hi)?,
    };
    if end_lo.ua >= end_hi.ua - 1.0e-6 || end_lo.ub >= end_hi.ub - 1.0e-6 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    // Fragment spans, ascending; interior junctions carry the shared seams.
    let mut spans: Vec<(f64, f64)> = chain_edges
        .iter()
        .map(|e| {
            let (a, b) = spine_param_span(spine, e)?;
            Ok((a.min(b), a.max(b)))
        })
        .collect::<Result<_, RollingBallError>>()?;
    spans.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let junctions: Vec<f64> = spans.iter().skip(1).map(|s| s.0).collect();
    if junctions.iter().any(|&j| {
        j <= end_lo.ua.max(end_lo.ub) + 1.0e-6 || j >= end_hi.ua.min(end_hi.ub) - 1.0e-6
    }) {
        // An end trim swallowing a whole fragment is out of scope.
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let band_surf = if chamfer {
        let concave_sign = if geom.concave { 1.0 } else { -1.0 };
        let semi_angle = concave_sign * s_plane * (FRAC_PI_2 / 2.0);
        GeomSurface::cone(ConicalSurface::new(
            Ax3::new_axes(geom.contact_center, geom.cax, geom.cxr),
            geom.spine_r,
            semi_angle,
        ))
    } else {
        GeomSurface::torus(ToroidalSurface::new(
            Ax3::new_axes(geom.contact_center, geom.cax, geom.cxr),
            geom.major_radius,
            dist,
        ))
    };

    // Seams between adjacent band fragments — one edge value shared by both.
    let seam_at = |u: f64| -> Edge {
        let p_pl = geom.c_plane.point(u);
        let p_cy = geom.c_cyl.point(u);
        if chamfer {
            Edge::between_points(p_pl, p_cy)
        } else {
            let tube = geom.contact_center + radial(geom.cax, geom.cxr, u) * geom.major_radius;
            crate::blend::quarter_arc(tube, dist, p_pl, p_cy)
        }
    };
    let seams: Vec<Edge> = junctions.iter().map(|&j| seam_at(j)).collect();

    let n_frag = spans.len();
    let mut faces: Vec<Face> = Vec::with_capacity(n_frag + solid.shell().faces().len());
    for k in 0..n_frag {
        let pa0 = if k == 0 { end_lo.ua } else { junctions[k - 1] };
        let pa1 = if k == n_frag - 1 { end_hi.ua } else { junctions[k] };
        let pb0 = if k == 0 { end_lo.ub } else { junctions[k - 1] };
        let pb1 = if k == n_frag - 1 { end_hi.ub } else { junctions[k] };
        if pa1 - pa0 <= 1.0e-9 || pb1 - pb0 <= 1.0e-9 {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
        let plane_arc = closed_rim_arc(&geom.c_plane, pa0, pa1);
        let cyl_arc = closed_rim_arc(&geom.c_cyl, pb0, pb1);
        let start_edge = if k == 0 {
            orient_edge_between(
                &end_lo.trim,
                geom.c_plane.point(pa0),
                geom.c_cyl.point(pb0),
            )
        } else {
            seams[k - 1].clone()
        };
        let end_edge = if k == n_frag - 1 {
            orient_edge_between(
                &end_hi.trim,
                geom.c_plane.point(pa1),
                geom.c_cyl.point(pb1),
            )
        } else {
            seams[k].clone()
        };
        let wire = Wire::from_edges([
            plane_arc,
            end_edge,
            cyl_arc.reversed(),
            start_edge.reversed(),
        ]);
        faces.push(Face::new(Some(band_surf.clone()), wire));
    }

    // Supports trimmed to the contact rings, clamped to the flush end params.
    let c_plane_edge = Edge::new(
        Some(GeomCurve::circle(geom.c_plane)),
        0.0,
        TAU,
        Vertex::new(geom.c_plane.point(0.0)),
        Vertex::new(geom.c_plane.point(TAU)),
    );
    let c_cyl_edge = Edge::new(
        Some(GeomCurve::circle(geom.c_cyl)),
        0.0,
        TAU,
        Vertex::new(geom.c_cyl.point(0.0)),
        Vertex::new(geom.c_cyl.point(TAU)),
    );
    let trimmed_cap_support = trim_face_along_spine_segments_clamped(
        &geom.plane_face,
        chain_edges,
        spine,
        &c_plane_edge,
        (end_lo.ua, end_hi.ua),
    )?;
    let mut trimmed_walls = Vec::new();
    for cyl in &geom.cyl_faces {
        trimmed_walls.push(trim_face_along_spine_segments_clamped(
            cyl,
            chain_edges,
            spine,
            &c_cyl_edge,
            (end_lo.ub, end_hi.ub),
        )?);
    }

    // End caps re-trimmed to the flush sections (both ends can land on the
    // SAME face — a bite whose two ends run out the same box side). A miter end
    // instead RETRACTS the earlier fillet's band to the mutual seam.
    let trim_cap = |cap: &Face, end: &FlushEnd| -> Result<Face, RollingBallError> {
        if end.miter {
            miter_retract_band_at_end(
                cap,
                end.corner,
                geom.c_plane.point(end.ua),
                geom.c_cyl.point(end.ub),
                &end.trim,
            )
        } else {
            trim_face_at_corner(
                cap,
                end.corner,
                geom.c_plane.point(end.ua),
                geom.c_cyl.point(end.ub),
                &end.trim,
            )
        }
    };
    let mut trimmed_caps: Vec<Face> = Vec::new();
    if same_face(&end_lo.cap, &end_hi.cap) {
        let once = trim_cap(&end_lo.cap, &end_lo)?;
        trimmed_caps.push(trim_cap(&once, &end_hi)?);
    } else {
        trimmed_caps.push(trim_cap(&end_lo.cap, &end_lo)?);
        trimmed_caps.push(trim_cap(&end_hi.cap, &end_hi)?);
    }

    for face in solid.shell().faces() {
        if same_face(&face, &geom.plane_face)
            || geom.cyl_faces.iter().any(|c| same_face(&face, c))
            || same_face(&face, &end_lo.cap)
            || same_face(&face, &end_hi.cap)
        {
            continue;
        }
        faces.push(face);
    }
    faces.push(trimmed_cap_support);
    faces.extend(trimmed_walls);
    faces.extend(trimmed_caps);

    // A blend only REMOVES material, so the rebuilt body must stay inside the
    // input's bounds. An oversize distance puts the contact rings off their
    // supports (below the wall, outside the cap) yet the spliced wires can
    // still close and sew "watertight" — the bounds are what actually give it
    // away.
    let (in_lo, in_hi) = approx_solid_bounds(solid);
    let tol = 1.0e-6;
    let grew = faces.iter().any(|f| {
        f.wires().iter().any(|w| {
            w.edges().iter().any(|e| {
                edge_sample_points(e).into_iter().any(|p| {
                    p.x() < in_lo.x() - tol
                        || p.y() < in_lo.y() - tol
                        || p.z() < in_lo.z() - tol
                        || p.x() > in_hi.x() + tol
                        || p.y() > in_hi.y() + tol
                        || p.z() > in_hi.z() + tol
                })
            })
        })
    });
    if grew {
        return Err(RollingBallError::InvalidRadius { radius: dist });
    }

    let result = Solid::new(sew(&faces, dist * 0.1));
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if let Some(accepted) = accept_subtractive_blend_result(&merged, &[]) {
        return Ok(accepted);
    }
    if let Some(accepted) = accept_subtractive_blend_result(&result, &[]) {
        return Ok(accepted);
    }
    Err(RollingBallError::InvalidTopology)
}

/// Chamfer a closed circular rim; the analogue of [`fillet_circular_edge_chain`]
/// for a constant-distance bevel. Only the closed-rim configuration is built here
/// (a plain cylinder top, a bored hole); an open arc chain returns `Err` so the
/// caller falls back to the per-edge chamfer.
pub fn chamfer_circular_edge_chain(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    dist: f64,
) -> Result<Solid, RollingBallError> {
    if chain_edges.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }
    if spine_wraps_full_circle(spine, chain_edges) {
        chamfer_closed_circular_rim(solid, chain_edges, spine, dist)
    } else {
        // Open chain (the "bite arc"): a cone band with flush end trims.
        blend_open_circular_chain(solid, chain_edges, spine, dist, true)
    }
}

/// Replace the closed rim loop of `cap` (its outer wire, or a matching inner hole
/// wire) with the contact ring, mapping each rim fragment to `contact` by param.
fn trim_cap_closed_loop(
    cap: &Face,
    spine: &Edge,
    contact: &Edge,
    chain_edges: &[Edge],
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(cap)?;
    let loop_matches = |wire: &Wire| -> bool {
        let edges = wire.edges();
        edges.len() == chain_edges.len()
            && edges.iter().all(|e| {
                chain_edges
                    .iter()
                    .any(|spine_edge| same_undirected_edge(e, spine_edge))
            })
    };
    let remap = |wire: &Wire| -> Result<Wire, RollingBallError> {
        let mut new_edges = Vec::new();
        for e in wire.edges() {
            new_edges.push(contact_subedge_for_spine_edge(spine, contact, &e)?);
        }
        Ok(Wire::from_edges(new_edges))
    };

    if let Some(outer) = cap.outer_wire() {
        if loop_matches(&outer) {
            let new_outer = remap(&outer)?;
            return Ok(Face::with_wires(
                cap.surface().cloned(),
                Some(new_outer),
                cap.inner_wires(),
                cap.orientation(),
            ));
        }
    }
    let mut new_inners = Vec::new();
    let mut found = false;
    for inner in cap.inner_wires() {
        if !found && loop_matches(&inner) {
            new_inners.push(remap(&inner)?);
            found = true;
        } else {
            new_inners.push(inner);
        }
    }
    if found {
        return Ok(Face::with_wires(
            cap.surface().cloned(),
            cap.outer_wire(),
            new_inners,
            cap.orientation(),
        ));
    }
    Err(RollingBallError::SpineNotOnFace)
}

fn circular_chain_support_faces(
    solid: &Solid,
    edges: &[Edge],
) -> Result<(Face, Vec<Face>), RollingBallError> {
    let mut plane: Option<Face> = None;
    for face in adjacent_faces(solid, &edges[0]) {
        if !matches!(face.surface(), Some(GeomSurface::Plane(_))) {
            continue;
        }
        if edges.iter().skip(1).all(|edge| {
            adjacent_faces(solid, edge)
                .into_iter()
                .any(|candidate| same_face(&candidate, &face))
        }) {
            plane = Some(face);
            break;
        }
    }
    let plane = plane.ok_or(RollingBallError::EdgeAdjacency { count: 1 })?;

    let mut cyls: Vec<Face> = Vec::new();
    for edge in edges {
        for face in adjacent_faces(solid, edge) {
            if same_face(&face, &plane) || !matches!(face.surface(), Some(GeomSurface::Cylinder(_)))
            {
                continue;
            }
            if cyls.iter().any(|seen| same_face(seen, &face)) {
                continue;
            }
            if let Some(first) = cyls.first() {
                if !cylinders_same_support(first, &face) {
                    return Err(RollingBallError::UnsolvableAdjacency {
                        reason: AdjacencyReason::UnsupportedSurfacePair,
                    });
                }
            }
            cyls.push(face);
        }
    }
    if cyls.is_empty() {
        return Err(RollingBallError::EdgeAdjacency { count: 1 });
    }
    Ok((plane, cyls))
}

fn cylinders_same_support(a: &Face, b: &Face) -> bool {
    let (Some(GeomSurface::Cylinder(a)), Some(GeomSurface::Cylinder(b))) =
        (a.surface(), b.surface())
    else {
        return false;
    };
    let ac = a.position().location();
    let bc = b.position().location();
    ac.distance(&bc) <= 1.0e-6
        && a.position()
            .direction()
            .dot(&b.position().direction())
            .abs()
            > 0.999_999
        && (a.radius() - b.radius()).abs() <= 1.0e-6
}

/// Find the two faces adjacent to `edge` in `solid`, solve the
/// rolling-ball contact curves, and build the blend face.
pub fn rolling_ball_fillet_edge(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let adjacent = adjacent_faces(solid, edge);
    if adjacent.len() != 2 {
        return Err(RollingBallError::EdgeAdjacency {
            count: adjacent.len(),
        });
    }

    // Both adjacent faces planar → analytic bisector solve. Any curved face
    // present → the curved solver (analytic plane/cylinder paths, else numeric).
    let both_planar = matches!(adjacent[0].surface(), Some(GeomSurface::Plane(_)))
        && matches!(adjacent[1].surface(), Some(GeomSurface::Plane(_)));

    if both_planar {
        // `sew` canonicalizes every sewn shell so each planar face's stored normal
        // agrees with its winding and the shell faces outward — but boolean CUT
        // results can retain tool-derived faces (pocket walls) whose effective
        // normal points INTO the material; the watertight/health gates don't see
        // orientation, so cross-check against the solid itself.
        let n_a = planar_outward_normal_checked(solid, &adjacent[0])?;
        let n_b = planar_outward_normal_checked(solid, &adjacent[1])?;
        if planar_edge_material_wedge_is_concave(solid, edge, n_a, n_b) == Some(true) {
            planar_blend_concave(edge, &adjacent[0], &adjacent[1], n_a, n_b, radius)
        } else {
            planar_blend(edge, &adjacent[0], &adjacent[1], n_a, n_b, radius)
        }
    } else {
        rolling_ball_between_curved_faces(solid, edge, &adjacent[0], &adjacent[1], radius)
    }
}

/// Solve a rolling-ball fillet between a plane and a cylinder.
///
/// Two analytic configurations are handled:
/// - **Cap fillet** — a circular edge with the plane normal parallel to the
///   axis; the ball rolls around the rim and the blend is a torus.
/// - **Longitudinal fillet** — the plane normal is perpendicular to the axis,
///   so the shared edge is a straight generator of the wall; the blend is a
///   cylinder with straight-line contacts (see [`rolling_ball_plane_perp_cylinder`]).
pub fn rolling_ball_between_curved_faces(
    solid: &Solid,
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let (plane_face, cyl_face, is_a_plane) =
        if matches!(face_a.surface(), Some(GeomSurface::Plane(_))) {
            (face_a, face_b, true)
        } else if matches!(face_b.surface(), Some(GeomSurface::Plane(_))) {
            (face_b, face_a, false)
        } else {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::UnsupportedSurfacePair,
            });
        };

    let _plane = match plane_face.surface() {
        Some(GeomSurface::Plane(p)) => p,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    let cyl = match cyl_face.surface() {
        Some(GeomSurface::Cylinder(c)) => c,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };

    let axis_dir = cyl.position().direction();
    let p0 = edge.source().point();
    let n_plane = face_outward_normal_at(plane_face, p0)?;

    // Cap fillet: a circular edge with the plane normal parallel to the axis.
    // The ball rolls around the rim, producing a toroidal blend surface.
    if let Some(GeomCurve::Circle(circle)) = edge.curve() {
        let center = circle.center();
        let spine_r = circle.radius();

        if n_plane.is_parallel(&axis_dir, 1e-4) {
            let concave_cut = is_concave_cut_cylinder(solid, cyl_face);
            let major_radius = if concave_cut {
                spine_r + radius
            } else {
                spine_r - radius
            };
            if major_radius <= tolerance::CONFUSION {
                return Err(RollingBallError::InvalidRadius { radius });
            }

            // Parametrise the contacts in the SPINE CIRCLE's own frame, not the
            // cylinder's. The edge's `first()/last()` params are angles in the spine
            // circle's frame; a reconstructed sketch-arc rim has a different
            // x-direction than the cylinder it bounds, so using `cyl`'s frame here
            // swept the blend over the wrong arc range (an extruded sketch arc would
            // fillet a different quarter of the rim than the selected edge).
            let cax = circle.axis();
            let cxr = circle.position().x_direction();

            let torus_center = center - GeomVec::from_dir(n_plane) * radius;
            let pos = Ax3::new_axes(torus_center, cax, cxr);
            let torus_surf = GeomSurface::torus(ToroidalSurface::new(pos, major_radius, radius));

            let contact_plane_r = major_radius;
            let contact_cyl_height = radius;

            let c_plane = Circle::new(Ax3::new_axes(center, cax, cxr), contact_plane_r);
            let c_cyl = Circle::new(
                Ax3::new_axes(
                    center - GeomVec::from_dir(n_plane) * contact_cyl_height,
                    cax,
                    cxr,
                ),
                spine_r,
            );

            let u0 = edge.first();
            let u1 = edge.last();

            let contact_plane_edge = Edge::new(
                Some(GeomCurve::circle(c_plane)),
                u0,
                u1,
                Vertex::new(c_plane.point(u0)),
                Vertex::new(c_plane.point(u1)),
            );

            let contact_cyl_edge = Edge::new(
                Some(GeomCurve::circle(c_cyl)),
                u0,
                u1,
                Vertex::new(c_cyl.point(u0)),
                Vertex::new(c_cyl.point(u1)),
            );

            let (contact_a, contact_b) = if is_a_plane {
                (contact_plane_edge, contact_cyl_edge)
            } else {
                (contact_cyl_edge, contact_plane_edge)
            };

            let c0 = torus_center + radial(cax, cxr, u0) * major_radius;
            let c1 = torus_center + radial(cax, cxr, u1) * major_radius;
            let centerline = Edge::between_points(c0, c1);

            let r0 = radial(cax, cxr, u0);
            let dir0 = Dir::new(r0.x(), r0.y(), r0.z());
            let r1 = radial(cax, cxr, u1);
            let dir1 = Dir::new(r1.x(), r1.y(), r1.z());

            let t0_vec = GeomVec::from_dir(cax).cross(&r0).normalized().unwrap();
            let t0 = Dir::new(t0_vec.x(), t0_vec.y(), t0_vec.z());
            let t1_vec = GeomVec::from_dir(cax).cross(&r1).normalized().unwrap();
            let t1 = Dir::new(t1_vec.x(), t1_vec.y(), t1_vec.z());

            let arc_start = contact_arc(
                c0,
                t0,
                dir0,
                radius,
                contact_b.start().point(),
                contact_a.start().point(),
            )?;
            let arc_end = contact_arc(
                c1,
                t1,
                dir1,
                radius,
                contact_a.end().point(),
                contact_b.end().point(),
            )?;

            let wire = Wire::from_edges([
                contact_a.clone(),
                arc_end.clone(),
                contact_b.clone().reversed(),
                arc_start.clone(),
            ]);
            let blend_face = Face::new(Some(torus_surf), wire);

            return Ok(RollingBallBlend {
                spine: edge.clone(),
                face_a: face_a.clone(),
                face_b: face_b.clone(),
                contact_a,
                contact_b,
                centerline,
                blend_face,
                start_arc: arc_start,
                end_arc: arc_end,
                radius,
                concave: false,
            });
        }
    }

    // Longitudinal fillet: the plane normal is perpendicular to the axis, so the
    // shared edge is a straight generator of the cylinder wall. Closed-form
    // solve → a cylindrical blend face with straight-line contacts. The same
    // tangency solve serves both wedge signs (the ball rides in the void wedge
    // either way); what differs is the topology bookkeeping, so classify the
    // material wedge here: a boss standing on a wall makes a REFLEX (concave)
    // crease whose fillet ADDS material — the convex corner closures must not
    // run on it.
    if !n_plane.is_parallel(&axis_dir, 1e-4) {
        let mut blend =
            rolling_ball_plane_perp_cylinder(edge, plane_face, cyl_face, is_a_plane, radius)?;
        let p1 = edge.target().point();
        let mid = Pnt::new(
            0.5 * (p0.x() + p1.x()),
            0.5 * (p0.y() + p1.y()),
            0.5 * (p0.z() + p1.z()),
        );
        if let Ok(n_cyl) = face_outward_normal_at(cyl_face, mid) {
            if planar_edge_material_wedge_is_concave(solid, edge, n_plane, n_cyl) == Some(true) {
                blend.concave = true;
            }
        }
        return Ok(blend);
    }

    Err(RollingBallError::UnsolvableAdjacency {
        reason: AdjacencyReason::UnsupportedSurfacePair,
    })
}

/// Solve a rolling-ball fillet for a straight generator edge between a plane
/// (whose normal is perpendicular to the cylinder axis) and a cylinder wall.
///
/// The plane is parallel to the cylinder axis, so the shared edge is a straight
/// generator line. The cross-section (perpendicular to the axis) reduces to a
/// 2D problem of a ball tangent to a line (the plane) and a circle (the
/// cylinder wall); solving it in closed form gives straight-line contacts on
/// both faces and a cylindrical blend surface of radius `radius`.
fn rolling_ball_plane_perp_cylinder(
    edge: &Edge,
    plane_face: &Face,
    cyl_face: &Face,
    is_a_plane: bool,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let cyl = match cyl_face.surface() {
        Some(GeomSurface::Cylinder(c)) => c,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    let axis = cyl.position().direction();
    let axis_vec = GeomVec::from_dir(axis);
    let axis_loc = cyl.position().location();
    let r_cyl = cyl.radius();

    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let n_plane = face_outward_normal_at(plane_face, p0)?;
    let n_vec = GeomVec::from_dir(n_plane);

    // The plane must contain the axis → its normal is perpendicular to the axis.
    if n_plane.is_parallel(&axis, 1e-4) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    }

    // Radial direction at the edge + axial parameter of p0.
    let z0 = (p0 - axis_loc).dot(&axis_vec);
    let o_perp = axis_loc + axis_vec * z0;
    let radial_vec = p0 - o_perp;
    let r_hat = GeomVec::from_dir(
        radial_vec
            .normalized()
            .ok_or(RollingBallError::DegenerateSpine)?,
    );
    // The edge must actually lie on the cylinder wall.
    if (radial_vec.magnitude() - r_cyl).abs() > 1e-6 * r_cyl.max(1.0) + 1e-6 {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    }

    // Convex cross-section solve. `c = cos(dihedral)` between the cylinder
    // radial and the plane normal at the edge; reduces to the classic
    // `α = √(R_c·(R_c + 2r)) − R_c` offset when the meeting is square (c = 0).
    let c = r_hat.dot(&n_vec);
    if c <= -1.0 + 1e-6 {
        return Err(RollingBallError::InvalidDihedral);
    }
    let a2 = r_cyl * r_cyl + 2.0 * radius * r_cyl / (1.0 + c);
    if a2 <= 0.0 {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::RadiusTooLarge,
        });
    }
    let a = a2.sqrt();
    let b = radius + c * (r_cyl - a);

    // Cross-section offsets relative to the axis point `o_perp`:
    //   ball center, plane contact (foot of perpendicular), cylinder contact.
    let center_off = r_hat * a + n_vec * b;
    let plane_off = r_hat * a + n_vec * (c * (r_cyl - a));
    let cyl_off = (r_hat * a + n_vec * b) * (r_cyl / (r_cyl + radius));

    let z1 = (p1 - axis_loc).dot(&axis_vec);
    let at = |off: GeomVec, z: f64| o_perp + off + axis_vec * z;

    let c0 = at(center_off, z0);
    let c1 = at(center_off, z1);
    let centerline = Edge::between_points(c0, c1);

    let contact_plane_edge = Edge::between_points(at(plane_off, z0), at(plane_off, z1));
    let contact_cyl_edge = Edge::between_points(at(cyl_off, z0), at(cyl_off, z1));
    let (contact_a, contact_b) = if is_a_plane {
        (contact_plane_edge, contact_cyl_edge)
    } else {
        (contact_cyl_edge, contact_plane_edge)
    };

    // Blend surface: a cylinder of radius `radius` swept along the centerline.
    let blend_surf = GeomSurface::cylinder(CylindricalSurface::new(
        Ax3::new_axes(c0, axis, n_plane),
        radius,
    ));

    // End arcs close the blend at each endpoint, swinging in the cross-section
    // plane (normal = axis) around the centerline endpoints.
    let arc_start = contact_arc(
        c0,
        axis,
        n_plane,
        radius,
        contact_b.start().point(),
        contact_a.start().point(),
    )?;
    let arc_end = contact_arc(
        c1,
        axis,
        n_plane,
        radius,
        contact_a.end().point(),
        contact_b.end().point(),
    )?;

    let wire = Wire::from_edges([
        contact_a.clone(),
        arc_end.clone(),
        contact_b.clone().reversed(),
        arc_start.clone(),
    ]);
    let blend_face = Face::new(Some(blend_surf), wire);

    // Reconstruct the original (face_a, face_b) ordering from the plane/cylinder
    // pair and the `is_a_plane` flag.
    let (face_a_clone, face_b_clone) = if is_a_plane {
        (plane_face.clone(), cyl_face.clone())
    } else {
        (cyl_face.clone(), plane_face.clone())
    };

    Ok(RollingBallBlend {
        spine: edge.clone(),
        face_a: face_a_clone,
        face_b: face_b_clone,
        contact_a,
        contact_b,
        centerline,
        blend_face,
        start_arc: arc_start,
        end_arc: arc_end,
        radius,
        concave: false,
    })
}

/// Solve a rolling-ball fillet for `edge` between two known adjacent planar
/// faces.
pub fn rolling_ball_between_planar_faces(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let n_a = planar_outward_normal(face_a)?;
    let n_b = planar_outward_normal(face_b)?;
    planar_blend(edge, face_a, face_b, n_a, n_b, radius)
}

/// Core rolling-ball planar blend with explicit, already-outward face normals.
/// Factored out of [`rolling_ball_between_planar_faces`] so
/// [`rolling_ball_fillet_edge`] can substitute solid-aware outward normals for a
/// face whose *stored* orientation is inward (a sewn prism cap), which
/// `planar_outward_normal` alone would misread.
fn planar_blend(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    n_a: Dir,
    n_b: Dir,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let spine_vec = p1 - p0;
    let spine_dir = spine_vec
        .normalized()
        .ok_or(RollingBallError::DegenerateSpine)?;

    let inward_a = -GeomVec::from_dir(n_a);
    let inward_b = -GeomVec::from_dir(n_b);
    let bisector = inward_a + inward_b;
    let bisector_dir = bisector
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    // The ball center sits at radius / sin(θ/2) from the edge, where θ is the
    // material wedge angle between the two faces. The unit inward normals span
    // φ = π − θ, so |n̂a + n̂b| = 2·cos(φ/2) = 2·sin(θ/2) — i.e. sin(θ/2) is
    // half the bisector length. (The previous |n̂a × n̂b| / |n̂a + n̂b| evaluated
    // to sin(φ/2) = cos(θ/2), which only coincides with sin(θ/2) at θ = 90°, so
    // every box test passed while non-right wedges placed the center too close
    // to the corner and the blend bulged outside the body.)
    let sin_half = (bisector.magnitude() * 0.5).max(tolerance::CONFUSION);
    if sin_half <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidDihedral);
    }

    let center_offset = GeomVec::from_dir(bisector_dir) * (radius / sin_half);
    let c0 = p0 + center_offset;
    let c1 = p1 + center_offset;

    let contact_offset_a = center_offset + GeomVec::from_dir(n_a) * radius;
    let contact_offset_b = center_offset + GeomVec::from_dir(n_b) * radius;
    let a0 = p0 + contact_offset_a;
    let a1 = p1 + contact_offset_a;
    let b0 = p0 + contact_offset_b;
    let b1 = p1 + contact_offset_b;

    let contact_a = Edge::between_points(a0, a1);
    let contact_b = Edge::between_points(b0, b1);
    let centerline = Edge::between_points(c0, c1);
    let arc_start = contact_arc(c0, spine_dir, n_b, radius, b0, a0)?;
    let arc_end = contact_arc(c1, spine_dir, n_a, radius, a1, b1)?;

    let wire = Wire::from_edges([
        contact_a.clone(),
        arc_end.clone(),
        contact_b.clone().reversed(),
        arc_start.clone(),
    ]);
    let surface = GeomSurface::cylinder(CylindricalSurface::new(
        Ax3::new_axes(c0, spine_dir, n_a),
        radius,
    ));
    let blend_face = Face::new(Some(surface), wire);

    Ok(RollingBallBlend {
        spine: edge.clone(),
        face_a: face_a.clone(),
        face_b: face_b.clone(),
        contact_a,
        contact_b,
        centerline,
        blend_face,
        start_arc: arc_start,
        end_arc: arc_end,
        radius,
        concave: false,
    })
}

/// Rolling-ball blend for a CONCAVE (reflex-material) planar edge — the inner
/// vertical corner of a pocket. The exact mirror of [`planar_blend`]: the ball
/// rides in the *void* wedge, tangent to both planes from their outward sides,
/// and the blend ADDS the material between the band and the corner. Center on
/// the outward bisector, contacts at the feet of the perpendiculars
/// (`center − n·r`), band = same cylinder with the material on the far side of
/// the tube (sew's winding propagation + global outward pass orient it).
fn planar_blend_concave(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    n_a: Dir,
    n_b: Dir,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let spine_vec = p1 - p0;
    let spine_dir = spine_vec
        .normalized()
        .ok_or(RollingBallError::DegenerateSpine)?;

    let out_a = GeomVec::from_dir(n_a);
    let out_b = GeomVec::from_dir(n_b);
    let bisector = out_a + out_b;
    let bisector_dir = bisector
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    // The ball rides the VOID wedge here, so the center sits at
    // radius / sin(θ_void/2). The outward normals point into the void and span
    // φ = π − θ_void, so |n̂a + n̂b| = 2·sin(θ_void/2) — same identity as the
    // convex path (see planar_blend); the previous cross/sum form evaluated to
    // cos(θ_void/2), correct only at θ_void = 90° (the usual pocket corner).
    let sin_half = (bisector.magnitude() * 0.5).max(tolerance::CONFUSION);
    if sin_half <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidDihedral);
    }

    let center_offset = GeomVec::from_dir(bisector_dir) * (radius / sin_half);
    let c0 = p0 + center_offset;
    let c1 = p1 + center_offset;

    // The center sits at distance `radius` on the OUTWARD side of each plane,
    // so the tangency foot is reached by walking back along the normal.
    let contact_offset_a = center_offset - GeomVec::from_dir(n_a) * radius;
    let contact_offset_b = center_offset - GeomVec::from_dir(n_b) * radius;
    let a0 = p0 + contact_offset_a;
    let a1 = p1 + contact_offset_a;
    let b0 = p0 + contact_offset_b;
    let b1 = p1 + contact_offset_b;

    let contact_a = Edge::between_points(a0, a1);
    let contact_b = Edge::between_points(b0, b1);
    let centerline = Edge::between_points(c0, c1);
    // Contact directions seen from the center are the REVERSED normals here
    // (`b0 − c0 = −n_b·r`), so the arc frames start on those.
    let arc_start = contact_arc(c0, spine_dir, n_b.reversed(), radius, b0, a0)?;
    let arc_end = contact_arc(c1, spine_dir, n_a.reversed(), radius, a1, b1)?;

    let wire = Wire::from_edges([
        contact_a.clone(),
        arc_end.clone(),
        contact_b.clone().reversed(),
        arc_start.clone(),
    ]);
    let surface = GeomSurface::cylinder(CylindricalSurface::new(
        Ax3::new_axes(c0, spine_dir, n_a.reversed()),
        radius,
    ));
    let blend_face = Face::new(Some(surface), wire);

    Ok(RollingBallBlend {
        spine: edge.clone(),
        face_a: face_a.clone(),
        face_b: face_b.clone(),
        contact_a,
        contact_b,
        centerline,
        blend_face,
        start_arc: arc_start,
        end_arc: arc_end,
        radius,
        concave: true,
    })
}

/// Classify the material wedge at a straight planar edge: `Some(true)` for a
/// reflex/concave wedge (inner pocket corner), `Some(false)` for the ordinary
/// convex wedge, `None` when the probes disagree (degenerate or too thin to
/// trust — callers should fall back to the convex path).
///
/// The discriminator: `û = normalize(n_a − n_b)` lies in the cross-section
/// plane perpendicular to the wedge bisector. For a material wedge of angle θ
/// the probes `mid ± ε·û` are BOTH inside the material iff θ > π and both in
/// the void iff θ < π (independent of the A/B labelling, which only flips the
/// sign of û).
pub(crate) fn planar_edge_material_wedge_is_concave(
    solid: &Solid,
    edge: &Edge,
    n_a: Dir,
    n_b: Dir,
) -> Option<bool> {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let edge_len = p0.distance(&p1);
    if edge_len <= 10.0 * tolerance::CONFUSION {
        return None;
    }
    let mid = Pnt::new(
        0.5 * (p0.x() + p1.x()),
        0.5 * (p0.y() + p1.y()),
        0.5 * (p0.z() + p1.z()),
    );
    let u = (GeomVec::from_dir(n_a) - GeomVec::from_dir(n_b)).normalized()?;
    let eps = (0.05 * edge_len).clamp(1.0e-4, 0.5);
    let probe = |sign: f64| {
        let p = mid + GeomVec::from_dir(u) * (sign * eps);
        crate::boolean::point_in_solid(&p, solid)
    };
    match (probe(1.0), probe(-1.0)) {
        (true, true) => Some(true),
        (false, false) => Some(false),
        _ => None,
    }
}

/// Public form of the concavity probe for a straight edge located by its
/// endpoints: resolves the edge's two adjacent faces in `solid` and, when both
/// are planar, classifies the material wedge. `None` when the edge can't be
/// resolved, a face is curved, or the probes are inconclusive.
pub fn edge_material_wedge_is_concave(solid: &Solid, edge: &Edge) -> Option<bool> {
    // Snap the request onto the body first: GUI-captured endpoints carry f32
    // quantization (~1e-6) that the raw adjacency tolerance rejects — the same
    // reason `fillet_edges` relocates before blending.
    let edge = relocate_edge(solid, edge).unwrap_or_else(|| edge.clone());
    let adjacent = adjacent_faces(solid, &edge);
    if adjacent.len() != 2 {
        return None;
    }
    // Outward normal of each adjacent face AT the edge midpoint. Planes and
    // cylinder walls are supported — the latter covers the boss-on-wall crease
    // (a straight generator edge between a plane and a cylinder), whose fillet
    // ADDS material exactly like a planar pocket corner.
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let mid = Pnt::new(
        0.5 * (p0.x() + p1.x()),
        0.5 * (p0.y() + p1.y()),
        0.5 * (p0.z() + p1.z()),
    );
    let normal_at = |face: &Face| -> Option<Dir> {
        match face.surface() {
            Some(GeomSurface::Plane(_)) => planar_outward_normal(face).ok(),
            Some(GeomSurface::Cylinder(_)) => face_outward_normal_at(face, mid).ok(),
            _ => None,
        }
    };
    let n_a = normal_at(&adjacent[0])?;
    let n_b = normal_at(&adjacent[1])?;
    planar_edge_material_wedge_is_concave(solid, &edge, n_a, n_b)
}

pub(crate) fn adjacent_faces(solid: &Solid, edge: &Edge) -> Vec<Face> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter(|face| {
            face.wires()
                .into_iter()
                .flat_map(|wire| wire.edges())
                .any(|candidate| {
                    same_undirected_edge(&candidate, edge)
                        || edge_contains_requested_span(&candidate, edge)
                })
        })
        .collect()
}

fn same_undirected_edge(a: &Edge, b: &Edge) -> bool {
    let a0 = a.start().point();
    let a1 = a.end().point();
    let b0 = b.start().point();
    let b1 = b.end().point();
    let tol = 10.0 * tolerance::CONFUSION;
    (a0.distance(&b0) <= tol && a1.distance(&b1) <= tol)
        || (a0.distance(&b1) <= tol && a1.distance(&b0) <= tol)
}

fn edge_contains_requested_span(container: &Edge, requested: &Edge) -> bool {
    let c0 = container.start().point();
    let c1 = container.end().point();
    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let c_len = c0.distance(&c1);
    let r_len = r0.distance(&r1);
    let tol = 10.0 * tolerance::CONFUSION;
    if c_len <= tol || r_len <= tol || r_len > c_len + tol {
        return false;
    }

    let Some(dir) = (c1 - c0).normalized() else {
        return false;
    };
    if point_line_distance(r0, c0, dir) > tol || point_line_distance(r1, c0, dir) > tol {
        return false;
    }

    let dir_vec = GeomVec::from_dir(dir);
    let t0 = (r0 - c0).dot(&dir_vec);
    let t1 = (r1 - c0).dot(&dir_vec);
    t0 >= -tol && t0 <= c_len + tol && t1 >= -tol && t1 <= c_len + tol
}

/// Perpendicular distance from `p` to the infinite line through `origin` with
/// unit direction `dir`.
fn point_line_distance(p: Pnt, origin: Pnt, dir: Dir) -> f64 {
    let v = p - origin;
    let along = GeomVec::from_dir(dir) * v.dot(&GeomVec::from_dir(dir));
    (v - along).magnitude()
}

/// Locate the current edge of `solid` that corresponds to `requested`, tolerating
/// endpoint drift left by earlier blends.
///
/// First tries an exact undirected endpoint match (the common case). Failing
/// that — e.g. when a prior fillet trimmed back the shared corner so one endpoint
/// moved — it matches the current straight edge that is **collinear** with the
/// request and **overlaps** its span the most: the surviving sub-segment of the
/// originally-selected edge. Returns that edge with its *actual* current
/// endpoints, so the blend runs along the surviving spine (the consumed end is
/// already rounded by the earlier fillet). Without this, multi-edge fillets that
/// share a corner fail with [`RollingBallError::SpineNotOnFace`].
pub(crate) fn relocate_edge(solid: &Solid, requested: &Edge) -> Option<Edge> {
    let current = solid.edges();
    if let Some(e) = current.iter().find(|c| same_undirected_edge(c, requested)) {
        return Some(e.clone());
    }
    if let Some(candidate) = current
        .iter()
        .find(|candidate| edge_contains_requested_span(candidate, requested))
    {
        return Some(snap_requested_span_to_container(candidate, requested));
    }

    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let len = r0.distance(&r1);
    if len <= tolerance::CONFUSION {
        return None;
    }
    let dir = (r1 - r0).normalized()?;
    let dir_vec = GeomVec::from_dir(dir);
    let tol = 10.0 * tolerance::CONFUSION;

    let mut best: Option<(f64, Edge)> = None;
    for c in &current {
        let c0 = c.start().point();
        let c1 = c.end().point();
        // Both endpoints must lie on the request's infinite line (collinear).
        if point_line_distance(c0, r0, dir) > tol || point_line_distance(c1, r0, dir) > tol {
            continue;
        }
        // 1-D spans along the request direction; keep the largest overlap with
        // [0, len]. A surviving sub-segment overlaps almost the whole request.
        let t0 = (c0 - r0).dot(&dir_vec);
        let t1 = (c1 - r0).dot(&dir_vec);
        let overlap = t0.max(t1).min(len) - t0.min(t1).max(0.0);
        if overlap <= tol {
            continue;
        }
        if best.as_ref().map_or(true, |(o, _)| overlap > *o) {
            best = Some((overlap, c.clone()));
        }
    }
    best.map(|(_, e)| e)
}

fn snap_requested_span_to_container(container: &Edge, requested: &Edge) -> Edge {
    let c0 = container.start().point();
    let c1 = container.end().point();
    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let c_len = c0.distance(&c1);
    let snap_tol = (c_len * 5.0e-4).clamp(10.0 * tolerance::CONFUSION, 0.01);

    let start = snap_point_to_edge_endpoint(r0, c0, c1, snap_tol);
    let end = snap_point_to_edge_endpoint(r1, c0, c1, snap_tol);
    if (start.distance(&c0) <= tolerance::CONFUSION && end.distance(&c1) <= tolerance::CONFUSION)
        || (start.distance(&c1) <= tolerance::CONFUSION
            && end.distance(&c0) <= tolerance::CONFUSION)
    {
        return container.clone();
    }
    if start.distance(&r0) > tolerance::CONFUSION || end.distance(&r1) > tolerance::CONFUSION {
        return Edge::between_points(start, end);
    }
    requested.clone()
}

fn snap_point_to_edge_endpoint(point: Pnt, a: Pnt, b: Pnt, tol: f64) -> Pnt {
    if point.distance(&a) <= tol {
        a
    } else if point.distance(&b) <= tol {
        b
    } else {
        point
    }
}

pub(crate) fn same_face(a: &Face, b: &Face) -> bool {
    a.id() == b.id()
}

pub(crate) fn endpoint_cap_faces(
    solid: &Solid,
    point: Pnt,
    face_a: &Face,
    face_b: &Face,
) -> Vec<Face> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter(|face| {
            !same_face(face, face_a) && !same_face(face, face_b) && face_contains_point(face, point)
        })
        .collect()
}

fn face_contains_point(face: &Face, point: Pnt) -> bool {
    face.wires().into_iter().any(|wire| {
        wire.edges().into_iter().any(|edge| {
            edge.source().point().distance(&point) <= 10.0 * tolerance::CONFUSION
                || edge.target().point().distance(&point) <= 10.0 * tolerance::CONFUSION
        })
    })
}

/// Shorten `edge` so the endpoint at `keep` stays put and its other endpoint moves
/// to `moved`, PRESERVING the edge's curve.
///
/// `Edge::between_points` always builds a straight line. Using it to shorten an
/// *arc* boundary edge — which happens when a fillet's end is trimmed against a
/// cylindrical cap face (e.g. an extruded sketch-arc wall) — would replace that
/// arc with an off-surface chord, so the cap face's wire no longer lies on its
/// cylinder and the sewn solid fails the watertight/health gate. For a circular
/// edge this rebuilds the true sub-arc; for any other curve it falls back to the
/// straight segment (the previous behaviour, correct for the planar caps).
fn shorten_edge_keep_curve(edge: &Edge, keep: Pnt, moved: Pnt) -> Edge {
    let Some(GeomCurve::Circle(circle)) = edge.curve() else {
        let source = edge.source().point();
        if source.distance(&keep) <= source.distance(&moved) {
            return Edge::between_points(keep, moved);
        }
        return Edge::between_points(moved, keep);
    };
    let circle = *circle;
    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let raw_angle = {
        let v = moved - center;
        v.dot(&y).atan2(v.dot(&x))
    };
    // `source()`/`target()` honour the wire orientation but `first()`/`last()`
    // are RAW storage — on a Reversed edge they belong to the opposite vertices.
    // Pair them consistently, or the kept end anchors to the WRONG parameter and
    // the trimmed arc comes out param/vertex-inconsistent (its stored end vertex
    // and its end parameter name different points). Downstream, discretization
    // walks the bogus param range — a stub arc nowhere near the stored endpoints
    // — leaving a chord across the true arc: mesh cracks, phantom membranes over
    // a circular bite, and the fillet validator rejecting every candidate.
    let (source_param, target_param) = if edge.orientation().is_forward() {
        (edge.first(), edge.last())
    } else {
        (edge.last(), edge.first())
    };
    let keep_is_source =
        keep.distance(&edge.source().point()) <= keep.distance(&edge.target().point());
    let keep_param = if keep_is_source {
        source_param
    } else {
        target_param
    };
    // Unwrap the moved endpoint's angle onto the same branch as the kept end so the
    // sub-arc travels the original short way around, not the reflex complement.
    let mut moved_param = raw_angle;
    use core::f64::consts::PI;
    while moved_param - keep_param > PI {
        moved_param -= 2.0 * PI;
    }
    while keep_param - moved_param > PI {
        moved_param += 2.0 * PI;
    }
    let curve = Some(GeomCurve::circle(circle));
    if keep_is_source {
        Edge::new(
            curve,
            keep_param,
            moved_param,
            edge.source(),
            Vertex::new(moved),
        )
    } else {
        Edge::new(
            curve,
            moved_param,
            keep_param,
            Vertex::new(moved),
            edge.target(),
        )
    }
}

fn move_edge_endpoint_keep_curve(edge: &Edge, old: Pnt, new: Pnt) -> Edge {
    let tol = 10.0 * tolerance::CONFUSION;
    let source = edge.source().point();
    let target = edge.target().point();
    if source.distance(&old) <= tol {
        return rebuild_edge_with_endpoints(edge, new, target);
    }
    if target.distance(&old) <= tol {
        return rebuild_edge_with_endpoints(edge, source, new);
    }
    edge.clone()
}

fn rebuild_edge_with_endpoints(edge: &Edge, start: Pnt, end: Pnt) -> Edge {
    let Some(GeomCurve::Circle(circle)) = edge.curve() else {
        return Edge::between_points(start, end);
    };
    let circle = *circle;
    let start_param = circle_parameter_for_point(&circle, start, edge.first(), edge.last());
    let end_param = circle_parameter_for_point(&circle, end, edge.first(), edge.last());
    Edge::new(
        Some(GeomCurve::circle(circle)),
        start_param,
        end_param,
        Vertex::new(start),
        Vertex::new(end),
    )
}

fn circle_parameter_for_point(circle: &Circle, point: Pnt, first: f64, last: f64) -> f64 {
    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let v = point - center;
    let raw = v.dot(&y).atan2(v.dot(&x));
    let lo = first.min(last);
    let hi = first.max(last);
    let mut best = raw;
    let mut best_score = f64::INFINITY;
    for k in -3..=3 {
        let candidate = raw + (k as f64) * core::f64::consts::TAU;
        let score = if candidate < lo {
            lo - candidate
        } else if candidate > hi {
            candidate - hi
        } else {
            0.0
        };
        if score < best_score {
            best = candidate;
            best_score = score;
        }
    }
    best
}

pub(crate) fn trim_face_along_spine(
    face: &Face,
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let Some(idx) = edges
        .iter()
        .position(|candidate| same_undirected_edge(candidate, spine))
    else {
        let Some(idx) = edges
            .iter()
            .position(|candidate| edge_contains_requested_span(candidate, spine))
        else {
            return Err(RollingBallError::SpineNotOnFace);
        };
        return trim_face_along_subspine(face, &edges, idx, spine, contact);
    };

    let prev_idx = (idx + n - 1) % n;
    let next_idx = (idx + 1) % n;
    let selected = &edges[idx];
    let contact_start = contact_point_for_spine_vertex(spine, contact, selected.source().point());
    let contact_end = contact_point_for_spine_vertex(spine, contact, selected.target().point());

    // --- BLEND OVERFLOW HANDLING / CLAMPING ---
    let mut contact_start_clamped = contact_start;
    let mut contact_end_clamped = contact_end;

    let prev = &edges[prev_idx];
    let next = &edges[next_idx];

    // Overflow clamping is a straight-line span test, so it only applies to
    // STRAIGHT neighbours. A circular neighbour legitimately EXTENDS past the
    // old corner along its own circle (e.g. a concave boss-crease blend whose
    // contact foot sits on the wall's rim arc beyond the seam vertex) — the
    // chord-distance heuristic misreads that as overflow and snaps the contact
    // to the arc's far end, collapsing the trim.
    let is_straight = |e: &Edge| matches!(e.curve(), Some(GeomCurve::Line(_)) | None);

    if is_straight(prev) {
        let len_prev = prev.source().point().distance(&prev.target().point());
        let dist_start_to_prev_start = contact_start.distance(&prev.source().point());
        if dist_start_to_prev_start > len_prev * 0.99 {
            contact_start_clamped = prev.source().point();
        }
    }

    if is_straight(next) {
        let len_next = next.source().point().distance(&next.target().point());
        let dist_end_to_next_end = contact_end.distance(&next.target().point());
        if dist_end_to_next_end > len_next * 0.99 {
            contact_end_clamped = next.target().point();
        }
    }

    let oriented_contact = orient_edge_between(contact, contact_start_clamped, contact_end_clamped);

    let mut new_edges = Vec::with_capacity(n);
    for (i, edge) in edges.iter().enumerate() {
        if i == prev_idx {
            new_edges.push(shorten_edge_keep_curve(
                edge,
                edge.source().point(),
                contact_start_clamped,
            ));
        } else if i == idx {
            new_edges.push(oriented_contact.clone());
        } else if i == next_idx {
            new_edges.push(shorten_edge_keep_curve(
                edge,
                edge.target().point(),
                contact_end_clamped,
            ));
        } else {
            new_edges.push(edge.clone());
        }
    }

    // A mitered corner extends the new blend's side contact onto the prior fillet's
    // own corner vertex, which collapses this face's pre-corner edge to zero length
    // (e.g. the side face's old fillet-end arc shrinks to a point). Drop any such
    // degenerate edges so the trimmed loop stays valid.
    new_edges.retain(|e| e.source().point().distance(&e.target().point()) > tolerance::CONFUSION);

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn trim_face_along_subspine(
    face: &Face,
    edges: &[Edge],
    idx: usize,
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    let selected = &edges[idx];
    let s0 = selected.source().point();
    let s1 = selected.target().point();
    let len2 = (s1 - s0).magnitude_squared();
    if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
        return Err(RollingBallError::DegenerateSpine);
    }

    let a = spine.source().point();
    let b = spine.target().point();
    let ta = (a - s0).dot(&(s1 - s0)) / len2;
    let tb = (b - s0).dot(&(s1 - s0)) / len2;
    let (run_start, run_end) = if ta <= tb { (a, b) } else { (b, a) };
    let contact_start = contact_point_for_spine_vertex(spine, contact, run_start);
    let contact_end = contact_point_for_spine_vertex(spine, contact, run_end);
    let oriented_contact = orient_edge_between(contact, contact_start, contact_end);

    let mut new_edges = Vec::with_capacity(edges.len() + 4);
    for (i, edge) in edges.iter().enumerate() {
        if i != idx {
            new_edges.push(edge.clone());
            continue;
        }

        push_nonzero_edge(&mut new_edges, shorten_edge_keep_curve(edge, s0, run_start));
        push_nonzero_edge(
            &mut new_edges,
            Edge::between_points(run_start, contact_start),
        );
        push_nonzero_edge(&mut new_edges, oriented_contact.clone());
        push_nonzero_edge(&mut new_edges, Edge::between_points(contact_end, run_end));
        push_nonzero_edge(&mut new_edges, shorten_edge_keep_curve(edge, s1, run_end));
    }

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn push_nonzero_edge(edges: &mut Vec<Edge>, edge: Edge) {
    if edge.source().point().distance(&edge.target().point()) > tolerance::CONFUSION {
        edges.push(edge);
    }
}

fn split_blend_face_for_spine_chain(
    blend: &mut RollingBallBlend,
    spine: &Edge,
    spine_edges: &[Edge],
) -> Result<(), RollingBallError> {
    let mut spans: Vec<(f64, f64)> = spine_edges
        .iter()
        .map(|edge| {
            let a = spine_parameter_for_point(spine, edge.source().point())?;
            let b = spine_parameter_for_point(spine, edge.target().point())?;
            Ok(if spine.last() >= spine.first() {
                (a.min(b), a.max(b))
            } else {
                (a.max(b), a.min(b))
            })
        })
        .collect::<Result<_, RollingBallError>>()?;
    spans.sort_by(|a, b| {
        a.0.min(a.1)
            .partial_cmp(&b.0.min(b.1))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut contact_a = Vec::new();
    let mut contact_b = Vec::new();
    for &(a, b) in &spans {
        contact_a.push(edge_on_contact_between_params(&blend.contact_a, a, b)?);
        contact_b.push(edge_on_contact_between_params(&blend.contact_b, a, b)?);
    }

    let mut wire_edges = Vec::with_capacity(contact_a.len() + contact_b.len() + 2);
    wire_edges.extend(contact_a);
    wire_edges.push(blend.end_arc.clone());
    wire_edges.extend(contact_b.into_iter().rev().map(|edge| edge.reversed()));
    wire_edges.push(blend.start_arc.clone());

    blend.blend_face = Face::new(
        blend.blend_face.surface().cloned(),
        Wire::from_edges(wire_edges),
    );
    Ok(())
}

fn trim_face_along_spine_segments(
    face: &Face,
    spine_edges: &[Edge],
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    trim_face_along_spine_segments_clamped(
        face,
        spine_edges,
        spine,
        contact,
        (f64::NEG_INFINITY, f64::INFINITY),
    )
}

/// [`trim_face_along_spine_segments`] with the contact run clamped to the spine
/// param range `clamp` — the open-chain flush termination: the outer contact
/// sub-arcs stop at the cap-plane crossings instead of the rim's own corners,
/// and the adjacent boundary edges shorten to those crossing points (which lie
/// ON their curves, e.g. a box's front-top edge), instead of being dragged off
/// their lines to the un-clamped ring ends.
fn trim_face_along_spine_segments_clamped(
    face: &Face,
    spine_edges: &[Edge],
    spine: &Edge,
    contact: &Edge,
    clamp: (f64, f64),
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 || spine_edges.is_empty() || spine_edges.len() >= n {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let selected: std::collections::HashSet<usize> = edges
        .iter()
        .enumerate()
        .filter_map(|(i, candidate)| {
            spine_edges
                .iter()
                .any(|spine_edge| same_undirected_edge(candidate, spine_edge))
                .then_some(i)
        })
        .collect();
    if selected.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }

    let run_start = selected
        .iter()
        .copied()
        .find(|&i| !selected.contains(&((i + n - 1) % n)))
        .ok_or(RollingBallError::UnsupportedTrimTopology)?;
    let mut run = Vec::new();
    let mut i = run_start;
    loop {
        if !selected.contains(&i) {
            break;
        }
        run.push(i);
        i = (i + 1) % n;
        if i == run_start {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
    }
    if run.len() != selected.len() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let run_end = *run
        .last()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?;
    let prev_idx = (run_start + n - 1) % n;
    let next_idx = (run_end + 1) % n;
    if selected.contains(&prev_idx) || selected.contains(&next_idx) || prev_idx == next_idx {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let run_start_point = edges[run_start].source().point();
    let run_end_point = edges[run_end].target().point();
    // A finite clamp bound is FORCED at the run's outer endpoints, not merely
    // clamped toward: at a flush end the crossing param lies inside the chain
    // span (clamp and force agree), while at a MITER end the band extends
    // BEYOND the chain into the region the earlier fillet vacated — the run
    // endpoint must be pushed OUT to the seam param, and the adjacent boundary
    // edge (the earlier band's contact curve) shortened to that same point.
    let forced_contact_param = |p: Pnt| -> Result<f64, RollingBallError> {
        let t = spine_parameter_for_point(spine, p)?;
        Ok(match (clamp.0.is_finite(), clamp.1.is_finite()) {
            (true, true) => {
                if (t - clamp.0).abs() <= (t - clamp.1).abs() {
                    clamp.0
                } else {
                    clamp.1
                }
            }
            (true, false) => t.max(clamp.0),
            (false, true) => t.min(clamp.1),
            (false, false) => t,
        })
    };
    let clamped_contact_point = |p: Pnt| -> Result<Pnt, RollingBallError> {
        let Some(curve) = contact.curve() else {
            return Ok(contact_point_for_spine_vertex(spine, contact, p));
        };
        Ok(curve.point(forced_contact_param(p)?))
    };
    let contact_start = clamped_contact_point(run_start_point)?;
    let contact_end = clamped_contact_point(run_end_point)?;

    // The chain's param extremes: the fragments owning them get their outer end
    // forced to the corresponding finite clamp bound (a miter extension).
    let mut frag_spans: std::collections::HashMap<usize, (f64, f64)> =
        std::collections::HashMap::new();
    let mut chain_lo = f64::INFINITY;
    let mut chain_hi = f64::NEG_INFINITY;
    for &i in &selected {
        let (o0, o1) = spine_param_span(spine, &edges[i])?;
        chain_lo = chain_lo.min(o0.min(o1));
        chain_hi = chain_hi.max(o0.max(o1));
        frag_spans.insert(i, (o0, o1));
    }

    let mut new_edges = Vec::with_capacity(n);
    for (i, edge) in edges.iter().enumerate() {
        if selected.contains(&i) {
            let (o0, o1) = frag_spans[&i];
            let asc = o1 >= o0;
            let (flo, fhi) = (o0.min(o1), o0.max(o1));
            let mut lo = flo.max(clamp.0);
            let mut hi = fhi.min(clamp.1);
            if clamp.0.is_finite() && flo <= chain_lo + 1.0e-9 && clamp.0 < hi {
                lo = clamp.0;
            }
            if clamp.1.is_finite() && fhi >= chain_hi - 1.0e-9 && clamp.1 > lo {
                hi = clamp.1;
            }
            // Interval collapsed or inverted: the fragment is entirely outside
            // the clamp range — drop it.
            if hi - lo <= 1.0e-9 {
                continue;
            }
            let (t0, t1) = if asc { (lo, hi) } else { (hi, lo) };
            new_edges.push(edge_on_contact_between_params(contact, t0, t1)?);
        } else {
            let moved_start = move_edge_endpoint_keep_curve(edge, run_start_point, contact_start);
            let moved = move_edge_endpoint_keep_curve(&moved_start, run_end_point, contact_end);
            new_edges.push(moved);
        }
    }

    new_edges.retain(|e| e.source().point().distance(&e.target().point()) > tolerance::CONFUSION);
    rebuild_face(face, Wire::from_edges(new_edges))
}

pub(crate) fn trim_face_at_corner(
    face: &Face,
    corner: Pnt,
    contact_a: Pnt,
    contact_b: Pnt,
    arc: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
    let outer = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?;

    // The corner usually sits on the outer loop (a box top losing its corner
    // region to a convex fillet). For a CONCAVE blend on a pocket edge, the
    // face at the pocket opening (the rim) holds the corner on an INNER wire
    // instead — the same splice shrinks the hole there, i.e. the face GAINS
    // the cross-section region the added material now caps.
    if let Ok(new_outer) = splice_arc_at_wire_corner(&outer, corner, contact_a, contact_b, arc) {
        return rebuild_face(face, new_outer);
    }
    let inners = face.inner_wires();
    for (k, inner) in inners.iter().enumerate() {
        if let Ok(new_inner) = splice_arc_at_wire_corner(inner, corner, contact_a, contact_b, arc) {
            let mut new_inners = inners.clone();
            new_inners[k] = new_inner;
            return Ok(Face::with_wires(
                face.surface().cloned(),
                Some(outer),
                new_inners,
                face.orientation(),
            ));
        }
    }
    Err(RollingBallError::UnsupportedTrimTopology)
}

/// Replace a wire's `corner` vertex with `arc`: shorten the two loop edges
/// meeting at the corner back to the blend's contact feet and thread the arc
/// between them. Pure wire surgery — agnostic to which side of the arc the
/// face keeps, so it serves both convex (region removed) and concave (region
/// gained) endpoint trims.
fn splice_arc_at_wire_corner(
    wire: &Wire,
    corner: Pnt,
    contact_a: Pnt,
    contact_b: Pnt,
    arc: &Edge,
) -> Result<Wire, RollingBallError> {
    let edges = wire.edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let Some(next_idx) = edges
        .iter()
        .position(|edge| edge.source().point().distance(&corner) <= 10.0 * tolerance::CONFUSION)
    else {
        return Err(RollingBallError::UnsupportedTrimTopology);
    };
    let prev_idx = (next_idx + n - 1) % n;
    if edges[prev_idx].target().point().distance(&corner) > 10.0 * tolerance::CONFUSION {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let prev = &edges[prev_idx];
    let next = &edges[next_idx];
    let da_prev = point_segment_distance(contact_a, prev.source().point(), prev.target().point());
    let db_prev = point_segment_distance(contact_b, prev.source().point(), prev.target().point());
    let (prev_contact, next_contact) = if da_prev <= db_prev {
        (contact_a, contact_b)
    } else {
        (contact_b, contact_a)
    };
    let oriented_arc = orient_edge_between(arc, prev_contact, next_contact);

    let mut new_edges = Vec::with_capacity(n + 1);
    for offset in 0..n {
        let i = (prev_idx + offset) % n;
        if i == prev_idx {
            new_edges.push(shorten_edge_keep_curve(
                prev,
                prev.source().point(),
                prev_contact,
            ));
            new_edges.push(oriented_arc.clone());
        } else if i == next_idx {
            new_edges.push(shorten_edge_keep_curve(
                next,
                next.target().point(),
                next_contact,
            ));
        } else {
            new_edges.push(edges[i].clone());
        }
    }

    Ok(Wire::from_edges(new_edges))
}

fn ensure_trimmable_face(face: &Face) -> Result<(), RollingBallError> {
    if !matches!(
        face.surface(),
        Some(GeomSurface::Plane(_)) | Some(GeomSurface::Cylinder(_))
    ) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::NotPlaneOrAnalytic,
        });
    }
    if face.outer_wire().is_none() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    // Inner wires (holes) are allowed: only the outer loop is re-trimmed for the
    // blend, and `rebuild_face` carries the holes through untouched. A hole that
    // intrudes on the trimmed boundary is caught by the final watertight gate.
    Ok(())
}

fn rebuild_face(face: &Face, outer: Wire) -> Result<Face, RollingBallError> {
    // Preserve any inner (hole) loops. A face being trimmed for a blend may own
    // holes — e.g. a top face with a bored pocket — that sit in the interior, far
    // from the edge being filleted. Dropping them would re-fill the holes (and
    // break watertightness against the bore wall). If a hole *does* reach the
    // trimmed boundary, the result fails the watertight/health gate in
    // `fillet_planar_edge` and surfaces as a clean error rather than bad geometry.
    Ok(Face::with_wires(
        face.surface().cloned(),
        Some(outer),
        face.inner_wires(),
        face.orientation(),
    ))
}

pub(crate) fn contact_point_for_spine_vertex(spine: &Edge, contact: &Edge, point: Pnt) -> Pnt {
    let spine_start = spine.source().point();
    if point.distance(&spine_start) <= 10.0 * tolerance::CONFUSION {
        contact.start().point()
    } else {
        contact.end().point()
    }
}

fn contact_subedge_for_spine_edge(
    spine: &Edge,
    contact: &Edge,
    edge: &Edge,
) -> Result<Edge, RollingBallError> {
    let (t0, t1) = spine_param_span(spine, edge)?;
    edge_on_contact_between_params(contact, t0, t1)
}

/// Map a rim fragment onto the spine's angular frame: `(t0, t1)` such that the
/// param interval traverses the SAME arc as `edge` (source → target, through the
/// fragment's own midpoint). Snapping each endpoint independently is ambiguous
/// at the 0/TAU seam: a fragment ending there can snap to the wrong branch,
/// handing every param-interval consumer (curve discretization, shared-edge
/// keys) the COMPLEMENTARY arc even though the endpoints — and therefore `sew`
/// and the watertightness check — still line up.
fn spine_param_span(spine: &Edge, edge: &Edge) -> Result<(f64, f64), RollingBallError> {
    use core::f64::consts::{PI, TAU};

    let t0 = spine_parameter_for_point(spine, edge.source().point())?;
    let mut t1 = spine_parameter_for_point(spine, edge.target().point())?;
    if let Some(curve) = edge.curve() {
        // A circle's param IS its angle, so the fragment's angular extent is its
        // raw param span, independent of the spine's frame.
        let span = (edge.last() - edge.first()).abs();
        if span <= tolerance::CONFUSION {
            return Ok((t0, t1));
        }
        if span >= TAU - tolerance::ANGULAR {
            // Full circle: direction is immaterial, but the interval must span
            // TAU (endpoint-snapping would collapse it to zero).
            return Ok((t0, t0 + TAU));
        }
        let mid = curve.point(0.5 * (edge.first() + edge.last()));
        let mut tm = spine_parameter_for_point(spine, mid)?;
        // Nearest representative of the midpoint angle to t0; the true offset is
        // span/2 < PI, so this branch choice is exact.
        while tm - t0 > PI {
            tm -= TAU;
        }
        while t0 - tm > PI {
            tm += TAU;
        }
        t1 = if tm >= t0 { t0 + span } else { t0 - span };
    }
    Ok((t0, t1))
}

fn edge_on_contact_between_params(
    contact: &Edge,
    t0: f64,
    t1: f64,
) -> Result<Edge, RollingBallError> {
    let Some(curve) = contact.curve().cloned() else {
        return Ok(Edge::between_points(
            contact.start().point(),
            contact.end().point(),
        ));
    };
    let p0 = curve.point(t0);
    let p1 = curve.point(t1);
    Ok(Edge::new(
        Some(curve),
        t0,
        t1,
        Vertex::new(p0),
        Vertex::new(p1),
    ))
}

fn spine_parameter_for_point(spine: &Edge, point: Pnt) -> Result<f64, RollingBallError> {
    let Some(GeomCurve::Circle(circle)) = spine.curve() else {
        let a = spine.start().point();
        let b = spine.end().point();
        let ab = b - a;
        let len2 = ab.magnitude_squared();
        if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
            return Err(RollingBallError::DegenerateSpine);
        }
        let t = (point - a).dot(&ab) / len2;
        return Ok(spine.first() + (spine.last() - spine.first()) * t);
    };

    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let v = point - center;
    let raw = v.dot(&y).atan2(v.dot(&x));
    let lo = spine.first().min(spine.last());
    let hi = spine.first().max(spine.last());
    let mut best = raw;
    let mut best_score = f64::INFINITY;
    for k in -3..=3 {
        let candidate = raw + (k as f64) * core::f64::consts::TAU;
        let score = if candidate < lo {
            lo - candidate
        } else if candidate > hi {
            candidate - hi
        } else {
            0.0
        };
        if score < best_score {
            best = candidate;
            best_score = score;
        }
    }
    Ok(best)
}

pub(crate) fn orient_edge_between(edge: &Edge, start: Pnt, end: Pnt) -> Edge {
    let tol = 10.0 * tolerance::CONFUSION;
    if edge.start().point().distance(&start) <= tol && edge.end().point().distance(&end) <= tol {
        edge.clone()
    } else {
        edge.reversed()
    }
}

fn point_segment_distance(point: Pnt, a: Pnt, b: Pnt) -> f64 {
    let ab = b - a;
    let len2 = ab.magnitude_squared();
    if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
        return point.distance(&a);
    }
    let t = ((point - a).dot(&ab) / len2).clamp(0.0, 1.0);
    point.distance(&(a + ab * t))
}

/// [`planar_outward_normal`] cross-checked against the solid: probe just off
/// the face interior on both sides. If material sits on the stored-outward
/// side and void on the stored-inward side, the stored orientation is
/// inverted (a boolean CUT can keep tool faces this way — watertightness and
/// loop-contiguity checks don't inspect orientation) — flip it. Ambiguous
/// probes (thin walls, a centroid that misses a holed face) keep the stored
/// normal.
pub(crate) fn planar_outward_normal_checked(
    solid: &Solid,
    face: &Face,
) -> Result<Dir, RollingBallError> {
    let n = planar_outward_normal(face)?;
    let Some(wire) = face.outer_wire() else {
        return Ok(n);
    };
    let mut acc = GeomVec::new(0.0, 0.0, 0.0);
    let mut count = 0usize;
    let mut bb_lo = [f64::INFINITY; 3];
    let mut bb_hi = [f64::NEG_INFINITY; 3];
    for edge in wire.edges() {
        let p = edge.source().point();
        acc += p - Pnt::origin();
        count += 1;
        for (k, v) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            bb_lo[k] = bb_lo[k].min(v);
            bb_hi[k] = bb_hi[k].max(v);
        }
    }
    if count == 0 {
        return Ok(n);
    }
    let centroid = Pnt::origin() + acc * (1.0 / count as f64);
    let diag = ((bb_hi[0] - bb_lo[0]).powi(2)
        + (bb_hi[1] - bb_lo[1]).powi(2)
        + (bb_hi[2] - bb_lo[2]).powi(2))
    .sqrt();
    let eps = (diag * 1.0e-3).clamp(1.0e-5, 0.05);
    let outward_probe = centroid + GeomVec::from_dir(n) * eps;
    let inward_probe = centroid - GeomVec::from_dir(n) * eps;
    let outward_material = crate::boolean::point_in_solid(&outward_probe, solid);
    let inward_material = crate::boolean::point_in_solid(&inward_probe, solid);
    if outward_material && !inward_material {
        return Ok(n.reversed());
    }
    Ok(n)
}

pub(crate) fn planar_outward_normal(face: &Face) -> Result<Dir, RollingBallError> {
    let normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    Ok(if face.orientation() == Orientation::Reversed {
        normal.reversed()
    } else {
        normal
    })
}

fn contact_arc(
    center: Pnt,
    mut axis: Dir,
    xdir: Dir,
    radius: f64,
    start: Pnt,
    end: Pnt,
) -> Result<Edge, RollingBallError> {
    let start_v = (start - center)
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    let end_v = (end - center)
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    // The edge's params are 0..angle measured FROM `start`, so the circle's
    // x-direction must be the start contact's radial direction — otherwise the
    // stored parameter range names a different portion of the circle than the
    // stored vertices (param/vertex-inconsistent edge), and discretization
    // draws the wrong arc. Callers pass an xdir that usually equals `start_v`
    // (planar blends), but the plane⊥cylinder solver's does not — derive it.
    let xdir = {
        let a = GeomVec::from_dir(axis);
        let sv = GeomVec::from_dir(start_v);
        match (sv - a * sv.dot(&a)).normalized() {
            Some(perp) => Dir::new(perp.x(), perp.y(), perp.z()),
            None => xdir,
        }
    };
    // A rolling-ball blend always sweeps the minor (convex) arc between its two
    // contact points. `angle_about_axis` returns a value in [0, TAU) measured in
    // one fixed rotational sense, so callers that feed `start`/`end` in opposite
    // order get complementary angles — one the 90°-ish convex arc, the other its
    // 270°-ish reflex complement. A `> PI` result means we measured the long way
    // around: flip the axis to swing the short way instead (this also re-points
    // the circle so the curve still runs start -> end). Without this clamp one of
    // the two end arcs of a planar edge blend sweeps the reflex side, producing a
    // cylinder face that renders round at one end and flat (chamfer-like) at the
    // other.
    let mut angle = angle_about_axis(start_v, end_v, axis);
    if angle > core::f64::consts::PI {
        axis = axis.reversed();
        angle = angle_about_axis(start_v, end_v, axis);
    }
    if angle <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidDihedral);
    }

    let circle = Circle::new(Ax3::new_axes(center, axis, xdir), radius);
    Ok(Edge::new(
        Some(GeomCurve::circle(circle)),
        0.0,
        angle,
        Vertex::new(start),
        Vertex::new(end),
    ))
}

fn angle_about_axis(start: Dir, end: Dir, axis: Dir) -> f64 {
    let x = GeomVec::from_dir(start);
    let y = GeomVec::from_dir(axis).cross(&x);
    let end_v = GeomVec::from_dir(end);
    let mut angle = end_v.dot(&y).atan2(end_v.dot(&x));
    if angle < 0.0 {
        angle += core::f64::consts::TAU;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_geom::{Plane, Surface};
    use openrcad_primitives::make_box;

    #[test]
    fn ordered_clamp_accepts_reversed_surface_bounds() {
        assert_eq!(clamp_ordered(2.12, 2.14, 2.09), 2.12);
        assert_eq!(clamp_ordered(2.00, 2.14, 2.09), 2.09);
        assert_eq!(clamp_ordered(2.20, 2.14, 2.09), 2.14);
    }

    #[test]
    fn concave_probe_classifies_pocket_and_box_edges() {
        // Pocketed block: inner vertical edge is concave, outer box edge convex.
        let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 5.0, 4.0), 10.0, 10.0, 6.0);
        let body = crate::boolean::boolean(&block, &tool, crate::BooleanOp::Cut);

        let pocket_edge =
            Edge::between_points(Pnt::new(5.0, 5.0, 4.0), Pnt::new(5.0, 5.0, 10.0));
        assert_eq!(
            edge_material_wedge_is_concave(&body, &pocket_edge),
            Some(true)
        );

        let outer_edge =
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(0.0, 0.0, 10.0));
        assert_eq!(
            edge_material_wedge_is_concave(&body, &outer_edge),
            Some(false)
        );
    }

    #[test]
    fn solves_contacts_for_box_edge() {
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);

        let blend = rolling_ball_fillet_edge(&solid, &edge, 0.5).unwrap();
        assert!((blend.radius - 0.5).abs() < 1e-12);
        assert!(matches!(
            blend.blend_face.surface(),
            Some(GeomSurface::Cylinder(_))
        ));

        let c0 = blend.centerline.start().point();
        assert!((c0.x() - 0.5).abs() < 1e-9);
        assert!((c0.y() - 0.5).abs() < 1e-9);

        let a0 = blend.contact_a.start().point();
        let b0 = blend.contact_b.start().point();
        let on_x_face = a0.x().abs() < 1e-9 || b0.x().abs() < 1e-9;
        let on_y_face = a0.y().abs() < 1e-9 || b0.y().abs() < 1e-9;
        assert!(on_x_face, "one contact curve lies on the X-min face");
        assert!(on_y_face, "one contact curve lies on the Y-min face");
        assert!(blend.blend_face.outer_wire().unwrap().is_closed());
    }

    #[test]
    fn fillets_single_planar_box_edge_into_watertight_solid() {
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);

        let filleted = fillet_planar_edge(&solid, &edge, 0.5).unwrap();

        assert_eq!(filleted.face_count(), 7);
        assert!(filleted.health_report().is_healthy());
        assert!(filleted.is_watertight());
        let cylinders = filleted
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 1);
    }

    #[test]
    fn box_edge_blend_takes_the_minor_arc_on_both_ends() {
        // Regression: `planar_blend` feeds `contact_arc` the two end arcs in
        // opposite point-order (`b -> a` vs `a -> b`). `angle_about_axis` measures
        // in one fixed rotational sense, so without the minor-arc clamp one end
        // sweeps the convex ~90° arc and the other its ~270° reflex complement —
        // a single cylinder face that renders round at one end and flat
        // (chamfer-like) at the other. A convex blend must take the minor
        // (<= PI) arc on *both* ends.
        use core::f64::consts::PI;

        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);
        let blend = rolling_ball_fillet_edge(&solid, &edge, 0.5).unwrap();

        // The arc edges store the sweep as their parameter span (first = 0).
        let start_sweep = blend.start_arc.last() - blend.start_arc.first();
        let end_sweep = blend.end_arc.last() - blend.end_arc.first();
        assert!(
            start_sweep <= PI + 1e-9,
            "start arc swept the reflex side: {start_sweep} rad"
        );
        assert!(
            end_sweep <= PI + 1e-9,
            "end arc swept the reflex side: {end_sweep} rad"
        );
        // A square box edge blends a clean quarter-round on each end.
        assert!(
            (start_sweep - PI / 2.0).abs() < 1e-9,
            "start sweep {start_sweep}"
        );
        assert!((end_sweep - PI / 2.0).abs() < 1e-9, "end sweep {end_sweep}");
    }

    #[test]
    fn error_variant_set_is_renamed() {
        // Compile-time confirmation that the obsolete variants (`NonPlanarAdjacentFace`,
        // `CornerAdjacency`) are gone and the new vocabulary exists.
        let err = RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        };
        assert!(format!("{err}").contains("could not be blended"));
        let diverged = RollingBallError::NewtonDiverged { iterations: 8 };
        assert!(format!("{diverged}").contains("diverged"));
        let blend = RollingBallError::BlendSurfaceBuild("sweep");
        assert!(format!("{blend}").contains("sweep"));

        // Exhaustiveness guard: every current variant is named, so a future
        // rename/addition cannot silently drop a branch.
        fn _exhaustive(e: &RollingBallError) -> &'static str {
            match e {
                RollingBallError::InvalidRadius { .. } => "r",
                RollingBallError::DegenerateSpine => "d",
                RollingBallError::EdgeAdjacency { .. } => "e",
                RollingBallError::UnsolvableAdjacency { .. } => "u",
                RollingBallError::InvalidDihedral => "i",
                RollingBallError::SpineNotOnFace => "s",
                RollingBallError::UnsupportedTrimTopology => "t",
                RollingBallError::NewtonDiverged { .. } => "n",
                RollingBallError::BlendSurfaceBuild(_) => "b",
                RollingBallError::InvalidTopology => "h",
            }
        }
    }

    #[test]
    fn rejects_boundary_edge_without_two_faces() {
        let face = Face::new(
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
        let solid = Solid::new(openrcad_topo::Shell::from_faces([face.clone()]));
        let edge = face.outer_wire().unwrap().edges()[0].clone();

        assert!(matches!(
            rolling_ball_fillet_edge(&solid, &edge, 0.1),
            Err(RollingBallError::EdgeAdjacency { count: 1 })
        ));
    }

    #[test]
    fn solves_curved_face_blending_cylinder() {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;

        let solid = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 6.0);
        let edge = solid
            .edges()
            .into_iter()
            .find(|e| {
                e.start().point().z().abs() < 1e-9
                    && matches!(e.curve(), Some(GeomCurve::Circle(_)))
            })
            .unwrap();

        let filleted = fillet_planar_edge(&solid, &edge, 0.2).unwrap();
        assert!(filleted.is_watertight());
        let tori = filleted
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Torus(_))))
            .count();
        assert!(tori > 0);
    }

    // Goal-test for the longitudinal (plane⊥axis) solver. Currently blocked on
    // the boolean engine: the cylinder-flat `Cut` below is a partial-imprint
    // case (the cut plane only partially crosses the cylinder wall), which the
    // split pass does not yet imprint into a clean D-shape — so the +Y
    // generator edge the solver needs never appears. See the boolean-frontier
    // `#[ignore]`d tests in tests/robustness.rs. Run with `cargo test --ignored`.
    #[test]
    #[ignore = "blocked on boolean cylinder-flat (longitudinal partial-imprint) robustness"]
    fn solves_longitudinal_plane_cylinder_fillet() {
        use crate::{boolean, BooleanOp};
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;

        // Cylinder (R=2, H=6, axis Z) shaved flat at x=0.5. The plane (x=0.5,
        // normal ⊥ axis) meets the cylinder wall along two straight generator
        // edges — the longitudinal (plane⊥axis) config.
        let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 6.0);
        let cutter = make_box(&Pnt::new(0.5, -3.0, -1.0), 10.0, 6.0, 8.0);
        let dshape = boolean(&cyl, &cutter, BooleanOp::Cut);

        // The +Y generator: a straight Z-line at (0.5, √(4-0.25), z).
        let y_gen = (4.0_f64 - 0.5 * 0.5).sqrt();
        let edge = dshape
            .edges()
            .into_iter()
            .find(|e| {
                let p0 = e.start().point();
                let p1 = e.end().point();
                (p0.x() - 0.5).abs() < 1e-6
                    && (p1.x() - 0.5).abs() < 1e-6
                    && (p0.y() - y_gen).abs() < 1e-4
                    && (p1.y() - y_gen).abs() < 1e-4
                    && (p0.z() - p1.z()).abs() > 5.0
            })
            .expect("D-shape has a +Y generator edge");

        let r = 0.3_f64;
        let blend = rolling_ball_fillet_edge(&dshape, &edge, r).expect("longitudinal solve");

        // Blend surface is a cylinder of the fillet radius.
        let surf_r = match blend.blend_face.surface() {
            Some(GeomSurface::Cylinder(c)) => c.radius(),
            other => panic!("expected cylindrical blend, got {other:?}"),
        };
        assert!((surf_r - r).abs() < 1e-9, "blend cylinder radius {surf_r}");

        // One contact lies on the plane (x=0.5), the other on the wall
        // (distance √(x²+y²) = 2 from the Z-axis).
        let a0 = blend.contact_a.start().point();
        let b0 = blend.contact_b.start().point();
        let on_plane = |p: Pnt| (p.x() - 0.5).abs() < 1e-6;
        let on_cyl = |p: Pnt| ((p.x() * p.x() + p.y() * p.y()).sqrt() - 2.0).abs() < 1e-6;
        assert!(
            (on_plane(a0) && on_cyl(b0)) || (on_cyl(a0) && on_plane(b0)),
            "contacts must lie on the plane and the cylinder: a0={a0:?} b0={b0:?}"
        );

        // Full surgical fillet must remain watertight.
        let filleted = fillet_planar_edge(&dshape, &edge, r).expect("longitudinal fillet");
        assert!(
            filleted.is_watertight(),
            "longitudinal fillet not watertight: {:?}",
            filleted.manifold_report()
        );
    }

    #[test]
    fn solves_gregory_corner_patch() {
        let corner = Pnt::origin();
        let p_a = Pnt::new(1.0, 0.0, 0.0);
        let p_b = Pnt::new(0.0, 1.0, 0.0);
        let patch = make_gregory_corner_patch(corner, p_a, p_b, 0.1);
        assert!(patch.surface().is_some());
        if let Some(GeomSurface::Gregory(surf)) = patch.surface() {
            assert_eq!(surf.point(0.0, 0.0), corner);
            assert_eq!(surf.point(1.0, 0.0), p_a);
            assert_eq!(surf.point(1.0, 1.0), p_b);
        } else {
            panic!("Expected GregorySurface");
        }
    }

    #[test]
    fn handles_blend_overflow_clamping() {
        let solid = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);
        let edge = solid
            .edges()
            .into_iter()
            .find(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 0.9
            })
            .unwrap();

        let filleted = fillet_planar_edge(&solid, &edge, 1.5);
        assert!(filleted.is_ok());
        assert!(filleted.unwrap().is_watertight());
    }

    #[test]
    fn fillet_edges_blends_two_independent_box_edges() {
        // Two diagonally-opposite vertical edges of the box do not share a face
        // corner, so the multi-edge API blends them independently and the result
        // stays watertight with one cylindrical blend face per edge.
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let verticals: Vec<Edge> = solid
            .edges()
            .into_iter()
            .filter(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                (p0.x() - p1.x()).abs() < 1e-9
                    && (p0.y() - p1.y()).abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 5.9
            })
            .collect();
        let pick = |x: f64, y: f64| -> Edge {
            verticals
                .iter()
                .find(|e| {
                    let p = e.start().point();
                    (p.x() - x).abs() < 1e-9 && (p.y() - y).abs() < 1e-9
                })
                .expect("vertical edge at corner")
                .clone()
        };
        let edges = [pick(0.0, 0.0), pick(4.0, 5.0)];

        let filleted = fillet_edges(&solid, &edges, 0.4).expect("multi-edge fillet");
        assert!(filleted.is_watertight(), "{:?}", filleted.manifold_report());
        let cylinders = filleted
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 2, "one cylindrical blend per filleted edge");
    }

    fn origin_vertical_edge(solid: &Solid) -> Edge {
        solid
            .edges()
            .into_iter()
            .find(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 5.9
            })
            .expect("box has a vertical origin edge")
    }
}
