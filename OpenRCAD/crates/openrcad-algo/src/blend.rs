//! Shared shape detection and cylinder blend builders for the fillet, chamfer,
//! and shell algorithms.
//!
//! The box paths live in their own modules; this module adds cylinder detection
//! plus the rolling-ball (torus), bevel (cone), and hollow (offset) constructions
//! for a cylinder at any orientation (the frame is recovered from the geometry).
//! Every builder splits each rim into three arcs —
//! mirroring [`openrcad_primitives::make_cylinder`] — so that the endpoint-based
//! edge dedup in [`Solid`] keeps each arc distinct, then sews the faces into a
//! watertight shell.

use core::f64::consts::{FRAC_PI_2, TAU};

use openrcad_foundation::{tolerance, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{
    Circle, ConicalSurface, Curve, CylindricalSurface, GeomCurve, GeomSurface, OffsetSurface,
    Plane, ToroidalSurface,
};
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};

use crate::sew::sew;

/// Why a blend builder could not run.
#[derive(Clone, Debug, PartialEq)]
pub enum BlendError {
    /// The solid is not a shape this builder recognises. A single box or cylinder
    /// primitive is handled at **any position/orientation** (detection recovers
    /// the local frame from the geometry), but arbitrary B-Reps — including
    /// boolean results — are not yet blendable.
    UnsupportedShape,
    /// The requested radius/distance/thickness does not fit the geometry. `max`
    /// is the largest value that would fit (exclusive) — the binding constraint
    /// is the smaller of a face/radius dimension and half the opposite span.
    ParameterTooLarge {
        /// The value the caller asked for.
        requested: f64,
        /// The exclusive upper bound that would fit this geometry.
        max: f64,
    },
}

impl core::fmt::Display for BlendError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BlendError::UnsupportedShape => f.write_str(
                "blend: unsupported shape — only single box or cylinder primitives are handled \
                 (at any position/orientation); arbitrary B-Reps and boolean results are not yet \
                 blendable. Build one with make_box/make_cylinder",
            ),
            BlendError::ParameterTooLarge { requested, max } => write!(
                f,
                "blend: requested {requested} but the geometry only admits values below {max} \
                 (limited by the smaller of the cross dimension and half the opposite span); \
                 retry with a value under {max}",
            ),
        }
    }
}

impl std::error::Error for BlendError {}

/// A cylinder recovered from a [`Solid`], in its own (arbitrary) frame.
pub struct CylinderInfo {
    /// Centre of the base cap, on the axis.
    pub base: Pnt,
    /// Axis direction (base → top).
    pub axis: Dir,
    /// A reference direction orthogonal to the axis.
    pub xref: Dir,
    /// Cylinder radius.
    pub radius: f64,
    /// Cylinder height.
    pub height: f64,
}

/// Recognise the cylinder produced by `make_cylinder`: two planar caps plus a
/// lateral wall of coaxial, equal-radius cylindrical faces.
pub fn detect_cylinder(solid: &Solid) -> Option<CylinderInfo> {
    let faces = solid.shell().faces();
    let mut planes: Vec<Plane> = Vec::new();
    let mut cyl: Option<CylindricalSurface> = None;
    for f in &faces {
        match f.surface() {
            Some(GeomSurface::Plane(p)) => planes.push(*p),
            Some(GeomSurface::Cylinder(c)) => {
                if let Some(prev) = &cyl {
                    // All lateral faces must share one axis and radius.
                    if (prev.radius() - c.radius()).abs() > 1e-6
                        || !prev
                            .position()
                            .direction()
                            .is_parallel(&c.position().direction(), 1e-6)
                    {
                        return None;
                    }
                } else {
                    cyl = Some(*c);
                }
            }
            _ => return None,
        }
    }

    let cyl = cyl?;
    if planes.len() != 2 {
        return None;
    }

    let axis = cyl.position().direction();
    let xref = cyl.position().x_direction();
    let axis_vec = GeomVec::from_dir(axis);

    // Project both cap centres onto the axis; the lower one is the base.
    let c0 = planes[0].position().location();
    let c1 = planes[1].position().location();
    let a0 = (c0 - cyl.position().location()).dot(&axis_vec);
    let a1 = (c1 - cyl.position().location()).dot(&axis_vec);
    let (base, top, height) = if a0 <= a1 {
        (c0, c1, a1 - a0)
    } else {
        (c1, c0, a0 - a1)
    };
    let _ = top;

    if height <= tolerance::CONFUSION || cyl.radius() <= tolerance::CONFUSION {
        return None;
    }

    Some(CylinderInfo {
        base,
        axis,
        xref,
        radius: cyl.radius(),
        height,
    })
}

const THIRDS: [f64; 4] = [0.0, TAU / 3.0, 2.0 * TAU / 3.0, TAU];

/// A circle on the axis frame at `centre` with the given `radius`.
fn ring(centre: Pnt, axis: Dir, xref: Dir, radius: f64) -> Circle {
    Circle::new(Ax3::new_axes(centre, axis, xref), radius)
}

/// The three arc edges of a rim circle, split at thirds.
fn arc_edges(circle: Circle) -> Vec<Edge> {
    let mut edges = Vec::with_capacity(3);
    for w in THIRDS.windows(2) {
        let (u0, u1) = (w[0], w[1]);
        edges.push(Edge::new(
            Some(GeomCurve::circle(circle)),
            u0,
            u1,
            Vertex::new(circle.point(u0)),
            Vertex::new(circle.point(u1)),
        ));
    }
    edges
}

/// A planar cap bounded by `circle`, supported by a plane through `plane_pt`
/// with the given `normal`.
fn cap_face(circle: Circle, plane_pt: Pnt, normal: Dir) -> Face {
    let wire = Wire::from_edges(arc_edges(circle));
    Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(
            plane_pt, normal,
        ))),
        wire,
    )
}

/// A quarter-circle arc of radius `r` about `centre`, from `p1` to `p2`.
fn quarter_arc(centre: Pnt, r: f64, p1: Pnt, p2: Pnt) -> Edge {
    let d1 = (p1 - centre) / r;
    let d2 = (p2 - centre) / r;
    let main = d1.cross(&d2);
    let pos = Ax3::new_axes(
        centre,
        Dir::new(main.x(), main.y(), main.z()),
        Dir::new(d1.x(), d1.y(), d1.z()),
    );
    Edge::new(
        Some(GeomCurve::circle(Circle::new(pos, r))),
        0.0,
        FRAC_PI_2,
        Vertex::new(p1),
        Vertex::new(p2),
    )
}

/// Build the three side faces of a band between a `lower` and `upper` rim,
/// supported by `surface`; `seam(i, lo, hi)` builds the vertical edge joining the
/// matching split points. When `reversed`, each face is flipped (inner shells).
fn band_faces(
    lower: Circle,
    upper: Circle,
    surface: GeomSurface,
    reversed: bool,
    mut seam: impl FnMut(usize, Pnt, Pnt) -> Edge,
) -> Vec<Face> {
    let lo_arcs = arc_edges(lower);
    let hi_arcs = arc_edges(upper);
    let lo_pts: Vec<Pnt> = (0..3).map(|i| lower.point(THIRDS[i])).collect();
    let hi_pts: Vec<Pnt> = (0..3).map(|i| upper.point(THIRDS[i])).collect();
    let seams: Vec<Edge> = (0..3).map(|i| seam(i, lo_pts[i], hi_pts[i])).collect();

    let mut faces = Vec::with_capacity(3);
    for i in 0..3 {
        let next = (i + 1) % 3;
        let wire = Wire::from_edges([
            lo_arcs[i].clone(),
            seams[next].clone(),
            hi_arcs[i].clone().reversed(),
            seams[i].clone().reversed(),
        ]);
        let face = Face::new(Some(surface.clone()), wire);
        faces.push(if reversed { face.reversed() } else { face });
    }
    faces
}

/// Unit radial direction `cos u · X + sin u · Y` for the cylinder frame.
fn radial(axis: Dir, xref: Dir, u: f64) -> GeomVec {
    let x = GeomVec::from_dir(xref);
    let y = GeomVec::from_dir(axis).cross(&GeomVec::from_dir(xref));
    x * u.cos() + y * u.sin()
}

/// Roll a constant-`radius` fillet along both circular rims of a cylinder.
pub fn fillet_cylinder(info: &CylinderInfo, radius: f64) -> Result<Solid, BlendError> {
    let CylinderInfo {
        base,
        axis,
        xref,
        radius: cr,
        height,
    } = *info;
    if radius * 2.0 >= height || radius >= cr {
        return Err(BlendError::ParameterTooLarge {
            requested: radius,
            max: cr.min(height / 2.0),
        });
    }
    let axis_vec = GeomVec::from_dir(axis);
    let top = base + axis_vec * height;
    let inner = cr - radius;

    // Four rims, bottom to top.
    let r0 = ring(base, axis, xref, inner); // bottom cap rim
    let r1 = ring(base + axis_vec * radius, axis, xref, cr); // wall bottom
    let r2 = ring(top - axis_vec * radius, axis, xref, cr); // wall top
    let r3 = ring(top, axis, xref, inner); // top cap rim

    let mut faces = vec![cap_face(r0, base, axis.reversed()), cap_face(r3, top, axis)];

    // Bottom torus fillet: tube centre circle at radius `inner`, height `radius`.
    let bottom_centre = base + axis_vec * radius;
    let torus_b = GeomSurface::torus(ToroidalSurface::new(
        Ax3::new_axes(bottom_centre, axis, xref),
        inner,
        radius,
    ));
    faces.extend(band_faces(r0, r1, torus_b, false, |i, lo, hi| {
        let tube = bottom_centre + radial(axis, xref, THIRDS[i]) * inner;
        quarter_arc(tube, radius, lo, hi)
    }));

    // Lateral wall.
    let wall = GeomSurface::cylinder(CylindricalSurface::new(Ax3::new_axes(base, axis, xref), cr));
    faces.extend(band_faces(r1, r2, wall, false, |_, lo, hi| {
        Edge::between_points(lo, hi)
    }));

    // Top torus fillet.
    let top_centre = top - axis_vec * radius;
    let torus_t = GeomSurface::torus(ToroidalSurface::new(
        Ax3::new_axes(top_centre, axis, xref),
        inner,
        radius,
    ));
    faces.extend(band_faces(r2, r3, torus_t, false, |i, lo, hi| {
        let tube = top_centre + radial(axis, xref, THIRDS[i]) * inner;
        quarter_arc(tube, radius, lo, hi)
    }));

    Ok(Solid::new(sew(&faces, radius * 0.1)))
}

/// Bevel both circular rims of a cylinder by `distance` (a 45° frustum chamfer).
pub fn chamfer_cylinder(info: &CylinderInfo, distance: f64) -> Result<Solid, BlendError> {
    let CylinderInfo {
        base,
        axis,
        xref,
        radius: cr,
        height,
    } = *info;
    if distance * 2.0 >= height || distance >= cr {
        return Err(BlendError::ParameterTooLarge {
            requested: distance,
            max: cr.min(height / 2.0),
        });
    }
    let axis_vec = GeomVec::from_dir(axis);
    let top = base + axis_vec * height;
    let inner = cr - distance;

    let r0 = ring(base, axis, xref, inner);
    let r1 = ring(base + axis_vec * distance, axis, xref, cr);
    let r2 = ring(top - axis_vec * distance, axis, xref, cr);
    let r3 = ring(top, axis, xref, inner);

    let mut faces = vec![cap_face(r0, base, axis.reversed()), cap_face(r3, top, axis)];

    // Bottom frustum: radius grows from `inner` to `cr` over axial `distance`
    // → semi-angle +45°, reference radius `inner` at the base.
    let cone_b = GeomSurface::cone(ConicalSurface::new(
        Ax3::new_axes(base, axis, xref),
        inner,
        FRAC_PI_2 / 2.0,
    ));
    faces.extend(band_faces(r0, r1, cone_b, false, |_, lo, hi| {
        Edge::between_points(lo, hi)
    }));

    let wall = GeomSurface::cylinder(CylindricalSurface::new(Ax3::new_axes(base, axis, xref), cr));
    faces.extend(band_faces(r1, r2, wall, false, |_, lo, hi| {
        Edge::between_points(lo, hi)
    }));

    // Top frustum: radius shrinks from `cr` to `inner` → semi-angle −45°,
    // reference radius `cr` at z = height − distance.
    let cone_t = GeomSurface::cone(ConicalSurface::new(
        Ax3::new_axes(top - axis_vec * distance, axis, xref),
        cr,
        -FRAC_PI_2 / 2.0,
    ));
    faces.extend(band_faces(r2, r3, cone_t, false, |_, lo, hi| {
        Edge::between_points(lo, hi)
    }));

    Ok(Solid::new(sew(&faces, distance * 0.1)))
}

/// Hollow a cylinder to wall `thickness`, leaving any cap in `open_faces` open.
pub fn shell_cylinder(
    info: &CylinderInfo,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    let CylinderInfo {
        base,
        axis,
        xref,
        radius: cr,
        height,
    } = *info;
    if thickness >= cr || thickness * 2.0 >= height {
        return Err(BlendError::ParameterTooLarge {
            requested: thickness,
            max: cr.min(height / 2.0),
        });
    }
    let axis_vec = GeomVec::from_dir(axis);
    let top = base + axis_vec * height;
    let inner_r = cr - thickness;

    // Which caps are removed?
    let mut bottom_open = false;
    let mut top_open = false;
    for f in open_faces {
        if let Some(GeomSurface::Plane(pl)) = f.surface() {
            let a = (pl.position().location() - base).dot(&axis_vec);
            if a.abs() < 1e-4 {
                bottom_open = true;
            } else if (a - height).abs() < 1e-4 {
                top_open = true;
            }
        }
    }

    let floor = if bottom_open {
        base
    } else {
        base + axis_vec * thickness
    };
    let ceil = if top_open {
        top
    } else {
        top - axis_vec * thickness
    };

    let outer = CylindricalSurface::new(Ax3::new_axes(base, axis, xref), cr);
    let inner_surf =
        GeomSurface::offset(OffsetSurface::new(GeomSurface::cylinder(outer), -thickness));
    let straight = |_: usize, lo: Pnt, hi: Pnt| Edge::between_points(lo, hi);

    let mut faces = Vec::new();

    // Outer wall.
    let ro_b = ring(base, axis, xref, cr);
    let ro_t = ring(top, axis, xref, cr);
    faces.extend(band_faces(
        ro_b,
        ro_t,
        GeomSurface::cylinder(outer),
        false,
        straight,
    ));

    // Inner wall (reversed so its material side faces the wall).
    let ri_floor = ring(floor, axis, xref, inner_r);
    let ri_ceil = ring(ceil, axis, xref, inner_r);
    faces.extend(band_faces(
        ri_floor,
        ri_ceil,
        inner_surf.clone(),
        true,
        straight,
    ));

    // Bottom: outer cap + inner cap, or a rim annulus if open.
    if bottom_open {
        let plane = GeomSurface::plane(Plane::from_point_normal(base, axis.reversed()));
        faces.extend(band_faces(ri_floor, ro_b, plane, false, straight));
    } else {
        faces.push(cap_face(ro_b, base, axis.reversed()));
        faces.push(cap_face(ring(floor, axis, xref, inner_r), floor, axis).reversed());
    }

    // Top: outer cap + inner cap, or a rim annulus if open.
    if top_open {
        let plane = GeomSurface::plane(Plane::from_point_normal(top, axis));
        faces.extend(band_faces(ri_ceil, ro_t, plane, false, straight));
    } else {
        faces.push(cap_face(ro_t, top, axis));
        faces.push(cap_face(ring(ceil, axis, xref, inner_r), ceil, axis.reversed()).reversed());
    }

    Ok(Solid::new(sew(&faces, thickness * 0.1)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Ax2;
    use openrcad_primitives::make_cylinder;

    fn unit_cyl() -> Solid {
        make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 6.0)
    }

    #[test]
    fn detects_cylinder() {
        let info = detect_cylinder(&unit_cyl()).expect("should detect a cylinder");
        assert!((info.radius - 2.0).abs() < 1e-9);
        assert!((info.height - 6.0).abs() < 1e-9);
    }

    #[test]
    fn fillet_cylinder_is_watertight_with_tori() {
        let s = fillet_cylinder(&detect_cylinder(&unit_cyl()).unwrap(), 0.5).unwrap();
        let (v, e, f) = (
            s.vertex_count() as i64,
            s.edge_count() as i64,
            s.face_count() as i64,
        );
        assert_eq!(v - e + f, 2, "Euler characteristic must be 2");
        let tori = s
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Torus(_))))
            .count();
        assert_eq!(tori, 6, "two fillet rims, three faces each");
    }

    #[test]
    fn chamfer_cylinder_is_watertight_with_cones() {
        let s = chamfer_cylinder(&detect_cylinder(&unit_cyl()).unwrap(), 0.5).unwrap();
        let (v, e, f) = (
            s.vertex_count() as i64,
            s.edge_count() as i64,
            s.face_count() as i64,
        );
        assert_eq!(v - e + f, 2);
        let cones = s
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Cone(_))))
            .count();
        assert_eq!(cones, 6);
    }

    #[test]
    fn shell_cylinder_open_top_has_offset_wall() {
        let cyl = unit_cyl();
        let top = cyl
            .shell()
            .faces()
            .into_iter()
            .find(|f| matches!(f.surface(), Some(GeomSurface::Plane(p)) if p.normal().z() > 0.9))
            .unwrap();
        let s = shell_cylinder(&detect_cylinder(&cyl).unwrap(), 0.3, &[top]).unwrap();
        let (v, e, f) = (
            s.vertex_count() as i64,
            s.edge_count() as i64,
            s.face_count() as i64,
        );
        assert_eq!(v - e + f, 2);
        let offsets = s
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Offset(_))))
            .count();
        assert_eq!(offsets, 3, "inner wall is three offset faces");
    }

    #[test]
    fn rejects_oversized_parameters() {
        let info = detect_cylinder(&unit_cyl()).unwrap();
        assert!(matches!(
            fillet_cylinder(&info, 5.0),
            Err(BlendError::ParameterTooLarge { .. })
        ));
    }
}
