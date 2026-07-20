use super::*;

/// Axis-aligned box solid, one corner at the origin, opposite at (w, h, d).
pub fn box_solid(w: f32, h: f32, d: f32) -> KernelSolid {
    consume_operation(
        "box primitive",
        openrcad::primitives::make_box_operation(&Pnt::origin(), w as f64, h as f64, d as f64),
    )
    .expect("positive box dimensions must produce a valid solid")
    .solid
}

/// Boolean-ready solid for a cylinder primitive: a **true smooth cylinder**
/// along +Y, base centered at origin.
///
/// OpenRCAD's boolean engine handles smooth cylindrical faces natively — cuts,
/// blind pockets, and (cylinder-as-object) booleans all come back watertight
/// (see the kernel's `repro_cylinder` tests). So the body keeps its smooth
/// analytic wall through a join/cut instead of re-tessellating into the striped
/// 48-gon prism the old "always facet it" rule produced (a workaround for a
/// since-retired truck panic). A 48-gon prism is kept only as a defensive
/// fallback should the native build ever fail.
pub fn cylinder_solid(r: f32, h: f32) -> Option<KernelSolid> {
    build_cylinder_solid(r as f64, h as f64).or_else(|| {
        use crate::geometry::{CoordinateSystem, Vec3};
        let segs = crate::CIRCLE_SEGS;
        let pts: Vec<(f32, f32)> = (0..segs)
            .map(|i| {
                let a = (i as f32 / segs as f32) * std::f32::consts::TAU;
                (r * a.cos(), r * a.sin())
            })
            .collect();
        // Right-handed frame whose normal is +Y (u = Z, v = X ⇒ u × v = +Y),
        // giving the base-at-origin, +Y-axis cylinder the primitive expects.
        let frame = CoordinateSystem::new(Vec3::ZERO, Vec3::Z, Vec3::X);
        build_extrusion_solid(&pts, &[], h as f64, &frame, true)
    })
}

/// Solid for one extruded sketch region. Straight boundary runs sweep to planar
/// laterals; co-circular runs (a drawn circle, a sketch-fillet arc) are rebuilt
/// into true circular-arc edges by [`loop_to_wire`] so they sweep to *smooth*
/// cylindrical walls instead of a fan of facets. Holed profiles try the holed
/// plane first and fall back to the outer boundary alone if the kernel can't
/// attach it.
///
/// OpenRCAD's boolean engine resolves native cylinder cuts/joins/bosses
/// watertight (see the kernel's `repro_cylinder` tests), so feeding it smooth
/// arc walls yields clean round pockets/bosses — not the striped facet result
/// the old "always a prism" rule produced. The parametric assembler still tries
/// the fully-analytic [`circular_cylinder_tool`] first for whole-circle
/// profiles; this is the general fallback for everything else.
pub fn extruded_region_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    extruded_region_solid_with_arcs(points, holes, depth, cs, &[])
}

/// [`extruded_region_solid`] plus the EXACT arc circles the boundary contains (from
/// analytic sketch fillet arcs) so a rounded rectangle / multi-arc profile sweeps
/// to exact cylindrical walls instead of `loop_to_wire`'s faceted refit. Empty
/// `arc_circles` ⇒ identical to `extruded_region_solid`.
pub fn extruded_region_solid_with_arcs(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    arc_circles: &[((f32, f32), f32)],
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return None;
    }
    // The rect-minus-circle heuristic recognises a rectangle with one circular side
    // BITE. A filleted profile has corner arcs, not a bite — and the heuristic
    // mis-fits those arcs into one big circle (≈ the half-diagonal). So when the
    // sketch handed us exact fillet arcs, skip it and build the exact-arc wire.
    if arc_circles.is_empty() {
        if let Some(solid) = rect_minus_circle_region_solid(points, holes, depth, cs) {
            return Some(solid);
        }
    }
    build_extrusion_solid_arcs(points, holes, depth as f64, cs, true, arc_circles)
        .or_else(|| build_extrusion_solid_arcs(points, &[], depth as f64, cs, true, arc_circles))
}

/// Extrude a detected sketch region from its exact arrangement when available.
/// Documents loaded from older payloads and unsupported analytic curve pairs
/// continue through the established sampled/arc-refit compatibility path.
pub fn extruded_sketch_region_solid(
    region: &crate::sketch::Region,
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    arc_circles: &[((f32, f32), f32)],
) -> Option<KernelSolid> {
    region
        .analytic
        .as_ref()
        .and_then(|analytic| build_analytic_extrusion_solid(analytic, f64::from(depth), cs))
        .or_else(|| {
            extruded_region_solid_with_arcs(&region.boundary, &region.holes, depth, cs, arc_circles)
        })
}

/// Find the kernel faces of `solid` geometrically matching a captured face
/// (world centroid + outward normal): plane contains the centroid, normal
/// parallel, boundary near it. Used to resolve GUI face selections (which are
/// mesh-level) to B-Rep faces for kernel ops like shell.
pub fn kernel_faces_matching(
    solid: &KernelSolid,
    centroid: [f32; 3],
    normal: [f32; 3],
) -> Vec<Face> {
    let c = Pnt::new(centroid[0] as f64, centroid[1] as f64, centroid[2] as f64);
    let n = GeomVec::new(normal[0] as f64, normal[1] as f64, normal[2] as f64);
    let mut best: Vec<(f64, Face)> = Vec::new();
    for face in solid.shell().faces() {
        let Some(GeomSurface::Plane(pl)) = face.surface() else {
            continue;
        };
        let mut fnormal = GeomVec::from_dir(pl.normal());
        if face.orientation() == Orientation::Reversed {
            fnormal = -fnormal;
        }
        let align = fnormal.dot(&n) / (fnormal.magnitude() * n.magnitude()).max(1e-12);
        if align < 0.99 {
            continue;
        }
        let anchor = face
            .outer_wire()
            .and_then(|w| w.edges().first().map(|e| e.source().point()));
        let Some(anchor) = anchor else { continue };
        let plane_dist = ((c - anchor).dot(
            &fnormal
                .normalized()
                .map(GeomVec::from_dir)
                .unwrap_or(fnormal),
        ))
        .abs();
        if plane_dist > 0.1 {
            continue;
        }
        // Nearness of the boundary's vertex average to the centroid, as the
        // tie-break between coplanar faces.
        let pts: Vec<Pnt> = face
            .outer_wire()
            .map(|w| w.edges().iter().map(|e| e.source().point()).collect())
            .unwrap_or_default();
        if pts.is_empty() {
            continue;
        }
        let k = pts.len() as f64;
        let avg = Pnt::new(
            pts.iter().map(|p| p.x()).sum::<f64>() / k,
            pts.iter().map(|p| p.y()).sum::<f64>() / k,
            pts.iter().map(|p| p.z()).sum::<f64>() / k,
        );
        best.push((avg.distance(&c), face.clone()));
    }
    best.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    best.into_iter().take(1).map(|(_, f)| f).collect()
}

/// An analytic cylinder solid at an arbitrary position/direction — the drill
/// bit for hole features. `None` on degenerate inputs.
pub fn cylinder_tool_at(
    origin: crate::geometry::Vec3,
    dir: crate::geometry::Vec3,
    radius: f64,
    length: f64,
) -> Option<KernelSolid> {
    if radius <= 0.0 || length <= 0.0 {
        return None;
    }
    let d = dir.normalize();
    if d == crate::geometry::Vec3::ZERO {
        return None;
    }
    let axis = Ax2::new(
        Pnt::new(origin.x as f64, origin.y as f64, origin.z as f64),
        Dir::new(d.x as f64, d.y as f64, d.z as f64),
    );
    consume_operation(
        "cylinder primitive",
        openrcad::primitives::make_cylinder_operation(&axis, radius, length),
    )
    .ok()
    .map(|outcome| outcome.solid)
}

/// A blind-hole cutter with one continuous analytic cylindrical wall and drill
/// point. Building it as a single revolved profile avoids an internal
/// cylinder/cone boolean seam and gives the final cut one watertight tool.
pub fn drill_point_tool_at(
    origin: crate::geometry::Vec3,
    dir: crate::geometry::Vec3,
    radius: f64,
    cylinder_length: f64,
    included_angle_deg: f64,
) -> Option<KernelSolid> {
    if radius <= 0.0
        || cylinder_length <= 0.0
        || !(included_angle_deg > 0.0 && included_angle_deg < 180.0)
    {
        return None;
    }
    let axis = dir.normalize();
    if axis == crate::geometry::Vec3::ZERO {
        return None;
    }
    let seed = if axis.x.abs() < 0.9 {
        crate::geometry::Vec3::X
    } else {
        crate::geometry::Vec3::Y
    };
    let radial = axis.cross(seed).normalize();
    let frame = crate::geometry::CoordinateSystem::new(origin, radial, axis);
    let tip_height = radius / (included_angle_deg * 0.5).to_radians().tan();
    let profile = [
        (0.0, 0.0),
        (radius as f32, 0.0),
        (radius as f32, cylinder_length as f32),
        (0.0, (cylinder_length + tip_height) as f32),
    ];
    revolved_region_solid(
        &profile,
        &[],
        &frame,
        origin,
        axis,
        std::f64::consts::TAU,
        &[],
    )
}

/// An analytic cone-frustum solid (base radius `r1` at `origin`, `r2` after
/// `length` along `dir`) — the countersink cutter. `None` on degenerate inputs.
pub fn cone_tool_at(
    origin: crate::geometry::Vec3,
    dir: crate::geometry::Vec3,
    r1: f64,
    r2: f64,
    length: f64,
) -> Option<KernelSolid> {
    if r1 <= 0.0 || length <= 0.0 {
        return None;
    }
    let d = dir.normalize();
    if d == crate::geometry::Vec3::ZERO {
        return None;
    }
    let axis = Ax2::new(
        Pnt::new(origin.x as f64, origin.y as f64, origin.z as f64),
        Dir::new(d.x as f64, d.y as f64, d.z as f64),
    );
    consume_operation(
        "cone primitive",
        openrcad::primitives::make_cone_operation(&axis, r1, r2, length),
    )
    .ok()
    .map(|outcome| outcome.solid)
}

/// Transform a kernel solid. Rigid motions (translation/rotation) map the
/// B-Rep directly. A REFLECTION flips handedness — every loop then winds
/// backward relative to its transformed surface — so the mirrored faces are
/// re-sewn: `sew`'s winding-consistency BFS plus its global signed-volume
/// outward pass restores a well-oriented shell.
pub fn transformed_solid(solid: &KernelSolid, t: &Trsf, is_reflection: bool) -> KernelSolid {
    transformed_solid_diagnostic(solid, t, is_reflection)
        .expect("a rigid transform of a valid solid must remain valid")
}

/// Policy-validated transform for feature evaluators that must report failure
/// without partially committing a body.
pub(crate) fn transformed_solid_diagnostic(
    solid: &KernelSolid,
    t: &Trsf,
    is_reflection: bool,
) -> Result<KernelSolid, String> {
    consume_operation(
        "body transform",
        openrcad::algo::transform_operation_with_policy(
            solid,
            t,
            is_reflection,
            &TolerancePolicy::STANDARD,
        ),
    )
    .map(|outcome| outcome.solid)
}

/// Resample a closed 2D polygon to exactly `n` points equally spaced by arc
/// length, starting at the original first vertex. Keeps corresponding indices
/// roughly aligned across sections/frames so a skin doesn't shear.
fn resample_ring_2d(pts: &[(f32, f32)], n: usize) -> Vec<(f32, f32)> {
    let m = pts.len();
    if m < 2 || n < 3 {
        return pts.to_vec();
    }
    // Equal vertex counts already provide one-to-one correspondence. Re-spacing
    // a non-square polygon by perimeter quarters moves its corners (a 10x8
    // rectangle becomes four diagonal chords), which can make an otherwise
    // valid drafted frustum fail to sew.
    if m == n {
        return pts.to_vec();
    }
    // Cumulative perimeter length at each original vertex (closed).
    let mut cum = vec![0.0f32; m + 1];
    for i in 0..m {
        let a = pts[i];
        let b = pts[(i + 1) % m];
        cum[i + 1] = cum[i] + ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    }
    let total = cum[m];
    if total <= f32::EPSILON {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(n);
    let mut seg = 0usize;
    for k in 0..n {
        let target = total * k as f32 / n as f32;
        while seg < m && cum[seg + 1] < target {
            seg += 1;
        }
        let s = seg.min(m - 1);
        let seg_len = cum[s + 1] - cum[s];
        let t = if seg_len > f32::EPSILON {
            (target - cum[s]) / seg_len
        } else {
            0.0
        };
        let a = pts[s];
        let b = pts[(s + 1) % m];
        out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
    }
    out
}

/// Loft a solid through complete material sections. Outer boundaries and holes
/// are independently resampled, unprojected to 3D, and handed to the kernel's
/// deterministic [`openrcad::algo::SectionLoops`] correspondence service.
/// `None` when there are fewer than two sections, hole topology changes, hole
/// matching is ambiguous, or the skin fails to close.
pub fn lofted_solid(sections: &[super::LoftSectionProfile]) -> Option<KernelSolid> {
    if sections.len() < 2 {
        return None;
    }
    // Common sampling: the densest section's count, bounded so a coarse
    // triangle doesn't force a fine section down to 3 points and vice-versa.
    let target_n = sections
        .iter()
        .map(|(_, outer, _)| outer.len())
        .max()
        .unwrap_or(0)
        .clamp(3, 256);
    let hole_target_n = sections
        .iter()
        .flat_map(|(_, _, holes)| holes.iter().map(Vec::len))
        .max()
        .map(|count| count.clamp(3, 256));
    let mut kernel_sections = Vec::with_capacity(sections.len());
    for (cs, outer, holes) in sections {
        if outer.len() < 3 || holes.iter().any(|hole| hole.len() < 3) {
            return None;
        }
        let resampled = resample_ring_2d(outer, target_n);
        let outer: Vec<Pnt> = resampled
            .iter()
            .map(|&(u, v)| {
                let p = cs.unproject(u, v);
                Pnt::new(p.x as f64, p.y as f64, p.z as f64)
            })
            .collect();
        let holes = holes
            .iter()
            .map(|hole| {
                let resampled = resample_ring_2d(hole, hole_target_n?);
                Some(
                    resampled
                        .iter()
                        .map(|&(u, v)| {
                            let point = cs.unproject(u, v);
                            Pnt::new(point.x as f64, point.y as f64, point.z as f64)
                        })
                        .collect(),
                )
            })
            .collect::<Option<Vec<_>>>()?;
        kernel_sections.push(openrcad::algo::SectionLoops { outer, holes });
    }
    match consume_operation(
        "loft skin",
        openrcad::algo::skin_section_loops_operation_with_policy(
            &kernel_sections,
            &TolerancePolicy::STANDARD,
        ),
    ) {
        Ok(outcome) => Some(outcome.solid),
        Err(e) => {
            log::warn!("loft failed: {e}");
            None
        }
    }
}

/// Sweep a complete material profile (outer boundary plus holes) along
/// `path_points` (an ordered 3D polyline) using rotation-minimizing frames
/// (double-reflection method), so the profile is transported without twist.
/// The profile is placed perpendicular to the path at its start (its drawn
/// orientation about the tangent is preserved as closely as possible). `None`
/// when the path is too short or the skin fails to close.
pub fn swept_solid(
    profile_cs: &crate::geometry::CoordinateSystem,
    profile_boundary: &[(f32, f32)],
    profile_holes: &[Vec<(f32, f32)>],
    path_points: &[crate::geometry::Vec3],
    total_twist_deg: f32,
    closed: bool,
) -> Option<KernelSolid> {
    use crate::geometry::Vec3;
    // Drop consecutive duplicate path points.
    let mut path: Vec<Vec3> = Vec::with_capacity(path_points.len());
    for &p in path_points {
        if path
            .last()
            .map(|q: &Vec3| q.sub(p).length() > 1e-6)
            .unwrap_or(true)
        {
            path.push(p);
        }
    }
    if path.len() < if closed { 3 } else { 2 }
        || profile_boundary.len() < 3
        || profile_holes.iter().any(|hole| hole.len() < 3)
        || !total_twist_deg.is_finite()
    {
        return None;
    }

    // Tangents by central difference (forward/back at the ends).
    let m = path.len();
    let tangent = |i: usize| -> Vec3 {
        let a = if i == 0 {
            if closed {
                path[m - 1]
            } else {
                path[0]
            }
        } else {
            path[i - 1]
        };
        let b = if i + 1 < m {
            path[i + 1]
        } else if closed {
            path[0]
        } else {
            path[m - 1]
        };
        b.sub(a).normalize()
    };

    // Initial frame perpendicular to t0. Preserve the profile's drawn "right"
    // axis (u) by projecting it into the plane ⊥ t0; fall back to v if u is
    // parallel to the tangent.
    let t0 = tangent(0);
    let mut r = profile_cs.u.sub(t0.mul(profile_cs.u.dot(t0))).normalize();
    if r.length() < 1e-4 {
        r = profile_cs.v.sub(t0.mul(profile_cs.v.dot(t0))).normalize();
    }
    if r.length() < 1e-4 {
        return None;
    }
    let mut s = t0.cross(r).normalize();
    let mut t = t0;

    let profile_point = |o: Vec3, r: Vec3, s: Vec3, uv: (f32, f32)| -> Pnt {
        let w = o.add(r.mul(uv.0)).add(s.mul(uv.1));
        Pnt::new(w.x as f64, w.y as f64, w.z as f64)
    };

    let section_at = |origin: Vec3, right: Vec3, up: Vec3| openrcad::algo::SectionLoops {
        outer: profile_boundary
            .iter()
            .map(|&uv| profile_point(origin, right, up, uv))
            .collect(),
        holes: profile_holes
            .iter()
            .map(|hole| {
                hole.iter()
                    .map(|&uv| profile_point(origin, right, up, uv))
                    .collect()
            })
            .collect(),
    };
    let initial_r = r;
    let initial_s = s;
    let mut frames = Vec::with_capacity(m);
    frames.push((r, s, t));
    for i in 1..m {
        // Double-reflection RMF transport of (r) from frame i-1 to i.
        let v1 = path[i].sub(path[i - 1]);
        let c1 = v1.dot(v1);
        let (r_l, t_l) = if c1 > 1e-12 {
            let r_l = r.sub(v1.mul(2.0 / c1 * v1.dot(r)));
            let t_l = t.sub(v1.mul(2.0 / c1 * v1.dot(t)));
            (r_l, t_l)
        } else {
            (r, t)
        };
        let t_next = tangent(i);
        let v2 = t_next.sub(t_l);
        let c2 = v2.dot(v2);
        let r_next = if c2 > 1e-12 {
            r_l.sub(v2.mul(2.0 / c2 * v2.dot(r_l)))
        } else {
            r_l
        }
        .normalize();
        let s_next = t_next.cross(r_next).normalize();
        r = r_next;
        s = s_next;
        t = t_next;
        frames.push((r, s, t));
    }

    // A closed RMF generally returns with a residual rotation (holonomy).
    // Measure that residual after transporting the final frame over the seam,
    // then distribute the opposite rotation by arc length. The seam therefore
    // closes exactly instead of concentrating a visible twist at one section.
    let transport_right = |from: Vec3, to: Vec3, tangent_from: Vec3, right: Vec3| {
        let chord = to.sub(from);
        let chord_len2 = chord.dot(chord);
        let (right_line, tangent_line) = if chord_len2 > 1.0e-12 {
            (
                right.sub(chord.mul(2.0 / chord_len2 * chord.dot(right))),
                tangent_from.sub(chord.mul(2.0 / chord_len2 * chord.dot(tangent_from))),
            )
        } else {
            (right, tangent_from)
        };
        let tangent_to = tangent(0);
        let normal_delta = tangent_to.sub(tangent_line);
        let normal_len2 = normal_delta.dot(normal_delta);
        if normal_len2 > 1.0e-12 {
            right_line
                .sub(normal_delta.mul(2.0 / normal_len2 * normal_delta.dot(right_line)))
                .normalize()
        } else {
            right_line.normalize()
        }
    };
    let closure_correction = if closed {
        let closure_r = transport_right(path[m - 1], path[0], t, r);
        let axis = tangent(0);
        axis.dot(closure_r.cross(initial_r))
            .atan2(closure_r.dot(initial_r))
    } else {
        0.0
    };
    let twist = (total_twist_deg as f64).to_radians() as f32;
    let rotate_about = |vector: Vec3, axis: Vec3, angle: f32| {
        vector
            .mul(angle.cos())
            .add(axis.cross(vector).mul(angle.sin()))
            .add(axis.mul(axis.dot(vector) * (1.0 - angle.cos())))
            .normalize()
    };
    let mut cumulative = vec![0.0f32; m];
    for index in 1..m {
        cumulative[index] = cumulative[index - 1] + path[index].sub(path[index - 1]).length();
    }
    let total_length = cumulative[m - 1]
        + if closed {
            path[0].sub(path[m - 1]).length()
        } else {
            0.0
        };
    if total_length <= f32::EPSILON {
        return None;
    }
    let mut sections = Vec::with_capacity(m + usize::from(closed));
    for (index, &(frame_r, _frame_s, frame_t)) in frames.iter().enumerate() {
        let fraction = cumulative[index] / total_length;
        let correction = fraction * (closure_correction + twist);
        let right = rotate_about(frame_r, frame_t, correction);
        let up = frame_t.cross(right).normalize();
        sections.push(section_at(path[index], right, up));
    }
    if closed {
        // Exact duplicate, not a separately recomputed frame: ordered skinning
        // connects the last unique path section to this seam and emits no caps.
        sections.push(section_at(path[0], initial_r, initial_s));
    }

    match consume_operation(
        "sweep skin",
        openrcad::algo::skin_ordered_section_loops_operation_with_policy(
            &sections,
            closed,
            &TolerancePolicy::STANDARD,
        ),
    ) {
        Ok(outcome) => Some(outcome.solid),
        Err(e) => {
            log::warn!("sweep failed: {e}");
            None
        }
    }
}

/// Parametric description of a single-start (or multi-start) helical thread,
/// resolved to base units (mm) and geometry-agnostic — the same struct drives a
/// metric (M), Unified (UNC/UNF), or fully custom thread; the presets just fill
/// these fields. See [`helical_thread_solid`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadSpec {
    /// Radius of the helix centerline (the tooth profile's `u = 0`), i.e. the
    /// pitch radius. The crest reaches `mean_radius + depth/2`, the root
    /// `mean_radius - depth/2` (signs flip for `internal`).
    pub mean_radius: f32,
    /// Axial advance per full turn (mm). For a multi-start thread this is the
    /// lead (pitch × starts); the caller sets it accordingly.
    pub pitch: f32,
    /// Axial length of the threaded region (mm).
    pub length: f32,
    /// Radial tooth height, crest to root (mm).
    pub depth: f32,
    /// Half the included thread angle (30° for a 60° metric/Unified V-thread).
    pub half_angle_deg: f32,
    /// Right-handed (advances +axis as it turns CCW) when true, else left.
    pub right_handed: bool,
    /// Tooth points radially inward (a tapped hole) when true, else outward.
    pub internal: bool,
    /// Facets per turn of the helix (sweep density). Higher = smoother, heavier.
    pub segments_per_turn: u32,
    /// Number of thread starts. `pitch` above carries the LEAD (starts × the
    /// per-tooth pitch); the analytic wall repeats the tooth `starts` times
    /// within one lead, so multi-start threads model correctly.
    pub starts: u32,
}

/// Build the helical thread **tooth** as a swept solid — an isoceles V-profile
/// transported along a helix about `axis_dir` through `axis_origin`. This is the
/// reusable rib/cutter the feature layer booleans into a body: UNION it onto a
/// shaft for an external thread, or DIFFERENCE it from a hole wall for an
/// internal one. Returns `None` if the parameters are degenerate or the sweep
/// fails to close.
pub fn helical_thread_solid(
    axis_origin: crate::geometry::Vec3,
    axis_dir: crate::geometry::Vec3,
    spec: &ThreadSpec,
) -> Option<KernelSolid> {
    use crate::geometry::{CoordinateSystem, Vec3};
    use std::f32::consts::TAU;

    if spec.pitch <= 1e-4 || spec.length <= 1e-4 || spec.depth <= 1e-4 || spec.mean_radius <= 1e-4 {
        return None;
    }
    let axis = axis_dir.normalize();
    // A radial basis (e1, e2) perpendicular to the axis.
    let seed = if axis.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let e1 = axis.cross(seed).normalize();
    let e2 = axis.cross(e1).normalize();

    let turns = spec.length / spec.pitch;
    let total_ang = turns * TAU;
    let steps = ((turns * spec.segments_per_turn as f32).ceil() as usize).max(12);
    let sign = if spec.right_handed { 1.0 } else { -1.0 };

    // Helix centerline polyline at the pitch radius.
    let mut path: Vec<Vec3> = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let f = i as f32 / steps as f32;
        let ang = sign * total_ang * f;
        let radial = e1
            .mul(ang.cos())
            .add(e2.mul(ang.sin()))
            .mul(spec.mean_radius);
        let axial = axis.mul(spec.length * f);
        path.push(axis_origin.add(radial).add(axial));
    }

    // V-profile in (u = radial, v = axial): apex at the crest, base at the root.
    let hd = spec.depth * 0.5;
    let half_axial = spec.depth * spec.half_angle_deg.to_radians().tan();
    let dir = if spec.internal { -1.0 } else { 1.0 };
    let profile: Vec<(f32, f32)> = vec![
        (dir * hd, 0.0),          // crest apex
        (-dir * hd, half_axial),  // root, +axial
        (-dir * hd, -half_axial), // root, −axial
    ];
    // Profile frame: u along the (start) radial direction, v along the axis.
    let profile_cs = CoordinateSystem::new(axis_origin, e1, axis);
    swept_solid(&profile_cs, &profile, &[], &path, 0.0, false)
}

/// The analytic helical thread **wall** (no caps): a handful of large
/// ruled-between-helices B-Rep faces — Fusion-style. The thread runs at FULL
/// depth over the whole span and is trimmed flush by the two end planes (like
/// Fusion), so the groove exits straight through the ends and each cap carries
/// the thread's cross-section outline instead of a plain circle. Per profile
/// segment there is ONE face spanning the entire thread length, so the whole
/// crest/flank/root each read, select, and shade as a single smooth spiral
/// band, and the wireframe shows only true helix boundary curves.
pub(crate) struct ThreadWall {
    /// The band faces (the complete lateral wall), one per profile segment.
    pub faces: Vec<Face>,
    /// The thread cross-section outline at `z0`: a closed chain of planar
    /// spiral/arc edges (one per band), shared with the band faces' wires.
    pub bottom_profile: Vec<Edge>,
    /// The cross-section outline at `z1`.
    pub top_profile: Vec<Edge>,
    /// Axis points at the two ends and the unit axis direction.
    pub bottom_center: Pnt,
    pub top_center: Pnt,
    pub axis: Dir,
}

/// The canonical radial frame of a thread wall about `axis`: the unit axis
/// vector plus the (e1, e2) radial basis with e1 at angle 0. Shared by the
/// wall builder and the wall replacement so angles agree.
pub(crate) fn thread_radial_basis(axis: GeomVec) -> Option<(GeomVec, GeomVec, GeomVec)> {
    let av = GeomVec::from_dir(axis.normalized()?);
    let seed = if av.x().abs() < 0.9 {
        GeomVec::new(1.0, 0.0, 0.0)
    } else {
        GeomVec::new(0.0, 1.0, 0.0)
    };
    let e1v = GeomVec::from_dir(av.cross(&seed).normalized()?);
    let e2v = av.cross(&e1v);
    Some((av, e1v, e2v))
}

/// Reverse a wire (each edge reversed, order flipped).
fn wire_reversed(w: &Wire) -> Wire {
    let mut es: Vec<Edge> = w.edges().iter().map(|e| e.reversed()).collect();
    es.reverse();
    Wire::from_edges(es)
}

/// Newell winding normal of a wire's sampled loop (orientation-aware
/// traversal, like the kernel's `loop_agrees_with_surface`).
fn wire_newell(wire: &Wire) -> GeomVec {
    const SAMPLES: usize = 6;
    let mut pts: Vec<Pnt> = Vec::new();
    for edge in wire.edges() {
        let (t0, t1) = (edge.first(), edge.last());
        let reversed = edge.orientation() != Orientation::Forward;
        for s in 0..SAMPLES {
            let f = s as f64 / SAMPLES as f64;
            let f = if reversed { 1.0 - f } else { f };
            let t = t0 + (t1 - t0) * f;
            let p = match edge.curve() {
                Some(c) => c.point(t),
                None => {
                    let a = edge.source().point();
                    let b = edge.target().point();
                    a + (b - a) * (s as f64 / SAMPLES as f64)
                }
            };
            pts.push(p);
        }
    }
    let mut nw = GeomVec::ZERO;
    if pts.len() < 3 {
        return nw;
    }
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        nw += GeomVec::new(
            (p.y() - q.y()) * (p.z() + q.z()),
            (p.z() - q.z()) * (p.x() + q.x()),
            (p.x() - q.x()) * (p.y() + q.y()),
        );
    }
    nw
}

/// Return `wire` wound CCW in the surface's own uv (the revolve/skin lesson:
/// the tessellator winds triangles from orientation ⊗ intrinsic normal), by
/// comparing the loop's Newell normal against du×dv at `center_uv`.
fn wire_ccw_on(wire: Wire, surf: &GeomSurface, center_uv: (f64, f64)) -> Wire {
    let nw = wire_newell(&wire);
    let (_, du, dv) = surf.d1(center_uv.0, center_uv.1);
    if nw.dot(&du.cross(&dv)) >= 0.0 {
        wire
    } else {
        wire_reversed(&wire)
    }
}

/// Order + orient a pool of edges into one head-to-tail closed chain, starting
/// from the first edge's own traversal. Greedy nearest-endpoint matching —
/// exact for the small loops the thread builder assembles.
fn chain_loop(mut pool: Vec<Edge>) -> Vec<Edge> {
    if pool.is_empty() {
        return pool;
    }
    let mut out = vec![pool.remove(0)];
    while !pool.is_empty() {
        let tail = out.last().unwrap().target().point();
        let mut best = 0usize;
        let mut best_rev = false;
        let mut best_d = f64::INFINITY;
        for (i, e) in pool.iter().enumerate() {
            let ds = tail.distance(&e.source().point());
            let dt = tail.distance(&e.target().point());
            if ds < best_d {
                best_d = ds;
                best = i;
                best_rev = false;
            }
            if dt < best_d {
                best_d = dt;
                best = i;
                best_rev = true;
            }
        }
        let e = pool.remove(best);
        out.push(if best_rev { e.reversed() } else { e });
    }
    out
}

/// Build the analytic thread wall between `z0` and `z1` (along the axis from
/// `origin`) on a cylinder of radius `r`.
///
/// Geometry: the tooth profile (crest flat, flank, root flat, flank — ISO-ish
/// 1:2 crest:root flats) is swept helically at FULL depth as exact
/// `Helix`-railed `RuledSurface` faces and trimmed flush by the two end
/// planes — the Fusion look, with the thread cross-section visible on the end
/// faces. Each end-plane trim of a band is a planar Archimedean spiral arc
/// (radius linear in angle), which the tapered lead-0 [`Helix`] represents
/// exactly; the per-band trims chain into one closed cross-section outline per
/// end (returned as `bottom_profile` / `top_profile`). The profile's top
/// boundary retraces its bottom boundary one lead along the SAME canonical
/// helix, so the whole wall is `4·starts` band faces sharing `4·starts` helix
/// rails and `2·4·starts` trim edges: V−E+F closes to a sphere with two caps.
/// The shared analytic frame of a thread build: the canonical radial basis,
/// the tooth profile levels within one lead, and the helix constant
/// `k = lead/2π`. Parameter `u` is the angle from `e1`; along level `lvl`'s
/// rail, `z = zb + levels[lvl].0 + k·u`. Extracted from [`thread_wall_faces`]
/// so the chamfer cut-through builder shares the exact same geometry.
pub(crate) struct ThreadProfile {
    pub origin: Pnt,
    /// Unit axis vector.
    pub av: GeomVec,
    pub axis_d: Dir,
    pub e1: Dir,
    pub e1v: GeomVec,
    pub e2v: GeomVec,
    /// Wall (crest) radius.
    pub r: f64,
    /// Signed axial advance per radian (`lead_signed / 2π`).
    pub k: f64,
    /// Base height: level offsets are measured from here.
    pub zb: f64,
    pub lead: f64,
    pub lead_signed: f64,
    /// Tooth profile levels within one lead: (axial offset, radial offset).
    /// `levels[n]` retraces `levels[0]` one lead along.
    pub levels: Vec<(f64, f64)>,
}

impl ThreadProfile {
    pub(crate) fn new(
        origin: Pnt,
        axis: GeomVec,
        r: f64,
        zb: f64,
        spec: &ThreadSpec,
    ) -> Option<Self> {
        use std::f64::consts::TAU;

        let lead = spec.pitch as f64;
        let depth = (spec.depth as f64).min(r * 0.8);
        if r <= 1e-3 || lead <= 1e-4 || depth <= 1e-4 {
            return None;
        }
        let (av, e1v, e2v) = thread_radial_basis(axis)?;
        let axis_d = av.normalized()?;
        let e1 = e1v.normalized()?;
        let lead_signed = if spec.right_handed { lead } else { -lead };
        let k = lead_signed / TAU;

        let teeth = spec.starts.max(1) as usize;
        let p_tooth = lead / teeth as f64;
        let dr = if spec.internal { depth } else { -depth };
        let mut wf = depth * (spec.half_angle_deg as f64).to_radians().tan();
        if 2.0 * wf > 0.9 * p_tooth {
            wf = 0.45 * p_tooth;
        }
        let rem = p_tooth - 2.0 * wf;
        let wc = rem / 3.0; // crest flat; root flat gets the remaining 2/3 (ISO-ish)
        let mut levels: Vec<(f64, f64)> = Vec::with_capacity(4 * teeth + 1);
        for t in 0..teeth {
            let h0 = t as f64 * p_tooth;
            levels.push((h0, 0.0));
            levels.push((h0 + wc, 0.0));
            levels.push((h0 + wc + wf, dr));
            levels.push((h0 + p_tooth - wf, dr));
        }
        levels.push((lead, 0.0));

        Some(Self {
            origin,
            av,
            axis_d,
            e1,
            e1v,
            e2v,
            r,
            k,
            zb,
            lead,
            lead_signed,
            levels,
        })
    }

    /// Band count (`4·starts`).
    pub(crate) fn n(&self) -> usize {
        self.levels.len() - 1
    }

    pub(crate) fn at(&self, z: f64) -> Pnt {
        self.origin + self.av * z
    }

    pub(crate) fn frame_at(&self, z: f64) -> Ax3 {
        Ax3::new_axes(self.at(z), self.axis_d, self.e1)
    }

    /// The angle at which profile level `lvl` crosses height `z`.
    pub(crate) fn u_at(&self, z: f64, lvl: usize) -> f64 {
        (z - self.zb - self.levels[lvl].0) / self.k
    }

    /// Rail curve of level `lvl`, param-aligned across levels (same angle ⇒
    /// same u).
    pub(crate) fn rail_curve(&self, lvl: usize) -> GeomCurve {
        use openrcad::geom::Helix;
        let (h, d) = self.levels[lvl];
        GeomCurve::helix(Helix::new(
            self.frame_at(self.zb + h),
            self.r + d,
            0.0,
            self.lead_signed,
        ))
    }

    /// End-plane trim curve of band `lvl` at height `zc` (a planar Archimedean
    /// spiral arc = tapered lead-0 helix), plus its `(lo, hi)` u-window.
    pub(crate) fn trim_curve(&self, zc: f64, lvl: usize) -> (GeomCurve, f64, f64) {
        use openrcad::geom::Helix;
        let (_, d0) = self.levels[lvl];
        let (_, d1) = self.levels[lvl + 1];
        let ta = self.u_at(zc, lvl);
        let tb = self.u_at(zc, lvl + 1);
        let (lo, hi) = (ta.min(tb), ta.max(tb));
        let taper = if (tb - ta).abs() > 1e-12 {
            (d1 - d0) / (tb - ta)
        } else {
            0.0
        };
        // Helix stores the radius at u = 0: radius(t) = r0 + taper·t.
        let r0 = self.r + d0 - taper * ta;
        let c = GeomCurve::helix(Helix::new(self.frame_at(zc), r0, taper, 0.0));
        (c, lo, hi)
    }
}

pub(crate) fn thread_wall_faces(
    origin: Pnt,
    axis: GeomVec,
    r: f64,
    z0: f64,
    z1: f64,
    spec: &ThreadSpec,
) -> Option<ThreadWall> {
    use openrcad::geom::RuledSurface;

    let z_span = z1 - z0;
    let lead = spec.pitch as f64;
    if z_span < lead * 1.05 {
        return None;
    }
    let tp = ThreadProfile::new(origin, axis, r, z0, spec)?;
    let n = tp.n(); // band count = 4·starts
    let u_at = |z: f64, lvl: usize| tp.u_at(z, lvl);
    let rail_curve = |lvl: usize| tp.rail_curve(lvl);
    let at = |z: f64| tp.at(z);
    let b_curve = rail_curve(0);

    // Rail edges, each trimmed to z ∈ [z0, z1]. Level n retraces level 0 one
    // lead along, and both trim to the same z window — the SAME edge.
    let b_edge = {
        let a = u_at(z0, 0);
        let b = u_at(z1, 0);
        let (lo, hi) = (a.min(b), a.max(b));
        Edge::new(
            Some(b_curve.clone()),
            lo,
            hi,
            Vertex::new(b_curve.point(lo)),
            Vertex::new(b_curve.point(hi)),
        )
    };
    let mut rail_edge: Vec<Edge> = Vec::with_capacity(n + 1);
    rail_edge.push(b_edge.clone());
    for lvl in 1..n {
        let c = rail_curve(lvl);
        let a = u_at(z0, lvl);
        let b = u_at(z1, lvl);
        let (lo, hi) = (a.min(b), a.max(b));
        rail_edge.push(Edge::new(
            Some(c.clone()),
            lo,
            hi,
            Vertex::new(c.point(lo)),
            Vertex::new(c.point(hi)),
        ));
    }
    rail_edge.push(b_edge);

    // End-plane trim edge of band `lvl` at height `zc`: radius varies linearly
    // with angle between the two levels — a planar spiral arc, i.e. a tapered
    // helix with zero lead lying in the cap plane.
    let trim_edge = |zc: f64, lvl: usize| -> Edge {
        let (c, lo, hi) = tp.trim_curve(zc, lvl);
        Edge::new(
            Some(c.clone()),
            lo,
            hi,
            Vertex::new(c.point(lo)),
            Vertex::new(c.point(hi)),
        )
    };
    let bottom_profile: Vec<Edge> = (0..n).map(|l| trim_edge(z0, l)).collect();
    let top_profile: Vec<Edge> = (0..n).map(|l| trim_edge(z1, l)).collect();

    // Band faces: one per profile segment, spanning the full thread length.
    let z_mid = 0.5 * (z0 + z1);
    let mut faces: Vec<Face> = Vec::with_capacity(n);
    for lvl in 0..n {
        let surf = GeomSurface::ruled(RuledSurface::new(rail_curve(lvl), rail_curve(lvl + 1)));
        let edges = chain_loop(vec![
            rail_edge[lvl].clone(),
            top_profile[lvl].clone(),
            rail_edge[lvl + 1].clone(),
            bottom_profile[lvl].clone(),
        ]);
        let um = 0.5 * (u_at(z_mid, lvl) + u_at(z_mid, lvl + 1));
        let wire = wire_ccw_on(Wire::from_edges(edges), &surf, (um, 0.5));
        faces.push(Face::new(Some(surf), wire));
    }

    Some(ThreadWall {
        faces,
        bottom_profile,
        top_profile,
        bottom_center: at(z0),
        top_center: at(z1),
        axis: tp.axis_d,
    })
}

/// A conical "lathe" radius bound adjacent to one end of a threaded wall — a
/// chamfer cone on a rod rim (or a countersink on a hole mouth). Radius is
/// linear in height: `R_c(z) = ra + rb·z`, riding the wall rim (radius `r` at
/// `z_base`) and ending at the far circle (`r_far` at `z_far`). Because every
/// thread band has both radius and height affine in its `(u, v)` parameters,
/// the band∩cone cut is a straight line in parameter space — i.e. an exact
/// tapered, leaded [`openrcad::geom::Helix`] in 3D. That's what makes a full
/// Fusion-style "thread cuts through the chamfer" analytic here.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ConeBound {
    pub ra: f64,
    pub rb: f64,
    pub z_base: f64,
    pub z_far: f64,
    pub r_far: f64,
}

impl ConeBound {
    fn radius_at(&self, z: f64) -> f64 {
        self.ra + self.rb * z
    }
}

/// How one end of the thread band stack terminates.
pub(crate) enum EndBound {
    /// Trimmed flush by a plane (a flat cap cut-through, or the hand-off plane
    /// to a runout ring).
    Plane(f64),
    /// Cut through an adjacent cone; the band extends to the cone's far plane.
    Cone(ConeBound),
}

impl EndBound {
    fn far_z(&self) -> f64 {
        match self {
            EndBound::Plane(z) => *z,
            EndBound::Cone(cb) => cb.z_far,
        }
    }
}

/// Sutherland–Hodgman clip of a convex polygon by the halfplane
/// `a + b·u + c·v ≥ 0`.
fn clip_halfplane(poly: &[(f64, f64)], a: f64, b: f64, c: f64) -> Vec<(f64, f64)> {
    let m = poly.len();
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(m + 2);
    for i in 0..m {
        let p = poly[i];
        let q = poly[(i + 1) % m];
        let fp = a + b * p.0 + c * p.1;
        let fq = a + b * q.0 + c * q.1;
        if fp >= 0.0 {
            out.push(p);
        }
        if (fp > 0.0 && fq < 0.0) || (fp < 0.0 && fq > 0.0) {
            let t = fp / (fp - fq);
            out.push((p.0 + (q.0 - p.0) * t, p.1 + (q.1 - p.1) * t));
        }
    }
    out
}

/// Split a helix-parameterized edge (param = absolute angle) at the given
/// angles lifted by whole turns into `[lo_u, hi_u]`, merging cuts within
/// ~1e-4 rad of the window ends so no sliver arcs appear.
fn split_edge_at_angles(curve: &GeomCurve, lo_u: f64, hi_u: f64, angles: &[f64]) -> Vec<Edge> {
    use std::f64::consts::TAU;
    let mut cuts = vec![lo_u, hi_u];
    for &s in angles {
        let k0 = ((lo_u - s) / TAU).floor() as i64;
        let k1 = ((hi_u - s) / TAU).ceil() as i64;
        for kk in k0..=k1 {
            let a = s + kk as f64 * TAU;
            if a > lo_u + 1e-4 && a < hi_u - 1e-4 {
                cuts.push(a);
            }
        }
    }
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    cuts.windows(2)
        .map(|w| {
            Edge::new(
                Some(curve.clone()),
                w[0],
                w[1],
                Vertex::new(curve.point(w[0])),
                Vertex::new(curve.point(w[1])),
            )
        })
        .collect()
}

/// Absolute angle of `p` about the thread axis in the shared (e1, e2) basis,
/// in [0, 2π).
fn angle_about(p: Pnt, origin: Pnt, av: &GeomVec, e1v: &GeomVec, e2v: &GeomVec) -> f64 {
    use std::f64::consts::TAU;
    let rel = p - origin;
    let rad = rel - *av * rel.dot(av);
    let a = rad.dot(e2v).atan2(rad.dot(e1v));
    if a < 0.0 {
        a + TAU
    } else {
        a
    }
}

/// Greedy endpoint chaining of an edge pool into one or more CLOSED loops.
/// `None` if any chain fails to close within `tol`.
fn chain_loops(mut pool: Vec<Edge>, tol: f64) -> Option<Vec<Vec<Edge>>> {
    let mut loops: Vec<Vec<Edge>> = Vec::new();
    while !pool.is_empty() {
        let mut cur = vec![pool.remove(0)];
        loop {
            let tail = cur.last().unwrap().target().point();
            let head = cur[0].source().point();
            if tail.distance(&head) <= tol && (cur.len() >= 2 || pool.is_empty()) {
                break;
            }
            let mut found: Option<(usize, bool)> = None;
            for (i, e) in pool.iter().enumerate() {
                if e.source().point().distance(&tail) <= tol {
                    found = Some((i, false));
                    break;
                }
                if e.target().point().distance(&tail) <= tol {
                    found = Some((i, true));
                    break;
                }
            }
            let Some((i, rev)) = found else {
                // A closed single edge (full circle) counts as its own loop.
                if cur.len() == 1 && tail.distance(&head) <= tol {
                    break;
                }
                return None;
            };
            let e = pool.remove(i);
            cur.push(if rev { e.reversed() } else { e });
        }
        loops.push(cur);
    }
    Some(loops)
}

impl ThreadProfile {
    /// Tooth radius at helical phase `h` (axial offset within one lead,
    /// measured like `levels`): piecewise-linear interpolation over the tooth
    /// profile. `h` is wrapped into [0, lead).
    fn profile_radius(&self, h: f64) -> f64 {
        let hh = h.rem_euclid(self.lead);
        let n = self.levels.len();
        for w in 0..n - 1 {
            let (h0, d0) = self.levels[w];
            let (h1, d1) = self.levels[w + 1];
            if hh <= h1 + 1e-12 {
                let f = if h1 - h0 > 1e-12 {
                    (hh - h0) / (h1 - h0)
                } else {
                    0.0
                };
                return self.r + d0 + (d1 - d0) * f;
            }
        }
        self.r + self.levels[n - 1].1
    }

    /// Tooth radius at absolute angle `u` and height `z`.
    fn thread_radius_at(&self, u: f64, z: f64) -> f64 {
        self.profile_radius(z - self.zb - self.k * u)
    }
}

/// The thread band stack clipped by per-end bounds: each band face is the
/// convex region of its `(u, v)` parameter rectangle cut by the end planes and
/// (for a [`EndBound::Cone`] end) the cone halfplane. Every polygon edge maps
/// to an exact analytic 3D edge: rails (leaded helices), plane trims (lead-0
/// tapered helices), and cone cuts (tapered leaded helices).
struct ClippedThreadWall {
    faces: Vec<Face>,
    /// Per band: the trim edge on the LO primary plane (flat-cut profile /
    /// runout hand-off / CASE-B cap piece at a cone end's far plane).
    lo_trims: Vec<Option<Edge>>,
    /// Per band: same for the HI primary plane.
    hi_trims: Vec<Option<Edge>>,
    /// Cone-cut sub-edges (split at cone seam angles) at the LO end.
    lo_cone_edges: Vec<Edge>,
    /// Cone-cut sub-edges at the HI end.
    hi_cone_edges: Vec<Edge>,
}

/// Build the clipped thread bands. `external` flips the material side of the
/// cone bound (an external chamfer removes material OUTSIDE the cone; an
/// internal countersink removes material INSIDE it). `seam_angles` are the
/// adjacent cone segments' seam angles — cone-cut edges are split there so the
/// cone remnant faces can share sub-edge endpoints exactly.
fn thread_bands_clipped(
    tp: &ThreadProfile,
    lo: &EndBound,
    hi: &EndBound,
    external: bool,
    seam_angles: &[f64],
) -> Option<ClippedThreadWall> {
    use openrcad::geom::{Helix, Line, RuledSurface};
    use std::f64::consts::TAU;

    let n = tp.n();
    let sigma = if external { 1.0 } else { -1.0 };
    let (z_lo, z_hi) = (lo.far_z(), hi.far_z());
    if z_hi - z_lo <= 1e-9 {
        return None;
    }
    let scale = (z_hi - z_lo).max(tp.r).max(1.0);

    // Rail edge cache per level (level n wraps to level 0 — same 3D rail).
    let mut rail_cache: Vec<Option<Edge>> = vec![None; n + 1];
    let mut rail_edge_for = |lvl: usize, z_a: f64, z_b: f64| -> Edge {
        let key = if lvl == n { 0 } else { lvl };
        if let Some(e) = &rail_cache[key] {
            return e.clone();
        }
        let c = tp.rail_curve(key);
        let ua = tp.u_at(z_a, key);
        let ub = tp.u_at(z_b, key);
        let (plo, phi) = (ua.min(ub), ua.max(ub));
        let e = Edge::new(
            Some(c.clone()),
            plo,
            phi,
            Vertex::new(c.point(plo)),
            Vertex::new(c.point(phi)),
        );
        rail_cache[key] = Some(e.clone());
        e
    };

    let split_at_seams = |curve: &GeomCurve, lo_u: f64, hi_u: f64| {
        split_edge_at_angles(curve, lo_u, hi_u, seam_angles)
    };

    let mut faces: Vec<Face> = Vec::with_capacity(n);
    let mut lo_trims: Vec<Option<Edge>> = vec![None; n];
    let mut hi_trims: Vec<Option<Edge>> = vec![None; n];
    let mut lo_cone_edges: Vec<Edge> = Vec::new();
    let mut hi_cone_edges: Vec<Edge> = Vec::new();

    for lvl in 0..n {
        let (h0, d0) = tp.levels[lvl];
        let (h1, d1) = tp.levels[lvl + 1];
        let (dh, dd) = (h1 - h0, d1 - d0);
        // z(u,v) = zb + h0 + k·u + dh·v ; R(v) = r + d0 + dd·v.
        let z_at = |u: f64, v: f64| tp.zb + h0 + tp.k * u + dh * v;
        let r_at = |v: f64| tp.r + d0 + dd * v;

        // Constraints as (a, b, c): a + b·u + c·v ≥ 0. Indices:
        // 0: v ≥ 0, 1: v ≤ 1, 2: lo primary plane, 3: hi primary plane,
        // 4: lo cone, 5: hi cone (absent entries get an always-true constraint).
        let mut cons: Vec<(f64, f64, f64)> = vec![
            (0.0, 0.0, 1.0),
            (1.0, 0.0, -1.0),
            (tp.zb + h0 - z_lo, tp.k, dh),
            (z_hi - tp.zb - h0, -tp.k, -dh),
        ];
        let cone_con = |cb: &ConeBound| -> (f64, f64, f64) {
            // σ·(ra + rb·z − R) ≥ 0
            (
                sigma * (cb.ra + cb.rb * (tp.zb + h0) - (tp.r + d0)),
                sigma * (cb.rb * tp.k),
                sigma * (cb.rb * dh - dd),
            )
        };
        if let EndBound::Cone(cb) = lo {
            cons.push(cone_con(cb));
        } else {
            cons.push((1.0, 0.0, 0.0));
        }
        if let EndBound::Cone(cb) = hi {
            cons.push(cone_con(cb));
        } else {
            cons.push((1.0, 0.0, 0.0));
        }

        // Initial box: generous u range from the z window, v ∈ [-0.5, 1.5].
        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        for l in [lvl, lvl + 1] {
            for z in [z_lo - tp.lead, z_hi + tp.lead] {
                let u = tp.u_at(z, l);
                u_min = u_min.min(u);
                u_max = u_max.max(u);
            }
        }
        let mut poly: Vec<(f64, f64)> = vec![
            (u_min - 1.0, -0.5),
            (u_max + 1.0, -0.5),
            (u_max + 1.0, 1.5),
            (u_min - 1.0, 1.5),
        ];
        for &(a, b, c) in &cons {
            poly = clip_halfplane(&poly, a, b, c);
            if poly.len() < 3 {
                break;
            }
        }
        if poly.len() < 3 {
            continue; // band fully cut away (e.g. crest inside the cone zone)
        }
        // Signed area sanity (also gives centroid for winding).
        let mut area2 = 0.0;
        let mut cx = 0.0;
        let mut cv = 0.0;
        for i in 0..poly.len() {
            let p = poly[i];
            let q = poly[(i + 1) % poly.len()];
            let w = p.0 * q.1 - q.0 * p.1;
            area2 += w;
            cx += (p.0 + q.0) * w;
            cv += (p.1 + q.1) * w;
        }
        if area2.abs() < 1e-12 {
            continue;
        }
        let centroid = (cx / (3.0 * area2), cv / (3.0 * area2));

        // Map each polygon edge to a 3D edge by its generating constraint.
        let mut pool: Vec<Edge> = Vec::new();
        let mut ok = true;
        for i in 0..poly.len() {
            let p = poly[i];
            let q = poly[(i + 1) % poly.len()];
            if ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)).sqrt() < 1e-10 {
                continue;
            }
            let mid = (0.5 * (p.0 + q.0), 0.5 * (p.1 + q.1));
            // Which constraint line does this edge ride?
            let mut tag = usize::MAX;
            let mut best = f64::INFINITY;
            for (ci, &(a, b, c)) in cons.iter().enumerate() {
                // Scale the residual so u-direction slopes don't dominate.
                let denom = (b * b + c * c).sqrt().max(1e-12);
                let v = (a + b * mid.0 + c * mid.1).abs() / denom;
                if v < best {
                    best = v;
                    tag = ci;
                }
            }
            if best > 1e-6 * scale.max(1.0) {
                ok = false;
                break;
            }
            match tag {
                0 => pool.push(rail_edge_for(lvl, z_at(p.0, 0.0), z_at(q.0, 0.0))),
                1 => pool.push(rail_edge_for(lvl + 1, z_at(p.0, 1.0), z_at(q.0, 1.0))),
                2 | 3 => {
                    let zc = if tag == 2 { z_lo } else { z_hi };
                    let (c, _, _) = tp.trim_curve(zc, lvl);
                    let (plo, phi) = (p.0.min(q.0), p.0.max(q.0));
                    let e = Edge::new(
                        Some(c.clone()),
                        plo,
                        phi,
                        Vertex::new(c.point(plo)),
                        Vertex::new(c.point(phi)),
                    );
                    if tag == 2 {
                        lo_trims[lvl] = Some(e.clone());
                    } else {
                        hi_trims[lvl] = Some(e.clone());
                    }
                    pool.push(e);
                }
                4 | 5 => {
                    let du = q.0 - p.0;
                    let (za, zbb) = (z_at(p.0, p.1), z_at(q.0, q.1));
                    let (rra, rrb) = (r_at(p.1), r_at(q.1));
                    let subs: Vec<Edge> = if du.abs() < 1e-9 {
                        // Vertical cut in (u,v): a straight ruling segment.
                        let pa = tp.at(za)
                            + (tp.rail_curve(lvl).point(p.0) - tp.at(z_at(p.0, 0.0)))
                                * (rra / (tp.r + d0));
                        let pb = tp.at(zbb)
                            + (tp.rail_curve(lvl).point(q.0) - tp.at(z_at(q.0, 0.0)))
                                * (rrb / (tp.r + d0));
                        match (pb - pa).normalized() {
                            Some(d) => vec![Edge::new(
                                Some(GeomCurve::line(Line::from_point_dir(pa, d))),
                                0.0,
                                pa.distance(&pb),
                                Vertex::new(pa),
                                Vertex::new(pb),
                            )],
                            None => Vec::new(),
                        }
                    } else {
                        // Tapered, leaded helix through the two endpoints.
                        let taper = (rrb - rra) / du;
                        let lead_c = TAU * (zbb - za) / du;
                        let r0 = rra - taper * p.0;
                        let z0 = za - (zbb - za) / du * p.0;
                        let c = GeomCurve::helix(Helix::new(tp.frame_at(z0), r0, taper, lead_c));
                        let (plo, phi) = (p.0.min(q.0), p.0.max(q.0));
                        split_at_seams(&c, plo, phi)
                    };
                    if tag == 4 {
                        lo_cone_edges.extend(subs.iter().cloned());
                    } else {
                        hi_cone_edges.extend(subs.iter().cloned());
                    }
                    pool.extend(subs);
                }
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            return None;
        }

        let surf = GeomSurface::ruled(RuledSurface::new(
            tp.rail_curve(lvl),
            tp.rail_curve(lvl + 1),
        ));
        let wire = wire_ccw_on(Wire::from_edges(chain_loop(pool)), &surf, centroid);
        faces.push(Face::new(Some(surf), wire));
    }

    Some(ClippedThreadWall {
        faces,
        lo_trims,
        hi_trims,
        lo_cone_edges,
        hi_cone_edges,
    })
}

/// A detected chamfer cone (or countersink) riding one end of the threaded
/// wall: which `keep` faces make it up (they get consumed and rebuilt with the
/// thread channel cut through them), the analytic bound, and the segments'
/// slant seam angles.
pub(crate) struct ConeNeighbor {
    pub face_idx: Vec<usize>,
    pub bound: ConeBound,
    pub seam_angles: Vec<f64>,
}

/// Find a coaxial cone adjacent to the wall extremity at `z_ext` (base circle
/// = the wall rim), narrowing INTO the material (`dir` = +1 for the wall's
/// z-max end). Returns `None` when the neighbors aren't a clean single-cone
/// ring — the caller falls back to the runout+collar construction.
#[allow(clippy::too_many_arguments)]
fn detect_cone_neighbor(
    keep: &[Face],
    origin: Pnt,
    av: GeomVec,
    e1v: GeomVec,
    e2v: GeomVec,
    r: f64,
    z_ext: f64,
    dir: f64,
    external: bool,
    z_eps: f64,
    r_eps: f64,
) -> Option<ConeNeighbor> {
    let z_of = |p: Pnt| (p - origin).dot(&av);
    let rad_of = |p: Pnt| {
        let rel = p - origin;
        (rel - av * rel.dot(&av)).magnitude()
    };

    let mut face_idx: Vec<usize> = Vec::new();
    let mut far: Vec<(f64, f64)> = Vec::new();
    let mut seam_angles: Vec<f64> = Vec::new();
    for (fi, face) in keep.iter().enumerate() {
        let Some(GeomSurface::Cone(c)) = face.surface() else {
            continue;
        };
        let pos = c.position();
        let d = GeomVec::from_dir(pos.direction());
        if d.dot(&av).abs() < 1.0 - 1e-4 {
            continue;
        }
        let rel = pos.location() - origin;
        let along = rel.dot(&av);
        if (rel - av * along).magnitude() > 1e-3 * r.max(1.0) {
            continue;
        }
        // Must ride the wall rim: some boundary vertex at (r, z_ext).
        let mut touches = false;
        let mut verts: Vec<Pnt> = Vec::new();
        for wire in &face.wires() {
            for e in wire.edges().iter() {
                let (p0, p1) = (e.start().point(), e.end().point());
                verts.push(p0);
                verts.push(p1);
                if (z_of(p0) - z_ext).abs() > z_eps || (z_of(p1) - z_ext).abs() > z_eps {
                    // A slant seam (or the far circle) — record seam angles for
                    // edges that span height.
                    if ((z_of(p0) - z_of(p1)).abs() > z_eps)
                        && ((rad_of(p0) - rad_of(p1)).abs() > r_eps)
                    {
                        let a = angle_about(p0, origin, &av, &e1v, &e2v);
                        let gap = |s: f64| {
                            let d = (s - a).abs() % std::f64::consts::TAU;
                            d.min(std::f64::consts::TAU - d)
                        };
                        if seam_angles.iter().all(|&s| gap(s) > 1e-6) {
                            seam_angles.push(a);
                        }
                    }
                }
                for p in [p0, p1] {
                    if (z_of(p) - z_ext).abs() <= z_eps && (rad_of(p) - r).abs() <= r_eps {
                        touches = true;
                    }
                }
            }
        }
        if !touches {
            continue;
        }
        face_idx.push(fi);
        for p in verts {
            let (z, rad) = (z_of(p), rad_of(p));
            if (z - z_ext).abs() <= z_eps && (rad - r).abs() <= r_eps {
                continue; // base ring
            }
            far.push((z, rad));
        }
    }
    if face_idx.is_empty() || far.is_empty() {
        return None;
    }
    // Every non-base vertex must sit on ONE far circle beyond z_ext.
    let (z_far, r_far) = far[0];
    for &(z, rad) in &far {
        if (z - z_far).abs() > z_eps * 10.0 || (rad - r_far).abs() > r_eps * 10.0 {
            return None;
        }
    }
    if dir * (z_far - z_ext) <= z_eps * 10.0 {
        return None;
    }
    let rb = (r_far - r) / (z_far - z_ext);
    if rb.abs() < 1e-6 {
        return None;
    }
    let sigma = if external { 1.0 } else { -1.0 };
    if sigma * rb * dir >= 0.0 {
        return None; // opens away from the material — not a rim chamfer
    }
    Some(ConeNeighbor {
        face_idx,
        bound: ConeBound {
            ra: r - rb * z_ext,
            rb,
            z_base: z_ext,
            z_far,
            r_far,
        },
        seam_angles,
    })
}

/// Largest-gap angular window (center, half-width) of a face's boundary
/// samples about the thread axis.
fn face_angular_window(
    face: &Face,
    origin: Pnt,
    av: &GeomVec,
    e1v: &GeomVec,
    e2v: &GeomVec,
) -> Option<(f64, f64)> {
    use std::f64::consts::TAU;
    let mut angs: Vec<f64> = Vec::new();
    for wire in &face.wires() {
        for e in wire.edges().iter() {
            let (t0, t1) = (e.first(), e.last());
            let (p0, p1) = (e.start().point(), e.end().point());
            let c = e.curve().cloned();
            for i in 0..=16 {
                let f = i as f64 / 16.0;
                let p = match &c {
                    Some(c) => c.point(t0 + (t1 - t0) * f),
                    None => p0 + (p1 - p0) * f,
                };
                angs.push(angle_about(p, origin, av, e1v, e2v));
            }
        }
    }
    if angs.is_empty() {
        return None;
    }
    angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let m = angs.len();
    let mut gap_i = 0usize;
    let mut gap = 0.0f64;
    for i in 0..m {
        let d = if i + 1 < m {
            angs[i + 1] - angs[i]
        } else {
            angs[0] + TAU - angs[m - 1]
        };
        if d > gap {
            gap = d;
            gap_i = i;
        }
    }
    let lo = angs[(gap_i + 1) % m];
    let end = if gap_i + 1 < m {
        angs[gap_i] + TAU
    } else {
        angs[gap_i]
    };
    Some((0.5 * (lo + end), 0.5 * (end - lo)))
}

/// Shortest angular distance between two angles.
fn ang_gap(a: f64, b: f64) -> f64 {
    use std::f64::consts::TAU;
    let d = (a - b).abs() % TAU;
    d.min(TAU - d)
}

/// Rebuild the chamfer cone's segment faces with the thread channel cut
/// through them: the far circle stays (verbatim in CASE A, the shared far
/// arcs in CASE B), the base circle is replaced by the crest cut arcs, slant
/// seams are retrimmed to the intervals where cone material survives, and the
/// channel walls are the bands' cone-cut sub-edges (the SAME objects, so sew
/// pairs band↔cone exactly). One segment may come back as several faces when
/// the channel severs it.
#[allow(clippy::too_many_arguments)]
fn cone_remnant_faces(
    tp: &ThreadProfile,
    cb: &ConeBound,
    segs: &[&Face],
    cut_edges: &[Edge],
    far_arcs: &[Edge],
    case_b: bool,
    external: bool,
    z_eps: f64,
    r_eps: f64,
) -> Option<Vec<Face>> {
    let origin = tp.origin;
    let av = tp.av;
    let sigma = if external { 1.0 } else { -1.0 };
    let z_of = |p: Pnt| (p - origin).dot(&av);
    let rad_of = |p: Pnt| {
        let rel = p - origin;
        (rel - av * rel.dot(&av)).magnitude()
    };
    // Outward direction of the cone surface at p (gradient of R − R_c, flipped
    // for internal material).
    let outward = |p: Pnt| -> GeomVec {
        let rel = p - origin;
        let z = rel.dot(&av);
        let rad = rel - av * z;
        let rho = rad * (1.0 / rad.magnitude().max(1e-12));
        (rho - av * cb.rb) * sigma
    };
    let edge_sample = |e: &Edge| -> Pnt {
        match e.curve() {
            Some(c) => c.point(0.5 * (e.first() + e.last())),
            None => {
                let (a, b) = (e.source().point(), e.target().point());
                a + (b - a) * 0.5
            }
        }
    };
    // z-window of the cone band.
    let (z_lo_c, z_hi_c) = (cb.z_base.min(cb.z_far), cb.z_base.max(cb.z_far));

    // Angles of all cut-edge endpoints, for locating seam crossings.
    let cut_end_data: Vec<(f64, f64)> = cut_edges
        .iter()
        .flat_map(|e| {
            [e.start().point(), e.end().point()]
                .into_iter()
                .map(|p| (angle_about(p, origin, &av, &tp.e1v, &tp.e2v), z_of(p)))
        })
        .collect();

    let mut out: Vec<Face> = Vec::new();
    for seg in segs {
        if !seg.inner_wires().is_empty() {
            return None;
        }
        let (center, half) = face_angular_window(seg, origin, &av, &tp.e1v, &tp.e2v)?;
        let outer = seg.outer_wire()?;
        let mut pool: Vec<Edge> = Vec::new();
        for e in outer.edges().iter() {
            let (p0, p1) = (e.start().point(), e.end().point());
            let (z0, z1) = (z_of(p0), z_of(p1));
            let far0 = (z0 - cb.z_far).abs() <= z_eps * 10.0
                && (rad_of(p0) - cb.r_far).abs() <= r_eps * 10.0;
            let far1 = (z1 - cb.z_far).abs() <= z_eps * 10.0
                && (rad_of(p1) - cb.r_far).abs() <= r_eps * 10.0;
            let base0 = (z0 - cb.z_base).abs() <= z_eps && (rad_of(p0) - tp.r).abs() <= r_eps;
            let base1 = (z1 - cb.z_base).abs() <= z_eps && (rad_of(p1) - tp.r).abs() <= r_eps;
            if far0 && far1 {
                if !case_b {
                    pool.push(e.clone()); // far circle kept verbatim
                }
                // CASE B: replaced by the shared far arcs below.
            } else if base0 && base1 {
                // Base circle: replaced by the crest cut arcs (in cut_edges).
            } else if (base0 && far1) || (far0 && base1) {
                // Slant seam: retrim to the intervals where cone material
                // survives the channel.
                let theta = angle_about(p0, origin, &av, &tp.e1v, &tp.e2v);
                let mut zs: Vec<f64> = vec![z_lo_c, z_hi_c];
                for &(a, z) in &cut_end_data {
                    if ang_gap(a, theta) < 1e-4 && z > z_lo_c + z_eps && z < z_hi_c - z_eps {
                        zs.push(z);
                    }
                }
                zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                zs.dedup_by(|a, b| (*a - *b).abs() < z_eps);
                let (t0, t1) = (e.first(), e.last());
                for w in zs.windows(2) {
                    let (za, zb2) = (w[0], w[1]);
                    if zb2 - za < z_eps * 10.0 {
                        continue;
                    }
                    let zm = 0.5 * (za + zb2);
                    // Cone survives where the tooth is at/above the cone.
                    if sigma * (tp.thread_radius_at(theta, zm) - cb.radius_at(zm)) < 0.0 {
                        continue;
                    }
                    // z is affine in the seam's parameter.
                    let tt = |z: f64| t0 + (t1 - t0) * (z - z0) / (z1 - z0);
                    let (ta, tb) = (tt(za), tt(zb2));
                    let c = e.curve()?.clone();
                    let (plo, phi) = (ta.min(tb), ta.max(tb));
                    let (pa, pb) = (c.point(plo), c.point(phi));
                    for p in [pa, pb] {
                        let zz = z_of(p);
                        if (rad_of(p) - cb.radius_at(zz)).abs() > r_eps * 10.0 {
                            return None; // seam isn't z-affine — bail honestly
                        }
                    }
                    pool.push(Edge::new(
                        Some(c),
                        plo,
                        phi,
                        Vertex::new(pa),
                        Vertex::new(pb),
                    ));
                }
            } else {
                return None;
            }
        }
        // Channel walls + crest/root arcs inside this segment's window.
        for e in cut_edges {
            let a = angle_about(edge_sample(e), origin, &av, &tp.e1v, &tp.e2v);
            if ang_gap(a, center) <= half + 1e-6 {
                pool.push(e.clone());
            }
        }
        if case_b {
            for e in far_arcs {
                let a = angle_about(edge_sample(e), origin, &av, &tp.e1v, &tp.e2v);
                if ang_gap(a, center) <= half + 1e-6 {
                    pool.push(e.clone());
                }
            }
        }
        if pool.is_empty() {
            continue;
        }
        let tol = (z_eps * 10.0).max(1e-6);
        let loops = chain_loops(pool, tol)?;
        // Winding: match the ORIGINAL segment's Newell-vs-outward sense.
        let s0 = {
            let n0 = wire_newell(&outer);
            let sample = outer
                .edges()
                .first()
                .map(edge_sample)
                .unwrap_or_else(|| tp.at(cb.z_base));
            n0.dot(&outward(sample))
        };
        for lp in loops {
            let sample = edge_sample(&lp[0]);
            let wire = Wire::from_edges(lp);
            let s1 = wire_newell(&wire).dot(&outward(sample));
            let wire = if s0 * s1 < 0.0 {
                wire_reversed(&wire)
            } else {
                wire
            };
            out.push(Face::with_wires(
                seg.surface().cloned(),
                Some(wire),
                Vec::new(),
                seg.orientation(),
            ));
        }
    }
    Some(out)
}

/// CASE B (the groove punches through to the cone's far plane): the arcs of
/// the far circle where the tooth still covers it, split at the cone seam
/// angles. Returns `(sub_arcs, cap_pool)` where `cap_pool` = the far-plane
/// band trims + the sub-arcs — the replacement rim outline for the cap face
/// that used to ride the plain far circle.
fn far_plane_arcs(
    tp: &ThreadProfile,
    cb: &ConeBound,
    trims: &[Option<Edge>],
    seam_angles: &[f64],
) -> Option<(Vec<Edge>, Vec<Edge>)> {
    use openrcad::geom::Helix;
    use std::f64::consts::TAU;

    let n = tp.n();
    let frame = tp.frame_at(cb.z_far);
    let circle = GeomCurve::helix(Helix::new(frame, cb.r_far, 0.0, 0.0));
    let descending = tp.k > 0.0;

    // Walk the profile chain at the far plane; gaps between present trims are
    // circle arcs.
    let mut arcs_raw: Vec<(f64, f64)> = Vec::new();
    let chain_start = tp.u_at(cb.z_far, 0);
    let mut cursor = chain_start;
    for (lvl, trim) in trims.iter().enumerate().take(n) {
        // The band's full window at the far plane, in chain order.
        let a = tp.u_at(cb.z_far, lvl);
        let b = tp.u_at(cb.z_far, lvl + 1);
        let _ = a;
        if let Some(e) = trim {
            let (plo, phi) = (e.first(), e.last());
            let (near, far_p) = if descending { (phi, plo) } else { (plo, phi) };
            if (cursor - near).abs() > 1e-9 {
                arcs_raw.push((near.min(cursor), near.max(cursor)));
            }
            cursor = far_p;
        }
        let _ = b;
    }
    let chain_end = tp.u_at(cb.z_far, n);
    if (cursor - chain_end).abs() > 1e-9 {
        arcs_raw.push((cursor.min(chain_end), cursor.max(chain_end)));
    }
    // Total coverage sanity: trims + arcs must tile one full turn.
    let covered: f64 = arcs_raw.iter().map(|(a, b)| b - a).sum::<f64>()
        + trims
            .iter()
            .flatten()
            .map(|e| e.last() - e.first())
            .sum::<f64>();
    if (covered - TAU).abs() > 1e-3 {
        return None;
    }

    let mut sub_arcs: Vec<Edge> = Vec::new();
    for (lo, hi) in arcs_raw {
        sub_arcs.extend(split_edge_at_angles(&circle, lo, hi, seam_angles));
    }
    let mut cap_pool: Vec<Edge> = trims.iter().flatten().cloned().collect();
    cap_pool.extend(sub_arcs.iter().cloned());
    Some((sub_arcs, cap_pool))
}

/// The runout ring that fades the thread's cross-section outline at one end of
/// the thread band into a plain circle of radius `r` at `z_circle`: one ruled
/// face per profile edge, lofting the (lead-0 tapered `Helix`) spiral arc — in
/// its own end plane — straight to the circle. Both rails are parameterized by
/// the SAME angle-from-e1 `u`, so rulings connect same-angle points and the
/// tooth section fades conically into the shank, like a real cut thread's
/// runout. The circle boundary is additionally split at `seam_angles` (the old
/// wall's axial seam angles, absolute radians) so the collar faces can share
/// arc endpoints exactly. Returns the runout faces plus the circle arcs (the
/// collar's boundary at `z_circle`).
fn thread_runout_faces(
    origin: Pnt,
    av: GeomVec,
    e1v: GeomVec,
    r: f64,
    profile: &[Edge],
    z_circle: f64,
    seam_angles: &[f64],
) -> Option<(Vec<Face>, Vec<Edge>)> {
    use openrcad::geom::{Helix, Line, RuledSurface};
    use std::f64::consts::TAU;

    let axis_d = av.normalized()?;
    let e1 = e1v.normalized()?;
    let frame = Ax3::new_axes(origin + av * z_circle, axis_d, e1);
    let circle = GeomCurve::helix(Helix::new(frame, r, 0.0, 0.0));

    // One straight side ruling per band boundary, cached by the profile-side
    // endpoint so angularly adjacent faces (and the wrap-around band, whose
    // window sits one full turn along) share the SAME edge object.
    let mut ruling_cache: Vec<(Pnt, Edge)> = Vec::new();
    let mut ruling_for = |a: Pnt, u: f64| -> Option<Edge> {
        for (p, e) in &ruling_cache {
            if p.distance(&a) < 1e-9 {
                return Some(e.clone());
            }
        }
        let b = circle.point(u);
        let d = (b - a).normalized()?;
        let len = a.distance(&b);
        let edge = Edge::new(
            Some(GeomCurve::line(Line::from_point_dir(a, d))),
            0.0,
            len,
            Vertex::new(a),
            Vertex::new(b),
        );
        ruling_cache.push((a, edge.clone()));
        Some(edge)
    };

    let mut faces: Vec<Face> = Vec::with_capacity(profile.len());
    let mut all_arcs: Vec<Edge> = Vec::new();
    for e in profile {
        let c_prof = e.curve()?.clone();
        let (lo, hi) = (e.first(), e.last());

        // Circle cut angles: the window ends plus every seam angle lifted by
        // whole turns into the window (merged when within ~1e-4 rad of an end,
        // so no sliver arcs).
        let mut cuts: Vec<f64> = vec![lo, hi];
        for &s in seam_angles {
            let k0 = ((lo - s) / TAU).floor() as i64;
            let k1 = ((hi - s) / TAU).ceil() as i64;
            for k in k0..=k1 {
                let a = s + k as f64 * TAU;
                if a > lo + 1e-4 && a < hi - 1e-4 {
                    cuts.push(a);
                }
            }
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        let mut arcs: Vec<Edge> = Vec::with_capacity(cuts.len() - 1);
        for w in cuts.windows(2) {
            arcs.push(Edge::new(
                Some(circle.clone()),
                w[0],
                w[1],
                Vertex::new(circle.point(w[0])),
                Vertex::new(circle.point(w[1])),
            ));
        }

        // The face: profile edge (the SAME object the thread band uses, so sew
        // pairs them exactly) + side rulings + the circle sub-arcs.
        let surf = GeomSurface::ruled(RuledSurface::new(c_prof.clone(), circle.clone()));
        let r_lo = ruling_for(c_prof.point(lo), lo)?;
        let r_hi = ruling_for(c_prof.point(hi), hi)?;
        let mut pool = vec![e.clone(), r_lo, r_hi];
        pool.extend(arcs.iter().cloned());
        let um = 0.5 * (lo + hi);
        let wire = wire_ccw_on(Wire::from_edges(chain_loop(pool)), &surf, (um, 0.5));
        faces.push(Face::new(Some(surf), wire));
        all_arcs.extend(arcs);
    }
    Some((faces, all_arcs))
}

/// Rebuild the old cylindrical wall faces as the plain COLLAR between the
/// runout circle at `z_cut` and the wall extremity at `z_keep`: extremity-side
/// boundary edges are kept **verbatim** (so whatever neighbors them — a flat
/// cap rim, a chamfer cone, a fillet torus — resews unchanged), axial seam
/// edges are retrimmed to `[z_cut, z_keep]` with their extremity endpoints
/// preserved, and the far boundary is replaced by the runout's circle arcs
/// (the SAME edge objects, so sew pairs collar↔runout exactly). Returns `None`
/// for any wall shape it can't classify — interrupted or oddly-bounded walls
/// keep the honest cosmetic fallback.
#[allow(clippy::too_many_arguments)]
fn collar_faces(
    wall_old: &[Face],
    origin: Pnt,
    av: GeomVec,
    e1v: GeomVec,
    e2v: GeomVec,
    r: f64,
    z_keep: f64,
    z_far: f64,
    z_cut: f64,
    circle_arcs: &[Edge],
    z_eps: f64,
    r_eps: f64,
) -> Option<Vec<Face>> {
    use openrcad::geom::Line;
    use std::f64::consts::{PI, TAU};

    let z_of = |p: Pnt| (p - origin).dot(&av);
    // Angle of a point about the axis, in the shared (e1, e2) basis.
    let angle_of = |p: Pnt| -> f64 {
        let rel = p - origin;
        let radial = rel - av * rel.dot(&av);
        radial.dot(&e2v).atan2(radial.dot(&e1v))
    };
    // Shortest signed angular difference a − b, wrapped to (−π, π].
    let ang_diff = |a: f64, b: f64| -> f64 {
        let mut d = (a - b) % TAU;
        if d <= -PI {
            d += TAU;
        } else if d > PI {
            d -= TAU;
        }
        d
    };

    // Classify each face's boundary: kept extremity edges (verbatim, in wire
    // order so the rebuilt loop preserves the old winding), axial seams
    // (retrimmed), far-extremity edges (dropped — the circle replaces them).
    let n = wall_old.len();
    let mut keeps: Vec<Vec<Edge>> = vec![Vec::new(); n];
    let mut seams: Vec<Vec<Edge>> = vec![Vec::new(); n];
    for (fi, face) in wall_old.iter().enumerate() {
        if !face.inner_wires().is_empty() {
            return None;
        }
        let outer = face.outer_wire()?;
        for e in outer.edges().iter() {
            let (z0, z1) = (z_of(e.start().point()), z_of(e.end().point()));
            let keep0 = (z0 - z_keep).abs() <= z_eps;
            let keep1 = (z1 - z_keep).abs() <= z_eps;
            let far0 = (z0 - z_far).abs() <= z_eps;
            let far1 = (z1 - z_far).abs() <= z_eps;
            if keep0 && keep1 {
                keeps[fi].push(e.clone());
            } else if far0 && far1 {
                // replaced by the runout circle
            } else if (keep0 && far1) || (far0 && keep1) {
                seams[fi].push(e.clone());
            } else {
                return None;
            }
        }
        if keeps[fi].is_empty() {
            return None;
        }
        // A periodic single-face wall carries its seam TWICE with coincident
        // endpoints; the greedy loop assembly below can't disambiguate — bail.
        for i in 0..seams[fi].len() {
            for j in i + 1..seams[fi].len() {
                let (a, b) = (&seams[fi][i], &seams[fi][j]);
                let same = a.start().point().distance(&b.start().point()) < z_eps
                    && a.end().point().distance(&b.end().point()) < z_eps;
                let swapped = a.start().point().distance(&b.end().point()) < z_eps
                    && a.end().point().distance(&b.start().point()) < z_eps;
                if same || swapped {
                    return None;
                }
            }
        }
    }

    // Retrim a spanning seam edge to [z_cut, z_keep]. The cut parameter comes
    // from z being affine in the raw parameter (true for straight axial seams);
    // the landed point is verified on the wall circle at z_cut, else we bail.
    let retrim = |e: &Edge| -> Option<Edge> {
        let (t0, t1) = (e.first(), e.last());
        let (p0, p1) = (e.start().point(), e.end().point());
        let (z0, z1) = (z_of(p0), z_of(p1));
        if (z1 - z0).abs() <= z_eps {
            return None;
        }
        let f = (z_cut - z0) / (z1 - z0);
        let keep_first = (z0 - z_keep).abs() <= z_eps;
        match e.curve() {
            Some(c) => {
                let c = c.clone();
                let t_cut = t0 + (t1 - t0) * f;
                let pc = c.point(t_cut);
                let rel = pc - origin;
                let zc = rel.dot(&av);
                let rad = (rel - av * zc).magnitude();
                if (zc - z_cut).abs() > z_eps || (rad - r).abs() > r_eps {
                    return None;
                }
                let keep_t = if keep_first { t0 } else { t1 };
                let (lo, hi) = (t_cut.min(keep_t), t_cut.max(keep_t));
                Some(Edge::new(
                    Some(c.clone()),
                    lo,
                    hi,
                    Vertex::new(c.point(lo)),
                    Vertex::new(c.point(hi)),
                ))
            }
            None => {
                // Curveless straight seam: rebuild the kept segment as a line.
                let pc = p0 + (p1 - p0) * f;
                let pk = if keep_first { p0 } else { p1 };
                let d = (pk - pc).normalized()?;
                Some(Edge::new(
                    Some(GeomCurve::line(Line::from_point_dir(pc, d))),
                    0.0,
                    pc.distance(&pk),
                    Vertex::new(pc),
                    Vertex::new(pk),
                ))
            }
        }
    };

    // Each face owns a contiguous angular window defined by its kept extremity
    // arc. Represent it as (center angle, half-width) from that arc's sample
    // angles (largest-gap complement, robust for a 120° arc). A circle arc is
    // assigned to the face whose window contains its midpoint angle — exact,
    // since the runout split the circle at the seam angles so no arc straddles
    // a window boundary.
    let windows: Vec<(f64, f64)> = keeps
        .iter()
        .map(|es| {
            let mut angs: Vec<f64> = Vec::new();
            for e in es {
                let (t0, t1) = (e.first(), e.last());
                let (p0, p1) = (e.start().point(), e.end().point());
                let c = e.curve().cloned();
                for i in 0..=16 {
                    let f = i as f64 / 16.0;
                    let p = match &c {
                        Some(c) => c.point(t0 + (t1 - t0) * f),
                        None => p0 + (p1 - p0) * f,
                    };
                    angs.push(angle_of(p));
                }
            }
            angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            // Largest gap between consecutive (wrapped) angles; the window is the
            // complement of that gap.
            let m = angs.len();
            let mut gap_i = 0usize;
            let mut gap = 0.0f64;
            for i in 0..m {
                let d = if i + 1 < m {
                    angs[i + 1] - angs[i]
                } else {
                    angs[0] + TAU - angs[m - 1]
                };
                if d > gap {
                    gap = d;
                    gap_i = i;
                }
            }
            // Window runs CCW from the sample after the gap to the one before it.
            let lo = angs[(gap_i + 1) % m];
            let end = if gap_i + 1 < m {
                angs[gap_i] + TAU
            } else {
                angs[gap_i]
            };
            let center = 0.5 * (lo + end);
            let half = 0.5 * (end - lo);
            (center, half)
        })
        .collect();
    let mut owner: Vec<usize> = Vec::with_capacity(circle_arcs.len());
    for arc in circle_arcs {
        let c = arc.curve()?;
        let am = angle_of(c.point(0.5 * (arc.first() + arc.last())));
        let mut best = (f64::INFINITY, 0usize);
        for (fi, &(center, half)) in windows.iter().enumerate() {
            // Distance past the window edge (≤ 0 when inside); pick the least.
            let over = ang_diff(am, center).abs() - half;
            if over < best.0 {
                best = (over, fi);
            }
        }
        owner.push(best.1);
    }

    // Assemble each collar face, seeding the chain with a kept extremity edge
    // in its original wire traversal so the old winding survives.
    let mut out: Vec<Face> = Vec::with_capacity(n);
    for (fi, face) in wall_old.iter().enumerate() {
        let mut pool: Vec<Edge> = keeps[fi].to_vec();
        for s in &seams[fi] {
            pool.push(retrim(s)?);
        }
        for (ai, arc) in circle_arcs.iter().enumerate() {
            if owner[ai] == fi {
                pool.push(arc.clone());
            }
        }
        let chained = chain_loop(pool);
        // The greedy chain must close head-to-tail; a gap means the boundary
        // wasn't the clean [extremity arcs + seams + circle] ring we assumed.
        let tol = (1e-6f64).max(z_eps);
        for i in 0..chained.len() {
            let a = chained[i].target().point();
            let b = chained[(i + 1) % chained.len()].source().point();
            if a.distance(&b) > tol {
                return None;
            }
        }
        out.push(Face::with_wires(
            face.surface().cloned(),
            Some(Wire::from_edges(chained)),
            Vec::new(),
            face.orientation(),
        ));
    }
    Some(out)
}

/// The old wall's axial seam angles (radians about the axis), deduped.
fn wall_seam_angles(
    wall_old: &[Face],
    origin: Pnt,
    av: GeomVec,
    e1v: GeomVec,
    e2v: GeomVec,
    z_eps: f64,
) -> Vec<f64> {
    let mut seam_angles: Vec<f64> = Vec::new();
    for face in wall_old {
        for wire in &face.wires() {
            for e in wire.edges().iter() {
                let (p0, p1) = (e.start().point(), e.end().point());
                let (za, zb) = ((p0 - origin).dot(&av), (p1 - origin).dot(&av));
                if (za - zb).abs() <= z_eps {
                    continue;
                }
                let a = angle_about(p0, origin, &av, &e1v, &e2v);
                if seam_angles.iter().all(|&s| ang_gap(s, a) > 1e-6) {
                    seam_angles.push(a);
                }
            }
        }
    }
    seam_angles
}

/// Fusion-style cut-through assembly: the thread bands extend THROUGH the
/// adjacent chamfer cone(s) and the cone segments are rebuilt with the helical
/// channel cut out of them; non-cone ends keep their FlatCut / runout+collar
/// treatment. Returns the resewn watertight solid, or `None` so the caller can
/// fall back to the pure runout construction.
#[allow(clippy::too_many_arguments)]
fn cone_cut_assembly(
    keep: &[Face],
    wall_old: &[Face],
    origin: Pnt,
    axis: GeomVec,
    av: GeomVec,
    e1v: GeomVec,
    e2v: GeomVec,
    r: f64,
    spec: &ThreadSpec,
    win_lo: f64,
    win_hi: f64,
    flat_bottom: bool,
    flat_top: bool,
    cone_bot: &Option<ConeNeighbor>,
    cone_top: &Option<ConeNeighbor>,
    z_min: f64,
    z_max: f64,
    z_eps: f64,
    r_eps: f64,
    policy: &TolerancePolicy,
) -> Option<KernelSolid> {
    let external = !spec.internal;
    let lead = spec.pitch as f64;
    let span = z_max - z_min;
    let eps_collar = (0.05 * lead).min(0.01 * span);
    let runout_len = (0.5 * lead).min(0.25 * (win_hi - win_lo));

    // Per-end bounds: cone cut-through > flat cut > runout hand-off plane.
    let (hi_bound, hi_circle) = if let Some(cn) = cone_top {
        (EndBound::Cone(cn.bound), None)
    } else if flat_top {
        (EndBound::Plane(win_hi), None)
    } else {
        let zr = if win_hi >= z_max - z_eps {
            z_max - eps_collar
        } else {
            win_hi
        };
        (EndBound::Plane(zr - runout_len), Some(zr))
    };
    let (lo_bound, lo_circle) = if let Some(cn) = cone_bot {
        (EndBound::Cone(cn.bound), None)
    } else if flat_bottom {
        (EndBound::Plane(win_lo), None)
    } else {
        let zr = if win_lo <= z_min + z_eps {
            z_min + eps_collar
        } else {
            win_lo
        };
        (EndBound::Plane(zr + runout_len), Some(zr))
    };
    let lo_primary = match &lo_bound {
        EndBound::Plane(z) => *z,
        EndBound::Cone(cb) => cb.z_base,
    };
    let hi_primary = match &hi_bound {
        EndBound::Plane(z) => *z,
        EndBound::Cone(cb) => cb.z_base,
    };
    if hi_primary - lo_primary < lead * 1.05 {
        return None;
    }

    let tp = ThreadProfile::new(origin, axis, r, lo_primary, spec)?;
    let mut split_angles: Vec<f64> = Vec::new();
    for cn in [cone_bot, cone_top].into_iter().flatten() {
        split_angles.extend(cn.seam_angles.iter().copied());
    }
    let bands = thread_bands_clipped(&tp, &lo_bound, &hi_bound, external, &split_angles)?;

    let mut faces: Vec<Face> = Vec::new();
    let consumed: HashSet<usize> = [cone_bot, cone_top]
        .into_iter()
        .flatten()
        .flat_map(|c| c.face_idx.iter().copied())
        .collect();

    // Per cone end: rebuild the cone segments with the channel cut through;
    // CASE B (the groove punches through the far plane) also queues a cap-rim
    // replacement on the face that rode the plain far circle.
    struct CapFix {
        z_far: f64,
        r_far: f64,
        pool: Vec<Edge>,
        done: bool,
    }
    let mut cap_fixes: Vec<CapFix> = Vec::new();
    for (cn, trims, cut_edges) in [
        (cone_bot, &bands.lo_trims, &bands.lo_cone_edges),
        (cone_top, &bands.hi_trims, &bands.hi_cone_edges),
    ] {
        let Some(cn) = cn else { continue };
        let case_b = trims.iter().any(|t| t.is_some());
        let (far_arcs, cap_pool) = if case_b {
            far_plane_arcs(&tp, &cn.bound, trims, &cn.seam_angles)?
        } else {
            (Vec::new(), Vec::new())
        };
        let segs: Vec<&Face> = cn.face_idx.iter().map(|&i| &keep[i]).collect();
        let remnants = cone_remnant_faces(
            &tp, &cn.bound, &segs, cut_edges, &far_arcs, case_b, external, z_eps, r_eps,
        )?;
        faces.extend(remnants);
        if case_b {
            cap_fixes.push(CapFix {
                z_far: cn.bound.z_far,
                r_far: cn.bound.r_far,
                pool: cap_pool,
                done: false,
            });
        }
    }

    // Flat-end profile outlines (all bands must have their trim).
    let profile_outline = |bottom: bool| -> Option<Vec<Edge>> {
        let trims = if bottom {
            &bands.lo_trims
        } else {
            &bands.hi_trims
        };
        trims.iter().cloned().collect()
    };
    let flat_profile_bottom = if flat_bottom {
        Some(profile_outline(true)?)
    } else {
        None
    };
    let flat_profile_top = if flat_top {
        Some(profile_outline(false)?)
    } else {
        None
    };

    let rim_of = |w: &Wire| -> Option<bool> {
        let mut on_bottom = true;
        let mut on_top = true;
        let mut any = false;
        for edge in w.edges().iter() {
            for p in [edge.start().point(), edge.end().point()] {
                any = true;
                let rel = p - origin;
                let z = rel.dot(&av);
                let rad = (rel - av * z).magnitude();
                if (rad - r).abs() > r_eps {
                    return None;
                }
                on_bottom &= (z - z_min).abs() <= z_eps;
                on_top &= (z - z_max).abs() <= z_eps;
            }
        }
        match (any, on_bottom, on_top) {
            (true, true, false) => Some(true),
            (true, false, true) => Some(false),
            _ => None,
        }
    };
    let rewound = |pool: Vec<Edge>, like: &Wire| -> Wire {
        let w = Wire::from_edges(chain_loop(pool));
        if wire_newell(like).dot(&wire_newell(&w)) < 0.0 {
            wire_reversed(&w)
        } else {
            w
        }
    };
    // Maps a kept wire to its replacement (rim outline at a flat end, or the
    // CASE-B mixed cap outline); `None` = unchanged.
    let map_wire = |w: &Wire, cap_fixes: &mut Vec<CapFix>| -> Option<Wire> {
        match rim_of(w) {
            Some(true) => {
                if let Some(pe) = &flat_profile_bottom {
                    return Some(rewound(pe.clone(), w));
                }
            }
            Some(false) => {
                if let Some(pe) = &flat_profile_top {
                    return Some(rewound(pe.clone(), w));
                }
            }
            None => {}
        }
        'fix: for cf in cap_fixes.iter_mut() {
            for e in w.edges().iter() {
                for p in [e.start().point(), e.end().point()] {
                    let rel = p - origin;
                    let z = rel.dot(&av);
                    let rad = (rel - av * z).magnitude();
                    if (z - cf.z_far).abs() > z_eps * 10.0 || (rad - cf.r_far).abs() > r_eps * 10.0
                    {
                        continue 'fix;
                    }
                }
            }
            cf.done = true;
            return Some(rewound(cf.pool.clone(), w));
        }
        None
    };

    for (fi, face) in keep.iter().enumerate() {
        if consumed.contains(&fi) {
            continue;
        }
        let outer = face.outer_wire();
        let inners = face.inner_wires();
        let new_outer = outer.as_ref().and_then(|w| map_wire(w, &mut cap_fixes));
        let new_inners: Vec<(Wire, bool)> = inners
            .into_iter()
            .map(|w| match map_wire(&w, &mut cap_fixes) {
                Some(nw) => (nw, true),
                None => (w, false),
            })
            .collect();
        if new_outer.is_none() && new_inners.iter().all(|(_, c)| !c) {
            faces.push(face.clone());
            continue;
        }
        let fo = match (outer, new_outer) {
            (_, Some(nw)) => Some(nw),
            (w, None) => w,
        };
        faces.push(Face::with_wires(
            face.surface().cloned(),
            fo,
            new_inners.into_iter().map(|(w, _)| w).collect(),
            face.orientation(),
        ));
    }
    if cap_fixes.iter().any(|c| !c.done) {
        return None; // CASE B but no cap face rode the far circle
    }

    // Runout + collar at any remaining plain end.
    let seams_wall = wall_seam_angles(wall_old, origin, av, e1v, e2v, z_eps);
    for (circle, bottom) in [(lo_circle, true), (hi_circle, false)] {
        let Some(zr) = circle else { continue };
        let profile = profile_outline(bottom)?;
        let (rf, arcs) = thread_runout_faces(origin, av, e1v, r, &profile, zr, &seams_wall)?;
        let (zk, zf) = if bottom {
            (z_min, z_max)
        } else {
            (z_max, z_min)
        };
        let cf = collar_faces(
            wall_old, origin, av, e1v, e2v, r, zk, zf, zr, &arcs, z_eps, r_eps,
        )?;
        faces.extend(rf);
        faces.extend(cf);
    }

    faces.extend(bands.faces);

    let mut solid = Solid::new(openrcad::algo::sew_with_policy(&faces, policy).ok()?.value);
    if !solid.is_watertight_with_policy(policy) {
        solid = openrcad::algo::merge::heal_tjunctions_with_policy(&solid, policy);
    }
    if !solid.is_watertight_with_policy(policy) {
        log::warn!("thread cone cut-through did not close watertight");
        return None;
    }
    Some(solid)
}

/// Cut a real thread into an existing body by **replacing its cylindrical wall
/// faces** (a hole wall for an internal thread, a rod or boss wall for an
/// external one) with the analytic helical thread wall — no boolean. A wall end
/// that abuts a flat cap rim cuts straight through the profile (Fusion-style,
/// the cap rim wire is swapped for the thread's cross-section outline). An end
/// abutting a CHAMFER CONE (or countersink) cuts THROUGH it: the bands extend
/// into the cone zone and the cone comes back with the helical channel carved
/// out — the Fusion look. Any other end — a fillet torus, an unrecognized
/// neighbor, or a partial-length stop set by `length`/`flip` — gets a RUNOUT
/// band fading the tooth to a plain circle plus a cylinder COLLAR whose
/// extremity edges are the old wall's own, so the neighbor resews untouched.
/// Returns `None` (leaving the body untouched) when no matching cylindrical
/// faces exist, the wall shape can't be classified, the thread window is
/// shorter than a full turn, or the resewn solid does not close watertight.
#[deprecated(note = "use threaded_replace_cylinder_wall_with_policy")]
pub fn threaded_replace_cylinder_wall(
    part: &KernelSolid,
    info: &CylinderFaceInfo,
    spec: &ThreadSpec,
    length: Option<f64>,
    flip: bool,
) -> Option<KernelSolid> {
    threaded_replace_cylinder_wall_with_policy(
        part,
        info,
        spec,
        length,
        flip,
        &TolerancePolicy::STANDARD,
    )
}

pub fn threaded_replace_cylinder_wall_with_policy(
    part: &KernelSolid,
    info: &CylinderFaceInfo,
    spec: &ThreadSpec,
    length: Option<f64>,
    flip: bool,
    policy: &TolerancePolicy,
) -> Option<KernelSolid> {
    policy.validate().ok()?;
    let want_dir = GeomVec::new(info.dir[0] as f64, info.dir[1] as f64, info.dir[2] as f64);
    let want_r = info.radius as f64;
    let want_origin = Pnt::new(
        info.origin[0] as f64,
        info.origin[1] as f64,
        info.origin[2] as f64,
    );

    // Partition the body's faces: the wall = every cylinder face on the same
    // axis line with the same radius; everything else is kept.
    let mut wall_old: Vec<Face> = Vec::new();
    let mut keep: Vec<Face> = Vec::new();
    let mut axis_exact: Option<(Pnt, GeomVec, f64)> = None;
    for face in part.shell().faces() {
        let is_wall = match face.surface() {
            Some(GeomSurface::Cylinder(cyl)) => {
                let pos = cyl.position();
                let d = GeomVec::from_dir(pos.direction());
                let parallel =
                    d.dot(&want_dir).abs() / want_dir.magnitude().max(1e-12) > 1.0 - 1e-4;
                let r_ok = (cyl.radius() - want_r).abs() <= 1e-3 * want_r.max(1.0);
                let rel = pos.location() - want_origin;
                let along = rel.dot(&want_dir) / want_dir.dot(&want_dir).max(1e-12);
                let radial = rel - want_dir * along;
                let axis_ok = radial.magnitude() <= 1e-3 * want_r.max(1.0);
                if parallel && r_ok && axis_ok {
                    if axis_exact.is_none() {
                        let dir = if d.dot(&want_dir) >= 0.0 { d } else { d * -1.0 };
                        axis_exact = Some((pos.location(), dir, cyl.radius()));
                    }
                    true
                } else {
                    false
                }
            }
            _ => false,
        };
        if is_wall {
            wall_old.push(face.clone());
        } else {
            keep.push(face.clone());
        }
    }
    let (origin, axis, r) = axis_exact?;
    let (av, e1v, e2v) = thread_radial_basis(axis)?;

    // Exact axial extent from the old wall's own topology.
    let mut z_min = f64::INFINITY;
    let mut z_max = f64::NEG_INFINITY;
    for face in &wall_old {
        for wire in &face.wires() {
            for edge in wire.edges().iter() {
                for p in [edge.start().point(), edge.end().point()] {
                    let z = (p - origin).dot(&av);
                    z_min = z_min.min(z);
                    z_max = z_max.max(z);
                }
            }
        }
    }
    if !z_min.is_finite() || z_max - z_min <= 1e-6 {
        return None;
    }
    let span = z_max - z_min;

    let z_eps = 1e-5 * span.max(1.0);
    let r_eps = 1e-4 * r.max(1.0);
    let rim_of = |w: &Wire| -> Option<bool> {
        // Some(true) = bottom rim, Some(false) = top rim, None = not a rim.
        let mut on_bottom = true;
        let mut on_top = true;
        let mut any = false;
        for edge in w.edges().iter() {
            for p in [edge.start().point(), edge.end().point()] {
                any = true;
                let rel = p - origin;
                let z = rel.dot(&av);
                let rad = (rel - av * z).magnitude();
                if (rad - r).abs() > r_eps {
                    return None;
                }
                on_bottom &= (z - z_min).abs() <= z_eps;
                on_top &= (z - z_max).abs() <= z_eps;
            }
        }
        match (any, on_bottom, on_top) {
            (true, true, false) => Some(true),
            (true, false, true) => Some(false),
            _ => None,
        }
    };

    // The thread window along the wall: full span, or a partial length
    // anchored at the z_max end (`flip` anchors at z_min instead).
    let full = length.is_none_or(|l| l <= 0.0 || l >= span - z_eps);
    let (win_lo, win_hi) = if full {
        (z_min, z_max)
    } else if flip {
        (z_min, (z_min + length.unwrap_or(span)).min(z_max))
    } else {
        ((z_max - length.unwrap_or(span)).max(z_min), z_max)
    };
    // A modeled thread must leave an ordinary analytic boundary for whatever
    // feature follows it in the body timeline. Cutting the helical profile
    // straight through a planar end cap made a later Join/Cut intersect a
    // high-degree thread outline at the cap and was both scale-fragile and
    // construction-order dependent. Terminate at every end through the same
    // deterministic runout + cylindrical collar used for partial threads.
    // The original cap wire is then preserved verbatim and subsequent body
    // operations meet a cylinder/plane pair instead of a helical seam.
    let flat_bottom = false;
    let flat_top = false;

    // Fusion-style cut-through: when a chamfer cone (or countersink) rides a
    // non-flat end of the window, extend the thread THROUGH it and carve the
    // helical channel out of the cone. Any failure falls back to the
    // runout+collar construction below.
    let external = !spec.internal;
    let cone_top = if !flat_top && win_hi >= z_max - z_eps {
        detect_cone_neighbor(
            &keep, origin, av, e1v, e2v, r, z_max, 1.0, external, z_eps, r_eps,
        )
    } else {
        None
    };
    let cone_bot = if !flat_bottom && win_lo <= z_min + z_eps {
        detect_cone_neighbor(
            &keep, origin, av, e1v, e2v, r, z_min, -1.0, external, z_eps, r_eps,
        )
    } else {
        None
    };
    if cone_top.is_some() || cone_bot.is_some() {
        if let Some(solid) = cone_cut_assembly(
            &keep,
            &wall_old,
            origin,
            axis,
            av,
            e1v,
            e2v,
            r,
            spec,
            win_lo,
            win_hi,
            flat_bottom,
            flat_top,
            &cone_bot,
            &cone_top,
            z_min,
            z_max,
            z_eps,
            r_eps,
            policy,
        ) {
            return Some(solid);
        }
        log::warn!("thread cone cut-through failed; falling back to runout+collar");
    }

    // Per-end geometry: a FlatCut end reaches the extremity exactly; a runout
    // end pulls the band back by `runout_len` and lands the fade circle at the
    // window end (inset by a hair-thin collar when the window touches an
    // extremity whose neighbor isn't a flat cap, e.g. a chamfered rim).
    let lead = spec.pitch as f64;
    // Keep the collar wider than the feature-operation coplanarity nudge used
    // by ZeroCAD (0.1 model unit at standard scale). This is expressed through
    // thread/part scale rather than a hard-coded global tolerance.
    let eps_collar = (0.25 * lead).min(0.05 * span);
    let runout_len = (0.5 * lead).min(0.25 * (win_hi - win_lo));
    let (z_band_hi, z_circle_top) = if flat_top {
        (win_hi, None)
    } else {
        let zr = if win_hi >= z_max - z_eps {
            z_max - eps_collar
        } else {
            win_hi
        };
        (zr - runout_len, Some(zr))
    };
    let (z_band_lo, z_circle_bot) = if flat_bottom {
        (win_lo, None)
    } else {
        let zr = if win_lo <= z_min + z_eps {
            z_min + eps_collar
        } else {
            win_lo
        };
        (zr + runout_len, Some(zr))
    };
    if z_band_hi - z_band_lo < lead * 1.05 {
        return None;
    }

    let wall = thread_wall_faces(origin, axis, r, z_band_lo, z_band_hi, spec)?;

    // Runout + collar for the non-flat ends. The collars need the old wall's
    // axial seam angles so runout circle arcs split where collar corners land.
    let mut extra_faces: Vec<Face> = Vec::new();
    if z_circle_top.is_some() || z_circle_bot.is_some() {
        let seam_angles = wall_seam_angles(&wall_old, origin, av, e1v, e2v, z_eps);
        if let Some(zr) = z_circle_top {
            let Some((rf, arcs)) =
                thread_runout_faces(origin, av, e1v, r, &wall.top_profile, zr, &seam_angles)
            else {
                log::warn!("thread top runout band failed to build");
                return None;
            };
            let Some(cf) = collar_faces(
                &wall_old, origin, av, e1v, e2v, r, z_max, z_min, zr, &arcs, z_eps, r_eps,
            ) else {
                log::warn!("thread top collar failed to build");
                return None;
            };
            extra_faces.extend(rf);
            extra_faces.extend(cf);
        }
        if let Some(zr) = z_circle_bot {
            let Some((rf, arcs)) =
                thread_runout_faces(origin, av, e1v, r, &wall.bottom_profile, zr, &seam_angles)
            else {
                log::warn!("thread bottom runout band failed to build");
                return None;
            };
            let Some(cf) = collar_faces(
                &wall_old, origin, av, e1v, e2v, r, z_min, z_max, zr, &arcs, z_eps, r_eps,
            ) else {
                log::warn!("thread bottom collar failed to build");
                return None;
            };
            extra_faces.extend(rf);
            extra_faces.extend(cf);
        }
    }

    // Re-trim the adjacent end faces at FlatCut ends: every kept wire that
    // rides that end's rim circle is replaced by the thread's cross-section
    // outline (the SAME edge objects the band faces use, so sewing pairs them
    // exactly). Rim wires at runout ends stay untouched — the collar keeps
    // their counterpart edges verbatim.
    let profile_wire = |bottom: bool, like: &Wire| -> Wire {
        let edges = if bottom {
            wall.bottom_profile.clone()
        } else {
            wall.top_profile.clone()
        };
        let w = Wire::from_edges(chain_loop(edges));
        // Match the replaced wire's winding so hole wires stay holes.
        let n_old = wire_newell(like);
        let n_new = wire_newell(&w);
        if n_old.dot(&n_new) < 0.0 {
            wire_reversed(&w)
        } else {
            w
        }
    };

    // Only rims at a FlatCut end get replaced; a rim at a runout end (e.g. the
    // far cap of a partial thread) must keep its wire — the collar sews to it.
    let rim_replaced = |w: &Wire| -> Option<bool> {
        rim_of(w).filter(|&bottom| if bottom { flat_bottom } else { flat_top })
    };

    let mut faces: Vec<Face> = Vec::new();
    for face in keep {
        let outer = face.outer_wire();
        let inners = face.inner_wires();
        let outer_rim = outer.as_ref().and_then(rim_replaced);
        let inner_rims: Vec<Option<bool>> = inners.iter().map(rim_replaced).collect();
        if outer_rim.is_none() && inner_rims.iter().all(|r| r.is_none()) {
            faces.push(face);
            continue;
        }
        let new_outer = match (outer, outer_rim) {
            (Some(w), Some(bottom)) => Some(profile_wire(bottom, &w)),
            (w, _) => w,
        };
        let new_inners: Vec<Wire> = inners
            .into_iter()
            .zip(inner_rims)
            .map(|(w, rim)| match rim {
                Some(bottom) => profile_wire(bottom, &w),
                None => w,
            })
            .collect();
        faces.push(Face::with_wires(
            face.surface().cloned(),
            new_outer,
            new_inners,
            face.orientation(),
        ));
    }
    faces.extend(wall.faces);
    faces.extend(extra_faces);

    let mut solid = Solid::new(openrcad::algo::sew_with_policy(&faces, policy).ok()?.value);
    if !solid.is_watertight_with_policy(policy) {
        // Safety net for hosts whose rim vertices don't line up exactly.
        solid = openrcad::algo::merge::heal_tjunctions_with_policy(&solid, policy);
    }
    let (solid, _) = solid.repair_pcurves(policy).ok()?;
    if !solid.is_watertight_with_policy(policy)
        || solid.validate_strict_with_policy(policy).is_err()
    {
        log::warn!("thread wall replacement did not close watertight");
        return None;
    }
    Some(solid)
}

/// Build a **whole threaded cylinder** as one watertight solid: the analytic
/// helical thread wall of [`thread_wall_faces`] closed with two planar caps
/// whose rims are the thread's cross-section outline (the thread cuts straight
/// through the ends, like Fusion). `4·starts + 2` large smooth B-Rep faces —
/// each crest/flank/root band spans the full thread length — and NO boolean:
/// fast and robust where a helical-tool difference against a smooth cylinder
/// is neither.
#[deprecated(note = "use threaded_cylinder_solid_with_policy")]
pub fn threaded_cylinder_solid(
    axis_origin: crate::geometry::Vec3,
    axis_dir: crate::geometry::Vec3,
    radius: f32,
    axial_min: f32,
    axial_max: f32,
    spec: &ThreadSpec,
) -> Option<KernelSolid> {
    threaded_cylinder_solid_with_policy(
        axis_origin,
        axis_dir,
        radius,
        axial_min,
        axial_max,
        spec,
        &TolerancePolicy::STANDARD,
    )
}

pub fn threaded_cylinder_solid_with_policy(
    axis_origin: crate::geometry::Vec3,
    axis_dir: crate::geometry::Vec3,
    radius: f32,
    axial_min: f32,
    axial_max: f32,
    spec: &ThreadSpec,
    policy: &TolerancePolicy,
) -> Option<KernelSolid> {
    policy.validate().ok()?;
    let origin = Pnt::new(
        axis_origin.x as f64,
        axis_origin.y as f64,
        axis_origin.z as f64,
    );
    let axis = GeomVec::new(axis_dir.x as f64, axis_dir.y as f64, axis_dir.z as f64);
    let wall = thread_wall_faces(
        origin,
        axis,
        radius as f64,
        axial_min as f64,
        axial_max as f64,
        spec,
    )?;
    let mut faces = wall.faces;

    let bottom_plane = GeomSurface::plane(Plane::from_point_normal(
        wall.bottom_center,
        Dir::new(-wall.axis.x(), -wall.axis.y(), -wall.axis.z()),
    ));
    let bottom_wire = Wire::from_edges(chain_loop(wall.bottom_profile.clone()));
    faces.push(Face::new(
        Some(bottom_plane.clone()),
        wire_ccw_on(bottom_wire, &bottom_plane, (0.0, 0.0)),
    ));
    let top_plane = GeomSurface::plane(Plane::from_point_normal(wall.top_center, wall.axis));
    let top_wire = Wire::from_edges(chain_loop(wall.top_profile.clone()));
    faces.push(Face::new(
        Some(top_plane.clone()),
        wire_ccw_on(top_wire, &top_plane, (0.0, 0.0)),
    ));

    let solid = Solid::new(openrcad::algo::sew_with_policy(&faces, policy).ok()?.value);
    let (solid, _) = solid.repair_pcurves(policy).ok()?;
    if !solid.is_watertight_with_policy(policy)
        || solid.validate_strict_with_policy(policy).is_err()
    {
        log::warn!("threaded cylinder wall did not close watertight");
        return None;
    }
    Some(solid)
}

/// Solid of revolution for one sketch region, about a world-space axis lying in
/// the sketch plane. Arc runs in the boundary are refit to true circles (like
/// [`extruded_region_solid_with_arcs`]) so revolved arcs become analytic tori/
/// spheres and straight runs cylinders/cones/planes. `None` when the profile
/// crosses the axis, the axis is out of plane, or the angle is degenerate.
pub fn revolved_region_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    cs: &crate::geometry::CoordinateSystem,
    axis_origin: crate::geometry::Vec3,
    axis_dir: crate::geometry::Vec3,
    angle_rad: f64,
    arc_circles: &[((f32, f32), f32)],
) -> Option<KernelSolid> {
    build_revolution_solid(
        points,
        holes,
        cs,
        axis_origin,
        axis_dir,
        angle_rad,
        arc_circles,
    )
}

/// Boolean fallback solid for one extruded sketch region, keeping the sampled
/// sketch polyline instead of reconstructing arcs into analytic cylinders.
///
/// The normal sketch-solid path above is preferred because it gives circular
/// cutouts true cylindrical topology. This faceted twin is only for emergency
/// edge-mod fallbacks where a cutter runout meets that cylindrical wall and the
/// boolean solver rejects the tangent analytic topology outright.
pub fn extruded_region_faceted_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return None;
    }
    build_extrusion_solid(points, holes, depth as f64, cs, false)
        .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs, false))
}

/// Canonical solid for a sketch region that is exactly a rectangle with one
/// circular side bite. This matches the user-visible box-minus-cylinder workflow:
/// build the rectangular prism first, then subtract an analytic cylinder.
pub fn rect_minus_circle_region_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    let (base, cutter) = rect_minus_circle_region_base_and_cutter(points, holes, depth, cs)?;
    difference(&base, &cutter)
}

/// Return the rectangular prism and analytic cylinder cutter for an unambiguous
/// rectangle-minus-circle sketch region.
pub fn rect_minus_circle_region_base_and_cutter(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<(KernelSolid, KernelSolid)> {
    rect_minus_circle_region_base_and_cutter_with_grow(points, holes, depth, cs, 0.0)
}

pub fn rect_minus_circle_region_base_and_grown_cutter(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    rect_minus_circle_region_base_and_cutter_with_grow(points, holes, depth, cs, radius_grow)
}

pub fn rect_minus_circle_region_base_and_faceted_cutter(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    let (rect_min, rect_max, circle_center, circle_radius) =
        rect_minus_circle_region_primitives(points, holes)?;
    rect_circle_base_and_faceted_cutter_from_primitives(
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        depth,
        cs,
        radius_grow,
    )
}

/// Return the rectangular prism and analytic cylinder cutter from the original
/// sketch primitives, avoiding the brittle "recover the circle from the sampled
/// region boundary" path used for generic fallback recognition.
pub fn rect_circle_base_and_cutter_from_primitives(
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    circle_center: (f32, f32),
    circle_radius: f32,
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    let (min_x, max_x) = if rect_min.0 <= rect_max.0 {
        (rect_min.0, rect_max.0)
    } else {
        (rect_max.0, rect_min.0)
    };
    let (min_y, max_y) = if rect_min.1 <= rect_max.1 {
        (rect_min.1, rect_max.1)
    } else {
        (rect_max.1, rect_min.1)
    };
    if max_x - min_x <= 1.0e-3
        || max_y - min_y <= 1.0e-3
        || circle_radius <= 1.0e-3
        || depth.abs() < f32::EPSILON
    {
        return None;
    }

    let rect = vec![
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ];
    let base = build_extrusion_solid(&rect, &[], depth as f64, cs, true)?;

    let sign = if depth < 0.0 { -1.0 } else { 1.0 };
    let overshoot = 0.25;
    // Shift the origin only — `CoordinateSystem::new` would recompute n = u × v
    // and flip the sweep on the left-handed ground plane (cutter below ground →
    // the boolean removes nothing and the bite silently vanishes).
    let cut_cs = cs.with_origin(cs.origin.add(cs.n.mul(-sign * overshoot)));
    let cylinder_profile = circle_loop_2d(circle_center, circle_radius + radius_grow.max(0.0));
    let cutter = circular_cylinder_tool(
        &cylinder_profile,
        &[],
        depth + sign * 2.0 * overshoot,
        &cut_cs,
    )?;
    Some((base, cutter))
}

pub fn rect_circle_base_and_faceted_cutter_from_primitives(
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    circle_center: (f32, f32),
    circle_radius: f32,
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    let (min_x, max_x) = if rect_min.0 <= rect_max.0 {
        (rect_min.0, rect_max.0)
    } else {
        (rect_max.0, rect_min.0)
    };
    let (min_y, max_y) = if rect_min.1 <= rect_max.1 {
        (rect_min.1, rect_max.1)
    } else {
        (rect_max.1, rect_min.1)
    };
    if max_x - min_x <= 1.0e-3
        || max_y - min_y <= 1.0e-3
        || circle_radius <= 1.0e-3
        || depth.abs() < f32::EPSILON
    {
        return None;
    }

    let rect = vec![
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ];
    let base = build_extrusion_solid(&rect, &[], depth as f64, cs, true)?;

    let sign = if depth < 0.0 { -1.0 } else { 1.0 };
    let overshoot = 0.25;
    // Origin shift only — see rect_circle_base_and_cutter_from_primitives.
    let cut_cs = cs.with_origin(cs.origin.add(cs.n.mul(-sign * overshoot)));
    let cylinder_profile = circle_loop_2d(circle_center, circle_radius + radius_grow.max(0.0));
    let cutter = extruded_region_faceted_solid(
        &cylinder_profile,
        &[],
        depth + sign * 2.0 * overshoot,
        &cut_cs,
    )?;
    Some((base, cutter))
}

pub(crate) fn rect_minus_circle_region_base_and_cutter_with_grow(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    let (rect_min, rect_max, circle_center, circle_radius) =
        rect_minus_circle_region_primitives(points, holes)?;
    rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        depth,
        cs,
        radius_grow,
    )
}

pub(crate) fn rect_minus_circle_region_primitives(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
) -> Option<((f32, f32), (f32, f32), (f32, f32), f32)> {
    if !holes.is_empty() || points.len() < 8 {
        return None;
    }

    let ((min_x, min_y), (max_x, max_y)) = loop_bounds_2d(points)?;
    if max_x - min_x <= 1.0e-3 || max_y - min_y <= 1.0e-3 {
        return None;
    }

    let on_rect_side = |p: (f32, f32)| {
        const SIDE_EPS: f32 = 0.08;
        (p.0 - min_x).abs() <= SIDE_EPS
            || (p.0 - max_x).abs() <= SIDE_EPS
            || (p.1 - min_y).abs() <= SIDE_EPS
            || (p.1 - max_y).abs() <= SIDE_EPS
    };

    let arc_pts: Vec<(f32, f32)> = points
        .iter()
        .copied()
        .filter(|&p| !on_rect_side(p))
        .collect();
    if arc_pts.len() < 5 {
        return None;
    }

    let (cx, cy, r) = circle_from_three_points_2d(
        arc_pts[0],
        arc_pts[arc_pts.len() / 2],
        arc_pts[arc_pts.len() - 1],
    )?;
    if !cx.is_finite() || !cy.is_finite() || !r.is_finite() || r <= 1.0e-3 {
        return None;
    }

    let circle_tol = (0.02 * r).max(0.12);
    let near_circle = |p: (f32, f32)| ((p.0 - cx).hypot(p.1 - cy) - r).abs() <= circle_tol;
    if arc_pts.iter().filter(|&&p| near_circle(p)).count() < arc_pts.len() * 3 / 4 {
        return None;
    }

    // A genuine rect-minus-circle MATERIAL boundary keeps at least two of the
    // rectangle's corners as vertices well clear of the circle (a mid-side bite
    // keeps all four, a corner bite keeps three). A plain circle profile — whose
    // tangent points against its own bounding box can otherwise satisfy the
    // side-hit test below — keeps none, and must NOT be mistaken for a bite: it
    // would fabricate a bite "void" covering the entire body.
    let corners = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ];
    let corner_hits = corners
        .iter()
        .filter(|corner| {
            points
                .iter()
                .any(|p| (p.0 - corner.0).abs() <= 0.08 && (p.1 - corner.1).abs() <= 0.08)
                && !near_circle(**corner)
        })
        .count();
    if corner_hits < 2 {
        return None;
    }

    // Require the circular run to enter and leave through the same rectangle
    // side. Rounded rectangle corners touch two different sides and should keep
    // the normal arc-reconstructed extrusion path.
    let side_hit_count = |side: usize| -> usize {
        points
            .iter()
            .filter(|&&p| {
                let on_side = match side {
                    0 => (p.0 - min_x).abs() <= 0.08,
                    1 => (p.0 - max_x).abs() <= 0.08,
                    2 => (p.1 - min_y).abs() <= 0.08,
                    _ => (p.1 - max_y).abs() <= 0.08,
                };
                on_side && near_circle(p)
            })
            .count()
    };
    if (0..4).map(side_hit_count).max().unwrap_or(0) < 2 {
        return None;
    }

    Some(((min_x, min_y), (max_x, max_y), (cx, cy), r))
}

/// Display mesh for one extruded sketch region. Plain polygon profiles keep the
/// lightweight analytic-prism mesh, while profiles containing reconstructed
/// circular arcs render from the same B-Rep solid used for booleans so the
/// viewport matches a box-minus-cylinder cutout.
pub fn extruded_region_display_mesh(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> MockMesh {
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return MockMesh::empty();
    }

    // Full circles already have a dedicated oriented-cylinder display path with
    // clean rim wireframes.
    if holes.is_empty() && circle_profile(points).is_some() {
        return MockMesh::make_extruded_sketch(points, holes, depth, cs);
    }

    if let Some(solid) = rect_minus_circle_region_solid(points, holes, depth, cs) {
        let mesh = MockMesh::from_solid(&solid);
        if !mesh.indices.is_empty()
            && render_mesh_normals_follow_winding(&mesh)
            && render_mesh_is_closed_manifold(&mesh)
        {
            return mesh;
        }
    }

    if profile_has_reconstructable_arcs(points, holes) {
        if let Some(solid) = build_extrusion_solid(points, holes, depth as f64, cs, true) {
            if solid_has_cylindrical_face(&solid) {
                let mesh = MockMesh::from_solid(&solid);
                let needs_clean_fallback = !holes.is_empty() || loop_is_concave(points);
                if !mesh.indices.is_empty()
                    && render_mesh_normals_follow_winding(&mesh)
                    && (!needs_clean_fallback || render_mesh_is_closed_manifold(&mesh))
                {
                    return mesh;
                }
            }
        }
    }

    let mut mesh = MockMesh::make_extruded_sketch(points, holes, depth, cs);
    if profile_has_reconstructable_arcs(points, holes) {
        let targets = arc_display_targets(points, holes);
        apply_arc_display_targets(&mut mesh, &targets, cs);
    }
    mesh
}

/// Display mesh derived **directly from the body's part solid** — the single
/// source of truth. The shaded surface and face/edge identity then always match
/// the kernel geometry a boolean/fillet will actually operate on (no separately
/// built display twin that can drift). Since [`MockMesh::from_solid`] now redraws
/// curved B-Rep edges as smooth analytic polylines, a cylinder rim reads as clean
/// here as on the old dedicated primitive path.
///
/// Falls back to the lightweight analytic prism/cylinder mesh
/// ([`extruded_region_display_mesh`]) only when meshing the part yields a cracked
/// or non-manifold render mesh (the same gate the rect-minus-circle path uses).
pub fn display_mesh_from_part(
    part: &KernelSolid,
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> MockMesh {
    try_display_mesh_from_part(part)
        .unwrap_or_else(|| extruded_region_display_mesh(points, holes, depth, cs))
}

/// Mesh a part solid for display, returning `None` when the result is cracked or
/// non-manifold (the caller then picks its own fallback). Since [`MockMesh::from_solid`]
/// now draws curved B-Rep edges as smooth analytic polylines, this gives a boolean
/// result — or a box/cylinder primitive — a wireframe as clean as the dedicated
/// analytic-mesh path, so the display can always derive from the single part solid.
pub fn try_display_mesh_from_part(part: &KernelSolid) -> Option<MockMesh> {
    let mesh = MockMesh::from_solid(part);
    (!mesh.indices.is_empty()
        && render_mesh_normals_follow_winding(&mesh)
        && render_mesh_is_closed_manifold(&mesh))
    .then_some(mesh)
}

pub(crate) fn build_cylinder_solid(r: f64, h: f64) -> Option<KernelSolid> {
    if r <= 0.0 || h <= 0.0 {
        return None;
    }
    // Base centered at the origin, swept along +Y — the axis the primitive
    // display path (`MockMesh::make_cylinder`) and its wireframe expect.
    consume_operation(
        "cylinder primitive",
        openrcad::primitives::make_cylinder_operation(&Ax2::new(Pnt::origin(), Dir::dy()), r, h),
    )
    .ok()
    .map(|outcome| outcome.solid)
}

// ---------------------------------------------------------------------------
// Arc reconstruction for extruded sketch profiles
//
// Sketch region boundaries arrive as dense polylines: every circle/arc the user
// drew (and every sketch-fillet) was flattened to line segments by region
// detection, which keeps only points (no curve provenance). Extruding those
// straight edges produces a fan of planar laterals — a "segmented" cylinder that
// shades as facets and litters the wireframe with vertical struts.
//
// `loop_to_wire` rebuilds the true geometry before the sweep: it finds maximal
// runs of consecutive boundary vertices that lie on a common circle and emits a
// single circular-arc `Edge` per run (split into <=135° pieces, matching the
// kernel's cylinder convention). `prism` then makes those arc edges into smooth
// cylindrical walls. Polygons have no co-circular run, so their corners stay
// sharp line edges — a rectangle still extrudes to a clean box.
// ---------------------------------------------------------------------------
