use super::*;

/// Axis-aligned box solid, one corner at the origin, opposite at (w, h, d).
pub fn box_solid(w: f32, h: f32, d: f32) -> KernelSolid {
    make_box(&Pnt::origin(), w as f64, h as f64, d as f64)
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
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return None;
    }
    if let Some(solid) = rect_minus_circle_region_solid(points, holes, depth, cs) {
        return Some(solid);
    }
    build_extrusion_solid(points, holes, depth as f64, cs, true)
        .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs, true))
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
    let cut_cs = crate::geometry::CoordinateSystem::new(
        cs.origin.add(cs.n.mul(-sign * overshoot)),
        cs.u,
        cs.v,
    );
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
    let cut_cs = crate::geometry::CoordinateSystem::new(
        cs.origin.add(cs.n.mul(-sign * overshoot)),
        cs.u,
        cs.v,
    );
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

pub(crate) fn build_cylinder_solid(r: f64, h: f64) -> Option<KernelSolid> {
    if r <= 0.0 || h <= 0.0 {
        return None;
    }
    // Base centered at the origin, swept along +Y — the axis the primitive
    // display path (`MockMesh::make_cylinder`) and its wireframe expect.
    Some(make_cylinder(&Ax2::new(Pnt::origin(), Dir::dy()), r, h))
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
