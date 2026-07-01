use super::*;

/// Run `f` with the panic hook silenced, restoring it afterward. `boolean_checked`
/// already catches the kernel's panics, but the *default* hook still prints the
/// panic (and any diagnostic dump) to stderr — which would spam the console on
/// every degraded boolean (e.g. a drag frame). Silencing it keeps recoverable
/// boolean failures quiet; the caller still just sees `None`.
pub(crate) fn quiet_panic<R>(f: impl FnOnce() -> R) -> R {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let r = f();
    std::panic::set_hook(prev);
    r
}

/// Boolean union (`a ∪ b`). Returns `None` if the kernel can't resolve the
/// configuration, panics, or produces a non-watertight result — callers decide
/// how to degrade. `boolean_checked` catches panics and rejects leaky output.
pub fn union(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    quiet_panic(|| boolean_checked(a, b, BooleanOp::Fuse).ok())
}

/// Boolean difference (`a − b`): subtract `b`'s volume from `a`. Returns `None`
/// on kernel failure or non-watertight output.
pub fn difference(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    quiet_panic(|| boolean_checked(a, b, BooleanOp::Cut).ok())
}

/// Boolean difference (`a − b`) that returns **one solid per connected
/// component** instead of a single shell.
///
/// A cut that *severs* `a` — e.g. a slot sliced clean through a bar, leaving two
/// separate lumps — comes back from the kernel as one watertight shell holding
/// both disjoint pieces (a valid B-Rep, but really two bodies). `difference`
/// hands that back as a single `KernelSolid`; the parametric evaluator instead
/// wants each lump as its own selectable body part. This runs the same guarded
/// cut, then splits the result into its connected components via
/// [`Solid::split_disconnected`].
///
/// Returns `None` on kernel failure or non-watertight output (the caller keeps
/// the part intact rather than dropping material). On success the vector always
/// has at least one element — an un-severed cut yields a single body.
pub fn difference_bodies(a: &KernelSolid, b: &KernelSolid) -> Option<Vec<KernelSolid>> {
    let result = difference(a, b)?;
    let parts = result.split_disconnected();
    let mut parts = if parts.is_empty() {
        vec![result]
    } else {
        parts
    };
    // Order the severed lumps by their canonical position key so the part list is
    // deterministic and position-based across rebuilds — the basis for a downstream
    // feature following a specific lump instead of a volatile list index.
    parts.sort_by_key(part_key);
    Some(parts)
}

/// Fallback for the common "rectangular pocket clean through an axis-aligned
/// block" case when the general boolean engine cannot resolve the coplanar
/// split. The result is rebuilt as one extruded face with a rectangular hole.
pub fn axis_aligned_through_cut(part: &KernelSolid, tool: &KernelSolid) -> Option<KernelSolid> {
    let (plo, phi) = solid_aabb(part)?;
    let (tlo, thi) = solid_aabb(tool)?;
    const EPS: f32 = 0.25;

    for axis in 0..3 {
        if tlo[axis] > plo[axis] + EPS || thi[axis] < phi[axis] - EPS {
            continue;
        }

        let axes: Vec<usize> = (0..3).filter(|&k| k != axis).collect();
        let a = axes[0];
        let b = axes[1];
        let ha0 = tlo[a].max(plo[a]);
        let ha1 = thi[a].min(phi[a]);
        let hb0 = tlo[b].max(plo[b]);
        let hb1 = thi[b].min(phi[b]);

        if ha0 <= plo[a] + EPS
            || ha1 >= phi[a] - EPS
            || hb0 <= plo[b] + EPS
            || hb1 >= phi[b] - EPS
            || ha1 <= ha0 + EPS
            || hb1 <= hb0 + EPS
        {
            continue;
        }

        let outer = vec![
            (0.0, 0.0),
            (phi[a] - plo[a], 0.0),
            (phi[a] - plo[a], phi[b] - plo[b]),
            (0.0, phi[b] - plo[b]),
        ];
        let hole = vec![
            (ha0 - plo[a], hb0 - plo[b]),
            (ha1 - plo[a], hb0 - plo[b]),
            (ha1 - plo[a], hb1 - plo[b]),
            (ha0 - plo[a], hb1 - plo[b]),
        ];

        let origin = Vec3::new(plo[0], plo[1], plo[2]);
        let cs = match axis {
            0 => crate::geometry::CoordinateSystem::new(origin, Vec3::Y, Vec3::Z),
            1 => crate::geometry::CoordinateSystem::new(origin, Vec3::X, Vec3::Z),
            _ => crate::geometry::CoordinateSystem::new(origin, Vec3::X, Vec3::Y),
        };

        return build_extrusion_solid(&outer, &[hole], (phi[axis] - plo[axis]) as f64, &cs, false);
    }

    None
}

/// Fallback for an axis-aligned cut when the exact boolean fails. It approximates
/// the removed volume by the tool AABB and decomposes `part - tool` into up to
/// six non-overlapping boxes, all kept as parts of the same ZeroCAD body.
pub fn axis_aligned_cut_parts(part: &KernelSolid, tool: &KernelSolid) -> Option<Vec<KernelSolid>> {
    let (plo, phi) = solid_aabb(part)?;
    let (tlo, thi) = solid_aabb(tool)?;
    const EPS: f32 = 0.01;

    let rlo = [tlo[0].max(plo[0]), tlo[1].max(plo[1]), tlo[2].max(plo[2])];
    let rhi = [thi[0].min(phi[0]), thi[1].min(phi[1]), thi[2].min(phi[2])];
    if (0..3).any(|k| rhi[k] <= rlo[k] + EPS) {
        return None;
    }

    let mut pieces = Vec::new();
    let mut push_box = |lo: [f32; 3], hi: [f32; 3]| {
        let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        if d.iter().all(|&v| v > EPS) {
            pieces.push(make_box(
                &Pnt::new(lo[0] as f64, lo[1] as f64, lo[2] as f64),
                d[0] as f64,
                d[1] as f64,
                d[2] as f64,
            ));
        }
    };

    push_box(plo, [rlo[0], phi[1], phi[2]]);
    push_box([rhi[0], plo[1], plo[2]], phi);

    let xlo = rlo[0];
    let xhi = rhi[0];
    push_box([xlo, plo[1], plo[2]], [xhi, rlo[1], phi[2]]);
    push_box([xlo, rhi[1], plo[2]], [xhi, phi[1], phi[2]]);

    let ylo = rlo[1];
    let yhi = rhi[1];
    push_box([xlo, ylo, plo[2]], [xhi, yhi, rlo[2]]);
    push_box([xlo, ylo, rhi[2]], [xhi, yhi, phi[2]]);

    (!pieces.is_empty()).then_some(pieces)
}
