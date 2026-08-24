use super::*;
use openrcad::foundation::{Trsf, Vec as GeomVec};
use openrcad::geom::{Curve, GeomCurve, GeomSurface};
use openrcad::topo::{Edge, Face, Orientation, Wire};
use std::collections::HashSet;

/// A cut's tool, tried in order, mirroring `JoinTool`: `smooth` is the analytic
/// cylinder for a circular pocket/drill (a clean round hole, not a 48-gon one);
/// `exact` is the faceted prism with the drawn dimensions; `expanded` is a
/// faceted fallback whose walls poke ~`CUT_WALL_GROW`mm past the body's faces to
/// dodge the coplanar-face case. `smooth` is `None` for non-circular profiles.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct CutTool {
    pub(crate) smooth: Option<KernelSolid>,
    pub(crate) exact: Option<KernelSolid>,
    #[serde(default)]
    pub(crate) exact_source: Option<ExactCutSource>,
    pub(crate) expanded: Option<KernelSolid>,
    #[serde(default)]
    pub(crate) expanded_source: Option<ExpandedCutSource>,
    // The same three tools swept the OPPOSITE direction. A cut is meant to remove
    // material; when the drawn direction sweeps into empty air (e.g. a positive
    // "pocket depth" on a top-face sketch, which `directional_cut` sends *up* away
    // from the body) it bites nothing and the op silently does nothing — the
    // reported "cut works once then never again". `apply_cut` falls back to these
    // when the drawn direction removes nothing from a body it should have cut.
    pub(crate) smooth_rev: Option<KernelSolid>,
    pub(crate) exact_rev: Option<KernelSolid>,
    #[serde(default)]
    pub(crate) exact_rev_source: Option<ExactCutSource>,
    pub(crate) expanded_rev: Option<KernelSolid>,
    #[serde(default)]
    pub(crate) expanded_rev_source: Option<ExpandedCutSource>,
    pub(crate) circle: Option<Circle>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct ExpandedCutSource {
    pub(crate) boundary: Vec<(f32, f32)>,
    pub(crate) holes: Vec<Vec<(f32, f32)>>,
    pub(crate) cs: CoordinateSystem,
    pub(crate) depth: f32,
}

/// Deferred analytic prism for either cut direction. Profile-aware through
/// cuts can consume this 2D source directly; general cuts build only the 3D
/// direction they actually need.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct ExactCutSource {
    pub(crate) region: crate::sketch::Region,
    pub(crate) cs: CoordinateSystem,
    pub(crate) depth: f32,
    pub(crate) arc_circles: Vec<((f32, f32), f32)>,
}

impl ExactCutSource {
    fn build(&self) -> Option<KernelSolid> {
        crate::mock_kernel::extruded_sketch_region_solid(
            &self.region,
            self.depth,
            &self.cs,
            &self.arc_circles,
        )
    }

    fn aabb(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for &(u, v) in &self.region.boundary {
            let base = self.cs.unproject(u, v);
            let end = base.add(self.cs.n.mul(self.depth));
            for point in [base, end] {
                for axis in 0..3 {
                    let value = [point.x, point.y, point.z][axis];
                    lo[axis] = lo[axis].min(value);
                    hi[axis] = hi[axis].max(value);
                }
            }
        }
        lo[0].is_finite().then_some((lo, hi))
    }
}

impl ExpandedCutSource {
    fn build(&self) -> Option<KernelSolid> {
        crate::mock_kernel::extruded_region_solid(&self.boundary, &self.holes, self.depth, &self.cs)
    }
}

impl CutTool {
    pub(crate) fn single_direction(
        smooth: Option<KernelSolid>,
        exact: Option<KernelSolid>,
        expanded: Option<KernelSolid>,
        circle: Option<Circle>,
    ) -> Self {
        Self {
            smooth,
            exact,
            exact_source: None,
            expanded,
            expanded_source: None,
            smooth_rev: None,
            exact_rev: None,
            exact_rev_source: None,
            expanded_rev: None,
            expanded_rev_source: None,
            circle,
        }
    }
}

/// Combine-style body cut: subtract the complete `tool` body from `target`.
/// The operation is atomic and requires a positive-volume overlap. A miss or
/// unchanged/failed boolean leaves both inputs untouched. On success the result
/// is the body produced by `node_id`; the tool is consumed unless `keep_tool` is
/// true. Face provenance still records which target/tool topology contributed.
pub(crate) fn apply_body_cut(
    node_id: &str,
    target: &str,
    tool: &str,
    keep_tool: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if target == tool {
        warnings.push(format!(
            "Body cut '{node_id}': target and cutting body must be different."
        ));
        return;
    }
    let Some(target_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "Body cut '{node_id}': target body '{target}' no longer exists."
        ));
        return;
    };
    let Some(tool_index) = live.iter().position(|body| body.id == tool) else {
        warnings.push(format!(
            "Body cut '{node_id}': cutting body '{tool}' no longer exists."
        ));
        return;
    };
    let target_body = live[target_index].clone();
    let tool_body = live[tool_index].clone();
    if target_body.parts.is_empty() || tool_body.parts.is_empty() {
        warnings.push(format!(
            "Body cut '{node_id}': both selected bodies need solid geometry."
        ));
        return;
    }

    let positive_overlap = target_body.parts.iter().any(|target_part| {
        tool_body
            .parts
            .iter()
            .any(|tool_part| solids_overlap_with_volume(target_part, tool_part))
    });
    if !positive_overlap {
        warnings.push(format!(
            "Body cut '{node_id}': the target and cutting body must overlap; \
             both original bodies were left unchanged."
        ));
        return;
    }

    // The exact-history path is unambiguous when one target part is cut by one
    // tool part. Preserve the target's existing face names and give generated
    // tool faces durable names while the boolean still has its lineage table.
    // Retaining this mesh also avoids tessellating the unchanged result again
    // on every checkpoint hit.
    let input_names: Option<Vec<Option<String>>> =
        match (&target_body.pristine, &target_body.parts[..]) {
            (Some(mesh), [part]) => Some(crate::mock_kernel::input_shell_face_names(mesh, part)),
            _ => None,
        };
    let owner_classes = input_names
        .as_ref()
        .map(|names| crate::mock_kernel::owner_classes_from_names(names));
    let exact_history_eligible = target_body.parts.len() == 1 && tool_body.parts.len() == 1;
    let mut result_parts = target_body.parts.clone();
    let mut cut_trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>> = None;
    let mut changed = false;
    for tool_part in &tool_body.parts {
        let mut next = Vec::new();
        for target_part in result_parts {
            if !solids_overlap_with_volume(&target_part, tool_part) {
                next.push(target_part);
                continue;
            }
            match crate::mock_kernel::difference_bodies_with_history(
                &target_part,
                tool_part,
                exact_history_eligible
                    .then_some(owner_classes.as_deref())
                    .flatten(),
            ) {
                Some(outcome) if cut_parts_changed(&target_part, &outcome.bodies) => {
                    changed = true;
                    if exact_history_eligible {
                        cut_trace = Some(outcome.face_history.clone());
                    }
                    next.extend(outcome.bodies);
                }
                _ => next.push(target_part),
            }
        }
        result_parts = next;
    }
    if !changed {
        warnings.push(format!(
            "Body cut '{node_id}': the solver could not subtract the overlapping body; \
             both original bodies were left unchanged."
        ));
        return;
    }

    let pristine = match (&cut_trace, &input_names, target_body.pristine.as_deref()) {
        (Some(histories), Some(names), Some(input_mesh)) => {
            crate::mock_kernel::propagate_face_names_via_body_histories(
                input_mesh,
                names,
                &result_parts,
                histories,
                node_id,
                &format!("cut:{node_id}"),
            )
            .map(std::sync::Arc::new)
        }
        _ => propagate_cut_face_names(
            &target_body.parts,
            target_body.pristine.as_deref(),
            &result_parts,
            node_id,
        )
        .map(std::sync::Arc::new),
    };

    let mut remove = vec![target_index];
    if !keep_tool {
        remove.push(tool_index);
    }
    remove.sort_unstable();
    remove.dedup();
    for index in remove.into_iter().rev() {
        live.remove(index);
    }
    if !result_parts.is_empty() {
        apply_new(
            live,
            LiveBody {
                id: node_id.into(),
                parts: result_parts,
                pristine,
                sketch_source: None,
            },
        );
    }
}

fn solids_overlap_with_volume(a: &KernelSolid, b: &KernelSolid) -> bool {
    let (Some((alo, ahi)), Some((blo, bhi))) = (
        crate::mock_kernel::solid_aabb(a),
        crate::mock_kernel::solid_aabb(b),
    ) else {
        return false;
    };
    (0..3).all(|axis| ahi[axis].min(bhi[axis]) - alo[axis].max(blo[axis]) > 1.0e-5)
}

pub(crate) fn recut_debug_enabled() -> bool {
    std::env::var_os("ZEROCAD_RECUT_DEBUG").is_some()
}

pub(crate) fn recut_debug(message: impl AsRef<str>) {
    if recut_debug_enabled() {
        eprintln!("[zerocad-recut] {}", message.as_ref());
    }
}

/// A cut tool that sweeps in the **drawn direction** (the sign of `depth`):
/// a negative depth cuts *into* the body the sketch sits on, a positive depth
/// sweeps *outward* from the sketch face. The tool starts one `CUT_OVERSHOOT`
/// behind the sketch plane and runs `|depth| + 2·CUT_OVERSHOOT` along the sweep
/// direction, so both end caps clear the body's faces — the near cap clears the
/// face the sketch sits on, and the far cap clears the back face when the cut
/// punches clean through. Returns `(start_plane, signed_sweep_depth)`.
pub(crate) fn directional_cut(cs: &CoordinateSystem, depth: f32) -> (CoordinateSystem, f32) {
    let sign = if depth < 0.0 { -1.0 } else { 1.0 };
    let start = cs.origin.add(cs.n.mul(-sign * CUT_OVERSHOOT));
    (
        CoordinateSystem::new(start, cs.u, cs.v),
        depth + sign * 2.0 * CUT_OVERSHOOT,
    )
}

/// Subtract one direction's tool variants from `part`, trying smooth → exact →
/// expanded (and their axis-aligned fallbacks). `None` if the tool's AABB misses
/// the part or the solver couldn't subtract it. All use the body-splitting
/// difference, so a cut that severs the part yields separate parts.
/// A successful one-direction cut: the severed parts, plus — when the general
/// boolean produced them — the kernel's exact face history and the pre-split
/// combined result its face indices refer to. Axis-aligned fallback paths
/// rebuild geometry without a boolean, so they carry no trace.
pub(crate) struct CutOutcome {
    pub(crate) parts: Vec<KernelSolid>,
    pub(crate) trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>>,
}

pub(crate) fn cut_part_one_dir(
    part: &KernelSolid,
    pbb: Option<&([f32; 3], [f32; 3])>,
    smooth: &Option<KernelSolid>,
    exact: &mut Option<KernelSolid>,
    exact_source: &Option<ExactCutSource>,
    expanded: &mut Option<KernelSolid>,
    expanded_source: &Option<ExpandedCutSource>,
    tbb: Option<&([f32; 3], [f32; 3])>,
    obj_classes: Option<&[Option<u64>]>,
) -> Option<CutOutcome> {
    let tbb = tbb?;
    let overlaps = pbb.is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, tbb, 0.05));
    if !overlaps {
        return None;
    }
    let changed_difference = |label: &str,
                              tool: &KernelSolid,
                              exact_reference: Option<&KernelSolid>| {
        let outcome = crate::mock_kernel::difference_bodies_with_history(part, tool, obj_classes)?;
        if cut_parts_changed(part, &outcome.bodies) {
            if label == "expanded" {
                let Some(exact_tool) = exact_reference else {
                    recut_debug("expanded cut candidate rejected: no exact reference tool");
                    return None;
                };
                match super::recovery_certificate::certify_expanded_cut(
                    part,
                    exact_tool,
                    tool,
                    &outcome.bodies,
                ) {
                    Ok(certificate) => recut_debug(format!(
                        "expanded cut recovery certified: {}",
                        certificate.summary()
                    )),
                    Err(error) => {
                        recut_debug(format!("expanded cut candidate rejected: {error}"));
                        return None;
                    }
                }
            }
            recut_debug(format!("cut variant '{label}' changed part"));
            Some(CutOutcome {
                parts: outcome.bodies,
                trace: Some(outcome.face_history),
            })
        } else {
            recut_debug(format!("cut variant '{label}' made no geometry change"));
            None
        }
    };
    if let Some(outcome) = smooth
        .as_ref()
        .and_then(|tool| changed_difference("smooth", tool, exact.as_ref()))
    {
        return Some(outcome);
    }
    if exact.is_none() {
        *exact = exact_source.as_ref().and_then(ExactCutSource::build);
    }
    if let Some(outcome) = exact
        .as_ref()
        .and_then(|tool| changed_difference("exact", tool, exact.as_ref()))
    {
        return Some(outcome);
    }
    if expanded.is_none() {
        *expanded = expanded_source.as_ref().and_then(ExpandedCutSource::build);
    }
    if let Some(outcome) = expanded
        .as_ref()
        .and_then(|tool| changed_difference("expanded", tool, exact.as_ref()))
    {
        return Some(outcome);
    }
    recut_debug("all cut variants failed or missed");
    None
}

fn cut_parts_changed(original: &KernelSolid, parts: &[KernelSolid]) -> bool {
    let before = solid_change_signature(original);
    let mut after_mesh = MockMesh::empty();
    for part in parts {
        after_mesh.append(MockMesh::from_solid(part));
    }
    let after = mesh_change_signature(&after_mesh);
    before != after
}

fn solid_change_signature(solid: &KernelSolid) -> (usize, usize, [i64; 6], [i64; 3]) {
    mesh_change_signature(&MockMesh::from_solid(solid))
}

fn mesh_change_signature(mesh: &MockMesh) -> (usize, usize, [i64; 6], [i64; 3]) {
    let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
    let mut lo = [i64::MAX; 3];
    let mut hi = [i64::MIN; 3];
    let mut sum = [0i64; 3];
    for v in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            let value = q(v[axis]);
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
            sum[axis] = sum[axis].wrapping_add(value);
        }
    }
    if mesh.vertices.is_empty() {
        lo = [0; 3];
        hi = [0; 3];
    }
    (
        mesh.vertices.len(),
        mesh.indices.len(),
        [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]],
        sum,
    )
}

pub(crate) fn cut_tool_bboxes(
    tool: &CutTool,
) -> (Option<([f32; 3], [f32; 3])>, Option<([f32; 3], [f32; 3])>) {
    let fwd_bb = tool
        .expanded
        .as_ref()
        .or(tool.exact.as_ref())
        .or(tool.smooth.as_ref())
        .and_then(crate::mock_kernel::solid_aabb)
        .or_else(|| tool.exact_source.as_ref().and_then(ExactCutSource::aabb));
    let rev_bb = tool
        .expanded_rev
        .as_ref()
        .or(tool.exact_rev.as_ref())
        .or(tool.smooth_rev.as_ref())
        .and_then(crate::mock_kernel::solid_aabb)
        .or_else(|| {
            tool.exact_rev_source
                .as_ref()
                .and_then(ExactCutSource::aabb)
        });
    (fwd_bb, rev_bb)
}

fn solid_projection_range(solid: &KernelSolid, cs: &CoordinateSystem) -> Option<(f32, f32)> {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for vertex in solid.vertices() {
        let point = vertex.point();
        let world =
            crate::geometry::Vec3::new(point.x() as f32, point.y() as f32, point.z() as f32);
        let height = world.sub(cs.origin).dot(cs.n);
        lo = lo.min(height);
        hi = hi.max(height);
    }
    (lo.is_finite() && hi.is_finite()).then_some((lo, hi))
}

fn exact_source_projection_range(
    source: &ExactCutSource,
    cs: &CoordinateSystem,
) -> Option<(f32, f32)> {
    let start = source.cs.origin.sub(cs.origin).dot(cs.n);
    let end = start + source.cs.n.dot(cs.n) * source.depth;
    (start.is_finite() && end.is_finite()).then_some((start.min(end), start.max(end)))
}

pub(crate) fn planar_section_height(face: &Face, cs: &CoordinateSystem) -> Option<f32> {
    let Some(GeomSurface::Plane(plane)) = face.surface() else {
        return None;
    };
    let normal = plane.normal();
    let alignment =
        (normal.x() as f32 * cs.n.x + normal.y() as f32 * cs.n.y + normal.z() as f32 * cs.n.z)
            .abs();
    if alignment < 0.9999 {
        return None;
    }
    let wire = face.outer_wire()?;
    let mut heights = wire.edges().into_iter().flat_map(|edge| {
        [edge.source().point(), edge.target().point()].map(|point| {
            crate::geometry::Vec3::new(point.x() as f32, point.y() as f32, point.z() as f32)
                .sub(cs.origin)
                .dot(cs.n)
        })
    });
    let first = heights.next()?;
    heights
        .try_fold((first, first), |(lo, hi), height| {
            let lo = lo.min(height);
            let hi = hi.max(height);
            ((hi - lo) <= 1.0e-4).then_some((lo, hi))
        })
        .map(|(lo, hi)| 0.5 * (lo + hi))
}

fn reverse_wire(wire: &Wire) -> Wire {
    let mut edges: Vec<Edge> = wire
        .edges()
        .into_iter()
        .map(|edge| edge.reversed())
        .collect();
    edges.reverse();
    Wire::from_edges(edges)
}

pub(crate) fn wire_signed_area(wire: &Wire, cs: &CoordinateSystem) -> f64 {
    let mut points = Vec::new();
    for edge in wire.edges() {
        let samples = if matches!(edge.curve(), Some(GeomCurve::Line(_))) {
            1
        } else {
            8
        };
        let (first, last) = if edge.orientation() == Orientation::Reversed {
            (edge.last(), edge.first())
        } else {
            (edge.first(), edge.last())
        };
        for sample in 0..samples {
            let fraction = sample as f64 / samples as f64;
            let point = edge
                .curve()
                .map(|curve| curve.point(first + (last - first) * fraction))
                .unwrap_or_else(|| edge.source().point());
            let projected = cs.project(crate::geometry::Vec3::new(
                point.x() as f32,
                point.y() as f32,
                point.z() as f32,
            ));
            points.push((f64::from(projected.0), f64::from(projected.1)));
        }
    }
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(&(ax, ay), &(bx, by))| ax * by - bx * ay)
        .sum::<f64>()
        * 0.5
}

pub(crate) fn orient_wire_like(
    wire: Wire,
    reference_sign: f64,
    same: bool,
    cs: &CoordinateSystem,
) -> Wire {
    let sign = wire_signed_area(&wire, cs).signum();
    let wanted = if same {
        reference_sign
    } else {
        -reference_sign
    };
    if sign != 0.0 && wanted != 0.0 && sign != wanted {
        reverse_wire(&wire)
    } else {
        wire
    }
}

pub(crate) fn wire_lies_in_region(
    wire: &Wire,
    region: &crate::sketch::Region,
    cs: &CoordinateSystem,
) -> bool {
    wire.edges().iter().all(|edge| {
        let samples = if matches!(edge.curve(), Some(GeomCurve::Line(_))) {
            2
        } else {
            9
        };
        (0..samples).all(|sample| {
            let fraction = sample as f64 / (samples - 1) as f64;
            let point = edge
                .curve()
                .map(|curve| curve.point(edge.first() + (edge.last() - edge.first()) * fraction))
                .unwrap_or_else(|| edge.start().point());
            let uv = cs.project(crate::geometry::Vec3::new(
                point.x() as f32,
                point.y() as f32,
                point.z() as f32,
            ));
            region.contains(uv)
        })
    })
}

/// Exact fast path for text engraved into one sketch prism. All disjoint glyph
/// profiles are combined in 2D, then the body is rebuilt once: either as a
/// through-cut prism or as two sewn depth sections for a blind pocket. This
/// avoids one fragile 3-D boolean per glyph while preserving counter islands.
/// Edge-crossing and non-parallel tools stay on the general boolean path.
fn try_prismatic_profile_cut(body: &LiveBody, tools: &mut [CutTool]) -> Option<Vec<KernelSolid>> {
    let [body_part] = body.parts.as_slice() else {
        return None;
    };
    let [body_source] = body.sketch_source.as_ref()?.regions.as_slice() else {
        return None;
    };
    let body_region = source_region(body_source)?;
    let body_range = solid_projection_range(body_part, &body_source.cs)?;
    let body_sign = body_source.depth.signum();
    let body_length = body_source.depth.abs();
    if body_sign == 0.0 || body_length <= 1.0e-5 {
        return None;
    }
    let body_cap = body_part
        .shell()
        .faces()
        .iter()
        .filter_map(|face| {
            planar_section_height(face, &body_source.cs).map(|height| (face, height))
        })
        .min_by(|(_, left), (_, right)| left.abs().total_cmp(&right.abs()))?
        .0
        .clone();
    let base_height = planar_section_height(&body_cap, &body_source.cs)?;
    let outer = body_cap.outer_wire()?;
    let outer_sign = wire_signed_area(&outer, &body_source.cs).signum();
    if outer_sign == 0.0 {
        return None;
    }
    // These wires already belong to this cap and therefore already carry the
    // winding convention expected by the kernel's prism builder. Preserve it.
    let mut main_holes: Vec<Wire> = body_cap.inner_wires();
    let mut islands = Vec::new();
    let mut floor_profiles = Vec::new();
    let mut common_cut_range: Option<(f32, f32)> = None;

    for tool in tools {
        let covers_body = |solid: &KernelSolid| {
            solid_projection_range(solid, &body_source.cs).is_some_and(|range| {
                range.0 <= body_range.0 + 1.0e-4 && range.1 >= body_range.1 - 1.0e-4
            })
        };
        let normalized_source_range = |source: &ExactCutSource| {
            if source.cs.n.dot(body_source.cs.n).abs() < 0.9999 {
                return None;
            }
            exact_source_projection_range(source, &body_source.cs).map(|(lo, hi)| {
                let a = lo * body_sign;
                let b = hi * body_sign;
                (a.min(b), a.max(b))
            })
        };
        let overlap_length =
            |range: (f32, f32)| (range.1.min(body_length) - range.0.max(0.0)).max(0.0);
        let direct_source = [tool.exact_source.as_ref(), tool.exact_rev_source.as_ref()]
            .into_iter()
            .flatten()
            .filter_map(|source| normalized_source_range(source).map(|range| (source, range)))
            .filter(|(_, range)| overlap_length(*range) > 1.0e-5)
            .max_by(|(_, left), (_, right)| {
                overlap_length(*left).total_cmp(&overlap_length(*right))
            });
        let direct_profile = direct_source.and_then(|source| {
            let (source, range) = source;
            let (outer, inners) =
                crate::mock_kernel::analytic_region_wires(&source.region, &source.cs)?;
            let height = source
                .cs
                .origin
                .sub(body_source.cs.origin)
                .dot(body_source.cs.n);
            Some((outer, inners, height, range))
        });
        let (tool_outer, tool_inners, tool_height, tool_range) =
            if let Some(profile) = direct_profile {
                profile
            } else {
                let forward = tool
                    .exact
                    .as_ref()
                    .or(tool.smooth.as_ref())
                    .filter(|solid| covers_body(solid));
                let covering = if let Some(forward) = forward {
                    forward
                } else if let Some(reverse) =
                    tool.smooth_rev.as_ref().filter(|solid| covers_body(solid))
                {
                    reverse
                } else {
                    if tool.exact_rev.is_none() {
                        tool.exact_rev = tool
                            .exact_rev_source
                            .as_ref()
                            .and_then(ExactCutSource::build);
                    }
                    tool.exact_rev.as_ref().filter(|solid| covers_body(solid))?
                };
                let covering_range = solid_projection_range(covering, &body_source.cs)?;
                let a = covering_range.0 * body_sign;
                let b = covering_range.1 * body_sign;
                let tool_range = (a.min(b), a.max(b));
                let (tool_cap, tool_height) = covering
                    .shell()
                    .faces()
                    .iter()
                    .filter_map(|face| {
                        planar_section_height(face, &body_source.cs)
                            .map(|height| (face.clone(), height))
                    })
                    .min_by(|(_, left), (_, right)| left.total_cmp(right))?;
                (
                    tool_cap.outer_wire()?,
                    tool_cap.inner_wires(),
                    tool_height,
                    tool_range,
                )
            };
        let clipped_range = (tool_range.0.max(0.0), tool_range.1.min(body_length));
        if clipped_range.1 - clipped_range.0 <= 1.0e-5 {
            return None;
        }
        if common_cut_range.is_some_and(|range| {
            (range.0 - clipped_range.0).abs() > 1.0e-4 || (range.1 - clipped_range.1).abs() > 1.0e-4
        }) {
            return None;
        }
        common_cut_range = Some(clipped_range);
        if !wire_lies_in_region(&tool_outer, &body_region, &body_source.cs) {
            return None;
        }
        let shift = body_source.cs.n.mul(base_height - tool_height);
        let transform = Trsf::translation(GeomVec::new(
            f64::from(shift.x),
            f64::from(shift.y),
            f64::from(shift.z),
        ));
        let base_outer = orient_wire_like(
            tool_outer.transformed(&transform),
            outer_sign,
            true,
            &body_source.cs,
        );
        let base_inners: Vec<Wire> = tool_inners
            .into_iter()
            .map(|counter| {
                orient_wire_like(
                    counter.transformed(&transform),
                    outer_sign,
                    true,
                    &body_source.cs,
                )
            })
            .collect();
        main_holes.push(base_outer.clone());
        islands.extend(base_inners.iter().cloned());
        floor_profiles.push((base_outer, base_inners));
    }

    let cut_range = common_cut_range?;
    let surface = body_cap.surface().cloned()?;
    const SECTION_TOLERANCE: f32 = 1.0e-4;
    let through =
        cut_range.0 <= SECTION_TOLERANCE && cut_range.1 >= body_length - SECTION_TOLERANCE;
    if !through {
        // The exact sectional reconstruction currently covers the normal face
        // workflow: a blind pocket entering from the prism's far cap. A sketch
        // on the source/base cap remains on the general cut path.
        if cut_range.1 < body_length - SECTION_TOLERANCE || cut_range.0 <= SECTION_TOLERANCE {
            return None;
        }
        let transition_depth = body_sign * cut_range.0;
        let upper_depth = body_source.depth - transition_depth;
        let lower = crate::mock_kernel::extruded_sketch_region_solid(
            &body_region,
            transition_depth,
            &body_source.cs,
            &[],
        )?;
        let shift = body_source.cs.n.mul(transition_depth - base_height);
        let transform = Trsf::translation(GeomVec::new(
            f64::from(shift.x),
            f64::from(shift.y),
            f64::from(shift.z),
        ));
        // Sweep the upper section on the body's original cap, where every glyph
        // wire already shares the cap's exact support plane, then translate the
        // completed section to the pocket floor as one coherent solid.
        let upper_profile = Face::with_wires(
            Some(surface.clone()),
            Some(outer),
            main_holes,
            body_cap.orientation(),
        );
        let upper_sweep = GeomVec::new(
            f64::from(body_source.cs.n.x * upper_depth),
            f64::from(body_source.cs.n.y * upper_depth),
            f64::from(body_source.cs.n.z * upper_depth),
        );
        let upper_main = match openrcad::algo::prism::prism_operation(&upper_profile, upper_sweep) {
            Ok(result) => result.value.transformed(&transform),
            Err(error) => {
                recut_debug(format!("sectional profile prism failed: {error:?}"));
                return None;
            }
        };
        let mut upper_islands = Vec::new();
        for island in &islands {
            let island_face = Face::with_wires(
                Some(surface.clone()),
                Some(island.clone()),
                Vec::new(),
                body_cap.orientation(),
            );
            upper_islands.push(
                openrcad::algo::prism::prism_operation(&island_face, upper_sweep)
                    .ok()?
                    .value
                    .transformed(&transform),
            );
        }

        let on_transition = |face: &Face| {
            planar_section_height(face, &body_source.cs)
                .is_some_and(|height| (height - transition_depth).abs() <= SECTION_TOLERANCE)
        };
        let mut faces: Vec<Face> = lower
            .shell()
            .faces()
            .iter()
            .filter(|face| !on_transition(face))
            .cloned()
            .collect();
        faces.extend(
            upper_main
                .shell()
                .faces()
                .iter()
                .filter(|face| !on_transition(face))
                .cloned(),
        );
        for island in &upper_islands {
            faces.extend(
                island
                    .shell()
                    .faces()
                    .iter()
                    .filter(|face| !on_transition(face))
                    .cloned(),
            );
        }
        for (floor_outer, floor_inners) in floor_profiles {
            let floor_source = Face::with_wires(
                Some(surface.clone()),
                Some(floor_outer),
                floor_inners,
                body_cap.orientation(),
            );
            let floor_solid = openrcad::algo::prism::prism_operation(&floor_source, upper_sweep)
                .ok()?
                .value
                .transformed(&transform);
            let floor_cap = floor_solid
                .shell()
                .faces()
                .iter()
                .find(|face| on_transition(face))?
                .clone();
            faces.push(floor_cap.reversed());
        }
        let shell = openrcad::algo::sew_with_policy(
            &faces,
            &openrcad::foundation::TolerancePolicy::STANDARD,
        )
        .ok()?
        .value;
        // Curved and circular profile families can make sewing choose opposite
        // stored plane senses for otherwise equivalent pocket floors. Correct
        // only floors whose EFFECTIVE normal points into the remaining material;
        // floors that already face the cavity must stay untouched. Reverse the
        // complete loop together with the face so shared-edge traversal remains
        // unchanged and the sewn topology stays strict-valid.
        let reversed_floor_ids: Vec<_> = shell
            .faces()
            .into_iter()
            .filter(|face| on_transition(face))
            .filter_map(|face| {
                let GeomSurface::Plane(plane) = face.surface()? else {
                    return None;
                };
                let normal = plane.normal();
                let alignment = (normal.x() as f32 * body_source.cs.n.x
                    + normal.y() as f32 * body_source.cs.n.y
                    + normal.z() as f32 * body_source.cs.n.z)
                    * body_sign
                    * if face.orientation() == Orientation::Reversed {
                        -1.0
                    } else {
                        1.0
                    };
                (alignment < 0.0).then_some(face.id())
            })
            .collect();
        let rebuilt = if reversed_floor_ids.is_empty() {
            KernelSolid::new(shell)
        } else {
            let shell_id = shell.id();
            let mut corrected_brep = shell.brep().as_ref().clone();
            for face_id in reversed_floor_ids {
                let loop_ids = {
                    let face = corrected_brep.faces.get_mut(face_id)?;
                    face.orientation = face.orientation.reversed();
                    face.outer_wire
                        .into_iter()
                        .chain(face.inner_wires.iter().copied())
                        .collect::<Vec<_>>()
                };
                for loop_id in loop_ids {
                    let loop_data = corrected_brep.loops.get_mut(loop_id)?;
                    loop_data.edges.reverse();
                    for coedge in &mut loop_data.edges {
                        coedge.orientation = coedge.orientation.reversed();
                    }
                }
            }
            KernelSolid::new(openrcad::topo::Shell::from_id(
                std::sync::Arc::new(corrected_brep),
                shell_id,
            ))
        };
        return (rebuilt.is_watertight()
            && rebuilt.health_report().is_healthy()
            && rebuilt.validate().is_ok()
            && rebuilt
                .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
                .is_ok())
        .then_some(vec![rebuilt]);
    }

    let profile = Face::with_wires(
        Some(surface.clone()),
        Some(outer),
        main_holes,
        body_cap.orientation(),
    );
    let sweep = GeomVec::new(
        f64::from(body_source.cs.n.x * body_source.depth),
        f64::from(body_source.cs.n.y * body_source.depth),
        f64::from(body_source.cs.n.z * body_source.depth),
    );
    let main = openrcad::algo::prism::prism_operation(&profile, sweep)
        .ok()?
        .value;
    let mut parts = vec![main];
    for island in islands {
        let island_face = Face::with_wires(
            Some(surface.clone()),
            Some(island),
            Vec::new(),
            body_cap.orientation(),
        );
        parts.push(
            openrcad::algo::prism::prism_operation(&island_face, sweep)
                .ok()?
                .value,
        );
    }
    Some(parts)
}

/// Apply a Cut extrude: subtract each tool from every body part whose AABB it
/// overlaps. For each part it tries the **drawn** direction first (smooth → exact
/// → expanded), then falls back to the **opposite** sweep when the drawn one
/// removes nothing — so a cut drawn away from the body (a positive pocket depth
/// on a top face) still bites instead of silently doing nothing. A solver failure
/// on a body the tool genuinely overlaps leaves the part intact (safer than
/// dropping a valid body) and warns; a part fully consumed by the cut is removed.
/// `draft` is reserved for preview-quality kernel settings; it never changes
/// construction history or invokes a feature-specific replay path.
pub(crate) fn apply_cut(
    live: &mut [LiveBody],
    extrude_id: &str,
    mut tools: Vec<CutTool>,
    boolean_target: Option<&str>,
    _draft: bool,
    warnings: &mut Vec<String>,
) {
    // A word cut through a simple sketch prism is a planar profile operation,
    // not five unrelated 3-D intersection problems. Resolve all such tools in
    // one exact prism rebuild before entering the per-tool boolean fallback.
    let mut rebuilt_bodies = HashSet::new();
    if tools.len() > 1 {
        for body in live.iter_mut() {
            if boolean_target.is_some_and(|target| target != body.id) {
                continue;
            }
            let before_parts = body.parts.clone();
            let before_pristine = body.pristine.clone();
            let Some(parts) = try_prismatic_profile_cut(body, &mut tools) else {
                continue;
            };
            body.pristine = propagate_cut_face_names(
                &before_parts,
                before_pristine.as_deref(),
                &parts,
                &body.id,
            )
            .map(std::sync::Arc::new);
            body.parts = parts;
            body.sketch_source = None;
            rebuilt_bodies.insert(body.id.clone());
        }
    }

    for tool in &mut tools {
        // Pre-test bbox per direction (expanded ⊇ exact ⊇ smooth).
        let (fwd_bb, rev_bb) = cut_tool_bboxes(tool);
        if fwd_bb.is_none() && rev_bb.is_none() {
            continue;
        }
        // Did the solver fail to subtract this tool from a body it actually
        // overlapped (either direction)? If so the body keeps material the user
        // meant to remove.
        let mut failed_on_overlap = false;
        for body in live.iter_mut() {
            if rebuilt_bodies.contains(&body.id) {
                continue;
            }
            // Named targeting: only the pinned body is cut (legacy `None`
            // keeps the historical every-overlapping-body behavior).
            if boolean_target.is_some_and(|t| t != body.id) {
                continue;
            }
            let before_parts = body.parts.clone();
            let before_pristine = body.pristine.clone();
            // Exact-history plumbing for the single-part common case: the input
            // body's face names per shell-face position, and the derived owner
            // classes the kernel's owner-aware merge respects.
            let input_names: Option<Vec<Option<String>>> =
                match (&before_pristine, &before_parts[..]) {
                    (Some(mesh), [part]) => {
                        Some(crate::mock_kernel::input_shell_face_names(mesh, part))
                    }
                    _ => None,
                };
            let owner_classes: Option<Vec<Option<u64>>> = input_names
                .as_ref()
                .map(|names| crate::mock_kernel::owner_classes_from_names(names));
            let mut cut_trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>> = None;
            let mut changed = false;
            let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
            for part in body.parts.drain(..) {
                let pbb = crate::mock_kernel::solid_aabb(&part);
                let overlaps_dir = |tbb: Option<&([f32; 3], [f32; 3])>| {
                    tbb.is_some_and(|t| {
                        pbb.as_ref()
                            .is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, t, 0.05))
                    })
                };
                // How much of this part the tool's AABB encloses, per direction.
                // The drawn direction's `CUT_OVERSHOOT` dips ~0.1mm past the sketch
                // plane, so a cut aimed *away* from the body still nicks a sliver and
                // would count as "done"; ordering by overlap volume instead sends the
                // cut the way the body actually lies (a deep pocket beats a sliver).
                let overlap_vol = |tbb: Option<&([f32; 3], [f32; 3])>| -> f32 {
                    match (pbb.as_ref(), tbb) {
                        (Some(p), Some(t)) => (0..3)
                            .map(|i| (p.1[i].min(t.1[i]) - p.0[i].max(t.0[i])).max(0.0))
                            .product(),
                        _ => 0.0,
                    }
                };
                let reverse_first = overlap_vol(rev_bb.as_ref()) > overlap_vol(fwd_bb.as_ref());
                let mut cut_direction = |reverse: bool| {
                    if reverse {
                        cut_part_one_dir(
                            &part,
                            pbb.as_ref(),
                            &tool.smooth_rev,
                            &mut tool.exact_rev,
                            &tool.exact_rev_source,
                            &mut tool.expanded_rev,
                            &tool.expanded_rev_source,
                            rev_bb.as_ref(),
                            owner_classes.as_deref(),
                        )
                    } else {
                        cut_part_one_dir(
                            &part,
                            pbb.as_ref(),
                            &tool.smooth,
                            &mut tool.exact,
                            &tool.exact_source,
                            &mut tool.expanded,
                            &tool.expanded_source,
                            fwd_bb.as_ref(),
                            owner_classes.as_deref(),
                        )
                    }
                };
                let cut_parts =
                    cut_direction(reverse_first).or_else(|| cut_direction(!reverse_first));
                match cut_parts {
                    Some(outcome) => {
                        changed = true;
                        cut_trace = outcome.trace;
                        next.extend(outcome.parts);
                    }
                    None => {
                        // Only a genuine solver failure (the tool overlapped this
                        // part in some direction) warrants the warning — a tool that
                        // simply misses the body is normal once both directions are
                        // tried.
                        if overlaps_dir(fwd_bb.as_ref()) || overlaps_dir(rev_bb.as_ref()) {
                            failed_on_overlap = true;
                        }
                        next.push(part);
                    }
                }
            }
            body.parts = next;
            if changed {
                let next_source = body.sketch_source.as_ref().and_then(|source| {
                    tool.circle
                        .and_then(|circle| sketch_source_after_circle_cut(source, circle))
                });
                // Propagate the input body's face names through the cut so a captured
                // face survives the boolean (Phase 3). When the kernel emitted an
                // exact history (general-boolean path, single named input part),
                // names ride the history — MODIFIED faces inherit their owner and
                // the cut's GENERATED walls get durable `cut:{node}:tool-face:{i}`
                // names. Otherwise the geometric matcher covers it; any mismatch
                // falls back to `None` (re-tessellation), never a wrong name.
                body.pristine = match (&cut_trace, &input_names) {
                    (Some(histories), Some(names))
                        if before_parts.len() == 1 && before_pristine.is_some() =>
                    {
                        crate::mock_kernel::propagate_face_names_via_body_histories(
                            before_pristine.as_ref().unwrap(),
                            names,
                            &body.parts,
                            histories,
                            &body.id,
                            &format!("cut:{extrude_id}"),
                        )
                        .map(std::sync::Arc::new)
                    }
                    _ => propagate_cut_face_names(
                        &before_parts,
                        before_pristine.as_deref(),
                        &body.parts,
                        &body.id,
                    )
                    .map(std::sync::Arc::new),
                };
                body.sketch_source = next_source;
            }
        }
        if failed_on_overlap {
            warnings.push(format!(
                "Cut '{extrude_id}': the solver couldn't subtract the tool from a \
                 body it overlaps, so that material was left intact. Try nudging \
                 the sketch off the coplanar face."
            ));
        }
    }
}

/// Build a name-propagated pristine mesh for a cut result, or `None` to fall back
/// to plain re-tessellation. Gated to one-part-in / one-part-out with a named input
/// mesh — the common case — so a captured face reattaches through the cut while
/// severing/multi-part cuts degrade safely to today's behavior.
fn propagate_cut_face_names(
    before_parts: &[KernelSolid],
    before_pristine: Option<&MockMesh>,
    after_parts: &[KernelSolid],
    body_id: &str,
) -> Option<MockMesh> {
    // Propagate from a single named input body; the cut may still SEVER it into
    // several lumps, and each lump inherits the names of the faces it continues
    // (mesh-to-mesh matching is per-part, so multi-part output is fine).
    if before_parts.len() != 1 || after_parts.is_empty() {
        return None;
    }
    let input_mesh = before_pristine?;
    let mut mesh = MockMesh::empty();
    for part in after_parts {
        mesh.append(crate::mock_kernel::propagate_face_names(
            input_mesh, part, body_id,
        ));
    }
    Some(mesh)
}

#[cfg(test)]
mod through_cut_tests {
    use super::*;

    fn one_region(curves: &SketchCurves) -> Region {
        let mut regions = crate::sketch::detect_regions_analytic(curves).expect("arrangement");
        assert_eq!(regions.len(), 1);
        regions.pop().unwrap()
    }

    #[test]
    fn multiple_through_profiles_rebuild_one_prism_and_preserve_counter_islands() {
        let mut plate_curves = SketchCurves::new();
        plate_curves.add_rectangle((-10.0, -5.0), (10.0, 5.0));
        let plate_region = one_region(&plate_curves);
        let depth = 2.0;
        let plate = crate::mock_kernel::extruded_sketch_region_solid(
            &plate_region,
            depth,
            &CoordinateSystem::XY,
            &[],
        )
        .expect("plate prism");
        let body = LiveBody {
            id: "plate".into(),
            parts: vec![plate],
            pristine: None,
            sketch_source: Some(SketchExtrudeSource {
                regions: vec![SketchExtrudeRegionSource {
                    boundary: plate_region.boundary.clone(),
                    holes: plate_region.holes.clone(),
                    depth,
                    cs: CoordinateSystem::XY,
                    rect_circle: None,
                    analytic: plate_region.analytic.clone(),
                }],
            }),
        };

        let mut ring_curves = SketchCurves::new();
        ring_curves.add_circle((-4.0, 0.0), 2.0);
        ring_curves.add_circle((-4.0, 0.0), 0.8);
        let ring_regions = crate::sketch::detect_regions_analytic(&ring_curves).expect("ring");
        let ring = ring_regions
            .into_iter()
            .find(|region| region.holes.len() == 1)
            .expect("annular region");
        let mut disc_curves = SketchCurves::new();
        disc_curves.add_circle((4.0, 0.0), 1.5);
        let disc = one_region(&disc_curves);
        let tool_cs = CoordinateSystem::XY.with_origin(crate::geometry::Vec3::new(0.0, 0.0, -0.1));
        let tool_depth = depth + 0.2;
        let mut tools = [&ring, &disc]
            .into_iter()
            .map(|region| {
                let solid = crate::mock_kernel::extruded_sketch_region_solid(
                    region,
                    tool_depth,
                    &tool_cs,
                    &[],
                )
                .expect("through tool");
                CutTool::single_direction(None, Some(solid), None, None)
            })
            .collect::<Vec<_>>();

        let rebuilt = try_prismatic_profile_cut(&body, &mut tools).expect("profile regularization");
        assert_eq!(
            rebuilt.len(),
            2,
            "the annulus counter must remain as a detached material island"
        );
        let part_volumes: Vec<f64> = rebuilt.iter().filter_map(solid_volume).collect();
        let actual: f64 = part_volumes.iter().sum();
        let expected = f64::from((plate_region.area - ring.area - disc.area) * depth);
        assert!(
            (actual - expected).abs() <= expected * 1.0e-3,
            "through-cut volume mismatch: expected {expected}, got {actual} from {part_volumes:?}"
        );
    }

    #[test]
    fn multiple_blind_profiles_sew_one_exact_pocket_with_a_counter() {
        let mut plate_curves = SketchCurves::new();
        plate_curves.add_rectangle((-10.0, -5.0), (10.0, 5.0));
        let plate_region = one_region(&plate_curves);
        let depth = 2.0;
        let plate = crate::mock_kernel::extruded_sketch_region_solid(
            &plate_region,
            depth,
            &CoordinateSystem::XY,
            &[],
        )
        .expect("plate prism");
        let body = LiveBody {
            id: "plate".into(),
            parts: vec![plate],
            pristine: None,
            sketch_source: Some(SketchExtrudeSource {
                regions: vec![SketchExtrudeRegionSource {
                    boundary: plate_region.boundary.clone(),
                    holes: plate_region.holes.clone(),
                    depth,
                    cs: CoordinateSystem::XY,
                    rect_circle: None,
                    analytic: plate_region.analytic.clone(),
                }],
            }),
        };

        let mut ring_curves = SketchCurves::new();
        ring_curves.add_circle((-4.0, 0.0), 2.0);
        ring_curves.add_circle((-4.0, 0.0), 0.8);
        let ring = crate::sketch::detect_regions_analytic(&ring_curves)
            .expect("ring")
            .into_iter()
            .find(|region| region.holes.len() == 1)
            .expect("annular region");
        let mut disc_curves = SketchCurves::new();
        disc_curves.add_circle((4.0, 0.0), 1.5);
        let disc = one_region(&disc_curves);
        let sketch_cs =
            CoordinateSystem::XY.with_origin(crate::geometry::Vec3::new(0.0, 0.0, depth));
        let nominal_pocket_depth = -0.8;
        let (cut_cs, cut_depth) = directional_cut(&sketch_cs, nominal_pocket_depth);
        let mut tools = [&ring, &disc]
            .into_iter()
            .map(|region| {
                let mut tool = CutTool::single_direction(None, None, None, None);
                tool.exact_source = Some(ExactCutSource {
                    region: region.clone(),
                    cs: cut_cs,
                    depth: cut_depth,
                    arc_circles: Vec::new(),
                });
                tool
            })
            .collect::<Vec<_>>();

        let rebuilt =
            try_prismatic_profile_cut(&body, &mut tools).expect("blind profile regularization");
        assert_eq!(rebuilt.len(), 1, "a blind pocket stays one connected solid");
        assert!(rebuilt[0].is_watertight());
        assert!(rebuilt[0].health_report().is_healthy());
        assert!(rebuilt[0].validate().is_ok());
        let actual = solid_volume(&rebuilt[0]).expect("blind pocket volume");
        let removed_depth = nominal_pocket_depth.abs() + CUT_OVERSHOOT;
        let expected =
            f64::from(plate_region.area * depth - (ring.area + disc.area) * removed_depth);
        assert!(
            (actual - expected).abs() <= expected * 1.0e-3,
            "blind-cut volume mismatch: expected {expected}, got {actual}"
        );
    }
}
