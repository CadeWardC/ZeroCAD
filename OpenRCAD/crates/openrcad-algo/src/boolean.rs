use crate::BooleanOp;
use core::f64::consts::TAU;
use openrcad_foundation::{Dir, Pnt, Trsf, Vec as GeomVec};
use openrcad_geom::{Circle, Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::arena::EdgeId;
use openrcad_topo::{BRepBuilder, Face, FaceId, HealthReport, Solid, Wire};
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::bvh::Bvh;
use crate::sew::sew;

/// Which input face a boolean-result face came from. `usize` is the face's
/// position in the operand's `shell().faces()` — the one face key that is
/// stable across the kernel boundary (tessellation assigns mesh face ids by
/// the same enumeration).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BooleanFaceSource {
    /// Position of the source face in the object's shell.
    Object(usize),
    /// Position of the source face in the tool's shell.
    Tool(usize),
}

/// Exact face correspondence emitted by the boolean pipeline itself (OCCT
/// `Modified`/`Generated` semantics, resolved to final result faces):
/// `face_source[i]` names the input face that result face `i` (position in the
/// result's `shell().faces()`) descends from. An `Object`-sourced face is a
/// *Modified* survivor/split of that object face; in a Cut, a `Tool`-sourced
/// face is a wall *Generated* by that tool face. An input face that appears in
/// no entry was *Deleted*. `None` means the pipeline could not attribute the
/// face (the caller should fall back to geometric matching, never guess).
///
/// Unlike a post-hoc surface-signature matcher, this correspondence is derived
/// from the split bookkeeping the boolean already performs internally
/// (`obj_sub`/`tool_sub` parent→child maps), so two same-plane faces with
/// different owners resolve by their true imprint boundaries.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct BooleanFaceHistory {
    pub face_source: Vec<Option<BooleanFaceSource>>,
}

impl BooleanFaceHistory {
    /// Source of result face at shell position `i`.
    pub fn source_of(&self, i: usize) -> Option<BooleanFaceSource> {
        self.face_source.get(i).copied().flatten()
    }
}

/// Which operand failed preflight validation for a boolean operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanInput {
    /// The solid being operated on.
    Object,
    /// The tool solid.
    Tool,
}

/// Structured boolean failure for applications that need recoverable modeling.
#[derive(Clone, Debug, PartialEq)]
pub enum BooleanError {
    /// One of the input solids is structurally invalid or not watertight.
    InvalidInput {
        /// The invalid operand.
        input: BooleanInput,
        /// Topology health diagnostics for that operand.
        report: HealthReport,
    },
    /// The boolean implementation panicked before it could produce a result.
    Panicked,
    /// The resulting solid failed structural health checks.
    InvalidOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
    /// The resulting solid is structurally valid but has open/free boundary
    /// edges, so it is unsafe to treat as a closed CAD solid.
    NonWatertightOutput {
        /// Topology health diagnostics for the output.
        report: HealthReport,
    },
    /// The result is topologically watertight but contains a degenerate "sliver":
    /// two near-coincident, overlapping parallel faces forming a near-zero-
    /// thickness wall. This passes the manifold/health checks yet is a tiny
    /// geometric lie that breaks downstream operations (notably filleting), so it
    /// is rejected up front rather than cached as a bad body.
    DegenerateSliver {
        /// The wall thickness detected (distance between the coincident planes).
        thickness: f64,
    },
}

impl core::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidInput { input, report } => {
                write!(f, "invalid boolean {input:?} input: {report:?}")
            }
            Self::Panicked => write!(f, "boolean operation panicked"),
            Self::InvalidOutput { report } => {
                write!(f, "boolean produced invalid topology: {report:?}")
            }
            Self::NonWatertightOutput { report } => {
                write!(f, "boolean produced a non-watertight solid: {report:?}")
            }
            Self::DegenerateSliver { thickness } => {
                write!(
                    f,
                    "boolean produced a degenerate sliver (wall {thickness:.2e})"
                )
            }
        }
    }
}

impl std::error::Error for BooleanError {}

/// Apply `op` and reject invalid inputs, panics, or unhealthy/non-watertight
/// outputs. This is the preferred entry point for CAD applications, where a
/// failed feature should be diagnosable instead of cached as a bad body.
pub fn boolean_checked(object: &Solid, tool: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    validate_operand(BooleanInput::Object, object)?;
    validate_operand(BooleanInput::Tool, tool)?;

    let result = catch_unwind(AssertUnwindSafe(|| boolean(object, tool, op)))
        .map_err(|_| BooleanError::Panicked)?;
    validate_output(result)
}

/// Apply `op`, then split a result that severed the body into one solid per
/// connected component.
///
/// A cut that slices a body in two (or a fuse of two bodies that don't actually
/// touch) is returned by [`boolean`] as a single shell holding several disjoint
/// pieces — topologically valid, but really multiple bodies. This is the
/// multi-body entry point: it returns each connected component separately via
/// [`Solid::split_disconnected`]. For the common case where the result is one
/// connected body, the returned vector has a single element.
pub fn boolean_bodies(object: &Solid, tool: &Solid, op: BooleanOp) -> Vec<Solid> {
    boolean(object, tool, op).split_disconnected()
}

/// Checked multi-body boolean: like [`boolean_checked`], but splits a severed
/// result into separate bodies and validates **each** one. Fails if any body is
/// unhealthy or non-watertight, so a half-formed sliver can't slip through.
pub fn boolean_checked_bodies(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
) -> Result<Vec<Solid>, BooleanError> {
    validate_operand(BooleanInput::Object, object)?;
    validate_operand(BooleanInput::Tool, tool)?;

    let result = catch_unwind(AssertUnwindSafe(|| boolean(object, tool, op)))
        .map_err(|_| BooleanError::Panicked)?;
    let bodies = result.split_disconnected();
    for body in &bodies {
        let report = body.health_report();
        if !report.is_healthy() {
            return Err(BooleanError::InvalidOutput { report });
        }
        if !body.is_watertight() {
            return Err(BooleanError::NonWatertightOutput { report });
        }
    }
    Ok(bodies)
}

/// Apply `op` between `object` and `tool`.
pub fn boolean(object: &Solid, tool: &Solid, op: BooleanOp) -> Solid {
    boolean_impl(object, tool, op, None, None, false).0
}

/// [`boolean`] plus the exact face correspondence ([`BooleanFaceHistory`]).
///
/// `obj_classes` / `tool_classes` optionally give each input face (by shell
/// position) an **owner class**: the coplanar/cocylindrical merge passes then
/// refuse to combine result faces descending from different classes, so a
/// caller's face identities survive the merge (the Bidarra owner-aware-merge
/// rule). `None` classes ⇒ merges behave exactly as [`boolean`].
pub fn boolean_with_history(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    obj_classes: Option<&[Option<u64>]>,
    tool_classes: Option<&[Option<u64>]>,
) -> (Solid, BooleanFaceHistory) {
    let (solid, history) = boolean_impl(object, tool, op, obj_classes, tool_classes, true);
    (solid, history.unwrap_or_default())
}

/// Checked variant of [`boolean_with_history`] — same validation as
/// [`boolean_checked`].
pub fn boolean_checked_with_history(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    obj_classes: Option<&[Option<u64>]>,
    tool_classes: Option<&[Option<u64>]>,
) -> Result<(Solid, BooleanFaceHistory), BooleanError> {
    validate_operand(BooleanInput::Object, object)?;
    validate_operand(BooleanInput::Tool, tool)?;

    let (result, history) = catch_unwind(AssertUnwindSafe(|| {
        boolean_with_history(object, tool, op, obj_classes, tool_classes)
    }))
    .map_err(|_| BooleanError::Panicked)?;
    let result = validate_output(result)?;
    Ok((result, history))
}

fn boolean_impl(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
    obj_classes: Option<&[Option<u64>]>,
    tool_classes: Option<&[Option<u64>]>,
    want_history: bool,
) -> (Solid, Option<BooleanFaceHistory>) {
    let tol = 1e-5;

    // 0. Fuzzy pre-snap: nudge the tool so a near-coincident, overlapping planar
    //    face becomes exactly coincident with the object's. This collapses the
    //    near-miss / flush cut onto the boolean's clean coincident-face path,
    //    preventing the watertight "sliver" that would otherwise break a later
    //    fillet. Narrow and size-relative, so a real clearance is never snapped.
    let fuzz = (bbox_diag(object) * SNAP_REL).clamp(1e-12, SNAP_CAP);
    let snapped = snap_tool_to_object(object, tool, fuzz);
    let tool = &snapped;

    // 1. Build staging builders
    let mut builder_obj = BRepBuilder::from_brep((**object.brep()).clone());
    let mut builder_tool = BRepBuilder::from_brep((**tool.brep()).clone());

    // 2. Perform intersection and splitting
    let faces_obj = object.shell().faces();
    let faces_tool = tool.shell().faces();
    let bvh_obj = Bvh::build(&faces_obj);
    let bvh_tool = Bvh::build(&faces_tool);
    let pairs = Bvh::overlapping_pairs(&bvh_obj, &bvh_tool);

    // A. Split all boundary edges at mutual intersection points
    for &(f_obj_id, f_tool_id) in &pairs {
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        for w_obj in f_obj.wires() {
            for e_obj in w_obj.edges() {
                for w_tool in f_tool.wires() {
                    for e_tool in w_tool.edges() {
                        if let (Some(c_obj), Some(c_tool)) = (e_obj.curve(), e_tool.curve()) {
                            let pts = crate::intersect::curve_curve(c_obj, c_tool, tol);
                            for pt in pts {
                                try_split_edge(
                                    &mut builder_obj,
                                    e_obj.id(),
                                    c_obj,
                                    e_obj.first(),
                                    e_obj.last(),
                                    &pt,
                                    tol,
                                );
                                try_split_edge(
                                    &mut builder_tool,
                                    e_tool.id(),
                                    c_tool,
                                    e_tool.first(),
                                    e_tool.last(),
                                    &pt,
                                    tol,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    // B. Split faces along intersection curves.
    //
    // Each original face is split only along the curves arising from the tool
    // faces it actually overlaps, and only its own descendant sub-faces are
    // re-split — never the whole builder. This keeps the work proportional to
    // the real intersections instead of exploding combinatorially across pairs.
    let mut obj_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_obj.iter().map(|f| (f.id(), vec![f.id()])).collect();
    let mut tool_sub: std::collections::HashMap<FaceId, Vec<FaceId>> =
        faces_tool.iter().map(|f| (f.id(), vec![f.id()])).collect();

    let mut splitting_edges_obj: std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    > = std::collections::HashMap::new();
    let mut splitting_edges_tool: std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    > = std::collections::HashMap::new();

    for &(f_obj_id, f_tool_id) in &pairs {
        let f_obj = Face::from_id(
            object.brep().clone(),
            f_obj_id,
            object.brep().faces[f_obj_id].orientation,
        );
        let f_tool = Face::from_id(
            tool.brep().clone(),
            f_tool_id,
            tool.brep().faces[f_tool_id].orientation,
        );

        let s_obj = f_obj.surface().unwrap();
        let s_tool = f_tool.surface().unwrap();

        if surfaces_are_coplanar(s_obj, s_tool, tol) {
            // Coplanar faces: split f_obj along f_tool's boundary edges, and
            // vice versa — restricted to each face's own descendants.
            for w_tool in f_tool.wires() {
                // A coplanar circular cap (cylinder rim) sitting fully inside
                // f_obj — the cylinder-boss case. Imprint the *whole* circle once
                // so the imprint engine bores a clean hole (its closed-curve path),
                // instead of three arcs it can't close into a hole.
                if let Some(circle) = wire_full_circle(&w_tool, tol) {
                    if circle_inside_face(&circle, &f_obj) {
                        let c = GeomCurve::Circle(circle);
                        let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                        split_tracked(
                            &mut builder_obj,
                            sub,
                            &c,
                            0.0,
                            TAU,
                            &mut splitting_edges_obj,
                            tol,
                        );
                        continue;
                    }
                }
                for e_tool in w_tool.edges() {
                    if let Some(c_tool) = e_tool.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(
                            c_tool,
                            e_tool.first(),
                            e_tool.last(),
                            &f_obj,
                            tol,
                        );
                        for (first, last) in intervals {
                            let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                            split_tracked(
                                &mut builder_obj,
                                sub,
                                c_tool,
                                first,
                                last,
                                &mut splitting_edges_obj,
                                tol,
                            );
                        }
                    }
                }
            }
            for w_obj in f_obj.wires() {
                if let Some(circle) = wire_full_circle(&w_obj, tol) {
                    if circle_inside_face(&circle, &f_tool) {
                        let c = GeomCurve::Circle(circle);
                        let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                        split_tracked(
                            &mut builder_tool,
                            sub,
                            &c,
                            0.0,
                            TAU,
                            &mut splitting_edges_tool,
                            tol,
                        );
                        continue;
                    }
                }
                for e_obj in w_obj.edges() {
                    if let Some(c_obj) = e_obj.curve() {
                        let intervals = crate::intersect::trim_curve_to_face(
                            c_obj,
                            e_obj.first(),
                            e_obj.last(),
                            &f_tool,
                            tol,
                        );
                        for (first, last) in intervals {
                            let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                            split_tracked(
                                &mut builder_tool,
                                sub,
                                c_obj,
                                first,
                                last,
                                &mut splitting_edges_tool,
                                tol,
                            );
                        }
                    }
                }
            }
        } else {
            // Intersecting surfaces: split each face along the trimmed intersection curves
            let curves = crate::intersect::surface_surface_curves(&f_obj, &f_tool, tol);
            for (curve, first, last) in curves {
                let sub = obj_sub.get_mut(&f_obj_id).unwrap();
                split_tracked(
                    &mut builder_obj,
                    sub,
                    &curve,
                    first,
                    last,
                    &mut splitting_edges_obj,
                    tol,
                );
                let sub = tool_sub.get_mut(&f_tool_id).unwrap();
                split_tracked(
                    &mut builder_tool,
                    sub,
                    &curve,
                    first,
                    last,
                    &mut splitting_edges_tool,
                    tol,
                );
            }
        }
    }

    // Partition faces that have accumulated splitting edges
    let run_partition = |builder: &mut BRepBuilder,
                         sub: &mut Vec<FaceId>,
                         split_map: &mut std::collections::HashMap<
        FaceId,
        Vec<openrcad_topo::arena::EdgeId>,
    >| {
        let mut new_sub = Vec::new();
        for &fid in sub.iter() {
            if let Some(edges) = split_map.remove(&fid) {
                if builder.brep().faces.contains_key(fid) {
                    let partitioned = builder.partition_face(fid, &edges);
                    new_sub.extend(partitioned);
                } else {
                    new_sub.push(fid);
                }
            } else {
                new_sub.push(fid);
            }
        }
        *sub = new_sub;
    };

    // Partition in the original faces' deterministic order, NOT HashMap order.
    // Iterating `obj_sub.values_mut()` walks the map in Rust's per-process random
    // HashMap order, so `partition_face` allocates new FaceIds in a different
    // order each run. That cascades through `sew` (which merges in kept-face
    // order) and the merges into a topologically different — though still valid —
    // result, making downstream fillet/tessellation flaky (the same body would
    // pass or fail the ghost-material check from run to run). Walking the
    // deterministic original face lists pins the order so the boolean is
    // reproducible, which is what stable downstream edge/face identity needs.
    for f in &faces_obj {
        if let Some(sub) = obj_sub.get_mut(&f.id()) {
            run_partition(&mut builder_obj, sub, &mut splitting_edges_obj);
        }
    }
    for f in &faces_tool {
        if let Some(sub) = tool_sub.get_mut(&f.id()) {
            run_partition(&mut builder_tool, sub, &mut splitting_edges_tool);
        }
    }

    // 3. Classify all split faces
    let mut kept_faces = Vec::new();
    // Parallel to `kept_faces`: which INPUT face (by shell position) each kept
    // split face descends from — read straight from the split bookkeeping
    // (`obj_sub`/`tool_sub`), so it is exact, not matched.
    let mut kept_sources: Vec<Option<BooleanFaceSource>> = Vec::new();
    let origin_obj: std::collections::HashMap<FaceId, usize> = faces_obj
        .iter()
        .enumerate()
        .flat_map(|(idx, f)| {
            obj_sub
                .get(&f.id())
                .into_iter()
                .flatten()
                .map(move |&child| (child, idx))
        })
        .collect();
    let origin_tool: std::collections::HashMap<FaceId, usize> = faces_tool
        .iter()
        .enumerate()
        .flat_map(|(idx, f)| {
            tool_sub
                .get(&f.id())
                .into_iter()
                .flatten()
                .map(move |&child| (child, idx))
        })
        .collect();

    let brep_obj = builder_obj.build(); // seals into Arc<BRep>
    let brep_tool = builder_tool.build();

    // BVHs over the *split* faces so the coplanar pre-check tests only the
    // handful of opposite-side faces a given face actually overlaps, instead of
    // scanning all of them (O(n·m) → ≈O(n·log m)).
    let split_faces_obj: Vec<Face> = brep_obj
        .faces
        .iter()
        .map(|(id, d)| Face::from_id(brep_obj.clone(), id, d.orientation))
        .collect();
    let split_faces_tool: Vec<Face> = brep_tool
        .faces
        .iter()
        .map(|(id, d)| Face::from_id(brep_tool.clone(), id, d.orientation))
        .collect();
    let bvh_split_obj = Bvh::build(&split_faces_obj);
    let bvh_split_tool = Bvh::build(&split_faces_tool);

    for (f_id, f_data) in &brep_obj.faces {
        let face = Face::from_id(brep_obj.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar_same = false;
        let mut coplanar_opposite = false;

        for ft_id in bvh_split_tool.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let ft_data = &brep_tool.faces[ft_id];
            let face_t = Face::from_id(brep_tool.clone(), ft_id, ft_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face.surface(), face_t.surface()) {
                if surfaces_are_coplanar(s_obj, s_tool, tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_tool, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_t) {
                        let n_obj = match s_obj {
                            GeomSurface::Plane(p) => p.normal(),
                            _ => openrcad_foundation::Dir::dz(),
                        };
                        let n_tool = match s_tool {
                            GeomSurface::Plane(p) => p.normal(),
                            _ => openrcad_foundation::Dir::dz(),
                        };
                        if n_obj.dot(&n_tool) > 0.0 {
                            coplanar_same = true;
                        } else {
                            coplanar_opposite = true;
                        }
                        break;
                    }
                }
            }
        }

        if coplanar_same {
            match op {
                BooleanOp::Fuse | BooleanOp::Common => {
                    kept_sources.push(origin_obj.get(&f_id).copied().map(BooleanFaceSource::Object));
                    kept_faces.push(face);
                }
                BooleanOp::Cut => {}
            }
        } else if coplanar_opposite {
            // Discard both
        } else {
            let inside = is_point_inside_solid(&pos, tool, &bvh_tool);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => !inside,
                BooleanOp::Common => inside,
            };
            if keep {
                kept_sources.push(origin_obj.get(&f_id).copied().map(BooleanFaceSource::Object));
                kept_faces.push(face);
            }
        }
    }

    for (f_id, f_data) in &brep_tool.faces {
        let face = Face::from_id(brep_tool.clone(), f_id, f_data.orientation);
        let pos = point_on_face(&face);

        let mut coplanar = false;
        for fo_id in bvh_split_obj.box_overlap(&crate::bvh::compute_face_bounds(&face)) {
            let fo_data = &brep_obj.faces[fo_id];
            let face_o = Face::from_id(brep_obj.clone(), fo_id, fo_data.orientation);
            if let (Some(s_obj), Some(s_tool)) = (face_o.surface(), face.surface()) {
                if surfaces_are_coplanar(s_obj, s_tool, tol) {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(s_obj, &pos, (0.0, 0.0));
                    if crate::intersect::is_inside_trimming_loops(u, v, &face_o) {
                        coplanar = true;
                        break;
                    }
                }
            }
        }

        if coplanar {
            // Discard tool's coincident face (already handled by object's side)
        } else {
            let inside = is_point_inside_solid(&pos, object, &bvh_obj);
            let keep = match op {
                BooleanOp::Fuse => !inside,
                BooleanOp::Cut => inside,
                BooleanOp::Common => inside,
            };
            if keep {
                kept_sources.push(origin_tool.get(&f_id).copied().map(BooleanFaceSource::Tool));
                if op == BooleanOp::Cut {
                    let reversed_face =
                        Face::from_id(brep_tool.clone(), f_id, f_data.orientation.reversed());
                    kept_faces.push(reversed_face);
                } else {
                    kept_faces.push(face);
                }
            }
        }
    }

    // 4. Sew kept faces together
    let shell = sew(&kept_faces, tol);
    let solid = Solid::new(shell);

    // 4b. Heal T-junctions: an imprint can split one face's boundary edge at a
    //     vertex without splitting the coincident edge of a perpendicular adjacent
    //     face (e.g. a boss footprint straddling a box edge), leaving the shell
    //     open. Split such edges at the stray interior vertex so the coincident
    //     edges share endpoints. A no-op (and skipped) when already watertight.
    let solid = crate::merge::heal_tjunctions(&solid, tol);

    // History/owner plumbing. `sew`/`heal`/`merge` each re-key FaceIds, so
    // origins are not threaded through them — they are *resolved*: every stage's
    // faces are geometric subsets of the kept split faces, so sampling a point
    // on a face and asking which kept face's trimming loops contain it recovers
    // the exact origin (imprint boundaries decide coplanar ties, not
    // signatures).
    let has_classes = obj_classes.is_some() || tool_classes.is_some();
    let class_map = if has_classes {
        let origins = resolve_face_origins(&solid, &kept_faces, &kept_sources, tol);
        let mut map: std::collections::HashMap<FaceId, u64> = std::collections::HashMap::new();
        for (face, origin) in solid.shell().faces().iter().zip(&origins) {
            // Tag the side into the class key so an object class value can
            // never collide with an equal tool class value.
            let class = match origin {
                Some(BooleanFaceSource::Object(i)) => obj_classes
                    .and_then(|c| c.get(*i).copied().flatten())
                    .map(|cl| (cl << 1) | 0),
                Some(BooleanFaceSource::Tool(i)) => tool_classes
                    .and_then(|c| c.get(*i).copied().flatten())
                    .map(|cl| (cl << 1) | 1),
                None => None,
            };
            if let Some(class) = class {
                map.insert(face.id(), class);
            }
        }
        Some(map)
    } else {
        None
    };

    // 5. Merge coplanar faces split by the imprint (e.g. a union of two boxes
    //    keeps the shared face as several coplanar strips), then cocylindrical
    //    faces split by it (a corner cut whose arc crosses a `make_cylinder` rim
    //    seam leaves the concave wall as two faces). Each is a no-op fallback
    //    unless it produces a watertight, healthy, smaller solid. When owner
    //    classes are supplied, faces of different owners are never merged (the
    //    merge would erase the identity the history just preserved).
    let solid = crate::merge::merge_coplanar_faces_classed(&solid, class_map.as_ref());
    let solid = crate::merge::merge_cocylindrical_faces_classed(&solid, class_map.as_ref());

    let history = want_history
        .then(|| resolve_face_origins(&solid, &kept_faces, &kept_sources, tol))
        .map(|face_source| BooleanFaceHistory { face_source });
    (solid, history)
}

/// For each face of `solid` (in shell order), the kept split face it descends
/// from: same supporting surface (the sample point projects onto it within
/// tolerance, outward normals aligned) *and* the sample point inside the kept
/// face's trimming loops. Containment against the true imprint boundaries is
/// what disambiguates coplanar candidates exactly.
fn resolve_face_origins(
    solid: &Solid,
    kept_faces: &[Face],
    kept_sources: &[Option<BooleanFaceSource>],
    tol: f64,
) -> Vec<Option<BooleanFaceSource>> {
    let surf_tol = (tol * 10.0).max(1e-6);
    solid
        .shell()
        .faces()
        .iter()
        .map(|face| {
            let pos = point_on_face(face);
            let n_face = effective_normal_at(face, &pos)?;
            kept_faces
                .iter()
                .zip(kept_sources)
                .find_map(|(kept, source)| {
                    let surf = kept.surface()?;
                    let (u, v) = crate::intersect::search_nearest_parameter(surf, &pos, (0.0, 0.0));
                    if surf.point(u, v).distance(&pos) > surf_tol {
                        return None;
                    }
                    let n_kept = effective_normal_uv(kept, u, v)?;
                    if n_face.dot(&n_kept) < 0.5 {
                        return None;
                    }
                    if !crate::intersect::is_inside_trimming_loops(u, v, kept) {
                        return None;
                    }
                    *source
                })
        })
        .collect()
}

/// Outward normal of `face` at the surface point nearest `pos` (dU × dV with
/// the face's orientation applied). `None` for a degenerate parameterization.
fn effective_normal_at(face: &Face, pos: &Pnt) -> Option<Dir> {
    let surf = face.surface()?;
    let (u, v) = crate::intersect::search_nearest_parameter(surf, pos, (0.0, 0.0));
    effective_normal_uv(face, u, v)
}

fn effective_normal_uv(face: &Face, u: f64, v: f64) -> Option<Dir> {
    let surf = face.surface()?;
    let (_, du, dv) = surf.d1(u, v);
    let n = du.cross(&dv);
    let n = n.normalized()?;
    let mut dir = Dir::new(n.x(), n.y(), n.z());
    if face.orientation() == openrcad_topo::Orientation::Reversed {
        dir = dir.reversed();
    }
    Some(dir)
}

fn validate_operand(input: BooleanInput, solid: &Solid) -> Result<(), BooleanError> {
    let report = solid.health_report();
    if !report.is_healthy() || !solid.is_watertight() {
        return Err(BooleanError::InvalidInput { input, report });
    }
    Ok(())
}

fn validate_output(solid: Solid) -> Result<Solid, BooleanError> {
    let report = solid.health_report();
    if !report.is_healthy() {
        return Err(BooleanError::InvalidOutput { report });
    }
    if !solid.is_watertight() {
        return Err(BooleanError::NonWatertightOutput { report });
    }
    let sliver_tol = (bbox_diag(&solid) * SLIVER_REL).clamp(1e-12, SLIVER_CAP);
    if let Some(thickness) = degenerate_sliver_thickness(&solid, sliver_tol) {
        return Err(BooleanError::DegenerateSliver { thickness });
    }
    Ok(solid)
}

// ---- Fuzzy coincidence handling (snap + sliver gate) -----------------------
//
// A cut tool whose planar face stops a hair short of (or flush with) a body face
// is a near-coincidence. Left alone the boolean emits a near-zero-thickness
// "sliver": topologically watertight, but a tiny geometric lie that breaks the
// downstream fillet (the "funnel"). We handle it in two narrow, intent-preserving
// steps — both size-relative with a hard cap so a deliberate clearance larger
// than the fuzz is never touched:
//   1. a pre-boolean snap that nudges the tool to make a near-coincident,
//      overlapping face pair exactly coincident (`snap_tool_to_object`);
//   2. a post-boolean gate that rejects any residual sliver
//      (`degenerate_sliver_thickness`, wired into `validate_output`).

/// Relative pre-boolean snap fuzz (× bbox diagonal), hard-capped. Only gaps below
/// this — numerical near-coincidences, not real clearances — are snapped.
const SNAP_REL: f64 = 1e-7;
const SNAP_CAP: f64 = 1e-4;
/// Relative sliver-rejection tolerance (× bbox diagonal), hard-capped. A wall
/// thinner than this that survived snapping is rejected as degenerate.
const SLIVER_REL: f64 = 1e-6;
const SLIVER_CAP: f64 = 1e-3;

/// A planar face reduced to its plane frame and outer-wire vertices.
struct PlanarFaceInfo {
    normal: Dir,
    x: Dir,
    y: Dir,
    offset: f64, // normal · (point on plane)
    verts: Vec<Pnt>,
}

fn planar_faces(solid: &Solid) -> Vec<PlanarFaceInfo> {
    let mut out = Vec::new();
    for f in solid.shell().faces() {
        if let Some(GeomSurface::Plane(p)) = f.surface() {
            let pos = p.position();
            let n = pos.direction();
            let loc = pos.location();
            let verts: Vec<Pnt> = match f.outer_wire() {
                Some(w) => w.edges().iter().map(|e| e.start().point()).collect(),
                None => continue,
            };
            if verts.is_empty() {
                continue;
            }
            out.push(PlanarFaceInfo {
                normal: n,
                x: pos.x_direction(),
                y: pos.y_direction(),
                offset: n.x() * loc.x() + n.y() * loc.y() + n.z() * loc.z(),
                verts,
            });
        }
    }
    out
}

/// Diagonal of the solid's bounding box (≥ 1.0), the length scale for fuzz.
fn bbox_diag(solid: &Solid) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for f in solid.shell().faces() {
        if let Some(w) = f.outer_wire() {
            for e in w.edges() {
                let p = e.start().point();
                for (k, c) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
                    lo[k] = lo[k].min(c);
                    hi[k] = hi[k].max(c);
                }
            }
        }
    }
    if lo[0] > hi[0] {
        return 1.0;
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2))
        .sqrt()
        .max(1.0)
}

#[inline]
fn dir_dot(a: Dir, b: Dir) -> f64 {
    a.x() * b.x() + a.y() * b.y() + a.z() * b.z()
}

/// Signed distance from `p` to face `f`'s plane.
#[inline]
fn plane_dist(f: &PlanarFaceInfo, p: Pnt) -> f64 {
    f.normal.x() * p.x() + f.normal.y() * p.y() + f.normal.z() * p.z() - f.offset
}

/// Do the two near-parallel planar faces overlap when projected onto `a`'s
/// in-plane axes? (Axis-aligned 2D bbox overlap — cheap and conservative.)
fn overlap_in_plane(a: &PlanarFaceInfo, b: &PlanarFaceInfo) -> bool {
    let proj = |verts: &[Pnt]| {
        let (mut u0, mut u1, mut v0, mut v1) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for p in verts {
            let u = a.x.x() * p.x() + a.x.y() * p.y() + a.x.z() * p.z();
            let v = a.y.x() * p.x() + a.y.y() * p.y() + a.y.z() * p.z();
            u0 = u0.min(u);
            u1 = u1.max(u);
            v0 = v0.min(v);
            v1 = v1.max(v);
        }
        (u0, u1, v0, v1)
    };
    let (au0, au1, av0, av1) = proj(&a.verts);
    let (bu0, bu1, bv0, bv1) = proj(&b.verts);
    au0 <= bu1 && bu0 <= au1 && av0 <= bv1 && bv0 <= av1
}

/// Pre-boolean fuzzy snap. If the tool sits within `fuzz` of coincidence with the
/// object on one or more compatible, overlapping planar-face pairs, translate the
/// whole tool so those pairs become exactly coincident — but only if a single
/// translation satisfies *every* detected pair. A conflicting multi-coincidence
/// (which one translation can't resolve) is left for the sliver gate.
fn snap_tool_to_object(object: &Solid, tool: &Solid, fuzz: f64) -> Solid {
    let obj_planes = planar_faces(object);
    let tool_planes = planar_faces(tool);
    // One required translation per near-coincident, overlapping pair.
    let mut reqs: Vec<(Dir, f64)> = Vec::new();
    for tp in &tool_planes {
        for op in &obj_planes {
            if dir_dot(tp.normal, op.normal).abs() < 0.999 {
                continue;
            }
            let d = plane_dist(op, tp.verts[0]); // tool plane's offset from obj plane
            if d == 0.0 || d.abs() >= fuzz {
                continue;
            }
            if !overlap_in_plane(op, tp) {
                continue;
            }
            reqs.push((op.normal, -d)); // close the gap: translate by op.normal·(-d)
        }
    }
    if reqs.is_empty() {
        return tool.clone();
    }
    // Compose one translation; orthogonal requirements add, a real conflict fails.
    let mut t = GeomVec::new(0.0, 0.0, 0.0);
    for &(n, s) in &reqs {
        t += GeomVec::from_dir(n) * s;
    }
    for &(n, s) in &reqs {
        let achieved = n.x() * t.x() + n.y() * t.y() + n.z() * t.z();
        if (achieved - s).abs() > fuzz * 0.5 {
            return tool.clone(); // conflicting coincidences — let the gate handle it
        }
    }
    if t.magnitude() < 1e-12 {
        return tool.clone();
    }
    tool.transformed(&Trsf::translation(t))
}

/// Smallest wall thickness of a degenerate sliver in `solid`: two planar faces
/// that are parallel, within `tol` of coincident, and overlap in projection.
/// `None` if no sliver. (Exactly-coincident faces, `d ≈ 0`, are excluded — those
/// surface as non-manifold/non-watertight and are caught earlier.)
fn degenerate_sliver_thickness(solid: &Solid, tol: f64) -> Option<f64> {
    let planes = planar_faces(solid);
    let mut found: Option<f64> = None;
    for i in 0..planes.len() {
        for j in (i + 1)..planes.len() {
            let (a, b) = (&planes[i], &planes[j]);
            if dir_dot(a.normal, b.normal).abs() < 0.999 {
                continue;
            }
            let d = plane_dist(a, b.verts[0]).abs();
            if d <= 1e-12 || d >= tol {
                continue;
            }
            if !overlap_in_plane(a, b) {
                continue;
            }
            found = Some(found.map_or(d, |f| f.min(d)));
        }
    }
    found
}

/// Split `edge_id` at `pt`, but only when `pt` genuinely lies in the *interior*
/// of the finite edge segment. Skips off-segment crossings (the host curves are
/// infinite lines) and crossings that fall on an existing endpoint — both would
/// otherwise spawn zero-length / duplicate edges that break the sewn topology.
fn try_split_edge(
    builder: &mut BRepBuilder,
    edge_id: EdgeId,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    pt: &Pnt,
    tol: f64,
) {
    if !builder.brep().edges.contains_key(edge_id) {
        return;
    }
    let t = project_point_on_curve(pt, curve, first, last);
    if curve.point(t).distance(pt) > 1e-6 {
        return; // off the finite segment
    }
    let ed = &builder.brep().edges[edge_id];
    let s_pt = builder.brep().vertices[ed.start].point;
    let e_pt = builder.brep().vertices[ed.end].point;
    if pt.distance(&s_pt) < tol || pt.distance(&e_pt) < tol {
        return; // coincides with an endpoint — no real split
    }
    let v = builder
        .brep_mut()
        .vertices
        .insert(openrcad_topo::arena::VertexData {
            point: *pt,
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
    builder.split_edge(edge_id, v, t);
}

/// Split every sub-face in `subfaces` along `curve`, replacing the list with the
/// resulting (possibly larger) set of descendant faces. Only these descendants
/// are ever revisited, so repeated calls across overlapping pairs stay bounded.
fn split_tracked(
    builder: &mut BRepBuilder,
    subfaces: &mut Vec<FaceId>,
    curve: &GeomCurve,
    first: f64,
    last: f64,
    splitting_edges_map: &mut std::collections::HashMap<FaceId, Vec<openrcad_topo::arena::EdgeId>>,
    tol: f64,
) {
    let mut result = Vec::with_capacity(subfaces.len());
    for &fid in subfaces.iter() {
        // A clean 2-point crosscut on a cylindrical face must be *queued* (not
        // split immediately): immediate bisection fragments the wall, so a
        // rect-minus-cylinder body collapses to a box (the vanishing circular
        // bite). Queuing lets every crosscut on the cylinder partition together
        // via the deferred `partition_face` path; `merge_cocylindrical_faces`
        // then coalesces the wall back to one analytic cylinder.
        let is_cylinder = builder
            .brep()
            .faces
            .get(fid)
            .and_then(|f| f.surface.as_ref())
            .is_some_and(|s| matches!(s, GeomSurface::Cylinder(_)));
        let force_queue_clean_crosscuts = is_cylinder
            || splitting_edges_map
                .get(&fid)
                .is_some_and(|edges| !edges.is_empty());
        let (next_faces, new_edges) = crate::imprint::imprint_curve_on_face(
            builder,
            fid,
            curve,
            first,
            last,
            force_queue_clean_crosscuts,
            tol,
        );
        if !new_edges.is_empty() {
            splitting_edges_map
                .entry(fid)
                .or_default()
                .extend(new_edges);
        }
        result.extend(next_faces);
    }
    *subfaces = result;
}

pub(crate) fn project_point_on_curve(p: &Pnt, curve: &GeomCurve, t_min: f64, t_max: f64) -> f64 {
    let (t_min, t_max) = ordered_curve_bounds(t_min, t_max);
    let mut best_t = t_min;
    let mut min_dist = p.distance(&curve.point(t_min));
    let steps = 10;
    for i in 0..=steps {
        let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
        let dist = p.distance(&curve.point(t));
        if dist < min_dist {
            min_dist = dist;
            best_t = t;
        }
    }

    let mut t = best_t;
    for _ in 0..5 {
        let (pt, tangent) = curve.d1(t);
        let diff = pt - *p;
        let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
        t = clamp_ordered(t - dt, t_min, t_max);
    }
    t
}

fn ordered_curve_bounds(t_min: f64, t_max: f64) -> (f64, f64) {
    let t_min = if t_min.is_finite() { t_min } else { -100.0 };
    let t_max = if t_max.is_finite() { t_max } else { 100.0 };
    if t_min <= t_max {
        (t_min, t_max)
    } else {
        (t_max, t_min)
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    let (lo, hi) = if min <= max { (min, max) } else { (max, min) };
    if value.is_nan() {
        lo
    } else {
        value.max(lo).min(hi)
    }
}

fn point_on_face(face: &Face) -> Pnt {
    // Preferred: the centroid of the outer loop's vertices, projected onto the
    // surface. For a convex face this is a robustly *interior* point, which the
    // ray-parity classifier needs — a near-boundary sample produces ambiguous
    // (boundary-grazing) hit counts. Only used if it actually lands inside the
    // trimming loops (non-convex faces fall through to the edge-offset probe).
    if let (Some(outer), Some(surf)) = (face.outer_wire(), face.surface()) {
        let pts: Vec<Pnt> = outer.edges().iter().map(|e| e.start().point()).collect();
        if !pts.is_empty() {
            let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
            for p in &pts {
                sum += p.coord();
            }
            let centroid = Pnt::from_xyz(sum / pts.len() as f64);
            let (cu, cv) = crate::intersect::search_nearest_parameter(surf, &centroid, (0.0, 0.0));
            let on_surf = surf.point(cu, cv);
            if on_surf.distance(&centroid) < 1e-6
                && crate::intersect::is_inside_trimming_loops(cu, cv, face)
            {
                let mut on_boundary = false;
                for wire in face.wires() {
                    for edge in wire.edges() {
                        if distance_point_to_edge(&on_surf, &edge) < 1e-4 {
                            on_boundary = true;
                            break;
                        }
                    }
                    if on_boundary {
                        break;
                    }
                }
                if !on_boundary {
                    return on_surf;
                }
            }
        }
    }

    if let Some(outer) = face.outer_wire() {
        let edges = outer.edges();
        if !edges.is_empty() {
            let e = &edges[0];
            let t_mid = 0.5 * (e.first() + e.last());
            if let Some(curve) = e.curve() {
                let mid_pt = curve.point(t_mid);
                if let Some(surf) = face.surface() {
                    let (u, v) =
                        crate::intersect::search_nearest_parameter(surf, &mid_pt, (0.0, 0.0));
                    let (_, tangent) = curve.d1(t_mid);
                    let normal = match surf {
                        GeomSurface::Plane(plane) => plane.normal(),
                        _ => {
                            let (_, su, sv) = eval_d1(surf, u, v);
                            su.cross(&sv)
                                .normalized()
                                .unwrap_or(openrcad_foundation::Dir::dz())
                        }
                    };

                    let dir1 = GeomVec::from_dir(
                        tangent
                            .cross(&openrcad_foundation::Vec::from_dir(normal))
                            .normalized()
                            .unwrap(),
                    );
                    let dir2 = -dir1;

                    let test_dist = 1e-3;
                    let p1 = mid_pt + dir1 * test_dist;
                    let (u1, v1) = crate::intersect::search_nearest_parameter(surf, &p1, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u1, v1, face) {
                        return p1;
                    }

                    let p2 = mid_pt + dir2 * test_dist;
                    let (u2, v2) = crate::intersect::search_nearest_parameter(surf, &p2, (u, v));
                    if crate::intersect::is_inside_trimming_loops(u2, v2, face) {
                        return p2;
                    }
                }
            }
        }
    }

    let vertices = face
        .wires()
        .iter()
        .flat_map(|w| w.edges())
        .map(|e| e.start().point())
        .collect::<Vec<_>>();
    if !vertices.is_empty() {
        let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
        for v in &vertices {
            sum += v.coord();
        }
        Pnt::from_xyz(sum / vertices.len() as f64)
    } else {
        Pnt::origin()
    }
}

fn is_point_inside_solid(p: &Pnt, solid: &Solid, bvh: &Bvh) -> bool {
    // A "generic" irrational-ish direction: a symmetric diagonal like (1,1,1)
    // skewers the corners/edges of axis-aligned boxes, defeating the parity
    // test. This direction avoids hitting integer-coordinate vertices/edges.
    let mut ray_dir = Dir::new(0.182_321, 0.523_157, 0.832_511);
    let mut rng: u64 = 42;

    // Cap the perturbation retries: a point that stubbornly grazes boundaries
    // should not spin forever. After the cap we accept the last parity reading.
    for _attempt in 0..32 {
        let mut intersection_count = 0;
        let mut hit_boundary = false;

        let ray_dir_vec = GeomVec::from_dir(ray_dir);

        // Only test faces whose AABB the ray actually crosses. The BVH prunes
        // the rest, so each cast is O(log F + hits) instead of O(F), and we
        // reconstruct only the hit faces (via Face::from_id) rather than
        // materializing the whole shell's face list on every call.
        let candidate_face_ids = bvh.ray_cast(p, &ray_dir_vec);

        'faces: for fid in candidate_face_ids {
            let orientation = solid.brep().faces[fid].orientation;
            let face = Face::from_id(solid.brep().clone(), fid, orientation);
            for hit in crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7) {
                let mut on_boundary = false;
                for wire in face.wires() {
                    for edge in wire.edges() {
                        let dist_to_edge = distance_point_to_edge(&hit, &edge);
                        if dist_to_edge < 1e-5 {
                            on_boundary = true;
                            break;
                        }
                    }
                    if on_boundary {
                        break;
                    }
                }

                if on_boundary {
                    hit_boundary = true;
                    break 'faces;
                }

                intersection_count += 1;
            }
        }

        if hit_boundary {
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let rx = ((rng & 0xff) as f64 / 255.0) - 0.5;
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let ry = ((rng & 0xff) as f64 / 255.0) - 0.5;
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
            let rz = ((rng & 0xff) as f64 / 255.0) - 0.5;

            ray_dir =
                Dir::from_vec(&(ray_dir_vec + GeomVec::new(rx, ry, rz) * 0.1)).unwrap_or(ray_dir);
            continue;
        }

        return (intersection_count % 2) == 1;
    }

    // Retries exhausted: fall back to a final parity reading without the
    // boundary veto so we always return a definite answer.
    let ray_dir_vec = GeomVec::from_dir(ray_dir);
    let mut count = 0;
    for fid in bvh.ray_cast(p, &ray_dir_vec) {
        let orientation = solid.brep().faces[fid].orientation;
        let face = Face::from_id(solid.brep().clone(), fid, orientation);
        count += crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7).len();
    }
    (count % 2) == 1
}

fn is_point_inside_solid_robust(p: &Pnt, solid: &Solid, bvh: &Bvh) -> bool {
    // Parity of ray-surface crossings along one direction (`true` => odd => inside),
    // skipping any cast whose hit grazes a face boundary edge (an unreliable count).
    // Returns `None` if every perturbation of this direction kept grazing.
    //
    // A single ray can still silently mis-count without grazing — e.g. a hit that
    // exits right at a cap/wall rim is dropped by the face-containment test, turning
    // an even count odd. So the caller votes across several independent directions:
    // an occasional miss is outvoted instead of flipping the classification.
    let cast_parity = |base: Dir, seed: u64| -> Option<bool> {
        let mut ray_dir = base;
        let mut rng: u64 = seed;
        for _attempt in 0..16 {
            let mut intersection_count = 0;
            let mut hit_boundary = false;
            let ray_dir_vec = GeomVec::from_dir(ray_dir);

            // Only test faces whose AABB the ray crosses (BVH prunes the rest), so
            // each cast is O(log F + hits) and only hit faces are reconstructed.
            'faces: for fid in bvh.ray_cast(p, &ray_dir_vec) {
                let orientation = solid.brep().faces[fid].orientation;
                let face = Face::from_id(solid.brep().clone(), fid, orientation);
                for hit in crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7) {
                    let mut on_boundary = false;
                    for wire in face.wires() {
                        for edge in wire.edges() {
                            if distance_point_to_edge(&hit, &edge) < 1e-5 {
                                on_boundary = true;
                                break;
                            }
                        }
                        if on_boundary {
                            break;
                        }
                    }
                    if on_boundary {
                        hit_boundary = true;
                        break 'faces;
                    }
                    intersection_count += 1;
                }
            }

            if hit_boundary {
                rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
                let rx = ((rng & 0xff) as f64 / 255.0) - 0.5;
                rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
                let ry = ((rng & 0xff) as f64 / 255.0) - 0.5;
                rng = rng.wrapping_mul(1103515245).wrapping_add(12345) & 0x7fffffff;
                let rz = ((rng & 0xff) as f64 / 255.0) - 0.5;
                ray_dir = Dir::from_vec(&(ray_dir_vec + GeomVec::new(rx, ry, rz) * 0.1))
                    .unwrap_or(ray_dir);
                continue;
            }
            return Some((intersection_count % 2) == 1);
        }
        None
    };

    // A spread of "generic" irrational-ish directions: a symmetric diagonal like
    // (1,1,1) skewers the corners/edges of axis-aligned boxes, defeating parity, so
    // every direction avoids integer-coordinate alignments. Voting across them makes
    // the classifier robust to a single direction's occasional rim mis-count.
    const DIRS: [(f64, f64, f64); 5] = [
        (0.182_321, 0.523_157, 0.832_511),
        (0.701_223, -0.337_419, 0.628_991),
        (-0.487_633, 0.811_077, 0.324_551),
        (0.273_194, 0.659_832, -0.700_447),
        (-0.638_915, -0.451_273, 0.623_881),
    ];

    let mut inside_votes = 0i32;
    let mut total = 0i32;
    for (i, &(x, y, z)) in DIRS.iter().enumerate() {
        let Some(dir) = Dir::from_vec(&GeomVec::new(x, y, z)) else {
            continue;
        };
        if let Some(parity) = cast_parity(dir, 42 + i as u64) {
            total += 1;
            if parity {
                inside_votes += 1;
            }
        }
    }

    if total == 0 {
        // Every direction kept grazing: fall back to one no-veto parity reading so
        // we always return a definite answer.
        let ray_dir_vec = GeomVec::new(0.182_321, 0.523_157, 0.832_511);
        let mut count = 0;
        for fid in bvh.ray_cast(p, &ray_dir_vec) {
            let orientation = solid.brep().faces[fid].orientation;
            let face = Face::from_id(solid.brep().clone(), fid, orientation);
            count += crate::intersect::ray_face_all(p, &ray_dir_vec, &face, 1e-7).len();
        }
        return (count % 2) == 1;
    }

    // Majority vote (ties -> inside is false, matching a strict-majority "inside").
    inside_votes * 2 > total
}

/// Whether `p` lies inside `solid` (ray-parity test).
///
/// Public wrapper over [`is_point_inside_solid`] that builds the face BVH the
/// same way the boolean engine does. Useful to classify a probe point against a
/// body — e.g. the rolling-ball fillet distinguishing a concave cut wall
/// (material outside the cylinder) from a convex prior-blend cylinder (inside).
pub fn point_in_solid(p: &Pnt, solid: &Solid) -> bool {
    let bvh = Bvh::build(&solid.shell().faces());
    is_point_inside_solid_robust(p, solid, &bvh)
}

fn distance_point_to_edge(p: &Pnt, edge: &openrcad_topo::Edge) -> f64 {
    if let Some(curve) = edge.curve() {
        let (t_min, t_max) = ordered_curve_bounds(edge.first(), edge.last());
        let mut best_t = t_min;
        let mut min_dist = p.distance(&curve.point(t_min));

        let steps = 10;
        for i in 0..=steps {
            let t = t_min + (t_max - t_min) * (i as f64) / steps as f64;
            let dist = p.distance(&curve.point(t));
            if dist < min_dist {
                min_dist = dist;
                best_t = t;
            }
        }

        let mut t = best_t;
        for _ in 0..5 {
            let (pt, tangent) = curve.d1(t);
            let diff = pt - *p;
            let dt = diff.dot(&tangent) / (tangent.dot(&tangent) + 1e-15);
            t = clamp_ordered(t - dt, t_min, t_max);
        }
        p.distance(&curve.point(t))
    } else {
        p.distance(&edge.start().point())
    }
}

/// If `wire` is a closed loop whose every edge is an arc of one common circle
/// (a cylinder cap rim), return that full circle. Used to imprint the whole
/// circle as a hole instead of three un-closable arcs.
fn wire_full_circle(wire: &Wire, tol: f64) -> Option<Circle> {
    let edges = wire.edges();
    if edges.len() < 2 {
        return None;
    }
    let mut circ: Option<Circle> = None;
    for e in &edges {
        match e.curve() {
            Some(GeomCurve::Circle(c)) => match &circ {
                Some(prev) => {
                    if prev.center().distance(&c.center()) > tol
                        || (prev.radius() - c.radius()).abs() > tol
                        || !prev.axis().is_parallel(&c.axis(), 1e-6)
                    {
                        return None;
                    }
                }
                None => circ = Some(*c),
            },
            _ => return None,
        }
    }
    circ
}

/// Whether `circle` lies on `face`'s surface and strictly inside its trimming
/// loops (sampled around the rim) — i.e. the cap sits fully within `face`.
fn circle_inside_face(circle: &Circle, face: &Face) -> bool {
    let Some(surf) = face.surface() else {
        return false;
    };
    for i in 0..8 {
        let u = TAU * (i as f64) / 8.0;
        let p = circle.point(u);
        let (uu, vv) = crate::intersect::search_nearest_parameter(surf, &p, (0.0, 0.0));
        if surf.point(uu, vv).distance(&p) > 1e-6 {
            return false;
        }
        if !crate::intersect::is_inside_trimming_loops(uu, vv, face) {
            return false;
        }
    }
    true
}

fn surfaces_are_coplanar(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> bool {
    match (s1, s2) {
        (GeomSurface::Plane(p1), GeomSurface::Plane(p2)) => {
            let dot = p1.normal().dot(&p2.normal());
            if (dot.abs() - 1.0).abs() > 1e-5 {
                return false;
            }
            let dist = GeomVec::from_dir(p1.normal()).dot(&(p2.location() - p1.location()));
            dist.abs() <= tol
        }
        _ => false,
    }
}

fn eval_d1(s: &GeomSurface, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec) {
    s.d1(u, v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;
    use openrcad_topo::Shell;

    /// Shell position of the input/result face whose plane has |normal·axis|≈1
    /// and passes through `coord` on `axis`.
    fn plane_face_pos(solid: &Solid, axis: usize, coord: f64) -> Option<usize> {
        solid.shell().faces().iter().position(|f| {
            let Some(GeomSurface::Plane(p)) = f.surface() else {
                return false;
            };
            let n = p.normal();
            if [n.x(), n.y(), n.z()][axis].abs() < 0.99 {
                return false;
            }
            let loc = p.location();
            (([loc.x(), loc.y(), loc.z()][axis]) - coord).abs() < 1e-6
        })
    }

    #[test]
    fn history_traces_union_faces_to_their_operands() {
        // obj [0,10]^3 ∪ tool [5,15]x[0,10]x[0,10] → one [0,15] box.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
        let (result, hist) = boolean_with_history(&obj, &tool, BooleanOp::Fuse, None, None);

        assert_eq!(
            hist.face_source.len(),
            result.shell().faces().len(),
            "one history entry per result face"
        );
        let x0 = plane_face_pos(&result, 0, 0.0).expect("x=0 face");
        let x15 = plane_face_pos(&result, 0, 15.0).expect("x=15 face");
        let obj_x0 = plane_face_pos(&obj, 0, 0.0).expect("object x=0 input face");
        let tool_x15 = plane_face_pos(&tool, 0, 15.0).expect("tool x=15 input face");
        assert_eq!(
            hist.source_of(x0),
            Some(BooleanFaceSource::Object(obj_x0)),
            "x=0 face must trace to the object's own x=0 input face"
        );
        assert_eq!(
            hist.source_of(x15),
            Some(BooleanFaceSource::Tool(tool_x15)),
            "x=15 face must trace to the tool's x=15 input face"
        );
        assert!(
            hist.face_source.iter().all(|s| s.is_some()),
            "every union face should be attributed, got {:?}",
            hist.face_source
        );
    }

    #[test]
    fn history_traces_cut_pocket_walls_to_the_tool() {
        // Square pillar punched through the box in Z: 4 generated hole walls.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(3.0, 3.0, -1.0), 4.0, 4.0, 12.0);
        let (result, hist) = boolean_with_history(&obj, &tool, BooleanOp::Cut, None, None);

        let tool_walls = hist
            .face_source
            .iter()
            .filter(|s| matches!(s, Some(BooleanFaceSource::Tool(_))))
            .count();
        assert!(
            tool_walls >= 4,
            "the 4 generated hole walls must trace to the tool, got {:?}",
            hist.face_source
        );
        // Modified: the box top survives (with a hole) and keeps its identity.
        let top = plane_face_pos(&result, 2, 10.0).expect("result keeps a z=10 top");
        let obj_top = plane_face_pos(&obj, 2, 10.0).expect("object z=10 input face");
        assert_eq!(
            hist.source_of(top),
            Some(BooleanFaceSource::Object(obj_top)),
            "the holed top face is a MODIFIED image of the object's top"
        );
        // Determinism: the correspondence is identical across rebuilds.
        let (_, hist2) = boolean_with_history(&obj, &tool, BooleanOp::Cut, None, None);
        assert_eq!(hist.face_source, hist2.face_source);
    }

    #[test]
    fn owner_classes_block_cross_owner_coplanar_merge() {
        // Two boxes side by side, tops coplanar at z=10. Unclassed, the merge
        // unifies the tops into ONE face; with distinct owner classes on every
        // face of each operand, the two tops must stay separate faces (each
        // attributed to its own operand) — the Bidarra owner-aware-merge rule.
        let obj = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let tool = make_box(&Pnt::new(10.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        let unclassed = boolean(&obj, &tool, BooleanOp::Fuse);
        let unclassed_tops = unclassed
            .shell()
            .faces()
            .iter()
            .filter(|f| match f.surface() {
                Some(GeomSurface::Plane(p)) => {
                    p.normal().z().abs() > 0.99 && (p.location().z() - 10.0).abs() < 1e-6
                }
                _ => false,
            })
            .count();
        assert_eq!(unclassed_tops, 1, "legacy merge unifies the coplanar tops");

        let obj_classes: Vec<Option<u64>> =
            (0..obj.shell().faces().len() as u64).map(Some).collect();
        let tool_classes: Vec<Option<u64>> =
            (0..tool.shell().faces().len() as u64).map(Some).collect();
        let (classed, hist) = boolean_with_history(
            &obj,
            &tool,
            BooleanOp::Fuse,
            Some(&obj_classes),
            Some(&tool_classes),
        );
        let classed_tops: Vec<usize> = classed
            .shell()
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| match f.surface() {
                Some(GeomSurface::Plane(p)) => (p.normal().z().abs() > 0.99
                    && (p.location().z() - 10.0).abs() < 1e-6)
                    .then_some(i),
                _ => None,
            })
            .collect();
        assert_eq!(
            classed_tops.len(),
            2,
            "owner-aware merge must keep the two differently-owned tops distinct"
        );
        let sides: Vec<_> = classed_tops.iter().map(|&i| hist.source_of(i)).collect();
        assert!(
            sides
                .iter()
                .any(|s| matches!(s, Some(BooleanFaceSource::Object(_))))
                && sides
                    .iter()
                    .any(|s| matches!(s, Some(BooleanFaceSource::Tool(_)))),
            "one top from each operand, got {sides:?}"
        );
    }

    #[test]
    fn ordered_curve_bounds_accept_reversed_and_nonfinite_limits() {
        assert_eq!(ordered_curve_bounds(2.14, 2.09), (2.09, 2.14));
        assert_eq!(ordered_curve_bounds(f64::NAN, 2.09), (-100.0, 2.09));
        assert_eq!(ordered_curve_bounds(2.14, f64::INFINITY), (2.14, 100.0));
    }

    #[test]
    fn primitives_pass_structural_validation() {
        // A well-formed box: contiguous loops, Euler–Poincaré V−E+F = 8−12+6 = 2.
        let b = make_box(&Pnt::new(-1.0, 2.0, 0.5), 2.0, 3.0, 4.0);
        b.assert_valid();
        assert_eq!(b.euler_characteristic(), 2);

        // Adversarial sliver: an extreme aspect ratio (thickness 1e-4 over a
        // 100×100 footprint) is still a topologically valid closed box. Its near
        // degeneracy is exactly the kind of input that trips fragile float
        // classification, so the structural invariants must still hold.
        let sliver = make_box(&Pnt::origin(), 100.0, 100.0, 1e-4);
        sliver.assert_valid();
        assert_eq!(sliver.euler_characteristic(), 2);
    }

    // ---- Fuzzy snap + sliver gate ------------------------------------------

    /// Box (21 × 11.5024 × 17.6) and a +Y cut cylinder whose back cap stops `gap`
    /// short of the back face (`gap > 0` = near-miss/sliver; `gap = 0` = flush).
    fn box_and_gap_cut(gap: f64) -> (Solid, Solid) {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;
        let back = 11.5024;
        let cube = make_box(&Pnt::new(-13.4, 0.0, -12.8), 21.0, back, 17.6);
        let cap_y = back - gap;
        let axis = Ax2::new_axes(Pnt::new(7.5, cap_y - 23.0, 5.0), Dir::dy(), Dir::dx());
        let cyl = make_cylinder(&axis, 7.81, 23.0);
        (cube, cyl)
    }

    /// Max vertex displacement between two structurally-identical solids.
    fn max_vertex_shift(a: &Solid, b: &Solid) -> f64 {
        let pts = |s: &Solid| -> Vec<Pnt> {
            s.shell()
                .faces()
                .iter()
                .filter_map(|f| f.outer_wire())
                .flat_map(|w| w.edges().into_iter().map(|e| e.start().point()))
                .collect()
        };
        pts(a)
            .iter()
            .zip(pts(b).iter())
            .map(|(x, y)| x.distance(y))
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn snap_fires_for_tiny_gap_but_preserves_real_clearance() {
        let (cube, _) = box_and_gap_cut(0.0);
        let fuzz = (bbox_diag(&cube) * SNAP_REL).clamp(1e-12, SNAP_CAP);

        // A tiny numerical gap (sub-fuzz) snaps: the tool is nudged.
        let (cube, tool) = box_and_gap_cut(1e-7);
        let snapped = snap_tool_to_object(&cube, &tool, fuzz);
        assert!(
            max_vertex_shift(&tool, &snapped) > 0.0,
            "a sub-fuzz near-coincidence should snap"
        );

        // A deliberate 0.1 mm clearance is far above the fuzz: the tool must NOT
        // move — the snap is not "CAD autocorrect with opinions".
        let (cube, tool) = box_and_gap_cut(0.1);
        let snapped = snap_tool_to_object(&cube, &tool, fuzz);
        assert_eq!(
            max_vertex_shift(&tool, &snapped),
            0.0,
            "a real clearance above the fuzz must be left untouched"
        );
    }

    #[test]
    fn snap_skips_when_planes_do_not_overlap() {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;
        // A cylinder cap that is near-coplanar with the box's back face (gap 1e-7)
        // but positioned far away in-plane (centre XZ = (100,100)) — no projected
        // overlap, so it must NOT be snapped despite the near-coincident plane.
        let back = 11.5024;
        let cube = make_box(&Pnt::new(-13.4, 0.0, -12.8), 21.0, back, 17.6);
        let cap_y = back - 1e-7;
        let axis = Ax2::new_axes(Pnt::new(100.0, cap_y - 23.0, 100.0), Dir::dy(), Dir::dx());
        let tool = make_cylinder(&axis, 2.0, 23.0);
        let fuzz = (bbox_diag(&cube) * SNAP_REL).clamp(1e-12, SNAP_CAP);
        let snapped = snap_tool_to_object(&cube, &tool, fuzz);
        assert_eq!(
            max_vertex_shift(&tool, &snapped),
            0.0,
            "near-coplanar but non-overlapping faces must not snap"
        );
    }

    #[test]
    fn tiny_gap_cut_is_clean_after_snap_and_fillets() {
        let (cube, tool) = box_and_gap_cut(1e-7);
        let cut = boolean_checked(&cube, &tool, BooleanOp::Cut)
            .expect("a tiny-gap cut must snap to a clean, watertight body");
        assert!(cut.is_watertight() && cut.health_report().is_healthy());

        // The fillet that funneled in the GUI: top-back edge into the scoop.
        let edge = openrcad_topo::Edge::between_points(
            Pnt::new(-13.4, 11.5024, 4.8),
            Pnt::new(-0.31, 11.5024, 4.8),
        );
        let filleted = crate::fillet_edges(&cut, std::slice::from_ref(&edge), 3.0)
            .expect("filleting into the snapped cut must succeed");
        assert!(filleted.is_watertight() && filleted.health_report().is_healthy());
    }

    #[test]
    fn watertight_sliver_is_rejected_by_checked() {
        // A gap above the snap fuzz but a degenerate sub-sliver-tol wall: the cut
        // is watertight yet a tiny geometric lie. `boolean_checked` must reject it
        // rather than hand back a body that breaks the fillet.
        let (cube, tool) = box_and_gap_cut(2e-5);
        let res = boolean_checked(&cube, &tool, BooleanOp::Cut);
        assert!(res.is_err(), "a degenerate sliver must be rejected, got Ok");
    }

    #[test]
    fn test_boolean_intersection_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Common (Intersection)
        let common = boolean(&cube1, &cube2, BooleanOp::Common);
        // The intersection should be a 5x10x10 box
        // Bounding box of the intersection should be [5, 0, 0] to [10, 10, 10]
        let (lo, hi) = common.bounding_box().corners().unwrap();
        assert!((lo.x() - 5.0).abs() < 1e-5);
        assert!((lo.y() - 0.0).abs() < 1e-5);
        assert!((lo.z() - 0.0).abs() < 1e-5);
        assert!((hi.x() - 10.0).abs() < 1e-5);
        assert!((hi.y() - 10.0).abs() < 1e-5);
        assert!((hi.z() - 10.0).abs() < 1e-5);
        assert_eq!(common.face_count(), 6);
        assert_eq!(common.vertex_count(), 8);
        assert_eq!(common.edge_count(), 12);
        assert!(
            common.is_watertight(),
            "intersection result must be watertight"
        );
    }

    #[test]
    fn test_boolean_difference_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Cut (Difference): cube1 - cube2
        let cut = boolean(&cube1, &cube2, BooleanOp::Cut);
        // The result should be a 5x10x10 box from [0, 0, 0] to [5, 10, 10]
        let (lo, hi) = cut.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 5.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert_eq!(cut.face_count(), 6);
        assert_eq!(cut.vertex_count(), 8);
        assert_eq!(cut.edge_count(), 12);
        assert!(cut.is_watertight(), "difference result must be watertight");
    }

    /// Box minus a through-cylinder. This exercises the curved-boolean path:
    /// the planar caps are bored (closed-circle hole cutting) and the cylinder
    /// lateral wall is trimmed where it crosses the caps.
    ///
    /// This used to be a known failure: the cylinder's three lateral faces share
    /// seam edges, and when topology stored a single orientation per edge, cutting
    /// one shared seam corrupted the adjacent walls' loops, leaving them untrimmed
    /// so the result spilled past z∈[0,10]. That is fixed: orientation is now a
    /// per-use property of each co-edge in a loop ([`OrientedEdge`]), not of the
    /// shared edge, so splitting a shared seam keeps every loop tracing cleanly.
    /// The `hi.z ≈ 10` assertion below (the drill extends to z=11) locks that in.
    ///
    /// [`OrientedEdge`]: openrcad_topo::arena::OrientedEdge
    #[test]
    fn test_boolean_box_minus_cylinder_drills_hole() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        // A 10×10×10 box with a radius-2 cylinder running through it along Z,
        // centred at (5,5), extending below and above the box so it pierces both
        // the top and bottom caps.
        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // The outer envelope is unchanged (the drill only removes interior
        // material), so the bounding box is still the original cube.
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((lo.z() - 0.0).abs() < 1e-4, "lo.z={}", lo.z());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());
        assert!((hi.z() - 10.0).abs() < 1e-4, "hi.z={}", hi.z());

        // A through-hole adds the cylindrical wall faces plus bores the two caps,
        // so the result must have strictly more faces than the original 6.
        assert!(
            result.face_count() > 6,
            "expected a drilled solid with extra faces, got {}",
            result.face_count()
        );
        if !result.is_watertight() {
            println!("All faces in the result solid:");
            for (idx, face) in result.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!(
                            "    Edge: {:?} -> {:?}",
                            edge.start().point(),
                            edge.end().point()
                        );
                    }
                }
            }
        }
        assert!(result.is_watertight(), "drilled solid must be watertight");
    }

    /// The curved-boolean engine completes without hanging (the historical
    /// blocker) and bores the box's caps. A lighter-weight companion to
    /// `test_boolean_box_minus_cylinder_drills_hole`, which additionally asserts
    /// the lateral walls are trimmed to z∈[0,10].
    #[test]
    fn test_boolean_box_minus_cylinder_runs_and_drills_caps() {
        use openrcad_foundation::{Ax2, Dir};
        use openrcad_primitives::make_cylinder;

        let box_solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(5.0, 5.0, -1.0), Dir::dz()), 2.0, 12.0);

        let result = boolean(&box_solid, &drill, BooleanOp::Cut);

        // Drilling removes no outer XY material: the footprint stays [0,10]².
        let (lo, hi) = result.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-4, "lo.x={}", lo.x());
        assert!((lo.y() - 0.0).abs() < 1e-4, "lo.y={}", lo.y());
        assert!((hi.x() - 10.0).abs() < 1e-4, "hi.x={}", hi.x());
        assert!((hi.y() - 10.0).abs() < 1e-4, "hi.y={}", hi.y());

        // The caps are bored and wall faces added, so the face count grows.
        assert!(
            result.face_count() > 6,
            "expected drilling to add faces, got {}",
            result.face_count()
        );
    }

    /// Two disjoint boxes packed into one shell split back into two solids,
    /// independent of the boolean engine. Locks in `split_disconnected`.
    #[test]
    fn split_disconnected_separates_two_packed_boxes() {
        let a = make_box(&Pnt::origin(), 5.0, 5.0, 5.0);
        let b = make_box(&Pnt::new(20.0, 0.0, 0.0), 5.0, 5.0, 5.0);
        let mut faces = a.shell().faces();
        faces.extend(b.shell().faces());
        let combined = Solid::new(Shell::from_faces(faces));
        // Combined shell: two euler-2 boxes → V−E+F = 16−24+12 = 4.
        assert_eq!(combined.euler_characteristic(), 4);

        let bodies = combined.split_disconnected();
        assert_eq!(bodies.len(), 2);
        for body in &bodies {
            assert_eq!(body.face_count(), 6);
            assert_eq!(body.euler_characteristic(), 2);
            assert!(body.is_watertight());
        }
    }

    /// A bar cut clean through the middle is severed into two separate bodies.
    /// `boolean` returns them as one shell (Euler=4); `boolean_bodies` splits it.
    #[test]
    fn cut_severing_a_bar_yields_two_bodies() {
        // A 30×10×10 bar along X, sliced by a tool that fully spans Y and Z and
        // removes x∈[10,20], leaving x∈[0,10] and x∈[20,30].
        let bar = make_box(&Pnt::origin(), 30.0, 10.0, 10.0);
        let knife = make_box(&Pnt::new(10.0, -1.0, -1.0), 10.0, 12.0, 12.0);

        let merged = boolean(&bar, &knife, BooleanOp::Cut);
        assert!(merged.is_watertight(), "severed cut should be watertight");

        let bodies = boolean_bodies(&bar, &knife, BooleanOp::Cut);
        assert_eq!(bodies.len(), 2, "a through-cut must produce two bodies");

        for body in &bodies {
            assert!(body.is_watertight(), "each severed body must be watertight");
            assert_eq!(body.euler_characteristic(), 2);
            let (lo, hi) = body.bounding_box().corners().unwrap();
            // Each piece is a 10×10×10 cube; only its X span differs.
            assert!(
                (hi.x() - lo.x() - 10.0).abs() < 1e-4,
                "x span {}",
                hi.x() - lo.x()
            );
            assert!((hi.y() - lo.y() - 10.0).abs() < 1e-4);
            assert!((hi.z() - lo.z() - 10.0).abs() < 1e-4);
        }

        // The two pieces sit at opposite ends of the original bar.
        let mut x_los: Vec<f64> = bodies
            .iter()
            .map(|b| b.bounding_box().corners().unwrap().0.x())
            .collect();
        x_los.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(
            (x_los[0] - 0.0).abs() < 1e-4,
            "near piece at x≈0, got {}",
            x_los[0]
        );
        assert!(
            (x_los[1] - 20.0).abs() < 1e-4,
            "far piece at x≈20, got {}",
            x_los[1]
        );
    }

    #[test]
    fn test_boolean_union_of_overlapping_cubes() {
        let cube1 = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let cube2 = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

        // Fuse (Union)
        let fuse = boolean(&cube1, &cube2, BooleanOp::Fuse);
        // The result bounding box should be [0, 0, 0] to [15, 10, 10]
        let (lo, hi) = fuse.bounding_box().corners().unwrap();
        assert!((lo.x() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.y() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((lo.z() - 0.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.x() - 15.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.y() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        assert!((hi.z() - 10.0).abs() < 1e-5, "lo={:?}, hi={:?}", lo, hi);
        if !fuse.is_watertight() {
            println!("All faces in the union solid:");
            for (idx, face) in fuse.shell().faces().iter().enumerate() {
                println!("Face {}:", idx);
                if let Some(surf) = face.surface() {
                    println!("  Surface: {:?}", surf);
                }
                for (w_idx, wire) in face.wires().iter().enumerate() {
                    println!("  Wire {}:", w_idx);
                    for edge in wire.edges() {
                        println!(
                            "    Edge: {:?} -> {:?}",
                            edge.start().point(),
                            edge.end().point()
                        );
                    }
                }
            }
        }
        assert!(fuse.is_watertight(), "union result must be watertight");
    }
}
