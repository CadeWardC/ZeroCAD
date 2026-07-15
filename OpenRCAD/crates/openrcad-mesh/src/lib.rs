#![forbid(unsafe_code)]
//! Tessellation for OpenRCAD (OCCT `TKMesh`).
//!
//! Converts analytic B-Rep surfaces into triangle meshes within a chordal
//! (distance) and angular error budget — the representation renderers and the
//! STL exporter consume.
//!
//! [`TriangleMesh`] is a serializable output type, and [`tessellate`] samples
//! each face independently before welding coincident mesh vertices into one
//! render/export mesh.

pub mod properties;
pub mod triangulate;

pub use properties::{mass_properties, MassProperties};

use openrcad_foundation::{
    BndBox, CancellationProbe, Cancelled, NeverCancelled, Pnt, TolerancePolicy,
    TolerancePolicyError, Trsf,
};
use openrcad_topo::{HealthReport, PcurveBuildError, ValidationError};
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

/// A triangle mesh: shared vertex positions plus integer triangles.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Vertex positions, de-duplicated where possible.
    pub vertices: Vec<Pnt>,
    /// Triangle indices into [`vertices`](Self::vertices), three per triangle.
    pub triangles: Vec<[u32; 3]>,
    /// Per-triangle source face index (parallel to [`triangles`](Self::triangles)).
    ///
    /// Each entry is the position of the originating face in
    /// `solid.shell().faces()`, so a renderer can map a picked triangle back to
    /// its topological [`Face`](openrcad_topo::Face). Empty when provenance was
    /// not tracked (e.g. meshes built directly via [`from_buffers`](Self::from_buffers)).
    pub face_ids: Vec<u32>,
}

/// GPU-ready, flat-shaded render buffers derived from a [`TriangleMesh`].
///
/// Triangles are *unwelded*: every triangle contributes three unique vertices
/// that all share the triangle's geometric (face) normal. This produces the
/// crisp, faceted "CAD look" — adjacent coplanar triangles read flat and sharp
/// edges stay sharp — at the cost of not sharing vertices between triangles.
///
/// All buffers use `f32`/`u32` for direct upload to a graphics API. No GPU types
/// leak into this crate; the renderer interprets these as it sees fit.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GpuMesh {
    /// Vertex positions, `[x, y, z]` per vertex, `3 * 3 * triangle_count` long.
    pub positions: Vec<f32>,
    /// Per-vertex normals, `[x, y, z]`, parallel to [`positions`](Self::positions).
    /// All three vertices of a triangle share its flat face normal.
    pub normals: Vec<f32>,
    /// Triangle indices: `0, 1, 2, 3, …` since vertices are not shared.
    pub indices: Vec<u32>,
    /// Per-triangle source face index, for pick/selection buffers.
    /// One entry per triangle (`indices.len() / 3`).
    pub face_ids: Vec<u32>,
}

/// Failure from strict Phase 1 tessellation.
#[derive(Clone, Debug, PartialEq)]
pub enum TessellationError {
    InvalidTolerancePolicy(TolerancePolicyError),
    InvalidBudget,
    InvalidTopology(ValidationError),
    UnhealthySolid(HealthReport),
    NonWatertightSolid(HealthReport),
    PcurveBuild(PcurveBuildError),
    Cancelled,
    InvalidMesh { degenerate_triangles: usize },
}

impl core::fmt::Display for TessellationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "invalid tessellation tolerance policy: {error}")
            }
            Self::InvalidBudget => {
                f.write_str("tessellation chord and angle errors must be finite and positive")
            }
            Self::InvalidTopology(error) => write!(f, "invalid tessellation input: {error}"),
            Self::UnhealthySolid(report) => {
                write!(f, "unhealthy tessellation input: {report:?}")
            }
            Self::NonWatertightSolid(report) => {
                write!(f, "non-watertight tessellation input: {report:?}")
            }
            Self::PcurveBuild(error) => {
                write!(f, "compatibility pcurve reconstruction failed: {error}")
            }
            Self::Cancelled => f.write_str("tessellation cancelled"),
            Self::InvalidMesh {
                degenerate_triangles,
            } => write!(
                f,
                "tessellation emitted {degenerate_triangles} degenerate triangles"
            ),
        }
    }
}

impl std::error::Error for TessellationError {}

impl From<Cancelled> for TessellationError {
    fn from(_: Cancelled) -> Self {
        Self::Cancelled
    }
}

impl TriangleMesh {
    /// An empty mesh.
    #[inline]
    pub const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            face_ids: Vec::new(),
        }
    }

    /// Build from raw vertex and triangle buffers, without face provenance.
    pub fn from_buffers(vertices: Vec<Pnt>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            face_ids: Vec::new(),
        }
    }

    /// Build from raw buffers plus a per-triangle source face index.
    ///
    /// `face_ids` must have one entry per triangle (or be empty).
    pub fn from_buffers_with_faces(
        vertices: Vec<Pnt>,
        triangles: Vec<[u32; 3]>,
        face_ids: Vec<u32>,
    ) -> Self {
        debug_assert!(face_ids.is_empty() || face_ids.len() == triangles.len());
        Self {
            vertices,
            triangles,
            face_ids,
        }
    }

    /// Number of vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of triangles.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Apply `t` to every vertex.
    pub fn transformed(&self, t: &Trsf) -> Self {
        Self {
            vertices: self.vertices.iter().map(|p| t.transform_point(p)).collect(),
            triangles: self.triangles.clone(),
            face_ids: self.face_ids.clone(),
        }
    }

    /// Axis-aligned box over all vertices.
    pub fn bounding_box(&self) -> BndBox {
        let mut b = BndBox::new();
        for p in &self.vertices {
            b.add(p);
        }
        b
    }

    /// A flat `[x,y,z]` position buffer, suitable for upload to a GPU vertex
    /// array.
    pub fn flat_positions(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.vertices.len() * 3);
        for p in &self.vertices {
            out.push(p.x());
            out.push(p.y());
            out.push(p.z());
        }
        out
    }

    /// Build flat-shaded, GPU-ready render buffers (`f32` positions/normals,
    /// `u32` indices, per-triangle face ids).
    ///
    /// Each triangle is emitted as three independent vertices sharing the
    /// triangle's geometric normal, giving a faceted CAD appearance. Degenerate
    /// triangles (zero-area) get a zero normal rather than `NaN`. When
    /// [`face_ids`](Self::face_ids) is populated it is copied through for picking;
    /// otherwise every triangle is tagged `0`.
    pub fn gpu_mesh(&self) -> GpuMesh {
        let tri_count = self.triangles.len();
        let mut positions = Vec::with_capacity(tri_count * 9);
        let mut normals = Vec::with_capacity(tri_count * 9);
        let mut indices = Vec::with_capacity(tri_count * 3);
        let mut face_ids = Vec::with_capacity(tri_count);

        for (i, tri) in self.triangles.iter().enumerate() {
            let a = self.vertices[tri[0] as usize];
            let b = self.vertices[tri[1] as usize];
            let c = self.vertices[tri[2] as usize];

            // Geometric (flat) normal; zero for degenerate triangles.
            let ab = b - a;
            let ac = c - a;
            let n = ab.cross(&ac);
            let len = n.magnitude();
            let (nx, ny, nz) = if len > 0.0 {
                (
                    (n.x() / len) as f32,
                    (n.y() / len) as f32,
                    (n.z() / len) as f32,
                )
            } else {
                (0.0, 0.0, 0.0)
            };

            for p in [a, b, c] {
                positions.push(p.x() as f32);
                positions.push(p.y() as f32);
                positions.push(p.z() as f32);
                normals.push(nx);
                normals.push(ny);
                normals.push(nz);
            }

            let base = (i * 3) as u32;
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);

            face_ids.push(self.face_ids.get(i).copied().unwrap_or(0));
        }

        GpuMesh {
            positions,
            normals,
            indices,
            face_ids,
        }
    }
}

/// Tessellate `solid` into a triangle mesh within `chord_err` (max surface
/// deviation) and `angle_err` (radians; max tangent turn per curved-edge
/// segment — keeps small fillets as smooth as large bores).
///
/// Boundary edges are discretized ONCE, shared by both adjacent faces
/// ([`triangulate::shared_edge_polylines`]) — the "edges first, faces second"
/// meshing order production kernels use — so per-face tessellations agree
/// exactly along shared boundaries and the combined mesh has no cracks to
/// stitch by construction.
#[deprecated(
    note = "use tessellate_checked; this wrapper reconstructs missing pcurves, discards validation errors by panicking, and returns no strictness metadata"
)]
pub fn tessellate(solid: &openrcad_topo::Solid, chord_err: f64, angle_err: f64) -> TriangleMesh {
    tessellate_compatibility_with_policy_and_cancel(
        solid,
        chord_err,
        angle_err,
        &TolerancePolicy::STANDARD,
        &NeverCancelled,
    )
    .unwrap_or_else(|error| panic!("tessellate compatibility wrapper: {error}"))
}

/// Strict tessellation using the standard document tolerance policy.
pub fn tessellate_checked(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
) -> Result<TriangleMesh, TessellationError> {
    tessellate_checked_with_policy(solid, chord_err, angle_err, &TolerancePolicy::STANDARD)
}

/// Strict policy-aware tessellation. Every surface-backed coedge must already
/// carry a valid stored pcurve; this function never reconstructs boundaries.
pub fn tessellate_checked_with_policy(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    policy: &TolerancePolicy,
) -> Result<TriangleMesh, TessellationError> {
    tessellate_checked_with_policy_and_cancel(solid, chord_err, angle_err, policy, &NeverCancelled)
}

/// Cancellable strict tessellation using the standard tolerance policy.
pub fn tessellate_checked_with_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    cancel: &dyn CancellationProbe,
) -> Result<TriangleMesh, TessellationError> {
    tessellate_checked_with_policy_and_cancel(
        solid,
        chord_err,
        angle_err,
        &TolerancePolicy::STANDARD,
        cancel,
    )
}

/// Cancellable strict policy-aware tessellation.
pub fn tessellate_checked_with_policy_and_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<TriangleMesh, TessellationError> {
    validate_tessellation_input(solid, chord_err, angle_err, policy)?;
    let mesh = tessellate_with_cancel_configured(solid, chord_err, angle_err, cancel, false)?;
    validate_mesh(mesh, policy)
}

/// Explicit compatibility adapter for Phase 3 operations that do not yet
/// store pcurves. Missing or stale pcurves are attached and validated before
/// the strict tessellator is invoked.
pub fn tessellate_compatibility_with_policy_and_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<TriangleMesh, TessellationError> {
    cancel.check_cancelled()?;
    let solid = prepare_compatibility_solid(solid, policy)?;
    tessellate_checked_with_policy_and_cancel(&solid, chord_err, angle_err, policy, cancel)
}

/// Cancellable tessellation. Cancellation is checked around shared-boundary
/// construction, independently for every face (including rayon workers), and
/// before each global repair pass. No partial mesh is returned.
#[deprecated(
    note = "use tessellate_checked_with_cancel or the explicitly named compatibility adapter; this wrapper reconstructs missing pcurves and panics on validation failure"
)]
pub fn tessellate_with_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    cancel: &dyn openrcad_foundation::CancellationProbe,
) -> Result<TriangleMesh, openrcad_foundation::Cancelled> {
    match tessellate_compatibility_with_policy_and_cancel(
        solid,
        chord_err,
        angle_err,
        &TolerancePolicy::STANDARD,
        cancel,
    ) {
        Ok(mesh) => Ok(mesh),
        Err(TessellationError::Cancelled) => Err(Cancelled),
        Err(error) => panic!("tessellate_with_cancel compatibility wrapper: {error}"),
    }
}

/// Cancellable display tessellation that additionally bounds non-axial
/// diagonals on long cylindrical strips. This prevents visible diagonal shading
/// across trimmed cylinders while the standard tessellator remains compact for
/// geometric analysis and intermediate validation meshes.
#[deprecated(
    note = "use tessellate_compatibility_for_display_with_policy_and_cancel only for allowlisted Phase 3 producers; strict Phase 1 paths use checked tessellation"
)]
pub fn tessellate_for_display_with_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    cancel: &dyn openrcad_foundation::CancellationProbe,
) -> Result<TriangleMesh, openrcad_foundation::Cancelled> {
    match tessellate_compatibility_for_display_with_policy_and_cancel(
        solid,
        chord_err,
        angle_err,
        &TolerancePolicy::STANDARD,
        cancel,
    ) {
        Ok(mesh) => Ok(mesh),
        Err(TessellationError::Cancelled) => Err(Cancelled),
        Err(error) => panic!("display tessellation compatibility wrapper: {error}"),
    }
}

/// Display-density variant of the explicit Phase 3 compatibility adapter.
pub fn tessellate_compatibility_for_display_with_policy_and_cancel(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    policy: &TolerancePolicy,
    cancel: &dyn CancellationProbe,
) -> Result<TriangleMesh, TessellationError> {
    cancel.check_cancelled()?;
    let solid = prepare_compatibility_solid(solid, policy)?;
    validate_tessellation_input(&solid, chord_err, angle_err, policy)?;
    let mesh = tessellate_with_cancel_configured(&solid, chord_err, angle_err, cancel, true)?;
    validate_mesh(mesh, policy)
}

/// Keep canonical Phase 1 solids on the projection-free path. Only a legacy
/// solid that fails strict pcurve validation enters reconstruction.
fn prepare_compatibility_solid(
    solid: &openrcad_topo::Solid,
    policy: &TolerancePolicy,
) -> Result<openrcad_topo::Solid, TessellationError> {
    if solid.validate_strict_with_policy(policy).is_ok() {
        return Ok(solid.clone());
    }
    solid
        .repair_pcurves(policy)
        .map(|(solid, _)| solid)
        .map_err(TessellationError::PcurveBuild)
}

fn validate_tessellation_input(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    policy: &TolerancePolicy,
) -> Result<(), TessellationError> {
    policy
        .validate()
        .map_err(TessellationError::InvalidTolerancePolicy)?;
    if !(chord_err.is_finite() && chord_err > 0.0 && angle_err.is_finite() && angle_err > 0.0) {
        return Err(TessellationError::InvalidBudget);
    }
    solid
        .validate_strict_with_policy(policy)
        .map_err(TessellationError::InvalidTopology)?;
    let health = solid.health_report_with_policy(policy);
    if !health.is_healthy() {
        return Err(TessellationError::UnhealthySolid(health));
    }
    if !solid.is_watertight_with_policy(policy) {
        return Err(TessellationError::NonWatertightSolid(health));
    }
    Ok(())
}

fn validate_mesh(
    mut mesh: TriangleMesh,
    policy: &TolerancePolicy,
) -> Result<TriangleMesh, TessellationError> {
    let tracks_faces = mesh.face_ids.len() == mesh.triangles.len();
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut face_ids = Vec::with_capacity(mesh.face_ids.len());
    let mut malformed = 0usize;
    let mut degenerate = 0usize;
    for (index, triangle) in mesh.triangles.iter().copied().enumerate() {
        let [a, b, c] = triangle;
        let Some(a) = mesh.vertices.get(a as usize) else {
            malformed += 1;
            continue;
        };
        let Some(b) = mesh.vertices.get(b as usize) else {
            malformed += 1;
            continue;
        };
        let Some(c) = mesh.vertices.get(c as usize) else {
            malformed += 1;
            continue;
        };
        let finite = [a, b, c]
            .iter()
            .all(|point| point.x().is_finite() && point.y().is_finite() && point.z().is_finite());
        if !finite {
            malformed += 1;
            continue;
        }
        if (*b - *a).cross(&(*c - *a)).magnitude() <= policy.resolution * policy.resolution {
            // A zero-area triangle contributes no surface and cannot close a
            // crack. Triangulators may create one while fanning a collinear
            // boundary chain; discard it before the strict mesh leaves the
            // kernel, keeping face provenance aligned.
            degenerate += 1;
            continue;
        }
        triangles.push(triangle);
        if tracks_faces {
            face_ids.push(mesh.face_ids[index]);
        }
    }
    if malformed > 0 || (triangles.is_empty() && degenerate > 0) {
        return Err(TessellationError::InvalidMesh {
            degenerate_triangles: malformed + degenerate,
        });
    }
    mesh.triangles = triangles;
    if tracks_faces {
        mesh.face_ids = face_ids;
    }
    Ok(mesh)
}

fn tessellate_with_cancel_configured(
    solid: &openrcad_topo::Solid,
    chord_err: f64,
    angle_err: f64,
    cancel: &dyn openrcad_foundation::CancellationProbe,
    bound_cylinder_diagonals: bool,
) -> Result<TriangleMesh, openrcad_foundation::Cancelled> {
    cancel.check_cancelled()?;
    let faces = solid.shell().faces();
    let shared = triangulate::shared_edge_polylines(&faces, chord_err, angle_err);
    cancel.check_cancelled()?;

    // Faces tessellate independently, so this parallelises cleanly across the
    // shell. The `parallel` feature (on by default) maps each face on a rayon
    // pool; with it disabled the identical work runs sequentially.
    #[cfg(feature = "parallel")]
    let meshes: Result<Vec<TriangleMesh>, openrcad_foundation::Cancelled> = {
        const PARALLEL_FACE_THRESHOLD: usize = 16;
        if faces.len() >= PARALLEL_FACE_THRESHOLD {
            faces
                .par_iter()
                .enumerate()
                .map(|(i, face)| {
                    cancel.check_cancelled()?;
                    Ok(triangulate::tessellate_face_budget_configured(
                        face,
                        chord_err,
                        angle_err,
                        i as u32,
                        Some(&shared),
                        bound_cylinder_diagonals,
                    ))
                })
                .collect()
        } else {
            faces
                .iter()
                .enumerate()
                .map(|(i, face)| {
                    cancel.check_cancelled()?;
                    Ok(triangulate::tessellate_face_budget_configured(
                        face,
                        chord_err,
                        angle_err,
                        i as u32,
                        Some(&shared),
                        bound_cylinder_diagonals,
                    ))
                })
                .collect()
        }
    };
    #[cfg(not(feature = "parallel"))]
    let meshes: Result<Vec<TriangleMesh>, openrcad_foundation::Cancelled> = faces
        .iter()
        .enumerate()
        .map(|(i, face)| {
            cancel.check_cancelled()?;
            Ok(triangulate::tessellate_face_budget_configured(
                face,
                chord_err,
                angle_err,
                i as u32,
                Some(&shared),
                bound_cylinder_diagonals,
            ))
        })
        .collect();

    let meshes = meshes?;
    cancel.check_cancelled()?;
    let mut combined = triangulate::combine(&meshes);
    // Safety net for boundaries the shared-edge pass could not cover (edges
    // whose geometric keys did not match across faces): stitch lens cracks
    // where two faces sampled a shared boundary differently.
    triangulate::stitch_boundary_lenses(&mut combined);
    cancel.check_cancelled()?;
    triangulate::refine_cylinder_mesh_edges(&mut combined, &faces, chord_err);
    // Cylinder-strip refinement can subdivide one side of a trim after the
    // first global stitch pass. Reconcile those newly introduced boundary
    // samples as the final meshing step so display-density output remains
    // crack-free too.
    if bound_cylinder_diagonals {
        triangulate::stitch_boundary_lenses(&mut combined);
    }
    cancel.check_cancelled()?;
    Ok(combined)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax2, Dir, Trsf, Vec};
    use std::collections::HashMap;
    use std::sync::Arc;

    #[test]
    fn cancelled_tessellation_returns_no_partial_mesh() {
        let solid = openrcad_primitives::make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let token = openrcad_foundation::CancellationToken::new();
        token.cancel();
        assert_eq!(
            tessellate_with_cancel(&solid, 0.05, 0.5, &token),
            Err(openrcad_foundation::Cancelled)
        );
    }

    #[test]
    fn strict_tessellation_rejects_missing_pcurve_before_meshing() {
        let solid = openrcad_primitives::make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0)
            .unwrap()
            .value;
        let mut brep = solid.brep().as_ref().clone();
        let coedge = brep
            .loops
            .values_mut()
            .find_map(|wire| wire.edges.first_mut())
            .unwrap();
        coedge.pcurve = None;
        let legacy = openrcad_topo::Solid::from_id(Arc::new(brep), solid.id());

        assert!(matches!(
            tessellate_checked(&legacy, 0.05, 0.5),
            Err(TessellationError::InvalidTopology(
                ValidationError::MissingPcurve { .. }
            ))
        ));

        let repaired = tessellate_compatibility_with_policy_and_cancel(
            &legacy,
            0.05,
            0.5,
            &TolerancePolicy::STANDARD,
            &NeverCancelled,
        )
        .unwrap();
        assert!(!repaired.triangles.is_empty());
    }

    #[test]
    fn checked_tessellation_cancellation_returns_no_partial_mesh() {
        let solid = openrcad_primitives::make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0)
            .unwrap()
            .value;
        let token = openrcad_foundation::CancellationToken::new();
        token.cancel();
        assert_eq!(
            tessellate_checked_with_cancel(&solid, 0.05, 0.5, &token),
            Err(TessellationError::Cancelled)
        );
    }

    #[test]
    fn plain_short_cylinder_has_a_compact_closed_display_mesh() {
        let solid =
            openrcad_primitives::make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 7.5, 2.0);
        let angle_err = std::f64::consts::TAU / 48.0;
        let faces = solid.shell().faces();
        let shared = triangulate::shared_edge_polylines(&faces, 0.05, angle_err);
        let local_counts: std::vec::Vec<(usize, usize)> = faces
            .iter()
            .enumerate()
            .map(|(index, face)| {
                let local = triangulate::tessellate_face_budget(
                    face,
                    0.05,
                    angle_err,
                    index as u32,
                    Some(&shared),
                );
                (local.vertex_count(), local.triangle_count())
            })
            .collect();
        let mesh = tessellate(&solid, 0.05, angle_err);

        // A 48-sided cylinder needs 96 wall triangles plus 48 per cap. Reject
        // the old fixed eight-row wall grid (1,110 triangles / 3,330 unwelded
        // display vertices).
        assert_eq!(
            mesh.triangle_count(),
            192,
            "plain cylinder should be a 48-segment wall plus two caps, got {} vertices / {} triangles; faces={local_counts:?}",
            mesh.vertex_count(),
            mesh.triangle_count(),
        );
        assert_eq!(mesh.gpu_mesh().positions.len() / 3, 576);

        // Every indexed edge of a closed cylinder is shared by exactly two
        // triangles; reducing support rows must not trade density for cracks.
        let mut edge_uses: HashMap<(u32, u32), usize> = HashMap::new();
        for triangle in &mesh.triangles {
            for (a, b) in [
                (triangle[0], triangle[1]),
                (triangle[1], triangle[2]),
                (triangle[2], triangle[0]),
            ] {
                *edge_uses.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        assert!(
            edge_uses.values().all(|&uses| uses == 2),
            "compact cylinder mesh must remain closed"
        );
    }

    #[test]
    fn mesh_counts_and_flat_positions() {
        let m = TriangleMesh::from_buffers(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(
            m.flat_positions(),
            vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        );
    }

    #[test]
    fn gpu_mesh_is_flat_shaded_and_carries_face_ids() {
        // A single triangle in the z=0 plane, tagged as face 7.
        let m = TriangleMesh::from_buffers_with_faces(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
            vec![7],
        );
        let g = m.gpu_mesh();
        // Unwelded: 3 vertices, 3 indices, 1 face id.
        assert_eq!(g.positions.len(), 9);
        assert_eq!(g.normals.len(), 9);
        assert_eq!(g.indices, vec![0, 1, 2]);
        assert_eq!(g.face_ids, vec![7]);
        // CCW triangle in z=0 → +Z normal, shared by all 3 vertices.
        for v in 0..3 {
            assert!((g.normals[v * 3] - 0.0).abs() < 1e-6);
            assert!((g.normals[v * 3 + 1] - 0.0).abs() < 1e-6);
            assert!((g.normals[v * 3 + 2] - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn gpu_mesh_degenerate_triangle_has_zero_normal() {
        // Collinear points → zero-area triangle, must not produce NaN.
        let m = TriangleMesh::from_buffers(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(2.0, 0.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let g = m.gpu_mesh();
        assert!(g.normals.iter().all(|n| n.is_finite()));
        assert_eq!(g.normals, vec![0.0; 9]);
        // No provenance → default face id 0.
        assert_eq!(g.face_ids, vec![0]);
    }

    #[test]
    fn transformed_mesh_moves_vertices() {
        let m = TriangleMesh::from_buffers(vec![Pnt::origin()], vec![]);
        let up = Trsf::translation(Vec::new(1.0, 2.0, 3.0));
        let m2 = m.transformed(&up);
        assert_eq!(m2.vertices[0], Pnt::new(1.0, 2.0, 3.0));
        // triangles unchanged.
        assert!(m2.triangles.is_empty());
    }

    #[test]
    fn bounding_box_covers_vertices() {
        let m = TriangleMesh::from_buffers(
            vec![Pnt::new(-1.0, -2.0, -3.0), Pnt::new(4.0, 5.0, 6.0)],
            vec![],
        );
        let (lo, hi) = m.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::new(-1.0, -2.0, -3.0));
        assert_eq!(hi, Pnt::new(4.0, 5.0, 6.0));
    }
}
