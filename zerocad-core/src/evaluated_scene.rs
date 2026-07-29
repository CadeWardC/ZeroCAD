//! Renderer-facing evaluated scene.
//!
//! The parametric evaluator and the v5 hydrated cache still speak in part-body
//! meshes. This module is the single boundary that turns those meshes into
//! immutable local geometry plus placed scene instances. Parts currently emit
//! one identity instance per body; assemblies can later share one geometry
//! across many instances without changing viewport consumers.

use crate::mock_kernel::EdgeCurveHint;
use crate::MockMesh;
use std::collections::{HashMap, HashSet};
use std::fmt;

pub type SharedSceneGeometries = std::sync::Arc<Vec<(String, MockMesh)>>;

/// Explicit scene-complexity measures.
///
/// Pool totals describe all available local geometries, referenced totals count
/// each geometry used by the scene once, and instance-expanded totals multiply
/// geometry cost across every placed instance regardless of future visibility.
/// `instance_count` is deliberately not called an occurrence count: one future
/// assembly occurrence may contain several body-level scene instances.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SceneStats {
    pub geometry_pool_count: u64,
    pub geometry_pool_vertices: u64,
    pub geometry_pool_triangles: u64,
    pub referenced_geometry_count: u64,
    pub referenced_vertices: u64,
    pub referenced_triangles: u64,
    pub instance_count: u64,
    pub instance_expanded_vertices: u64,
    pub instance_expanded_triangles: u64,
}

/// A render-facing rigid transform.
///
/// The authoritative assembly placement will remain an f64 translation plus
/// quaternion. Evaluation converts that value into this compact f32 affine form
/// once, before any renderer, picker, bounds calculation, or thumbnail sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScenePlacement {
    linear: [[f32; 3]; 3],
    translation: [f32; 3],
}

impl ScenePlacement {
    pub const IDENTITY: Self = Self {
        linear: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation: [0.0; 3],
    };

    pub fn transform_point(self, point: [f32; 3]) -> [f32; 3] {
        let vector = self.transform_vector(point);
        [
            vector[0] + self.translation[0],
            vector[1] + self.translation[1],
            vector[2] + self.translation[2],
        ]
    }

    pub fn transform_vector(self, vector: [f32; 3]) -> [f32; 3] {
        [
            self.linear[0][0] * vector[0]
                + self.linear[0][1] * vector[1]
                + self.linear[0][2] * vector[2],
            self.linear[1][0] * vector[0]
                + self.linear[1][1] * vector[1]
                + self.linear[1][2] * vector[2],
            self.linear[2][0] * vector[0]
                + self.linear[2][1] * vector[1]
                + self.linear[2][2] * vector[2],
        ]
    }

    pub fn is_identity(self) -> bool {
        self == Self::IDENTITY
    }

    pub fn transform_bounds(self, min: [f32; 3], max: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        let mut world_min = [f32::INFINITY; 3];
        let mut world_max = [f32::NEG_INFINITY; 3];
        for &x in &[min[0], max[0]] {
            for &y in &[min[1], max[1]] {
                for &z in &[min[2], max[2]] {
                    let point = self.transform_point([x, y, z]);
                    for axis in 0..3 {
                        world_min[axis] = world_min[axis].min(point[axis]);
                        world_max[axis] = world_max[axis].max(point[axis]);
                    }
                }
            }
        }
        (world_min, world_max)
    }

    /// Stable bit fingerprint used by the current per-slot GPU lowering.
    ///
    /// Milestone 4 will move this transform into an instance buffer; until then,
    /// a placement change correctly invalidates only the affected lowered slot.
    pub fn fingerprint(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        for value in self.linear.iter().flatten().chain(self.translation.iter()) {
            hash ^= value.to_bits() as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        hash
    }

    /// Column-major local-to-world matrix for GPU instance records.
    pub fn to_column_major_matrix(self) -> [[f32; 4]; 4] {
        [
            [self.linear[0][0], self.linear[1][0], self.linear[2][0], 0.0],
            [self.linear[0][1], self.linear[1][1], self.linear[2][1], 0.0],
            [self.linear[0][2], self.linear[1][2], self.linear[2][2], 0.0],
            [
                self.translation[0],
                self.translation[1],
                self.translation[2],
                1.0,
            ],
        ]
    }

    /// Materialize a world-space mesh only for legacy algorithms that require
    /// one, such as the CPU section-plane clipper. Identity scenes avoid this
    /// copy at their call sites.
    pub fn transform_mesh(self, mesh: &MockMesh) -> MockMesh {
        let mut transformed = mesh.clone();
        // Deliberately exhaustive: adding a field to MockMesh must force an
        // explicit decision about whether rigid placement transforms it.
        let MockMesh {
            vertices,
            indices: _,
            edge_vertices,
            edge_indices: _,
            edge_face_normals,
            face_ids: _,
            edge_groups: _,
            edge_refs,
            face_refs,
        } = &mut transformed;

        for vertex in vertices.chunks_exact_mut(6) {
            let point = self.transform_point([vertex[0], vertex[1], vertex[2]]);
            let normal = self.transform_vector([vertex[3], vertex[4], vertex[5]]);
            vertex[..3].copy_from_slice(&point);
            vertex[3..6].copy_from_slice(&normal);
        }
        for vertex in edge_vertices.chunks_exact_mut(3) {
            let point = self.transform_point([vertex[0], vertex[1], vertex[2]]);
            vertex.copy_from_slice(&point);
        }
        for normals in edge_face_normals.chunks_exact_mut(3) {
            let normal = self.transform_vector([normals[0], normals[1], normals[2]]);
            normals.copy_from_slice(&normal);
        }
        for edge in edge_refs {
            edge.p0 = self.transform_point(edge.p0);
            edge.p1 = self.transform_point(edge.p1);
            edge.n1 = self.transform_vector(edge.n1);
            edge.n2 = self.transform_vector(edge.n2);
            // Deliberately exhaustive for the same reason as the MockMesh
            // destructure above: every future analytic curve hint must declare
            // how rigid placement affects its geometric data.
            match edge.curve.as_mut() {
                None | Some(EdgeCurveHint::Line) => {}
                Some(EdgeCurveHint::Circle {
                    center,
                    axis,
                    x_dir,
                    ..
                }) => {
                    *center = self.transform_point(*center);
                    *axis = self.transform_vector(*axis);
                    *x_dir = self.transform_vector(*x_dir);
                }
            }
        }
        for face in face_refs {
            face.centroid = self.transform_point(face.centroid);
            face.normal = self.transform_vector(face.normal);
        }
        transformed
    }

    pub fn from_rotation_translation(
        mut linear: [[f32; 3]; 3],
        mut translation: [f32; 3],
    ) -> Option<Self> {
        if !rigid_transform_is_valid(linear, translation) {
            return None;
        }
        canonicalize_signed_zero(&mut linear, &mut translation);
        Some(Self {
            linear,
            translation,
        })
    }
}

/// One placed, selectable occurrence of a local geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct SceneInstance {
    entity_id: String,
    geometry_index: usize,
    placement: ScenePlacement,
}

impl SceneInstance {
    pub fn new(
        entity_id: impl Into<String>,
        geometry_index: usize,
        placement: ScenePlacement,
    ) -> Self {
        Self {
            entity_id: entity_id.into(),
            geometry_index,
            placement,
        }
    }

    pub fn entity_id(&self) -> &str {
        &self.entity_id
    }

    pub fn geometry_index(&self) -> usize {
        self.geometry_index
    }

    pub fn placement(&self) -> ScenePlacement {
        self.placement
    }
}

/// Immutable local geometry and its placed scene instances.
#[derive(Debug)]
pub struct EvaluatedScene {
    geometries: SharedSceneGeometries,
    instances: Vec<SceneInstance>,
    entity_index: HashMap<String, usize>,
    world_bounds: Option<([f32; 3], [f32; 3])>,
    stats: SceneStats,
}

pub type SharedEvaluatedScene = std::sync::Arc<EvaluatedScene>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SceneBuildError {
    EmptyEntityId,
    DuplicateEntityId(String),
    MissingGeometry {
        entity_id: String,
        geometry_index: usize,
        geometry_count: usize,
    },
}

impl fmt::Display for SceneBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyEntityId => formatter.write_str("scene entity id cannot be empty"),
            Self::DuplicateEntityId(entity_id) => {
                write!(formatter, "duplicate scene entity id `{entity_id}`")
            }
            Self::MissingGeometry {
                entity_id,
                geometry_index,
                geometry_count,
            } => write!(
                formatter,
                "scene entity `{entity_id}` references geometry {geometry_index}, \
                 but only {geometry_count} geometries exist"
            ),
        }
    }
}

impl std::error::Error for SceneBuildError {}

impl EvaluatedScene {
    /// Adapt the current part evaluator output: every body is unique local
    /// geometry with one identity instance carrying the existing body id.
    pub fn from_part_bodies(geometries: SharedSceneGeometries) -> Self {
        let instances: Vec<SceneInstance> = geometries
            .iter()
            .enumerate()
            .map(|(geometry_index, (entity_id, _))| SceneInstance {
                entity_id: entity_id.clone(),
                geometry_index,
                placement: ScenePlacement::IDENTITY,
            })
            .collect();
        let world_bounds = scene_bounds(&geometries, &instances);
        let stats = scene_stats(&geometries, &instances);
        let entity_index = instances
            .iter()
            .enumerate()
            .map(|(index, instance)| (instance.entity_id.clone(), index))
            .collect::<HashMap<_, _>>();
        assert_eq!(
            entity_index.len(),
            instances.len(),
            "part evaluator emitted duplicate body entity ids"
        );
        Self {
            geometries,
            instances,
            entity_index,
            world_bounds,
            stats,
        }
    }

    pub fn from_instances(
        geometries: SharedSceneGeometries,
        instances: Vec<SceneInstance>,
    ) -> Result<Self, SceneBuildError> {
        validate_instances(&geometries, &instances)?;
        let world_bounds = scene_bounds(&geometries, &instances);
        let stats = scene_stats(&geometries, &instances);
        let entity_index = instances
            .iter()
            .enumerate()
            .map(|(index, instance)| (instance.entity_id.clone(), index))
            .collect();
        Ok(Self {
            geometries,
            instances,
            entity_index,
            world_bounds,
            stats,
        })
    }

    pub fn instances(&self) -> &[SceneInstance] {
        &self.instances
    }

    pub fn mesh(&self, instance: &SceneInstance) -> &MockMesh {
        &self.geometries[instance.geometry_index].1
    }

    pub fn find(&self, entity_id: &str) -> Option<(&SceneInstance, &MockMesh)> {
        let instance = &self.instances[*self.entity_index.get(entity_id)?];
        Some((instance, self.mesh(instance)))
    }

    pub fn geometries(&self) -> &SharedSceneGeometries {
        &self.geometries
    }

    /// Conservative world-space bounds. A local AABB is cached conceptually
    /// per geometry and transformed once per instance, so shared definitions
    /// do not repeat vertex work for every occurrence.
    pub fn world_bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        self.world_bounds
    }

    pub fn stats(&self) -> SceneStats {
        self.stats
    }

    pub fn is_empty(&self) -> bool {
        self.instances.is_empty()
    }
}

fn scene_stats(geometries: &[(String, MockMesh)], instances: &[SceneInstance]) -> SceneStats {
    let geometry_counts: Vec<(u64, u64)> = geometries
        .iter()
        .map(|(_, mesh)| {
            (
                count_to_u64(mesh.vertices.len() / 6),
                count_to_u64(mesh.indices.len() / 3),
            )
        })
        .collect();
    let mut stats = SceneStats {
        geometry_pool_count: count_to_u64(geometries.len()),
        ..SceneStats::default()
    };
    for &(vertices, triangles) in &geometry_counts {
        stats.geometry_pool_vertices = stats.geometry_pool_vertices.saturating_add(vertices);
        stats.geometry_pool_triangles = stats.geometry_pool_triangles.saturating_add(triangles);
    }

    let mut referenced = HashSet::with_capacity(instances.len().min(geometries.len()));
    for instance in instances {
        stats.instance_count = stats.instance_count.saturating_add(1);
        let Some(&(vertices, triangles)) = geometry_counts.get(instance.geometry_index) else {
            continue;
        };
        stats.instance_expanded_vertices =
            stats.instance_expanded_vertices.saturating_add(vertices);
        stats.instance_expanded_triangles =
            stats.instance_expanded_triangles.saturating_add(triangles);
        if referenced.insert(instance.geometry_index) {
            stats.referenced_geometry_count = stats.referenced_geometry_count.saturating_add(1);
            stats.referenced_vertices = stats.referenced_vertices.saturating_add(vertices);
            stats.referenced_triangles = stats.referenced_triangles.saturating_add(triangles);
        }
    }
    stats
}

fn count_to_u64(count: usize) -> u64 {
    u64::try_from(count).unwrap_or(u64::MAX)
}

fn rigid_transform_is_valid(linear: [[f32; 3]; 3], translation: [f32; 3]) -> bool {
    const TOLERANCE: f32 = 1.0e-4;
    if !linear
        .iter()
        .flatten()
        .chain(translation.iter())
        .all(|value| value.is_finite())
    {
        return false;
    }

    for row in &linear {
        let norm_squared = dot(*row, *row);
        if (norm_squared - 1.0).abs() > TOLERANCE {
            return false;
        }
    }
    for first in 0..3 {
        for second in first + 1..3 {
            if dot(linear[first], linear[second]).abs() > TOLERANCE {
                return false;
            }
        }
    }

    let determinant = linear[0][0] * (linear[1][1] * linear[2][2] - linear[1][2] * linear[2][1])
        - linear[0][1] * (linear[1][0] * linear[2][2] - linear[1][2] * linear[2][0])
        + linear[0][2] * (linear[1][0] * linear[2][1] - linear[1][1] * linear[2][0]);
    (determinant - 1.0).abs() <= TOLERANCE
}

fn dot(left: [f32; 3], right: [f32; 3]) -> f32 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn canonicalize_signed_zero(linear: &mut [[f32; 3]; 3], translation: &mut [f32; 3]) {
    for value in linear.iter_mut().flatten().chain(translation.iter_mut()) {
        if *value == 0.0 {
            *value = 0.0;
        }
    }
}

fn validate_instances(
    geometries: &[(String, MockMesh)],
    instances: &[SceneInstance],
) -> Result<(), SceneBuildError> {
    let mut entity_ids = HashSet::with_capacity(instances.len());
    for instance in instances {
        if instance.entity_id.is_empty() {
            return Err(SceneBuildError::EmptyEntityId);
        }
        if !entity_ids.insert(instance.entity_id.as_str()) {
            return Err(SceneBuildError::DuplicateEntityId(
                instance.entity_id.clone(),
            ));
        }
        if instance.geometry_index >= geometries.len() {
            return Err(SceneBuildError::MissingGeometry {
                entity_id: instance.entity_id.clone(),
                geometry_index: instance.geometry_index,
                geometry_count: geometries.len(),
            });
        }
    }
    Ok(())
}

fn scene_bounds(
    geometries: &[(String, MockMesh)],
    instances: &[SceneInstance],
) -> Option<([f32; 3], [f32; 3])> {
    // Compute each local geometry bound once. Reusing a detailed definition
    // across many occurrences stays O(vertices + occurrences), rather than
    // transforming every vertex again for every occurrence.
    let local_bounds: Vec<_> = geometries
        .iter()
        .map(|(_, mesh)| mesh_bounds(mesh))
        .collect();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for instance in instances {
        let Some((local_min, local_max)) =
            local_bounds.get(instance.geometry_index).copied().flatten()
        else {
            continue;
        };
        let (instance_min, instance_max) =
            instance.placement.transform_bounds(local_min, local_max);
        for axis in 0..3 {
            min[axis] = min[axis].min(instance_min[axis]);
            max[axis] = max[axis].max(instance_max[axis]);
        }
    }
    min[0].is_finite().then_some((min, max))
}

fn mesh_bounds(mesh: &MockMesh) -> Option<([f32; 3], [f32; 3])> {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex[axis]);
            max[axis] = max[axis].max(vertex[axis]);
        }
    }
    min[0].is_finite().then_some((min, max))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn part_adapter_preserves_ids_meshes_and_identity_bounds() {
        let bodies = std::sync::Arc::new(vec![
            ("first".into(), MockMesh::make_box(2.0, 4.0, 6.0)),
            ("second".into(), MockMesh::make_box(1.0, 1.0, 1.0)),
        ]);
        let scene = EvaluatedScene::from_part_bodies(bodies.clone());

        assert_eq!(scene.instances.len(), 2);
        assert_eq!(scene.instances[0].entity_id(), "first");
        assert_eq!(scene.instances[1].entity_id(), "second");
        assert!(scene
            .instances
            .iter()
            .all(|instance| instance.placement().is_identity()));
        assert!(std::sync::Arc::ptr_eq(scene.geometries(), &bodies));
        assert_eq!(
            scene.world_bounds(),
            Some(([0.0, 0.0, 0.0], [2.0, 4.0, 6.0]))
        );
        let stats = scene.stats();
        assert_eq!(stats.geometry_pool_count, 2);
        assert_eq!(stats.referenced_geometry_count, 2);
        assert_eq!(stats.instance_count, 2);
        assert_eq!(
            stats.instance_expanded_vertices,
            stats.geometry_pool_vertices
        );
        assert_eq!(
            stats.instance_expanded_triangles,
            stats.geometry_pool_triangles
        );
    }

    #[test]
    fn placement_transforms_points_vectors_mesh_metadata_and_bounds() {
        let placement = ScenePlacement::from_rotation_translation(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            [10.0, 20.0, 30.0],
        )
        .unwrap();
        assert_eq!(
            placement.transform_point([2.0, 3.0, 4.0]),
            [7.0, 22.0, 34.0]
        );
        assert_eq!(
            placement.transform_vector([2.0, 3.0, 4.0]),
            [-3.0, 2.0, 4.0]
        );

        let bodies = std::sync::Arc::new(vec![("box".into(), MockMesh::make_box(2.0, 4.0, 6.0))]);
        let scene = EvaluatedScene::from_instances(
            bodies,
            vec![SceneInstance::new("placed", 0, placement)],
        )
        .unwrap();
        assert_eq!(
            scene.world_bounds(),
            Some(([6.0, 20.0, 30.0], [10.0, 22.0, 36.0]))
        );

        let (_, mesh) = scene.find("placed").unwrap();
        let transformed = placement.transform_mesh(mesh);
        assert_eq!(transformed.face_refs.len(), mesh.face_refs.len());
        assert_ne!(transformed.vertices, mesh.vertices);
    }

    #[test]
    fn public_scene_boundary_rejects_non_rigid_and_ambiguous_instances() {
        assert!(ScenePlacement::from_rotation_translation(
            [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        )
        .is_none());
        assert!(ScenePlacement::from_rotation_translation(
            [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        )
        .is_none());

        let bodies = std::sync::Arc::new(vec![("box".into(), MockMesh::make_box(1.0, 1.0, 1.0))]);
        let duplicate = EvaluatedScene::from_instances(
            bodies.clone(),
            vec![
                SceneInstance::new("same", 0, ScenePlacement::IDENTITY),
                SceneInstance::new("same", 0, ScenePlacement::IDENTITY),
            ],
        )
        .unwrap_err();
        assert_eq!(duplicate, SceneBuildError::DuplicateEntityId("same".into()));

        let missing = EvaluatedScene::from_instances(
            bodies,
            vec![SceneInstance::new("missing", 1, ScenePlacement::IDENTITY)],
        )
        .unwrap_err();
        assert_eq!(
            missing,
            SceneBuildError::MissingGeometry {
                entity_id: "missing".into(),
                geometry_index: 1,
                geometry_count: 1,
            }
        );

        let canonical_zero = ScenePlacement::from_rotation_translation(
            [[1.0, -0.0, 0.0], [0.0, 1.0, -0.0], [-0.0, 0.0, 1.0]],
            [-0.0, 0.0, -0.0],
        )
        .unwrap();
        assert!(canonical_zero.is_identity());
        assert_eq!(
            canonical_zero.fingerprint(),
            ScenePlacement::IDENTITY.fingerprint()
        );
    }

    #[test]
    fn many_instances_share_one_geometry_and_bound_without_expansion() {
        let geometries =
            std::sync::Arc::new(vec![("box".into(), MockMesh::make_box(1.0, 1.0, 1.0))]);
        let instances = (0..1_000)
            .map(|index| {
                let placement = ScenePlacement::from_rotation_translation(
                    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                    [index as f32, 0.0, 0.0],
                )
                .unwrap();
                SceneInstance::new(format!("occurrence-{index}"), 0, placement)
            })
            .collect();
        let scene = EvaluatedScene::from_instances(geometries.clone(), instances).unwrap();

        assert_eq!(scene.geometries().len(), 1);
        assert_eq!(scene.instances().len(), 1_000);
        assert!(std::sync::Arc::ptr_eq(scene.geometries(), &geometries));
        assert_eq!(
            scene.world_bounds(),
            Some(([0.0, 0.0, 0.0], [1_000.0, 1.0, 1.0]))
        );
        let stats = scene.stats();
        assert_eq!(stats.geometry_pool_count, 1);
        assert_eq!(stats.referenced_geometry_count, 1);
        assert_eq!(stats.instance_count, 1_000);
        assert_eq!(
            stats.instance_expanded_vertices,
            stats.geometry_pool_vertices.saturating_mul(1_000)
        );
        assert_eq!(
            stats.instance_expanded_triangles,
            stats.geometry_pool_triangles.saturating_mul(1_000)
        );
    }

    #[test]
    fn stats_distinguish_unreferenced_pool_geometry_from_instance_expansion() {
        let geometries = std::sync::Arc::new(vec![
            ("used".into(), MockMesh::make_box(1.0, 1.0, 1.0)),
            ("unused".into(), MockMesh::make_box(5.0, 5.0, 5.0)),
        ]);
        let used_vertices = count_to_u64(geometries[0].1.vertices.len() / 6);
        let used_triangles = count_to_u64(geometries[0].1.indices.len() / 3);
        let scene = EvaluatedScene::from_instances(
            geometries,
            vec![SceneInstance::new("only-used", 0, ScenePlacement::IDENTITY)],
        )
        .unwrap();

        let stats = scene.stats();
        assert_eq!(stats.geometry_pool_count, 2);
        assert_eq!(stats.referenced_geometry_count, 1);
        assert_eq!(stats.instance_count, 1);
        assert_eq!(stats.referenced_vertices, used_vertices);
        assert_eq!(stats.referenced_triangles, used_triangles);
        assert_eq!(stats.instance_expanded_vertices, used_vertices);
        assert_eq!(stats.instance_expanded_triangles, used_triangles);
        assert!(stats.geometry_pool_vertices > stats.referenced_vertices);
    }
}
