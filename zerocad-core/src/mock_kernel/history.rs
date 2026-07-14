//! Boolean face **history** — an input→output face correspondence, the modeling
//! layer's equivalent of OCCT's `Modified`/`Generated`/`IsDeleted` history.
//!
//! Persistent topological naming needs to know, for each face of a boolean
//! result, which input face it descended from, so a face's durable name (and the
//! edges derived from it) survive a cut/join. The OpenRCAD boolean discards that
//! provenance internally (partition → classify → sew → merge each allocate fresh
//! `FaceId`s with no parent link), and `Face`/`FaceData` carry no attribute field
//! to thread it through.
//!
//! Rather than surgically thread provenance through the kernel pipeline (invasive,
//! and it must stay clippy-clean), we recover the correspondence **post-hoc**: a
//! boolean splits and re-trims faces but never changes the *supporting surface* a
//! face lies on. So each result face is matched to the input face sharing its
//! surface identity (plane: normal + offset; cylinder: axis line + radius). Object
//! faces win ties over tool faces (the "earliest feature owns the merged face"
//! rule), which also resolves the coincident-surface case of a coplanar join.
//!
//! Result faces are indexed the same way `solid_to_flat_mesh` assigns
//! `face_ids` — the position in `solid.shell().faces()` — so the caller can map a
//! result mesh face ref straight onto its source input face.

use super::*;
use openrcad::geom::GeomSurface;

/// Which input face a boolean-result face descended from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceSource {
    /// Index into the object solid's `shell().faces()`.
    Object(usize),
    /// Index into the tool solid's `shell().faces()`.
    Tool(usize),
}

/// Input→output face correspondence for one boolean op, indexed by **result face
/// index** (== the mesh `face_id` from `solid_to_flat_mesh`).
#[derive(Debug, Clone, Default)]
pub struct BooleanHistory {
    /// For each result face, the input face it came from — or `None` when no input
    /// face shares its surface (a genuinely new/unrecognized face).
    pub face_source: Vec<Option<FaceSource>>,
}

impl BooleanHistory {
    /// The source of result face `i`, if known.
    pub fn source_of(&self, result_face_index: usize) -> Option<FaceSource> {
        self.face_source.get(result_face_index).copied().flatten()
    }
}

/// A canonical, quantized identity for the analytic surfaces ZeroCAD uses. Two
/// faces share a surface iff their signatures are equal. Non-analytic surfaces
/// return `None` (they fall through to no-match, handled by the caller).
#[derive(Clone, PartialEq, Eq)]
enum SurfaceSig {
    Plane {
        n: (i64, i64, i64),
        offset: i64,
    },
    Cylinder {
        foot: (i64, i64, i64),
        dir: (i64, i64, i64),
        radius: i64,
    },
}

fn q(v: f64) -> i64 {
    (v * 1.0e4).round() as i64
}

/// Sign-normalize a direction so a vector and its negation hash identically (a
/// plane/cylinder is unoriented for identity purposes — a boolean may flip a
/// face's stored normal without changing which surface it lies on).
fn sign_normalize(mut x: f64, mut y: f64, mut z: f64) -> (f64, f64, f64) {
    let lead = if x.abs() > 1e-9 {
        x
    } else if y.abs() > 1e-9 {
        y
    } else {
        z
    };
    if lead < 0.0 {
        x = -x;
        y = -y;
        z = -z;
    }
    (x, y, z)
}

fn surface_sig(surface: Option<&GeomSurface>) -> Option<SurfaceSig> {
    match surface? {
        GeomSurface::Plane(p) => {
            let n = p.normal();
            let (nx, ny, nz) = sign_normalize(n.x(), n.y(), n.z());
            let loc = p.location();
            let offset = loc.x() * nx + loc.y() * ny + loc.z() * nz;
            Some(SurfaceSig::Plane {
                n: (q(nx), q(ny), q(nz)),
                offset: q(offset),
            })
        }
        GeomSurface::Cylinder(c) => {
            let pos = c.position();
            let d = pos.direction();
            let (dx, dy, dz) = sign_normalize(d.x(), d.y(), d.z());
            let loc = pos.location();
            // Canonicalize the axis point to the foot of the perpendicular from the
            // origin so any generator naming the same axis line hashes equal.
            let t = loc.x() * dx + loc.y() * dy + loc.z() * dz;
            let foot = (
                q(loc.x() - dx * t),
                q(loc.y() - dy * t),
                q(loc.z() - dz * t),
            );
            Some(SurfaceSig::Cylinder {
                foot,
                dir: (q(dx), q(dy), q(dz)),
                radius: q(c.radius()),
            })
        }
        _ => None,
    }
}

/// Compute the input→output face correspondence for `result = op(obj, tool)` by
/// matching each result face to the input face sharing its supporting surface.
pub fn boolean_face_history(
    obj: &KernelSolid,
    tool: &KernelSolid,
    result: &KernelSolid,
) -> BooleanHistory {
    let obj_faces = obj.shell().faces();
    let tool_faces = tool.shell().faces();
    let obj_sigs: Vec<Option<SurfaceSig>> =
        obj_faces.iter().map(|f| surface_sig(f.surface())).collect();
    let tool_sigs: Vec<Option<SurfaceSig>> = tool_faces
        .iter()
        .map(|f| surface_sig(f.surface()))
        .collect();

    let face_source = result
        .shell()
        .faces()
        .iter()
        .map(|rf| {
            let sig = surface_sig(rf.surface())?;
            // Object faces win ties (earliest-feature-owns rule).
            if let Some(j) = obj_sigs.iter().position(|s| s.as_ref() == Some(&sig)) {
                return Some(FaceSource::Object(j));
            }
            if let Some(j) = tool_sigs.iter().position(|s| s.as_ref() == Some(&sig)) {
                return Some(FaceSource::Tool(j));
            }
            None
        })
        .collect();

    BooleanHistory { face_source }
}

/// For each face of `result`, the index of the face in `source` sharing its
/// supporting surface (or `None`). Unlike [`boolean_face_history`] this ignores
/// the tool — for *name propagation* only the faces that descend from the named
/// source body matter; a fresh cut wall (on the tool's surface) matches nothing
/// and stays unnamed, which is correct.
pub fn match_result_faces_to_source(
    source: &KernelSolid,
    result: &KernelSolid,
) -> Vec<Option<usize>> {
    let src_sigs: Vec<Option<SurfaceSig>> = source
        .shell()
        .faces()
        .iter()
        .map(|f| surface_sig(f.surface()))
        .collect();
    result
        .shell()
        .faces()
        .iter()
        .map(|rf| {
            let sig = surface_sig(rf.surface())?;
            src_sigs.iter().position(|s| s.as_ref() == Some(&sig))
        })
        .collect()
}

/// Tessellate `result_solid` and carry each named face of `input_mesh` onto the
/// result face that continues it — the propagation that lets a captured face
/// survive a boolean.
///
/// Matching is done in **mesh space** to avoid coupling to kernel face-index
/// ordering (a body's display mesh and its part solid need not enumerate faces
/// the same way): a result face inherits an input face's name when their outward
/// normals align *and* the result face's centroid lies on the input face's plane.
/// This is exact for the planar caps/sides that carry names (a boolean never
/// moves a surviving planar face off its plane); a split input face passes its
/// name to every piece. Result faces with no named planar match (new cut/boss
/// walls, curved faces) stay unnamed, which is correct. Geometry is identical to
/// [`MockMesh::from_solid`]; only `face_refs` names are added.
pub fn propagate_face_names(
    input_mesh: &MockMesh,
    result_solid: &KernelSolid,
    body_id: &str,
) -> MockMesh {
    let mut mesh = MockMesh::from_solid(result_solid);
    for face_ref in &mut mesh.face_refs {
        if let Some(name) = matching_input_face_name(input_mesh, face_ref.centroid, face_ref.normal)
        {
            face_ref.topology = Some(MeshTopologyFaceRef {
                body_id: Some(body_id.to_string()),
                component_id: None,
                topology_version: Some(0),
                face_id: Some(name),
                surface_kind: None,
            });
        }
    }
    // Now that result faces are named, give each edge its adjacent face-owner pair
    // so edges reattach through the boolean by identity too.
    populate_edge_adjacent_face_names(&mut mesh);
    mesh
}

/// The name of the input face this `(centroid, normal)` continues: same outward
/// direction and the centroid lying on the input face's plane.
///
/// Two *coplanar* named input faces (a flush boss top beside the base's top, a
/// severed face's two halves) are indistinguishable by plane alone — the old
/// closest-plane tie could hand a result face the *neighbour's* owner, exactly
/// the wrong-entity substitution the naming work exists to prevent. So when
/// more than one candidate shares the plane, the tie breaks to the input face
/// whose actual triangle footprint is closest to (ideally contains) the result
/// face's centroid.
fn matching_input_face_name(
    input_mesh: &MockMesh,
    centroid: [f32; 3],
    normal: [f32; 3],
) -> Option<String> {
    let mut candidates: Vec<(f32, u32, String)> = Vec::new();
    for i in &input_mesh.face_refs {
        let Some(name) = i.topology.as_ref().and_then(|t| t.face_id.clone()) else {
            continue;
        };
        let ndot = normal[0] * i.normal[0] + normal[1] * i.normal[1] + normal[2] * i.normal[2];
        if ndot < 0.99 {
            continue;
        }
        let dist = ((centroid[0] - i.centroid[0]) * i.normal[0]
            + (centroid[1] - i.centroid[1]) * i.normal[1]
            + (centroid[2] - i.centroid[2]) * i.normal[2])
            .abs();
        if dist > 1.0e-2 {
            continue;
        }
        candidates.push((dist, i.face_id, name));
    }
    match candidates.len() {
        0 => None,
        1 => Some(candidates.remove(0).2),
        _ => {
            // Coplanar ambiguity: prefer the face whose triangles the centroid
            // actually lies in/near, then the closest plane as the final tie.
            candidates
                .into_iter()
                .map(|(plane_dist, mesh_face_id, name)| {
                    let footprint =
                        distance_to_mesh_face_triangles(input_mesh, mesh_face_id, centroid);
                    (footprint, plane_dist, name)
                })
                .min_by(|a, b| {
                    (a.0, a.1)
                        .partial_cmp(&(b.0, b.1))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(_, _, name)| name)
        }
    }
}

/// Smallest distance from `p` to any triangle of the input-mesh face
/// `mesh_face_id` — 0 when the point lies inside the face's footprint.
fn distance_to_mesh_face_triangles(mesh: &MockMesh, mesh_face_id: u32, p: [f32; 3]) -> f32 {
    let mut best = f32::INFINITY;
    let tri_count = mesh.indices.len() / 3;
    for t in 0..tri_count {
        if mesh.face_ids.get(t) != Some(&mesh_face_id) {
            continue;
        }
        let v = |k: usize| -> [f32; 3] {
            let i = mesh.indices[t * 3 + k] as usize * 6;
            [mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]]
        };
        best = best.min(point_triangle_distance(p, v(0), v(1), v(2)));
        if best == 0.0 {
            return 0.0;
        }
    }
    best
}

/// Euclidean distance from a point to a triangle (Ericson, *Real-Time
/// Collision Detection* §5.1.5 closest-point construction).
pub(crate) fn point_triangle_distance(p: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f32 {
    let sub = |u: [f32; 3], v: [f32; 3]| [u[0] - v[0], u[1] - v[1], u[2] - v[2]];
    let dot = |u: [f32; 3], v: [f32; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    let closest = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let d3 = dot(ab, bp);
        let d4 = dot(ac, bp);
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                let t = d1 / (d1 - d3);
                [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]]
            } else {
                let cp = sub(p, c);
                let d5 = dot(ab, cp);
                let d6 = dot(ac, cp);
                if d6 >= 0.0 && d5 <= d6 {
                    c
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        let t = d2 / (d2 - d6);
                        [a[0] + t * ac[0], a[1] + t * ac[1], a[2] + t * ac[2]]
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            let bc = sub(c, b);
                            [b[0] + t * bc[0], b[1] + t * bc[1], b[2] + t * bc[2]]
                        } else {
                            let denom = 1.0 / (va + vb + vc);
                            let v = vb * denom;
                            let w = vc * denom;
                            [
                                a[0] + ab[0] * v + ac[0] * w,
                                a[1] + ab[1] * v + ac[1] * w,
                                a[2] + ab[2] * v + ac[2] * w,
                            ]
                        }
                    }
                }
            }
        }
    };
    let d = sub(p, closest);
    dot(d, d).sqrt()
}

/// The durable face name for each **shell face position** of `solid`, read off
/// its named display mesh. Names live on the mesh's canonical face ids (a
/// cylinder's arc-thirds share one id), so every shell face of a group reports
/// the group's name. Faces the matcher can't attribute stay `None`.
pub fn input_shell_face_names(input_mesh: &MockMesh, solid: &KernelSolid) -> Vec<Option<String>> {
    let mesh = MockMesh::from_solid(solid);
    let by_canonical: std::collections::HashMap<u32, Option<String>> = mesh
        .face_refs
        .iter()
        .map(|fr| {
            (
                fr.face_id,
                matching_input_face_name(input_mesh, fr.centroid, fr.normal),
            )
        })
        .collect();
    let groups = crate::mock_kernel::cylinder_surface_groups(solid);
    let mut canonical: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    for (fi, &g) in groups.iter().enumerate() {
        canonical.entry(g).or_insert(fi as u32);
    }
    (0..solid.shell().faces().len())
        .map(|i| {
            groups
                .get(i)
                .and_then(|g| canonical.get(g))
                .and_then(|c| by_canonical.get(c).cloned().flatten())
        })
        .collect()
}

/// Owner classes for the kernel's owner-aware merge: same name ⇒ same class,
/// unnamed ⇒ `None` (wildcard, merges as legacy geometry always has).
pub fn owner_classes_from_names(names: &[Option<String>]) -> Vec<Option<u64>> {
    use std::hash::{Hash, Hasher};
    names
        .iter()
        .map(|n| {
            n.as_ref().map(|name| {
                let mut h = std::collections::hash_map::DefaultHasher::new();
                name.hash(&mut h);
                h.finish()
            })
        })
        .collect()
}

/// Name a boolean result using the kernel's **exact** face history: a face
/// tracing to object input face `i` inherits `input_names[i]` (MODIFIED); a
/// face tracing to tool face `i` gets the durable generated name
/// `{generated_prefix}:tool-face:{i}` (GENERATED — e.g. a cut's bore wall,
/// which the geometric matcher must leave unnamed). Faces the history could
/// not attribute fall back to the geometric matcher; still-unnamed faces stay
/// unnamed — never a guessed identity.
pub fn propagate_face_names_via_history(
    input_mesh: &MockMesh,
    input_names: &[Option<String>],
    result_solid: &KernelSolid,
    history: &openrcad::algo::BooleanFaceHistory,
    body_id: &str,
    generated_prefix: &str,
) -> MockMesh {
    use openrcad::algo::BooleanFaceSource;
    let mut mesh = MockMesh::from_solid(result_solid);
    for face_ref in &mut mesh.face_refs {
        // The mesh's canonical face id IS a result shell-face position.
        let shell_idx = face_ref.face_id as usize;
        let name = match history.source_of(shell_idx) {
            Some(BooleanFaceSource::Object(i)) => input_names.get(i).cloned().flatten(),
            Some(BooleanFaceSource::Tool(i)) => Some(format!("{generated_prefix}:tool-face:{i}")),
            None => None,
        }
        .or_else(|| matching_input_face_name(input_mesh, face_ref.centroid, face_ref.normal));
        if let Some(name) = name {
            face_ref.topology = Some(MeshTopologyFaceRef {
                body_id: Some(body_id.to_string()),
                component_id: None,
                topology_version: Some(0),
                face_id: Some(name),
                surface_kind: None,
            });
        }
    }
    populate_edge_adjacent_face_names(&mut mesh);
    mesh
}

/// A canonical, position-based identity for a body **part** (one connected lump):
/// its quantized axis-aligned bounding box (min corner then max corner). Lumps in
/// different places get distinct keys, and the key is stable across rebuilds, so a
/// severing cut can hand back its parts in a deterministic order and a downstream
/// feature can follow a specific lump instead of guessing by list position.
pub fn part_key(solid: &KernelSolid) -> [i64; 6] {
    match crate::mock_kernel::solid_aabb(solid) {
        Some((mn, mx)) => {
            let q = |v: f32| (v as f64 * 1.0e4).round() as i64;
            [q(mn[0]), q(mn[1]), q(mn[2]), q(mx[0]), q(mx[1]), q(mx[2])]
        }
        None => [0; 6],
    }
}

/// Serialized, deterministic identity for one connected component of a body.
/// The component key is deliberately geometry-derived: cuts already sort their
/// severed results by this same quantized AABB, so selection and evaluation use
/// one canonical notion of component identity rather than vector position.
pub fn component_id(solid: &KernelSolid) -> String {
    let key = part_key(solid);
    format!(
        "{}:{}:{}:{}:{}:{}",
        key[0], key[1], key[2], key[3], key[4], key[5]
    )
}

/// Attach body + connected-component provenance to every selectable face in a
/// per-solid mesh without disturbing its durable design face name.
pub fn stamp_face_component(mesh: &mut MockMesh, body_id: &str, solid: &KernelSolid) {
    let component_id = component_id(solid);
    for face in &mut mesh.face_refs {
        let topology = face
            .topology
            .get_or_insert_with(MeshTopologyFaceRef::default);
        topology.body_id = Some(body_id.to_string());
        topology.component_id = Some(component_id.clone());
    }
}

/// Stamp a mesh that may already combine several disconnected body parts. Face
/// centroids are assigned to the nearest part AABB (zero distance when inside).
/// This is used for pristine analytic meshes whose face names predate the final
/// body assembly, and makes their selection metadata match kernel tessellation.
pub fn stamp_body_face_components(mesh: &mut MockMesh, body_id: &str, parts: &[KernelSolid]) {
    let components: Vec<([f32; 3], [f32; 3], String)> = parts
        .iter()
        .filter_map(|part| {
            crate::mock_kernel::solid_aabb(part).map(|(min, max)| (min, max, component_id(part)))
        })
        .collect();
    for face in &mut mesh.face_refs {
        let nearest = components.iter().min_by(|a, b| {
            let distance = |(min, max, _): &([f32; 3], [f32; 3], String)| {
                (0..3)
                    .map(|axis| {
                        if face.centroid[axis] < min[axis] {
                            (min[axis] - face.centroid[axis]).powi(2)
                        } else if face.centroid[axis] > max[axis] {
                            (face.centroid[axis] - max[axis]).powi(2)
                        } else {
                            0.0
                        }
                    })
                    .sum::<f32>()
            };
            distance(a)
                .partial_cmp(&distance(b))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let topology = face
            .topology
            .get_or_insert_with(MeshTopologyFaceRef::default);
        topology.body_id = Some(body_id.to_string());
        topology.component_id = nearest.map(|(_, _, id)| id.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad::foundation::{Trsf, Vec as GeomVec};

    /// Find the index of the result face whose supporting plane has |normal·axis|≈1
    /// on the given world axis and passes through `coord` on that axis.
    fn plane_face_at(result: &KernelSolid, axis: usize, coord: f64) -> Option<usize> {
        result.shell().faces().iter().position(|f| {
            let Some(GeomSurface::Plane(p)) = f.surface() else {
                return false;
            };
            let n = p.normal();
            let na = [n.x(), n.y(), n.z()][axis];
            if na.abs() < 0.99 {
                return false;
            }
            let loc = p.location();
            (([loc.x(), loc.y(), loc.z()][axis]) - coord).abs() < 1e-6
        })
    }

    #[test]
    fn union_maps_each_outer_face_to_its_source_solid() {
        // obj box [0,10]^3 ; tool box translated +5 in X -> [5,15]x[0,10]x[0,10].
        // Union is one [0,15] box. The x=0 face comes from obj, x=15 from tool.
        let obj = box_solid(10.0, 10.0, 10.0);
        let tool = box_solid(10.0, 10.0, 10.0)
            .transformed(&Trsf::translation(GeomVec::new(5.0, 0.0, 0.0)));
        let result = union(&obj, &tool).expect("axis-aligned box union should succeed");
        let hist = boolean_face_history(&obj, &tool, &result);

        let neg_x = plane_face_at(&result, 0, 0.0).expect("result has an x=0 face");
        let pos_x = plane_face_at(&result, 0, 15.0).expect("result has an x=15 face");
        assert!(
            matches!(hist.source_of(neg_x), Some(FaceSource::Object(_))),
            "x=0 outer face must trace to the object, got {:?}",
            hist.source_of(neg_x)
        );
        assert!(
            matches!(hist.source_of(pos_x), Some(FaceSource::Tool(_))),
            "x=15 outer face must trace to the tool, got {:?}",
            hist.source_of(pos_x)
        );
        // Every result face resolves to some input face (nothing unrecognized).
        assert!(
            hist.face_source.iter().all(|s| s.is_some()),
            "every union-result face should trace to an input face, got {:?}",
            hist.face_source
        );
    }

    #[test]
    fn difference_maps_hole_walls_to_the_tool() {
        // A 4x4 square pillar punched clean through the box in Z leaves 4 hole
        // walls, each on one of the tool's side planes -> Tool. The box's own outer
        // faces (possibly split) trace to the Object.
        let obj = box_solid(10.0, 10.0, 10.0);
        let tool =
            box_solid(4.0, 4.0, 12.0).transformed(&Trsf::translation(GeomVec::new(3.0, 3.0, -1.0)));
        let result = difference(&obj, &tool).expect("through-pocket difference should succeed");
        let hist = boolean_face_history(&obj, &tool, &result);

        let tool_walls = hist
            .face_source
            .iter()
            .filter(|s| matches!(s, Some(FaceSource::Tool(_))))
            .count();
        assert!(
            tool_walls >= 4,
            "the 4 hole walls must trace to the tool, got {tool_walls} tool-sourced faces in {:?}",
            hist.face_source
        );
        let obj_faces = hist
            .face_source
            .iter()
            .filter(|s| matches!(s, Some(FaceSource::Object(_))))
            .count();
        assert!(
            obj_faces >= 5,
            "the box's outer faces must trace to the object, got {obj_faces}"
        );
    }

    /// One flat quad (two triangles) at `z = 0` with normal +Z, spanning
    /// `x ∈ [x0, x1] × y ∈ [0, 10]`, appended to `mesh` as face `fid` named
    /// `name`.
    fn push_named_quad(mesh: &mut MockMesh, fid: u32, name: &str, x0: f32, x1: f32) {
        let base = (mesh.vertices.len() / 6) as u32;
        for (x, y) in [(x0, 0.0), (x1, 0.0), (x1, 10.0), (x0, 10.0)] {
            mesh.vertices.extend_from_slice(&[x, y, 0.0, 0.0, 0.0, 1.0]);
        }
        mesh.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        mesh.face_ids.extend_from_slice(&[fid, fid]);
        mesh.face_refs.push(MeshFaceRef {
            face_id: fid,
            centroid: [(x0 + x1) * 0.5, 5.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            topology: Some(MeshTopologyFaceRef {
                body_id: Some("b".to_string()),
                component_id: None,
                topology_version: Some(0),
                face_id: Some(name.to_string()),
                surface_kind: None,
            }),
        });
    }

    #[test]
    fn coplanar_faces_disambiguate_by_footprint_not_plane() {
        // Two same-plane, same-normal named faces side by side — the flush-boss /
        // severed-face configuration where plane+normal matching alone cannot
        // tell the owners apart. The centroid's containing footprint must win.
        let mut input = MockMesh::empty();
        push_named_quad(&mut input, 0, "owner:left", 0.0, 10.0);
        push_named_quad(&mut input, 1, "owner:right", 10.0, 20.0);

        assert_eq!(
            matching_input_face_name(&input, [5.0, 5.0, 0.0], [0.0, 0.0, 1.0]).as_deref(),
            Some("owner:left"),
            "a centroid inside the left face must inherit the LEFT owner"
        );
        assert_eq!(
            matching_input_face_name(&input, [15.0, 5.0, 0.0], [0.0, 0.0, 1.0]).as_deref(),
            Some("owner:right"),
            "a centroid inside the right face must inherit the RIGHT owner"
        );
    }

    #[test]
    fn severing_cut_yields_deterministic_ordered_parts() {
        // A slot x[9,11] slicing fully through a bar severs it into two lumps.
        let obj = box_solid(20.0, 10.0, 10.0);
        let tool = box_solid(2.0, 12.0, 12.0)
            .transformed(&Trsf::translation(GeomVec::new(9.0, -1.0, -1.0)));
        let parts = difference_bodies(&obj, &tool).expect("slot cut should succeed");
        assert_eq!(parts.len(), 2, "the slot must sever the bar into two lumps");

        let keys: Vec<[i64; 6]> = parts.iter().map(part_key).collect();
        assert_ne!(
            keys[0], keys[1],
            "the two lumps must have distinct identities"
        );
        assert!(
            keys[0] < keys[1],
            "parts must come back ordered by canonical key, got {keys:?}"
        );

        // Identity is stable across rebuilds — same order, same keys.
        let parts2 = difference_bodies(&obj, &tool).unwrap();
        let keys2: Vec<[i64; 6]> = parts2.iter().map(part_key).collect();
        assert_eq!(keys, keys2, "part identity must be stable across rebuilds");
    }
}
