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
    let tool_sigs: Vec<Option<SurfaceSig>> =
        tool_faces.iter().map(|f| surface_sig(f.surface())).collect();

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
pub fn match_result_faces_to_source(source: &KernelSolid, result: &KernelSolid) -> Vec<Option<usize>> {
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
/// direction and the centroid lying on the input face's plane. Ties break to the
/// closest plane.
fn matching_input_face_name(
    input_mesh: &MockMesh,
    centroid: [f32; 3],
    normal: [f32; 3],
) -> Option<String> {
    let mut best: Option<(f32, String)> = None;
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
        if best.as_ref().is_none_or(|(bd, _)| dist < *bd) {
            best = Some((dist, name));
        }
    }
    best.map(|(_, name)| name)
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
        let tool =
            box_solid(10.0, 10.0, 10.0).transformed(&Trsf::translation(GeomVec::new(5.0, 0.0, 0.0)));
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
        let tool = box_solid(4.0, 4.0, 12.0)
            .transformed(&Trsf::translation(GeomVec::new(3.0, 3.0, -1.0)));
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

    #[test]
    fn severing_cut_yields_deterministic_ordered_parts() {
        // A slot x[9,11] slicing fully through a bar severs it into two lumps.
        let obj = box_solid(20.0, 10.0, 10.0);
        let tool = box_solid(2.0, 12.0, 12.0)
            .transformed(&Trsf::translation(GeomVec::new(9.0, -1.0, -1.0)));
        let parts = difference_bodies(&obj, &tool).expect("slot cut should succeed");
        assert_eq!(parts.len(), 2, "the slot must sever the bar into two lumps");

        let keys: Vec<[i64; 6]> = parts.iter().map(part_key).collect();
        assert_ne!(keys[0], keys[1], "the two lumps must have distinct identities");
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
