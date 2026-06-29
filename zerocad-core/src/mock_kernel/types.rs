use super::*;

/// The kernel's solid type (an `openrcad` B-Rep solid). Re-exported so the
/// parametric evaluator can hold solids between features and combine them with
/// boolean operations (join/cut) before tessellating to a `MockMesh`.
pub type KernelSolid = Solid;

/// Analytic curve metadata for a selected display edge.
///
/// Viewport wireframes are rendered as line segments, but selected-edge CAD
/// operations need the original analytic curve when a group of segments is
/// really one circular rim. This hint is intentionally small and serializable so
/// parametric edge-mod features can persist the selected curve without storing
/// kernel topology ids.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind")]
pub enum EdgeCurveHint {
    Line,
    Circle {
        center: [f32; 3],
        axis: [f32; 3],
        x_dir: [f32; 3],
        radius: f32,
        start: f32,
        end: f32,
        closed: bool,
    },
}

/// Exact-ish selectable edge metadata aligned to a [`MockMesh::edge_groups`] id.
///
/// Wireframe segments are drawn as chords, but selection and later edge mods need
/// one stable topological edge record: endpoints, adjacent normals, and the
/// analytic curve when the selected group is a circular rim.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MeshEdgeRef {
    pub group: u32,
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub n1: [f32; 3],
    pub n2: [f32; 3],
    #[serde(default)]
    pub curve: Option<EdgeCurveHint>,
    #[serde(default)]
    pub topology: Option<MeshTopologyEdgeRef>,
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct MeshTopologyEdgeRef {
    #[serde(default)]
    pub body_id: Option<String>,
    #[serde(default)]
    pub topology_version: Option<u64>,
    #[serde(default)]
    pub edge_id: Option<String>,
    #[serde(default)]
    pub adjacent_face_ids: Vec<String>,
    #[serde(default)]
    pub curve_kind: Option<String>,
    #[serde(default)]
    pub adjacent_surface_kinds: Vec<String>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MockMesh {
    /// Interleaved [x, y, z, nx, ny, nz] per vertex.
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
    /// Flat [x, y, z] per vertex, paired in order for line segments.
    pub edge_vertices: Vec<f32>,
    /// Index pairs into `edge_vertices` (each consecutive pair = one line).
    pub edge_indices: Vec<u32>,
    /// The two adjacent face normals per edge, as [n1x,n1y,n1z, n2x,n2y,n2z].
    /// Length is `(edge_indices.len() / 2) * 6`. Used for hidden-line removal:
    /// an edge is hidden only when *both* its faces point away from the camera.
    /// May be empty for legacy meshes — renderers must treat empty as "no info".
    #[serde(default)]
    pub edge_face_normals: Vec<f32>,
    /// One B-rep face id per triangle (length == `indices.len() / 3`). Triangles
    /// sharing an id belong to the same planar/cylindrical face, which lets the
    /// viewport select a whole face at once. Ids are unique within a mesh and are
    /// rebased on `append`/merge so combined meshes keep distinct faces.
    #[serde(default)]
    pub face_ids: Vec<u32>,
    /// One group id per edge **segment** (length == `edge_indices.len() / 2`).
    /// Segments sharing an id form a single topological edge — every chord of one
    /// fillet arc, or the thirds of a cylinder's rim — so the viewport can select
    /// and highlight a whole curved edge at once instead of a lone chord. Empty on
    /// legacy/cached meshes; consumers then fall back to per-segment selection.
    #[serde(default)]
    pub edge_groups: Vec<u32>,
    /// One record per selectable edge group. New B-Rep tessellations populate
    /// this so the GUI can pass exact topological edge data into EdgeMod instead
    /// of re-inferring it from drawn chord segments.
    #[serde(default)]
    pub edge_refs: Vec<MeshEdgeRef>,
}

impl MockMesh {
    pub fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            edge_vertices: Vec::new(),
            edge_indices: Vec::new(),
            edge_face_normals: Vec::new(),
            face_ids: Vec::new(),
            edge_groups: Vec::new(),
            edge_refs: Vec::new(),
        }
    }

    /// Largest face id currently in this mesh, or `None` when there are no faces.
    fn max_face_id(&self) -> Option<u32> {
        self.face_ids.iter().copied().max()
    }

    /// Append another mesh into this one, rebasing its indices. Used to build a
    /// combined mesh (e.g. an extrude preview spanning several faces).
    pub fn append(&mut self, other: MockMesh) {
        let v_offset = (self.vertices.len() / 6) as u32;
        let e_offset = (self.edge_vertices.len() / 3) as u32;
        // Shift incoming face ids past ours so the two meshes' faces stay distinct.
        let f_offset = self.max_face_id().map_or(0, |m| m + 1);
        // Likewise shift edge group ids so the two meshes' topological edges
        // (e.g. each body's fillet arcs) stay independently selectable.
        let g_offset = self.edge_groups.iter().copied().max().map_or(0, |m| m + 1);

        self.vertices.reserve(other.vertices.len());
        self.indices.reserve(other.indices.len());
        self.edge_vertices.reserve(other.edge_vertices.len());
        self.edge_indices.reserve(other.edge_indices.len());
        self.edge_face_normals
            .reserve(other.edge_face_normals.len());
        self.face_ids.reserve(other.face_ids.len());
        self.edge_groups.reserve(other.edge_groups.len());
        self.edge_refs.reserve(other.edge_refs.len());

        self.vertices.extend(other.vertices);
        for idx in other.indices {
            self.indices.push(idx + v_offset);
        }
        self.edge_vertices.extend(other.edge_vertices);
        for idx in other.edge_indices {
            self.edge_indices.push(idx + e_offset);
        }
        self.edge_face_normals.extend(other.edge_face_normals);
        for fid in other.face_ids {
            self.face_ids.push(fid + f_offset);
        }
        for g in other.edge_groups {
            self.edge_groups.push(g + g_offset);
        }
        for mut edge_ref in other.edge_refs {
            let old_group = edge_ref.group;
            edge_ref.group += g_offset;
            if let Some(topology) = &mut edge_ref.topology {
                let old_generated = format!("mesh:{old_group}");
                if topology.edge_id.as_deref() == Some(old_generated.as_str()) {
                    topology.edge_id = Some(format!("mesh:{}", edge_ref.group));
                }
            }
            self.edge_refs.push(edge_ref);
        }
    }

    /// Axis-aligned box with one corner at the origin, opposite corner at (w, h, d).
    pub fn make_box(w: f32, h: f32, d: f32) -> Self {
        let solid = box_solid(w, h, d);

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, false, false);

        let (edge_vertices, edge_indices, edge_face_normals) = build_box_wireframe(w, h, d);
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);
        let edge_refs = mesh_edge_refs_from_groups(
            &edge_vertices,
            &edge_indices,
            &edge_face_normals,
            &edge_groups,
        );

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
            edge_refs,
        }
    }

    /// Cylinder along the +Y axis, base centered at origin, radius r, height h.
    /// `segments` is currently advisory — truck tessellates to TESS_TOL.
    pub fn make_cylinder(r: f32, h: f32, segments: u32) -> Self {
        let solid = match build_cylinder_solid(r as f64, h as f64) {
            Some(s) => s,
            None => return Self::empty(),
        };

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, false, false);
        let (edge_vertices, edge_indices, edge_face_normals) =
            build_cylinder_wireframe(r, h, segments.max(4));
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);
        let edge_refs = mesh_edge_refs_from_groups(
            &edge_vertices,
            &edge_indices,
            &edge_face_normals,
            &edge_groups,
        );

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
            edge_refs,
        }
    }

    /// Extrude a 2D face (outer `points` plus optional `holes`, given in the
    /// 2D `cs` plane coordinates) by `depth` along the plane normal. Holes
    /// produce a through-pocket (e.g. a square hole drawn inside a circle
    /// extrudes to a tube).
    pub fn make_extruded_sketch(
        points: &[(f32, f32)],
        holes: &[Vec<(f32, f32)>],
        depth: f32,
        cs: &crate::geometry::CoordinateSystem,
    ) -> Self {
        if points.len() < 3 || depth.abs() < f32::EPSILON {
            return Self::empty();
        }

        // A hole-free circular profile is displayed as a true cylinder: a smooth
        // curved wall plus a clean wireframe (two rim circles + a few silhouette
        // struts), instead of the faceted prism the boolean path uses. Anything
        // else (polygons, holed profiles) extrudes as a prism.
        let circle = if holes.is_empty() {
            circle_profile(points)
        } else {
            None
        };

        let solid = match circle {
            Some((cu, cv, r)) => oriented_cylinder_solid(cs, cu, cv, r, depth),
            None => build_extrusion_solid(points, holes, depth as f64, cs, false)
                .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs, false)),
        };
        let solid = match solid {
            Some(s) => s,
            None => return Self::empty(),
        };

        // Orient the shell robustly (triangle adjacency + signed volume) rather
        // than the cheap per-triangle centroid repair. A region split out of an
        // arrangement of overlapping sketch shapes is frequently NON-CONVEX (e.g.
        // a rectangle with a circular bite where a circle crossed it); the
        // centroid test then misjudges triangles on the concave side and leaves
        // them inward-facing, so they back-face cull and the body renders with
        // holes. Convex profiles (a plain rectangle, the cylinder cap) orient
        // identically either way, so this is strictly safer.
        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, true, false);

        let (edge_vertices, edge_indices, edge_face_normals) = match circle {
            Some((cu, cv, r)) => build_oriented_cylinder_wireframe(cs, cu, cv, r, depth),
            None => build_extrusion_wireframe(points, holes, depth, cs),
        };
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);
        let edge_refs = mesh_edge_refs_from_groups(
            &edge_vertices,
            &edge_indices,
            &edge_face_normals,
            &edge_groups,
        );

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
            edge_refs,
        }
    }

    /// Tessellate an arbitrary kernel solid (typically a boolean result) into a
    /// renderable mesh. Unlike the analytic constructors above, the wireframe is
    /// extracted from the solid's B-Rep edges, and hidden-line normals are left
    /// empty (the renderer then shows every edge).
    pub fn from_solid(solid: &KernelSolid) -> Self {
        let (vertices, indices, face_ids) = solid_to_flat_mesh(solid, true, false);
        // Derive the wireframe from the tessellation's *feature* edges (borders
        // between two distinct faces), not the raw B-Rep edge list. This gives
        // every edge its two adjacent face normals — so the renderer's
        // hidden-line removal works for boolean results just like it does for
        // primitives, instead of x-raying every edge — and it silently drops the
        // degenerate zero-area "fin" edges a boolean can leave in the B-Rep
        // (they produce no triangle, so no feature edge, so no stray spike).
        // Faces on the same analytic cylinder (a wall the kernel splits into arc-
        // faces) share a group id, so their construction seams aren't drawn as edges.
        let surface_group = cylinder_surface_groups(solid);
        let (mut edge_vertices, mut edge_indices, mut edge_face_normals, mut edge_pairs) =
            mesh_feature_edges(&vertices, &indices, &face_ids, &surface_group);
        add_missing_straight_brep_edges(
            solid,
            &vertices,
            &indices,
            &face_ids,
            &surface_group,
            &mut edge_vertices,
            &mut edge_indices,
            &mut edge_face_normals,
            &mut edge_pairs,
        );
        // Chain the per-triangle chord segments back into whole topological edges:
        // two connected segments belong to the same edge when they border the same
        // pair of *surfaces* (`edge_pairs`, canonicalized through `surface_group` so
        // a cylinder split into arc-faces still reads as one). This is what lets the
        // viewport select a fillet arc — or a full circular rim — as one curve.
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, Some(&edge_pairs));
        let edge_refs = mesh_edge_refs_from_groups(
            &edge_vertices,
            &edge_indices,
            &edge_face_normals,
            &edge_groups,
        );
        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
            edge_refs,
        }
    }
}

// ---------------------------------------------------------------------------
// Public solid builders + boolean operations (used by the parametric evaluator
// to compose join/cut features). Each returns an `openrcad` solid so several
// features can be combined before a single tessellation pass.
// ---------------------------------------------------------------------------
