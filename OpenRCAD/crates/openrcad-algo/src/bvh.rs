//! Flat, cache-friendly Bounding Volume Hierarchy (BVH) spatial accelerator for B-Rep faces.
//! Implements Surface Area Heuristic (SAH) binning for O(log N) query performance.

use openrcad_foundation::{tolerance, BndBox, Pnt, Vec as GeomVec};
use openrcad_geom::{Curve, GeomSurface};
use openrcad_topo::{Face, FaceId};

/// Bounding Volume Hierarchy node.
#[derive(Clone, Debug)]
pub struct BvhNode {
    /// Bounding box of all elements in this node.
    pub bounds: BndBox,
    /// Left child index (if interior) or start index in global face_ids list (if leaf).
    pub left: u32,
    /// Right child index (if interior) or end index (exclusive) in global face_ids list (if leaf).
    pub right: u32,
    /// Flag indicating whether this node is a leaf.
    pub is_leaf: bool,
}

/// A flat, contiguous Surface Area Heuristic (SAH) Bbounding Volume Hierarchy.
#[derive(Clone, Debug)]
pub struct Bvh {
    /// Contiguous buffer of tree nodes. Node 0 is always the root.
    pub nodes: Vec<BvhNode>,
    /// Global array of face IDs. Leaf nodes store indices into this slice.
    pub face_ids: Vec<FaceId>,
}

impl Bvh {
    /// Build a SAH BVH over the given slice of faces.
    pub fn build(faces: &[Face]) -> Self {
        if faces.is_empty() {
            return Self {
                nodes: vec![BvhNode {
                    bounds: BndBox::new(),
                    left: 0,
                    right: 0,
                    is_leaf: true,
                }],
                face_ids: Vec::new(),
            };
        }

        // 1. Compute bounding boxes for all faces
        let face_boxes: Vec<BndBox> = faces.iter().map(compute_face_bounds).collect();
        let face_ids: Vec<FaceId> = faces.iter().map(|f| f.id()).collect();

        let mut face_indices: Vec<usize> = (0..faces.len()).collect();
        let mut nodes = Vec::new();

        // 2. Build the tree recursively
        Self::build_recursive(&mut nodes, &mut face_indices, &face_boxes, 0, faces.len());

        // 3. Reorder global face_ids to match the partitioned indices in the leaves
        let reordered_face_ids = face_indices.iter().map(|&idx| face_ids[idx]).collect();

        Self {
            nodes,
            face_ids: reordered_face_ids,
        }
    }

    fn build_recursive(
        nodes: &mut Vec<BvhNode>,
        face_indices: &mut [usize],
        face_boxes: &[BndBox],
        start: usize,
        end: usize,
    ) -> u32 {
        let count = end - start;
        let node_idx = nodes.len() as u32;

        // Allocate a dummy node that we will overwrite later
        nodes.push(BvhNode {
            bounds: BndBox::new(),
            left: 0,
            right: 0,
            is_leaf: false,
        });

        // Compute overall bounding box for this node
        let mut node_bounds = BndBox::new();
        for &idx in &face_indices[start..end] {
            node_bounds.add_box(&face_boxes[idx]);
        }

        // Base case: leaf node
        if count <= 1 {
            nodes[node_idx as usize] = BvhNode {
                bounds: node_bounds,
                left: start as u32,
                right: end as u32,
                is_leaf: true,
            };
            return node_idx;
        }

        // Compute centroid bounding box
        let mut centroid_bounds = BndBox::new();
        for &idx in &face_indices[start..end] {
            if let Some((lo, hi)) = face_boxes[idx].corners() {
                centroid_bounds.add(&lo.midpoint(&hi));
            }
        }

        let corners = centroid_bounds.corners();
        if corners.is_none() {
            // Degenerate/empty bounds, make a leaf
            nodes[node_idx as usize] = BvhNode {
                bounds: node_bounds,
                left: start as u32,
                right: end as u32,
                is_leaf: true,
            };
            return node_idx;
        }

        let (c_min, c_max) = corners.unwrap();
        let dx = c_max.x() - c_min.x();
        let dy = c_max.y() - c_min.y();
        let dz = c_max.z() - c_min.z();

        // Choose split axis as the longest centroid dimension
        let axis = if dx > dy && dx > dz {
            0
        } else if dy > dz {
            1
        } else {
            2
        };

        let min_val = match axis {
            0 => c_min.x(),
            1 => c_min.y(),
            _ => c_min.z(),
        };
        let max_val = match axis {
            0 => c_max.x(),
            1 => c_max.y(),
            _ => c_max.z(),
        };
        let range = max_val - min_val;

        // If centroid range is too small, split in half
        if range < 1e-12 {
            let mid = start + count / 2;
            let left_child = Self::build_recursive(nodes, face_indices, face_boxes, start, mid);
            let right_child = Self::build_recursive(nodes, face_indices, face_boxes, mid, end);
            nodes[node_idx as usize] = BvhNode {
                bounds: node_bounds,
                left: left_child,
                right: right_child,
                is_leaf: false,
            };
            return node_idx;
        }

        // SAH Binning: 16 bins
        #[derive(Clone, Copy)]
        struct Bin {
            count: usize,
            bounds: BndBox,
        }
        let mut bins = [Bin {
            count: 0,
            bounds: BndBox::new(),
        }; 16];

        for &idx in &face_indices[start..end] {
            let box_corners = face_boxes[idx].corners().unwrap();
            let centroid = box_corners.0.midpoint(&box_corners.1);
            let val = match axis {
                0 => centroid.x(),
                1 => centroid.y(),
                _ => centroid.z(),
            };
            let mut bin_idx = ((val - min_val) / range * 16.0) as usize;
            if bin_idx >= 16 {
                bin_idx = 15;
            }
            bins[bin_idx].count += 1;
            bins[bin_idx].bounds.add_box(&face_boxes[idx]);
        }

        // Evaluate SAH cost of each split plane
        let mut min_cost = f64::INFINITY;
        let mut best_split_bin = 0;

        for i in 0..15 {
            let mut left_bounds = BndBox::new();
            let mut left_count = 0;
            for j in 0..=i {
                left_bounds.add_box(&bins[j].bounds);
                left_count += bins[j].count;
            }

            let mut right_bounds = BndBox::new();
            let mut right_count = 0;
            for j in (i + 1)..16 {
                right_bounds.add_box(&bins[j].bounds);
                right_count += bins[j].count;
            }

            let sa_node = surface_area(&node_bounds);
            let cost = 0.125
                + (surface_area(&left_bounds) / sa_node) * left_count as f64
                + (surface_area(&right_bounds) / sa_node) * right_count as f64;

            if cost < min_cost {
                min_cost = cost;
                best_split_bin = i;
            }
        }

        // Partition elements according to best split
        let split_val = min_val + (best_split_bin + 1) as f64 * (range / 16.0);
        let mut mid = start;
        for i in start..end {
            let idx = face_indices[i];
            let box_corners = face_boxes[idx].corners().unwrap();
            let centroid = box_corners.0.midpoint(&box_corners.1);
            let val = match axis {
                0 => centroid.x(),
                1 => centroid.y(),
                _ => centroid.z(),
            };
            if val <= split_val {
                face_indices.swap(i, mid);
                mid += 1;
            }
        }

        // Handle degenerate splits where all elements went to one side
        if mid == start || mid == end {
            mid = start + count / 2;
        }

        // Build children
        let left_child = Self::build_recursive(nodes, face_indices, face_boxes, start, mid);
        let right_child = Self::build_recursive(nodes, face_indices, face_boxes, mid, end);

        nodes[node_idx as usize] = BvhNode {
            bounds: node_bounds,
            left: left_child,
            right: right_child,
            is_leaf: false,
        };

        node_idx
    }

    /// Perform a box overlap query: returns all FaceIds whose bounding boxes overlap the query box.
    pub fn box_overlap(&self, query_box: &BndBox) -> Vec<FaceId> {
        let mut out = Vec::new();
        if self.nodes.is_empty() {
            return out;
        }
        self.box_overlap_recursive(0, query_box, &mut out);
        out
    }

    fn box_overlap_recursive(&self, node_idx: u32, query_box: &BndBox, out: &mut Vec<FaceId>) {
        let node = &self.nodes[node_idx as usize];
        if node.bounds.is_out_box(query_box) {
            return;
        }

        if node.is_leaf {
            for idx in (node.left as usize)..(node.right as usize) {
                out.push(self.face_ids[idx]);
            }
        } else {
            self.box_overlap_recursive(node.left, query_box, out);
            self.box_overlap_recursive(node.right, query_box, out);
        }
    }

    /// Perform a ray cast query: returns all FaceIds whose bounding boxes intersect the ray.
    pub fn ray_cast(&self, ray_origin: &Pnt, ray_dir: &GeomVec) -> Vec<FaceId> {
        let mut out = Vec::new();
        if self.nodes.is_empty() {
            return out;
        }
        self.ray_cast_recursive(0, ray_origin, ray_dir, &mut out);
        out
    }

    fn ray_cast_recursive(
        &self,
        node_idx: u32,
        ray_origin: &Pnt,
        ray_dir: &GeomVec,
        out: &mut Vec<FaceId>,
    ) {
        let node = &self.nodes[node_idx as usize];
        if !intersects_ray(&node.bounds, ray_origin, ray_dir) {
            return;
        }

        if node.is_leaf {
            for idx in (node.left as usize)..(node.right as usize) {
                out.push(self.face_ids[idx]);
            }
        } else {
            self.ray_cast_recursive(node.left, ray_origin, ray_dir, out);
            self.ray_cast_recursive(node.right, ray_origin, ray_dir, out);
        }
    }

    /// Perform a dual-tree BVH traversal to find all pairs of FaceIds (one from bvh_a, one from bvh_b)
    /// whose bounding boxes overlap.
    pub fn overlapping_pairs(bvh_a: &Bvh, bvh_b: &Bvh) -> Vec<(FaceId, FaceId)> {
        let mut out = Vec::new();
        if bvh_a.nodes.is_empty() || bvh_b.nodes.is_empty() {
            return out;
        }
        Self::overlapping_pairs_recursive(bvh_a, 0, bvh_b, 0, &mut out);
        out
    }

    fn overlapping_pairs_recursive(
        bvh_a: &Bvh,
        node_a_idx: u32,
        bvh_b: &Bvh,
        node_b_idx: u32,
        out: &mut Vec<(FaceId, FaceId)>,
    ) {
        let node_a = &bvh_a.nodes[node_a_idx as usize];
        let node_b = &bvh_b.nodes[node_b_idx as usize];

        if node_a.bounds.is_out_box(&node_b.bounds) {
            return;
        }

        if node_a.is_leaf && node_b.is_leaf {
            for idx_a in (node_a.left as usize)..(node_a.right as usize) {
                for idx_b in (node_b.left as usize)..(node_b.right as usize) {
                    let id_a = bvh_a.face_ids[idx_a];
                    let id_b = bvh_b.face_ids[idx_b];
                    out.push((id_a, id_b));
                }
            }
        } else if node_a.is_leaf {
            Self::overlapping_pairs_recursive(bvh_a, node_a_idx, bvh_b, node_b.left, out);
            Self::overlapping_pairs_recursive(bvh_a, node_a_idx, bvh_b, node_b.right, out);
        } else if node_b.is_leaf {
            Self::overlapping_pairs_recursive(bvh_a, node_a.left, bvh_b, node_b_idx, out);
            Self::overlapping_pairs_recursive(bvh_a, node_a.right, bvh_b, node_b_idx, out);
        } else {
            Self::overlapping_pairs_recursive(bvh_a, node_a.left, bvh_b, node_b.left, out);
            Self::overlapping_pairs_recursive(bvh_a, node_a.left, bvh_b, node_b.right, out);
            Self::overlapping_pairs_recursive(bvh_a, node_a.right, bvh_b, node_b.left, out);
            Self::overlapping_pairs_recursive(bvh_a, node_a.right, bvh_b, node_b.right, out);
        }
    }
}

/// Compute bounding box of a topological Face.
pub fn compute_face_bounds(face: &Face) -> BndBox {
    let mut bounds = BndBox::new();

    // Add all boundary vertices and sample curves
    for wire in face.wires() {
        for edge in wire.edges() {
            bounds.add(&edge.start().point());
            bounds.add(&edge.end().point());

            if let Some(curve) = edge.curve() {
                let first = edge.first();
                let last = edge.last();
                // Sample 8 points along the edge curve
                for i in 1..8 {
                    let t = first + (last - first) * (i as f64) / 8.0;
                    bounds.add(&curve.point(t));
                }
            }
        }
    }

    // Add surface-specific geometry bounds. Analytic surfaces (plane/cylinder/
    // cone/sphere/torus) are bounded by their trimming edges, already sampled
    // above; only a B-spline's free poles can bulge past the trim.
    if let Some(GeomSurface::BSpline(bspline)) = face.surface() {
        for row in bspline.poles() {
            for pole in row {
                bounds.add(pole);
            }
        }
    }

    bounds.enlarge(tolerance::CONFUSION);
    bounds
}

fn surface_area(bounds: &BndBox) -> f64 {
    if let Some((lo, hi)) = bounds.corners() {
        let dx = hi.x() - lo.x();
        let dy = hi.y() - lo.y();
        let dz = hi.z() - lo.z();
        2.0 * (dx * dy + dy * dz + dz * dx)
    } else {
        0.0
    }
}

fn intersects_ray(bounds: &BndBox, ray_origin: &Pnt, ray_dir: &GeomVec) -> bool {
    let (lo, hi) = match bounds.get() {
        Some((l, h)) => (l, h),
        None => return false,
    };

    let mut tmin = 0.0f64;
    let mut tmax = f64::INFINITY;

    // Axis X
    let dx = ray_dir.x();
    if dx.abs() < 1e-15 {
        if ray_origin.x() < lo.x() || ray_origin.x() > hi.x() {
            return false;
        }
    } else {
        let inv_d = 1.0 / dx;
        let mut t1 = (lo.x() - ray_origin.x()) * inv_d;
        let mut t2 = (hi.x() - ray_origin.x()) * inv_d;
        if inv_d < 0.0 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return false;
        }
    }

    // Axis Y
    let dy = ray_dir.y();
    if dy.abs() < 1e-15 {
        if ray_origin.y() < lo.y() || ray_origin.y() > hi.y() {
            return false;
        }
    } else {
        let inv_d = 1.0 / dy;
        let mut t1 = (lo.y() - ray_origin.y()) * inv_d;
        let mut t2 = (hi.y() - ray_origin.y()) * inv_d;
        if inv_d < 0.0 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return false;
        }
    }

    // Axis Z
    let dz = ray_dir.z();
    if dz.abs() < 1e-15 {
        if ray_origin.z() < lo.z() || ray_origin.z() > hi.z() {
            return false;
        }
    } else {
        let inv_d = 1.0 / dz;
        let mut t1 = (lo.z() - ray_origin.z()) * inv_d;
        let mut t2 = (hi.z() - ray_origin.z()) * inv_d;
        if inv_d < 0.0 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return false;
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Pnt, Vec as GeomVec};
    use openrcad_topo::{Edge, Wire};

    fn make_square_face(offset_x: f64) -> Face {
        let w = Wire::from_edges([
            Edge::between_points(
                Pnt::new(offset_x, 0.0, 0.0),
                Pnt::new(offset_x + 1.0, 0.0, 0.0),
            ),
            Edge::between_points(
                Pnt::new(offset_x + 1.0, 0.0, 0.0),
                Pnt::new(offset_x + 1.0, 1.0, 0.0),
            ),
            Edge::between_points(
                Pnt::new(offset_x + 1.0, 1.0, 0.0),
                Pnt::new(offset_x, 1.0, 0.0),
            ),
            Edge::between_points(Pnt::new(offset_x, 1.0, 0.0), Pnt::new(offset_x, 0.0, 0.0)),
        ]);
        Face::new(None, w)
    }

    #[test]
    fn test_bvh_build_and_queries() {
        use openrcad_topo::BRep;
        use std::sync::Arc;

        let f1 = make_square_face(0.0);
        let f2 = make_square_face(2.0);

        // Merge both into a single BRep so they have unique FaceIds
        let mut brep = BRep::new();
        let map1 = brep.merge(f1.brep());
        let map2 = brep.merge(f2.brep());

        let id1 = map1.faces[&f1.id()];
        let id2 = map2.faces[&f2.id()];

        let shared_brep = Arc::new(brep);
        let face1 = Face::from_id(shared_brep.clone(), id1, f1.orientation());
        let face2 = Face::from_id(shared_brep, id2, f2.orientation());

        let bvh = Bvh::build(&[face1.clone(), face2.clone()]);
        assert_eq!(bvh.face_ids.len(), 2);
        assert_ne!(face1.id(), face2.id());

        // Query overlap box around f1
        let mut qb1 = BndBox::new();
        qb1.add(&Pnt::new(0.25, 0.25, -0.5));
        qb1.add(&Pnt::new(0.75, 0.75, 0.5));
        let overlaps1 = bvh.box_overlap(&qb1);
        assert!(overlaps1.contains(&face1.id()));
        assert!(!overlaps1.contains(&face2.id()));

        // Query ray casting through f2
        let ray_org = Pnt::new(2.5, 0.5, 5.0);
        let ray_dir = GeomVec::new(0.0, 0.0, -1.0);
        let ray_hits = bvh.ray_cast(&ray_org, &ray_dir);
        assert!(ray_hits.contains(&face2.id()));
        assert!(!ray_hits.contains(&face1.id()));

        // Query ray casting through gap
        let ray_org_gap = Pnt::new(1.5, 0.5, 5.0);
        let ray_hits_gap = bvh.ray_cast(&ray_org_gap, &ray_dir);
        assert!(ray_hits_gap.is_empty());
    }

    #[test]
    fn test_overlapping_pairs() {
        use openrcad_topo::BRep;
        use std::sync::Arc;

        let f1 = make_square_face(0.0);
        let f2 = make_square_face(0.5); // overlaps f1
        let f3 = make_square_face(2.0); // does not overlap f1

        let mut brep1 = BRep::new();
        let map1 = brep1.merge(f1.brep());
        let shared_brep1 = Arc::new(brep1);
        let face1 = Face::from_id(shared_brep1.clone(), map1.faces[&f1.id()], f1.orientation());
        let bvh1 = Bvh::build(std::slice::from_ref(&face1));

        let mut brep2 = BRep::new();
        let map2 = brep2.merge(f2.brep());
        let map3 = brep2.merge(f3.brep());
        let shared_brep2 = Arc::new(brep2);
        let face2 = Face::from_id(shared_brep2.clone(), map2.faces[&f2.id()], f2.orientation());
        let face3 = Face::from_id(shared_brep2, map3.faces[&f3.id()], f3.orientation());
        let bvh2 = Bvh::build(&[face2.clone(), face3.clone()]);

        let pairs = Bvh::overlapping_pairs(&bvh1, &bvh2);
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (face1.id(), face2.id()));
    }
}
