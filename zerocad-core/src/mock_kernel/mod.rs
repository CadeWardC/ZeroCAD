//! Geometry kernel — backed by the `openrcad` pure-Rust B-Rep CAD kernel.
//!
//! The `MockMesh` name and field layout are preserved so existing parametric
//! and rendering code keeps working unchanged. Internally each constructor
//! builds a real `openrcad::topo::Solid`, tessellates it via `openrcad::mesh`,
//! and flattens the result into the same interleaved position+normal vertex
//! buffer the egui painter expects.
//!
//! Wireframe edges are still produced analytically (matching the previous
//! procedural output) — extracting them from the B-Rep topology is deferred
//! to the GPU-viewport phase.

use std::collections::{HashMap, HashSet};

use openrcad::algo::{BlendContour, BlendCurveHint, BlendKind, BooleanOp};
pub use openrcad::algo::{BooleanFaceHistory, BooleanFaceSource};
use openrcad::foundation::{Ax2, Ax3, Dir, Pnt, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad::geom::{
    BSplineCurve, Circle, Curve, CylindricalSurface, Ellipse, GeomCurve, GeomSurface, Plane,
};
use openrcad::topo::{Edge, Face, Orientation, Solid, Vertex, Wire};

use crate::geometry::Vec3;

pub(crate) type LoftSectionProfile = (
    crate::geometry::CoordinateSystem,
    Vec<(f32, f32)>,
    Vec<Vec<(f32, f32)>>,
);

pub(crate) type SmoothLoftSectionProfile = (
    crate::geometry::CoordinateSystem,
    crate::sketch::AnalyticSketchRegion,
);

mod arc_display;
mod blend;
mod boolean;
mod circle_geom;
mod geom_utils;
mod history;
mod mesh_topology;
mod outcome;
mod primitives;
mod section;
mod tessellation;
mod types;
mod wireframe;

#[allow(unused_imports)]
pub(crate) use arc_display::*;
pub use blend::*;
pub use boolean::*;
pub use circle_geom::*;
pub use geom_utils::*;
#[allow(unused_imports)]
pub use history::*;
pub use mesh_topology::mesh_face_boundary_2d;
#[allow(unused_imports)]
pub(crate) use mesh_topology::*;
pub(crate) use outcome::*;
pub use primitives::*;
pub use section::*;
#[allow(unused_imports)]
pub(crate) use tessellation::*;
pub use types::*;
#[allow(unused_imports)]
pub(crate) use wireframe::*;

/// Clear evaluator thread-local integration state after an unwound kernel call.
/// Request-owned solids and graph caches are dropped by the worker separately.
pub fn reset_evaluator_thread_state() {
    clear_kernel_cancellation();
    reset_diagnostics();
    clear_preview_tess_override();
}

#[cfg(test)]
mod wireframe_tests {
    use super::*;
    use crate::geometry::CoordinateSystem;

    /// Count edges that run parallel to the sweep axis (i.e. vertical struts):
    /// their endpoints share x/y and differ by `|depth|` along z (CS::XY here).
    fn count_struts(ev: &[f32], ei: &[u32], depth: f32) -> usize {
        ei.chunks_exact(2)
            .filter(|p| {
                let (a, b) = (p[0] as usize * 3, p[1] as usize * 3);
                let dz = (ev[a + 2] - ev[b + 2]).abs();
                let dxy = ((ev[a] - ev[b]).powi(2) + (ev[a + 1] - ev[b + 1]).powi(2)).sqrt();
                (dz - depth.abs()).abs() < 1e-3 && dxy < 1e-3
            })
            .count()
    }

    #[test]
    fn same_face_triangle_diagonal_is_not_a_feature_edge() {
        let vertices = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 0];

        let (_ev, ei, _, _) = mesh_feature_edges(&vertices, &indices, &face_ids, &[]);

        assert_eq!(
            ei.len() / 2,
            0,
            "triangle edges internal to one B-Rep face must not be selectable or drawn"
        );
    }

    #[test]
    fn sharp_polygon_struts_every_true_corner() {
        // A square has four genuine 90° corners → exactly four vertical struts.
        let square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let (ev, ei, _) = build_extrusion_wireframe(&square, &[], 5.0, &CoordinateSystem::XY);
        assert_eq!(count_struts(&ev, &ei, 5.0), 4);
    }

    #[test]
    fn smooth_circle_extrude_has_no_strut_fan() {
        // A sketched circle is a many-sided polygon whose consecutive walls differ
        // by only a few degrees. Pre-fix this drew one strut per segment (a dense
        // fan); now the smooth wall must produce ZERO struts.
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let (ev, ei, _) = build_extrusion_wireframe(&circle, &[], 8.0, &CoordinateSystem::XY);
        assert_eq!(count_struts(&ev, &ei, 8.0), 0);
    }

    #[test]
    fn analytic_sketch_circle_reaches_brep_as_circle_edges() {
        let mut curves = crate::sketch::SketchCurves::new();
        curves.add_circle((3.0, -2.0), 5.0);
        let region = crate::sketch::detect_regions_analytic(&curves)
            .expect("circle arrangement")
            .into_iter()
            .next()
            .expect("circle region");

        let solid = extruded_sketch_region_solid(&region, 7.0, &CoordinateSystem::XY, &[])
            .expect("analytic circle prism");

        assert!(solid
            .edges()
            .iter()
            .any(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_)))));
    }

    #[test]
    fn analytic_nested_circles_build_inner_wire_prism() {
        let mut curves = crate::sketch::SketchCurves::new();
        curves.add_circle((0.0, 0.0), 8.0);
        curves.add_circle((0.0, 0.0), 3.0);
        let regions = crate::sketch::detect_regions_analytic(&curves).expect("nested circles");
        let annulus = regions
            .iter()
            .find(|region| !region.holes.is_empty())
            .expect("annular region");

        let solid = extruded_sketch_region_solid(annulus, 4.0, &CoordinateSystem::XY, &[])
            .expect("analytic annulus prism");

        assert_eq!(solid.shells().len(), 1);
        assert!(
            solid
                .edges()
                .iter()
                .filter(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_))))
                .count()
                >= 8
        );
    }

    /// Glyph outlines are chains of OPEN Bézier segments meeting end to end, so
    /// sketch text depends on a closed loop assembled from open control-point
    /// splines reaching the B-Rep as real B-spline edges. If the analytic path
    /// ever drops that provenance the loop falls back to sampled chords and
    /// every letter extrudes as a faceted prism.
    #[test]
    fn analytic_open_spline_chain_reaches_brep_as_bspline_edges() {
        let mut curves = crate::sketch::SketchCurves::new();
        curves.add_line((0.0, 0.0), (10.0, 0.0));
        curves.add_spline(crate::sketch::Spline {
            kind: crate::sketch::SplineKind::ControlPoint,
            points: vec![(10.0, 0.0), (14.0, 3.0), (14.0, 7.0), (10.0, 10.0)],
            degree: 3,
            knots: Vec::new(),
            weights: Vec::new(),
            closed: false,
            periodic: false,
            continuity: crate::sketch::SplineContinuity::default(),
            trim: None,
        });
        curves.add_line((10.0, 10.0), (0.0, 10.0));
        curves.add_line((0.0, 10.0), (0.0, 0.0));

        let region = crate::sketch::detect_regions_analytic(&curves)
            .expect("open-spline chain arranges")
            .into_iter()
            .next()
            .expect("bulged rectangle region");

        let solid = extruded_sketch_region_solid(&region, 4.0, &CoordinateSystem::XY, &[])
            .expect("analytic spline prism");

        assert!(
            solid
                .edges()
                .iter()
                .any(|edge| matches!(edge.curve(), Some(GeomCurve::BSpline(_)))),
            "the spline span must survive as a B-spline edge, not sampled chords"
        );
    }

    fn swept_volume(region: &crate::sketch::Region, depth: f32) -> f64 {
        let solid =
            extruded_sketch_region_solid(region, depth, &CoordinateSystem::XY, &[]).expect("prism");
        assert!(solid.is_watertight(), "swept solid must be watertight");
        crate::parametric::solid_volume(&solid).expect("closed solid")
    }

    fn circle_region(radius: f32) -> crate::sketch::Region {
        let mut curves = crate::sketch::SketchCurves::new();
        curves.add_circle((0.0, 0.0), radius);
        crate::sketch::detect_regions_analytic(&curves)
            .expect("circle arrangement")
            .into_iter()
            .next()
            .expect("disc region")
    }

    /// KNOWN BUG - a circular prism's cylindrical walls face inward.
    ///
    /// Per-face divergence contributions for an r=8, h=4 disc prism:
    ///   cap z=0   Plane    Forward     0.00
    ///   cap z=4   Plane    Forward  +266.36
    ///   4x wall   Cylinder Reversed -133.53 each
    /// The wall magnitudes are right (534 against an expected 536) but their
    /// sign is inverted, so the solid's signed volume is -267.76; the reported
    /// 267.76 is only its absolute value.
    ///
    /// This is the cap-orientation bug repeated on the laterals: `sew` resolves
    /// a winding/normal conflict by flipping the face's orientation flag, which
    /// leaves an inward *effective* normal that watertight and health checks
    /// cannot see. The cap case was fixed by reversing the loop winding
    /// (`reversed_wire`); the cylindrical laterals never got the same treatment.
    ///
    /// Nothing to do with sketch text - it affects every curved extrusion, and
    /// therefore every mass property, volume readout and STL export over one.
    /// It survives because watertight, health and closed-mesh checks all pass
    /// and shading uses per-vertex normals rather than winding.

    #[test]
    fn circular_prism_encloses_its_full_volume() {
        let volume = swept_volume(&circle_region(8.0), 4.0);
        let expected = std::f64::consts::PI * 64.0 * 4.0;
        assert!(
            (volume - expected).abs() <= expected * 0.02,
            "disc prism should enclose {expected:.2} mm3, got {volume:.2}"
        );
    }

    /// Follows from the same defect; kept because a holed profile is the shape
    /// text emboss actually produces, so it must be re-checked after the fix.
    #[test]
    fn annulus_prism_volume_excludes_the_inner_wire() {
        let mut curves = crate::sketch::SketchCurves::new();
        curves.add_circle((0.0, 0.0), 8.0);
        curves.add_circle((0.0, 0.0), 3.0);
        let annulus = crate::sketch::detect_regions_analytic(&curves)
            .expect("nested circles")
            .into_iter()
            .find(|region| !region.holes.is_empty())
            .expect("annular region");
        assert!((annulus.area - std::f32::consts::PI * 55.0).abs() < 0.01);

        let volume = swept_volume(&annulus, 4.0);
        let expected = std::f64::consts::PI * 55.0 * 4.0;
        assert!(
            (volume - expected).abs() <= expected * 0.02,
            "annulus prism should enclose {expected:.2} mm3, got {volume:.2}"
        );
    }

    #[test]
    fn bored_hole_wall_has_no_fake_seam_struts() {
        // A 40x20x10 block with a Ø8 pocket bored 6mm into its top face. The kernel
        // builds the pocket wall as a smooth analytic cylinder, but represents it as
        // 3 arc-faces (thirds); the longitudinal seams between those arc-faces are a
        // construction artifact, not design edges. Drawing them made the hole read as
        // a notched/faceted circle from the top. They run straight down the wall, so
        // they're vertical struts of the *pocket* depth (6) — assert none survive,
        // while the box's four real 90° corner edges (height 10) still draw.
        let block = openrcad::primitives::make_box_operation(&Pnt::origin(), 40.0, 20.0, 10.0)
            .unwrap()
            .value;
        let drill = openrcad::primitives::make_cylinder_operation(
            &Ax2::new(Pnt::new(20.0, 10.0, 4.0), Dir::dz()),
            4.0,
            6.0,
        )
        .unwrap()
        .value;
        let body = openrcad::algo::boolean_operation(&block, &drill, BooleanOp::Cut)
            .expect("bore should cut")
            .value;
        let mesh = MockMesh::from_solid(&body);
        assert_eq!(
            count_struts(&mesh.edge_vertices, &mesh.edge_indices, 6.0),
            0,
            "the 3-arc cylinder wall's longitudinal seams must be suppressed"
        );
        assert_eq!(
            count_struts(&mesh.edge_vertices, &mesh.edge_indices, 10.0),
            4,
            "the box's four real 90-degree corner edges must still draw"
        );
    }

    #[test]
    fn lone_boundary_edges_are_dropped() {
        // Two coplanar triangles meeting along a diagonal, the four outer edges
        // owned by a single triangle each. On a closed solid such lone edges are
        // tessellation cracks, not design edges — they must be dropped. The
        // shared diagonal is coplanar (same stored normal) so it's suppressed too,
        // leaving no wireframe at all.
        let v = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 1];
        let (_ev, ei, _, _) = mesh_feature_edges(&v, &indices, &face_ids, &[]);
        assert_eq!(ei.len() / 2, 0, "lone boundary/crack edges must be dropped");
    }

    #[test]
    fn genuine_perpendicular_crease_is_kept() {
        // Two triangles sharing the x-axis edge but lying in perpendicular planes
        // (z=0 and y=0). That 90° crease is a real design edge and must survive.
        let v = vec![
            // Triangle A (face 0) in z=0, normal +Z
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            // Triangle B (face 1) in y=0, normal +Y
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, 1.0, 0.0, //
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 1];
        let (ev, ei, _, _) = mesh_feature_edges(&v, &indices, &face_ids, &[]);

        assert_eq!(
            ei.len() / 2,
            1,
            "the 90° crease should be the one kept edge"
        );
        let (a, b) = (ei[0] as usize * 3, ei[1] as usize * 3);
        let on = |k: usize, p: (f32, f32, f32)| {
            (ev[k] - p.0).abs() < 1e-4
                && (ev[k + 1] - p.1).abs() < 1e-4
                && (ev[k + 2] - p.2).abs() < 1e-4
        };
        let shared = (on(a, (0.0, 0.0, 0.0)) && on(b, (1.0, 0.0, 0.0)))
            || (on(a, (1.0, 0.0, 0.0)) && on(b, (0.0, 0.0, 0.0)));
        assert!(shared, "kept edge should be the shared x-axis crease");
    }

    #[test]
    fn fillet_arc_chords_group_into_one_edge() {
        // Fillet a vertical box edge: the blend's two end arcs are circular, so
        // each tessellates into several chords. Those chords must collapse into a
        // single `edge_groups` id apiece, so the viewport selects the whole arc as
        // one curve (the user's request) instead of a lone chord.
        let solid = openrcad::primitives::make_box_operation(&Pnt::origin(), 10.0, 10.0, 10.0)
            .unwrap()
            .value;
        let edge = solid
            .edges()
            .into_iter()
            .find(|e| {
                let p0 = e.start().point();
                let p1 = e.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 9.9
            })
            .expect("box has a vertical origin edge");
        let filleted =
            openrcad::algo::fillet_edges(&solid, &[edge], 2.0).expect("fillet the box edge");
        let mesh = MockMesh::from_solid(&filleted);

        let seg_count = mesh.edge_indices.len() / 2;
        assert!(seg_count > 0, "fillet produced no wireframe edges");
        assert_eq!(
            mesh.edge_groups.len(),
            seg_count,
            "one group id per edge segment"
        );

        let mut sizes: HashMap<u32, usize> = HashMap::new();
        for &g in &mesh.edge_groups {
            *sizes.entry(g).or_insert(0) += 1;
        }
        // At least one edge (a fillet end arc) is many chords grouped as one.
        let max_group = sizes.values().copied().max().unwrap_or(0);
        assert!(
            max_group >= 2,
            "no multi-chord arc was grouped into one edge: sizes={sizes:?}"
        );
        // Grouping actually collapsed segments — fewer groups than chords.
        assert!(
            sizes.len() < seg_count,
            "grouping collapsed nothing: {} groups for {seg_count} chords",
            sizes.len()
        );
    }

    #[test]
    fn smooth_circle_rim_groups_into_one_edge() {
        // A sketched circle extrudes to a smooth cylinder whose rim is one circular
        // edge drawn as many chords. The geometric (tangent-continuity) grouping —
        // used where there's no B-Rep face provenance — must chain those chords
        // into a single edge so the rim selects as one curve.
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let mesh = MockMesh::make_extruded_sketch(&circle, &[], 8.0, &CoordinateSystem::XY);

        let seg_count = mesh.edge_indices.len() / 2;
        assert_eq!(
            mesh.edge_groups.len(),
            seg_count,
            "one group id per segment"
        );

        let mut sizes: HashMap<u32, usize> = HashMap::new();
        for &g in &mesh.edge_groups {
            *sizes.entry(g).or_insert(0) += 1;
        }
        // A rim circle chains into one large group (at least half its chords).
        let big = sizes.values().filter(|&&c| c >= n / 2).count();
        assert!(
            big >= 1,
            "rim circle was not chained into one edge: sizes={sizes:?}"
        );
    }
}

#[cfg(test)]
mod arc_reconstruction_tests {
    use super::*;
    use crate::geometry::CoordinateSystem;
    use openrcad::geom::GeomSurface;

    fn cylinder_faces(solid: &KernelSolid) -> usize {
        solid
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
            .count()
    }

    /// A finely-sampled circular arc (a sketch fillet / drawn arc) must rebuild
    /// into cylindrical walls when extruded — not a fan of planar facets.
    #[test]
    fn semicircle_d_profile_extrudes_to_smooth_cylinder_wall() {
        // "D" shape: a right semicircle (radius 5 about (5,5)) closed by three
        // straight edges down the left. 24 arc facets — pre-fix this swept 24
        // planar strips; now it must be smooth cylindrical wall(s).
        let mut pts: Vec<(f32, f32)> = Vec::new();
        let steps = 24;
        for i in 0..=steps {
            let t = -std::f32::consts::FRAC_PI_2 + (i as f32 / steps as f32) * std::f32::consts::PI;
            pts.push((5.0 + 5.0 * t.cos(), 5.0 + 5.0 * t.sin()));
        }
        pts.push((0.0, 10.0));
        pts.push((0.0, 0.0));

        let solid = extruded_region_solid(&pts, &[], 5.0, &CoordinateSystem::XY)
            .expect("D profile should extrude");
        assert!(
            cylinder_faces(&solid) >= 1,
            "the semicircle must become cylindrical wall(s), got {} cylinders / {} faces",
            cylinder_faces(&solid),
            solid.face_count()
        );
        // A 180° arc capped at 135°/piece → 2 cylinder faces + 3 straight laterals
        // + 2 caps. The exact split count isn't contractual, but the shell must be
        // a closed, healthy solid.
        assert!(solid.is_watertight(), "D extrusion must be watertight");
        assert!(
            solid.health_report().is_healthy(),
            "D extrusion must be healthy"
        );
    }

    #[test]
    fn display_mesh_for_mixed_profiles_is_render_safe() {
        let mut pts: Vec<(f32, f32)> = Vec::new();
        let steps = 24;
        for i in 0..=steps {
            let t = -std::f32::consts::FRAC_PI_2 + (i as f32 / steps as f32) * std::f32::consts::PI;
            pts.push((5.0 + 5.0 * t.cos(), 5.0 + 5.0 * t.sin()));
        }
        pts.push((0.0, 10.0));
        pts.push((0.0, 0.0));

        let solid = extruded_region_solid(&pts, &[], 5.0, &CoordinateSystem::XY)
            .expect("D profile should extrude");
        assert!(
            cylinder_faces(&solid) >= 1,
            "D-profile kernel solid should keep a cylindrical wall"
        );

        let mesh = extruded_region_display_mesh(&pts, &[], 5.0, &CoordinateSystem::XY);
        assert!(
            render_mesh_is_closed_manifold(&mesh),
            "D-profile display mesh should be closed and manifold"
        );
        assert!(
            render_mesh_normals_follow_winding(&mesh),
            "D-profile display mesh normals should agree with winding"
        );
    }

    /// A polygon the user genuinely wants faceted (here a rectangle) has no
    /// co-circular run, so every corner stays a sharp line edge.
    #[test]
    fn rectangle_stays_a_six_face_box() {
        let rect = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)];
        let solid = extruded_region_solid(&rect, &[], 4.0, &CoordinateSystem::XY)
            .expect("rectangle should extrude");
        assert_eq!(
            cylinder_faces(&solid),
            0,
            "a box must have no cylinder faces"
        );
        assert_eq!(solid.face_count(), 6, "a box is 6 faces");
        assert!(solid.is_watertight());
    }

    /// A full sketched circle fed straight to the prism path (the boolean-tool
    /// fallback) rebuilds into a smooth cylinder, not a 48-gon.
    #[test]
    fn full_circle_profile_rebuilds_to_cylinder() {
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let solid = extruded_region_solid(&circle, &[], 8.0, &CoordinateSystem::XY)
            .expect("circle should extrude");
        assert!(
            cylinder_faces(&solid) >= 1,
            "a circle must rebuild to cylindrical wall(s), got {} cylinders",
            cylinder_faces(&solid)
        );
        assert!(solid.is_watertight(), "circle extrusion must be watertight");
    }

    #[test]
    fn display_mesh_keeps_intentional_octagon_faceted() {
        let octagon: Vec<(f32, f32)> = (0..8)
            .map(|i| {
                let a = (i as f32 / 8.0) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let mesh = extruded_region_display_mesh(&octagon, &[], 4.0, &CoordinateSystem::XY);
        let faces: std::collections::HashSet<u32> = mesh.face_ids.iter().copied().collect();
        assert!(
            faces.len() >= 10,
            "octagon display should stay an 8-sided prism, got {} faces",
            faces.len()
        );
    }

    /// An octagon turns 45° per vertex — well past the arc-vs-corner threshold —
    /// so it must stay a faceted prism, never collapse to a cylinder.
    #[test]
    fn octagon_is_not_mistaken_for_a_circle() {
        let octagon: Vec<(f32, f32)> = (0..8)
            .map(|i| {
                let a = (i as f32 / 8.0) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let solid = extruded_region_solid(&octagon, &[], 4.0, &CoordinateSystem::XY)
            .expect("octagon should extrude");
        assert_eq!(
            cylinder_faces(&solid),
            0,
            "an octagon is an intentional polygon — keep it faceted"
        );
        assert!(solid.is_watertight());
    }
}
