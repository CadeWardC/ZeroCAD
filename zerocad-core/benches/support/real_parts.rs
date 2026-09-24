//! Frozen mechanical workloads shared by benchmarks and lifecycle regressions.
use zerocad_core::parametric::FaceRef;
use zerocad_core::*;

pub fn add(graph: &mut ParametricGraph, id: &str, feature: FeatureType) {
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature,
    });
}

fn sketch(
    graph: &mut ParametricGraph,
    id: &str,
    curves: SketchCurves,
    z: f32,
    shapes: Vec<sketch::SketchShape>,
) {
    add(
        graph,
        id,
        FeatureType::Sketch {
            cs: CoordinateSystem::XY.with_origin(Vec3::new(0., 0., z)),
            curves,
            shapes,
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    );
}

fn extrude(graph: &mut ParametricGraph, id: &str, source: &str, depth: f32, mode: ExtrudeMode) {
    add(
        graph,
        id,
        FeatureType::Extrude {
            target: (mode == ExtrudeMode::Cut).then(|| "base_1".into()),
            depth,
            region_indices: vec![],
            mode,
            depth_expr: None,
            draft_angle_deg: 0.,
            draft_angle_expr: None,
        },
    );
    graph.add_dependency(source, id);
}

fn plate() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0., 0.), (60., 40.));
    sketch(&mut graph, "sketch_0", curves, 0., vec![]);
    extrude(&mut graph, "base_1", "sketch_0", 8., ExtrudeMode::NewBody);
    graph
}

pub fn patterned_plate() -> ParametricGraph {
    let mut graph = plate();
    let mut holes = SketchCurves::new();
    for x in [10., 30., 50.] {
        for y in [10., 30.] {
            holes.add_circle((x, y), 2.5);
        }
    }
    sketch(&mut graph, "holes_2", holes, 10., vec![]);
    extrude(&mut graph, "cut_3", "holes_2", -12., ExtrudeMode::Cut);
    graph.add_dependency("base_1", "cut_3");
    graph
}

pub fn open_enclosure() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add(
        &mut graph,
        "base_1",
        FeatureType::Box {
            w: 60.,
            h: 40.,
            d: 20.,
        },
    );
    add(
        &mut graph,
        "shell_2",
        FeatureType::Shell {
            target: "base_1".into(),
            thickness: 2.,
            thickness_expr: None,
            open_faces: vec![FaceRef {
                centroid: [30., 20., 20.],
                normal: [0., 0., 1.],
                topology: None,
            }],
        },
    );
    graph.add_dependency("base_1", "shell_2");
    graph
}

pub fn filleted_block() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add(
        &mut graph,
        "base_1",
        FeatureType::Box {
            w: 60.,
            h: 40.,
            d: 20.,
        },
    );
    add(
        &mut graph,
        "fillet_2",
        FeatureType::EdgeMod {
            target: "base_1".into(),
            dist: 1.5,
            dist_expr: None,
            kind: sketch::CornerKind::Fillet,
            edge: EdgeRef {
                p0: [0., 0., 0.],
                p1: [60., 0., 0.],
                n1: [0., -1., 0.],
                n2: [0., 0., -1.],
                curve: Some(mock_kernel::EdgeCurveHint::Line),
                topology: None,
            },
        },
    );
    graph.add_dependency("base_1", "fillet_2");
    graph
}

pub fn engraved_plate() -> ParametricGraph {
    engraved_plate_text("ZeroCAD 17")
}

pub fn engraved_plate_text(label: &str) -> ParametricGraph {
    let mut graph = plate();
    let shape = text::bake_text_shape(
        epaint_default_fonts::HACK_REGULAR,
        0,
        label,
        &text::TextParams {
            size_mm: 7.,
            ..Default::default()
        },
        text::TextPlacement {
            origin: (5., 20.),
            ..Default::default()
        },
    )
    .unwrap();
    sketch(&mut graph, "text_2", SketchCurves::new(), 8., vec![shape]);
    extrude(&mut graph, "cut_3", "text_2", -0.5, ExtrudeMode::Cut);
    graph.add_dependency("base_1", "cut_3");
    graph
}

pub fn curved_step_import() -> ParametricGraph {
    let solid = openrcad::primitives::make_cylinder_operation(
        &openrcad::foundation::Ax2::new(
            openrcad::foundation::Pnt::origin(),
            openrcad::foundation::Dir::dz(),
        ),
        12.,
        30.,
    )
    .unwrap()
    .value;
    let mut data = Vec::new();
    openrcad::exchange::write_step_bodies(&[("shaft", &solid)], &mut data).unwrap();
    let mut graph = ParametricGraph::new();
    add(
        &mut graph,
        "base_1",
        FeatureType::Import {
            step_data: String::from_utf8(data).unwrap(),
            label: "analytic-shaft.step".into(),
        },
    );
    graph
}

pub type PartBuilder = fn() -> ParametricGraph;
pub const PARTS: &[(&str, PartBuilder)] = &[
    ("patterned_plate", patterned_plate),
    ("open_enclosure", open_enclosure),
    ("filleted_block", filleted_block),
    ("engraved_plate", engraved_plate),
    ("curved_step_import", curved_step_import),
];
