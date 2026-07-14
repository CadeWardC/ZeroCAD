use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

pub const SMALL_CORPUS: &str = "small_part";
pub const HUNDRED_FEATURE_CORPUS: &str = "dependent_100_feature";
pub const FIVE_HUNDRED_FEATURE_CORPUS: &str = "dependent_500_feature";
pub const IMPORTED_STEP_CORPUS: &str = "imported_step_box";
#[cfg(test)]
#[allow(dead_code)]
pub const EXTERNAL_STEP_CORPUS: &str = "external_nist_bracket";
pub const DIFFICULT_KERNEL_CORPUS: &str = "difficult_through_hole";

fn add_feature(graph: &mut ParametricGraph, id: &str, name: &str, feature: FeatureType) {
    graph.add_feature(FeatureNode {
        id: id.to_string(),
        name: name.to_string(),
        feature,
    });
}

fn drilled_box(width: f32, depth: f32, height: f32, radius: f32) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add_feature(
        &mut graph,
        "base_1",
        "Base block",
        FeatureType::Box {
            w: width,
            h: depth,
            d: height,
        },
    );
    add_feature(
        &mut graph,
        "drill_2",
        "Drill",
        FeatureType::Cylinder {
            r: radius,
            h: depth + 2.0,
        },
    );
    add_feature(
        &mut graph,
        "position_drill_3",
        "Position drill",
        FeatureType::BodyTransform {
            source: "drill_2".to_string(),
            translation: [width * 0.5, -1.0, height * 0.5],
            copy: false,
        },
    );
    graph.add_dependency("drill_2", "position_drill_3");
    add_feature(
        &mut graph,
        "through_hole_4",
        "Through hole",
        FeatureType::BodyCut {
            target: "base_1".to_string(),
            tool: "position_drill_3".to_string(),
            keep_tool: false,
        },
    );
    graph.add_dependency("base_1", "through_hole_4");
    graph.add_dependency("position_drill_3", "through_hole_4");
    graph
}

pub fn small_part_history() -> ParametricGraph {
    drilled_box(40.0, 30.0, 12.0, 4.0)
}

pub fn dependent_feature_history(feature_count: usize) -> ParametricGraph {
    assert!(
        feature_count > 0,
        "a corpus must contain at least one feature"
    );
    let mut graph = ParametricGraph::new();
    add_feature(
        &mut graph,
        "box_0001",
        "Seed box",
        FeatureType::Box {
            w: 20.0,
            h: 12.0,
            d: 8.0,
        },
    );

    let mut source = "box_0001".to_string();
    for sequence in 2..=feature_count {
        let id = format!("move_{sequence:04}");
        add_feature(
            &mut graph,
            &id,
            &format!("Move {sequence}"),
            FeatureType::BodyTransform {
                source: source.clone(),
                translation: [0.02, 0.01, 0.0],
                copy: false,
            },
        );
        graph.add_dependency(&source, &id);
        source = id;
    }
    graph
}

pub fn hundred_feature_history() -> ParametricGraph {
    dependent_feature_history(100)
}

pub fn five_hundred_feature_history() -> ParametricGraph {
    dependent_feature_history(500)
}

fn box_step_data() -> String {
    let solid =
        openrcad::primitives::make_box(&openrcad::foundation::Pnt::origin(), 20.0, 12.0, 8.0);
    let path =
        std::env::temp_dir().join(format!("zerocad_phase0_import_{}.step", std::process::id()));
    let path_text = path.to_string_lossy().into_owned();
    openrcad::exchange::write_step(&solid, &path_text).expect("write imported STEP corpus");
    let step = std::fs::read_to_string(&path).expect("read imported STEP corpus");
    let _ = std::fs::remove_file(path);
    step
}

pub fn imported_step_history() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add_feature(
        &mut graph,
        "import_1",
        "Imported STEP box",
        FeatureType::Import {
            step_data: box_step_data(),
            label: "phase0_imported_box.step".to_string(),
        },
    );
    graph
}

#[cfg(test)]
#[allow(dead_code)]
pub fn external_step_history() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    add_feature(
        &mut graph,
        "external_import_1",
        "NIST AP203 bracket",
        FeatureType::Import {
            step_data: include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/fixtures/step/nist-bracket1-part.stp"
            ))
            .to_string(),
            label: "nist-bracket1-part.stp".to_string(),
        },
    );
    graph
}

pub fn difficult_kernel_history() -> ParametricGraph {
    let width: f32 = 50.453_705;
    let depth: f32 = 58.851_513;
    let height: f32 = 28.052_053;
    let radius = width.min(height) * 0.254_935_32;
    drilled_box(width, depth, height, radius)
}
