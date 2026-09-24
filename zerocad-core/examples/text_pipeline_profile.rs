//! Stage timings for the frozen text workload; run optimized on an idle machine.
#[path = "../benches/support/real_parts.rs"]
mod real_parts;
use openrcad::foundation::{TolerancePolicy, Trsf, Vec as GeomVec};
use std::time::Instant;

fn main() {
    let graph = real_parts::PARTS
        .iter()
        .find(|(name, _)| *name == "engraved_plate")
        .unwrap()
        .1();
    let solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
    let input = &solids[0].1[0];
    let policy = TolerancePolicy::STANDARD;
    let mut stages = serde_json::Map::new();
    macro_rules! measure {
        ($name:literal, $expression:expr) => {{
            let start = Instant::now();
            let result = $expression;
            stages.insert(
                $name.into(),
                serde_json::json!(start.elapsed().as_secs_f64() * 1000.),
            );
            result
        }};
    }
    let operation = measure!(
        "canonical_operation_ms",
        openrcad::algo::transform_operation_with_policy(
            input,
            &Trsf::translation(GeomVec::new(0.25, 0., 0.)),
            false,
            &policy
        )
        .unwrap()
    );
    assert!(operation.validation.is_valid());
    assert!(operation.recovery.actions.is_empty());
    let value = measure!(
        "transform_ms",
        input.transformed(&Trsf::translation(GeomVec::new(0.25, 0., 0.)))
    );
    let (value, _) = measure!(
        "pcurve_repair_ms",
        value.repair_operation_pcurves(&policy).unwrap()
    );
    let value = measure!(
        "merge_planes_ms",
        openrcad::algo::merge::merge_coplanar_faces_classed_with_policy(&value, None, &policy)
    );
    let value = measure!(
        "merge_cylinders_ms",
        openrcad::algo::merge::merge_cocylindrical_faces_classed_with_policy(&value, None, &policy)
    );
    let report = measure!(
        "validation_report_ms",
        openrcad::topo::ValidationReport::for_solid(&value, &policy)
    );
    assert!(report.is_valid());
    measure!(
        "strict_validation_ms",
        value.validate_strict_with_policy(&policy).unwrap()
    );
    let history = measure!(
        "history_ms",
        openrcad::topo::TopologyHistory::conservative_unary(input, &value)
    );
    assert!(history.coverage_for_solid(&value).is_complete());
    println!("{}", serde_json::Value::Object(stages));
}
