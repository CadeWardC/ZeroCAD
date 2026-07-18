#![no_main]

use libfuzzer_sys::fuzz_target;
use zerocad_core::{
    AnalyticPrimitiveV1, BooleanCaseV1, BooleanContactClass, BooleanOperandV1,
    BooleanReplayOutcome, BooleanVerificationOp, RigidTransformV1,
};

fuzz_target!(|data: &[u8]| {
    let byte = |index: usize| data.get(index).copied().unwrap_or(index as u8);
    let side = 1.0 + f64::from(byte(0)) / 255.0 * 30.0;
    let overlap = 0.05 + f64::from(byte(1)) / 255.0 * 0.90;
    let object = BooleanOperandV1 {
        primitive: AnalyticPrimitiveV1::Box { size: [side; 3] },
        transform: RigidTransformV1::default(),
    };
    let tool = BooleanOperandV1 {
        primitive: AnalyticPrimitiveV1::Box { size: [side; 3] },
        transform: RigidTransformV1 {
            translation: [side * (1.0 - overlap), 0.0, 0.0],
            ..RigidTransformV1::default()
        },
    };
    let case = BooleanCaseV1 {
        operation: BooleanVerificationOp::Union,
        object,
        tool,
        contact_class: BooleanContactClass::Overlapping,
        logarithmic_scale: (byte(2) % 5) as i8 - 2,
    };
    let replay = case.replay().expect("bounded overlap case is valid");
    assert_eq!(replay.outcome, BooleanReplayOutcome::Resolved);
    assert_eq!(replay.body_count, 1);
});
