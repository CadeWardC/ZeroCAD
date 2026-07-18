#![no_main]

use libfuzzer_sys::fuzz_target;
use zerocad_core::BooleanCaseV1;

fuzz_target!(|data: &[u8]| {
    let case = BooleanCaseV1::from_fuzz_bytes(data);
    let identity = case.identity().expect("bounded decoder emits valid cases");
    let replay = case
        .replay()
        .expect("valid case reaches a structured outcome");
    assert_eq!(replay.identity, identity.hex());
});
