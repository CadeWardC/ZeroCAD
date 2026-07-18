# Boolean fuzzing

These targets are intentionally a pinned-nightly Ubuntu/WSL2 lane. Stable
Windows CI replays promoted JSON cases through `zerocad-core/tests/boolean_replay.rs`
and never needs the libFuzzer runtime.

```text
rustup toolchain install nightly-2026-07-01
cargo +nightly-2026-07-01 install cargo-fuzz --locked
cargo +nightly-2026-07-01 fuzz run boolean_safety
cargo +nightly-2026-07-01 fuzz run boolean_resolvable
```

Promote a minimized finding by recording its `BooleanCaseV1` JSON under
`zerocad-core/tests/boolean_cases/`; the SHA-256 identity always comes from the
canonical binary encoding, never from JSON formatting.
