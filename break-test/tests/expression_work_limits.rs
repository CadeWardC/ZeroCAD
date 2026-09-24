//! Expressions loaded from documents must not exhaust a worker's stack or memory.
use zerocad_core::expr;

#[test]
fn deeply_nested_input_is_rejected_without_process_abort() {
    const CHILD: &str = "ZEROCAD_BREAK_EXPRESSION_CHILD";
    if std::env::var_os(CHILD).is_some() {
        let expression = format!("{}1{}", "(".repeat(10_000), ")".repeat(10_000));
        assert!(expr::eval(&expression, &Default::default()).is_err());
        return;
    }
    // Stack overflow aborts cannot be caught with catch_unwind. Isolate the
    // reproducer so failure is reported by the test harness, not a lost suite.
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "deeply_nested_input_is_rejected_without_process_abort",
            "--nocapture",
        ])
        .env(CHILD, "1")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "child failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn long_unary_chains_do_not_use_recursive_stack_frames() {
    for count in [64, 255, 1024, 8191] {
        let expression = format!("{}2", "-".repeat(count));
        assert_eq!(
            expr::eval(&expression, &Default::default()).unwrap(),
            if count % 2 == 0 { 2. } else { -2. }
        );
    }
}

#[test]
fn oversized_text_is_rejected_by_all_tokenizing_entry_points() {
    let expression = "1+".repeat(100_000);
    assert!(expr::eval(&expression, &Default::default())
        .unwrap_err()
        .contains("limit"));
    assert!(expr::identifiers(&expression)
        .unwrap_err()
        .contains("limit"));
}

#[test]
fn ordinary_depth_and_long_flat_arithmetic_still_work() {
    for depth in [0, 1, 16, 32] {
        let expression = format!("{}2+3{}", "(".repeat(depth), ")".repeat(depth));
        assert_eq!(expr::eval(&expression, &Default::default()).unwrap(), 5.);
    }
    let expression = std::iter::repeat_n("1", 2048).collect::<Vec<_>>().join("+");
    assert_eq!(expr::eval(&expression, &Default::default()).unwrap(), 2048.);
}
