//! Machine-readable canonical-case observations for the external OCCT runner.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("expected case JSON path")?;
    let value: serde_json::Value = serde_json::from_slice(&std::fs::read(path)?)?;
    let case: zerocad_core::BooleanCaseV1 =
        serde_json::from_value(value.get("case").unwrap_or(&value).clone())?;
    let observation = case.observe();
    println!(
        "{}",
        serde_json::json!({
            "identity": case.identity()?.hex(),
            "observation": observation.as_ref().ok(),
            "error": observation.as_ref().err(),
        })
    );
    Ok(())
}
