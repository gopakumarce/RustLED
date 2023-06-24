const ESP_TARGETS: [&str; 1] = ["xtensa"];
// Necessary because of this issue: https://github.com/rust-lang/cargo/issues/9641
fn main() -> Result<(), Box<dyn std::error::Error>> {
    if let Ok(var) = std::env::var("CARGO_CFG_TARGET_ARCH") {
        if ESP_TARGETS.iter().any(|s| *s == var) {
            embuild::build::CfgArgs::output_propagated("ESP_IDF")?;
            embuild::build::LinkArgs::output_propagated("ESP_IDF")?;
        }
    }
    Ok(())
}
