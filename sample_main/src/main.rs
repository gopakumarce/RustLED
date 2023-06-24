mod platform_libs;

fn main() -> anyhow::Result<()> {
    platform_libs::platform_init()?;
    Ok(())
}
