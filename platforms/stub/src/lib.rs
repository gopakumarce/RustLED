pub fn platform_init(_pins: &[u32]) -> anyhow::Result<()> {
    Ok(())
}

pub unsafe fn show_pixels(bytes: Vec<Vec<u8>>) -> (bool, Vec<Vec<u8>>) {
    (false, bytes)
}
