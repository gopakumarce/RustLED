use std::thread;
use std::time::Duration;
mod platform_libs;
use log::info;
use rustled_common::NUM_COLORS;

pub fn blink_leds() {
    loop {
        info!("Sleeping..");
        thread::sleep(Duration::from_secs(1));
    }
}

fn main() -> anyhow::Result<()> {
    println!("Initializing RustLED");
    platform_libs::platform_init(&[4])?;
    let colors = vec![vec![100; 256]; NUM_COLORS];
    unsafe { platform_libs::show_pixels(colors) };
    blink_leds();
    Ok(())
}
