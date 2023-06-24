cfg_if::cfg_if! {
    if #[cfg(target_arch = "xtensa")] {
        pub mod esp32;
        pub mod common;
        pub use esp32::platform_init;
        pub use crate::common::show_pixels;
    }
}
