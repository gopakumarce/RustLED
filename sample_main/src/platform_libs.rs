cfg_if::cfg_if! {
    if #[cfg(target_arch = "xtensa")] {
        pub use rustled_platforms_esp::platform_init;
        pub use rustled_platforms_esp::common::show_pixels;

    } else {
        pub use rustled_platforms_stub::platform_init;
        pub use rustled_platforms_stub::show_pixels;
    }
}
