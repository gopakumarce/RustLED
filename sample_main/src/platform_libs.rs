cfg_if::cfg_if! {
    if #[cfg(target_arch = "xtensa")] {
        pub use rustled_platforms_esp::platform_init;
        pub use rustled_platforms_esp::common::show_pixels;
        pub use rustled_platforms_esp::common::delay;

    } else {
        pub use rustled_platforms_stub::platform_init;
        pub use rustled_platforms_stub::show_pixels;
        pub use rustled_platforms_stub::delay;
    }
}
