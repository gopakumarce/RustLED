cfg_if::cfg_if! {
    if #[cfg(target_arch = "xtensa")] {
        pub use rustled_platforms_esp::platform_init;
    } else {
        pub use rustled_platforms_stub::platform_init;
    }
}
