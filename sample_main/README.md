# RustLED

* To build the code here say "cargo +esp build"

* See instructions to setup an environment with the right toolchains etc.. required for building
ESP projects using rust here https://github.com/esp-rs/esp-idf-template

* The template installation still asked me for the esp libclang path, so I had to follow instructions
here https://github.com/esp-rs/rust-build#linux-and-macos - those instructions install libclang and 
xtensa-esp32-elf-gcc, but the xtensa-esp32-elf-gcc seems to be installed by the initial template 
creation mentioned in the link mentioned above, so really only the libclang needs
to be required from the link here. And the environment variable LIBCLANG_PATH should be updated as 
mentioned in that link.

NOTE: on ubuntu 18.04, the libc version is 2.27 and that wont work with cargo +esp - it needs glibc
2.29. Ubuntu 20.04 is the minimum release which has 2.29

* The embuild crate is what compiles and links to esp-idf C code and generates rust bindings for
it. And that crate then propagates various environment variables and linker flags etc.. as cargo build
script output (cargo:foo=bar variables). And those variables and flags need to be available to whichever
crate is the "main" crate (binary crate) because those flags need to be available during link time etc..
Because of that, even though the main crate does not directly depend on esp-idf-sys (it needs to depend
only on platform_esp), it still needs  to add esp-idf-sys as a dependency and it still needs a build.rs 
that with a couple of lines to parse and pass all those flags to embuild etc.. So if you use this crate
in your own project with your own main/binary crate, do the following
-- add a dependency to esp-idf-sys just like we have done in sample_main
-- add a build.rs like we have in sample_main.  
-- add a .cargo/config.toml in the sample_main with the settings needed for esp.
-- In the Cargo.toml of your binary crate add the below
   ```
   [[package.metadata.esp-idf-sys.extra_components]]
   bindings_header = "bindings.h"
   ```
-- Copy the bindings.h header file here to your binary crate
   TODO: This is the UGLIEST step of all, need to figure out some way to avoid this step at least

* If you are using a cargo workspace, do "export ESP_IDF_SYS_ROOT_CRATE=<your-binary-crate-name>" so that
the embuild crate knows which is the binary crate. In a cargo workspace, "cargo metadata" does not know 
which is the binary crate, and embuild needs to know that information or else it will end up with some complaints
like "ldproxy --linker-arg missing" etc.. In a non-cargo-workspace situation this should not be needed.
So in this example we set "export ESP_IDF_SYS_ROOT_CRATE=rustled-sample-main"

