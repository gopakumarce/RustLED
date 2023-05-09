# RustLED

* Install esp toolchain - there is an espup (https://github.com/esp-rs/espup) that makes these 
installations easy like using rustup. This step will give an export-esp.sh with some environment
variables which needs to be exported before proceeding further. This is what I add in my .bashrc

```
export ESP_IDF_SYS_ROOT_CRATE=rustled-sample-main
. ~/export-esp.sh
```

more about ESP_IDF_SYS_ROOT_CRATE later below

* To build, at the root of the workspace, say the below
  ```cargo +esp build --release --target=xtensa-esp32-espidf --package rustled-sample-main```
  This will just build everything and the example main binary included here
 
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

* If you are using a cargo workspace in your project, do "export ESP_IDF_SYS_ROOT_CRATE=<your-binary-crate-name>" so that
the embuild crate knows which is the binary crate. In a cargo workspace, "cargo metadata" does not know 
which is the binary crate, and embuild needs to know that information or else it will end up with some complaints
like "ldproxy --linker-arg missing" etc.. In a non-cargo-workspace situation this should not be needed.
So in this example we set "export ESP_IDF_SYS_ROOT_CRATE=rustled-sample-main"

Some more links on the various esp-idf environment options here - https://github.com/esp-rs/esp-idf-sys/blob/master/BUILD-OPTIONS.md

* To flash an image this is one way of doing it

cargo +esp install espflash
"sudo ~/.cargo/bin/espflash /dev/ttyS0 target/release/build/<binary name>  --speed 115200"

* The --monitor option with espflash doesnt seem to work too well, its easy to just use
"minicom -D /dev/ttyS0" to get a nice clean console output

