# RustLED

LED Display stuff for neopixel/WS2812 like LED panels.

The initial template was setup using instructions here https://github.com/esp-rs/esp-idf-template and
also here https://betterprogramming.pub/rust-for-iot-is-it-time-67b14ab34b8 .. Note that the 
rust-toolchain.toml has a channel=esp setting which is what makes the cargo commands to search for
targets outside the normal default targets like x86_64-unknown-linux etc.., we can also specify 
channel as cargo +esp --target xtensa-esp32-espidf for example

The template installation still asked me for the esp libclang path, so I had to follow instructions
here https://github.com/esp-rs/rust-build#linux-and-macos - those instructions install libclang and 
xtensa-esp32-elf-gcc, but the xtensa-esp32-elf-gcc seems to be installed by the initial template 
creation mentioned in the very first link mentioned in this section, so really only the libclang needs
to be required from this above link. And the environment variable LIBCLANG_PATH should be updated as 
mentioned in that link.

Note that on ubuntu 18.04, the libc version is 2.27 and that wont work with cargo +esp - it needs glibc
2.29. So 20.04 is the minimum release which has 2.29

The code here is heavily borrowed from the examples here https://github.com/ivmarkov/rust-esp32-std-demo
