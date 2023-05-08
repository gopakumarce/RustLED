# RustLED

LED Display stuff for neopixel/WS2812 like LED panels.

The initial template was setup using instructions here https://github.com/esp-rs/esp-idf-template and
also here https://betterprogramming.pub/rust-for-iot-is-it-time-67b14ab34b8 .. From the second link,
all that I did was use the nightly toolchains (is that needed ?), i.e I did the below

```
rustup install nightly
rustup component add rust-src --toolchain nightly
rustup target add riscv32imc-unknown-none-elf
```

But the template installation still asked me for the esp libclang path, so I had to follow instructions
here https://github.com/esp-rs/rust-build#linux-and-macos - those instructions install libclang and 
xtensa-esp32-elf-gcc, but the xtensa-esp32-elf-gcc seems to be installed by the initial template 
creation mentioned in the very first link mentioned in this section, so really only the libclang needs
to be required from this above link. And the environment variable LIBCLANG_PATH should be updated as 
mentioned in that link.

Note that on ubuntu 18.04, the libc version is 2.27 and that wont work with cargo +esp - it needs glibc
2.29. So 20.04 is the minimum release which has 2.29

The code here is heavily borrowed from the examples here https://github.com/ivmarkov/rust-esp32-std-demo

