mod platform_libs;

use esp_idf_sys::esp_timer_get_time;
use log::info;
use platform_libs::{delay, show_pixels};
use rustled_common::{BLUE, GREEN, NUM_COLORS, RED};
use rustled_platforms_common::{ColorOrder, LedSpec};

const NUM_LEDS: usize = 50;
const NUM_PINS: usize = 2;
const GPIO4: u32 = 4;
const GPIO2: u32 = 2;

// This is from the Arduino examples FastLED->ColorPalette where we
// just do one call of FillLEDsFromPaletteColors(0) and FastLED.show()
// and inside the fillBuffer() API in FastLED, we dump the final u8
// values that are sent to the DMA buffers and paste that here.
fn fastled_color_palette_example(colors: &mut Vec<Vec<Vec<u8>>>, pin: usize) {
    let r = [
        0x00000040, 0x0000003e, 0x0000003c, 0x00000039, 0x00000037, 0x00000036, 0x00000034,
        0x00000032, 0x0000002f, 0x0000002d, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b,
        0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b,
        0x0000002b, 0x00000027, 0x00000023, 0x0000001e, 0x0000001a, 0x00000016, 0x00000013,
        0x0000000e, 0x0000000a, 0x00000006, 0x00000002, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000,
    ];
    let g = [
        0x00000000, 0x00000001, 0x00000003, 0x00000004, 0x00000005, 0x00000007, 0x00000008,
        0x0000000a, 0x0000000b, 0x0000000d, 0x0000000e, 0x0000000f, 0x00000011, 0x00000012,
        0x00000013, 0x00000015, 0x00000016, 0x00000018, 0x00000019, 0x0000001b, 0x0000001c,
        0x0000001e, 0x0000001f, 0x00000020, 0x00000022, 0x00000023, 0x00000024, 0x00000026,
        0x00000027, 0x00000029, 0x0000002a, 0x0000002b, 0x0000002c, 0x0000002b, 0x00000029,
        0x00000028, 0x00000026, 0x00000025, 0x00000024, 0x00000022, 0x00000021, 0x0000001f,
        0x0000001e, 0x0000001c, 0x00000019, 0x00000016, 0x00000013, 0x00000010, 0x0000000e,
        0x0000000b,
    ];
    let b = [
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000002, 0x00000004,
        0x00000006, 0x00000008, 0x0000000a, 0x0000000b, 0x0000000d, 0x0000000f, 0x00000011,
        0x00000013, 0x00000016, 0x0000001a, 0x0000001e, 0x00000022, 0x00000025, 0x00000029,
        0x0000002d,
    ];
    colors[pin][RED].extend_from_slice(&r);
    colors[pin][GREEN].extend_from_slice(&g);
    colors[pin][BLUE].extend_from_slice(&b);
}

fn all_red(colors: &mut Vec<Vec<Vec<u8>>>, pin: usize) {
    for _ in 0..NUM_LEDS {
        colors[pin][RED].push(255);
        colors[pin][GREEN].push(0);
        colors[pin][BLUE].push(0);
    }
}

fn all_green(colors: &mut Vec<Vec<Vec<u8>>>, pin: usize) {
    for _ in 0..NUM_LEDS {
        colors[pin][RED].push(0);
        colors[pin][GREEN].push(255);
        colors[pin][BLUE].push(0);
    }
}

fn all_blue(colors: &mut Vec<Vec<Vec<u8>>>, pin: usize) {
    for _ in 0..NUM_LEDS {
        colors[pin][RED].push(0);
        colors[pin][GREEN].push(0);
        colors[pin][BLUE].push(255);
    }
}

fn alternate_rgb(colors: &mut Vec<Vec<Vec<u8>>>, pin: usize) {
    let mut next = 0;
    for _ in 0..NUM_LEDS {
        if next == 0 {
            // red
            colors[pin][RED].push(255);
            colors[pin][GREEN].push(0);
            colors[pin][BLUE].push(0);
        } else if next == 1 {
            // green
            colors[pin][RED].push(0);
            colors[pin][GREEN].push(255);
            colors[pin][BLUE].push(0);
        } else if next == 2 {
            // blue
            colors[pin][RED].push(0);
            colors[pin][GREEN].push(0);
            colors[pin][BLUE].push(255);
        }
        next = (next + 1) % 3;
    }
}

fn colors_clear(colors: &mut Vec<Vec<Vec<u8>>>, num_pins: usize) {
    for pin in 0..num_pins {
        colors[pin][RED].clear();
        colors[pin][GREEN].clear();
        colors[pin][BLUE].clear();
    }
}

fn main() -> anyhow::Result<()> {
    println!("Initializing RustLED");
    // WS2811 spec copied from src/chipsets.h WS2811Controller800Khz in FastLED codebase
    let spec = LedSpec {
        // ws2811 specs from FastLED chipsets.h
        order: ColorOrder::BRG,
        t1_ns: 320,
        t2_ns: 320,
        t3_ns: 640,
    };

    let pins: [u32; NUM_PINS] = [GPIO4, GPIO2];
    platform_libs::platform_init(&pins, spec)?;
    let mut colors = vec![vec![vec![]; NUM_COLORS]; NUM_PINS];
    let mut success;

    loop {
        info!("Sleeping.. {}", unsafe { esp_timer_get_time() / 1000000 });
        fastled_color_palette_example(&mut colors, 0);
        alternate_rgb(&mut colors, 1);
        (success, colors) = unsafe { show_pixels(colors) };
        assert!(success);
        colors_clear(&mut colors, 2);
        unsafe { delay(1000) };

        alternate_rgb(&mut colors, 0);
        fastled_color_palette_example(&mut colors, 1);
        (success, colors) = unsafe { show_pixels(colors) };
        assert!(success);
        colors_clear(&mut colors, 2);
        unsafe { delay(1000) };

        all_red(&mut colors, 0);
        all_blue(&mut colors, 1);
        (success, colors) = unsafe { show_pixels(colors) };
        assert!(success);
        colors_clear(&mut colors, 2);
        unsafe { delay(1000) };

        all_green(&mut colors, 0);
        all_green(&mut colors, 1);
        (success, colors) = unsafe { show_pixels(colors) };
        assert!(success);
        colors_clear(&mut colors, 2);
        unsafe { delay(1000) };

        all_blue(&mut colors, 0);
        all_red(&mut colors, 1);
        (success, colors) = unsafe { show_pixels(colors) };
        assert!(success);
        colors_clear(&mut colors, 2);
        unsafe { delay(1000) };
    }
}
