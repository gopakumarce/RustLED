use anyhow::Result;
use log::info;
use std::thread;
use std::time::Duration;

use crate::common::{
    start_mqtt, start_wifi, LedI2s, I2S_LL_EVENT_TX_EOF, MAX_CHANNELS, MAX_TRANSPOSED_WORDS,
    NUM_COLORS, PRECISION_TICKS,
};
use esp_idf_hal::prelude::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_sys::GPIO_MODE_DEF_OUTPUT;
use esp_idf_sys::{
    esp_intr_disable, esp_intr_enable, esp_rom_gpio_connect_out_signal, gpio_ll_iomux_func_sel,
    gpio_set_direction, i2s_hal_clock_info_t, i2s_hal_context_t, i2s_hal_init,
    i2s_hal_set_tx_clock, i2s_ll_clear_intr_status, i2s_ll_enable_intr, i2s_ll_rx_reset,
    i2s_ll_rx_reset_dma, i2s_ll_rx_reset_fifo, i2s_ll_rx_stop, i2s_ll_set_out_link_addr,
    i2s_ll_start_out_link, i2s_ll_tx_enable_intr, i2s_ll_tx_reset, i2s_ll_tx_reset_dma,
    i2s_ll_tx_reset_fifo, i2s_ll_tx_start, i2s_ll_tx_stop, lcd_periph_signals,
    periph_module_enable, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT, I2S_LL_MCLK_DIVIDER_MAX,
    PIN_FUNC_GPIO,
};

use rustled_platforms_common::{
    calculate_divisors, calibrate_clocks, transpose32, CalibrationResult, LedSpec,
};

// TODO: This should be replaced by esp_clk_tree_src_get_freq_hz whenever the
// latest esp-idf is released
const ESP32_I2S_DEF_PLL_FREQ: u64 = 160 * 1_000_000;
const BASE_CLOCK: u64 = ESP32_I2S_DEF_PLL_FREQ;

unsafe fn empty(i2s: &LedI2s) {
    let buf = if i2s.cur_buffer == 0 {
        i2s.descriptor0
    } else {
        i2s.descriptor1
    } as *mut u32;
    for i in 0..MAX_TRANSPOSED_WORDS * NUM_COLORS {
        let offset = i2s.pulses_per_bit * i;
        for j in 0..i2s.ones_for_zero {
            *buf.add(offset + j) = 0xffffffff;
        }
        for j in i2s.ones_for_one..i2s.pulses_per_bit {
            *buf.add(offset + j) = 0;
        }
    }
}

unsafe fn fill_buffer(i2s: &mut LedI2s, bytes: [&[u8]; NUM_COLORS]) {
    let num_pins = i2s.num_pins();
    if bytes.len() != num_pins * NUM_COLORS {
        info!("Bad pins {}, {}", bytes.len(), num_pins);
        return;
    }

    for c in 0..NUM_COLORS {
        for i in 0..num_pins {
            // Store the colors from pins in reverse order of pins
            i2s.pixel_row[c * NUM_COLORS + i] = bytes[c][num_pins - (i + 1)];
        }
    }

    let buf = if i2s.cur_buffer == 0 {
        i2s.descriptor0
    } else {
        i2s.descriptor1
    } as *mut u32;
    for color in 0..NUM_COLORS {
        transpose32(
            &i2s.pixel_row[color * NUM_COLORS * MAX_CHANNELS..],
            &mut i2s.pixel_bits[color * NUM_COLORS * MAX_CHANNELS..],
        );
        for word in 0..MAX_TRANSPOSED_WORDS {
            // Think of pixel_bits as a word which is in blocks of four bytes
            let byte0 = i2s.pixel_bits[color * NUM_COLORS + word];
            let byte1 = i2s.pixel_bits[color * NUM_COLORS + word + 1];
            let byte2 = i2s.pixel_bits[color * NUM_COLORS + word + 2];
            let byte3 = i2s.pixel_bits[color * NUM_COLORS + word + 3];
            // reverse the bits
            let word_bits =
                (byte0 << 24) as u32 | (byte1 << 16) as u32 | (byte2 << 8) as u32 | byte3 as u32;
            for pulse in i2s.ones_for_zero..i2s.ones_for_one {
                *buf.add(
                    color * MAX_TRANSPOSED_WORDS * i2s.pulses_per_bit
                        + word * i2s.pulses_per_bit
                        + pulse,
                ) = word_bits;
            }
        }
    }
}

unsafe fn i2s_reset(hal: &mut i2s_hal_context_t) {
    i2s_ll_tx_reset(hal.dev);
    i2s_ll_tx_reset_dma(hal.dev);
    i2s_ll_tx_reset_fifo(hal.dev);
    i2s_ll_rx_reset(hal.dev);
    i2s_ll_rx_reset_dma(hal.dev);
    i2s_ll_rx_reset_fifo(hal.dev);

    (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(1);
    (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(1);
    (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(0);
    (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(0);
}

unsafe fn i2s_stop(i2s: &mut LedI2s) {
    esp_intr_disable(i2s.intr_handle);
    i2s_reset(&mut i2s.hal);
    i2s_ll_rx_stop(i2s.hal.dev);
    i2s_ll_tx_stop(i2s.hal.dev);
}

unsafe fn i2s_start(i2s: &mut LedI2s, descriptor: u32) {
    i2s_reset(&mut i2s.hal);
    (*i2s.hal.dev)
        .lc_conf
        .__bindgen_anon_1
        .set_out_data_burst_en(1);
    (*i2s.hal.dev)
        .lc_conf
        .__bindgen_anon_1
        .set_outdscr_burst_en(1);
    i2s_ll_set_out_link_addr(i2s.hal.dev, descriptor);
    i2s_ll_start_out_link(i2s.hal.dev);

    i2s_ll_clear_intr_status(i2s.hal.dev, I2S_LL_EVENT_TX_EOF);
    (*i2s.hal.dev).int_raw.__bindgen_anon_1.set_out_dscr_err(1);

    i2s_ll_enable_intr(i2s.hal.dev, I2S_LL_EVENT_TX_EOF, true);
    esp_intr_enable(i2s.intr_handle);

    (*i2s.hal.dev).int_ena.val = 0;
    i2s_ll_tx_enable_intr(i2s.hal.dev);

    //start transmission
    i2s_ll_tx_start(i2s.hal.dev);
}

unsafe fn configure_i2s(device: u32) -> std::result::Result<LedI2s, &'static str> {
    if device > lcd_periph_signals.buses.len() as u32 {
        return std::result::Result::Err("bad device");
    }
    periph_module_enable(lcd_periph_signals.buses[device as usize].module);
    let mut hal: i2s_hal_context_t = Default::default();
    i2s_hal_init(&mut hal, device as i32);
    i2s_reset(&mut hal);

    (*hal.dev).conf.__bindgen_anon_1.set_tx_msb_right(1);
    (*hal.dev).conf.__bindgen_anon_1.set_tx_mono(0);
    (*hal.dev).conf.__bindgen_anon_1.set_tx_short_sync(0);
    (*hal.dev).conf.__bindgen_anon_1.set_tx_msb_shift(0);
    (*hal.dev).conf.__bindgen_anon_1.set_tx_right_first(1);
    (*hal.dev).conf.__bindgen_anon_1.set_tx_slave_mod(0);

    (*hal.dev).conf2.val = 0;
    (*hal.dev).conf2.__bindgen_anon_1.set_lcd_en(1);
    (*hal.dev).conf2.__bindgen_anon_1.set_lcd_tx_wrx2_en(0);
    (*hal.dev).conf2.__bindgen_anon_1.set_lcd_tx_sdx2_en(0);

    (*hal.dev).sample_rate_conf.val = 0;
    (*hal.dev)
        .sample_rate_conf
        .__bindgen_anon_1
        .set_tx_bits_mod(32);
    (*hal.dev)
        .sample_rate_conf
        .__bindgen_anon_1
        .set_tx_bck_div_num(1);

    (*hal.dev).clkm_conf.val = 0;
    (*hal.dev).clkm_conf.__bindgen_anon_1.set_clka_en(0);

    // WS2812 spec copied from src/chipsets.h WS2812Controller800Khz in FastLED codebase
    let spec = LedSpec {
        t1_ns: 250,
        t2_ns: 625,
        t3_ns: 375,
    };
    let clk_cfg = configure_clocks(&mut hal, &spec);

    (*hal.dev).fifo_conf.val = 0;
    (*hal.dev)
        .fifo_conf
        .__bindgen_anon_1
        .set_tx_fifo_mod_force_en(1);
    (*hal.dev).fifo_conf.__bindgen_anon_1.set_tx_fifo_mod(3);
    (*hal.dev).fifo_conf.__bindgen_anon_1.set_tx_data_num(32);
    (*hal.dev).fifo_conf.__bindgen_anon_1.set_dscr_en(1);

    (*hal.dev).conf1.val = 0;
    (*hal.dev).conf1.__bindgen_anon_1.set_tx_stop_en(0);
    (*hal.dev).conf1.__bindgen_anon_1.set_tx_pcm_bypass(1);

    (*hal.dev).conf_chan.val = 0;
    (*hal.dev).conf_chan.__bindgen_anon_1.set_tx_chan_mod(1);
    (*hal.dev).timing.val = 0;

    LedI2s::new(hal, device, &clk_cfg)
}

unsafe fn add_i2_pin(i2s: &mut LedI2s, data_pin: u32) -> std::result::Result<(), &'static str> {
    let Some(i2s_pin) = i2s.next_pin() else {
        return std::result::Result::Err("i2s lcd out of pins");
    };
    gpio_ll_iomux_func_sel(data_pin, PIN_FUNC_GPIO);
    gpio_set_direction(data_pin as i32, GPIO_MODE_DEF_OUTPUT);
    esp_rom_gpio_connect_out_signal(data_pin, i2s_pin, false, false);

    Ok(())
}

unsafe fn configure_clocks(hal: &mut i2s_hal_context_t, spec: &LedSpec) -> CalibrationResult {
    let result = calibrate_clocks(spec, BASE_CLOCK, PRECISION_TICKS);
    let mclk = BASE_CLOCK as u32 / result.divisor as u32;
    let divisors = calculate_divisors(
        BASE_CLOCK as u32,
        mclk,
        result.divisor as u32,
        I2S_LL_MCLK_DIVIDER_MAX,
    );
    info!(
        "sclk {}, mclk {}, mclk_div {}, mclk_div1 {}, 
        numerator {}, denominator {}",
        BASE_CLOCK,
        mclk,
        result.divisor,
        divisors.divisor,
        divisors.numerator,
        divisors.denominator
    );
    let cfg = i2s_hal_clock_info_t {
        sclk: BASE_CLOCK as u32,
        mclk,
        mclk_div: result.divisor as u16,
        bclk: mclk as u32,
        bclk_div: 1,
    };
    i2s_hal_set_tx_clock(hal, &cfg, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT);

    result
}

fn blink_leds() {
    loop {
        info!("Sleeping..");
        thread::sleep(Duration::from_secs(1));
    }
}

pub fn platform_init() -> Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_sys::link_patches();
    println!("Hello from Rust!");
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take()?;
    let _wifi = start_wifi(peripherals.modem, sysloop)?;
    let _mqtt = start_mqtt()?;
    let _i2s0 = unsafe { configure_i2s(0) };
    blink_leds();

    Ok(())
}
