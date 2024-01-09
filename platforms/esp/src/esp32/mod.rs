use anyhow::Result;
use log::error;

use crate::common::{add_i2s_pin, LedI2s, LedI2sPlatform};
use esp_idf_sys::{i2s_hal_context_t, i2s_hal_init, lcd_periph_signals, periph_module_enable};

use rustled_platforms_common::LedSpec;

struct PlatformESP32 {}

impl LedI2sPlatform for PlatformESP32 {
    unsafe fn i2s_ll_ahbm_reset(&self, hal: i2s_hal_context_t) {
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(1);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(1);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(0);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(0);
    }

    unsafe fn i2s_ll_burst_en(&self, hal: i2s_hal_context_t) {
        (*hal.dev).lc_conf.__bindgen_anon_1.set_out_data_burst_en(1);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_outdscr_burst_en(1);
    }

    unsafe fn i2s_ll_set_out_dscr(&self, hal: i2s_hal_context_t) {
        (*hal.dev).int_raw.__bindgen_anon_1.set_out_dscr_err(1);
    }
}

unsafe fn configure_i2s(device: u32) -> std::result::Result<(), &'static str> {
    if device > lcd_periph_signals.buses.len() as u32 {
        return std::result::Result::Err("bad device");
    }
    periph_module_enable(lcd_periph_signals.buses[device as usize].module);
    let mut hal: i2s_hal_context_t = Default::default();
    i2s_hal_init(&mut hal, device as i32);

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

    // WS2812 spec copied from src/chipsets.h WS2812Controller800Khz in FastLED codebase
    let spec = LedSpec {
        t1_ns: 250,
        t2_ns: 625,
        t3_ns: 375,
    };
    let platform = Box::new(PlatformESP32 {});
    LedI2s::new(hal, device, spec, platform)
}

pub fn platform_init(pins: &[u32]) -> Result<()> {
    esp_idf_sys::link_patches();
    println!("Hello from Rust!");
    esp_idf_svc::log::EspLogger::initialize_default();
    unsafe {
        if let Err(error) = configure_i2s(0) {
            error!("I2S config error {error}");
        } else {
            for pin in pins {
                if let Err(error) = add_i2s_pin(*pin) {
                    error!("I2S error {error} adding pin {pin}");
                    break;
                }
            }
        }
    };

    Ok(())
}
