use anyhow::Result;
use log::error;

use crate::common::{add_i2s_pin, start_mqtt, start_wifi, LedI2s, LedI2sPlatform};
use esp_idf_hal::prelude::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_sys::{
    i2s_fifo_init, i2s_hal_context_t, i2s_hal_init, lcd_periph_signals, periph_module_enable,
    I2S_OUTDSCR_BURST_EN_S, I2S_OUT_DATA_BURST_EN_S,
};

use rustled_platforms_common::LedSpec;

const I2S_BASE_CLK: usize = 80000000;

struct PlatformESP32 {}

impl LedI2sPlatform for PlatformESP32 {
    unsafe fn reset(&self, hal: i2s_hal_context_t) {
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(1);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(1);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_rst(0);
        (*hal.dev).lc_conf.__bindgen_anon_1.set_ahbm_fifo_rst(0);
    }

    unsafe fn start(&self, hal: i2s_hal_context_t) {
        let data_burst_en = 1 << I2S_OUT_DATA_BURST_EN_S;
        let dscr_burst_en = 1 << I2S_OUTDSCR_BURST_EN_S;
        (*hal.dev).lc_conf.val = data_burst_en | dscr_burst_en | data_burst_en;
        (*hal.dev).int_raw.__bindgen_anon_1.set_out_dscr_err(1);
    }
}

unsafe fn configure_i2s(device: u32, spec: LedSpec) -> std::result::Result<(), &'static str> {
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

    i2s_fifo_init(hal.dev);

    (*hal.dev).conf1.val = 0;
    (*hal.dev).conf1.__bindgen_anon_1.set_tx_stop_en(0);
    (*hal.dev).conf1.__bindgen_anon_1.set_tx_pcm_bypass(1);

    (*hal.dev).conf_chan.val = 0;
    (*hal.dev).conf_chan.__bindgen_anon_1.set_tx_chan_mod(1);
    (*hal.dev).timing.val = 0;

    let platform = Box::new(PlatformESP32 {});
    /*
     * A note on I2S_BASE_CLK. The actual default PLL clock frequency on esp32 is 160Mhz.
     * We can get the frequency using the call to esp_clk_tree_src_get_freq_hz(). With
     * that used in the scale_clocks() math, the divisors we get works well for ws2812b,
     * but ws2811 goes crazy - basically ws2811 needs double the clock that comes out if
     * we do the math with 160Mhz. The FastLED clockless_i2s_esp32.h uses the hardcoded
     * I2S_BASE_CLK which looked mysterious rather than just getting it dynamically using
     * the clk_tree call, but maybe the above is the reason, so we do the same here till
     * we figure out what the right math is
     */
    LedI2s::new(hal, device, spec, I2S_BASE_CLK, platform)
}

pub fn platform_init(pins: &[u32], spec: LedSpec) -> Result<()> {
    esp_idf_sys::link_patches();
    println!("Hello from Rust!");
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take()?;
    if let Err(error) = start_wifi(peripherals.modem, sysloop) {
        error!("Wifi Error {error}");
    }
    if let Err(error) = start_mqtt() {
        error!("Mqtt Error {error}")
    }
    unsafe {
        if let Err(error) = configure_i2s(0, spec) {
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
