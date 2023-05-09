use anyhow::Result;
use embedded_svc::mqtt::client::{Connection, MessageImpl, QoS};
use embedded_svc::utils::mqtt::client::ConnState;
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_hal::peripheral;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use log::{error, info};
use std::time::Duration;
use std::{env, ptr, thread};

use esp_idf_hal::prelude::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_sys::{
    esp_intr_alloc_intrstatus, esp_intr_disable, esp_intr_enable, esp_rom_gpio_connect_out_signal,
    gpio_ll_iomux_func_sel, gpio_set_direction, heap_caps_malloc, i2s_hal_clock_info_t,
    i2s_hal_context_t, i2s_hal_init, i2s_hal_set_tx_clock, i2s_ll_clear_intr_status,
    i2s_ll_enable_intr, i2s_ll_get_intr_status_reg, i2s_ll_rx_reset, i2s_ll_rx_reset_dma,
    i2s_ll_rx_reset_fifo, i2s_ll_rx_stop, i2s_ll_set_out_link_addr, i2s_ll_start_out_link,
    i2s_ll_tx_enable_intr, i2s_ll_tx_reset, i2s_ll_tx_reset_dma, i2s_ll_tx_reset_fifo,
    i2s_ll_tx_start, i2s_ll_tx_stop, intr_handle_t, lcd_periph_signals, lldesc_t,
    periph_module_enable, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT, ESP_INTR_FLAG_INTRDISABLED,
    ESP_INTR_FLAG_LOWMED, ESP_INTR_FLAG_SHARED, ESP_OK, I2S_LL_MCLK_DIVIDER_MAX, MALLOC_CAP_DMA,
    PIN_FUNC_GPIO,
};
use esp_idf_sys::{EspError, GPIO_MODE_DEF_OUTPUT};

const SSID: &str = env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
const PASSWORD: &str = env!("RUST_ESP32_STD_DEMO_WIFI_PASS");
const DATA_PIN: u32 = 4;
const I2S_LL_EVENT_TX_EOF: u32 = 1 << 12;
const NUM_COLOR_CHANNELS: usize = 3;
// TODO: This should be replaced by esp_clk_tree_src_get_freq_hz whenever the
// latest esp-idf is released
const ESP32_I2S_DEF_PLL_FREQ: u64 = 160 * 1_000_000;
const BASE_CLOCK: u64 = ESP32_I2S_DEF_PLL_FREQ;
const NANOSECS: u64 = 1_000_000_000;
const PRECISION_TICKS: usize = 10;

struct LedI2s {
    hal: i2s_hal_context_t,
    next: usize,
    device: u32,
    intr_handle: intr_handle_t,
    descriptor0: *const lldesc_t,
    descriptor1: *const lldesc_t,
}

impl LedI2s {
    unsafe fn next_pin(&mut self) -> Option<u32> {
        let device = self.device as usize;
        if self.next >= lcd_periph_signals.buses[device].data_sigs.len() {
            return None;
        }
        let ret = Some(lcd_periph_signals.buses[device].data_sigs[self.next] as u32);
        self.next += 1;
        ret
    }
}

struct LedSpec {
    t1_ns: usize,
    t2_ns: usize,
    t3_ns: usize,
}

fn blink_leds() {
    loop {
        info!("Sleeping..");
        thread::sleep(Duration::from_secs(1));
    }
}

fn main() -> Result<()> {
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

fn start_mqtt() -> Result<EspMqttClient<ConnState<MessageImpl, EspError>>> {
    info!("About to start MQTT client");

    let conf = MqttClientConfiguration {
        client_id: Some("FooBar"),
        ..Default::default()
    };
    let (mut client, mut connection) =
        EspMqttClient::new_with_conn("mqtt://73.234.163.55:1883", &conf)?;

    info!("MQTT client started");

    thread::spawn(move || {
        info!("MQTT Listening for messages");
        while let Some(msg) = connection.next() {
            match msg {
                Err(e) => info!("MQTT Message ERROR: {}", e),
                Ok(msg) => info!("MQTT Message: {:?}", msg),
            }
        }
        info!("MQTT connection loop exit");
    });

    client.subscribe("topic/messages", QoS::AtMostOnce)?;
    info!("Subscribed to all topics (rust-esp32-std-demo)");

    Ok(client)
}

fn start_wifi(
    modem: impl peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<BlockingWifi<EspWifi<'static>>> {
    info!("Starting wifi..");
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), Some(nvs))?, sysloop)?;

    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.into(),
        channel: None,
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start()?;
    info!("Wifi started");

    wifi.connect()?;
    info!("Wifi connected");

    wifi.wait_netif_up()?;
    info!("Wifi netif up");

    Ok(wifi)
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

unsafe extern "C" fn i2s_interrupt_handler(_arg: *mut ::core::ffi::c_void) {}

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
    configure_clocks(&mut hal, &spec);

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

    // TODO: setup DMA descriptors (link_addr ?)

    let interrupt_source = lcd_periph_signals.buses[device as usize].irq_id;
    let mut intr_handle = ptr::null_mut();
    let isr_flags: u32 = ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
    let ret = esp_intr_alloc_intrstatus(
        interrupt_source,
        isr_flags as i32,
        i2s_ll_get_intr_status_reg(hal.dev) as u32,
        I2S_LL_EVENT_TX_EOF,
        Some(i2s_interrupt_handler),
        ptr::null_mut(),
        &mut intr_handle,
    );
    if ret != ESP_OK {
        error!("i2s interrupt alloc failed {ret}");
        return Err("i2s interrupt alloc failed");
    }
    // Keep interrupts off for now
    i2s_ll_enable_intr(hal.dev, I2S_LL_EVENT_TX_EOF, false);

    // TODO: calculate pulses_per_bit
    let pulses_per_bit = 100;
    let descriptor0 = allocate_dma_buffers(32 * NUM_COLOR_CHANNELS * pulses_per_bit);
    let descriptor1 = allocate_dma_buffers(32 * NUM_COLOR_CHANNELS * pulses_per_bit);
    (*descriptor0).__bindgen_anon_1.qe.stqe_next = descriptor1;
    (*descriptor1).__bindgen_anon_1.qe.stqe_next = descriptor0;

    Ok(LedI2s {
        hal,
        next: 0,
        device,
        intr_handle,
        descriptor0,
        descriptor1,
    })
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

unsafe fn allocate_dma_buffers(bufsize: usize) -> *mut lldesc_t {
    let size = std::mem::size_of::<lldesc_t>();
    let raw: *mut u8 = unsafe { heap_caps_malloc(size, MALLOC_CAP_DMA) as *mut u8 };
    unsafe {
        ptr::write_bytes(raw, 0, size);
    }
    let mut descriptor = raw as *mut lldesc_t;

    let buffer: *mut u8 = unsafe { heap_caps_malloc(size, MALLOC_CAP_DMA) as *mut u8 };
    unsafe {
        ptr::write_bytes(buffer, 0, bufsize);
    }

    (*descriptor).set_length(bufsize as u32);
    (*descriptor).set_size(bufsize as u32);
    (*descriptor).set_owner(1);
    (*descriptor).set_sosf(1);
    (*descriptor).buf = buffer;
    (*descriptor).set_offset(0);
    (*descriptor).__bindgen_anon_1.empty = 0;
    (*descriptor).set_eof(1);
    (*descriptor).__bindgen_anon_1.qe.stqe_next = ptr::null_mut();

    descriptor
}

// Copied off i2s_ll_tx_set_mclk, just for debugging, to see what values
// are eventually set
fn calculate_divisors(sclk: u32, mclk: u32, mut mclk_div: u32) -> (u32, u32, u32) {
    let mut ma: u32;
    let mut mb: u32;
    let mut denominator: u32 = 1;
    let mut numerator: u32 = 0;

    let freq_diff: u32 = sclk.abs_diff(mclk * mclk_div) as u32;
    if freq_diff == 0 {
        return (mclk_div, numerator, denominator);
    }
    let decimal: f64 = freq_diff as f64 / mclk as f64;
    // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 / 126.0
    if decimal > 125.0 / 126.0 {
        mclk_div += 1;
        return (mclk_div, numerator, denominator);
    }
    let mut min: u32 = !0;
    for a in 2..=I2S_LL_MCLK_DIVIDER_MAX {
        let b = ((a as f64 * (freq_diff as f64 / mclk as f64)) as f64 + 0.5) as u32;
        ma = freq_diff * a;
        mb = mclk * b;
        if ma == mb {
            denominator = a;
            numerator = b;
            return (mclk_div, numerator, denominator);
        }
        if (mb.abs_diff(ma) - ma) < min {
            denominator = a;
            numerator = b;
            min = mb.abs_diff(ma);
        }
    }

    (mclk_div, numerator, denominator)
}

// The led spec is basically copied from FastLED code, its such that a zero is
// represented by signal that is high for t1 nsecs and low for t2+t3 nsecs.
// And a one is represented as high for t1+t2 nsecs and low for t3 nsecs.
// There are no t1/t2/t3 in the manufacturer's specs, the manufacturers just
// specify the high/low times for zero and one. The FastLED folks have split
// it into three pieces to make it a lil more "scaleable" as discussed below
//
// Lets say the base clock of the esp is B (like 160Mhz). So in the spec, lets
// say t1/t2/t3 ns is T1/T2/T3 ticks of the base clock. Remember that a zero or
// one of the LED needs t1+t2+t3 nsecs == T1+T2+T3 ticks of the base clock. So
// we need to send T1+T2+T3 bits on the I2S data pin to make it look like ONE
// bit for the LED. So obviously, a large T1+T2+T2 means that we need to store
// too many bits in dma memory, and also too many interrupts to handle once the
// dma is succesful (to load the next set of bits etc..). So the goal is how can
// we reduce the number of clock bits needed to represent one LED bit. And we can
// do that by scaling down the clock. ESP provides a setting by which the base
// clock can be scaled down as B/(N + b/a) where (N + b/a) is a value with three
// configurable parameters N, b and a.
//
// So for us the task is that we need to scale down T1, T2 and T3 into values
// say K1, K3 K3 where K1 = T1/(N + b/a) and K2 = T2/(N + b/a) and K3 = T3/(N + b/a)
// So each of the three pieces are scaled down, and hence the total sum is
// also scaled down. But now the challenge is in finding these values for N, b and a
//
// We are again copying what FastLED folks did. So their approach is that lets say
// we are willing to trade off on certain amount of "precision". For example,
// K1 times (N + b/a) might not be equal to T1, but rather T1 plus or minus some
// "precision/tolerance" number of clock ticks say p1. And similarly p2, and p3 for
// T2 and T3. And p1, p2 and p3 might all not be the same, i.e the difference
// when scaling back each of those three pieces might be different, but within the
// tolerance levels.
//
// So given that is the approach, we basically start to find a divisor for T1, T2, T3
// such that the remainder is within some "precision" we define. All the manufactures
// talk about the tolerance in their specs, so we can translate that to some number
// of base clock ticks. And we find the largest divisor that is within the precision
// tolerance
fn calibrate_clocks(spec: &LedSpec) -> (u32, u32, u16) {
    let t1 = (spec.t1_ns as u64 * BASE_CLOCK) / NANOSECS;
    let t2 = (spec.t2_ns as u64 * BASE_CLOCK) / NANOSECS;
    let t3 = (spec.t3_ns as u64 * BASE_CLOCK) / NANOSECS;

    let min = std::cmp::min(t3, std::cmp::min(t1, t2));
    let mut divisor: u64 = 1;
    for d in (1..=min).rev() {
        let mut stop = false;
        for p in 0..PRECISION_TICKS as u64 {
            if t1 % d <= p && t2 % d <= p && t3 % d <= p {
                divisor = d as u64;
                stop = true;
                break;
            }
        }
        if stop {
            break;
        }
    }

    (
        BASE_CLOCK as u32,
        (BASE_CLOCK / divisor) as u32,
        divisor as u16,
    )
}

unsafe fn configure_clocks(hal: &mut i2s_hal_context_t, spec: &LedSpec) {
    let (sclk, mclk, mclk_div) = calibrate_clocks(spec);
    let (mclk_div1, b, a) = calculate_divisors(sclk, mclk, mclk_div as u32);
    info!("sclk {sclk}, mclk {mclk}, mclk_div {mclk_div}, mclk_div1 {mclk_div1}, b {b}, a {a}");
    let cfg = i2s_hal_clock_info_t {
        sclk,
        mclk,
        mclk_div,
        bclk: mclk,
        bclk_div: 1,
    };
    i2s_hal_set_tx_clock(hal, &cfg, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock() {
        let spec = LedSpec {
            t1_ns: 250,
            t2_ns: 625,
            t3_ns: 375,
        };
        let (sclk, mclk, mclk_div) = calibrate_clocks(&spec);
        let (mclk_div1, b, a) = calculate_divisors(sclk, mclk, mclk_div as u32);
        println!(
            "sclk {sclk}, mclk {mclk}, mclk_div {mclk_div}, mclk_div1 {mclk_div1}, b {b}, a {a}"
        );
    }
}
