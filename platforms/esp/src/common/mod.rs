use anyhow::Result;
use core::cell::RefCell;
use critical_section::Mutex;
use esp_idf_hal::peripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::mqtt::client::ConnState;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::mqtt::client::{MessageImpl, QoS};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use esp_idf_sys::GPIO_MODE_DEF_OUTPUT;
use esp_idf_sys::{
    esp_intr_alloc, esp_intr_disable, esp_intr_enable, heap_caps_malloc, i2s_hal_context_t,
    i2s_ll_clear_intr_status, i2s_ll_enable_intr, i2s_ll_rx_reset, i2s_ll_rx_reset_dma,
    i2s_ll_rx_reset_fifo, i2s_ll_rx_stop, i2s_ll_set_out_link_addr, i2s_ll_start_out_link,
    i2s_ll_tx_enable_intr, i2s_ll_tx_reset, i2s_ll_tx_reset_dma, i2s_ll_tx_reset_fifo,
    i2s_ll_tx_start, i2s_ll_tx_stop, intr_handle_t, lcd_periph_signals, lldesc_t,
    xQueueGenericCreate, xQueueGenericSend, xQueueGiveFromISR, xQueueSemaphoreTake, EspError,
    QueueHandle_t, TickType_t, UBaseType_t, _frxt_setup_switch, esp_rom_gpio_connect_out_signal,
    gpio_ll_iomux_func_sel, gpio_set_direction, i2s_hal_clock_info_t, i2s_hal_set_tx_clock,
    soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT, ESP_OK, GPIO_PIN_MUX_REG,
    I2S_LL_MCLK_DIVIDER_MAX, MALLOC_CAP_DMA, PIN_FUNC_GPIO,
};
use log::{error, info};
use rustled_common::NUM_COLORS;
use rustled_platforms_common::{calculate_divisors, calibrate_clocks, LedSpec};
use rustled_platforms_common::{transpose32, CalibrationResult};
use std::{ptr, thread};
const SSID: &str = env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
const PASSWORD: &str = env!("RUST_ESP32_STD_DEMO_WIFI_PASS");
pub const I2S_LL_EVENT_TX_EOF: u32 = 1 << 12;
pub const PRECISION_TICKS: usize = 10;
// 32 channels one byte each (one byte color). Actually all espX platforms only have
// 24 channels as of today, but the max is 32 so that we have a "32 bit word" where
// we can transpose the color's bits into etc.. (see transpose32())
pub const MAX_CHANNEL_BYTES: usize = 32;
// If we make a word from all bit0s from all the channels, a word with bit1 of all channels etc..,
// how many such words will we have. Basically its the total number of bits of all channel's bytes
// organized as 32 bit words
pub const MAX_CHANNEL_32BIT_WORDS: usize = (MAX_CHANNEL_BYTES * 8) / 32;
// TODO: This should be replaced by esp_clk_tree_src_get_freq_hz whenever the
// latest esp-idf is released
const ESP32_I2S_DEF_PLL_FREQ: u64 = 160 * 1_000_000;
const BASE_CLOCK: u64 = ESP32_I2S_DEF_PLL_FREQ;

static LEDI2S: Mutex<RefCell<Option<LedI2s>>> = Mutex::new(RefCell::new(None));
static mut SEMAPHORE: QueueHandle_t = std::ptr::null_mut();

pub trait LedI2sPlatform {
    unsafe fn i2s_ll_ahbm_reset(&self, _hal: i2s_hal_context_t) {}
    unsafe fn i2s_ll_burst_en(&self, _hal: i2s_hal_context_t) {}
    unsafe fn i2s_ll_set_out_dscr(&self, _hal: i2s_hal_context_t) {}
}

pub struct DmaDescriptor {
    pub buffer: *mut u8,
    pub descriptor: *const lldesc_t,
}

unsafe impl Send for LedI2s {}

pub struct LedI2s {
    pub hal: i2s_hal_context_t,
    next: usize,
    pub device: u32,
    pub intr_handle: intr_handle_t,
    pub next_descriptor: usize,
    pub descriptors: Vec<DmaDescriptor>,
    pub ones_for_zero: usize,
    pub ones_for_one: usize,
    pub pulses_per_bit: usize,
    pub num_channels: usize,
    pub color_blocks: usize,
    pub cur_block: usize,
    pub color_bytes: Vec<Vec<u8>>,
    pub platform: Box<dyn LedI2sPlatform>,
}

unsafe extern "C" fn i2s_interrupt_handler(_arg: *mut ::core::ffi::c_void) {
    critical_section::with(|cs| {
        LEDI2S.borrow(cs).borrow_mut().as_mut().map(|i2s| {
            if (*i2s.hal.dev).int_st.__bindgen_anon_1.out_eof() != 0 {
                (*i2s.hal.dev).int_clr.val = 0;
                if !i2s.fill_buffer() {
                    give_semaphore_from_isr(SEMAPHORE);
                }
            }
        });
    });
}

impl LedI2s {
    pub unsafe fn new(
        mut hal: i2s_hal_context_t,
        device: u32,
        spec: LedSpec,
        platform: Box<dyn LedI2sPlatform>,
    ) -> std::result::Result<(), &'static str> {
        let interrupt_source = lcd_periph_signals.buses[device as usize].irq_id;
        let mut intr_handle = ptr::null_mut();
        let ret = esp_intr_alloc(
            interrupt_source,
            0,
            Some(i2s_interrupt_handler),
            std::ptr::null_mut(),
            &mut intr_handle,
        );
        if ret != ESP_OK {
            error!("i2s interrupt alloc failed {ret}");
            return Err("i2s interrupt alloc failed");
        }
        // Keep interrupts off for now
        i2s_ll_enable_intr(hal.dev, I2S_LL_EVENT_TX_EOF, false);

        let clk_cfg = configure_clocks(&mut hal, &spec);
        let pulses_per_bit = clk_cfg.t1_pulses + clk_cfg.t2_pulses + clk_cfg.t3_pulses;
        let ones_for_one = clk_cfg.t1_pulses + clk_cfg.t2_pulses;
        let ones_for_zero = clk_cfg.t1_pulses;
        let num_channels = lcd_periph_signals.buses[device as usize].data_sigs.len();
        let descriptors = Self::allocate_dma_descriptors(2, pulses_per_bit);
        // Various parts of the code assume that buffers holding the data can be cast into
        // array of u32 - see the usage of MAX_CHANNEL_32BIT_WORDS as an example.
        assert!(MAX_CHANNEL_BYTES >= 32 && MAX_CHANNEL_BYTES % 32 == 0);
        // Just so that a new platform doesnt cause surprises
        assert!(num_channels <= MAX_CHANNEL_BYTES);

        SEMAPHORE = create_binary_semaphore();
        give_semaphore(SEMAPHORE);

        let mut i2s = Self {
            hal,
            next: 0,
            device,
            intr_handle,
            next_descriptor: 0,
            descriptors,
            ones_for_zero,
            ones_for_one,
            pulses_per_bit,
            num_channels,
            color_blocks: 0,
            cur_block: 0,
            color_bytes: vec![],
            platform,
        };
        i2s.reset();
        critical_section::with(|cs| LEDI2S.borrow_ref_mut(cs).replace(i2s));
        Ok(())
    }

    unsafe fn allocate_dma_descriptors(
        ndescriptors: usize,
        pulses_per_bit: usize,
    ) -> Vec<DmaDescriptor> {
        let mut first: *mut lldesc_t = ptr::null_mut();
        let mut last: *mut lldesc_t = ptr::null_mut();
        let mut descriptors = vec![];

        for _ in 0..ndescriptors {
            let (buffer, descriptor) =
                Self::allocate_dma_descriptor(NUM_COLORS * MAX_CHANNEL_BYTES * pulses_per_bit);
            if last == ptr::null_mut() {
                first = descriptor;
                last = descriptor;
            } else {
                (*last).__bindgen_anon_1.qe.stqe_next = descriptor;
                last = descriptor;
            }
            descriptors.push(DmaDescriptor { buffer, descriptor });
        }
        if last != ptr::null_mut() {
            (*last).__bindgen_anon_1.qe.stqe_next = first;
        }
        descriptors
    }

    unsafe fn allocate_dma_descriptor(bufsize: usize) -> (*mut u8, *mut lldesc_t) {
        let desc_size = std::mem::size_of::<lldesc_t>();
        let raw: *mut u8 = unsafe { heap_caps_malloc(desc_size, MALLOC_CAP_DMA) as *mut u8 };
        unsafe {
            ptr::write_bytes(raw, 0, desc_size);
        }
        let descriptor = raw as *mut lldesc_t;

        let buffer: *mut u8 = unsafe { heap_caps_malloc(bufsize, MALLOC_CAP_DMA) as *mut u8 };
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

        (buffer, descriptor)
    }

    unsafe fn next_pin(&mut self) -> Option<u32> {
        let device = self.device as usize;
        if self.next >= self.num_channels {
            return None;
        }
        let ret = Some(lcd_periph_signals.buses[device].data_sigs[self.next] as u32);
        self.next += 1;
        ret
    }

    fn num_pins(&self) -> usize {
        self.next
    }

    fn get_next_dma_buffer(&mut self) -> *mut u8 {
        if self.next_descriptor >= self.descriptors.len() {
            self.next_descriptor = 0;
        }
        let buffer = self.descriptors[self.next_descriptor].buffer;
        self.next_descriptor += 1;
        buffer
    }

    unsafe fn empty(&mut self, buf: *mut u32) {
        for i in 0..MAX_CHANNEL_32BIT_WORDS * NUM_COLORS {
            let offset = self.pulses_per_bit * i;
            for j in 0..self.ones_for_zero {
                *buf.add(offset + j) = 0xffffffff;
            }
            for j in self.ones_for_one..self.pulses_per_bit {
                *buf.add(offset + j) = 0;
            }
        }
    }

    unsafe fn fill_buffer(&mut self) -> bool {
        if self.cur_block >= self.color_blocks {
            return false;
        }
        let buf = self.get_next_dma_buffer() as *mut u32;
        self.empty(buf);

        let mut pixel_bytes = [0; MAX_CHANNEL_BYTES];
        let mut pixel_words = [0; MAX_CHANNEL_BYTES];
        let num_pins = self.num_pins();
        let d = self.cur_block;
        self.cur_block += 1;

        for color in 0..NUM_COLORS {
            let bytes = &self.color_bytes[color][d * num_pins..(d + 1) * num_pins];
            for i in 0..num_pins {
                // Store the colors from pins in reverse order of pins
                pixel_bytes[i] = bytes[num_pins - (i + 1)];
            }
            transpose32(&pixel_bytes, &mut pixel_words);
            for word in 0..MAX_CHANNEL_32BIT_WORDS {
                let offset = word * 4;
                // Think of pixel_words as a word which is in blocks of four bytes
                let byte0 = pixel_words[offset];
                let byte1 = pixel_words[offset + 1];
                let byte2 = pixel_words[offset + 2];
                let byte3 = pixel_words[offset + 3];
                // reverse the bits
                let word_bits = (u32::from(byte0) << 24)
                    | (u32::from(byte1) << 16)
                    | (u32::from(byte2) << 8)
                    | u32::from(byte3);
                for pulse in self.ones_for_zero..self.ones_for_one {
                    *buf.add(
                        color * MAX_CHANNEL_32BIT_WORDS * self.pulses_per_bit
                            + word * self.pulses_per_bit
                            + pulse,
                    ) = word_bits;
                }
            }
        }
        true
    }

    unsafe fn show_pixels_internal(self: &mut LedI2s, bytes: Vec<Vec<u8>>) -> (bool, Vec<Vec<u8>>) {
        if bytes.len() != NUM_COLORS {
            error!(
                "Expecting exactly {} colors, got {}",
                NUM_COLORS,
                bytes.len()
            );
            return (false, bytes);
        }

        let num_pins = self.num_pins();
        if num_pins == 0 {
            error!("No channels configured yet");
            return (false, bytes);
        }

        let mut color_blocks = 0;

        for c in 0..NUM_COLORS {
            let modulo = bytes[c].len() % num_pins;
            let blocks = bytes[c].len() / num_pins;
            if modulo != 0 {
                error!(
                    "Need color bytes for every configured channel {} {}, {}",
                    c,
                    bytes[c].len(),
                    num_pins
                );
                return (false, bytes);
            }
            if color_blocks == 0 {
                color_blocks = blocks
            }
            if blocks != color_blocks {
                error!(
                    "EVery color needs the same amount of data {}, {}, {} / {}",
                    c,
                    bytes[c].len(),
                    blocks,
                    color_blocks,
                );
                return (false, bytes);
            }
        }

        let _ = std::mem::replace(&mut self.color_bytes, bytes);
        self.cur_block = 0;
        self.color_blocks = color_blocks;

        (true, vec![])
    }

    unsafe fn stop(&mut self) {
        esp_intr_disable(self.intr_handle);
        self.reset();
        i2s_ll_rx_stop(self.hal.dev);
        i2s_ll_tx_stop(self.hal.dev);
        self.next_descriptor = 0;
    }

    unsafe fn start(self: &mut LedI2s) {
        self.next_descriptor = 0;
        self.reset();
        self.platform.i2s_ll_burst_en(self.hal);
        i2s_ll_set_out_link_addr(self.hal.dev, self.descriptors[0].descriptor as u32);
        i2s_ll_start_out_link(self.hal.dev);
        i2s_ll_clear_intr_status(self.hal.dev, I2S_LL_EVENT_TX_EOF);
        self.platform.i2s_ll_set_out_dscr(self.hal);

        esp_intr_enable(self.intr_handle);
        (*self.hal.dev).int_ena.val = 0;
        i2s_ll_tx_enable_intr(self.hal.dev);

        //start transmission
        i2s_ll_tx_start(self.hal.dev);
    }

    pub unsafe fn reset(&mut self) {
        i2s_ll_tx_reset(self.hal.dev);
        i2s_ll_tx_reset_dma(self.hal.dev);
        i2s_ll_tx_reset_fifo(self.hal.dev);
        i2s_ll_rx_reset(self.hal.dev);
        i2s_ll_rx_reset_dma(self.hal.dev);
        i2s_ll_rx_reset_fifo(self.hal.dev);
        self.platform.i2s_ll_ahbm_reset(self.hal);
    }
}

pub unsafe fn add_i2s_pin(data_pin: u32) -> std::result::Result<(), &'static str> {
    critical_section::with(|cs| {
        LEDI2S.borrow(cs).borrow_mut().as_mut().map(|i2s| {
            let Some(i2s_pin) = i2s.next_pin() else {
                return std::result::Result::Err("i2s lcd out of pins");
            };
            gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[data_pin as usize], PIN_FUNC_GPIO);
            gpio_set_direction(data_pin as i32, GPIO_MODE_DEF_OUTPUT);
            esp_rom_gpio_connect_out_signal(data_pin, i2s_pin, false, false);

            Ok(())
        })
    })
    .unwrap()
}

pub unsafe fn configure_clocks(hal: &mut i2s_hal_context_t, spec: &LedSpec) -> CalibrationResult {
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

pub unsafe fn show_pixels(bytes: Vec<Vec<u8>>) -> (bool, Vec<Vec<u8>>) {
    take_semaphore(SEMAPHORE);

    let (ret, buf) = critical_section::with(|cs| {
        LEDI2S
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .show_pixels_internal(bytes)
    });
    if !ret {
        give_semaphore(SEMAPHORE);
        return (ret, buf);
    }

    // Fill up all the dma descriptors
    critical_section::with(|cs| {
        LEDI2S.borrow(cs).borrow_mut().as_mut().map(|i2s| {
            for _ in 0..i2s.descriptors.len() {
                if !i2s.fill_buffer() {
                    break;
                }
            }
            i2s.start();
        });
    });

    take_semaphore(SEMAPHORE);

    let ret = critical_section::with(|cs| {
        LEDI2S.borrow(cs).borrow_mut().as_mut().map(|i2s| {
            i2s.stop();
            let buf = std::mem::replace(&mut i2s.color_bytes, vec![]);
            (true, buf)
        })
    })
    .unwrap();

    give_semaphore(SEMAPHORE);
    ret
}

// TODO: HACK ALERT: freertos code defines xSemaphoreCreateBinary as a #define.
// Who on earth defines APIs as #define !! Anyways, because of that, bindgen
// doesnt generate proper bindings and hence we are having to replicate equivalent
// of that #define here manually
pub unsafe fn create_binary_semaphore() -> QueueHandle_t {
    xQueueGenericCreate(1 as UBaseType_t, 0 as UBaseType_t, 3)
}

pub unsafe fn give_semaphore(semaphore: QueueHandle_t) {
    xQueueGenericSend(semaphore, 0 as *const core::ffi::c_void, 0, 0);
}

pub unsafe fn give_semaphore_from_isr(semaphore: QueueHandle_t) {
    let mut has_woken: i32 = 0;
    xQueueGiveFromISR(semaphore, &mut has_woken);
    if has_woken == 1 {
        _frxt_setup_switch();
    }
}

pub unsafe fn take_semaphore(semaphore: QueueHandle_t) {
    xQueueSemaphoreTake(semaphore, 0xffffffff as TickType_t);
}

pub fn start_mqtt() -> Result<EspMqttClient<'static, ConnState<MessageImpl, EspError>>> {
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

pub fn start_wifi(
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
