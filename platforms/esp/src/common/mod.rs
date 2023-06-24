use anyhow::Result;
use embedded_svc::mqtt::client::{Connection, MessageImpl, QoS};
use embedded_svc::utils::mqtt::client::ConnState;
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_hal::peripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use esp_idf_sys::{
    esp_intr_alloc_intrstatus, heap_caps_malloc, i2s_hal_context_t, i2s_ll_enable_intr,
    i2s_ll_get_intr_status_reg, intr_handle_t, lcd_periph_signals, lldesc_t, EspError,
    ESP_INTR_FLAG_INTRDISABLED, ESP_INTR_FLAG_LOWMED, ESP_INTR_FLAG_SHARED, ESP_OK, MALLOC_CAP_DMA,
};
use log::{error, info};
use rustled_platforms_common::CalibrationResult;
use std::{ptr, thread};

const SSID: &str = env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
const PASSWORD: &str = env!("RUST_ESP32_STD_DEMO_WIFI_PASS");
pub const I2S_LL_EVENT_TX_EOF: u32 = 1 << 12;
pub const PRECISION_TICKS: usize = 10;
pub const NUM_COLORS: usize = 3;
pub const MAX_CHANNELS: usize = 32;
// If we make a word from all bit0s from all the channels, a word with bit1 of all channels etc..,
// how many such words will we have. Basically its the total number of bits of all channel's bytes
// organized as 32 bit words
pub const MAX_TRANSPOSED_WORDS: usize = (MAX_CHANNELS * 8) / 32;

unsafe fn allocate_dma_buffers(bufsize: usize) -> (*mut u8, *mut lldesc_t) {
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

    (buffer, descriptor)
}

pub struct LedI2s {
    pub hal: i2s_hal_context_t,
    next: usize,
    pub device: u32,
    pub intr_handle: intr_handle_t,
    pub dma_buffer0: *mut u8,
    pub dma_buffer1: *mut u8,
    pub cur_buffer: usize,
    pub descriptor0: *const lldesc_t,
    pub descriptor1: *const lldesc_t,
    pub ones_for_zero: usize,
    pub ones_for_one: usize,
    pub pulses_per_bit: usize,
    pub zero_bits: Vec<u32>,
    pub one_bits: Vec<u32>,
    pub pixel_row: Vec<u8>,
    pub pixel_bits: Vec<u8>,
    pub num_channels: usize,
}

unsafe extern "C" fn i2s_interrupt_handler(_arg: *mut ::core::ffi::c_void) {}

impl LedI2s {
    pub unsafe fn new(
        hal: i2s_hal_context_t,
        device: u32,
        clk_cfg: &CalibrationResult,
    ) -> std::result::Result<Self, &'static str> {
        let interrupt_source = lcd_periph_signals.buses[device as usize].irq_id;
        let mut intr_handle = ptr::null_mut();
        let isr_flags: u32 =
            ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
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

        let pulses_per_bit = clk_cfg.t1_pulses + clk_cfg.t2_pulses + clk_cfg.t3_pulses;
        let (dma_buffer0, descriptor0) =
            allocate_dma_buffers(NUM_COLORS * MAX_CHANNELS * pulses_per_bit);
        let (dma_buffer1, descriptor1) =
            allocate_dma_buffers(NUM_COLORS * MAX_CHANNELS * pulses_per_bit);
        (*descriptor0).__bindgen_anon_1.qe.stqe_next = descriptor1;
        (*descriptor1).__bindgen_anon_1.qe.stqe_next = descriptor0;

        let ones_for_one = clk_cfg.t1_pulses + clk_cfg.t2_pulses;
        let ones_for_zero = clk_cfg.t1_pulses;
        let mut one_bits = vec![0; pulses_per_bit];
        let mut zero_bits = vec![0; pulses_per_bit];
        for i in 0..ones_for_one {
            one_bits[i] = 0xFFFFFF00;
        }
        for i in ones_for_one..pulses_per_bit {
            one_bits[i] = 0x00000000;
        }
        for i in 0..ones_for_zero {
            zero_bits[i] = 0xFFFFFF00;
        }
        for i in ones_for_zero..pulses_per_bit {
            zero_bits[i] = 0x00000000;
        }

        let pixel_row = vec![0; NUM_COLORS * MAX_CHANNELS];
        let pixel_bits = vec![0; NUM_COLORS * MAX_CHANNELS];
        let num_channels = lcd_periph_signals.buses[device as usize].data_sigs.len();

        Ok(Self {
            hal,
            next: 0,
            device,
            intr_handle,
            dma_buffer0,
            dma_buffer1,
            cur_buffer: 0,
            descriptor0,
            descriptor1,
            ones_for_zero,
            ones_for_one,
            pulses_per_bit,
            zero_bits,
            one_bits,
            pixel_row,
            pixel_bits,
            num_channels,
        })
    }

    pub unsafe fn next_pin(&mut self) -> Option<u32> {
        let device = self.device as usize;
        if self.next >= self.num_channels {
            return None;
        }
        let ret = Some(lcd_periph_signals.buses[device].data_sigs[self.next] as u32);
        self.next += 1;
        ret
    }

    pub fn num_pins(&self) -> usize {
        self.next
    }
}

pub fn start_mqtt() -> Result<EspMqttClient<ConnState<MessageImpl, EspError>>> {
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
