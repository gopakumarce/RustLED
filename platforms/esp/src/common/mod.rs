use esp_idf_sys::GPIO_MODE_DEF_OUTPUT;
use esp_idf_sys::{
    esp_intr_alloc, esp_intr_enable, esp_rom_gpio_connect_out_signal, gpio_ll_iomux_func_sel,
    gpio_set_direction, heap_caps_malloc, i2s_hal_clock_info_t, i2s_hal_context_t,
    i2s_hal_set_tx_clock, i2s_ll_rx_reset, i2s_ll_rx_reset_dma, i2s_ll_rx_reset_fifo,
    i2s_ll_set_out_link_addr, i2s_ll_start_out_link, i2s_ll_tx_enable_intr, i2s_ll_tx_reset,
    i2s_ll_tx_reset_dma, i2s_ll_tx_reset_fifo, i2s_ll_tx_start, intr_handle_t, lcd_periph_signals,
    lldesc_t, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT, ESP_OK, GPIO_PIN_MUX_REG,
    I2S_LL_MCLK_DIVIDER_MAX, MALLOC_CAP_DMA, PIN_FUNC_GPIO,
};
use log::{error, info};
use rustled_common::NUM_COLORS;
use rustled_platforms_common::CalibrationResult;
use rustled_platforms_common::{calculate_divisors, calibrate_clocks, LedSpec};
use std::ptr;
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
    panic!("GOPA FINALLY");
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
        //i2s_ll_enable_intr(hal.dev, I2S_LL_EVENT_TX_EOF, false);

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

        println!("GOPA 1");

        let mut i2s = Self {
            hal,
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

        println!("GOPA reset");

        let data_pin = 4;
        let i2s_pin = lcd_periph_signals.buses[device as usize].data_sigs[0] as u32;
        gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[data_pin as usize], PIN_FUNC_GPIO);
        gpio_set_direction(data_pin as i32, GPIO_MODE_DEF_OUTPUT);
        esp_rom_gpio_connect_out_signal(data_pin, i2s_pin, false, false);

        println!("GOPA start");

        i2s.start();

        println!("GOPA started");

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

    unsafe fn start(self: &mut LedI2s) {
        self.next_descriptor = 0;
        self.reset();
        self.platform.i2s_ll_burst_en(self.hal);
        //i2s_ll_set_out_link_addr(self.hal.dev, self.descriptors[0].descriptor as u32);
        i2s_ll_set_out_link_addr(self.hal.dev, !0 as u32);
        i2s_ll_start_out_link(self.hal.dev);
        //i2s_ll_clear_intr_status(self.hal.dev, I2S_LL_EVENT_TX_EOF);
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

pub unsafe fn add_i2s_pin(_data_pin: u32) -> std::result::Result<(), &'static str> {
    Ok(())
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
    //i2s_hal_set_tx_clock(hal, &cfg, soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT);

    result
}

pub unsafe fn show_pixels(bytes: Vec<Vec<u8>>) -> (bool, Vec<Vec<u8>>) {
    (true, bytes)
}
