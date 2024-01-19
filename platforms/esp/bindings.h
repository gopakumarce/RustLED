#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"
#include "hal/gpio_ll.h"
#include "hal/gpio_types.h"
#include "soc/lcd_periph.h"
#include "soc/i2s_reg.h"
#include "esp_intr_alloc.h"
#include "esp_private/esp_clk.h"
#include "esp_clk_tree.h"

#ifndef __BINDINGS_H__
#define __BINDINGS_H__
static inline void gopa_init(i2s_dev_t *i2s)
{
    // -- Main configuration
    // i2s->conf.tx_msb_right = 1;
    // i2s->conf.tx_mono = 0;
    // i2s->conf.tx_short_sync = 0;
    // i2s->conf.tx_msb_shift = 0;
    // i2s->conf.tx_right_first = 1; // 0;//1;
    // i2s->conf.tx_slave_mod = 0;

    // -- Set parallel mode
    // i2s->conf2.val = 0;
    // i2s->conf2.lcd_en = 1;
    // i2s->conf2.lcd_tx_wrx2_en = 0; // 0 for 16 or 32 parallel output
    // i2s->conf2.lcd_tx_sdx2_en = 0; // HN

    // -- Set up the clock rate and sampling
    // i2s->sample_rate_conf.val = 0;
    // i2s->sample_rate_conf.tx_bits_mod = 32; // Number of parallel bits/pins
    // i2s->sample_rate_conf.tx_bck_div_num = 1;
    // i2s->clkm_conf.val = 0;
    // i2s->clkm_conf.clka_en = 0;

    // -- Data clock is computed as Base/(div_num + (div_b/div_a))
    //    Base is 80Mhz, so 80/(10 + 0/1) = 8Mhz
    //    One cycle is 125ns
    // i2s->clkm_conf.clkm_div_a = 10;
    // i2s->clkm_conf.clkm_div_b = 0;
    // i2s->clkm_conf.clkm_div_num = 1;

    i2s->fifo_conf.val = 0;
    i2s->fifo_conf.tx_fifo_mod_force_en = 1;
    i2s->fifo_conf.tx_fifo_mod = 3;  // 32-bit single channel data
    i2s->fifo_conf.tx_data_num = 32; // fifo length
    i2s->fifo_conf.dscr_en = 1;      // fifo will use dma

    // i2s->conf1.val = 0;
    // i2s->conf1.tx_stop_en = 0;
    // i2s->conf1.tx_pcm_bypass = 1;

    // i2s->conf_chan.val = 0;
    // i2s->conf_chan.tx_chan_mod = 1; // Mono mode, with tx_msb_right = 1, everything goes to right-channel

    // i2s->timing.val = 0;
}

static inline void i2sReset(i2s_dev_t *i2s)
{
    // Serial.println("I2S reset");
    const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    i2s->lc_conf.val |= lc_conf_reset_flags;
    i2s->lc_conf.val &= ~lc_conf_reset_flags;

    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    i2s->conf.val |= conf_reset_flags;
    i2s->conf.val &= ~conf_reset_flags;
}

static inline void i2sReset_DMA(i2s_dev_t *i2s)
{
    i2s->lc_conf.in_rst = 1;
    i2s->lc_conf.in_rst = 0;
    i2s->lc_conf.out_rst = 1;
    i2s->lc_conf.out_rst = 0;
}

static inline void i2sReset_FIFO(i2s_dev_t *i2s)
{
    i2s->conf.rx_fifo_reset = 1;
    i2s->conf.rx_fifo_reset = 0;
    i2s->conf.tx_fifo_reset = 1;
    i2s->conf.tx_fifo_reset = 0;
}
#endif