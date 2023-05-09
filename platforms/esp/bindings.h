#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
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

#ifdef CONFIG_IDF_TARGET_ESP32

// These xtos_foo apis are defined in the esp32.rom.ld files in esp-idf,
// but the esp-idf rust bindings doesnt process them. Hence we are having
// to manually define them here, similar will be the case for anything
// in the rom.ld files which we might need in future
unsigned int (*_xtos_ints_on_ptr)(unsigned int mask) = (unsigned int (*)(unsigned int))0x4000bf88;
unsigned int (*_xtos_ints_off_ptr)(unsigned int mask) = (unsigned int (*)(unsigned int))0x4000bfac;

unsigned int _xtos_ints_on(unsigned int mask)
{
    return _xtos_ints_on_ptr(mask);
}

unsigned int _xtos_ints_off(unsigned int mask)
{
    return _xtos_ints_off_ptr(mask);
}

void *xt_get_interrupt_handler_arg(int n)
{
    return 0;
}

static inline void i2s_fifo_init(i2s_dev_t *i2s)
{
    i2s->fifo_conf.val = 0;
    i2s->fifo_conf.tx_fifo_mod_force_en = 1;
    i2s->fifo_conf.tx_fifo_mod = 3;  // 32-bit single channel data
    i2s->fifo_conf.tx_data_num = 32; // fifo length
    i2s->fifo_conf.dscr_en = 1;      // fifo will use dma
}

#endif /* CONFIG_IDF_TARGET_ESP32 */

#endif /* __BINDINGS_H__ */
