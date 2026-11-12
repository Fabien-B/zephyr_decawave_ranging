/* SPDX-License-Identifier: Apache-2.0 */
/* Dedicated nRF52833 SPIM3 transport. The radio owner is the sole caller.
 * Keep interrupts enabled; poll DMA completion instead of sleeping/waking for
 * each tiny register access. This is deliberately not a shared SPI controller.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <hal/nrf_spim.h>
#include "pdec_radio.h"

#define SPI_NODE DT_INST(0, pdec_nrf52833_spim3)
#define MAX_DMA BIT_MASK(DT_PROP(SPI_NODE, easydma_maxcnt_bits))
#define BOUNCE_SIZE 256U
#define TIMEOUT_MARGIN_US 1000U

BUILD_ASSERT(DT_REG_ADDR(SPI_NODE) == NRF_SPIM3_BASE, "Polling port requires SPIM3");
#if defined(CONFIG_SOC_NRF52833)
BUILD_ASSERT(IS_ENABLED(CONFIG_NRFX_SPIM), "Nordic pinctrl requires SPIM HAL support");
#endif
PINCTRL_DT_DEFINE(SPI_NODE);
static NRF_SPIM_Type *const regs = (NRF_SPIM_Type *)DT_REG_ADDR(SPI_NODE);
static uint8_t bounce[BOUNCE_SIZE] __aligned(4);
static uint32_t frequency;

static bool dma_ram(const void *buffer, size_t len)
{
    uintptr_t address = (uintptr_t)buffer;
    if (len && address > UINTPTR_MAX - (len - 1)) { return false; }
    return nrf_dma_accessible_check(regs, buffer) &&
        (!len || nrf_dma_accessible_check(regs, (const uint8_t *)buffer + len - 1));
}

static int set_frequency(uint32_t hz)
{
    nrf_spim_frequency_t setting;
    switch (hz) {
    case 125000: setting = NRF_SPIM_FREQ_125K; break;
    case 250000: setting = NRF_SPIM_FREQ_250K; break;
    case 500000: setting = NRF_SPIM_FREQ_500K; break;
    case 1000000: setting = NRF_SPIM_FREQ_1M; break;
    case 2000000: setting = NRF_SPIM_FREQ_2M; break;
    case 4000000: setting = NRF_SPIM_FREQ_4M; break;
    case 8000000: setting = NRF_SPIM_FREQ_8M; break;
    case 16000000: setting = NRF_SPIM_FREQ_16M; break;
    case 32000000: setting = NRF_SPIM_FREQ_32M; break;
    default: return -EINVAL;
    }
    if (hz > DT_PROP(SPI_NODE, max_frequency)) { return -EINVAL; }
    if (frequency != hz) {
        nrf_spim_disable(regs);
        nrf_spim_frequency_set(regs, setting);
        frequency = hz;
    }
    nrf_spim_enable(regs);
    return 0;
}

static int dma_transfer(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len)
{
    uint32_t wire_us = DIV_ROUND_UP(MAX(tx_len, rx_len) * 8000000ULL, frequency);
    uint32_t budget = k_us_to_cyc_ceil32(wire_us + TIMEOUT_MARGIN_US);
    nrf_spim_event_clear(regs, NRF_SPIM_EVENT_END);
    nrf_spim_event_clear(regs, NRF_SPIM_EVENT_STOPPED);
    nrf_spim_tx_buffer_set(regs, tx, tx_len);
    nrf_spim_rx_buffer_set(regs, rx, rx_len);
    uint32_t start = k_cycle_get_32();
    nrf_spim_task_trigger(regs, NRF_SPIM_TASK_START);
    while (!nrf_spim_event_check(regs, NRF_SPIM_EVENT_END)) {
        if ((uint32_t)(k_cycle_get_32() - start) >= budget) {
            nrf_spim_task_trigger(regs, NRF_SPIM_TASK_STOP);
            start = k_cycle_get_32();
            while (!nrf_spim_event_check(regs, NRF_SPIM_EVENT_STOPPED) &&
                   (uint32_t)(k_cycle_get_32() - start) < k_us_to_cyc_ceil32(TIMEOUT_MARGIN_US)) { }
            /* Disable even if STOPPED did not arrive; don't leave DMA running
             * against the caller's buffers after an error return.
             */
            nrf_spim_disable(regs);
            return -ETIMEDOUT;
        }
    }
    /* Ignore counters for disabled DMA directions; their old value is immaterial. */
    if ((tx_len && nrf_spim_tx_amount_get(regs) != tx_len) ||
        (rx_len && nrf_spim_rx_amount_get(regs) != rx_len)) {
        return -EIO;
    }
    return 0;
}

struct cursor {
    const struct spi_buf_set *set;
    size_t index, offset;
};

static size_t current(struct cursor *cursor, uint8_t **data)
{
    while (cursor->set && cursor->index < cursor->set->count) {
        const struct spi_buf *buffer = &cursor->set->buffers[cursor->index];
        if (cursor->offset < buffer->len) {
            *data = buffer->buf ? (uint8_t *)buffer->buf + cursor->offset : NULL;
            return buffer->len - cursor->offset;
        }
        cursor->index++;
        cursor->offset = 0;
    }
    *data = NULL;
    return 0;
}

static int polling_transceive(const struct device *dev, const struct spi_config *config,
                              const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
    ARG_UNUSED(dev);
    pdec_radio_assert_owner();
    if (config->operation != SPI_WORD_SET(8) || !spi_cs_is_gpio(config)) {
        return -ENOTSUP;
    }
    int ret = set_frequency(config->frequency);
    if (ret) { return ret; }
    ret = gpio_pin_set_dt(&config->cs.gpio, 1);
    if (ret) { return ret; }
    if (config->cs.delay) { k_busy_wait(config->cs.delay); }
    struct cursor out = {.set = tx}, in = {.set = rx};
    while (true) {
        uint8_t *out_data, *in_data;
        size_t out_len = current(&out, &out_data), in_len = current(&in, &in_data);
        if (!out_len && !in_len) { break; }
        size_t count = MIN(out_len && in_len ? MIN(out_len, in_len) : MAX(out_len, in_len), MAX_DMA);
        size_t write_len = out_len ? count : 0, read_len = in_data ? count : 0;
        if (in_data && !dma_ram(in_data, read_len)) { ret = -EINVAL; break; }
        if (out_data && !dma_ram(out_data, write_len)) {
            count = MIN(count, sizeof(bounce));
            write_len = count;
            read_len = in_data ? count : 0;
            memcpy(bounce, out_data, count);
            out_data = bounce;
        } else if (!out_data) {
            /* NULL TX means clock the overrun character. If RX is also
             * discarded, TX DMA must still generate those clock cycles.
             */
            write_len = 0;
            if (!in_data) {
                count = MIN(count, sizeof(bounce));
                memset(bounce, DT_PROP(SPI_NODE, overrun_character), count);
                out_data = bounce;
                write_len = count;
            }
        }
        ret = dma_transfer(out_data, write_len, in_data, read_len);
        if (ret) { break; }
        if (out_len) { out.offset += count; }
        if (in_len) { in.offset += count; }
    }
    if (config->cs.delay) { k_busy_wait(config->cs.delay); }
    int cs_ret = gpio_pin_set_dt(&config->cs.gpio, 0);
    return ret ? ret : cs_ret;
}

static int polling_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    nrf_spim_disable(regs);
    nrf_spim_int_disable(regs, UINT32_MAX);
    int ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPI_NODE), PINCTRL_STATE_DEFAULT);
    if (ret) { return ret; }
    nrf_spim_configure(regs, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
    nrf_spim_orc_set(regs, DT_PROP(SPI_NODE, overrun_character));
    nrf_spim_iftiming_set(regs, DT_PROP(SPI_NODE, rx_delay));
    frequency = 0;
    return 0;
}

static DEVICE_API(spi, polling_api) = {.transceive = polling_transceive};
DEVICE_DT_DEFINE(SPI_NODE, polling_init, NULL, NULL, NULL,
                 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &polling_api);
