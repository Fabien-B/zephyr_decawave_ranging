/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "dw3000_spi.h"
#include "pdec_radio.h"


/* Application SPI port based on the pinned decadriver platform source.
 * Pack header + payload into RAM so nrfx can use one continuous DMA chunk.
 * All transfers belong to the radio owner; scratch buffers are not reentrant.
 */
#define COALESCED_SPI_SIZE 1028U
static uint8_t tx_scratch[COALESCED_SPI_SIZE] __aligned(4);
static uint8_t rx_scratch[COALESCED_SPI_SIZE] __aligned(4);

LOG_MODULE_DECLARE(dw3000);

#define TX_WAIT_RESP_NRF52840_DELAY 30

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, decawave_dw3000))

static const struct device* spi;
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST);
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;

int dw3000_spi_init(void)
{
	/* set common SPI config */
	for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++) {
		spi_cfgs[i].cs = cs_ctrl;
		spi_cfgs[i].operation = SPI_WORD_SET(8);
	}

	/* SPI clock speed: Slow 2MHz, Max clock speed from DTS
	 * We have to keep two different config structures due to the way the SPI
	 * driver works */
	spi_cfgs[0].frequency = 2000000;
	spi_cfgs[1].frequency = DT_PROP(DW_INST, spi_max_frequency);
	spi_cfg = &spi_cfgs[0];

	spi = DEVICE_DT_GET(DW_SPI);
	if (!device_is_ready(spi)) {
		LOG_ERR("DW3000 SPI binding failed");
		return -ENODEV;
	} else {
		LOG_INF("DW3000 SPI (max %dMHz)", spi_cfgs[1].frequency / 1000000);
	}

#if CONFIG_PM_DEVICE
	enum pm_device_state pstate;
	int rc = pm_device_state_get(spi, &pstate);
	if (rc) {
		LOG_ERR("PM state get %d", rc);
	}

	if (pstate != PM_DEVICE_STATE_ACTIVE) {
		rc = pm_device_action_run(spi, PM_DEVICE_ACTION_RESUME);
		if (rc) {
			LOG_ERR("PM resume %d", rc);
		}
	}
#endif

	// initialized correctly at boot but after fini we need to reconfigure
	return gpio_pin_configure_dt(&spi_cfg->cs.gpio, GPIO_OUTPUT_HIGH);
}

void dw3000_spi_speed_slow(void)
{
	spi_cfg = &spi_cfgs[0];
}

void dw3000_spi_speed_fast(void)
{
	spi_cfg = &spi_cfgs[1];
}

void dw3000_spi_fini(void)
{
	// TODO: I can't find a SPI uninit function in Zephyr
#if CONFIG_PM_DEVICE
	int rc = pm_device_action_run(spi, PM_DEVICE_ACTION_SUSPEND);
	if (rc) {
		LOG_ERR("PM FINI suspend %d", rc);
	}
#endif
	gpio_pin_configure_dt(&spi_cfg->cs.gpio, GPIO_DISCONNECTED);
}

static int32_t write_transfer(uint16_t header_len, const uint8_t *header,
                              uint16_t body_len, const uint8_t *body,
                              bool with_crc, uint8_t crc)
{
    pdec_radio_assert_owner();
    if ((header_len && header == NULL) || (body_len && body == NULL)) {
        return -EINVAL;
    }
    size_t total = (size_t)header_len + body_len + (with_crc ? 1U : 0U);
    if (total <= sizeof(tx_scratch)) {
        if (header_len) { memcpy(tx_scratch, header, header_len); }
        if (body_len) { memcpy(tx_scratch + header_len, body, body_len); }
        if (with_crc) { tx_scratch[total - 1] = crc; }
        const struct spi_buf buffer = {.buf = tx_scratch, .len = total};
        const struct spi_buf_set tx = {.buffers = &buffer, .count = 1};
        return spi_transceive(spi, spi_cfg, &tx, NULL);
    }

    /* Preserve large diagnostic/bulk transfers without large stack buffers or
     * allocation. These rare accesses retain the original scatter/gather path.
     */
    const struct spi_buf buffers[] = {
        {.buf = (void *)header, .len = header_len},
        {.buf = (void *)body, .len = body_len},
        {.buf = &crc, .len = 1},
    };
    const struct spi_buf_set tx = {.buffers = buffers, .count = with_crc ? 3 : 2};
    return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int32_t dw3000_spi_write_crc(uint16_t header_len, const uint8_t *header,
                            uint16_t body_len, const uint8_t *body, uint8_t crc)
{
    return write_transfer(header_len, header, body_len, body, true, crc);
}

int32_t dw3000_spi_write(uint16_t header_len, const uint8_t *header,
                        uint16_t body_len, const uint8_t *body)
{
    return write_transfer(header_len, header, body_len, body, false, 0);
}

int32_t dw3000_spi_read(uint16_t header_len, uint8_t *header,
                       uint16_t body_len, uint8_t *body)
{
    pdec_radio_assert_owner();
    if ((header_len && header == NULL) || (body_len && body == NULL)) {
        return -EINVAL;
    }
    size_t total = (size_t)header_len + body_len;
    int ret;
    if (total <= sizeof(tx_scratch)) {
        if (header_len) { memcpy(tx_scratch, header, header_len); }
        /* Same dummy MOSI value as the Nordic SPIM's overrun character. Equal
         * full-duplex buffer lengths avoid a new DMA chunk at the header end.
         */
        memset(tx_scratch + header_len, DT_PROP_OR(DW_SPI, overrun_character, 0xff), body_len);
        const struct spi_buf tx_buffer = {.buf = tx_scratch, .len = total};
        const struct spi_buf rx_buffer = {.buf = rx_scratch, .len = total};
        const struct spi_buf_set tx = {.buffers = &tx_buffer, .count = 1};
        const struct spi_buf_set rx = {.buffers = &rx_buffer, .count = 1};
        ret = spi_transceive(spi, spi_cfg, &tx, &rx);
        if (ret == 0 && body_len) {
            memcpy(body, rx_scratch + header_len, body_len);
        }
    } else {
        const struct spi_buf tx_buffer = {.buf = header, .len = header_len};
        const struct spi_buf rx_buffers[] = {
            {.buf = NULL, .len = header_len},
            {.buf = body, .len = body_len},
        };
        const struct spi_buf_set tx = {.buffers = &tx_buffer, .count = 1};
        const struct spi_buf_set rx = {.buffers = rx_buffers, .count = 2};
        ret = spi_transceive(spi, spi_cfg, &tx, &rx);
    }

#if (CONFIG_SOC_NRF52840_QIAA)
    if (ret == 0) {
        for (volatile int i = 0; i < TX_WAIT_RESP_NRF52840_DELAY; i++) { }
    }
#endif
    return ret;
}

void dw3000_spi_wakeup()
{
	/* CS pin should be configured as active low
	 * To wake up, we set CS to 1, which will pull it low, for 500us */
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);
}
