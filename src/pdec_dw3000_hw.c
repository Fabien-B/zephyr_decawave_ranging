/* Application hardware port, based on the pinned decadriver platform/dw3000_hw.c.
 * Keep reset, wakeup and SPI setup here; IRQ processing belongs to pdec_radio.c.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pdec_radio.h"
#include <zephyr/sys/atomic.h>
#include "dw3000_hw.h"
#include "dw3000_spi.h"

LOG_MODULE_REGISTER(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)

static struct gpio_callback gpio_cb;
static bool irq_initialized;
static atomic_t irq_enabled;

struct dw3000_config {
    struct gpio_dt_spec gpio_irq;
    struct gpio_dt_spec gpio_reset;
    struct gpio_dt_spec gpio_wakeup;
    struct gpio_dt_spec gpio_spi_pol;
    struct gpio_dt_spec gpio_spi_pha;
};

static const struct dw3000_config conf = {
    .gpio_irq = GPIO_DT_SPEC_GET_OR(DW_INST, irq_gpios, {0}),
    .gpio_reset = GPIO_DT_SPEC_GET_OR(DW_INST, reset_gpios, {0}),
    .gpio_wakeup = GPIO_DT_SPEC_GET_OR(DW_INST, wakeup_gpios, {0}),
    .gpio_spi_pol = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pol_gpios, {0}),
    .gpio_spi_pha = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pha_gpios, {0}),
};

int dw3000_hw_init(void)
{
    /* Reset */
    if (conf.gpio_reset.port) {
        gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
        LOG_INF("RESET on %s pin %d", conf.gpio_reset.port->name,
                conf.gpio_reset.pin);
    }

    /* Wakeup (optional) */
    if (conf.gpio_wakeup.port) {
        gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_OUTPUT_ACTIVE);
        LOG_INF("WAKEUP on %s pin %d", conf.gpio_wakeup.port->name,
                conf.gpio_wakeup.pin);
    }

    /* SPI Polarity (optional) */
    if (conf.gpio_spi_pol.port) {
        gpio_pin_configure_dt(&conf.gpio_spi_pol, GPIO_OUTPUT_INACTIVE);
        LOG_INF("SPI_POL on %s pin %d", conf.gpio_spi_pol.port->name,
                conf.gpio_spi_pol.pin);
    }

    /* SPI Phase (optional) */
    if (conf.gpio_spi_pha.port) {
        gpio_pin_configure_dt(&conf.gpio_spi_pha, GPIO_OUTPUT_INACTIVE);
        LOG_INF("SPI_PHA on %s pin %d", conf.gpio_spi_pha.port->name,
                conf.gpio_spi_pha.pin);
    }

    return dw3000_spi_init();
}

bool pdec_dw3000_irq_active(void)
{
    return irq_initialized && gpio_pin_get_dt(&conf.gpio_irq) > 0;
}

static void dw3000_hw_isr(const struct device *dev, struct gpio_callback *cb,
                          uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    pdec_radio_irq_notify();
}

int dw3000_hw_init_interrupt(void)
{
    if (!gpio_is_ready_dt(&conf.gpio_irq)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&conf.gpio_irq, GPIO_INPUT);
    if (ret != 0) {
        return ret;
    }
    gpio_init_callback(&gpio_cb, dw3000_hw_isr, BIT(conf.gpio_irq.pin));
    ret = gpio_add_callback(conf.gpio_irq.port, &gpio_cb);
    if (ret != 0) {
        return ret;
    }
    irq_initialized = true;
    ret = gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_RISING);
    if (ret != 0) {
        irq_initialized = false;
        gpio_remove_callback(conf.gpio_irq.port, &gpio_cb);
        return ret;
    }
    atomic_set(&irq_enabled, 1);
    /* An edge could have occurred before interrupt registration. */
    if (pdec_dw3000_irq_active()) {
        pdec_radio_irq_notify();
    }
    return 0;
}

void dw3000_hw_interrupt_enable(void)
{
    if (!irq_initialized) {
        return;
    }
    if (gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_RISING) == 0) {
        atomic_set(&irq_enabled, 1);
        /* forcetrxoff() masks GPIO IRQs: do not lose an edge while masked. */
        if (pdec_dw3000_irq_active()) {
            pdec_radio_irq_notify();
        }
    }
}

void dw3000_hw_interrupt_disable(void)
{
    if (irq_initialized &&
        gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE) == 0) {
        atomic_clear(&irq_enabled);
    }
}

bool dw3000_hw_interrupt_is_enabled(void)
{
    return atomic_get(&irq_enabled) != 0;
}

void dw3000_hw_fini(void)
{
    if (irq_initialized) {
        dw3000_hw_interrupt_disable();
        gpio_remove_callback(conf.gpio_irq.port, &gpio_cb);
        irq_initialized = false;
        gpio_pin_configure_dt(&conf.gpio_irq, GPIO_DISCONNECTED);
    }
    if (conf.gpio_reset.port) {
        gpio_pin_configure_dt(&conf.gpio_reset, GPIO_DISCONNECTED);
    }
    if (conf.gpio_wakeup.port) {
        gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_DISCONNECTED);
    }

    dw3000_spi_fini();
}

void dw3000_hw_reset()
{
    if (!conf.gpio_reset.port) {
        LOG_ERR("No HW reset configured");
        return;
    }

    gpio_pin_configure_dt(&conf.gpio_reset, GPIO_OUTPUT_ACTIVE);
    k_msleep(1); // 10 us?
    gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
    k_msleep(2);
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
    if (conf.gpio_wakeup.port) {
        /* Use WAKEUP pin if available */
        LOG_INF("WAKEUP PIN");
        gpio_pin_set_dt(&conf.gpio_wakeup, 1);
        k_msleep(1);
        gpio_pin_set_dt(&conf.gpio_wakeup, 0);

    } else {
        /* Use SPI CS pin */
        LOG_INF("WAKEUP CS");
        dw3000_spi_wakeup();
    }
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
    if (conf.gpio_wakeup.port) {
        gpio_pin_set_dt(&conf.gpio_wakeup, 0);
    }
}
