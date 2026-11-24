/**
 * @file spi_fake.c
 * @brief ws-hal-stm32f7 SPI — HOST FAKE (scripted RX, TX capture, busy/timeout/error seams).
 */
#include <string.h>

#include <ws/hal/stm32f7/spi.h>

#define WS_SPI_FAKE_BUF_CAP 256u

typedef struct {
    uint8_t rx[WS_SPI_FAKE_BUF_CAP];
    size_t rx_head;
    size_t rx_len;
    uint8_t tx[WS_SPI_FAKE_BUF_CAP];
    size_t tx_len;
    uint32_t actual_hz;
    uint32_t transfer_count;
    bool busy;
    bool timeout;
    bool bus_error;
} s_spi_inst_fake_t;

static s_spi_inst_fake_t s_spis[WS_SPI_INSTANCE_COUNT];

static ws_rcc_periph_t s_spi_rcc_periph(ws_spi_instance_t inst)
{
    return (inst == WS_SPI_1) ? WS_RCC_PERIPH_SPI1 : WS_RCC_PERIPH_SPI2;
}

static uint32_t s_spi_pclk_hz(ws_spi_instance_t inst)
{
    return (inst == WS_SPI_1) ? ws_rcc_apb2_hz() : ws_rcc_apb1_hz();
}

static bool s_pin_valid(const ws_gpio_t *pin)
{
    return ((uint32_t)pin->port <= (uint32_t)WS_GPIO_PORT_K) && (pin->pin < WS_GPIO_PIN_COUNT);
}

static bool s_derive_br(uint32_t pclk_hz, uint32_t max_hz, uint32_t *out_actual_hz)
{
    if (pclk_hz == 0u || max_hz == 0u) {
        return false;
    }
    for (uint32_t idx = 0u; idx < 8u; ++idx) {
        const uint32_t div = 2u << idx;
        const uint32_t sck_hz = pclk_hz / div;
        if (sck_hz <= max_hz) {
            *out_actual_hz = sck_hz;
            return true;
        }
    }
    return false;
}

ws_spi_error_t ws_spi_init(ws_spi_t *spi, const ws_spi_cfg_t *cfg)
{
    if (spi == NULL || cfg == NULL || cfg->max_hz == 0u || cfg->af > 15u ||
        (unsigned)cfg->instance >= WS_SPI_INSTANCE_COUNT ||
        (unsigned)cfg->mode > (unsigned)WS_SPI_MODE_3 || (!cfg->use_miso && !cfg->use_mosi) ||
        !s_pin_valid(&cfg->sck_pin) || (cfg->use_miso && !s_pin_valid(&cfg->miso_pin)) ||
        (cfg->use_mosi && !s_pin_valid(&cfg->mosi_pin))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid SPI handle or config", "ws_spi.init");
    }

    uint32_t actual_hz = 0u;
    if (!s_derive_br(s_spi_pclk_hz(cfg->instance), cfg->max_hz, &actual_hz)) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "underivable SPI prescaler for max_hz", "ws_spi.init");
    }

    const ws_gpio_cfg_t pin_cfg = {
        .mode = WS_GPIO_MODE_ALTERNATE,
        .pull = WS_GPIO_PULL_NONE,
        .speed = WS_GPIO_SPEED_HIGH,
        .af = cfg->af,
    };

    ws_gpio_t sck = cfg->sck_pin;
    if (WS_FAILED(ws_gpio_init(&sck, &pin_cfg))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "SPI SCK pin config failed", "ws_spi.init");
    }
    if (cfg->use_miso) {
        ws_gpio_t miso = cfg->miso_pin;
        if (WS_FAILED(ws_gpio_init(&miso, &pin_cfg))) {
            return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                          "SPI MISO pin config failed", "ws_spi.init");
        }
    }
    if (cfg->use_mosi) {
        ws_gpio_t mosi = cfg->mosi_pin;
        if (WS_FAILED(ws_gpio_init(&mosi, &pin_cfg))) {
            return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                          "SPI MOSI pin config failed", "ws_spi.init");
        }
    }
    if (WS_FAILED(ws_rcc_enable_periph(s_spi_rcc_periph(cfg->instance)))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "RCC failed to enable SPI", "ws_spi.init");
    }

    s_spis[(unsigned)cfg->instance].actual_hz = actual_hz;
    spi->instance = cfg->instance;
    spi->actual_hz = actual_hz;
    spi->initialized = true;
    return WS_ERR_OK(ws_spi_error_t);
}

ws_spi_error_t ws_spi_transfer(ws_spi_t *spi, const uint8_t *tx, uint8_t *rx, size_t len,
                               uint32_t timeout_ms)
{
    if (spi == NULL || !spi->initialized || (unsigned)spi->instance >= WS_SPI_INSTANCE_COUNT ||
        (len > 0u && tx == NULL && rx == NULL)) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid SPI handle or buffers", "ws_spi.transfer");
    }
    if (len == 0u) {
        return WS_ERR_OK(ws_spi_error_t);
    }

    s_spi_inst_fake_t *f = &s_spis[(unsigned)spi->instance];
    if (f->busy) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BUSY, WS_SEVERITY_DEGRADED, "SPI busy (fake)",
                      "ws_spi.transfer");
    }
    if (f->timeout) {
        ws_timebase_fake_advance_us((uint64_t)timeout_ms * 1000ULL);
        return WS_ERR(ws_spi_error_t, WS_SPI_TIMEOUT, WS_SEVERITY_DEGRADED, "SPI timed out (fake)",
                      "ws_spi.transfer");
    }
    if (f->bus_error) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BUS_ERROR, WS_SEVERITY_DEGRADED,
                      "SPI hardware bus error (fake)", "ws_spi.transfer");
    }

    for (size_t i = 0u; i < len; ++i) {
        const uint8_t out_byte = (tx != NULL) ? tx[i] : 0x00u;
        if (f->tx_len < WS_SPI_FAKE_BUF_CAP) {
            f->tx[f->tx_len] = out_byte;
            f->tx_len += 1u;
        }
        uint8_t in_byte = 0xFFu;
        if (f->rx_len > 0u) {
            in_byte = f->rx[f->rx_head];
            f->rx_head += 1u;
            f->rx_len -= 1u;
            if (f->rx_len == 0u) {
                f->rx_head = 0u;
            }
        }
        if (rx != NULL) {
            rx[i] = in_byte;
        }
    }

    f->transfer_count += 1u;
    return WS_ERR_OK(ws_spi_error_t);
}

ws_spi_error_t ws_spi_read(ws_spi_t *spi, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
    return ws_spi_transfer(spi, NULL, rx, len, timeout_ms);
}

ws_spi_error_t ws_spi_write(ws_spi_t *spi, const uint8_t *tx, size_t len, uint32_t timeout_ms)
{
    return ws_spi_transfer(spi, tx, NULL, len, timeout_ms);
}

void ws_spi_fake_reset(void)
{
    (void)memset(s_spis, 0, sizeof(s_spis));
}

void ws_spi_fake_queue_rx(ws_spi_instance_t instance, const uint8_t *buf, size_t len)
{
    if ((unsigned)instance >= WS_SPI_INSTANCE_COUNT || buf == NULL || len == 0u) {
        return;
    }
    s_spi_inst_fake_t *f = &s_spis[(unsigned)instance];
    if (f->rx_head > 0u && f->rx_len > 0u) {
        (void)memmove(f->rx, f->rx + f->rx_head, f->rx_len);
        f->rx_head = 0u;
    }
    for (size_t i = 0u; i < len; ++i) {
        if ((f->rx_head + f->rx_len) < WS_SPI_FAKE_BUF_CAP) {
            f->rx[f->rx_head + f->rx_len] = buf[i];
            f->rx_len += 1u;
        }
    }
}

size_t ws_spi_fake_take_tx(ws_spi_instance_t instance, uint8_t *buf, size_t len)
{
    if ((unsigned)instance >= WS_SPI_INSTANCE_COUNT || buf == NULL || len == 0u) {
        return 0u;
    }
    s_spi_inst_fake_t *f = &s_spis[(unsigned)instance];
    const size_t n = (len < f->tx_len) ? len : f->tx_len;
    (void)memcpy(buf, f->tx, n);
    (void)memmove(f->tx, f->tx + n, f->tx_len - n);
    f->tx_len -= n;
    return n;
}

void ws_spi_fake_set_busy(ws_spi_instance_t instance, bool busy)
{
    if ((unsigned)instance < WS_SPI_INSTANCE_COUNT) {
        s_spis[(unsigned)instance].busy = busy;
    }
}

void ws_spi_fake_set_timeout(ws_spi_instance_t instance, bool timeout)
{
    if ((unsigned)instance < WS_SPI_INSTANCE_COUNT) {
        s_spis[(unsigned)instance].timeout = timeout;
    }
}

void ws_spi_fake_set_bus_error(ws_spi_instance_t instance, bool bus_error)
{
    if ((unsigned)instance < WS_SPI_INSTANCE_COUNT) {
        s_spis[(unsigned)instance].bus_error = bus_error;
    }
}

uint32_t ws_spi_fake_transfer_count(ws_spi_instance_t instance)
{
    return ((unsigned)instance < WS_SPI_INSTANCE_COUNT) ? s_spis[(unsigned)instance].transfer_count
                                                        : 0u;
}

uint32_t ws_spi_fake_actual_hz(ws_spi_instance_t instance)
{
    return ((unsigned)instance < WS_SPI_INSTANCE_COUNT) ? s_spis[(unsigned)instance].actual_hz : 0u;
}
