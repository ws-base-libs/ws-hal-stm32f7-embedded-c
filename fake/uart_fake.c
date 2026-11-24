/**
 * @file uart_fake.c
 * @brief ws-hal-stm32f7 UART — HOST FAKE (multi-instance, timeout stall seam).
 */
#include <string.h>

#include <ws/hal/stm32f7/uart.h>

#define WS_UART_FAKE_OUTPUT_CAP 512u
#define WS_UART_FAKE_RX_CAP 256u

typedef struct {
    uint8_t output[WS_UART_FAKE_OUTPUT_CAP];
    size_t output_len;
    uint8_t rx[WS_UART_FAKE_RX_CAP];
    size_t rx_head;
    size_t rx_len;
    bool tx_stall;
    bool rx_overrun;
    bool rx_error;
} s_uart_inst_fake_t;

static s_uart_inst_fake_t s_uarts[WS_UART_INSTANCE_COUNT];

static ws_rcc_periph_t s_usart_rcc_periph(ws_uart_instance_t inst)
{
    return (inst == WS_UART_1) ? WS_RCC_PERIPH_USART1 : WS_RCC_PERIPH_USART6;
}

ws_uart_error_t ws_uart_init(ws_uart_t *uart, const ws_uart_cfg_t *cfg)
{
    if (uart == NULL || cfg == NULL || cfg->baud == 0u || cfg->af > 15u ||
        (unsigned)cfg->instance >= WS_UART_INSTANCE_COUNT ||
        (uint32_t)cfg->tx_pin.port > (uint32_t)WS_GPIO_PORT_K ||
        cfg->tx_pin.pin >= WS_GPIO_PIN_COUNT ||
        (uint32_t)cfg->rx_pin.port > (uint32_t)WS_GPIO_PORT_K ||
        cfg->rx_pin.pin >= WS_GPIO_PIN_COUNT) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART handle or config", "ws_uart.init");
    }

    const uint32_t pclk2 = ws_rcc_apb2_hz();
    const uint64_t div64 = ((uint64_t)pclk2 + ((uint64_t)cfg->baud / 2ULL)) / (uint64_t)cfg->baud;
    if (div64 < 16ULL || div64 > 65535ULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "underivable USART BRR divider", "ws_uart.init");
    }

    const ws_gpio_cfg_t tx_cfg = {
        .mode = WS_GPIO_MODE_ALTERNATE,
        .pull = WS_GPIO_PULL_NONE,
        .speed = WS_GPIO_SPEED_HIGH,
        .af = cfg->af,
    };
    const ws_gpio_cfg_t rx_cfg = {
        .mode = WS_GPIO_MODE_ALTERNATE,
        .pull = WS_GPIO_PULL_UP,
        .speed = WS_GPIO_SPEED_HIGH,
        .af = cfg->af,
    };
    ws_gpio_t tx = cfg->tx_pin;
    ws_gpio_t rx = cfg->rx_pin;
    if (WS_FAILED(ws_gpio_init(&tx, &tx_cfg)) || WS_FAILED(ws_gpio_init(&rx, &rx_cfg))) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "UART pin configuration failed", "ws_uart.init");
    }
    if (WS_FAILED(ws_rcc_enable_periph(s_usart_rcc_periph(cfg->instance)))) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "RCC failed to enable USART", "ws_uart.init");
    }

    s_uarts[(unsigned)cfg->instance].rx_overrun = false;
    s_uarts[(unsigned)cfg->instance].rx_error = false;
    uart->instance = cfg->instance;
    uart->initialized = true;
    return WS_ERR_OK(ws_uart_error_t);
}

ws_uart_error_t ws_uart_write(ws_uart_t *uart, const uint8_t *buf, size_t len, uint32_t timeout_ms,
                              size_t *out_written)
{
    if (out_written != NULL) {
        *out_written = 0u;
    }
    if (uart == NULL || !uart->initialized || (unsigned)uart->instance >= WS_UART_INSTANCE_COUNT ||
        (buf == NULL && len > 0u)) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART handle or buffer", "ws_uart.write");
    }
    if (len == 0u) {
        return WS_ERR_OK(ws_uart_error_t);
    }

    s_uart_inst_fake_t *f = &s_uarts[(unsigned)uart->instance];
    if (f->tx_stall) {
        ws_timebase_fake_advance_us((uint64_t)timeout_ms * 1000ULL);
        return WS_ERR(ws_uart_error_t, WS_UART_TIMEOUT, WS_SEVERITY_DEGRADED,
                      "UART TX stalled (fake)", "ws_uart.write");
    }

    for (size_t i = 0u; i < len; ++i) {
        if (f->output_len < WS_UART_FAKE_OUTPUT_CAP) {
            f->output[f->output_len] = buf[i];
            f->output_len += 1u;
        }
    }
    if (out_written != NULL) {
        *out_written = len;
    }
    return WS_ERR_OK(ws_uart_error_t);
}

ws_uart_error_t ws_uart_write_str(ws_uart_t *uart, const char *s, uint32_t timeout_ms,
                                  size_t *out_written)
{
    if (s == NULL) {
        if (out_written != NULL) {
            *out_written = 0u;
        }
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE, "NULL string",
                      "ws_uart.write_str");
    }
    return ws_uart_write(uart, (const uint8_t *)s, strlen(s), timeout_ms, out_written);
}

ws_uart_error_t ws_uart_read_byte(ws_uart_t *uart, uint8_t *out)
{
    if (uart == NULL || !uart->initialized || (unsigned)uart->instance >= WS_UART_INSTANCE_COUNT ||
        out == NULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART handle or NULL out", "ws_uart.read_byte");
    }
    s_uart_inst_fake_t *f = &s_uarts[(unsigned)uart->instance];
    if (f->rx_overrun || f->rx_error) {
        const bool was_ore = f->rx_overrun;
        const bool was_err = f->rx_error;
        f->rx_overrun = false;
        f->rx_error = false;
        if (was_err && f->rx_len > 0u) {
            f->rx_head += 1u;
            f->rx_len -= 1u;
            if (f->rx_len == 0u) {
                f->rx_head = 0u;
            }
        }
        if (was_ore) {
            return WS_ERR(ws_uart_error_t, WS_UART_OVERRUN, WS_SEVERITY_DEGRADED,
                          "UART RX overrun (fake, cleared)", "ws_uart.read_byte");
        }
        return WS_ERR(ws_uart_error_t, WS_UART_RX_ERROR, WS_SEVERITY_DEGRADED,
                      "UART RX framing/noise/parity error (fake, cleared)", "ws_uart.read_byte");
    }
    if (f->rx_len == 0u) {
        return WS_ERR(ws_uart_error_t, WS_UART_NO_DATA, WS_SEVERITY_RECOVERABLE, "UART RX empty",
                      "ws_uart.read_byte");
    }
    *out = f->rx[f->rx_head];
    f->rx_head += 1u;
    f->rx_len -= 1u;
    if (f->rx_len == 0u) {
        f->rx_head = 0u;
    }
    return WS_ERR_OK(ws_uart_error_t);
}

void ws_uart_fake_reset(void)
{
    (void)memset(s_uarts, 0, sizeof(s_uarts));
}

size_t ws_uart_fake_take_output(ws_uart_instance_t instance, uint8_t *buf, size_t len)
{
    if ((unsigned)instance >= WS_UART_INSTANCE_COUNT || buf == NULL || len == 0u) {
        return 0u;
    }
    s_uart_inst_fake_t *f = &s_uarts[(unsigned)instance];
    const size_t n = (len < f->output_len) ? len : f->output_len;
    (void)memcpy(buf, f->output, n);
    (void)memmove(f->output, f->output + n, f->output_len - n);
    f->output_len -= n;
    return n;
}

void ws_uart_fake_inject_rx(ws_uart_instance_t instance, const uint8_t *buf, size_t len)
{
    if ((unsigned)instance >= WS_UART_INSTANCE_COUNT || buf == NULL) {
        return;
    }
    s_uart_inst_fake_t *f = &s_uarts[(unsigned)instance];
    for (size_t i = 0u; i < len; ++i) {
        const size_t used = f->rx_head + f->rx_len;
        if (used < WS_UART_FAKE_RX_CAP) {
            f->rx[used] = buf[i];
            f->rx_len += 1u;
        }
    }
}

void ws_uart_fake_inject_overrun(ws_uart_instance_t instance)
{
    if ((unsigned)instance < WS_UART_INSTANCE_COUNT) {
        s_uarts[(unsigned)instance].rx_overrun = true;
    }
}

void ws_uart_fake_inject_rx_error(ws_uart_instance_t instance)
{
    if ((unsigned)instance < WS_UART_INSTANCE_COUNT) {
        s_uarts[(unsigned)instance].rx_error = true;
    }
}

void ws_uart_fake_set_tx_stall(ws_uart_instance_t instance, bool stall)
{
    if ((unsigned)instance < WS_UART_INSTANCE_COUNT) {
        s_uarts[(unsigned)instance].tx_stall = stall;
    }
}
