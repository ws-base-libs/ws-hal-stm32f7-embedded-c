/**
 * @file uart.c
 * @brief ws-hal-stm32f7 UART — TARGET implementation (USART1 / USART6 over CMSIS).
 *
 * Multi-instance handle, bounded ws_time_us() timeouts, clock enable through
 * rcc.h, pin configuration through gpio.h — never touches RCC or GPIO registers.
 */
#if !defined(STM32F746xx)
#error "ws-hal-stm32f7 target build requires STM32F746xx"
#endif

#include "stm32f746xx.h"

#include <stddef.h>
#include <string.h>

#include <ws/hal/stm32f7/uart.h>

static USART_TypeDef *s_usart_regs(ws_uart_instance_t inst)
{
    switch (inst) {
    case WS_UART_1:
        return USART1;
    case WS_UART_6:
        return USART6;
    default:
        return NULL;
    }
}

static ws_rcc_periph_t s_usart_rcc_periph(ws_uart_instance_t inst)
{
    return (inst == WS_UART_1) ? WS_RCC_PERIPH_USART1 : WS_RCC_PERIPH_USART6;
}

static ws_uart_error_t s_check_cfg(const ws_uart_cfg_t *cfg)
{
    if (cfg == NULL || cfg->baud == 0u || cfg->af > 15u ||
        (unsigned)cfg->instance >= WS_UART_INSTANCE_COUNT ||
        (uint32_t)cfg->tx_pin.port > (uint32_t)WS_GPIO_PORT_K ||
        cfg->tx_pin.pin >= WS_GPIO_PIN_COUNT ||
        (uint32_t)cfg->rx_pin.port > (uint32_t)WS_GPIO_PORT_K ||
        cfg->rx_pin.pin >= WS_GPIO_PIN_COUNT) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART config", "ws_uart.init");
    }
    return WS_ERR_OK(ws_uart_error_t);
}

ws_uart_error_t ws_uart_init(ws_uart_t *uart, const ws_uart_cfg_t *cfg)
{
    if (uart == NULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "NULL uart handle", "ws_uart.init");
    }
    ws_uart_error_t err = s_check_cfg(cfg);
    if (WS_FAILED(err)) {
        return err;
    }

    const uint32_t pclk2 = ws_rcc_apb2_hz();
    /* Rounded 16x oversampling divider in 64-bit to avoid uint32 overflow (RM0385: 16..65535). */
    const uint64_t div64 = ((uint64_t)pclk2 + ((uint64_t)cfg->baud / 2ULL)) / (uint64_t)cfg->baud;
    if (div64 < 16ULL || div64 > 65535ULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "underivable USART BRR divider", "ws_uart.init");
    }
    const uint32_t div = (uint32_t)div64;

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

    USART_TypeDef *regs = s_usart_regs(cfg->instance);
    regs->CR1 = 0u;
    regs->CR2 = 0u;
    regs->CR3 = 0u;
    regs->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
    regs->BRR = div;
    regs->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

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
    if (uart == NULL || !uart->initialized || (buf == NULL && len > 0u)) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART handle or buffer", "ws_uart.write");
    }
    if (len == 0u) {
        return WS_ERR_OK(ws_uart_error_t);
    }

    USART_TypeDef *regs = s_usart_regs(uart->instance);
    if (regs == NULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "unknown UART instance", "ws_uart.write");
    }

    const uint64_t start = ws_time_us();
    const uint64_t budget_us = (uint64_t)timeout_ms * 1000ULL;
    size_t written = 0u;

    while (written < len) {
        while ((regs->ISR & USART_ISR_TXE) == 0u) {
            if ((ws_time_us() - start) >= budget_us) {
                if (out_written != NULL) {
                    *out_written = written;
                }
                return WS_ERR(ws_uart_error_t, WS_UART_TIMEOUT, WS_SEVERITY_DEGRADED,
                              "UART TXE wait timed out", "ws_uart.write");
            }
        }
        regs->TDR = (uint32_t)buf[written];
        ++written;
    }

    while ((regs->ISR & USART_ISR_TC) == 0u) {
        if ((ws_time_us() - start) >= budget_us) {
            if (out_written != NULL) {
                *out_written = written;
            }
            return WS_ERR(ws_uart_error_t, WS_UART_TIMEOUT, WS_SEVERITY_DEGRADED,
                          "UART TC wait timed out", "ws_uart.write");
        }
    }

    if (out_written != NULL) {
        *out_written = written;
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
    if (uart == NULL || !uart->initialized || out == NULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid UART handle or NULL out", "ws_uart.read_byte");
    }
    USART_TypeDef *regs = s_usart_regs(uart->instance);
    if (regs == NULL) {
        return WS_ERR(ws_uart_error_t, WS_UART_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "unknown UART instance", "ws_uart.read_byte");
    }
    const uint32_t isr = regs->ISR;
    const uint32_t rx_err_mask = USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE;
    if ((isr & rx_err_mask) != 0u) {
        uint32_t icr = 0u;
        if ((isr & USART_ISR_ORE) != 0u) {
            icr |= USART_ICR_ORECF;
        }
        if ((isr & USART_ISR_FE) != 0u) {
            icr |= USART_ICR_FECF;
        }
        if ((isr & USART_ISR_NE) != 0u) {
            icr |= USART_ICR_NCF;
        }
        if ((isr & USART_ISR_PE) != 0u) {
            icr |= USART_ICR_PECF;
        }
        /* Clear latched USARTv2 RX error flags via ICR so shift->RDR transfer resumes. */
        regs->ICR = icr;
        if ((isr & (USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0u &&
            (isr & USART_ISR_RXNE) != 0u) {
            volatile uint32_t discard = regs->RDR;
            (void)discard;
        }
        if ((isr & USART_ISR_ORE) != 0u) {
            return WS_ERR(ws_uart_error_t, WS_UART_OVERRUN, WS_SEVERITY_DEGRADED,
                          "UART RX overrun (ORE cleared)", "ws_uart.read_byte");
        }
        return WS_ERR(ws_uart_error_t, WS_UART_RX_ERROR, WS_SEVERITY_DEGRADED,
                      "UART RX framing/noise/parity error (cleared)", "ws_uart.read_byte");
    }
    if ((isr & USART_ISR_RXNE) == 0u) {
        return WS_ERR(ws_uart_error_t, WS_UART_NO_DATA, WS_SEVERITY_RECOVERABLE, "UART RX empty",
                      "ws_uart.read_byte");
    }
    *out = (uint8_t)(regs->RDR & USART_RDR_RDR_Msk);
    return WS_ERR_OK(ws_uart_error_t);
}
