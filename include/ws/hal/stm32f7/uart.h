/**
 * @file uart.h
 * @brief ws-hal-stm32f7 UART — multi-instance UART with bounded timeouts.
 *
 * Includes <ws/hal/stm32f7/gpio.h> and <ws/hal/stm32f7/timebase.h>.
 * Enables peripheral clocks through rcc.h and configures pins through gpio.h;
 * never touches RCC or GPIO registers directly.
 */
#ifndef WS_WS_HAL_STM32F7_UART_H
#define WS_WS_HAL_STM32F7_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ws/error/error.h>
#include <ws/hal/stm32f7/gpio.h>
#include <ws/hal/stm32f7/timebase.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_UART_OK = 0,
    WS_UART_BAD_CONFIG,
    WS_UART_TIMEOUT,
    WS_UART_NO_DATA,
    WS_UART_OVERRUN,
    WS_UART_RX_ERROR,
} ws_uart_err_t;
WS_ERROR_TYPE(ws_uart_err_t, ws_uart_error_t);

typedef enum {
    WS_UART_1 = 0, /**< USART1 (APB2; ST-Link VCP on STM32F746G-DISCO: PA9/PB7 AF7). */
    WS_UART_6, /**< USART6 (APB2; Arduino D1/D0 on STM32F746G-DISCO: PC6/PC7 AF8). */
} ws_uart_instance_t;

#define WS_UART_INSTANCE_COUNT (WS_UART_6 + 1)

typedef struct {
    ws_uart_instance_t instance;
    uint32_t baud;
    ws_gpio_t tx_pin;
    ws_gpio_t rx_pin;
    uint8_t af;
} ws_uart_cfg_t;

typedef struct {
    ws_uart_instance_t instance;
    bool initialized;
} ws_uart_t;

ws_uart_error_t ws_uart_init(ws_uart_t *uart, const ws_uart_cfg_t *cfg);

/**
 * @brief Blocking byte transmit with explicit timeout in milliseconds.
 *
 * Writes up to @p len bytes; if @p out_written is non-NULL, stores the number
 * of bytes actually transmitted before completion or timeout.
 */
ws_uart_error_t ws_uart_write(ws_uart_t *uart, const uint8_t *buf, size_t len, uint32_t timeout_ms,
                              size_t *out_written);

/** @brief Blocking NUL-terminated string transmit with explicit timeout. */
ws_uart_error_t ws_uart_write_str(ws_uart_t *uart, const char *s, uint32_t timeout_ms,
                                  size_t *out_written);

/**
 * @brief Non-blocking single-byte read.
 *
 * Clears STM32F7 USARTv2 ORE/FE/NE/PE flags via ICR so RX cannot wedge on
 * overrun (RM0385 §31.5.6).
 *
 * @return WS_UART_OK when a byte is read into @p out, WS_UART_NO_DATA when
 *         empty, WS_UART_OVERRUN when ORE was latched (cleared; any buffered
 *         byte remains readable on the next call), WS_UART_RX_ERROR when
 *         FE/NE/PE was latched (cleared and corrupted byte discarded), or
 *         WS_UART_BAD_CONFIG on invalid handle/NULL out.
 */
ws_uart_error_t ws_uart_read_byte(ws_uart_t *uart, uint8_t *out);

#if defined(WS_HAL_STM32F7_HOST_FAKE)
void ws_uart_fake_reset(void);
size_t ws_uart_fake_take_output(ws_uart_instance_t instance, uint8_t *buf, size_t len);
void ws_uart_fake_inject_rx(ws_uart_instance_t instance, const uint8_t *buf, size_t len);
void ws_uart_fake_inject_overrun(ws_uart_instance_t instance);
void ws_uart_fake_inject_rx_error(ws_uart_instance_t instance);
void ws_uart_fake_set_tx_stall(ws_uart_instance_t instance, bool stall);
#endif /* WS_HAL_STM32F7_HOST_FAKE */

#ifdef __cplusplus
}
#endif

#endif /* WS_WS_HAL_STM32F7_UART_H */
