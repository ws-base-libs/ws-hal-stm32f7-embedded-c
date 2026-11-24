/**
 * @file timebase.h
 * @brief ws-hal-stm32f7 TIMEBASE — 64-bit monotonic microsecond clock on TIM2.
 *
 * Uses STM32F7 TIM2 (32-bit free-running timer on APB1 + TIM2_IRQHandler high
 * word extension) rather than SysTick so FreeRTOS owns SysTick_Handler with
 * zero conflict (embedded_system_spec.md §12).
 *
 * Includes <ws/hal/stm32f7/rcc.h> and enables TIM2 clock automatically.
 */
#ifndef WS_WS_HAL_STM32F7_TIMEBASE_H
#define WS_WS_HAL_STM32F7_TIMEBASE_H

#include <stdint.h>

#include <ws/error/error.h>
#include <ws/hal/stm32f7/rcc.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_TIMEBASE_OK = 0,
    WS_TIMEBASE_BAD_CONFIG,
} ws_timebase_err_t;
WS_ERROR_TYPE(ws_timebase_err_t, ws_timebase_error_t);

/**
 * @brief Timebase configuration.
 *
 * Pass @c tim_clk_hz = 0 to auto-query ws_rcc_apb1_tim_hz() (recommended),
 * or an explicit APB1 timer clock in Hz (exact multiple of 1 MHz, 1..216 MHz).
 */
typedef struct {
    uint32_t tim_clk_hz;
} ws_timebase_cfg_t;

ws_timebase_error_t ws_timebase_init(const ws_timebase_cfg_t *cfg);

/** @brief 64-bit monotonic microseconds since init (ISR-safe, wrap-race-free). */
uint64_t ws_time_us(void);

/** @brief Busy-wait for @p us microseconds. */
void ws_time_delay_us(uint32_t us);

/** @brief Busy-wait for @p ms milliseconds (64-bit safe). */
void ws_time_delay_ms(uint32_t ms);

#if defined(WS_HAL_STM32F7_HOST_FAKE)
void ws_timebase_fake_reset(void);
void ws_timebase_fake_set_us(uint64_t us);
void ws_timebase_fake_advance_us(uint64_t us);
#endif /* WS_HAL_STM32F7_HOST_FAKE */

#ifdef __cplusplus
}
#endif

#endif /* WS_WS_HAL_STM32F7_TIMEBASE_H */
