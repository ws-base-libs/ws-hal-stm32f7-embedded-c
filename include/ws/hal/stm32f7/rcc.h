/**
 * @file rcc.h
 * @brief ws-hal-stm32f7 RCC — reset & clock control for STM32F7 (clocks as CONFIG).
 *
 * The ONLY file allowed to touch RCC/PWR/FLASH clock-tree registers (in src/rcc.c).
 * Peripheral drivers enable their clocks through ws_rcc_enable_gpio_port() and
 * ws_rcc_enable_periph() and query bus clocks through ws_rcc_*_hz().
 *
 * Verified against STM32F746xx datasheet DS10916 Rev 5 and RM0385.
 */
#ifndef WS_WS_HAL_STM32F7_RCC_H
#define WS_WS_HAL_STM32F7_RCC_H

#include <stdbool.h>
#include <stdint.h>

#include <ws/error/error.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_RCC_OK = 0,
    WS_RCC_BAD_FREQ, /**< NULL, out-of-range, or underivable PLL/prescaler config. */
    WS_RCC_BAD_PORT, /**< GPIO port enum out of range. */
    WS_RCC_BAD_PERIPH, /**< Peripheral enum out of range. */
    WS_RCC_TIMEOUT, /**< Hardware ready flag (HSE/PLL/Over-Drive/SWS) timed out. */
} ws_rcc_err_t;
WS_ERROR_TYPE(ws_rcc_err_t, ws_rcc_error_t);

typedef enum {
    WS_GPIO_PORT_A = 0,
    WS_GPIO_PORT_B,
    WS_GPIO_PORT_C,
    WS_GPIO_PORT_D,
    WS_GPIO_PORT_E,
    WS_GPIO_PORT_F,
    WS_GPIO_PORT_G,
    WS_GPIO_PORT_H,
    WS_GPIO_PORT_I,
    WS_GPIO_PORT_J,
    WS_GPIO_PORT_K,
} ws_gpio_port_t;

#define WS_GPIO_PORT_COUNT (WS_GPIO_PORT_K + 1)

typedef enum {
    WS_RCC_PERIPH_TIM2 = 0,
    WS_RCC_PERIPH_USART1,
    WS_RCC_PERIPH_USART6,
} ws_rcc_periph_t;

#define WS_RCC_PERIPH_COUNT (WS_RCC_PERIPH_USART6 + 1)

/**
 * @brief Clock-tree configuration (frequencies & source as config inputs).
 *
 * - hse_hz == 0: use 16 MHz HSI (direct when sysclk_hz == 16 MHz, else via PLL).
 * - hse_hz > 0:  use external HSE (4..26 MHz, exact multiple of 1 MHz; e.g.
 *                25 MHz with hse_bypass = true on STM32F746G-DISCO per UM1907).
 * - sysclk_hz:   1..216 MHz (HCLK = SYSCLK, AHB /1).
 * - apb1_hz:     <= 54 MHz, exact quotient of sysclk_hz by {1,2,4,8,16}.
 * - apb2_hz:     <= 108 MHz, exact quotient of sysclk_hz by {1,2,4,8,16}.
 */
typedef struct {
    uint32_t hse_hz;
    bool hse_bypass;
    uint32_t sysclk_hz;
    uint32_t apb1_hz;
    uint32_t apb2_hz;
} ws_rcc_cfg_t;

ws_rcc_error_t ws_rcc_init(const ws_rcc_cfg_t *cfg);
ws_rcc_error_t ws_rcc_enable_gpio_port(ws_gpio_port_t port);
ws_rcc_error_t ws_rcc_enable_periph(ws_rcc_periph_t periph);

/** @brief Active clock queries (default to 16 MHz HSI reset state before ws_rcc_init). */
uint32_t ws_rcc_sysclk_hz(void);
uint32_t ws_rcc_apb1_hz(void);
uint32_t ws_rcc_apb2_hz(void);
/** @brief APB1 timer kernel clock (PCLK1 when PPRE1=/1, else 2*PCLK1 per RM0385). */
uint32_t ws_rcc_apb1_tim_hz(void);

#if defined(WS_HAL_STM32F7_HOST_FAKE)
typedef struct {
    uint32_t hse_hz;
    bool hse_bypass;
    uint32_t sysclk_hz;
    uint32_t apb1_hz;
    uint32_t apb2_hz;
    uint32_t apb1_prescaler;
    uint32_t apb2_prescaler;
    uint32_t pll_m;
    uint32_t pll_n;
    uint32_t pll_p;
    uint32_t flash_ws;
    bool overdrive;
} ws_rcc_fake_state_t;

void ws_rcc_fake_reset(void);
bool ws_rcc_fake_port_enabled(ws_gpio_port_t port);
uint32_t ws_rcc_fake_port_enable_count(ws_gpio_port_t port);
bool ws_rcc_fake_periph_enabled(ws_rcc_periph_t periph);
void ws_rcc_fake_get_state(ws_rcc_fake_state_t *out);
#endif /* WS_HAL_STM32F7_HOST_FAKE */

#ifdef __cplusplus
}
#endif

#endif /* WS_WS_HAL_STM32F7_RCC_H */
