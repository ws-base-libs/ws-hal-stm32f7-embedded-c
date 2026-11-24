/**
 * @file timebase.c
 * @brief ws-hal-stm32f7 TIMEBASE — TARGET implementation over 32-bit TIM2.
 *
 * Uses TIM2 (32-bit general-purpose timer on APB1) at 1 MHz (1 us/tick) plus
 * TIM2_IRQHandler high-word extension for a true 64-bit monotonic counter.
 * SysTick_Handler is left completely untouched for FreeRTOS.
 */
#if !defined(STM32F746xx)
#error "ws-hal-stm32f7 target build requires STM32F746xx"
#endif

#include "stm32f746xx.h"

#include <stdbool.h>
#include <stddef.h>

#include <ws/hal/stm32f7/timebase.h>

#define WS_TIMEBASE_TIM_MAX_HZ (216000000UL)

static volatile uint32_t s_wrap_hi = 0u;

ws_timebase_error_t ws_timebase_init(const ws_timebase_cfg_t *cfg)
{
    if (cfg == NULL) {
        return WS_ERR(ws_timebase_error_t, WS_TIMEBASE_BAD_CONFIG, WS_SEVERITY_FATAL,
                      "NULL timebase cfg", "ws_timebase.init");
    }

    const uint32_t clk_hz = (cfg->tim_clk_hz != 0u) ? cfg->tim_clk_hz : ws_rcc_apb1_tim_hz();
    if (clk_hz == 0u || clk_hz > WS_TIMEBASE_TIM_MAX_HZ || (clk_hz % 1000000UL) != 0u) {
        return WS_ERR(ws_timebase_error_t, WS_TIMEBASE_BAD_CONFIG, WS_SEVERITY_FATAL,
                      "TIM2 clock must be 1..216 MHz (exact MHz)", "ws_timebase.init");
    }

    const uint32_t psc = (clk_hz / 1000000UL) - 1u;
    if (psc > 0xFFFFu) {
        return WS_ERR(ws_timebase_error_t, WS_TIMEBASE_BAD_CONFIG, WS_SEVERITY_FATAL,
                      "TIM2 prescaler overflow", "ws_timebase.init");
    }

    if (WS_FAILED(ws_rcc_enable_periph(WS_RCC_PERIPH_TIM2))) {
        return WS_ERR(ws_timebase_error_t, WS_TIMEBASE_BAD_CONFIG, WS_SEVERITY_FATAL,
                      "RCC failed to enable TIM2", "ws_timebase.init");
    }

    NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->CR1 = 0u;
    TIM2->DIER = 0u;
    TIM2->PSC = psc;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->CNT = 0u;
    TIM2->EGR = TIM_EGR_UG; /* latch prescaler immediately */
    TIM2->SR = 0u; /* clear UG-induced UIF */

    s_wrap_hi = 0u;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;

    return WS_ERR_OK(ws_timebase_error_t);
}

void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0u) {
        TIM2->SR = ~TIM_SR_UIF;
        /* Read-back barrier prevents Cortex-M7 buffered-write spurious re-entry (ARM AN321). */
        (void)TIM2->SR;
        s_wrap_hi += 1u;
    }
}

uint64_t ws_time_us(void)
{
    uint32_t hi1;
    uint32_t cnt;
    bool uif;
    do {
        hi1 = s_wrap_hi;
        cnt = TIM2->CNT;
        uif = (TIM2->SR & TIM_SR_UIF) != 0u;
    } while (hi1 != s_wrap_hi);

    /* If TIM2 wrapped while IRQs were masked, CNT is small and UIF is set. */
    if (uif && (cnt < 0x80000000UL)) {
        hi1 += 1u;
    }
    return ((uint64_t)hi1 << 32) | (uint64_t)cnt;
}

void ws_time_delay_us(uint32_t us)
{
    const uint64_t start = ws_time_us();
    while ((ws_time_us() - start) < (uint64_t)us) {
        /* busy-wait */
    }
}

void ws_time_delay_ms(uint32_t ms)
{
    const uint64_t start = ws_time_us();
    const uint64_t total = (uint64_t)ms * 1000ULL;
    while ((ws_time_us() - start) < total) {
        /* busy-wait */
    }
}
