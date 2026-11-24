/**
 * @file timebase_fake.c
 * @brief ws-hal-stm32f7 TIMEBASE — HOST FAKE (steerable 64-bit clock).
 */
#include <stddef.h>

#include <ws/hal/stm32f7/timebase.h>

#define WS_TIMEBASE_TIM_MAX_HZ (216000000UL)

static uint64_t s_now_us = 0u;

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
    if (WS_FAILED(ws_rcc_enable_periph(WS_RCC_PERIPH_TIM2))) {
        return WS_ERR(ws_timebase_error_t, WS_TIMEBASE_BAD_CONFIG, WS_SEVERITY_FATAL,
                      "RCC failed to enable TIM2", "ws_timebase.init");
    }

    s_now_us = 0u;
    return WS_ERR_OK(ws_timebase_error_t);
}

uint64_t ws_time_us(void)
{
    return s_now_us;
}

void ws_time_delay_us(uint32_t us)
{
    s_now_us += (uint64_t)us;
}

void ws_time_delay_ms(uint32_t ms)
{
    s_now_us += (uint64_t)ms * 1000ULL;
}

void ws_timebase_fake_reset(void)
{
    s_now_us = 0u;
}

void ws_timebase_fake_set_us(uint64_t us)
{
    s_now_us = us;
}

void ws_timebase_fake_advance_us(uint64_t us)
{
    s_now_us += us;
}
