/**
 * @file timebase_test.c
 * @brief Standalone host tests for ws-hal-stm32f7 TIMEBASE (TIM2).
 */
#include <stdio.h>

#include <ws/hal/stm32f7/timebase.h>

static int s_checks = 0;
static int s_failures = 0;

static void check(int cond, const char *desc)
{
    s_checks++;
    if (!cond) {
        s_failures++;
        fprintf(stderr, "FAIL: %s\n", desc);
    }
}

int main(void)
{
    ws_rcc_fake_reset();
    ws_timebase_fake_reset();

    /* Auto-query APB1 timer clock after 216 MHz RCC init (108 MHz). */
    const ws_rcc_cfg_t rcc_cfg = {
        .hse_hz = 25000000u,
        .hse_bypass = true,
        .sysclk_hz = 216000000u,
        .apb1_hz = 54000000u,
        .apb2_hz = 108000000u,
    };
    check(WS_OK(ws_rcc_init(&rcc_cfg)), "rcc init ok");

    const ws_timebase_cfg_t auto_cfg = { .tim_clk_hz = 0u };
    check(WS_OK(ws_timebase_init(&auto_cfg)), "timebase auto-query init ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_TIM2), "TIM2 clock enabled");
    check(ws_time_us() == 0u, "clock starts at 0");

    ws_time_delay_us(250u);
    check(ws_time_us() == 250u, "delay_us advances 250 us");
    ws_time_delay_ms(5u);
    check(ws_time_us() == 5250u, "delay_ms advances 5000 us");

    /* 64-bit steer past 32-bit wrap (~71.5 min). */
    ws_timebase_fake_set_us(0x100000000ULL);
    ws_timebase_fake_advance_us(1000u);
    check(ws_time_us() == 0x1000003E8ULL, "64-bit monotonic past 32-bit boundary");

    /* Rejections. */
    check(ws_timebase_init(NULL).code == WS_TIMEBASE_BAD_CONFIG, "NULL cfg rejected");
    const ws_timebase_cfg_t bad_non_mhz = { .tim_clk_hz = 16500000u };
    check(ws_timebase_init(&bad_non_mhz).code == WS_TIMEBASE_BAD_CONFIG, "non-MHz clock rejected");

    printf("timebase_test: %d checks, %d failures: %s\n", s_checks, s_failures,
           s_failures == 0 ? "PASS" : "FAIL");
    return s_failures == 0 ? 0 : 1;
}
