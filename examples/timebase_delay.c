/**
 * @file timebase_delay.c
 * @brief Example: 64-bit TIM2 monotonic timebase and delay via ws-hal-stm32f7.
 */
#include <ws/hal/stm32f7/timebase.h>

int main(void)
{
    static const ws_timebase_cfg_t k_tb_cfg = { .tim_clk_hz = 0u }; /* auto-query RCC */
    if (WS_FAILED(ws_timebase_init(&k_tb_cfg))) {
        return 1;
    }
    const uint64_t t0 = ws_time_us();
    ws_time_delay_ms(10u);
    return (ws_time_us() - t0 >= 10000ULL) ? 0 : 1;
}
