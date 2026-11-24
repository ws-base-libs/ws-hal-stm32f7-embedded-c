/**
 * @file rcc_clocks.c
 * @brief Example: STM32F746G-DISCO 216 MHz clock-tree init via ws-hal-stm32f7.
 */
#include <ws/hal/stm32f7/rcc.h>

int main(void)
{
    static const ws_rcc_cfg_t k_rcc_cfg = {
        .hse_hz = 25000000u,
        .hse_bypass = true,
        .sysclk_hz = 216000000u,
        .apb1_hz = 54000000u,
        .apb2_hz = 108000000u,
    };
    return WS_OK(ws_rcc_init(&k_rcc_cfg)) ? 0 : 1;
}
