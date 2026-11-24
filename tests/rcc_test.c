/**
 * @file rcc_test.c
 * @brief Standalone host tests for ws-hal-stm32f7 RCC.
 */
#include <stdio.h>

#include <ws/hal/stm32f7/rcc.h>

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
    check(ws_rcc_sysclk_hz() == 16000000u, "reset sysclk is 16 MHz HSI");
    check(ws_rcc_apb1_tim_hz() == 16000000u, "reset APB1 timer clock is 16 MHz");

    /* STM32F746G-DISCO: 25 MHz HSE bypass -> 216 MHz SYSCLK, 54 MHz APB1, 108 MHz APB2. */
    const ws_rcc_cfg_t k_disco_cfg = {
        .hse_hz = 25000000u,
        .hse_bypass = true,
        .sysclk_hz = 216000000u,
        .apb1_hz = 54000000u,
        .apb2_hz = 108000000u,
    };
    ws_rcc_error_t err = ws_rcc_init(&k_disco_cfg);
    check(WS_OK(err), "216 MHz DISCO clock init ok");
    ws_rcc_fake_state_t st;
    ws_rcc_fake_get_state(&st);
    check(st.pll_m == 25u && st.pll_n == 432u && st.pll_p == 2u, "PLL M=25 N=432 P=2 derived");
    check(st.apb1_prescaler == 4u && st.apb2_prescaler == 2u, "APB1 /4 and APB2 /2 derived");
    check(st.flash_ws == 7u && st.overdrive, "7 Flash WS and Over-Drive enabled at 216 MHz");
    check(ws_rcc_apb1_tim_hz() == 108000000u, "APB1 timer clock is 2*PCLK1 = 108 MHz");

    /* HSI direct 16 MHz mode. */
    const ws_rcc_cfg_t k_hsi_cfg = {
        .hse_hz = 0u,
        .hse_bypass = false,
        .sysclk_hz = 16000000u,
        .apb1_hz = 16000000u,
        .apb2_hz = 16000000u,
    };
    check(WS_OK(ws_rcc_init(&k_hsi_cfg)), "16 MHz HSI direct ok");
    ws_rcc_fake_get_state(&st);
    check(st.pll_n == 0u && st.flash_ws == 0u && !st.overdrive, "HSI direct bypasses PLL & OD");

    /* Rejections: NULL, over-limit, bad HSE, bad APB ratio. */
    check(ws_rcc_init(NULL).code == WS_RCC_BAD_FREQ, "NULL cfg rejected");
    ws_rcc_cfg_t bad = k_disco_cfg;
    bad.sysclk_hz = 217000000u;
    check(ws_rcc_init(&bad).code == WS_RCC_BAD_FREQ, ">216 MHz rejected");
    bad = k_disco_cfg;
    bad.apb1_hz = 55000000u;
    check(ws_rcc_init(&bad).code == WS_RCC_BAD_FREQ, ">54 MHz APB1 rejected");
    bad = k_disco_cfg;
    bad.hse_hz = 2500000u;
    check(ws_rcc_init(&bad).code == WS_RCC_BAD_FREQ, "<4 MHz HSE rejected");

    /* GPIO port & peripheral clock enables. */
    check(WS_OK(ws_rcc_enable_gpio_port(WS_GPIO_PORT_A)), "enable GPIOA ok");
    check(WS_OK(ws_rcc_enable_gpio_port(WS_GPIO_PORT_A)), "re-enable GPIOA idempotent");
    check(ws_rcc_fake_port_enable_count(WS_GPIO_PORT_A) == 1u, "GPIOA enabled once");
    check(ws_rcc_enable_gpio_port((ws_gpio_port_t)99).code == WS_RCC_BAD_PORT, "bad port rejected");

    check(WS_OK(ws_rcc_enable_periph(WS_RCC_PERIPH_TIM2)), "enable TIM2 ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_TIM2), "TIM2 clock active");
    check(WS_OK(ws_rcc_enable_periph(WS_RCC_PERIPH_SPI1)), "enable SPI1 ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_SPI1), "SPI1 clock active");
    check(WS_OK(ws_rcc_enable_periph(WS_RCC_PERIPH_SPI2)), "enable SPI2 ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_SPI2), "SPI2 clock active");
    check(ws_rcc_enable_periph((ws_rcc_periph_t)99).code == WS_RCC_BAD_PERIPH,
          "bad periph rejected");

    printf("rcc_test: %d checks, %d failures: %s\n", s_checks, s_failures,
           s_failures == 0 ? "PASS" : "FAIL");
    return s_failures == 0 ? 0 : 1;
}
