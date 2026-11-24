/**
 * @file rcc.c
 * @brief ws-hal-stm32f7 RCC — TARGET implementation over CMSIS registers.
 *
 * Verified against STM32F746xx datasheet DS10916 Rev 5 and RM0385:
 *   - SYSCLK <= 216 MHz, APB1 <= 54 MHz, APB2 <= 108 MHz
 *   - HSE: 4..26 MHz (exact MHz); HSI: 16 MHz
 *   - PLL: VCO input 1 MHz (PLLM = src_hz / 1 MHz), VCO output 100..432 MHz,
 *     PLLP in {2, 4, 6, 8}
 *   - Over-Drive enabled for SYSCLK > 180 MHz (PWR_CR1_ODEN / ODSWEN)
 *   - Flash wait states at 2.7..3.6 V: 0..7 WS per 30 MHz step + ART + prefetch
 */
#if !defined(STM32F746xx)
#error "ws-hal-stm32f7 target build requires STM32F746xx"
#endif

#include "stm32f746xx.h"

#include <stddef.h>

#include <ws/hal/stm32f7/rcc.h>

#define WS_RCC_HSI_HZ (16000000UL)
#define WS_RCC_HSE_MIN_HZ (4000000UL)
#define WS_RCC_HSE_MAX_HZ (26000000UL)
#define WS_RCC_SYSCLK_MAX_HZ (216000000UL)
#define WS_RCC_APB1_MAX_HZ (54000000UL)
#define WS_RCC_APB2_MAX_HZ (108000000UL)
#define WS_RCC_OVERDRIVE_HZ (180000000UL)
#define WS_RCC_WAIT_LIMIT (1000000UL)

static const uint32_t k_apb_divs[] = { 1u, 2u, 4u, 8u, 16u };
static const uint32_t k_apb1_cfgr[] = {
    RCC_CFGR_PPRE1_DIV1, RCC_CFGR_PPRE1_DIV2,  RCC_CFGR_PPRE1_DIV4,
    RCC_CFGR_PPRE1_DIV8, RCC_CFGR_PPRE1_DIV16,
};
static const uint32_t k_apb2_cfgr[] = {
    RCC_CFGR_PPRE2_DIV1, RCC_CFGR_PPRE2_DIV2,  RCC_CFGR_PPRE2_DIV4,
    RCC_CFGR_PPRE2_DIV8, RCC_CFGR_PPRE2_DIV16,
};

static uint32_t s_sysclk_hz = WS_RCC_HSI_HZ;
static uint32_t s_apb1_hz = WS_RCC_HSI_HZ;
static uint32_t s_apb2_hz = WS_RCC_HSI_HZ;
static uint32_t s_apb1_div = 1u;

typedef struct {
    uint32_t apb1_div;
    uint32_t apb2_div;
    bool use_pll;
    uint32_t pll_m;
    uint32_t pll_n;
    uint32_t pll_p;
    uint32_t flash_ws;
    bool overdrive;
} s_rcc_plan_t;

static uint32_t s_derive_apb_div(uint32_t sysclk_hz, uint32_t apb_hz)
{
    if (apb_hz == 0u || apb_hz > sysclk_hz) {
        return 0u;
    }
    for (uint32_t i = 0u; i < (sizeof(k_apb_divs) / sizeof(k_apb_divs[0])); ++i) {
        if ((apb_hz * k_apb_divs[i]) == sysclk_hz) {
            return k_apb_divs[i];
        }
    }
    return 0u;
}

static ws_rcc_error_t s_plan_clocks(const ws_rcc_cfg_t *cfg, s_rcc_plan_t *plan)
{
    if (cfg == NULL) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL, "NULL rcc cfg",
                      "ws_rcc.init");
    }
    if (cfg->sysclk_hz == 0u || cfg->sysclk_hz > WS_RCC_SYSCLK_MAX_HZ || cfg->apb1_hz == 0u ||
        cfg->apb1_hz > WS_RCC_APB1_MAX_HZ || cfg->apb2_hz == 0u ||
        cfg->apb2_hz > WS_RCC_APB2_MAX_HZ) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL,
                      "clock frequency out of STM32F746 limits", "ws_rcc.init");
    }
    if (cfg->hse_hz != 0u && (cfg->hse_hz < WS_RCC_HSE_MIN_HZ || cfg->hse_hz > WS_RCC_HSE_MAX_HZ ||
                              (cfg->hse_hz % 1000000u) != 0u)) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL,
                      "HSE must be 0 or 4..26 MHz (exact MHz)", "ws_rcc.init");
    }

    const uint32_t d1 = s_derive_apb_div(cfg->sysclk_hz, cfg->apb1_hz);
    const uint32_t d2 = s_derive_apb_div(cfg->sysclk_hz, cfg->apb2_hz);
    if (d1 == 0u || d2 == 0u) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL,
                      "underivable APB prescaler ratio", "ws_rcc.init");
    }

    plan->apb1_div = d1;
    plan->apb2_div = d2;
    plan->flash_ws = (cfg->sysclk_hz - 1u) / 30000000UL;
    plan->overdrive = (cfg->sysclk_hz > WS_RCC_OVERDRIVE_HZ);

    if (cfg->hse_hz == 0u && cfg->sysclk_hz == WS_RCC_HSI_HZ) {
        plan->use_pll = false;
        plan->pll_m = 0u;
        plan->pll_n = 0u;
        plan->pll_p = 0u;
        return WS_ERR_OK(ws_rcc_error_t);
    }

    const uint32_t src_hz = (cfg->hse_hz != 0u) ? cfg->hse_hz : WS_RCC_HSI_HZ;
    const uint32_t pll_m = src_hz / 1000000UL; /* 1 MHz VCO input (RM0385: 1..2 MHz) */
    if (pll_m < 2u || pll_m > 63u) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL, "PLLM out of range",
                      "ws_rcc.init");
    }

    static const uint32_t k_pllp[] = { 2u, 4u, 6u, 8u };
    uint32_t chosen_n = 0u;
    uint32_t chosen_p = 0u;
    for (uint32_t i = 0u; i < 4u; ++i) {
        const uint64_t vco = (uint64_t)cfg->sysclk_hz * (uint64_t)k_pllp[i];
        if ((vco % 1000000ULL) == 0ULL) {
            const uint64_t n = vco / 1000000ULL;
            if (n >= 100ULL && n <= 432ULL) {
                chosen_n = (uint32_t)n;
                chosen_p = k_pllp[i];
                break;
            }
        }
    }
    if (chosen_n == 0u) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL,
                      "cannot derive exact STM32F746 PLL M/N/P", "ws_rcc.init");
    }

    plan->use_pll = true;
    plan->pll_m = pll_m;
    plan->pll_n = chosen_n;
    plan->pll_p = chosen_p;
    return WS_ERR_OK(ws_rcc_error_t);
}

static bool s_wait_mask(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    for (uint32_t i = 0u; i < WS_RCC_WAIT_LIMIT; ++i) {
        if ((*reg & mask) == expected) {
            return true;
        }
    }
    return false;
}

ws_rcc_error_t ws_rcc_init(const ws_rcc_cfg_t *cfg)
{
    s_rcc_plan_t plan;
    ws_rcc_error_t err = s_plan_clocks(cfg, &plan);
    if (WS_FAILED(err)) {
        return err;
    }

    uint32_t ppre1 = k_apb1_cfgr[0];
    uint32_t ppre2 = k_apb2_cfgr[0];
    for (uint32_t i = 0u; i < (sizeof(k_apb_divs) / sizeof(k_apb_divs[0])); ++i) {
        if (k_apb_divs[i] == plan.apb1_div) {
            ppre1 = k_apb1_cfgr[i];
        }
        if (k_apb_divs[i] == plan.apb2_div) {
            ppre2 = k_apb2_cfgr[i];
        }
    }

    /* Enable PWR peripheral clock and select Voltage Scale 1 (needed up to 216 MHz). */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    {
        volatile uint32_t rd = RCC->APB1ENR;
        (void)rd;
    }
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS;

    /* Ensure HSI is active before reconfiguring PLL / switching sources. */
    RCC->CR |= RCC_CR_HSION;
    if (!s_wait_mask(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY)) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL, "HSI ready timeout",
                      "ws_rcc.init");
    }

    /* Switch SYSCLK to HSI safely before altering PLL. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    if (!s_wait_mask(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI)) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL, "HSI switch timeout",
                      "ws_rcc.init");
    }

    if (!plan.use_pll) {
        FLASH->ACR = (plan.flash_ws & FLASH_ACR_LATENCY_Msk) | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;
        if (!s_wait_mask(&FLASH->ACR, FLASH_ACR_LATENCY_Msk, plan.flash_ws)) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                          "FLASH latency verify timeout", "ws_rcc.init");
        }
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                    RCC_CFGR_HPRE_DIV1 | ppre1 | ppre2;
    } else {
        if (cfg->hse_hz != 0u) {
            /* RM0385 §5.3.1: HSEBYP can only be modified while HSE oscillator is disabled. */
            RCC->CR &= ~RCC_CR_HSEON;
            if (!s_wait_mask(&RCC->CR, RCC_CR_HSERDY, 0u)) {
                return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                              "HSE disable timeout", "ws_rcc.init");
            }
            if (cfg->hse_bypass) {
                RCC->CR |= RCC_CR_HSEBYP;
            } else {
                RCC->CR &= ~RCC_CR_HSEBYP;
            }
            RCC->CR |= RCC_CR_HSEON;
            if (!s_wait_mask(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY)) {
                return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                              "HSE ready timeout", "ws_rcc.init");
            }
        }

        /* Disable PLL before modifying PLLCFGR. */
        RCC->CR &= ~RCC_CR_PLLON;
        if (!s_wait_mask(&RCC->CR, RCC_CR_PLLRDY, 0u)) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL, "PLL disable timeout",
                          "ws_rcc.init");
        }

        const uint32_t pllp_bits = ((plan.pll_p / 2u) - 1u) << RCC_PLLCFGR_PLLP_Pos;
        const uint32_t pllq_div = (plan.pll_n + 47u) / 48u; /* <= 48 MHz USB/SDMMC */
        const uint32_t pllq_clamped = (pllq_div < 2u) ? 2u : (pllq_div > 15u ? 15u : pllq_div);
        const uint32_t pll_src =
            (cfg->hse_hz != 0u) ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI;

        /* RM0385 §5.3.2: read-modify-write to keep reserved bits (incl. bit 29) at reset value. */
        const uint32_t pll_mask = RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP |
                                  RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ;
        RCC->PLLCFGR = (RCC->PLLCFGR & ~pll_mask) | (plan.pll_m << RCC_PLLCFGR_PLLM_Pos) |
                       (plan.pll_n << RCC_PLLCFGR_PLLN_Pos) | pllp_bits | pll_src |
                       (pllq_clamped << RCC_PLLCFGR_PLLQ_Pos);

        RCC->CR |= RCC_CR_PLLON;
        if (!s_wait_mask(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL, "PLL lock timeout",
                          "ws_rcc.init");
        }

        if (plan.overdrive) {
            PWR->CR1 |= PWR_CR1_ODEN;
            if (!s_wait_mask(&PWR->CSR1, PWR_CSR1_ODRDY, PWR_CSR1_ODRDY)) {
                return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                              "Over-Drive enable timeout", "ws_rcc.init");
            }
            PWR->CR1 |= PWR_CR1_ODSWEN;
            if (!s_wait_mask(&PWR->CSR1, PWR_CSR1_ODSWRDY, PWR_CSR1_ODSWRDY)) {
                return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                              "Over-Drive switch timeout", "ws_rcc.init");
            }
        }

        /* Set flash wait states and ART accelerator BEFORE raising SYSCLK (RM0385 §3.4.2). */
        FLASH->ACR = (plan.flash_ws & FLASH_ACR_LATENCY_Msk) | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;
        if (!s_wait_mask(&FLASH->ACR, FLASH_ACR_LATENCY_Msk, plan.flash_ws)) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL,
                          "FLASH latency verify timeout", "ws_rcc.init");
        }

        /* Program AHB/APB prescalers BEFORE switching SYSCLK to PLL. */
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                    RCC_CFGR_HPRE_DIV1 | ppre1 | ppre2;

        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
        if (!s_wait_mask(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL)) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_TIMEOUT, WS_SEVERITY_FATAL, "PLL switch timeout",
                          "ws_rcc.init");
        }
    }

    s_sysclk_hz = cfg->sysclk_hz;
    s_apb1_hz = cfg->apb1_hz;
    s_apb2_hz = cfg->apb2_hz;
    s_apb1_div = plan.apb1_div;
    SystemCoreClock = cfg->sysclk_hz;

    return WS_ERR_OK(ws_rcc_error_t);
}

ws_rcc_error_t ws_rcc_enable_gpio_port(ws_gpio_port_t port)
{
    if ((uint32_t)port > (uint32_t)WS_GPIO_PORT_K) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_PORT, WS_SEVERITY_RECOVERABLE,
                      "GPIO port out of range", "ws_rcc.enable_gpio_port");
    }

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN << (uint32_t)port);
    {
        volatile uint32_t rd = RCC->AHB1ENR;
        (void)rd;
    }
    return WS_ERR_OK(ws_rcc_error_t);
}

ws_rcc_error_t ws_rcc_enable_periph(ws_rcc_periph_t periph)
{
    switch (periph) {
    case WS_RCC_PERIPH_TIM2:
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        {
            volatile uint32_t rd = RCC->APB1ENR;
            (void)rd;
        }
        return WS_ERR_OK(ws_rcc_error_t);
    case WS_RCC_PERIPH_USART1:
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        {
            volatile uint32_t rd = RCC->APB2ENR;
            (void)rd;
        }
        return WS_ERR_OK(ws_rcc_error_t);
    case WS_RCC_PERIPH_USART6:
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        {
            volatile uint32_t rd = RCC->APB2ENR;
            (void)rd;
        }
        return WS_ERR_OK(ws_rcc_error_t);
    default:
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_PERIPH, WS_SEVERITY_RECOVERABLE,
                      "peripheral out of range", "ws_rcc.enable_periph");
    }
}

uint32_t ws_rcc_sysclk_hz(void)
{
    return s_sysclk_hz;
}

uint32_t ws_rcc_apb1_hz(void)
{
    return s_apb1_hz;
}

uint32_t ws_rcc_apb2_hz(void)
{
    return s_apb2_hz;
}

uint32_t ws_rcc_apb1_tim_hz(void)
{
    return (s_apb1_div == 1u) ? s_apb1_hz : (s_apb1_hz * 2u);
}
