/**
 * @file rcc_fake.c
 * @brief ws-hal-stm32f7 RCC — HOST FAKE (mirrors src/rcc.c validation & derivation).
 */
#include <string.h>

#include <ws/hal/stm32f7/rcc.h>

#define WS_RCC_HSI_HZ (16000000UL)
#define WS_RCC_HSE_MIN_HZ (4000000UL)
#define WS_RCC_HSE_MAX_HZ (26000000UL)
#define WS_RCC_SYSCLK_MAX_HZ (216000000UL)
#define WS_RCC_APB1_MAX_HZ (54000000UL)
#define WS_RCC_APB2_MAX_HZ (108000000UL)
#define WS_RCC_OVERDRIVE_HZ (180000000UL)

static const uint32_t k_apb_divs[] = { 1u, 2u, 4u, 8u, 16u };

static struct {
    bool inited;
    ws_rcc_fake_state_t st;
    bool port_enabled[WS_GPIO_PORT_COUNT];
    uint32_t port_enable_count[WS_GPIO_PORT_COUNT];
    bool periph_enabled[WS_RCC_PERIPH_COUNT];
} s_rcc = {
    .st =
        {
            .sysclk_hz = WS_RCC_HSI_HZ,
            .apb1_hz = WS_RCC_HSI_HZ,
            .apb2_hz = WS_RCC_HSI_HZ,
            .apb1_prescaler = 1u,
            .apb2_prescaler = 1u,
        },
};

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

ws_rcc_error_t ws_rcc_init(const ws_rcc_cfg_t *cfg)
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

    uint32_t pll_m = 0u;
    uint32_t pll_n = 0u;
    uint32_t pll_p = 0u;
    if (!(cfg->hse_hz == 0u && cfg->sysclk_hz == WS_RCC_HSI_HZ)) {
        const uint32_t src_hz = (cfg->hse_hz != 0u) ? cfg->hse_hz : WS_RCC_HSI_HZ;
        pll_m = src_hz / 1000000UL;
        if (pll_m < 2u || pll_m > 63u) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL, "PLLM out of range",
                          "ws_rcc.init");
        }
        static const uint32_t k_pllp[] = { 2u, 4u, 6u, 8u };
        for (uint32_t i = 0u; i < 4u; ++i) {
            const uint64_t vco = (uint64_t)cfg->sysclk_hz * (uint64_t)k_pllp[i];
            if ((vco % 1000000ULL) == 0ULL) {
                const uint64_t n = vco / 1000000ULL;
                if (n >= 100ULL && n <= 432ULL) {
                    pll_n = (uint32_t)n;
                    pll_p = k_pllp[i];
                    break;
                }
            }
        }
        if (pll_n == 0u) {
            return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_FREQ, WS_SEVERITY_FATAL,
                          "cannot derive exact STM32F746 PLL M/N/P", "ws_rcc.init");
        }
    }

    s_rcc.inited = true;
    s_rcc.st.hse_hz = cfg->hse_hz;
    s_rcc.st.hse_bypass = cfg->hse_bypass;
    s_rcc.st.sysclk_hz = cfg->sysclk_hz;
    s_rcc.st.apb1_hz = cfg->apb1_hz;
    s_rcc.st.apb2_hz = cfg->apb2_hz;
    s_rcc.st.apb1_prescaler = d1;
    s_rcc.st.apb2_prescaler = d2;
    s_rcc.st.pll_m = pll_m;
    s_rcc.st.pll_n = pll_n;
    s_rcc.st.pll_p = pll_p;
    s_rcc.st.flash_ws = (cfg->sysclk_hz - 1u) / 30000000UL;
    s_rcc.st.overdrive = (cfg->sysclk_hz > WS_RCC_OVERDRIVE_HZ);
    return WS_ERR_OK(ws_rcc_error_t);
}

ws_rcc_error_t ws_rcc_enable_gpio_port(ws_gpio_port_t port)
{
    if ((uint32_t)port > (uint32_t)WS_GPIO_PORT_K) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_PORT, WS_SEVERITY_RECOVERABLE,
                      "GPIO port out of range", "ws_rcc.enable_gpio_port");
    }
    if (!s_rcc.port_enabled[(unsigned)port]) {
        s_rcc.port_enabled[(unsigned)port] = true;
        s_rcc.port_enable_count[(unsigned)port] += 1u;
    }
    return WS_ERR_OK(ws_rcc_error_t);
}

ws_rcc_error_t ws_rcc_enable_periph(ws_rcc_periph_t periph)
{
    if ((unsigned)periph >= WS_RCC_PERIPH_COUNT) {
        return WS_ERR(ws_rcc_error_t, WS_RCC_BAD_PERIPH, WS_SEVERITY_RECOVERABLE,
                      "peripheral out of range", "ws_rcc.enable_periph");
    }
    s_rcc.periph_enabled[(unsigned)periph] = true;
    return WS_ERR_OK(ws_rcc_error_t);
}

uint32_t ws_rcc_sysclk_hz(void)
{
    return s_rcc.st.sysclk_hz;
}

uint32_t ws_rcc_apb1_hz(void)
{
    return s_rcc.st.apb1_hz;
}

uint32_t ws_rcc_apb2_hz(void)
{
    return s_rcc.st.apb2_hz;
}

uint32_t ws_rcc_apb1_tim_hz(void)
{
    return (s_rcc.st.apb1_prescaler == 1u) ? s_rcc.st.apb1_hz : (s_rcc.st.apb1_hz * 2u);
}

void ws_rcc_fake_reset(void)
{
    (void)memset(&s_rcc, 0, sizeof(s_rcc));
    s_rcc.st.sysclk_hz = WS_RCC_HSI_HZ;
    s_rcc.st.apb1_hz = WS_RCC_HSI_HZ;
    s_rcc.st.apb2_hz = WS_RCC_HSI_HZ;
    s_rcc.st.apb1_prescaler = 1u;
    s_rcc.st.apb2_prescaler = 1u;
}

bool ws_rcc_fake_port_enabled(ws_gpio_port_t port)
{
    return ((uint32_t)port <= (uint32_t)WS_GPIO_PORT_K) && s_rcc.port_enabled[(unsigned)port];
}

uint32_t ws_rcc_fake_port_enable_count(ws_gpio_port_t port)
{
    return ((uint32_t)port <= (uint32_t)WS_GPIO_PORT_K) ? s_rcc.port_enable_count[(unsigned)port]
                                                        : 0u;
}

bool ws_rcc_fake_periph_enabled(ws_rcc_periph_t periph)
{
    return ((unsigned)periph < WS_RCC_PERIPH_COUNT) && s_rcc.periph_enabled[(unsigned)periph];
}

void ws_rcc_fake_get_state(ws_rcc_fake_state_t *out)
{
    if (out != NULL) {
        *out = s_rcc.st;
    }
}
