/**
 * @file spi.c
 * @brief ws-hal-stm32f7 SPI — TARGET implementation (SPI1 / SPI2 over CMSIS).
 *
 * Verified against STM32F746xx RM0385 §33:
 *   - 8-bit frame mode requires CR2.DS = 0b0111 AND CR2.FRXTH = 1 so RXNE fires
 *     at 1 byte instead of 2 bytes (RM0385 §33.5.9).
 *   - Data register MUST be accessed via 8-bit pointer *(volatile uint8_t *)&SPIx->DR;
 *     a 16-bit write packs two bytes into the STM32F7 TX FIFO (RM0385 §33.5.9).
 *   - Enables peripheral clocks strictly through rcc.h and configures pins via gpio.h.
 */
#if !defined(STM32F746xx)
#error "ws-hal-stm32f7 target build requires STM32F746xx"
#endif

#include "stm32f746xx.h"

#include <stddef.h>

#include <ws/hal/stm32f7/spi.h>

#define WS_SPI_DISABLE_TIMEOUT_US (10000ULL)

static SPI_TypeDef *s_spi_regs(ws_spi_instance_t inst)
{
    switch (inst) {
    case WS_SPI_1:
        return SPI1;
    case WS_SPI_2:
        return SPI2;
    default:
        return NULL;
    }
}

static ws_rcc_periph_t s_spi_rcc_periph(ws_spi_instance_t inst)
{
    return (inst == WS_SPI_1) ? WS_RCC_PERIPH_SPI1 : WS_RCC_PERIPH_SPI2;
}

static uint32_t s_spi_pclk_hz(ws_spi_instance_t inst)
{
    return (inst == WS_SPI_1) ? ws_rcc_apb2_hz() : ws_rcc_apb1_hz();
}

static bool s_pin_valid(const ws_gpio_t *pin)
{
    return ((uint32_t)pin->port <= (uint32_t)WS_GPIO_PORT_K) && (pin->pin < WS_GPIO_PIN_COUNT);
}

static bool s_derive_br(uint32_t pclk_hz, uint32_t max_hz, uint32_t *out_br_idx,
                        uint32_t *out_actual_hz)
{
    if (pclk_hz == 0u || max_hz == 0u) {
        return false;
    }
    for (uint32_t idx = 0u; idx < 8u; ++idx) {
        const uint32_t div = 2u << idx; /* 2, 4, 8, 16, 32, 64, 128, 256 */
        const uint32_t sck_hz = pclk_hz / div;
        if (sck_hz <= max_hz) {
            *out_br_idx = idx;
            *out_actual_hz = sck_hz;
            return true;
        }
    }
    return false;
}

static ws_spi_error_t s_check_cfg(const ws_spi_cfg_t *cfg)
{
    if (cfg == NULL || cfg->max_hz == 0u || cfg->af > 15u ||
        (unsigned)cfg->instance >= WS_SPI_INSTANCE_COUNT ||
        (unsigned)cfg->mode > (unsigned)WS_SPI_MODE_3 || (!cfg->use_miso && !cfg->use_mosi) ||
        !s_pin_valid(&cfg->sck_pin) || (cfg->use_miso && !s_pin_valid(&cfg->miso_pin)) ||
        (cfg->use_mosi && !s_pin_valid(&cfg->mosi_pin))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid SPI config", "ws_spi.init");
    }
    return WS_ERR_OK(ws_spi_error_t);
}

static void s_drain_rx_and_clear_ovr(SPI_TypeDef *regs)
{
    volatile uint8_t *const dr8 = (volatile uint8_t *)&regs->DR;
    for (uint32_t i = 0u; i < 8u && (regs->SR & SPI_SR_RXNE) != 0u; ++i) {
        const uint8_t discard = *dr8;
        (void)discard;
    }
    if ((regs->SR & SPI_SR_OVR) != 0u) {
        const uint8_t discard_dr = *dr8;
        const uint32_t discard_sr = regs->SR;
        (void)discard_dr;
        (void)discard_sr;
    }
}

ws_spi_error_t ws_spi_init(ws_spi_t *spi, const ws_spi_cfg_t *cfg)
{
    if (spi == NULL) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE, "NULL spi handle",
                      "ws_spi.init");
    }
    const ws_spi_error_t chk = s_check_cfg(cfg);
    if (WS_FAILED(chk)) {
        return chk;
    }

    const uint32_t pclk_hz = s_spi_pclk_hz(cfg->instance);
    uint32_t br_idx = 0u;
    uint32_t actual_hz = 0u;
    if (!s_derive_br(pclk_hz, cfg->max_hz, &br_idx, &actual_hz)) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "underivable SPI prescaler for max_hz", "ws_spi.init");
    }

    const ws_gpio_cfg_t pin_cfg = {
        .mode = WS_GPIO_MODE_ALTERNATE,
        .pull = WS_GPIO_PULL_NONE,
        .speed = WS_GPIO_SPEED_HIGH,
        .af = cfg->af,
    };

    ws_gpio_t sck = cfg->sck_pin;
    if (WS_FAILED(ws_gpio_init(&sck, &pin_cfg))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "SPI SCK pin config failed", "ws_spi.init");
    }
    if (cfg->use_miso) {
        ws_gpio_t miso = cfg->miso_pin;
        if (WS_FAILED(ws_gpio_init(&miso, &pin_cfg))) {
            return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                          "SPI MISO pin config failed", "ws_spi.init");
        }
    }
    if (cfg->use_mosi) {
        ws_gpio_t mosi = cfg->mosi_pin;
        if (WS_FAILED(ws_gpio_init(&mosi, &pin_cfg))) {
            return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                          "SPI MOSI pin config failed", "ws_spi.init");
        }
    }

    if (WS_FAILED(ws_rcc_enable_periph(s_spi_rcc_periph(cfg->instance)))) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "RCC failed to enable SPI", "ws_spi.init");
    }

    SPI_TypeDef *regs = s_spi_regs(cfg->instance);
    if (regs == NULL) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "unknown SPI instance", "ws_spi.init");
    }

    /* Disable cleanly before reconfiguring CR1/CR2. */
    if ((regs->CR1 & SPI_CR1_SPE) != 0u) {
        const uint64_t t0 = ws_time_us();
        while ((regs->SR & SPI_SR_BSY) != 0u) {
            if ((ws_time_us() - t0) >= WS_SPI_DISABLE_TIMEOUT_US) {
                return WS_ERR(ws_spi_error_t, WS_SPI_BUSY, WS_SEVERITY_DEGRADED,
                              "SPI busy during re-init", "ws_spi.init");
            }
        }
        regs->CR1 &= ~SPI_CR1_SPE;
    }

    /* 8-bit data size (DS=0b0111) + FRXTH=1 (RXNE at 1 byte, RM0385 §33.5.9). */
    regs->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;

    uint32_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (br_idx << SPI_CR1_BR_Pos);
    if (cfg->mode == WS_SPI_MODE_2 || cfg->mode == WS_SPI_MODE_3) {
        cr1 |= SPI_CR1_CPOL;
    }
    if (cfg->mode == WS_SPI_MODE_1 || cfg->mode == WS_SPI_MODE_3) {
        cr1 |= SPI_CR1_CPHA;
    }
    regs->CR1 = cr1;
    regs->CR1 |= SPI_CR1_SPE;
    s_drain_rx_and_clear_ovr(regs);

    spi->instance = cfg->instance;
    spi->actual_hz = actual_hz;
    spi->initialized = true;
    return WS_ERR_OK(ws_spi_error_t);
}

ws_spi_error_t ws_spi_transfer(ws_spi_t *spi, const uint8_t *tx, uint8_t *rx, size_t len,
                               uint32_t timeout_ms)
{
    if (spi == NULL || !spi->initialized || (unsigned)spi->instance >= WS_SPI_INSTANCE_COUNT ||
        (len > 0u && tx == NULL && rx == NULL)) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid SPI handle or buffers", "ws_spi.transfer");
    }
    if (len == 0u) {
        return WS_ERR_OK(ws_spi_error_t);
    }

    SPI_TypeDef *regs = s_spi_regs(spi->instance);
    if (regs == NULL) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "unknown SPI instance", "ws_spi.transfer");
    }
    if ((regs->SR & SPI_SR_BSY) != 0u) {
        return WS_ERR(ws_spi_error_t, WS_SPI_BUSY, WS_SEVERITY_DEGRADED,
                      "SPI peripheral busy on transfer start", "ws_spi.transfer");
    }

    s_drain_rx_and_clear_ovr(regs);

    volatile uint8_t *const dr8 = (volatile uint8_t *)&regs->DR;
    const uint64_t start = ws_time_us();
    const uint64_t budget_us = (uint64_t)timeout_ms * 1000ULL;
    const uint32_t err_mask = SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE;

    for (size_t i = 0u; i < len; ++i) {
        while ((regs->SR & SPI_SR_TXE) == 0u) {
            if ((regs->SR & err_mask) != 0u) {
                s_drain_rx_and_clear_ovr(regs);
                return WS_ERR(ws_spi_error_t, WS_SPI_BUS_ERROR, WS_SEVERITY_DEGRADED,
                              "SPI hardware error during TXE wait", "ws_spi.transfer");
            }
            if ((ws_time_us() - start) >= budget_us) {
                return WS_ERR(ws_spi_error_t, WS_SPI_TIMEOUT, WS_SEVERITY_DEGRADED,
                              "SPI TXE wait timed out", "ws_spi.transfer");
            }
        }

        /* 8-bit write to DR so STM32F7 FIFO shifts exactly one byte (RM0385 §33.5.9). */
        *dr8 = (tx != NULL) ? tx[i] : 0x00u;

        while ((regs->SR & SPI_SR_RXNE) == 0u) {
            if ((regs->SR & err_mask) != 0u) {
                s_drain_rx_and_clear_ovr(regs);
                return WS_ERR(ws_spi_error_t, WS_SPI_BUS_ERROR, WS_SEVERITY_DEGRADED,
                              "SPI hardware error during RXNE wait", "ws_spi.transfer");
            }
            if ((ws_time_us() - start) >= budget_us) {
                return WS_ERR(ws_spi_error_t, WS_SPI_TIMEOUT, WS_SEVERITY_DEGRADED,
                              "SPI RXNE wait timed out", "ws_spi.transfer");
            }
        }

        const uint8_t in_byte = *dr8;
        if (rx != NULL) {
            rx[i] = in_byte;
        }
    }

    while ((regs->SR & SPI_SR_BSY) != 0u) {
        if ((ws_time_us() - start) >= budget_us) {
            return WS_ERR(ws_spi_error_t, WS_SPI_TIMEOUT, WS_SEVERITY_DEGRADED,
                          "SPI BSY clear wait timed out", "ws_spi.transfer");
        }
    }

    if ((regs->SR & err_mask) != 0u) {
        s_drain_rx_and_clear_ovr(regs);
        return WS_ERR(ws_spi_error_t, WS_SPI_BUS_ERROR, WS_SEVERITY_DEGRADED,
                      "SPI hardware error flagged after transfer", "ws_spi.transfer");
    }

    return WS_ERR_OK(ws_spi_error_t);
}

ws_spi_error_t ws_spi_read(ws_spi_t *spi, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
    return ws_spi_transfer(spi, NULL, rx, len, timeout_ms);
}

ws_spi_error_t ws_spi_write(ws_spi_t *spi, const uint8_t *tx, size_t len, uint32_t timeout_ms)
{
    return ws_spi_transfer(spi, tx, NULL, len, timeout_ms);
}
