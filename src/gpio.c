/**
 * @file gpio.c
 * @brief ws-hal-stm32f7 GPIO — TARGET implementation over CMSIS registers.
 *
 * Verified against STM32F746xx datasheet DS10916 Rev 5 and RM0385.
 * Writes OTYPER/OSPEEDR/PUPDR/AFR BEFORE MODER so alternate-function pins
 * never glitch onto AF0 during configuration.
 */
#if !defined(STM32F746xx)
#error "ws-hal-stm32f7 target build requires STM32F746xx"
#endif

#include "stm32f746xx.h"

#include <stddef.h>

#include <ws/hal/stm32f7/gpio.h>

static GPIO_TypeDef *const k_gpio_ports[WS_GPIO_PORT_COUNT] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK,
};

static ws_gpio_error_t s_check_handle(const ws_gpio_t *pin)
{
    if (pin == NULL || (uint32_t)pin->port > (uint32_t)WS_GPIO_PORT_K ||
        pin->pin >= WS_GPIO_PIN_COUNT) {
        return WS_ERR(ws_gpio_error_t, WS_GPIO_BAD_PIN, WS_SEVERITY_RECOVERABLE,
                      "GPIO port or pin out of range", "ws_gpio");
    }
    return WS_ERR_OK(ws_gpio_error_t);
}

static ws_gpio_error_t s_check_cfg(const ws_gpio_cfg_t *cfg)
{
    if (cfg == NULL || (unsigned)cfg->mode > (unsigned)WS_GPIO_MODE_ANALOG ||
        (unsigned)cfg->pull > (unsigned)WS_GPIO_PULL_DOWN ||
        (unsigned)cfg->speed > (unsigned)WS_GPIO_SPEED_VERY_HIGH || cfg->af > 15u) {
        return WS_ERR(ws_gpio_error_t, WS_GPIO_BAD_CONFIG, WS_SEVERITY_RECOVERABLE,
                      "invalid GPIO config", "ws_gpio.init");
    }
    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_init(ws_gpio_t *pin, const ws_gpio_cfg_t *cfg)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }
    err = s_check_cfg(cfg);
    if (WS_FAILED(err)) {
        return err;
    }

    if (WS_FAILED(ws_rcc_enable_gpio_port(pin->port))) {
        return WS_ERR(ws_gpio_error_t, WS_GPIO_BAD_PIN, WS_SEVERITY_RECOVERABLE,
                      "RCC failed to enable GPIO port", "ws_gpio.init");
    }

    GPIO_TypeDef *gpio = k_gpio_ports[(unsigned)pin->port];
    const uint32_t pin2 = (uint32_t)pin->pin * 2u;
    const uint32_t mask2 = 3u << pin2;
    const uint32_t mask1 = 1u << pin->pin;

    /* Program OTYPER, OSPEEDR, PUPDR, and AFR BEFORE MODER to avoid AF0 glitch. */
    if (cfg->mode == WS_GPIO_MODE_OUTPUT_OD) {
        gpio->OTYPER |= mask1;
    } else {
        gpio->OTYPER &= ~mask1;
    }

    gpio->OSPEEDR = (gpio->OSPEEDR & ~mask2) | ((uint32_t)cfg->speed << pin2);
    gpio->PUPDR = (gpio->PUPDR & ~mask2) | ((uint32_t)cfg->pull << pin2);

    if (cfg->mode == WS_GPIO_MODE_ALTERNATE) {
        const uint32_t shift = ((uint32_t)pin->pin & 7u) * 4u;
        __IO uint32_t *afr = &gpio->AFR[(uint32_t)pin->pin >> 3u];
        *afr = (*afr & ~(15u << shift)) | ((uint32_t)cfg->af << shift);
    }

    uint32_t moder = 0u;
    switch (cfg->mode) {
    case WS_GPIO_MODE_INPUT:
        moder = 0u;
        break;
    case WS_GPIO_MODE_OUTPUT_PP:
    case WS_GPIO_MODE_OUTPUT_OD:
        moder = 1u;
        break;
    case WS_GPIO_MODE_ALTERNATE:
        moder = 2u;
        break;
    case WS_GPIO_MODE_ANALOG:
    default:
        moder = 3u;
        break;
    }
    gpio->MODER = (gpio->MODER & ~mask2) | (moder << pin2);

    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_write(ws_gpio_t *pin, bool level)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }

    const uint32_t mask1 = 1u << pin->pin;
    k_gpio_ports[(unsigned)pin->port]->BSRR = level ? mask1 : (mask1 << 16u);
    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_read(const ws_gpio_t *pin, bool *out)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }
    if (out == NULL) {
        return WS_ERR(ws_gpio_error_t, WS_GPIO_BAD_CONFIG, WS_SEVERITY_RECOVERABLE, "NULL out",
                      "ws_gpio.read");
    }

    *out = (k_gpio_ports[(unsigned)pin->port]->IDR & (1u << pin->pin)) != 0u;
    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_toggle(ws_gpio_t *pin)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }

    const uint32_t mask1 = 1u << pin->pin;
    GPIO_TypeDef *gpio = k_gpio_ports[(unsigned)pin->port];
    gpio->BSRR = ((gpio->ODR & mask1) != 0u) ? (mask1 << 16u) : mask1;
    return WS_ERR_OK(ws_gpio_error_t);
}
