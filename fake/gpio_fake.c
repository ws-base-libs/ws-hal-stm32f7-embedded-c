/**
 * @file gpio_fake.c
 * @brief ws-hal-stm32f7 GPIO — HOST FAKE.
 */
#include <string.h>

#include <ws/hal/stm32f7/gpio.h>

static struct {
    ws_gpio_fake_pin_state_t pins[WS_GPIO_PORT_COUNT][WS_GPIO_PIN_COUNT];
    bool injected[WS_GPIO_PORT_COUNT][WS_GPIO_PIN_COUNT];
    bool injected_level[WS_GPIO_PORT_COUNT][WS_GPIO_PIN_COUNT];
} s_gpio;

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

    ws_gpio_fake_pin_state_t *st = &s_gpio.pins[(unsigned)pin->port][pin->pin];
    st->configured = true;
    st->mode = cfg->mode;
    st->pull = cfg->pull;
    st->speed = cfg->speed;
    st->af = cfg->af;
    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_write(ws_gpio_t *pin, bool level)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }
    s_gpio.pins[(unsigned)pin->port][pin->pin].level = level;
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
    if (s_gpio.injected[(unsigned)pin->port][pin->pin]) {
        *out = s_gpio.injected_level[(unsigned)pin->port][pin->pin];
    } else {
        *out = s_gpio.pins[(unsigned)pin->port][pin->pin].level;
    }
    return WS_ERR_OK(ws_gpio_error_t);
}

ws_gpio_error_t ws_gpio_toggle(ws_gpio_t *pin)
{
    ws_gpio_error_t err = s_check_handle(pin);
    if (WS_FAILED(err)) {
        return err;
    }
    ws_gpio_fake_pin_state_t *st = &s_gpio.pins[(unsigned)pin->port][pin->pin];
    st->level = !st->level;
    return WS_ERR_OK(ws_gpio_error_t);
}

void ws_gpio_fake_reset(void)
{
    (void)memset(&s_gpio, 0, sizeof(s_gpio));
}

void ws_gpio_fake_inject_read(ws_gpio_port_t port, uint8_t pin, bool level)
{
    if ((uint32_t)port <= (uint32_t)WS_GPIO_PORT_K && pin < WS_GPIO_PIN_COUNT) {
        s_gpio.injected[(unsigned)port][pin] = true;
        s_gpio.injected_level[(unsigned)port][pin] = level;
    }
}

bool ws_gpio_fake_get_pin_state(ws_gpio_port_t port, uint8_t pin, ws_gpio_fake_pin_state_t *out)
{
    if (out == NULL || (uint32_t)port > (uint32_t)WS_GPIO_PORT_K || pin >= WS_GPIO_PIN_COUNT) {
        return false;
    }
    *out = s_gpio.pins[(unsigned)port][pin];
    return true;
}
