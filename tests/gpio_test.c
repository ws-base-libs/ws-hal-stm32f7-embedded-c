/**
 * @file gpio_test.c
 * @brief Standalone host tests for ws-hal-stm32f7 GPIO.
 */
#include <stdio.h>

#include <ws/hal/stm32f7/gpio.h>

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
    ws_gpio_fake_reset();

    ws_gpio_t led = { .port = WS_GPIO_PORT_I, .pin = 1u };
    const ws_gpio_cfg_t cfg = {
        .mode = WS_GPIO_MODE_OUTPUT_PP,
        .pull = WS_GPIO_PULL_NONE,
        .speed = WS_GPIO_SPEED_LOW,
        .af = 0u,
    };

    check(WS_OK(ws_gpio_init(&led, &cfg)), "gpio init ok");
    check(ws_rcc_fake_port_enabled(WS_GPIO_PORT_I), "gpio init enabled GPIOI clock");

    check(WS_OK(ws_gpio_write(&led, true)), "write high ok");
    bool level = false;
    check(WS_OK(ws_gpio_read(&led, &level)) && level, "read back high");
    check(WS_OK(ws_gpio_toggle(&led)), "toggle ok");
    check(WS_OK(ws_gpio_read(&led, &level)) && !level, "toggled to low");

    ws_gpio_fake_inject_read(WS_GPIO_PORT_I, 1u, true);
    check(WS_OK(ws_gpio_read(&led, &level)) && level, "injected read wins");

    ws_gpio_t bad_pin = { .port = WS_GPIO_PORT_A, .pin = 16u };
    check(ws_gpio_init(&bad_pin, &cfg).code == WS_GPIO_BAD_PIN, "pin 16 rejected");
    ws_gpio_cfg_t bad_cfg = cfg;
    bad_cfg.af = 16u;
    check(ws_gpio_init(&led, &bad_cfg).code == WS_GPIO_BAD_CONFIG, "af 16 rejected");
    check(ws_gpio_read(&led, NULL).code == WS_GPIO_BAD_CONFIG, "NULL out rejected");

    printf("gpio_test: %d checks, %d failures: %s\n", s_checks, s_failures,
           s_failures == 0 ? "PASS" : "FAIL");
    return s_failures == 0 ? 0 : 1;
}
