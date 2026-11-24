/**
 * @file gpio_blink.c
 * @brief Example: LED GPIO init and toggle via ws-hal-stm32f7.
 */
#include <ws/hal/stm32f7/gpio.h>

int main(void)
{
    static const ws_gpio_cfg_t k_led_cfg = {
        .mode = WS_GPIO_MODE_OUTPUT_PP,
        .pull = WS_GPIO_PULL_NONE,
        .speed = WS_GPIO_SPEED_LOW,
        .af = 0u,
    };
    ws_gpio_t led = { .port = WS_GPIO_PORT_I, .pin = 1u };
    if (WS_FAILED(ws_gpio_init(&led, &k_led_cfg))) {
        return 1;
    }
    (void)ws_gpio_write(&led, true);
    (void)ws_gpio_toggle(&led);
    return 0;
}
