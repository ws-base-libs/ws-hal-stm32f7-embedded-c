/**
 * @file gpio.h
 * @brief ws-hal-stm32f7 GPIO — pin configuration and I/O for STM32F7.
 *
 * Includes <ws/hal/stm32f7/rcc.h> and enables port clocks automatically.
 * Errors use WS_ERROR_TYPE() from <ws/error/error.h>.
 */
#ifndef WS_WS_HAL_STM32F7_GPIO_H
#define WS_WS_HAL_STM32F7_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include <ws/error/error.h>
#include <ws/hal/stm32f7/rcc.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_GPIO_OK = 0,
    WS_GPIO_BAD_PIN,
    WS_GPIO_BAD_CONFIG,
} ws_gpio_err_t;
WS_ERROR_TYPE(ws_gpio_err_t, ws_gpio_error_t);

#define WS_GPIO_PIN_COUNT 16u

typedef enum {
    WS_GPIO_MODE_INPUT = 0,
    WS_GPIO_MODE_OUTPUT_PP,
    WS_GPIO_MODE_OUTPUT_OD,
    WS_GPIO_MODE_ALTERNATE,
    WS_GPIO_MODE_ANALOG,
} ws_gpio_mode_t;

typedef enum {
    WS_GPIO_PULL_NONE = 0,
    WS_GPIO_PULL_UP,
    WS_GPIO_PULL_DOWN,
} ws_gpio_pull_t;

typedef enum {
    WS_GPIO_SPEED_LOW = 0,
    WS_GPIO_SPEED_MEDIUM,
    WS_GPIO_SPEED_HIGH,
    WS_GPIO_SPEED_VERY_HIGH,
} ws_gpio_speed_t;

typedef struct {
    ws_gpio_port_t port;
    uint8_t pin;
} ws_gpio_t;

typedef struct {
    ws_gpio_mode_t mode;
    ws_gpio_pull_t pull;
    ws_gpio_speed_t speed;
    uint8_t af;
} ws_gpio_cfg_t;

ws_gpio_error_t ws_gpio_init(ws_gpio_t *pin, const ws_gpio_cfg_t *cfg);
ws_gpio_error_t ws_gpio_write(ws_gpio_t *pin, bool level);
ws_gpio_error_t ws_gpio_read(const ws_gpio_t *pin, bool *out);
ws_gpio_error_t ws_gpio_toggle(ws_gpio_t *pin);

#if defined(WS_HAL_STM32F7_HOST_FAKE)
typedef struct {
    bool configured;
    bool level;
    ws_gpio_mode_t mode;
    ws_gpio_pull_t pull;
    ws_gpio_speed_t speed;
    uint8_t af;
} ws_gpio_fake_pin_state_t;

void ws_gpio_fake_reset(void);
void ws_gpio_fake_inject_read(ws_gpio_port_t port, uint8_t pin, bool level);
bool ws_gpio_fake_get_pin_state(ws_gpio_port_t port, uint8_t pin, ws_gpio_fake_pin_state_t *out);
#endif /* WS_HAL_STM32F7_HOST_FAKE */

#ifdef __cplusplus
}
#endif

#endif /* WS_WS_HAL_STM32F7_GPIO_H */
