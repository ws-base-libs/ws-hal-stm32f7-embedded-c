# ws-hal-stm32f7-embedded-c

STM32F7 per-peripheral HAL base library (`embedded_system_spec.md` §14, `embedded_c_tech_spec.md` §11.1).

Register access lives **only** in `src/*.c` over CMSIS register definitions (`STM32F746xx`, verified against **DS10916 Rev 5**, **RM0385**, and **UM1907**). Every fallible API returns a `WS_ERROR_TYPE` value from `<ws/error/error.h>`.

## Peripherals & API reference

| Header | Target | Summary |
|---|---|---|
| `<ws/hal/stm32f7/rcc.h>` | `src/rcc.c` / `fake/rcc_fake.c` | Full clock-tree setup (HSI direct or HSE/HSI → PLL, Voltage Scale 1, Over-Drive >180 MHz, Flash wait states + ART, APB1/APB2 prescalers, `SystemCoreClock`), GPIO/peripheral clock enables, bus clock queries |
| `<ws/hal/stm32f7/gpio.h>` | `src/gpio.c` / `fake/gpio_fake.c` | Includes `rcc.h`; configures `OTYPER`/`OSPEEDR`/`PUPDR`/`AFR` before `MODER` (glitch-free AF switch); `ws_gpio_init/write/read/toggle` |
| `<ws/hal/stm32f7/timebase.h>` | `src/timebase.c` / `fake/timebase_fake.c` | Includes `rcc.h`; 64-bit monotonic µs clock on **32-bit TIM2** (`TIM2_IRQHandler`, zero conflict with FreeRTOS `SysTick_Handler`) with wrap-race-free `ws_time_us()` and delays |
| `<ws/hal/stm32f7/uart.h>` | `src/uart.c` / `fake/uart_fake.c` | Includes `gpio.h` + `timebase.h`; multi-instance `ws_uart_t` (`WS_UART_1`, `WS_UART_6`), clock via `rcc.h`, rounded BRR, bounded `ws_time_us()` TX timeouts, typed `WS_UART_NO_DATA` RX |

## Examples

Runnable host-fake examples live under `examples/`: `rcc_clocks.c`, `gpio_blink.c`, `timebase_delay.c`, `uart_hello.c`.

## Host fake test seams (`WS_HAL_STM32F7_HOST_FAKE`)

- **RCC:** `ws_rcc_fake_reset`, `ws_rcc_fake_port_enabled`, `ws_rcc_fake_port_enable_count`, `ws_rcc_fake_periph_enabled`, `ws_rcc_fake_get_state`
- **GPIO:** `ws_gpio_fake_reset`, `ws_gpio_fake_inject_read`, `ws_gpio_fake_get_pin_state`
- **Timebase:** `ws_timebase_fake_reset`, `ws_timebase_fake_set_us`, `ws_timebase_fake_advance_us`
- **UART:** `ws_uart_fake_reset`, `ws_uart_fake_take_output`, `ws_uart_fake_inject_rx`, `ws_uart_fake_set_tx_stall`

## Build & test (standalone)

```sh
cmake -S . -B build -G Ninja -DWS_ERROR_DIR=/path/to/ws-error-embedded-c
cmake --build build && ctest --test-dir build --output-on-failure
```

Target cross-check compiles `src/*.c` with `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard` whenever `arm-none-eabi-gcc` and CMSIS headers (`CMSIS_INCLUDE_DIRS` or `$WS_ROOT/ws-libs/`) are present.

## Quirks & errata notes

- **FPU precision:** STM32F745/F746 carries a **single-precision** FPU (`-mfpu=fpv5-sp-d16`, DS10916 Rev 5); never compile with `-mfpu=fpv5-d16`.
- **STM32F746G-DISCO HSE:** X2 is an active 25 MHz oscillator driving `PH0/OSC_IN` (UM1907) — set `.hse_hz = 25000000u, .hse_bypass = true`.
- **No SysTick squat:** `timebase.c` owns `TIM2_IRQHandler`, leaving `SysTick_Handler` free for FreeRTOS.
- **No direct `RCC->` pokes outside `rcc.c`:** `gpio.c`, `timebase.c`, and `uart.c` enable their clocks strictly through `rcc.h`.
