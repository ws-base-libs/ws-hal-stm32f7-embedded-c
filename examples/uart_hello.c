/**
 * @file uart_hello.c
 * @brief Example: ST-Link VCP USART1 console hello with bounded timeout.
 */
#include <ws/hal/stm32f7/uart.h>

int main(void)
{
    static const ws_timebase_cfg_t k_tb_cfg = { .tim_clk_hz = 0u };
    static const ws_uart_cfg_t k_uart_cfg = {
        .instance = WS_UART_1,
        .baud = 115200u,
        .tx_pin = { .port = WS_GPIO_PORT_A, .pin = 9u },
        .rx_pin = { .port = WS_GPIO_PORT_B, .pin = 7u },
        .af = 7u,
    };
    ws_uart_t console = { 0 };
    if (WS_FAILED(ws_timebase_init(&k_tb_cfg)) || WS_FAILED(ws_uart_init(&console, &k_uart_cfg))) {
        return 1;
    }
    size_t written = 0u;
    return WS_OK(ws_uart_write_str(&console, "hello world\n", 100u, &written)) ? 0 : 1;
}
