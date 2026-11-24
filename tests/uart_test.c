/**
 * @file uart_test.c
 * @brief Standalone host tests for ws-hal-stm32f7 UART (multi-instance + timeouts).
 */
#include <stdio.h>
#include <string.h>

#include <ws/hal/stm32f7/uart.h>

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
    ws_timebase_fake_reset();
    ws_uart_fake_reset();

    const ws_uart_cfg_t cfg1 = {
        .instance = WS_UART_1,
        .baud = 115200u,
        .tx_pin = { .port = WS_GPIO_PORT_A, .pin = 9u },
        .rx_pin = { .port = WS_GPIO_PORT_B, .pin = 7u },
        .af = 7u,
    };
    const ws_uart_cfg_t cfg6 = {
        .instance = WS_UART_6,
        .baud = 115200u,
        .tx_pin = { .port = WS_GPIO_PORT_C, .pin = 6u },
        .rx_pin = { .port = WS_GPIO_PORT_C, .pin = 7u },
        .af = 8u,
    };

    ws_uart_t u1 = { 0 };
    ws_uart_t u6 = { 0 };
    check(WS_OK(ws_uart_init(&u1, &cfg1)), "USART1 init ok");
    check(WS_OK(ws_uart_init(&u6, &cfg6)), "USART6 init ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_USART1), "USART1 clock enabled via rcc");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_USART6), "USART6 clock enabled via rcc");

    /* Write & capture per instance independently. */
    size_t n1 = 0u;
    size_t n6 = 0u;
    check(WS_OK(ws_uart_write_str(&u1, "hello1\n", 100u, &n1)) && n1 == 7u, "USART1 write_str ok");
    check(WS_OK(ws_uart_write_str(&u6, "hello6\n", 100u, &n6)) && n6 == 7u, "USART6 write_str ok");

    uint8_t cap[32] = { 0 };
    size_t got = ws_uart_fake_take_output(WS_UART_1, cap, sizeof(cap));
    check(got == 7u && memcmp(cap, "hello1\n", 7u) == 0, "USART1 output isolated");
    got = ws_uart_fake_take_output(WS_UART_6, cap, sizeof(cap));
    check(got == 7u && memcmp(cap, "hello6\n", 7u) == 0, "USART6 output isolated");

    /* Bounded timeout on TX stall. */
    ws_uart_fake_set_tx_stall(WS_UART_1, true);
    const uint64_t t0 = ws_time_us();
    ws_uart_error_t terr = ws_uart_write_str(&u1, "stall", 50u, &n1);
    check(WS_FAILED(terr) && terr.code == WS_UART_TIMEOUT && n1 == 0u, "TX stall returns TIMEOUT");
    check((ws_time_us() - t0) == 50000ULL, "TX stall consumed 50 ms budget");
    ws_uart_fake_set_tx_stall(WS_UART_1, false);

    /* Non-blocking RX returns typed WS_UART_NO_DATA when empty, WS_UART_OK on data. */
    uint8_t rx_byte = 0u;
    check(ws_uart_read_byte(&u1, &rx_byte).code == WS_UART_NO_DATA, "empty RX -> WS_UART_NO_DATA");
    const uint8_t sample[] = { 'O', 'K' };
    ws_uart_fake_inject_rx(WS_UART_1, sample, sizeof(sample));
    check(WS_OK(ws_uart_read_byte(&u1, &rx_byte)) && rx_byte == 'O', "RX byte 1 ok");
    check(WS_OK(ws_uart_read_byte(&u1, &rx_byte)) && rx_byte == 'K', "RX byte 2 ok");
    check(ws_uart_read_byte(&u1, &rx_byte).code == WS_UART_NO_DATA, "RX drained -> NO_DATA");

    /* Overrun (ORE) reports WS_UART_OVERRUN, clears latch, and preserves queued byte. */
    const uint8_t post_ore[] = { 'Z' };
    ws_uart_fake_inject_rx(WS_UART_1, post_ore, sizeof(post_ore));
    ws_uart_fake_inject_overrun(WS_UART_1);
    check(ws_uart_read_byte(&u1, &rx_byte).code == WS_UART_OVERRUN, "ORE reports WS_UART_OVERRUN");
    check(WS_OK(ws_uart_read_byte(&u1, &rx_byte)) && rx_byte == 'Z',
          "RX unblocked after ORE clear");

    /* Framing/noise/parity error clears latch and discards corrupted byte. */
    const uint8_t bad_and_good[] = { 0xFFu, 'A' };
    ws_uart_fake_inject_rx(WS_UART_1, bad_and_good, sizeof(bad_and_good));
    ws_uart_fake_inject_rx_error(WS_UART_1);
    check(ws_uart_read_byte(&u1, &rx_byte).code == WS_UART_RX_ERROR,
          "FE/NE/PE reports WS_UART_RX_ERROR");
    check(WS_OK(ws_uart_read_byte(&u1, &rx_byte)) && rx_byte == 'A',
          "subsequent valid byte readable after RX error");

    /* Validation rejections. */
    ws_uart_cfg_t bad = cfg1;
    bad.baud = 0u;
    check(ws_uart_init(&u1, &bad).code == WS_UART_BAD_CONFIG, "baud 0 rejected");
    ws_uart_t uninit = { 0 };
    check(ws_uart_read_byte(&uninit, &rx_byte).code == WS_UART_BAD_CONFIG,
          "uninit handle rejected");

    printf("uart_test: %d checks, %d failures: %s\n", s_checks, s_failures,
           s_failures == 0 ? "PASS" : "FAIL");
    return s_failures == 0 ? 0 : 1;
}
