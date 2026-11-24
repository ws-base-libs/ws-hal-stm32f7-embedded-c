/**
 * @file spi_test.c
 * @brief Standalone host tests for ws-hal-stm32f7 SPI (transfer, busy, timeout, error paths).
 */
#include <stdio.h>
#include <string.h>

#include <ws/hal/stm32f7/spi.h>

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
    ws_spi_fake_reset();

    /* Configure STM32F746G-DISCO clock tree (APB1=54 MHz, APB2=108 MHz). */
    const ws_rcc_cfg_t rcc_cfg = {
        .hse_hz = 25000000u,
        .hse_bypass = true,
        .sysclk_hz = 216000000u,
        .apb1_hz = 54000000u,
        .apb2_hz = 108000000u,
    };
    check(WS_OK(ws_rcc_init(&rcc_cfg)), "RCC 216 MHz init ok");

    /* SPI2 read-only config matching MAX31855 pin map: PI1(SCK), PB14(MISO), AF5, <= 5 MHz. */
    const ws_spi_cfg_t spi2_cfg = {
        .instance = WS_SPI_2,
        .max_hz = 5000000u,
        .mode = WS_SPI_MODE_0,
        .sck_pin = { .port = WS_GPIO_PORT_I, .pin = 1u },
        .miso_pin = { .port = WS_GPIO_PORT_B, .pin = 14u },
        .mosi_pin = { .port = WS_GPIO_PORT_B, .pin = 15u },
        .use_miso = true,
        .use_mosi = false,
        .af = 5u,
    };

    ws_spi_t spi2 = { 0 };
    check(WS_OK(ws_spi_init(&spi2, &spi2_cfg)), "SPI2 init ok");
    check(ws_rcc_fake_periph_enabled(WS_RCC_PERIPH_SPI2), "SPI2 clock enabled via rcc");
    /* 54 MHz / 16 = 3.375 MHz <= 5 MHz (54 MHz / 8 = 6.75 MHz > 5 MHz). */
    check(spi2.actual_hz == 3375000u && ws_spi_fake_actual_hz(WS_SPI_2) == 3375000u,
          "SPI2 prescaler derived 54 MHz / 16 = 3.375 MHz");

    /* Verify PI1 & PB14 configured as AF5 and PB15 left untouched (use_mosi = false). */
    ws_gpio_fake_pin_state_t st = { 0 };
    check(ws_gpio_fake_get_pin_state(WS_GPIO_PORT_I, 1u, &st) && st.configured &&
              st.mode == WS_GPIO_MODE_ALTERNATE && st.af == 5u,
          "PI1 configured as AF5 SCK");
    check(ws_gpio_fake_get_pin_state(WS_GPIO_PORT_B, 14u, &st) && st.configured &&
              st.mode == WS_GPIO_MODE_ALTERNATE && st.af == 5u,
          "PB14 configured as AF5 MISO");
    check(ws_gpio_fake_get_pin_state(WS_GPIO_PORT_B, 15u, &st) && !st.configured,
          "PB15 untouched when use_mosi=false");

    /* Scripted 4-byte MAX31855-like read. */
    const uint8_t frame[4] = { 0x01u, 0x90u, 0x19u, 0x00u };
    ws_spi_fake_queue_rx(WS_SPI_2, frame, sizeof(frame));
    uint8_t rx[4] = { 0 };
    check(WS_OK(ws_spi_read(&spi2, rx, sizeof(rx), 10u)), "SPI2 4-byte read ok");
    check(memcmp(rx, frame, sizeof(frame)) == 0, "SPI2 received scripted RX bytes");
    check(ws_spi_fake_transfer_count(WS_SPI_2) == 1u, "SPI2 transfer count incremented");

    uint8_t tx_cap[8] = { 0xAAu, 0xAAu, 0xAAu, 0xAAu };
    check(ws_spi_fake_take_tx(WS_SPI_2, tx_cap, sizeof(tx_cap)) == 4u && tx_cap[0] == 0x00u &&
              tx_cap[3] == 0x00u,
          "ws_spi_read transmitted 0x00 dummy bytes");

    /* Full-duplex transfer on SPI1 (APB2 = 108 MHz -> /16 = 6.75 MHz <= 10 MHz). */
    const ws_spi_cfg_t spi1_cfg = {
        .instance = WS_SPI_1,
        .max_hz = 10000000u,
        .mode = WS_SPI_MODE_3,
        .sck_pin = { .port = WS_GPIO_PORT_A, .pin = 5u },
        .miso_pin = { .port = WS_GPIO_PORT_A, .pin = 6u },
        .mosi_pin = { .port = WS_GPIO_PORT_A, .pin = 7u },
        .use_miso = true,
        .use_mosi = true,
        .af = 5u,
    };
    ws_spi_t spi1 = { 0 };
    check(WS_OK(ws_spi_init(&spi1, &spi1_cfg)), "SPI1 init ok");
    check(spi1.actual_hz == 6750000u, "SPI1 prescaler derived 108 MHz / 16 = 6.75 MHz");
    const uint8_t tx_cmd[3] = { 0x9Fu, 0x00u, 0x00u };
    const uint8_t rx_id[3] = { 0x00u, 0xEFu, 0x40u };
    ws_spi_fake_queue_rx(WS_SPI_1, rx_id, sizeof(rx_id));
    uint8_t rx_out[3] = { 0 };
    check(WS_OK(ws_spi_transfer(&spi1, tx_cmd, rx_out, sizeof(tx_cmd), 10u)),
          "SPI1 full-duplex transfer ok");
    check(memcmp(rx_out, rx_id, sizeof(rx_id)) == 0, "SPI1 RX isolated from SPI2");
    check(ws_spi_fake_take_tx(WS_SPI_1, tx_cap, sizeof(tx_cap)) == 3u &&
              memcmp(tx_cap, tx_cmd, sizeof(tx_cmd)) == 0,
          "SPI1 TX captured");

    /* Write-only convenience helper. */
    check(WS_OK(ws_spi_write(&spi1, tx_cmd, 2u, 10u)), "SPI1 write-only ok");
    check(ws_spi_fake_take_tx(WS_SPI_1, tx_cap, sizeof(tx_cap)) == 2u, "SPI1 write TX captured");

    /* Busy fault path. */
    ws_spi_fake_set_busy(WS_SPI_2, true);
    check(ws_spi_read(&spi2, rx, sizeof(rx), 10u).code == WS_SPI_BUSY,
          "busy SPI returns WS_SPI_BUSY");
    ws_spi_fake_set_busy(WS_SPI_2, false);

    /* Timeout fault path advances simulated clock by timeout_ms. */
    ws_spi_fake_set_timeout(WS_SPI_2, true);
    const uint64_t t0 = ws_time_us();
    check(ws_spi_read(&spi2, rx, sizeof(rx), 25u).code == WS_SPI_TIMEOUT,
          "timeout SPI returns WS_SPI_TIMEOUT");
    check((ws_time_us() - t0) == 25000ULL, "timeout consumed 25 ms budget");
    ws_spi_fake_set_timeout(WS_SPI_2, false);

    /* Hardware bus error path (OVR/MODF/FRE). */
    ws_spi_fake_set_bus_error(WS_SPI_2, true);
    check(ws_spi_read(&spi2, rx, sizeof(rx), 10u).code == WS_SPI_BUS_ERROR,
          "bus error returns WS_SPI_BUS_ERROR");
    ws_spi_fake_set_bus_error(WS_SPI_2, false);

    /* Validation rejections. */
    ws_spi_cfg_t bad = spi2_cfg;
    bad.max_hz = 100u; /* 54 MHz / 256 = 210.9 kHz > 100 Hz -> underivable */
    check(ws_spi_init(&spi2, &bad).code == WS_SPI_BAD_CONFIG, "underivable low max_hz rejected");
    bad = spi2_cfg;
    bad.use_miso = false;
    bad.use_mosi = false;
    check(ws_spi_init(&spi2, &bad).code == WS_SPI_BAD_CONFIG, "neither MISO nor MOSI rejected");
    ws_spi_t uninit = { 0 };
    check(ws_spi_read(&uninit, rx, sizeof(rx), 10u).code == WS_SPI_BAD_CONFIG,
          "uninit handle rejected");
    check(ws_spi_transfer(&spi2, NULL, NULL, 4u, 10u).code == WS_SPI_BAD_CONFIG,
          "both TX and RX NULL with len>0 rejected");
    check(WS_OK(ws_spi_transfer(&spi2, NULL, NULL, 0u, 10u)), "zero-length transfer is no-op OK");

    printf("spi_test: %d checks, %d failures: %s\n", s_checks, s_failures,
           s_failures == 0 ? "PASS" : "FAIL");
    return s_failures == 0 ? 0 : 1;
}
