/**
 * @file spi_transfer.c
 * @brief Runnable example: SPI2 read of a 4-byte sensor frame on STM32F746G-DISCO.
 */
#include <stdio.h>

#include <ws/hal/stm32f7/spi.h>

int main(void)
{
    const ws_rcc_cfg_t rcc = {
        .hse_hz = 25000000u,
        .hse_bypass = true,
        .sysclk_hz = 216000000u,
        .apb1_hz = 54000000u,
        .apb2_hz = 108000000u,
    };
    if (WS_FAILED(ws_rcc_init(&rcc))) {
        return 1;
    }

    ws_spi_t spi2 = { 0 };
    const ws_spi_cfg_t cfg = {
        .instance = WS_SPI_2,
        .max_hz = 5000000u,
        .mode = WS_SPI_MODE_0,
        .sck_pin = { .port = WS_GPIO_PORT_I, .pin = 1u },
        .miso_pin = { .port = WS_GPIO_PORT_B, .pin = 14u },
        .use_miso = true,
        .use_mosi = false,
        .af = 5u,
    };
    if (WS_FAILED(ws_spi_init(&spi2, &cfg))) {
        return 1;
    }

    const uint8_t sample_frame[4] = { 0x01u, 0x90u, 0x19u, 0x00u };
    ws_spi_fake_queue_rx(WS_SPI_2, sample_frame, sizeof(sample_frame));

    uint8_t rx[4] = { 0 };
    if (WS_FAILED(ws_spi_read(&spi2, rx, sizeof(rx), 10u))) {
        return 1;
    }
    printf("SPI2 @ %u Hz read: %02X %02X %02X %02X\n", spi2.actual_hz, rx[0], rx[1], rx[2], rx[3]);
    return 0;
}
