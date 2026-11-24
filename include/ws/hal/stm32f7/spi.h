/**
 * @file spi.h
 * @brief ws-hal-stm32f7 SPI — synchronous master SPI with bounded timeouts.
 *
 * Includes <ws/hal/stm32f7/gpio.h> and <ws/hal/stm32f7/timebase.h>.
 * Enables peripheral clocks through rcc.h and configures SCK/MISO/MOSI pins
 * through gpio.h; never touches RCC or GPIO registers directly.
 *
 * Chip-select (CS) pins are owned by the calling peripheral driver via gpio.h
 * so multiple devices can share one SPI bus cleanly.
 */
#ifndef WS_WS_HAL_STM32F7_SPI_H
#define WS_WS_HAL_STM32F7_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ws/error/error.h>
#include <ws/hal/stm32f7/gpio.h>
#include <ws/hal/stm32f7/timebase.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_SPI_OK = 0,
    WS_SPI_BAD_CONFIG, /**< NULL handle/cfg, invalid pin/AF/mode, or underivable SCK prescaler. */
    WS_SPI_BUSY, /**< Peripheral is already busy on entry. */
    WS_SPI_TIMEOUT, /**< TXE/RXNE/BSY wait exceeded caller timeout_ms. */
    WS_SPI_BUS_ERROR, /**< Hardware OVR, MODF, or FRE fault flagged during transfer. */
} ws_spi_err_t;
WS_ERROR_TYPE(ws_spi_err_t, ws_spi_error_t);

typedef enum {
    WS_SPI_1 = 0, /**< SPI1 (APB2). */
    WS_SPI_2, /**< SPI2 (APB1; STM32F746G-DISCO Arduino D13/D12/D11: PI1/PB14/PB15 AF5). */
} ws_spi_instance_t;

#define WS_SPI_INSTANCE_COUNT (WS_SPI_2 + 1)

typedef enum {
    WS_SPI_MODE_0 = 0, /**< CPOL=0, CPHA=0 (clock idle low, sample on rising edge). */
    WS_SPI_MODE_1, /**< CPOL=0, CPHA=1 (clock idle low, sample on falling edge). */
    WS_SPI_MODE_2, /**< CPOL=1, CPHA=0 (clock idle high, sample on falling edge). */
    WS_SPI_MODE_3, /**< CPOL=1, CPHA=1 (clock idle high, sample on rising edge). */
} ws_spi_mode_t;

typedef struct {
    ws_spi_instance_t instance;
    uint32_t max_hz; /**< Maximum allowed SCK frequency; HAL picks fastest pclk/2^k <= max_hz. */
    ws_spi_mode_t mode;
    ws_gpio_t sck_pin;
    ws_gpio_t miso_pin; /**< Used when use_miso is true. */
    ws_gpio_t mosi_pin; /**< Used when use_mosi is true. */
    bool use_miso;
    bool use_mosi;
    uint8_t af;
} ws_spi_cfg_t;

typedef struct {
    ws_spi_instance_t instance;
    uint32_t actual_hz;
    bool initialized;
} ws_spi_t;

ws_spi_error_t ws_spi_init(ws_spi_t *spi, const ws_spi_cfg_t *cfg);

/**
 * @brief Synchronous 8-bit MSB-first full-duplex transfer with bounded timeout.
 *
 * When @p tx is NULL, transmits 0x00 dummy bytes. When @p rx is NULL, discards
 * received bytes. At least one of @p tx or @p rx must be non-NULL when @p len > 0.
 * Waits for SPI BSY to clear before returning so the caller can safely deassert CS.
 */
ws_spi_error_t ws_spi_transfer(ws_spi_t *spi, const uint8_t *tx, uint8_t *rx, size_t len,
                               uint32_t timeout_ms);

/** @brief Convenience read-only transfer (sends 0x00 dummy bytes). */
ws_spi_error_t ws_spi_read(ws_spi_t *spi, uint8_t *rx, size_t len, uint32_t timeout_ms);

/** @brief Convenience write-only transfer (discards received bytes). */
ws_spi_error_t ws_spi_write(ws_spi_t *spi, const uint8_t *tx, size_t len, uint32_t timeout_ms);

#if defined(WS_HAL_STM32F7_HOST_FAKE)
void ws_spi_fake_reset(void);
void ws_spi_fake_queue_rx(ws_spi_instance_t instance, const uint8_t *buf, size_t len);
size_t ws_spi_fake_take_tx(ws_spi_instance_t instance, uint8_t *buf, size_t len);
void ws_spi_fake_set_busy(ws_spi_instance_t instance, bool busy);
void ws_spi_fake_set_timeout(ws_spi_instance_t instance, bool timeout);
void ws_spi_fake_set_bus_error(ws_spi_instance_t instance, bool bus_error);
uint32_t ws_spi_fake_transfer_count(ws_spi_instance_t instance);
uint32_t ws_spi_fake_actual_hz(ws_spi_instance_t instance);
#endif /* WS_HAL_STM32F7_HOST_FAKE */

#ifdef __cplusplus
}
#endif

#endif /* WS_WS_HAL_STM32F7_SPI_H */
