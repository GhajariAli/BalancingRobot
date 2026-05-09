#include "sh2/sh2_hal.h"
#include "sh2/sh2_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

#define SPI_HOST    SPI2_HOST
#define PIN_SCK     GPIO_NUM_4
#define PIN_MISO    GPIO_NUM_5
#define PIN_MOSI    GPIO_NUM_6
#define PIN_CS      GPIO_NUM_7
#define PIN_INT     GPIO_NUM_2
#define PIN_RST     GPIO_NUM_3

#define SPI_MAX_TRANSFER  256

static spi_device_handle_t spi;
static sh2_Hal_t hal;

static void spi_init(void) {
    spi_bus_config_t bus = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_TRANSFER,
    };
    spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .mode = 3,
        .clock_speed_hz = 1000000,   // 1 MHz — safe for debugging
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 4,
        .cs_ena_posttrans = 4,
    };
    spi_bus_add_device(SPI_HOST, &dev, &spi);
}

static void reset_bno085(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));  // give it plenty of boot time
}

static int hal_open(sh2_Hal_t *self) {
    spi_init();

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    reset_bno085();
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self) {
    spi_bus_remove_device(spi);
    spi_bus_free(SPI_HOST);
}

static int hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    if (gpio_get_level(PIN_INT) != 0) return 0;
    if (len > SPI_MAX_TRANSFER) len = SPI_MAX_TRANSFER;

    // dummy TX buffer (all zeros)
    static uint8_t tx_buf[SPI_MAX_TRANSFER];
    memset(tx_buf, 0, len);

    spi_transaction_t txn = {
        .length = len * 8,
        .rxlength = len * 8,
        .rx_buffer = buf,
        .tx_buffer = tx_buf,
    };

    esp_err_t ret = spi_device_transmit(spi, &txn);
    if (ret != ESP_OK) return 0;

    *t_us = (uint32_t)(esp_timer_get_time());
    return len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    if (len > SPI_MAX_TRANSFER) return 0;

    static uint8_t rx_buf[SPI_MAX_TRANSFER];

    spi_transaction_t txn = {
        .length = len * 8,
        .rxlength = len * 8,
        .tx_buffer = buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(spi, &txn);
    return (ret == ESP_OK) ? len : 0;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    return (uint32_t)(esp_timer_get_time());
}

sh2_Hal_t *bno085_get_hal(void) {
    hal.open      = hal_open;
    hal.close     = hal_close;
    hal.read      = hal_read;
    hal.write     = hal_write;
    hal.getTimeUs = hal_get_time_us;
    return &hal;
}