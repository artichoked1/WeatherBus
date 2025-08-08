/*
 * esp_idf_hal.c
 *
 * This file is part of the WeatherBus library.
 * Copyright (c) 2025 Nikolai Patrick
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * This is an example HAL designed to be built on the application level, not as a library.
 * It has been intentially left out of the CMakeLists.txt.
 * 
 * If you want to use this HAL, paste it into your project have it built there.
 * This is so you can use the HAL without having to modify the library itself, and also to access
 * the uart device directly for things like putting the transceiver into shutdown mode.
 */
#if defined(ESP_PLATFORM)

#include "weatherbus.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RS485.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define UART_PORT   UART_NUM_1
#define TX_PIN      GPIO_NUM_17
#define RX_PIN      GPIO_NUM_16
#define DE_PIN      GPIO_NUM_18
#define RE_PIN      GPIO_NUM_19
#define BAUD_RATE   9600

rs485_uart_t uart_dev = {
    .uart_port = UART_NUM_1,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_16,
    .de_pin = GPIO_NUM_18,
    .re_pin = GPIO_NUM_19,
    .baud_rate = 9600
};

sensorbus_error_t sensorbus_hal_uart_init() {
    if (rs485_uart_init(&uart_dev) != ESP_OK) {
        return SENSORBUS_ERR_FAILURE;
    }
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_send_bytes(uint8_t* data, size_t len) {
    if (rs485_uart_write(&uart_dev, data, len) != ESP_OK) {
        return SENSORBUS_ERR_FAILURE;
    }
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms) {
    
    TickType_t ticks;

    if (timeout_ms == 0) {
    ticks = portMAX_DELAY;
    } else {
    ticks = pdMS_TO_TICKS(timeout_ms);
    }

    int r = uart_read_bytes(uart_dev.uart_port, byte, 1, ticks);

    if (r == 1) {
        return SENSORBUS_OK;
    }
    if (timeout_ms == 0) {
        return SENSORBUS_ERR_FAILURE;
    }
    return SENSORBUS_ERR_TIMEOUT;
}

void sensorbus_hal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint64_t sensorbus_hal_get_time_ms() {
    return esp_timer_get_time() / 1000; // Convert microseconds to milliseconds
}

void sensorbus_hal_purge_rx(void) {
    // drop any bytes in the UART driver
    uart_flush_input(UART_NUM_1);    // or whichever UART you use
}

#endif