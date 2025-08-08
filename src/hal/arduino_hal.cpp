/*
 * arduino_hal.cpp
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
 * Since Arduino/PlatformIO just globs all .cpp files, I have prevented it from being compiled
 * with the ENABLED define.
 * 
 * If you want to use this HAL, paste it into your project have it built there.
 * This is so you can use the HAL without having to modify the library itself, and also to access
 * the uart device directly for things like putting the trnasceiver into shutdown mode.
 */
// #define ENABLED

#ifdef ENABLED
#if defined(__AVR__)

#include "weatherbus.h"
#include "RS485.h"

rs485_uart_t uart_dev = {
    .serial_port = &Serial2, // Use Serial2 for RS485 communication
    .de_pin = 23,            // DE pin for RS485
    .re_pin = 22,            // RE pin for RS485
    .baud_rate = 9600        // Baud rate for RS485 communication
};

sensorbus_error_t sensorbus_hal_uart_init() {
    rs485_init(&uart_dev);
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_send_bytes(uint8_t* data, size_t len) {
    rs485_begin_transmission(&uart_dev);
    rs485_write_buffer(&uart_dev, data, len);
    rs485_end_transmission(&uart_dev);
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms) {
    uint32_t start = millis();

    while(1) {
        if (rs485_available(&uart_dev)) {
            *byte = rs485_read(&uart_dev);
            return SENSORBUS_OK;
        }

        // If timeout_ms is 0, block forever
        if (timeout_ms == 0) {
            continue;
        }

        // Otherwise check for timeout
        if ((millis() - start) >= timeout_ms) {
            return SENSORBUS_ERR_TIMEOUT;
        }
    }
}

void sensorbus_hal_delay_ms(uint32_t ms) {
    delay(ms);
}

uint64_t sensorbus_hal_get_time_ms() {
    return millis();
}

#endif
#endif