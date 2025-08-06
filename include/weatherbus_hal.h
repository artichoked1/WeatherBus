#pragma once

/*
 * weatherbus_hal.h
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
 * WeatherBus only has a few hardware dependencies, mainly UART and some timing stuff.
 * The HAL abstracts these dependencies so it can be ported to different platforms easily.
 * Instead of controlling the DE/RE pins directly, this is managed by the HAL to allow for
 * potentially different data-link layers to be used in the future such as CAN.
 * I have written a simple RS485 wrapper around serial for ESP-IDF and Arduino which can be found on my GitHub.
 */

#include "weatherbus_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send a byte array over the bus.
 * 
 * @param data Pointer to the data to send.
 * @param len Length of the data in bytes.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_hal_send_bytes(uint8_t* data, size_t len);

/**
 * @brief Receive a single byte from the bus.
 * 
 * @param byte Pointer to store the received byte.
 * @param timeout_ms Timeout in milliseconds, if set to 0, it will block forever.
 * @return SENSORBUS_OK on success, SENSORBUS_ERR_TIMEOUT if no byte was received within the timeout,
 *         or an error code.
 */
sensorbus_error_t sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms);

/**
 * @brief Initialize the UART for the bus.
 * 
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_hal_uart_init(void);

/**
 * @brief Freeze the MCU for a given number of milliseconds.
 * 
 * @param ms 
 */
void sensorbus_hal_delay_ms(uint32_t ms);

/**
 * @brief Get the current run time in milliseconds.
 * 
 * @return Current time in milliseconds since boot.
 */
uint64_t sensorbus_hal_get_time_ms(void);

void sensorbus_hal_purge_rx(void);

#ifdef __cplusplus
}
#endif