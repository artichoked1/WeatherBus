#pragma once

/*
 * weatherbus_transport.h
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
 * This file provides the main transport layer for WeatherBus communication.
 * It handles sending and receiving packets, including discovery and data exchange.
 */

#include "weatherbus_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the bus.
 * This sets up UART and resets the parser state.
 * 
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_init(void);

/**
 * @brief Send a message on the bus.
 * 
 * @param type Message type (e.g., QUERY, RESPONSE).
 * @param device_id 32-bit device ID.
 * @param payload Pointer to the payload data.
 * @param len Length of the payload data.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t* payload, uint8_t len);

/**
 * @brief Receive a complete packet from the bus.
 * It blocks forever until a complete packet is received.
 * 
 * @param out_packet 
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_receive_blocking(sensorbus_packet_t *out_packet);

/**
 * @brief Receive a packet from the bus with a timeout.
 * It will wait for up to `SENSORBUS_FRAME_TIMEOUT_MS` milliseconds for a complete packet.
 * 
 * @param timeout_ms 
 * @param out_packet 
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_receive_timeout(uint32_t timeout_ms, sensorbus_packet_t *out_packet);

/**
 * @brief Apply a discovery delay based on the device ID and timing configuration.
 * This MUST be called by slaves before sending their discovery reply to avoid a stampede of replies and collisions.
 * 
 * @param device_id The 32-bit device ID of the sensor.
 * @param config Pointer to the timing configuration.
 */
void sensorbus_apply_discovery_delay(uint32_t device_id, const sensorbus_timing_config_t* config);

#ifdef __cplusplus
}
#endif