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
#include "weatherbus_hal.h"
#include "weatherbus_parser.h"
#include "weatherbus_crc.h"
#include <string.h>

sensorbus_error_t sensorbus_init(void)
{
	if (sensorbus_hal_uart_init() != SENSORBUS_OK)
		return SENSORBUS_ERR_FAILURE; // UART init failed

	sensorbus_reset_parser();

	return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t *payload, uint8_t len)
{
	if (len > SENSORBUS_MAX_PAYLOAD) return SENSORBUS_ERR_FORMAT; // Payload too long

	uint8_t packet[SENSORBUS_MAX_PACKET_SIZE];
	size_t idx = 0;

	packet[idx++] = SENSORBUS_START_BYTE;   // Start delimiter
	packet[idx++] = 1 + 4 + len;            // Packet length: message type (1) + device ID (4) + payload (len)
	packet[idx++] = type;                   // Message type

	// Now we bit-shift the device ID into place high-order first (big-endian).
	// We prevent any weird cast issues by using 0xFF to mask the bytes.
	packet[idx++] = (device_id >> 24) & 0xFF;
	packet[idx++] = (device_id >> 16) & 0xFF;
	packet[idx++] = (device_id >> 8) & 0xFF;
	packet[idx++] = (device_id) & 0xFF;

	// Copy the payload into the packet.
	// memcpy() is more efficient than a loop for this.
	memcpy(&packet[idx], payload, len);
	idx += len;

	// Calculate the CRC-8 checksum for the packet.
	uint8_t crc = sensorbus_calc_crc8(&packet[1], idx - 1); // Exclude start byte
	packet[idx++] = crc;


	if (sensorbus_hal_send_bytes(packet, idx) != SENSORBUS_OK)
		return SENSORBUS_ERR_FAILURE;   // Sending failed

	return SENSORBUS_OK;                    // Successfully sent
}

sensorbus_error_t sensorbus_receive_blocking(sensorbus_packet_t *out)
{
	uint8_t b;
	sensorbus_error_t res;

	sensorbus_reset_parser();

	while (1) {
		// Block forever for the next byte
		res = sensorbus_hal_receive_byte(&b, 0);
		if (res != SENSORBUS_OK)
			// fatal bus error
			return res;
		res = sensorbus_parser_tick(b, out);
		if (res != SENSORBUS_IN_PROGRESS)
			return res;
	}
}

sensorbus_error_t sensorbus_receive_timeout(uint32_t		timeout_ms,
					    sensorbus_packet_t *out)
{
	int64_t deadline = sensorbus_hal_get_time_ms() + timeout_ms;
	uint8_t b;
	sensorbus_error_t res;

	sensorbus_reset_parser();

	while (sensorbus_hal_get_time_ms() < deadline) {
		res = sensorbus_hal_receive_byte(&b, 10);
		if (res == SENSORBUS_ERR_TIMEOUT)
			continue;
		if (res != SENSORBUS_OK)
			return res;
		res = sensorbus_parser_tick(b, out);
		if (res != SENSORBUS_IN_PROGRESS)
			return res;
	}
	return SENSORBUS_ERR_TIMEOUT;
}


void sensorbus_apply_discovery_delay(uint32_t device_id, const sensorbus_timing_config_t *config)
{
	if (!config) return;
	uint16_t delay_slots = (device_id % config->max_delay_ms);
	uint32_t delay = delay_slots * config->slot_time_ms;
	sensorbus_hal_delay_ms(delay);
}
