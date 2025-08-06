/*
 * weatherbus_parser.h
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
 * This file provides the main parser for incoming bytes.
 * It's stateful, so it takes one byte of the frame at a time and returns
 * the parsed packet when complete.
 */

#include "weatherbus_types.h"
#include "weatherbus_hal.h"
#include "weatherbus_crc.h"
#include <string.h>

const sensorbus_timing_config_t SENSORBUS_DEFAULT_TIMING = {
	.slot_time_ms	= SENSORBUS_DEFAULT_SLOT_TIME_MS,
	.max_delay_ms	= SENSORBUS_DEFAULT_MAX_DELAY_MS
};

static enum {
	PARSER_WAIT_START,
	PARSER_WAIT_LENGTH,
	PARSER_WAIT_BODY,
	PARSER_WAIT_CRC
} parser_state = PARSER_WAIT_START;

static uint8_t parser_buffer[SENSORBUS_MAX_PACKET_SIZE];
static uint8_t parser_length = 0;
static uint8_t parser_index = 0;

// This is the parser and state machine for the SensorBus protocol.
// The message format is as follows:
// [START(1)][LEN(1)][TYPE(1)][DEVICE_ID(4)][PAYLOAD][CHECKSUM(1)]
// Where:
// - START is the start byte (0xAA).
// - LEN is the length of the packet (including TYPE, DEVICE_ID, and PAYLOAD).
// - TYPE is the message type (e.g., QUERY, RESPONSE).
// - DEVICE_ID is the 32-bit ID of the device sending the message.
// - PAYLOAD is the data being sent (up to SENSORBUS_MAX_PAYLOAD bytes).
// - CHECKSUM is the CRC-8 checksum of the packet (excluding START).
sensorbus_error_t sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t *out_packet)
{
	switch (parser_state) {
	case PARSER_WAIT_START:
		if (byte == SENSORBUS_START_BYTE) {
			parser_index = 0;
			parser_state = PARSER_WAIT_LENGTH;
		}
		break;

	case PARSER_WAIT_LENGTH:
		parser_length = byte;
		if (parser_length > SENSORBUS_MAX_PACKET_SIZE - 3) {
			parser_state = PARSER_WAIT_START;
			return SENSORBUS_ERR_FORMAT;
		}
		parser_buffer[parser_index++] = byte;
		parser_state = PARSER_WAIT_BODY;
		break;

	case PARSER_WAIT_BODY:
		parser_buffer[parser_index++] = byte;
		if (parser_index == parser_length + 1) // +1 for length byte
			parser_state = PARSER_WAIT_CRC;
		break;

	case PARSER_WAIT_CRC: {
		uint8_t crc = byte;
		uint8_t actual_crc = sensorbus_calc_crc8(parser_buffer, parser_index);
		parser_state = PARSER_WAIT_START; // Reset for next packet

		if (crc != actual_crc)
			return SENSORBUS_ERR_CRC; // CRC mismatch

		out_packet->msg_type = parser_buffer[1];
		out_packet->device_id = (uint32_t)parser_buffer[2] << 24;
		out_packet->device_id |= (uint32_t)parser_buffer[3] << 16;
		out_packet->device_id |= (uint32_t)parser_buffer[4] << 8;
		out_packet->device_id |= (uint32_t)parser_buffer[5] << 0;
		out_packet->payload_len = parser_length - 5;
		memcpy(out_packet->payload, &parser_buffer[6], out_packet->payload_len);
		return SENSORBUS_OK; // Packet complete
	}
	}

	return SENSORBUS_IN_PROGRESS; // In progress
}

void sensorbus_reset_parser(void)
{
	parser_state = PARSER_WAIT_START;
	parser_length = 0;
	parser_index = 0;
}
