/*
 * weatherbus_types.h
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
 * This file defines the types, constants, and enums used throughout the WeatherBus library.
 */

#include <string.h>
#include "weatherbus.h"

/**
 * @brief Initialize a payload builder.
 *
 * Sets the initial length of the payload to 0.
 *
 * @param pb Pointer to the payload builder to initialize.
 * @return SENSORBUS_OK on success, or an error code.
 */
void pb_init(payload_builder_t *pb)
{
	pb->len = 0;
}

/**
 * @brief Add a sensor record to the payload builder.
 *
 * This function adds a sensor record with the specified type, format, index,
 * and data to the payload builder. It checks if there is enough space in the
 * payload before adding the record.
 *
 * @param pb Pointer to the payload builder.
 * @param type Sensor type ID.
 * @param fmt The datatype/format used. Use `sensorbus_format_t`.
 * @param index 5-bit index for the sensor.
 * @param data Pointer to the raw data for the sensor.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_pb_add_sensor(payload_builder_t *pb, uint8_t type, sensorbus_format_t fmt, uint8_t index, const void *data)
{
	uint8_t len = SENSORBUS_FMT_LEN[fmt];

	if (len == 0 || pb->len + 2 + len > SENSORBUS_MAX_PAYLOAD)
		return SENSORBUS_ERR_FORMAT;

	pb->buf[pb->len++] = type;
	pb->buf[pb->len++] = (uint8_t)((fmt << 5) | (index & 0x1F));

	if (data)
		memcpy(&pb->buf[pb->len], data, len);
	pb->len += len;

	return SENSORBUS_OK;
}

/**
 * @brief Helper function to add 32-bit float sensor record to the payload builder.
 *
 * This function adds a sensor record with the specified type, index, and
 * a 32-bit float value to the payload builder.
 *
 * @param pb Pointer to the payload builder.
 * @param type Sensor type ID.
 * @param index 5-bit index for the sensor.
 * @param value The float value to add.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t pb_add_float(payload_builder_t *pb, uint8_t type, uint8_t index, float value)
{
	return sensorbus_pb_add_sensor(pb, type, SENSORBUS_FMT_FLOAT32, index, &value);
}

/**
 * @brief Decode a raw response payload into an array of sensorbus_sensor_t structures.
 *
 * @param buf pointer to incoming bytes
 * @param buf_len number of bytes in buf
 * @param sensors array to fill
 * @param out_count pointer to size_t to receive how many entries were written
 *
 * @return SENSORBUS_OK on success,
 *         SENSORBUS_ERR_FORMAT if any parsing error (truncation/invalid fmt).
 */
sensorbus_error_t sensorbus_pb_decode_sensors(const uint8_t *buf, uint8_t buf_len, sensorbus_sensor_t *sensors, size_t *out_count)
{
	size_t count = 0;
	uint8_t pos = 0;

	while (pos + 2 <= buf_len) {
		if (count >= SENSORBUS_MAX_TLVS)
			return SENSORBUS_ERR_FORMAT;

		uint8_t type = buf[pos++];
		uint8_t hdr = buf[pos++];
		uint8_t fmt = hdr >> 5;
		uint8_t index = hdr & 0x1F;

		if (fmt > 7)
			return SENSORBUS_ERR_FORMAT;
		uint8_t len = SENSORBUS_FMT_LEN[fmt];
		if (pos + len > buf_len)
			return SENSORBUS_ERR_FORMAT;

		// Fill the struct
		sensors[count].type = type;
		sensors[count].index = index;
		sensors[count].format = fmt;
		memcpy(sensors[count].value, &buf[pos], len);

		pos += len;
		count += 1;
	}

	*out_count = count;
	return SENSORBUS_OK;
}

/**
 * @brief Add a sensor descriptor to the payload builder.
 *
 * For queries and discovery responses, 2-byte descriptors are used to
 * tell what sensors are available (or what ones are requested in the case of a query)
 *
 * @param pb Pointer to the payload builder.
 * @param type Sensor type ID.
 * @param index 5-bit index for the sensor.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t sensorbus_pb_add_descriptor(payload_builder_t *pb, uint8_t type, uint8_t index)
{
	// need exactly 2 bytes
	if ((size_t)pb->len + 2 > SENSORBUS_MAX_PAYLOAD)
		return SENSORBUS_ERR_FORMAT;
	pb->buf[pb->len++] = type;
	pb->buf[pb->len++] = (uint8_t)(0 << 5 | (index & 0x1F));

	return SENSORBUS_OK;
}

/**
 * @brief Decode a buffer of descriptors into an array of sensorbus_sensor_t structs.
 *
 * @param buf incoming byte buffer
 * @param buf_len length of buf
 * @param out array to populate
 * @param out_count pointer to size_t to receive number of queries parsed
 * @return SENSORBUS_OK on success,
 *         SENSORBUS_ERR_FORMAT on any parse error
 */
sensorbus_error_t sensorbus_pb_decode_descriptors(const uint8_t *buf, uint8_t buf_len, sensorbus_sensor_t *out, size_t *out_count)
{
	size_t count = 0;
	uint8_t pos = 0;

	// Parse as many 2-byte entries as will fit
	while (pos + 2 <= buf_len && count < SENSORBUS_MAX_TLVS) {
		uint8_t type = buf[pos++];
		uint8_t hdr = buf[pos++];
		uint8_t fmt = hdr >> 5;
		uint8_t idx = hdr & 0x1F;

		if (fmt > 7)
			return SENSORBUS_ERR_FORMAT;

		out[count].type = type;
		out[count].index = idx;
		out[count].format = (sensorbus_format_t)fmt;
		// No value bytes in a query. Clear?
		// memset(out[count].value, 0, SENSORBUS_MAX_VALUE_LEN);

		count++;
	}

	// if we didn’t consume exactly all bytes, its buggered
	if (pos != buf_len)
		return SENSORBUS_ERR_FORMAT;

	*out_count = count;
	return SENSORBUS_OK;
}
