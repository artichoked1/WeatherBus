#pragma once

/*
 * weatherbus_payload.h
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
 * This file provides the payload builder and utility functions for WeatherBus packets.
 * It allows you to build payloads for sending sensor data, descriptors, and other information.
 * 
 */

#include "weatherbus_types.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

void pb_init(payload_builder_t* pb);

sensorbus_error_t sensorbus_pb_add_sensor(payload_builder_t* pb,
                                uint8_t type,
                                sensorbus_format_t fmt,
                                uint8_t index, 
                                const void* data);

sensorbus_error_t sensorbus_pb_decode_sensors(const uint8_t* buf,
                                          uint8_t buf_len,
                                          sensorbus_sensor_t* sensors,
                                          size_t* out_count);

sensorbus_error_t sensorbus_pb_add_descriptor(payload_builder_t* pb,
                                              uint8_t type,
                                              uint8_t index);

sensorbus_error_t sensorbus_pb_decode_descriptors(const uint8_t* buf,
                                                  uint8_t buf_len, 
                                                  sensorbus_sensor_t*
                                                  out,size_t* out_count);

sensorbus_error_t pb_add_float(payload_builder_t* pb, 
                                uint8_t type,
                                uint8_t index,
                                float value);




/**
 * @brief Read an unsigned 8-bit value.
 */
static inline uint8_t sensorbus_to_uint8(const sensorbus_sensor_t* s) {
  return s->value[0];
}

/**
 * @brief Read a signed 8-bit value.
 */
static inline int8_t sensorbus_to_int8(const sensorbus_sensor_t* s) {
  return (int8_t)s->value[0];
}

/**
 * @brief Read an unsigned 16-bit little-endian value.
 */
static inline uint16_t sensorbus_to_uint16(const sensorbus_sensor_t* s) {
  return (uint16_t)s->value[0] | ((uint16_t)s->value[1] << 8);
}

/**
 * @brief Read a signed 16-bit little-endian value.
 */
static inline int16_t sensorbus_to_int16(const sensorbus_sensor_t* s) {
  return (int16_t)((uint16_t)s->value[0] | ((uint16_t)s->value[1] << 8));
}

/**
 * @brief Read an unsigned 32-bit little-endian value.
 */
static inline uint32_t sensorbus_to_uint32(const sensorbus_sensor_t* s) {
  return (uint32_t)s->value[0] | ((uint32_t)s->value[1] << 8) |
         ((uint32_t)s->value[2] << 16) | ((uint32_t)s->value[3] << 24);
}

/**
 * @brief Read a signed 32-bit little-endian value.
 */
static inline int32_t sensorbus_to_int32(const sensorbus_sensor_t* s) {
  return (int32_t)sensorbus_to_uint32(s);
}

/**
 * @brief Read a 32-bit IEEE-754 float.
 */
static inline float sensorbus_to_float32(const sensorbus_sensor_t* s) {
  float v;
  memcpy(&v, s->value, sizeof(v));
  return v;
}

/**
 * @brief Read a 64-bit IEEE-754 double.
 */
static inline double sensorbus_to_float64(const sensorbus_sensor_t* s) {
  double v;
  memcpy(&v, s->value, sizeof(v));
  return v;
}

static inline float sensorbus_to_temp_centi(const sensorbus_sensor_t* s) {
  int16_t raw;
  memcpy(&raw, s->value, sizeof(raw));
  return raw / 100.0f;
}

#ifdef __cplusplus
}
#endif