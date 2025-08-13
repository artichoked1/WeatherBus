#pragma once

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

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

//--- Config ---//

/// @brief The start byte for SensorBus packets.
#define SENSORBUS_START_BYTE 0xAA

/// @brief The maximum payload size for SensorBus packets.
#define SENSORBUS_MAX_PAYLOAD 32

/// @brief The maximum size of a SensorBus packet, including all headers and CRC.
#define SENSORBUS_MAX_PACKET_SIZE (1 + 1 + 1 + 4 + SENSORBUS_MAX_PAYLOAD + 1) // start + len + type + id + payload + crc


/// @brief How big a discovery "slot" that a slave waits before replying to a discovery request is.
#define SENSORBUS_DEFAULT_SLOT_TIME_MS 25

/// @brief The maximum delay range for discovery replies, in milliseconds.
#define SENSORBUS_DEFAULT_MAX_DELAY_MS 30

/// @brief The maximum length of a value in a TILV record.
#define SENSORBUS_MAX_VALUE_LEN 4

/// @brief The maximum time the bus will wait to receive a complete frame in milliseconds.
#define SENSORBUS_FRAME_TIMEOUT_MS 1000 // Timeout for receiving a complete frame


/// @brief The maximum length of the "value" field in a TILV record. Usually 4 bytes for a 32-bit float.
#define SENSORBUS_MAX_RECORD_DATA 4

/// @brief Maximum possible TILV record size (3-byte TIL header + V max data)
#define SENSORBUS_MAX_RECORD_SIZE (3 + SENSORBUS_MAX_RECORD_DATA)

/// @brief Maximum number of TLV entries you can fit into a packet assuming every record could be full-size
#define SENSORBUS_MAX_TLVS (SENSORBUS_MAX_PAYLOAD / SENSORBUS_MAX_RECORD_SIZE)

/// @brief The maximum number of sensors that can be discovered in a single discovery response packet.
#define SNENSORBUS_MAX_SENSOR_DISCOVERY_SLOTS (SENSORBUS_MAX_PAYLOAD / 2) // 2 bytes per sensor (type + index)

#define MAX_PKT   SENSORBUS_MAX_PAYLOAD

//--- Structs and Enums ---//

/// @brief Timing configuration for discovery and response slots.
typedef struct {
    uint16_t slot_time_ms;   // time per response slot
    uint16_t max_delay_ms;   // maximum delay range
} sensorbus_timing_config_t;

extern const sensorbus_timing_config_t SENSORBUS_DEFAULT_TIMING;

/**
 * @brief Message types for SensorBus communication.
 * 
 * These types define the purpose of each message sent over the bus.
 * - `SENSORBUS_QUERY`: A request for data from a specific sensor.
 * - `SENSORBUS_RESPONSE`: A response containing sensor data.
 * - `SENSORBUS_SET`: A command to set a property or configuration on a device.
 * - `SENSORBUS_ACK`: An acknowledgment of a received message.
 * - `SENSORBUS_DISCOVERY`: A request to discover devices on the bus.
 * - `SENSORBUS_DISCOVERY_REPLY`: A response to a discovery request, containing device information.
 */
typedef enum {
    SENSORBUS_QUERY = 0x01,
    SENSORBUS_RESPONSE = 0x02,
    SENSORBUS_SET = 0x03,
    SENSORBUS_ACK = 0x04,
    SENSORBUS_DISCOVERY = 0x05,
    SENSORBUS_DISCOVERY_REPLY = 0x06,
} sensorbus_msg_type_t;

/**
 * @brief Error codes for WeatherBus operations.
 * 
 * These codes indicate the result of various operations on the bus.
 * - `SENSORBUS_OK`: Operation completed successfully.
 * - `SENSORBUS_IN_PROGRESS`: Operation is still in progress, more data is needed.
 * - `SENSORBUS_ERR_CRC`: CRC check failed, data may be corrupted.
 * - `SENSORBUS_ERR_FORMAT`: Data format is invalid or unexpected.
 * - `SENSORBUS_ERR_TIMEOUT`: Operation timed out waiting for data.
 * - `SENSORBUS_ERR_FAILURE`: General failure, operation could not be completed.
 */
typedef enum {
    SENSORBUS_OK = 0,
    SENSORBUS_IN_PROGRESS = 1,
    SENSORBUS_ERR_CRC = -1,
    SENSORBUS_ERR_FORMAT = -2,
    SENSORBUS_ERR_TIMEOUT = -3,
    SENSORBUS_ERR_FAILURE = -4
} sensorbus_error_t;

/**
 * @brief WeatherBus packet structure.
 */
typedef struct {
    uint8_t msg_type;
    uint32_t device_id;
    uint8_t payload[SENSORBUS_MAX_PAYLOAD];
    uint8_t payload_len;
} sensorbus_packet_t;

/**
 * @brief WeatherBus packet header structure.
 * 
 * This is used to encode or parse incoming packets byte-by-byte.
 */
typedef struct {
    uint8_t buf[SENSORBUS_MAX_PAYLOAD];
    uint8_t len;
} payload_builder_t;

typedef enum {
    SENSORBUS_FMT_UINT8    = 0,  // 1 byte  unsigned
    SENSORBUS_FMT_UINT16   = 1,  // 2 bytes unsigned
    SENSORBUS_FMT_FLOAT32  = 2,  // 4 bytes IEEE-754 single
    SENSORBUS_FMT_FLOAT64  = 3,  // 8 bytes IEEE-754 double (or custom)
    SENSORBUS_FMT_SFIX16_2DP = 4, // Signed fixed-point 16-bit int with 2 decimal places, scaled x100. Good for temperatures, etc.
    SENSORBUS_FMT_UFIX16_1DP = 5, // Unsigned fixed-point 16-bit int with 1 decimal place, scaled x10. Good for pressure.
    // 6…7 reserved for future use
} sensorbus_format_t;

/// lookup table: format to number of data bytes
static const uint8_t SENSORBUS_FMT_LEN[8] = {
    [SENSORBUS_FMT_UINT8]   = 1,
    [SENSORBUS_FMT_UINT16]  = 2,
    [SENSORBUS_FMT_FLOAT32] = 4,
    [SENSORBUS_FMT_FLOAT64] = 8,
    [SENSORBUS_FMT_SFIX16_2DP] = 2,
    [SENSORBUS_FMT_UFIX16_1DP] = 2,
    // others default to 0
};

/**
 * @brief Sensor data structure for WeatherBus.
 * 
 * This structure represents a single sensor's data in the bus.
 * It includes the type of sensor, an index for multiple sensors of the same type,
 * It is used for sending sensor data and for querying specific sensors.
 */
typedef struct {
    uint8_t type;                        // original sensor type ID
    uint8_t index;                       // 5-bit slot number on the bus
    sensorbus_format_t format;                      // 3-bit fmt code (0…7)
    uint8_t value[SENSORBUS_MAX_VALUE_LEN];  // raw bytes, length = SENSORBUS_FMT_LEN[format]
} sensorbus_sensor_t;

#ifdef __cplusplus
}
#endif