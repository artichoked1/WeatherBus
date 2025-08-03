#include <string.h>

#include "WeatherBus.h"

/**
 * @brief Initialize a payload builder.
 *
 * Sets the initial length of the payload to 0.
 *
 * @param pb Pointer to the payload builder to initialize.
 * @return SENSORBUS_OK on success, or an error code.
 */
void pb_init(payload_builder_t* pb) { pb->len = 0; }

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
sensorbus_error_t pb_add_sensor(payload_builder_t* pb, uint8_t type, sensorbus_format_t fmt, uint8_t index, const void* data) {
  uint8_t len = SENSORBUS_FMT_LEN[fmt];
  if (len == 0 || pb->len + 2 + len > SENSORBUS_MAX_PAYLOAD) {
    return SENSORBUS_ERR_FORMAT;
  }

  pb->buf[pb->len++] = type;
  pb->buf[pb->len++] = (uint8_t)((fmt << 5) | (index & 0x1F));

  if (data) {
    memcpy(&pb->buf[pb->len], data, len);
  }
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
sensorbus_error_t pb_add_float(payload_builder_t* pb, uint8_t type, uint8_t index, float value) {
  return pb_add_sensor(pb, type, SENSORBUS_FMT_FLOAT32, index, &value);
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
sensorbus_error_t sensorbus_decode_payload(const uint8_t* buf, uint8_t buf_len, sensorbus_sensor_t* sensors, size_t* out_count) {
  size_t count = 0;
  uint8_t pos = 0;

  while (pos + 2 <= buf_len) {
    if (count >= SENSORBUS_MAX_TLVS) {
      return SENSORBUS_ERR_FORMAT;
    }

    uint8_t type = buf[pos++];
    uint8_t hdr = buf[pos++];
    uint8_t fmt = hdr >> 5;
    uint8_t index = hdr & 0x1F;

    if (fmt > 7) {
      return SENSORBUS_ERR_FORMAT;
    }
    uint8_t len = SENSORBUS_FMT_LEN[fmt];
    if (pos + len > buf_len) {
      return SENSORBUS_ERR_FORMAT;
    }

    // fill the struct
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
 * @brief Add a query record to the payload builder.
 *
 * Adds a query record with the specified type and index to the
 * payload builder.
 *
 * @param pb Pointer to the payload builder.
 * @param type Sensor type ID.
 * @param index 5-bit index for the sensor.
 * @return SENSORBUS_OK on success, or an error code.
 */
sensorbus_error_t pb_add_query(payload_builder_t* pb, uint8_t type,
                               uint8_t index) {
  // need exactly 2 bytes
  if ((size_t)pb->len + 2 > SENSORBUS_MAX_PAYLOAD) {
    return SENSORBUS_ERR_FORMAT;
  }
  pb->buf[pb->len++] = type;
  pb->buf[pb->len++] = (uint8_t)(0 << 5 | (index & 0x1F));

  return SENSORBUS_OK;
}