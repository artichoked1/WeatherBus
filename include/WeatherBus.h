#pragma once

#include <stdint.h>
#include <stddef.h>

#define SENSORBUS_START_BYTE  0xAA
#define SENSORBUS_MAX_PAYLOAD 32
#define SENSORBUS_MAX_PACKET_SIZE (1 + 1 + 1 + 4 + SENSORBUS_MAX_PAYLOAD + 1) // start + len + type + id + payload + crc

typedef struct {
    uint16_t slot_time_ms;   // time per response slot
    uint16_t max_delay_ms;   // maximum delay range
} sensorbus_timing_config_t;

// Default discovery timing config
#define SENSORBUS_DEFAULT_SLOT_TIME_MS  10
#define SENSORBUS_DEFAULT_MAX_DELAY_MS  64
#define SENSORBUS_MAX_VALUE_LEN 4
extern const sensorbus_timing_config_t SENSORBUS_DEFAULT_TIMING;


// Maximum data bytes per record
#define SENSORBUS_MAX_RECORD_DATA    4
// Maximum possible record size (3-byte header + max data)
#define SENSORBUS_MAX_RECORD_SIZE    (3 + SENSORBUS_MAX_RECORD_DATA)
// Maximum number of TLV entries assuming every record could be full-size
// Maximum number of full-size TLV entries (integer division truncates remainder)
#define SENSORBUS_MAX_TLVS           (SENSORBUS_MAX_PAYLOAD / SENSORBUS_MAX_RECORD_SIZE)

// Message types
typedef enum {
    SENSORBUS_QUERY = 0x01,
    SENSORBUS_RESPONSE = 0x02,
    SENSORBUS_SET = 0x03,
    SENSORBUS_ACK = 0x04,
    SENSORBUS_DISCOVERY = 0x05,
    SENSORBUS_DISCOVERY_REPLY = 0x06,
} sensorbus_msg_type_t;

// Error codes
typedef enum {
    SENSORBUS_OK = 0,
    SENSORBUS_IN_PROGRESS = 1,
    SENSORBUS_ERR_CRC = -1,
    SENSORBUS_ERR_FORMAT = -2,
    SENSORBUS_ERR_TIMEOUT = -3,
    SENSORBUS_ERR_FAILURE = -4
} sensorbus_error_t;

typedef struct {
    uint8_t msg_type;
    uint32_t device_id;
    uint8_t payload[SENSORBUS_MAX_PAYLOAD];
    uint8_t payload_len;
} sensorbus_packet_t;

typedef struct {
    uint8_t buf[SENSORBUS_MAX_PAYLOAD];
    uint8_t len;
} payload_builder_t;

typedef struct {
    uint8_t        type;
    uint8_t        index;
    uint8_t        len;
    const uint8_t* value;
} sensorbus_sensor_t;

#ifdef __cplusplus
extern "C" {
#endif

// Public API
sensorbus_error_t sensorbus_init(void);
sensorbus_error_t sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t* payload, uint8_t len);
sensorbus_error_t sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t* out_packet); // non-blocking state machine parser
sensorbus_error_t sensorbus_receive_blocking(sensorbus_packet_t *out_packet);
sensorbus_error_t sensorbus_receive_timeout(uint32_t timeout_ms, sensorbus_packet_t *out_packet);
void sensorbus_apply_discovery_delay(uint32_t device_id, const sensorbus_timing_config_t* config);

// Hardware abstraction layer (replace with platform-specific impls)
sensorbus_error_t sensorbus_hal_send_bytes(uint8_t* data, size_t len);
sensorbus_error_t sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms);
sensorbus_error_t sensorbus_hal_uart_init(void);
void sensorbus_hal_delay_ms(uint32_t ms);
uint64_t sensorbus_hal_get_time_ms(void);

// Utility
uint8_t sensorbus_calc_crc8(const uint8_t* data, size_t len);

sensorbus_error_t pb_init(payload_builder_t* pb);
sensorbus_error_t pb_add_tlv(payload_builder_t* pb, uint8_t type, uint8_t index, const void* data, uint8_t data_len);
sensorbus_error_t pb_add_float(payload_builder_t* pb, uint8_t type, uint8_t index, float value);
sensorbus_error_t pb_add_error(payload_builder_t* pb, uint8_t type, uint8_t index);
sensorbus_error_t pb_decode_tlv(const uint8_t* buf, uint8_t buf_len, sensorbus_sensor_t* out, size_t* out_count);
sensorbus_error_t pb_add_query(payload_builder_t *pb, sensorbus_sensor_t *sensor);

void sensorbus_reset_parser(void);

#ifdef __cplusplus
}
#endif