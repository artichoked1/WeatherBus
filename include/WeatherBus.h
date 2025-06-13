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
extern const sensorbus_timing_config_t SENSORBUS_DEFAULT_TIMING;

// Message types
typedef enum {
    SENSORBUS_QUERY = 0x01,
    SENSORBUS_RESPONSE = 0x02,
    SENSORBUS_SET = 0x03,
    SENSORBUS_ACK = 0x04,
    SENSORBUS_DISCOVERY = 0x05,
    SENSORBUS_DISCOVERY_REPLY = 0x06,
} sensorbus_msg_type_t;

// Parsed message struct
typedef struct {
    uint8_t msg_type;
    uint32_t device_id;
    uint8_t payload[SENSORBUS_MAX_PAYLOAD];
    uint8_t payload_len;
} sensorbus_packet_t;

#ifdef __cplusplus
extern "C" {
#endif

// Public API
void sensorbus_init(void);
int sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t* payload, uint8_t len);
int sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t* out_packet); // non-blocking state machine parser
int sensorbus_receive(sensorbus_packet_t* out_packet); // blocking wrapper
void sensorbus_apply_discovery_delay(uint32_t device_id, const sensorbus_timing_config_t* config);

// Hardware abstraction layer (replace with platform-specific impls)
void sensorbus_hal_send_bytes(const uint8_t* data, size_t len);
int  sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms);
void sensorbus_hal_delay_ms(uint32_t ms);
void sensorbus_hal_uart_init(void);

// Utility
uint8_t sensorbus_calc_crc8(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif