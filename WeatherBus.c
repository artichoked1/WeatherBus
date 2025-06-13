#include "WeatherBus.h"
#include <string.h>

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
const sensorbus_timing_config_t SENSORBUS_DEFAULT_TIMING = {
    .slot_time_ms = SENSORBUS_DEFAULT_SLOT_TIME_MS,
    .max_delay_ms = SENSORBUS_DEFAULT_MAX_DELAY_MS
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

void sensorbus_init(void) {
    sensorbus_hal_uart_init();
    parser_state = PARSER_WAIT_START;
    parser_length = 0;
    parser_index = 0;
}

int sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t* payload, uint8_t len) {
    if (len > SENSORBUS_MAX_PAYLOAD) return -1; // Fail if payload is too large

    uint8_t packet[SENSORBUS_MAX_PACKET_SIZE]; 
    size_t idx = 0;

    packet[idx++] = SENSORBUS_START_BYTE; // Start delimiter
    packet[idx++] = 1 + 4 + len; // Packet length: message type (1) + device ID (4) + payload (len)
    packet[idx++] = type; // Message type

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

    sensorbus_hal_send_bytes(packet, idx);
    return 0;
}

// Non-blocking state machine parser
int sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t* out_packet) {
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
                break;
            }
            parser_buffer[parser_index++] = byte;
            parser_state = PARSER_WAIT_BODY;
            break;

        case PARSER_WAIT_BODY:
            parser_buffer[parser_index++] = byte;
            if (parser_index == parser_length + 1) { // +1 for length byte
                parser_state = PARSER_WAIT_CRC;
            }
            break;

        case PARSER_WAIT_CRC: {
            uint8_t crc = byte;
            uint8_t actual_crc = sensorbus_calc_crc8(parser_buffer, parser_index);
            parser_state = PARSER_WAIT_START; // Reset for next packet

            if (crc != actual_crc) {
                return -2; // CRC mismatch
            }

            out_packet->msg_type = parser_buffer[1];
            out_packet->device_id = (parser_buffer[2] << 24) | (parser_buffer[3] << 16) | (parser_buffer[4] << 8) | parser_buffer[5];
            out_packet->payload_len = parser_length - 5;
            memcpy(out_packet->payload, &parser_buffer[6], out_packet->payload_len);
            return 1; // Packet complete
        }
    }

    return 0; // In progress
}

int sensorbus_receive(sensorbus_packet_t* out_packet) {
    uint8_t byte;
    while (1) {
        if (sensorbus_hal_receive_byte(&byte, 100) != 0) return -1;
        int res = sensorbus_parser_tick(byte, out_packet);
        if (res == 1 || res == -2) return res;
    }
}

void sensorbus_apply_discovery_delay(uint32_t device_id, const sensorbus_timing_config_t* config) {
    if (!config) return;
    uint16_t delay_slots = (device_id % config->max_delay_ms);
    uint32_t delay = delay_slots * config->slot_time_ms;
    sensorbus_hal_delay_ms(delay);
}

uint8_t sensorbus_calc_crc8(const uint8_t* data, size_t len) {
    // CRC-8-ATM: x^8 + x^2 + x + 1 = 0x07
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
