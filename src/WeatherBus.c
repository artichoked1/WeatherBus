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

sensorbus_error_t sensorbus_init(void) {
    if (sensorbus_hal_uart_init() != SENSORBUS_OK) {
        return SENSORBUS_ERR_FAILURE; // UART init failed
    }

    sensorbus_reset_parser();

    return SENSORBUS_OK;
}

void sensorbus_reset_parser(void) {
    parser_state = PARSER_WAIT_START;
    parser_length = 0;
    parser_index = 0;
}

sensorbus_error_t sensorbus_send(sensorbus_msg_type_t type, uint32_t device_id, const uint8_t* payload, uint8_t len) {
    if (len > SENSORBUS_MAX_PAYLOAD) return SENSORBUS_ERR_FORMAT; // Payload too long

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

    
    if(sensorbus_hal_send_bytes(packet, idx) != SENSORBUS_OK) {
        return SENSORBUS_ERR_FAILURE; // Sending failed
    }

    return SENSORBUS_OK; // Successfully sent
}

// Non-blocking state machine parser
sensorbus_error_t sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t* out_packet) {
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
            if (parser_index == parser_length + 1) { // +1 for length byte
                parser_state = PARSER_WAIT_CRC;
            }
            break;

        case PARSER_WAIT_CRC: {
            uint8_t crc = byte;
            uint8_t actual_crc = sensorbus_calc_crc8(parser_buffer, parser_index);
            parser_state = PARSER_WAIT_START; // Reset for next packet

            if (crc != actual_crc) {
                return SENSORBUS_ERR_CRC; // CRC mismatch
            }

            out_packet->msg_type = parser_buffer[1];
            out_packet->device_id  = (uint32_t)parser_buffer[2] << 24;
            out_packet->device_id |= (uint32_t)parser_buffer[3] << 16;
            out_packet->device_id |= (uint32_t)parser_buffer[4] <<  8;
            out_packet->device_id |= (uint32_t)parser_buffer[5] <<  0;
            out_packet->payload_len = parser_length - 5;
            memcpy(out_packet->payload, &parser_buffer[6], out_packet->payload_len);
            return SENSORBUS_OK; // Packet complete
        }
    }

    return SENSORBUS_IN_PROGRESS; // In progress
}

sensorbus_error_t sensorbus_receive_blocking(sensorbus_packet_t *out) {
    uint8_t b;
    sensorbus_error_t res;
    sensorbus_reset_parser();

    while (1) {
        // Block forever for the next byte
        res = sensorbus_hal_receive_byte(&b, 0);
        if (res != SENSORBUS_OK) {
            // fatal bus error
            return res;
        }
        res = sensorbus_parser_tick(b, out);
        if (res != SENSORBUS_IN_PROGRESS) {
            return res;
        }
    }
}

sensorbus_error_t sensorbus_receive_timeout(uint32_t timeout_ms,
                                           sensorbus_packet_t *out) {
    int64_t deadline = sensorbus_hal_get_time_ms() + timeout_ms;
    uint8_t b;
    sensorbus_error_t res;

    sensorbus_reset_parser();

    while (sensorbus_hal_get_time_ms() < deadline) {
        res = sensorbus_hal_receive_byte(&b, 10);
        if (res == SENSORBUS_ERR_TIMEOUT) {
            continue;
        }
        if (res != SENSORBUS_OK) {
            return res;
        }
        res = sensorbus_parser_tick(b, out);
        if (res != SENSORBUS_IN_PROGRESS) {
            return res;
        }
    }
    return SENSORBUS_ERR_TIMEOUT;
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