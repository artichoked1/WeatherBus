#include "WeatherBus.h"

sensorbus_error_t pb_init(payload_builder_t* pb) {
    pb->len = 0;
    return SENSORBUS_OK;
}

sensorbus_error_t pb_add_tlv(payload_builder_t* pb, uint8_t type, uint8_t index, const void* data, uint8_t data_len) {

    // Check if the payload is too big
    if ((uint8_t)(pb->len + 3 + data_len) > SENSORBUS_MAX_PAYLOAD) {
        return SENSORBUS_ERR_FORMAT;
    }

    // Add the TLV record, or in this case TILV.
    pb->buf[pb->len++] = type;
    pb->buf[pb->len++] = index;
    pb->buf[pb->len++] = data_len;
    if (data_len > 0 && data) {
        memcpy(&pb->buf[pb->len], data, data_len);
    }
    pb->len += data_len;

    return SENSORBUS_OK;
}

sensorbus_error_t pb_decode_tlv(const uint8_t* buf,uint8_t buf_len,sensorbus_sensor_t* out,size_t* out_count) {
    size_t count = 0;
    uint8_t pos = 0;

    while (pos + 3 <= buf_len && count < SENSORBUS_MAX_TLVS) {
        uint8_t t = buf[pos++];
        uint8_t i = buf[pos++];
        uint8_t L = buf[pos++];
        if (pos + L > buf_len) {
            return SENSORBUS_ERR_FORMAT;
        }
        out[count].type  = t;
        out[count].index = i;
        out[count].len   = L;
        out[count].value = (L > 0 ? &buf[pos] : NULL);
        pos += L;
        count++;
    }
    if (pos != buf_len){
        return SENSORBUS_ERR_FORMAT;
    }
    *out_count = count;
    return SENSORBUS_OK;
}

sensorbus_error_t pb_add_descriptor(payload_builder_t* pb,uint8_t type,uint8_t index) {
    return pb_add_tlv(pb, type, index, NULL, 0);
}

sensorbus_error_t pb_add_float(payload_builder_t* pb, uint8_t type, uint8_t index, float value) {
    return pb_add_tlv(pb, type, index, &value, sizeof(value));
}

sensorbus_error_t pb_add_error(payload_builder_t* pb, uint8_t type, uint8_t index) {
    const uint8_t err_val[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    return pb_add_tlv(pb, type, index, err_val, sizeof(err_val));
}

sensorbus_error_t pb_add_query(payload_builder_t *pb, sensorbus_sensor_t *sensor){
    // each query is exactly 2 bytes
    if (pb->len + 2 > SENSORBUS_MAX_PAYLOAD) {
        return SENSORBUS_ERR_FORMAT;
    }
    pb->buf[pb->len++] = sensor->type;
    pb->buf[pb->len++] = sensor->index;
    return SENSORBUS_OK;
}