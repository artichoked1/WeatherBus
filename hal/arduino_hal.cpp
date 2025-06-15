#include "WeatherBus.h"
#include "RS485.h"

rs485_uart_t uart_dev = {
    .serial_port = &Serial2, // Use Serial2 for RS485 communication
    .de_pin = 23,            // DE pin for RS485
    .re_pin = 22,            // RE pin for RS485
    .baud_rate = 9600        // Baud rate for RS485 communication
};

sensorbus_error_t sensorbus_hal_uart_init() {
    rs485_init(&uart_dev);
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_send_bytes(uint8_t* data, size_t len) {
    rs485_begin_transmission(&uart_dev);
    rs485_write_buffer(&uart_dev, data, len);
    rs485_end_transmission(&uart_dev);
    return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
      if (rs485_available(&uart_dev)) {
          *byte = rs485_read(&uart_dev);
          return SENSORBUS_OK;
      }
  }
  return SENSORBUS_ERR_TIMEOUT;
}

void sensorbus_hal_delay_ms(uint32_t ms) {
    delay(ms);
}