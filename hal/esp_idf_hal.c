#include "WeatherBus.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RS485.h"
#include "driver/gpio.h"

#define UART_PORT   UART_NUM_1
#define TX_PIN      GPIO_NUM_17
#define RX_PIN      GPIO_NUM_16
#define DE_PIN      GPIO_NUM_18
#define RE_PIN      GPIO_NUM_19
#define BAUD_RATE   9600

rs485_uart_t uart_dev = {
    .uart_port = UART_NUM_1,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_16,
    .de_pin = GPIO_NUM_18,
    .re_pin = GPIO_NUM_19,
    .baud_rate = 9600
};

void sensorbus_hal_uart_init() {
    if (rs485_uart_init(&uart_dev) != ESP_OK) {
        printf("Failed to initialise RS485\n");
        return;
    }
}

void sensorbus_hal_send_bytes(const uint8_t* data, size_t len) {
    rs485_uart_write(&uart_dev, data, len);
}

int sensorbus_hal_receive_byte(uint8_t* byte, uint32_t timeout_ms) {
    return rs485_uart_read(&uart_dev, byte, 1, pdMS_TO_TICKS(timeout_ms));
}

void sensorbus_hal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
