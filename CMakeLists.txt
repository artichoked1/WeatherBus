if(ESP_PLATFORM)
idf_component_register(SRCS "PayloadBuilder.c" "WeatherBus.c" "./hal/esp_idf_hal.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver ESP-RS485 esp_timer)
    return()
endif()