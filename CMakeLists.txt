if(ESP_PLATFORM)
idf_component_register(SRCS "WeatherBus.c" "./hal/esp_idf_hal.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver ESP-RS485)
    return()
endif()