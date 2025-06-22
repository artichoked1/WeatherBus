if(ESP_PLATFORM)
    set(SOURCES
        "src/PayloadBuilder.c"
        "src/WeatherBus.c"
        "src/hal/esp_idf_hal.c"
    )

    idf_component_register(SRCS ${SOURCES}
                           INCLUDE_DIRS "include"
                           REQUIRES driver ESP-RS485 esp_timer)
endif()