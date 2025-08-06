if(ESP_PLATFORM)
    set(SOURCES
        "src/weatherbus_crc.c"
        "src/weatherbus_parser.c"
        "src/weatherbus_payload.c"
        "src/weatherbus_transport.c"
    )

    idf_component_register(SRCS ${SOURCES}
                           INCLUDE_DIRS "include"
                           REQUIRES driver ESP-RS485 esp_timer)
endif()