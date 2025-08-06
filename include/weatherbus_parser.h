/*
 * weatherbus_parser.h
 *
 * This file is part of the WeatherBus library.
 * Copyright (c) 2025 Nikolai Patrick
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * This file provides the main parser for incoming bytes.
 * It's stateful, so it takes one byte of the frame at a time and returns
 * the parsed packet when complete.
 */

#pragma once
#include "weatherbus_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The main parser for incoming bytes.
 * It is stateful, so it takes one byte of the frame at a time and returns the parsed packet when complete.
 * 
 * @param byte 
 * @param out_packet 
 * @return SENSORBUS_OK on success, SENSORBUS_IN_PROGRESS if more bytes are needed,
 *         or an error code (e.g., SENSORBUS_ERR_CRC, SENSORBUS_ERR_FORMAT). 
 */
sensorbus_error_t sensorbus_parser_tick(uint8_t byte, sensorbus_packet_t* out_packet);

/**
 * @brief It resets the parser state.
 * 
 */
void sensorbus_reset_parser(void);

#ifdef __cplusplus
}
#endif
