#pragma once

/*
 * weatherbus_crc.h
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
 * This file provides the CRC utility functions for the protocol.
 */

#include "weatherbus_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate the CRC-8 checksum for a given data buffer.
 * 
 * @param data Pointer to the data buffer.
 * @param len Length of the data buffer in bytes.
 * @return The calculated CRC-8 checksum.
 */
uint8_t sensorbus_calc_crc8(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif