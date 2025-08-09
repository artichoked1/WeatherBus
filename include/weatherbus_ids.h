#pragma once

/*
 * weatherbus_ids.h
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
 * This file provides the IDs used to identify different devices and sensor types.
 */

#define SENSORBUS_SENSOR_TYPE_AIR_TEMPERATURE_CELCIUS 0x01
#define SENSORBUS_SENSOR_TYPE_HUMIDITY_PRH 0x02
#define SENSORBUS_SENSOR_TYPE_PRESSURE_PA 0x03
#define SENSORBUS_SENSOR_TYPE_WIND_SPEED_MPS 0x04
#define SENSORBUS_SENSOR_TYPE_WIND_DIRECTION_DEG 0x05
#define SENSORBUS_SENSOR_TYPE_CUMULATIVE_RAINFALL_MM 0x06
#define SENSORBUS_SENSOR_TYPE_SOLAR_RADIATION_W_M2 0x07
#define SENSORBUS_SENSOR_TYPE_UV_INDEX 0x08
#define SENSORBUS_SENSOR_TYPE_LIGHT_INTENSITY_LUX 0x09
#define SENSORBUS_SENSOR_TYPE_AIR_QUALITY_PPM 0x0A
#define SENSORBUS_SENSOR_TYPE_SOIL_MOISTURE_PERCENT 0x0B
#define SENSORBUS_SENSOR_TYPE_SOIL_TEMPERATURE_CELCIUS 0x0C
#define SENSORBUS_SENSOR_TYPE_CANOPY_TEMPERATURE_CELCIUS 0x0D
#define SENSORBUS_SENSOR_TYPE_WATER_TEMPERATURE_CELCIUS 0x0E
#define SENSORBUS_SENSOR_TYPE_WATER_LEVEL_CM 0x0F

#define SENSORBUS_VENDOR_ID_ARTICHOKE_TECHNOLOGIES 0x01

#define SENSORBUS_PRODUCT_ID_WS1_COORDINATOR 0x01
#define SENSORBUS_PRODUCT_ID_WS1_STEVENSON 0x02
#define SENSORBUS_PRODUCT_ID_WS1_RAIN_GAUGE 0x03
#define SENSORBUS_PRODUCT_ID_WS1_ANEMOMETER 0x04
#define SENSORBUS_PRODUCT_ID_WS1_WIND_VANE 0x05
#define SENSORBUS_PRODUCT_ID_WS1_UV_SENSOR 0x06
