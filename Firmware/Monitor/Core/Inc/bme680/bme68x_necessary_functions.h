/*
 * Library Name: 	BME68x (I2C) - STM32 HAL and BME68x API Based Library
 * Written By:		Ahmet Batuhan Günaltay (Great thanks to David Bird for IAQ calculation method and functions.)
 * Date Written:	26/05/2021 (DD/MM/YYYY)
 * Last Modified:	26/05/2021 (DD/MM/YYYY)
 * Description:		This library measures temperature, humidity, pressure and IAQ with BME680x (I2C) sensors based on STM32 HAL
 * 					and BME68x API. For more information about the IAQ calculation check: https://github.com/G6EJD/BME680-Example
 * References:
 * 	- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf [Datasheet]
 *
 * Copyright (C) 2021 - Ahmet Batuhan Günaltay
 * (IAQ calculation substantial portion, the ideas and concepts is Copyright (c) David Bird 2018)
 *
	This software library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This software library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  */

/* Necessary functions for BME68x API are defined here. (Functions that are hardware dependent.) */

/* Libraries */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "bme68x.h"
#include "bme68x_defs.h"
#include "stdio.h"
#include "string.h"

/* Function prototypes */
// Complete initialisation and initial configuration of the I2C device. One function to configure all and then enables us to take measurements.
int8_t bme68x_start(struct bme68x_data *dataPtr, I2C_HandleTypeDef *handler);

// Forced mode measurement are taken with this function. Single-shot forced readings from the sensor.
int8_t bme68x_single_measure(struct bme68x_data *dataPtr);

// Get IAQ function.
float bme68x_iaq();

// I2C write function for BME68x API.
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

// I2C read function for BME68x API.
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

// Configuration function for BME68x API.
int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf);

// Sleep function for gas heating process. (Required by BME68x API)
void bme68x_delay_us(uint32_t period, void *intf_ptr);

/* Necessary IAQ calculation functions. */
void bme68x_GetGasReference();
int8_t bme68x_GetHumidityScore();
int8_t bme68x_GetGasScore();

#ifdef __cplusplus
}
#endif
