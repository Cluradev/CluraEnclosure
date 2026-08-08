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
#include "bme680/bme68x_necessary_functions.h"
#include "../debug_utils.h"

// I2C handler
I2C_HandleTypeDef BME68x_I2C_Handler; // I2C peripheral for the device.

// BME68x device address:
uint8_t dev_addr = BME68X_I2C_ADDR_HIGH; // High is 0x77 and low is 0x76

// BME68x Variables
struct bme68x_dev bme;
struct bme68x_data * BME68x_DATA;
int8_t rslt;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
uint32_t del_period;
uint8_t n_fields;

// IAQ Calculation Variables
float humidity_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int8_t getgasreference_count = 0;
float gas_lower_limit = 5000;   // Bad air quality limit
float gas_upper_limit = 50000;  // Good air quality limit

/* Complete init. function. */
int8_t bme68x_start(struct bme68x_data *dataPtr, I2C_HandleTypeDef *handler) {

    debug_print("bme68x_start: Begin Init...\n");

	// I2C handler copy
	memcpy(&BME68x_I2C_Handler, handler, sizeof(*handler));
    debug_print("bme68x_start: I2C handler copied\n");

	// Init.
    int8_t result =bme68x_interface_init(&bme, BME68X_I2C_INTF);
    if (result != 0) {
           debug_print("bme68x_start: Interface Init FAILED!\n");
           return result;
       }
       debug_print("bme68x_start: Interface Init SUCCESS\n");

       result =bme68x_init(&bme);
       if (result != 0) {
               debug_print("bme68x_start: Sensor Init FAILED!\n");
               return result;
           }
           debug_print("bme68x_start: Sensor Init SUCCESS\n");


	// Init. for data variable
	BME68x_DATA = dataPtr;

	// Configuration
	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	conf.filter = BME68X_FILTER_SIZE_3;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_2X;
	conf.os_pres = BME68X_OS_4X;
	conf.os_temp = BME68X_OS_8X;
	result =bme68x_set_conf(&conf, &bme);
	if (result != 0) {
	        debug_print("bme68x_start: Config FAILED!\n");
	        return result;
	    }
	    debug_print("bme68x_start: Config SUCCESS\n");

	// Heat conf.
	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = 320;
	heatr_conf.heatr_dur = 150;
	rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

	 if (rslt != 0) {
	        debug_print("bme68x_start: Heater Config FAILED!\n");
	        return rslt;
	    }
	    debug_print("bme68x_start: Heater Config SUCCESS\n");
	// Gather gas reference for the IAQ calculation
	bme68x_GetGasReference();

    debug_print("bme68x_start: Init Complete!\n");

	return rslt;
}

/* Force mode measurement. */
int8_t bme68x_single_measure(struct bme68x_data *dataPtr) {

	bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

	/* Calculate delay period in microseconds */
	del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme)
			+ (heatr_conf.heatr_dur * 1000);
	bme.delay_us(del_period, bme.intf_ptr);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	rslt = bme68x_get_data(BME68X_FORCED_MODE, dataPtr, &n_fields, &bme);

	return rslt;
}

/* Necessary functions. */
// I2C write function.
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;

	if (HAL_I2C_Mem_Write(&BME68x_I2C_Handler, (uint16_t) (dev_addr << 1), reg_addr, 1,
			(uint8_t*) reg_data, len, 15) == HAL_OK)
		return 0;

	return 1;
}

// I2C read function.
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;

	if (HAL_I2C_Mem_Read(&BME68x_I2C_Handler, (uint16_t) ((dev_addr << 1) | 0x1), reg_addr,
			1, reg_data, len, 15) == HAL_OK)
		return 0;

	return 1;
}

// BME68x delay function
void bme68x_delay_us(uint32_t period, void *intf_ptr) {
	HAL_Delay(period / 1000);
}

// BME68x interface function
int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf) {
	int8_t rslt = BME68X_OK;

	if (bme != NULL) {

		// Check for the device on the I2C line
		if (HAL_I2C_IsDeviceReady(&BME68x_I2C_Handler, (uint16_t) (dev_addr << 1), 5, 5)
				== HAL_OK) {
			// Device found at the I2C line.
			rslt = 0;
		} else {
			rslt = -2; // Communication error.
			return rslt;
		}

		/* Bus configuration : I2C */
		if (intf == BME68X_I2C_INTF) {
			bme->read = bme68x_i2c_read;
			bme->write = bme68x_i2c_write;
			bme->intf = BME68X_I2C_INTF;
		} else {
			return -2;
		}

		bme->delay_us = bme68x_delay_us;
		bme->intf_ptr = &dev_addr;
		bme->amb_temp = 30; /* The ambient temperature in deg C is used for defining the heater temperature */
	} else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

/*
 This substantial portion of IAQ calculation is shared with the written permission from the author David Bird.
 This substantial portion of IAQ calculation, the ideas and concepts is Copyright (c) David Bird 2018. All rights to this substantial portion are reserved.

 Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
 1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
 2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
 3. You may not, except with my express written permission, distribute or commercially exploit the content.
 4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.
 The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
 software use is visible to an end-user.

 THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
 OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 See more at http://www.dsbird.org.uk
*/

/* IAQ functions */
void bme68x_GetGasReference() {
	// Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.

	int readings = 10;
	for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
		bme68x_single_measure(BME68x_DATA);
		gas_reference += BME68x_DATA->gas_resistance;
	}
	gas_reference = gas_reference / readings;

}

//Calculate humidity contribution to IAQ index
int8_t bme68x_GetHumidityScore() {

	if (BME68x_DATA->humidity >= 38 && BME68x_DATA->humidity <= 42) // Humidity +/-5% around optimum
		humidity_score = 0.25 * 100;
	else { // Humidity is sub-optimal
		if (BME68x_DATA->humidity < 38)
			humidity_score = 0.25 / hum_reference * BME68x_DATA->humidity * 100;
		else {
			humidity_score = ((-0.25 / (100 - hum_reference)
					* BME68x_DATA->humidity) + 0.416666) * 100;
		}
	}

	return humidity_score;
}

//Calculate gas contribution to IAQ index
int8_t bme68x_GetGasScore() {

	gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference
			- (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit))))
			* 100.00;
	if (gas_score > 75)
		gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
	if (gas_score < 0)
		gas_score = 0; // Sometimes gas readings can go outside of expected scale minimum

	return gas_score;
}

float bme68x_iaq() {

	float air_quality_score = (100
			- (bme68x_GetHumidityScore(BME68x_DATA)
					+ bme68x_GetGasScore(BME68x_DATA))) * 5;

	// If 5 measurements passed, recalculate the gas reference.
	if ((getgasreference_count++) % 5 == 0)
		bme68x_GetGasReference(BME68x_DATA);

	return air_quality_score;

}
