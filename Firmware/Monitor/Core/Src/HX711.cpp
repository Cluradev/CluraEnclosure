/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "HX711.h"

#include "debug_utils.h"


// Constructor
HX711::HX711(GPIO_TypeDef *data_port, uint16_t data_pin,
		GPIO_TypeDef *clock_port, uint16_t clock_pin) :
		data_port(data_port), data_pin(data_pin), clock_port(clock_port), clock_pin(
				clock_pin), offset(0), scale(1.0){
}

// Initialize the HX711
void HX711::init() {
	// Configure the data pin as input (pull-up mode)
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = data_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(data_port, &GPIO_InitStruct);

	// Configure the clock pin as output
	GPIO_InitStruct.Pin = clock_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(clock_port, &GPIO_InitStruct);
}
bool HX711::isReady() {
    return !HAL_GPIO_ReadPin(data_port, data_pin);
}
// Wait for HX711 to be ready
void HX711::waitForReady() {
	uint32_t timeout = 2000;
	uint32_t timestamp = HAL_GetTick();
	while (HAL_GPIO_ReadPin(data_port, data_pin) == GPIO_PIN_SET) {
		this->isTimedout=false;
		HAL_Delay(10); // Wait until the data line goes low
		if(HAL_GetTick()-timestamp >=timeout)
		{
			this->isTimedout=true;
			break;
		}
	}
}
bool HX711::calibrate(long known_weight) {
    if (known_weight <= 0) {
        return 0; // Avoid division by zero
    }

//    tare();
    HAL_Delay(500);
    if(!this->isTimedout){
    uint32_t raw_value = readAverage(255);
    this->scale = (long)(raw_value - this->offset) / known_weight;
    return 1;
}
    return 0;
}
// Read raw data from the HX711
int32_t HX711::readRaw() {

	waitForReady();
	if (this->isTimedout) {
	        return 0xFFFFFFFF;
	    }
	int32_t count = 0;
	for (int i = 0; i < 24; ++i) {
		// Pulse the clock pin
		HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_SET);
		__NOP(); // Short delay
		count = count << 1;
				if (HAL_GPIO_ReadPin(data_port, data_pin)) count++;
				HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_RESET);

	}

	// Set the clock pin for the 25th pulse
	HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_SET);
	__NOP();
	HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_RESET);

	// Convert the data to signed 32-bit
	count = count ^ 0x800000;
	return count;
}

// Tare (set current weight as zero)
void HX711::tare() {
	this->offset = readRaw();
}
int32_t HX711::readAverage(uint8_t times) {
    uint64_t sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += readRaw();
        HAL_Delay(10);
    }
    return (uint32_t)(sum / times);
}
// Set the scale factor
void HX711::setScale(long _scale) {
	this->scale = _scale;
}
long  HX711::getScale() {
	return (long)this->scale;
}
void HX711::setOffset(long _offset) {
	this->offset = _offset;
}
long  HX711::getOffset() {
	return (long)this->offset;
}
// Get the weight in whatever units
long HX711::getWeight() {
	uint32_t raw_data = readRaw();
	if (raw_data == 0xFFFFFFFF) {
	    // Handle error: amplifier not connected
	    return LONG_MIN ;
	}
	long w = (long)(((float)(raw_data - this->offset)) / this->scale);
	return w <5000?w:0;
}
