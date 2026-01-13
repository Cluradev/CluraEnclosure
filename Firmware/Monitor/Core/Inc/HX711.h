/*
 * CLURA ENCLOSURE - RevB firmware
 */

#ifndef HX711_H
#define HX711_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h" // STM32 HAL library
#include <climits>
#ifdef __cplusplus
}
#endif

class HX711 {
private:
	GPIO_TypeDef *data_port;   // Port for data pin
	uint16_t data_pin;         // Data pin
	GPIO_TypeDef *clock_port;  // Port for clock pin
	uint16_t clock_pin;        // Clock pin
	uint32_t offset;           // Offset for tare weight
	float scale;               // Scale factor for calibration

public:
	// Constructor
	HX711(GPIO_TypeDef *data_port, uint16_t data_pin, GPIO_TypeDef *clock_port,
			uint16_t clock_pin);
	bool isTimedout = false;
	// Initialize the HX711
	void init();

	// Read raw data from the sensor
	int32_t readRaw();

	// Tare (set current weight as zero)
	void tare();

	// Set the scale factor for weight calibration
	void setScale(long scale);
	long getScale();
	void setOffset(long scale);
	long getOffset();

	// Get weight in grams
	long getWeight();
	bool calibrate(long known_weight);
	bool isReady();

private:
	void waitForReady();      // Helper method to wait for the HX711 to be ready

	int32_t readAverage(uint8_t times = 10);
};

#endif // HX711_H
