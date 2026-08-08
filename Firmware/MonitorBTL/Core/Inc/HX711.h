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

/* Samples averaged when establishing a calibration point (both the zero/tare
 * intercept and the known-weight slope).
 *
 * Each sample costs a full conversion period - about 110 ms at the HX711's
 * ~10 SPS - and the main loop is blocked throughout, so this number is felt
 * directly by the operator: 16 samples is ~1.8 s per cell, 64 was ~7 s, and
 * the original 255 was nearly half a minute.
 *
 * 16 is ample. HX711 noise is on the order of +-50 counts, which at ~740
 * counts/gram is ~0.07 g; averaging 16 samples cuts that by 4x, far below
 * anything that matters for weighing spools. */
#define HX711_CAL_SAMPLES 16

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
