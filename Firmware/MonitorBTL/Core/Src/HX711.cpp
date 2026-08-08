/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "HX711.h"
#include "parameters.h"   /* NOT_FOUND */

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
    int32_t raw_value = readAverage(HX711_CAL_SAMPLES);

    /* Amplifier stopped responding mid-calibration - do not derive a factor
     * from an error value. */
    if ((uint32_t)raw_value == 0xFFFFFFFFu) {
        return 0;
    }

    /* Signed difference for the same reason as getWeight(), and a FLOAT
     * division - the old expression divided two longs and only then assigned
     * the already-truncated result to the float member. */
    int32_t delta  = raw_value - (int32_t)this->offset;
    float   factor = (float)delta / (float)known_weight;

    /* getScale() persists this as a long, so anything under 1 count/gram
     * rounds to 0 and would make getWeight() divide by zero on the next boot.
     * Reject such a calibration rather than storing a broken factor. */
    if (factor > -1.0f && factor < 1.0f) {
        return 0;
    }

    /* Set BOTH ends of the linear fit, not just the slope.
     *
     * The slope is persisted as a long (getScale()), so quantise it here and
     * then re-derive the intercept from the QUANTISED value. Otherwise the
     * stored pair no longer passes through the calibration point once the
     * slope has been rounded, and that rounding error is multiplied by the
     * full known weight. Doing it this way makes weight(raw_value) come back
     * as exactly known_weight. */
    long qScale = (long)factor;

    this->scale  = (float)qScale;
    this->offset = (uint32_t)(raw_value - (int32_t)(qScale * known_weight));

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
	/* D-15: the 25 clock pulses below are timing-critical. A UART, tacho or
	 * SysTick interrupt landing mid-read corrupts the sample, and stretching a
	 * clock-high beyond ~60 us puts the HX711 into power-down. The window is
	 * only ~30 us, so masking interrupts here is cheap. */
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

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

	__set_PRIMASK(primask);   /* end of the timing-critical window */

	// Convert the data to signed 32-bit
	count = count ^ 0x800000;
	return count;
}

// Tare (set current weight as zero)
void HX711::tare() {
	/* Average the zero point instead of taking a single raw sample. The slope
	 * has always been averaged; leaving the intercept as one noisy reading made
	 * the zero unstable, and that error shifts EVERY subsequent weight. */
	int32_t v = readAverage(HX711_CAL_SAMPLES);

	/* Keep the previous offset if the amplifier did not respond - overwriting
	 * it with the error value would wreck every later reading. */
	if ((uint32_t)v != 0xFFFFFFFFu) {
		this->offset = (uint32_t)v;
	}
}
int32_t HX711::readAverage(uint8_t times) {
    if (times == 0) times = 1;

    int64_t  sum  = 0;
    uint16_t good = 0;

    for (uint8_t i = 0; i < times; i++) {
        int32_t v = readRaw();

        /* A failed read already burned waitForReady()'s full 2 s timeout, and
         * it returns 0xFFFFFFFF - which the old code happily summed into the
         * average. Stop at the first failure: continuing would cost
         * `times` x 2 s (over two minutes for a disconnected cell) and produce
         * a meaningless result anyway. */
        if (this->isTimedout || (uint32_t)v == 0xFFFFFFFFu) {
            break;
        }

        sum += v;
        good++;

        /* No HAL_Delay here: waitForReady() inside readRaw() already blocks
         * until the next conversion is ready, so an extra delay was pure dead
         * time on every single sample. */
    }

    if (good == 0) {
        return (int32_t)0xFFFFFFFF;   /* propagate the failure to the caller */
    }
    return (int32_t)(sum / good);
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
	    /* D-19: amplifier not responding. Was LONG_MIN, which no caller
	     * recognised; NOT_FOUND is the sentinel the rest of the firmware
	     * already understands and renders. */
	    return NOT_FOUND;
	}
	/* Both raw_data and offset are uint32_t, so subtracting them directly was
	 * UNSIGNED arithmetic: any reading below the tare point (thermal drift, or
	 * simply lifting the spool off) wrapped to ~4.29e9 instead of going
	 * negative. Cast to signed first - the values are 24-bit, so int32 is
	 * ample. */
	int32_t delta = (int32_t)raw_data - (int32_t)this->offset;

	/* A zero scale would divide by zero. Reachable from a bad calibration
	 * whose factor truncates to 0 when persisted as a long. */
	if (this->scale == 0.0f) {
	    return NOT_FOUND;
	}

	long w = (long)((float)delta / this->scale);

	/* D-19: an out-of-range reading used to be reported as 0 g, which is
	 * indistinguishable from "nothing on the holder". Report it as an error
	 * instead so the UI can show that the reading is not trustworthy. */
	return (w < 5000) ? w : NOT_FOUND;
}
