/*
 * Output.h
 *
 *  Created on: Feb 23, 2025
 *      Author: Fishbone
 */

#ifndef SRC_OUTPUT_H_
#define SRC_OUTPUT_H_
#include "stm32f4xx_hal.h"

class Output {
private:
	GPIO_TypeDef *_port;
		 uint16_t _pin;
public:
	Output(GPIO_TypeDef *Port, uint16_t Pin);

	void begin();
	void high();
	void low();
	void toggle();
	// Beep function: count = number of beeps, delay = time between beeps (ms)
	void beep(uint8_t count, uint16_t delay);

};

#endif /* SRC_OUTPUT_H_ */
