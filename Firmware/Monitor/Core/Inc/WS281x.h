/*
 * WS281x.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Daniel Fishbone
 */

#ifndef INC_WS281X_H_
#define INC_WS281X_H_

#include "stm32f4xx_hal.h"

#define _numLEDs 100
#define _pwmDataSize  (250 + _numLEDs * 24 + 50)

class WS281x {
public:
    WS281x(TIM_HandleTypeDef* htim, uint32_t channel);
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
    void show();
    void clear();
private:
    void prepareData();

    TIM_HandleTypeDef* _htim;
    uint32_t _channel;
    uint8_t _ledData[_numLEDs*3];
    uint16_t _pwmData[_pwmDataSize];
};
#endif /* INC_WS281X_H_ */
