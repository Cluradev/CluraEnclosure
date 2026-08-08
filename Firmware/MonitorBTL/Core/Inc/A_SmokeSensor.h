/*
 * A_SmokeSensor.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Fishbone
 */



#ifndef A_SMOKESENSOR_H
#define A_SMOKESENSOR_H

#include "stm32f4xx_hal.h"
#define DEFAULT_SMOKE_THRESHOLD 1500
#define DEFAULT_SMOKE_TOLERANCE 5

class A_SmokeSensor {
public:
    A_SmokeSensor(GPIO_TypeDef *port, uint16_t pin, ADC_HandleTypeDef *adcHandle, uint32_t adcChannel, uint16_t threshold = DEFAULT_SMOKE_THRESHOLD,int tolerance=DEFAULT_SMOKE_TOLERANCE);
    void begin();
    uint32_t readRaw();
    bool isSmokeDetected();
    ~A_SmokeSensor();

private:
    GPIO_TypeDef *_port;
    uint16_t _pin;
    ADC_HandleTypeDef *_adc;
    uint32_t _channel;
    uint16_t _threshold;
    uint32_t _lastValue;
    int _tolerance;
};

#endif // A_SMOKESENSOR_H
