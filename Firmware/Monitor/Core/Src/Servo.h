/*
 * Servo.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Fishbone
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_


#include "stm32f4xx_hal.h"

class Servo {
public:
    Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t minPulse = 5, uint32_t maxPulse = 20);

    void start();
    void stop();
    void moveTo(uint8_t angle, uint8_t speed = 1); // non-blocking
    void setAngle(uint8_t angle);                  // instant
    void update();
    void writeRaw(uint32_t pulse);

private:
    TIM_HandleTypeDef *m_htim;
    uint32_t m_channel;
    uint32_t m_minPulse;
    uint32_t m_maxPulse;

    uint32_t _previousTime;
    uint32_t _deltaT; // ms between updates

    uint8_t _currentAngle;
    uint8_t _targetAngle;
    uint8_t _steps;
};


#endif /* SRC_SERVO_H_ */
