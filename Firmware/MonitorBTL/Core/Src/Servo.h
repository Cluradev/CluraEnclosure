/*
 * Servo.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Fishbone
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_


#include "stm32f4xx_hal.h"

// Minimal servo driver. TIM5 is configured for 1 count = 1 us, so minPulse/maxPulse
// (microseconds) map 1:1 to the compare register. Angles are assumed in range (0..180).
// The detach/hold timing lives in applyServoControl(), not here.
class Servo {
public:
    Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t minPulse = 500, uint32_t maxPulse = 2000);

    void start();                  // start PWM output
    void stop();                   // stop PWM output
    void setAngle(uint8_t angle);  // drive the pulse for 'angle'
    void detach();                 // cut the pulse (compare = 0); servo relaxes
    void writeRaw(uint32_t pulse); // write the compare register directly (microseconds)

private:
    TIM_HandleTypeDef *m_htim;
    uint32_t m_channel;
    uint32_t m_minPulse;
    uint32_t m_maxPulse;
};


#endif /* SRC_SERVO_H_ */
