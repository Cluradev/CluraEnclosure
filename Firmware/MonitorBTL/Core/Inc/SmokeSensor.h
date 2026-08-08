/*
 * SmokeSensor.h
 *
 *  Created on: Feb 20, 2025
 *      Author: Fishbone
 */

#ifndef SRC_SMOKESENSOR_H_
#define SRC_SMOKESENSOR_H_

#include "stm32f4xx_hal.h"

class SmokeSensor {
private:
    GPIO_TypeDef *alarmPort;  // GPIO Port for Alarm
    uint16_t alarmPin;        // GPIO Pin for Alarm
    GPIO_TypeDef *mutePort;   // GPIO Port for Mute
    uint16_t mutePin;         // GPIO Pin for Mute

public:
    SmokeSensor(GPIO_TypeDef *alarmPort, uint16_t alarmPin, GPIO_TypeDef *mutePort, uint16_t mutePin);

    void begin();

    bool isSmokeDetected();

    void mute();

};


#endif /* SRC_SMOKESENSOR_H_ */
