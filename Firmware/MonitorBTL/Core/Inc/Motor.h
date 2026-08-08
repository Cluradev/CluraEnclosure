/*
 * Motor.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Fishbone
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f4xx_hal.h"  // Adjust include as needed for your device

class Motor {
public:
    /**
     * @brief Constructor for the Motor class.
     * @param htim Pointer to the timer handle already configured for PWM.
     * @param pwmChannel Timer channel for PWM output (e.g., TIM_CHANNEL_1).
     * @param tachPort (Optional) GPIO port for the tachometer signal. Pass nullptr if not used.
     * @param tachPin  (Optional) GPIO pin for the tachometer signal. Ignored if tachPort is nullptr.
     */
	bool mode;
    Motor(TIM_HandleTypeDef* htim,
          uint8_t pwmChannel,
          GPIO_TypeDef* tachPort = nullptr,
          uint16_t tachPin = 0)
        : mode(0),
          _htim(htim),
          _pwmChannel(pwmChannel),
          _tachPort(tachPort),
          _tachPin(tachPin)
    {        // Start PWM on the given timer channel in the main script, for some reason it doesnt work here .
//    		HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);

    }

    int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
        // Linearly map x from [in_min, in_max] to [out_min, out_max]
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    void setSpeed(uint8_t targetSpeed) {
        if (targetSpeed > 100) {
            targetSpeed = 100;


        }
        /* D-06: full scale used to be hard-coded to 39, which is correct for
         * TIM3 (ARR 39) but wrong for TIM2 (ARR 255) - the extra fan was
         * therefore capped at 39/256 = ~15% duty no matter what was asked for.
         * Read the timer's real auto-reload instead so every motor scales
         * correctly regardless of which timer drives it. */
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_htim);
        uint32_t pwmCompareValue = ((uint32_t)targetSpeed * arr) / 100u;
// Had to use this her the TIM_CHANNEL macro didn't work when passed to the function
    	switch (_pwmChannel){
        	case 1:
                __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, pwmCompareValue);

        		break;
        	case 2:
                __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, pwmCompareValue);
    			break;
        	case 3:
                __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_3, pwmCompareValue);
    			break;
        	case 4:
                __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_4, pwmCompareValue);
    			break;
        	}
    }


    uint32_t readTachometer() {
        if (_tachPort == nullptr) {
            // Not configured tachometer yet.
             return 0;
        }
        return 0;
    }

private:
    TIM_HandleTypeDef* _htim;   // Handle for the timer used for PWM.
    uint32_t _pwmChannel;       // Timer channel for PWM output use 1,2,3or 4 for some reason the actual timer macro doesnt work.
    GPIO_TypeDef* _tachPort;    // Optional tachometer GPIO port.
    uint16_t _tachPin;          // Optional tachometer GPIO pin.
};



#endif /* MOTOR_H_ */
