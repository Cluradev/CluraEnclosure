/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "SmokeSensor.h"

SmokeSensor::SmokeSensor(GPIO_TypeDef *aPort, uint16_t aPin, GPIO_TypeDef *mPort, uint16_t mPin)
    : alarmPort(aPort), alarmPin(aPin), mutePort(mPort), mutePin(mPin) {}

void SmokeSensor::begin() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = alarmPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(alarmPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = mutePin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(mutePort, &GPIO_InitStruct);
}

// Check Smoke
bool SmokeSensor::isSmokeDetected() {
    return HAL_GPIO_ReadPin(alarmPort, alarmPin) == GPIO_PIN_SET;  // High = Smoke detected
}

// Mute Alarm
void SmokeSensor::mute() {
    HAL_GPIO_WritePin(mutePort, mutePin, GPIO_PIN_RESET);  // Low Level is Mute
    HAL_Delay(5000);  // wait 5 seconds
    HAL_GPIO_WritePin(mutePort, mutePin, GPIO_PIN_SET);  // Release for unMute
}
