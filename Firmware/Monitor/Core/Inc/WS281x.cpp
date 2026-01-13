/*
 * WS281x.cpp
 *
 *  Created on: Jun 10, 2025
 *      Author: Daniel Fishbone
 */

#include <WS281x.h>
#include <cstring> // for memset

// Timings in timer counts (adjust for your clock frequency)

#define WS281x_0_HIGH  37     // 0.35 µs high (CCR for “0” bit)
#define WS281x_1_HIGH  88     // 0.7 µs high  (CCR for “1” bit)
#define WS281x_PERIOD 124     // 1.25 µs period (ARR)
volatile bool ws281x_done = false;
WS281x::WS281x(TIM_HandleTypeDef* htim, uint32_t channel)
    : _htim(htim), _channel(channel){
//    _ledData = new uint8_t[numLEDs * 3]; // RGB for each LED
//    _pwmDataSize = 250+(numLEDs * 24) + 50;    // 24 bits per LED + reset
//    _pwmData = new uint16_t[_pwmDataSize];
    clear();
}

void WS281x::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
    if (n < _numLEDs) {
        _ledData[n * 3]     = g; // WS281x uses GRB order
        _ledData[n * 3 + 1] = r;
        _ledData[n * 3 + 2] = b;
    }
}

void WS281x::clear() {
    memset(_ledData, 0, _numLEDs * 3);
}

void WS281x::prepareData() {
    uint32_t idx = 0;

    //  Add initial reset pulse (long LOW before data)
    for (uint8_t i = 0; i < 250; i++) {   // 250 * 1.25 µs = ~312 µs
        _pwmData[idx++] = 0;
    }

    //  encode the actual LED data
    for (uint16_t i = 0; i < _numLEDs * 3; i++) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            if (_ledData[i] & (1 << bit)) {
                _pwmData[idx++] = WS281x_1_HIGH;
            } else {
                _pwmData[idx++] = WS281x_0_HIGH;
            }
        }
    }

    //  Add final reset pulse (long LOW after data)
    while (idx < _pwmDataSize) {
        _pwmData[idx++] = 0;
    }
}



void WS281x::show() {
    // 1. Set pin back to alternate function (before sending data)
	prepareData();
	ws281x_done = false;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8; // your actual pin
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;  // <-- Make sure this matches CubeMX!
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);     // your actual port





    HAL_TIM_PWM_Start_DMA(_htim, _channel, (uint32_t*)_pwmData, _pwmDataSize);
    // Wait for DMA transfer complete
//    while (HAL_DMA_GetState(_htim->hdma[TIM_DMA_ID_CC1]) != HAL_DMA_STATE_READY) {}
//    HAL_TIM_PWM_Stop_DMA(_htim, _channel);
    while (!ws281x_done) {
            // Optional: add a timeout
        }

}

