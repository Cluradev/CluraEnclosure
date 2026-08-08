/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "A_SmokeSensor.h"
#include <stdlib.h>

A_SmokeSensor::A_SmokeSensor(GPIO_TypeDef *port, uint16_t pin,
		ADC_HandleTypeDef *adcHandle, uint32_t adcChannel, uint16_t threshold,
		int tolerance) :
		_port(port), _pin(pin), _adc(adcHandle), _channel(adcChannel), _threshold(
				threshold), _tolerance(tolerance) {
	_lastValue = 0;
}

void A_SmokeSensor::begin() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(_port, &GPIO_InitStruct);
}

uint32_t A_SmokeSensor::readRaw() {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = _channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	uint32_t value = 0;
	/* D-18: was 100 blocking conversions per call, for BOTH sensors, on EVERY
	 * loop pass. 64 keeps most of the noise rejection at a third less cost;
	 * the caller also now rates-limits how often this runs. */
	int sampleCount = 64;
	HAL_ADC_ConfigChannel(_adc, &sConfig);

	for (int i = 0; i < sampleCount; ++i) {
		HAL_ADC_Start(_adc);
		HAL_ADC_PollForConversion(_adc, HAL_MAX_DELAY);
		value += HAL_ADC_GetValue(_adc);
		HAL_ADC_Stop(_adc);

	}
	value /= sampleCount;
	int diff = (int) value - _lastValue;
	if (abs(diff) >= this->_tolerance) {
		_lastValue = value;
	}
	return _lastValue;
}

bool A_SmokeSensor::isSmokeDetected() {
	return readRaw() > _threshold;
}

A_SmokeSensor::~A_SmokeSensor() {
}
