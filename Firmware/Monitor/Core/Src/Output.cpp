/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "Output.h"

Output::Output(GPIO_TypeDef *Port, uint16_t Pin) : _port(Port), _pin(Pin) {

}

void Output::begin() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(_port, &GPIO_InitStruct);
}
void Output::high() {
    HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
}

void Output::low() {
    HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
}

void Output::toggle() {
    HAL_GPIO_TogglePin(_port, _pin);
}

void Output::beep(uint8_t count, uint16_t delay) {
    for (uint8_t i = 0; i < count; i++) {
        high();
        HAL_Delay(delay);
        low();
        HAL_Delay(delay);
    }
}
