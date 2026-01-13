/*
 * debug_utils.c
 *
 *  Created on: Feb 20, 2025
 *      Author: Fishbone
 */




#include "stm32f4xx_hal.h"
#include <string.h>

#define DEBUG_FLAG 0
extern UART_HandleTypeDef huart2;

void debug_print(const char *msg) {
	if(DEBUG_FLAG ==1){
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
	return;
}
