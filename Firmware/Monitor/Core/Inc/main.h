/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DET_Pin GPIO_PIN_13
#define DET_GPIO_Port GPIOC
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOA
#define TACHO_A_Pin GPIO_PIN_1
#define TACHO_A_GPIO_Port GPIOA
#define TACHO_A_EXTI_IRQn EXTI1_IRQn
#define SCREEN_RX_Pin GPIO_PIN_2
#define SCREEN_RX_GPIO_Port GPIOA
#define SCREEN_TX_Pin GPIO_PIN_3
#define SCREEN_TX_GPIO_Port GPIOA
#define HX1SCK_Pin GPIO_PIN_4
#define HX1SCK_GPIO_Port GPIOA
#define HX1DT_Pin GPIO_PIN_5
#define HX1DT_GPIO_Port GPIOA
#define MEMS_SMOKE_Pin GPIO_PIN_6
#define MEMS_SMOKE_GPIO_Port GPIOA
#define MQ2_Pin GPIO_PIN_7
#define MQ2_GPIO_Port GPIOA
#define pwm_60_Pin GPIO_PIN_0
#define pwm_60_GPIO_Port GPIOB
#define pwm_140_Pin GPIO_PIN_1
#define pwm_140_GPIO_Port GPIOB
#define pwm_reg_Pin GPIO_PIN_10
#define pwm_reg_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_13
#define CLK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define LED_PIN_Pin GPIO_PIN_8
#define LED_PIN_GPIO_Port GPIOA
#define PMS_RX_Pin GPIO_PIN_9
#define PMS_RX_GPIO_Port GPIOA
#define PMS_TX_Pin GPIO_PIN_10
#define PMS_TX_GPIO_Port GPIOA
#define SSR2_Pin GPIO_PIN_11
#define SSR2_GPIO_Port GPIOA
#define SSR1_Pin GPIO_PIN_12
#define SSR1_GPIO_Port GPIOA
#define MUTE_Pin GPIO_PIN_15
#define MUTE_GPIO_Port GPIOA
#define ALARM_Pin GPIO_PIN_3
#define ALARM_GPIO_Port GPIOB
#define HX2SCK_Pin GPIO_PIN_4
#define HX2SCK_GPIO_Port GPIOB
#define TACHO_B_Pin GPIO_PIN_5
#define TACHO_B_GPIO_Port GPIOB
#define TACHO_B_EXTI_IRQn EXTI9_5_IRQn
#define HX2DT_Pin GPIO_PIN_6
#define HX2DT_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
