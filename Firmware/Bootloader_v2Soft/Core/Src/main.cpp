/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ff.h"                 // For low-level file handling
#include "stm32f4xx_hal_flash.h" // For FLASH functions (erase, program)
#define APP_START_ADDRESS 0x8010000
#define BOOT_FLAG_REG RTC->BKP0R
#define BOOT_FLAG_NO_UPDATE  0xABCD1234
#define BOOT_FLAG_CLEAR      0x00000000
#define FIRMWARE_PATH "monitor.bin"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum class err_t {OK,NOT_OK,SD_MOUNT_ERR,NOT_FOUND,UNKNOWN_ERR};
typedef void (*jumpApp)(void);
typedef void (*p_app_entry_t)(void);
static inline int is_valid_msp(uint32_t msp) {
    return (msp >= 0x20000000U && msp <= 0x2003FFFFU);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void bootflag_write(uint32_t v);
static uint32_t bootflag_read(void);
static void bootflag_clear(void);
err_t checkFile(const char *filename);
err_t deleteFile(const char *filename);
void flash_firmware_from_sd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void bootflag_write(uint32_t v) {
    HAL_PWR_EnableBkUpAccess();
    BOOT_FLAG_REG = v;
    HAL_PWR_DisableBkUpAccess();
}
static uint32_t bootflag_read(void) {
    HAL_PWR_EnableBkUpAccess();
    uint32_t v = BOOT_FLAG_REG;
    HAL_PWR_DisableBkUpAccess();
    return v;
}
static void bootflag_clear(void) {
    bootflag_write(BOOT_FLAG_CLEAR);
}

extern "C" void bootloaderStart(uint32_t app_addr) {
	uint32_t app_msp = *(uint32_t *)APP_START_ADDRESS;
	    uint32_t app_reset = *(uint32_t *)(APP_START_ADDRESS + 4);

	    if (app_msp == 0xFFFFFFFF || app_reset == 0xFFFFFFFF) return;
	    if (!is_valid_msp(app_msp)) return;

	    __disable_irq();
	    SysTick->CTRL = 0;
	    SysTick->LOAD = 0;
	    SysTick->VAL  = 0;

	    HAL_RCC_DeInit();
	    HAL_DeInit();

	    SCB->VTOR = APP_START_ADDRESS;
	    __set_MSP(app_msp);
	    __DSB(); __ISB();

	    p_app_entry_t app_entry = (p_app_entry_t)(app_reset | 1U);
	    app_entry();
}
err_t checkFile(const char *filename){
	FATFS FatFs;
		FatFs.fs_type = FS_FAT32;
		FIL file;
		FRESULT fres;
		fres =f_mount(&FatFs, "", 1);
		if ( fres!= FR_OK) {
			return err_t::SD_MOUNT_ERR;
				}
		fres = f_open(&file, filename, FA_READ);
			if (fres == FR_NO_FILE) {
				return err_t::NOT_FOUND;
			}
			if(fres == FR_OK) {
				return err_t::OK;
			}
			return err_t::UNKNOWN_ERR;
}
err_t deleteFile(const char *filename){
	FATFS FatFs;
	FatFs.fs_type = FS_FAT32;
	FIL file;
	FRESULT fres;
	fres =f_mount(&FatFs, "", 1);
	if ( fres!= FR_OK) {
		return err_t::SD_MOUNT_ERR;
			}
	fres = f_open(&file, filename, FA_READ);
		if (fres == FR_NO_FILE) {
			return err_t::OK;
		}
		fres = f_close(&file);
		fres = f_unlink(filename);
		if (fres == FR_OK) {
			return err_t::OK;
		}
return err_t::NOT_OK;
}
void flash_firmware_from_sd(void) {
		FATFS FatFs;
		FatFs.fs_type = FS_FAT32;
		FIL file;
		UINT br;
		uint8_t buffer[512];
		uint32_t address = APP_START_ADDRESS;  // App start
		FRESULT fres;

		fres =f_mount(&FatFs, "", 1);
		if ( fres!= FR_OK) {
			HAL_UART_Transmit(&huart2,(uint8_t *) "page 33", 7, HAL_MAX_DELAY); //set to fail page
			  HAL_Delay(50);
			  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
			  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
			return;
		}

		if (f_open(&file, FIRMWARE_PATH, FA_READ) != FR_OK) return;

		 if (HAL_FLASH_Unlock() != HAL_OK) return;
		FLASH_EraseInitTypeDef EraseInit;
		uint32_t SectorError;
   		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInit.Sector = FLASH_SECTOR_2;
		EraseInit.NbSectors = 6;            //  if needed

		if (HAL_FLASHEx_Erase(&EraseInit, &SectorError) != HAL_OK) {
			HAL_FLASH_Lock();
			f_close(&file);
			HAL_UART_Transmit(&huart2,(uint8_t *) "page 33", 7, HAL_MAX_DELAY); //set to fail page
			  HAL_Delay(50);
			  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
			  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
			return;
		}

		 while (f_read(&file, buffer, sizeof(buffer), &br) == FR_OK && br > 0) {
		        for (UINT i = 0; i < br; i += 4) {
		            uint32_t word = 0xFFFFFFFF;
		            uint32_t bytes = (br - i >= 4) ? 4U : (br - i);
		            memcpy(&word, &buffer[i], bytes);
		            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) != HAL_OK) {
		                HAL_FLASH_Lock();
		                f_close(&file);
		                return;
		            }
		            address += 4;
		        }
		    }
		f_close(&file);
		HAL_UART_Transmit(&huart2,(uint8_t *) "page 32", 7, HAL_MAX_DELAY); //set to pass page
	  HAL_Delay(50);
	  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
	  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);

		HAL_FLASH_Lock();
	f_unlink(FIRMWARE_PATH);


	for(int i =5;i>=0; i--)
	{
//		  const char component = "n0";
		 char command[64];
		 snprintf(command, sizeof(command), "%s.val=%d", "n0", i);
		 HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
		 HAL_Delay(50);
	  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
	  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
	  HAL_Delay(1000);

}
		bootflag_write(BOOT_FLAG_NO_UPDATE);
		int y = 500;
		HAL_Delay(1500);

		HAL_NVIC_SystemReset();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */
  uint32_t x = bootflag_read();
  if (bootflag_read() == BOOT_FLAG_NO_UPDATE) {
      bootflag_clear();                     // clear it so we don’t loop forever
      bootloaderStart(APP_START_ADDRESS);   // safe jump to app
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  FATFS FatFs;
//  FatFs.fs_type = FS_FAT32;
  FIL file;
  FRESULT fres;
  fres = f_mount(&FatFs,"", 1); //1=mount now
  	if (fres != FR_OK) {
  	}
   fres = f_open(&file, FIRMWARE_PATH, FA_READ);
      if (fres == FR_OK) {

    	  HAL_UART_Transmit(&huart2,(uint8_t *) "page 31", 7, HAL_MAX_DELAY);
    	  HAL_Delay(50);
    	  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
    	  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
    	  HAL_Delay(2000);

          f_close(&file);
          flash_firmware_from_sd();
      }

      bootflag_write(BOOT_FLAG_NO_UPDATE);
         NVIC_SystemReset();
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(800);

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_DET_Pin */
  GPIO_InitStruct.Pin = SD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT1_Pin BOOT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
