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
/* The erase in flash_firmware_from_sd() starts at FLASH_SECTOR_4, which is the
 * sector containing 0x08010000. If APP_START_ADDRESS is ever moved, the erase
 * range must move with it or the update will erase the wrong region. */
static_assert(APP_START_ADDRESS == 0x8010000,
              "APP_START_ADDRESS must match FLASH_SECTOR_4 (0x08010000)");

/* Application region = sectors 4..7 = 64K + 128K + 128K + 128K */
#define APP_MAX_SIZE  (448u * 1024u)
/* Smallest thing that could plausibly be a firmware image (vector table). */
#define APP_MIN_SIZE  (512u)

/* HMI page ids - must match the .tft */
#define PAGE_UPDATING 31
#define PAGE_PASS     32
#define PAGE_FAIL     33
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum class err_t {OK,NOT_OK,SD_MOUNT_ERR,NOT_FOUND,UNKNOWN_ERR};

/* Outcome of an SD firmware update attempt. Anything other than OK means the
 * application must NOT be launched and the boot flag must NOT be set. */
enum class fw_t {
    OK,          /* programmed and verified            */
    ERR_OPEN,    /* card or file not readable          */
    ERR_SIZE,    /* file too small / too big for region*/
    ERR_READ,    /* card failed mid-read               */
    ERR_IMAGE,   /* not a plausible firmware image     */
    ERR_ERASE,   /* flash erase failed                 */
    ERR_PROGRAM, /* flash write failed                 */
    ERR_VERIFY   /* read-back did not match the file   */
};

typedef void (*jumpApp)(void);
typedef void (*p_app_entry_t)(void);
static inline int is_valid_msp(uint32_t msp) {
    /* STM32F411CE has 128K of SRAM (0x20000000..0x2001FFFF) and the stack
     * starts at the top of it, so a valid initial MSP cannot exceed
     * 0x20020000. The old bound of 0x2003FFFF allowed a corrupt image with a
     * stack pointer in non-existent RAM to pass validation. */
    return (msp >= 0x20000000U && msp <= 0x20020000U);
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
fw_t flash_firmware_from_sd(void);
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

/** Send "page N" to the HMI, followed by the 0xFF terminator. */
static void hmi_set_page(uint8_t page) {
    char cmd[16];
    /* strlen() of what actually landed in the buffer - snprintf() returns the
     * length it WOULD have written, which would over-read a truncated buffer. */
    snprintf(cmd, sizeof(cmd), "page %u", (unsigned)page);
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), HAL_MAX_DELAY);
    HAL_Delay(50);
    uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
    HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
}

/**
 * Is a plausible application already programmed at APP_START_ADDRESS?
 * Checks the two words of the vector table: the initial stack pointer must
 * point into real RAM, and the reset vector must point inside the app region.
 */
static bool app_is_valid(void) {
    uint32_t msp = *(uint32_t *)APP_START_ADDRESS;
    uint32_t rst = *(uint32_t *)(APP_START_ADDRESS + 4);

    if (msp == 0xFFFFFFFFU || rst == 0xFFFFFFFFU) return false;   /* erased */
    if (!is_valid_msp(msp))                       return false;
    if (rst <  APP_START_ADDRESS)                 return false;
    if (rst >= (APP_START_ADDRESS + APP_MAX_SIZE))return false;
    return true;
}

/**
 * Bitwise CRC-32 (polynomial 0xEDB88320), chainable: feed the previous result
 * back in as `crc`. Start with 0. Used to prove that what ended up in flash is
 * byte-for-byte what was read from the card.
 */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return ~crc;
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
/**
 * Program monitor.bin from the SD card into the application region.
 *
 * ORDER MATTERS. The old version erased the application as soon as the file
 * merely opened, so a corrupt/truncated file or a card that failed mid-read
 * destroyed the working firmware and left the board with nothing to run.
 * Now the entire image is read and checked BEFORE a single byte of flash is
 * touched, and the result is verified by read-back afterwards:
 *
 *   1. size sanity      - must fit the region and be big enough for a vector table
 *   2. full read pass   - proves the card can deliver every byte; computes CRC32
 *   3. image sanity     - initial MSP and reset vector must be plausible
 *   4. erase            - only now, once the image is known good
 *   5. program
 *   6. verify           - CRC32 of the flashed region must match the file
 *
 * The caller decides what to do with the result; this function deliberately
 * does not set the boot flag, delete the file, or reset.
 */
fw_t flash_firmware_from_sd(void) {
		FATFS FatFs;
		FatFs.fs_type = FS_FAT32;
		FIL file;
		UINT br;
		uint8_t buffer[512];
		uint32_t address = APP_START_ADDRESS;  // App start
		FRESULT fres;

		uint32_t fileSize  = 0;
		uint32_t crcFile   = 0;
		uint32_t remaining = 0;
		uint32_t imgMsp    = 0;
		uint32_t imgRst    = 0;

		fres =f_mount(&FatFs, "", 1);
		if ( fres!= FR_OK) {
			return fw_t::ERR_OPEN;
		}

		if (f_open(&file, FIRMWARE_PATH, FA_READ) != FR_OK) return fw_t::ERR_OPEN;

		/* ── 1. Size sanity (B-06) ─────────────────────────────────────────
		 * An oversized file would program past the end of the app region; an
		 * undersized one cannot even contain a vector table. */
		fileSize = (uint32_t)f_size(&file);
		if (fileSize < APP_MIN_SIZE || fileSize > APP_MAX_SIZE) {
			f_close(&file);
			return fw_t::ERR_SIZE;
		}

		/* ── 2. Full read pass BEFORE erasing (B-02) ───────────────────────
		 * If the card is going to fail, it fails here - while the existing
		 * application is still intact and the board still boots. */
		crcFile   = 0;
		remaining = fileSize;
		while (remaining > 0U) {
			UINT want = (remaining > sizeof(buffer)) ? (UINT)sizeof(buffer)
			                                         : (UINT)remaining;
			if (f_read(&file, buffer, want, &br) != FR_OK || br != want) {
				f_close(&file);
				return fw_t::ERR_READ;
			}
			if (remaining == fileSize) {            /* first block */
				memcpy(&imgMsp, &buffer[0], 4);
				memcpy(&imgRst, &buffer[4], 4);
			}
			crcFile    = crc32_update(crcFile, buffer, br);
			remaining -= br;
		}

		/* ── 3. Does it even look like firmware for this board? ────────────*/
		if (!is_valid_msp(imgMsp) ||
		    imgRst <  APP_START_ADDRESS ||
		    imgRst >= (APP_START_ADDRESS + fileSize)) {
			f_close(&file);
			return fw_t::ERR_IMAGE;
		}

		/* Rewind for the programming pass. */
		if (f_lseek(&file, 0) != FR_OK) {
			f_close(&file);
			return fw_t::ERR_READ;
		}

		/* ── 4. Only now is it safe to erase ───────────────────────────────*/
		 if (HAL_FLASH_Unlock() != HAL_OK) { f_close(&file); return fw_t::ERR_ERASE; }
		FLASH_EraseInitTypeDef EraseInit;
		uint32_t SectorError;
   		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		/* STM32F411CE sector map:
		 *   0: 0x08000000  16K ┐
		 *   1: 0x08004000  16K │ bootloader (64K, sectors 0-3)
		 *   2: 0x08008000  16K │
		 *   3: 0x0800C000  16K ┘
		 *   4: 0x08010000  64K ┐
		 *   5: 0x08020000 128K │ application (448K, sectors 4-7)
		 *   6: 0x08040000 128K │
		 *   7: 0x08060000 128K ┘
		 *
		 * This used to erase from FLASH_SECTOR_2 for 6 sectors (2..7), which
		 * includes sectors 2-3 - 32K INSIDE the bootloader's own region. It only
		 * survived because the bootloader happens to fit in sectors 0-1 (it ends
		 * around 0x08007D7C, ~640 bytes short of sector 2). Had it ever grown past
		 * that, the update would have erased its own running code, leaving neither
		 * a bootloader nor an application: an unrecoverable brick needing ST-Link.
		 *
		 * Erase only the application region. */
		EraseInit.Sector = FLASH_SECTOR_4;
		EraseInit.NbSectors = 4;            // sectors 4..7 = the whole 448K app area

		if (HAL_FLASHEx_Erase(&EraseInit, &SectorError) != HAL_OK) {
			HAL_FLASH_Lock();
			f_close(&file);
			return fw_t::ERR_ERASE;
		}

		/* ── 5. Program ────────────────────────────────────────────────────*/
		remaining = fileSize;
		while (remaining > 0U) {
			UINT want = (remaining > sizeof(buffer)) ? (UINT)sizeof(buffer)
			                                         : (UINT)remaining;
			if (f_read(&file, buffer, want, &br) != FR_OK || br != want) {
				HAL_FLASH_Lock();
				f_close(&file);
				return fw_t::ERR_READ;
			}
			for (UINT i = 0; i < br; i += 4) {
				uint32_t word  = 0xFFFFFFFF;   /* pad a short tail with 0xFF */
				uint32_t bytes = (br - i >= 4) ? 4U : (br - i);
				memcpy(&word, &buffer[i], bytes);
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) != HAL_OK) {
					HAL_FLASH_Lock();
					f_close(&file);
					return fw_t::ERR_PROGRAM;
				}
				address += 4;
			}
			remaining -= br;
		}

		HAL_FLASH_Lock();
		f_close(&file);

		/* ── 6. Verify what actually landed in flash (B-07) ────────────────
		 * Compare a CRC of the programmed region against the CRC computed
		 * from the file in step 2. Reads flash directly - no SD access. */
		if (crc32_update(0, (const uint8_t *)APP_START_ADDRESS, fileSize) != crcFile) {
			return fw_t::ERR_VERIFY;
		}

		/* Consume the update file only once the image is proven good. */
		f_unlink(FIRMWARE_PATH);
		return fw_t::OK;
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
  		/* No card, or an unreadable one - nothing can be updated from here.
  		 * f_open below fails too, so we fall through to the "no update file"
  		 * path at the end, which launches the existing application or shows
  		 * the failure page if there isn't one. */
  	}
   fres = f_open(&file, FIRMWARE_PATH, FA_READ);
      if (fres == FR_OK) {
          f_close(&file);

    	  hmi_set_page(PAGE_UPDATING);
    	  HAL_Delay(2000);

          fw_t result = flash_firmware_from_sd();

          if (result == fw_t::OK) {
              hmi_set_page(PAGE_PASS);

              for (int i = 5; i >= 0; i--) {
                  char command[64];
                  snprintf(command, sizeof(command), "%s.val=%d", "n0", i);
                  HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
                  HAL_Delay(50);
                  uint8_t endCommand[3] = {0xFF, 0xFF, 0xFF};
                  HAL_UART_Transmit(&huart2, endCommand, sizeof(endCommand), HAL_MAX_DELAY);
                  HAL_Delay(1000);
              }
              HAL_Delay(1500);

              /* Image verified - allow the next boot to launch it. */
              bootflag_write(BOOT_FLAG_NO_UPDATE);
              NVIC_SystemReset();
          }
          else {
              /* B-08: the update failed. Do NOT set the boot flag and do NOT
               * delete monitor.bin, so we never launch a half-written image and
               * the update simply retries on the next boot. If the failure
               * happened before the erase (the usual case now), the previous
               * application is still intact and untouched. */
              hmi_set_page(PAGE_FAIL);
              HAL_Delay(3000);
              NVIC_SystemReset();
          }
      }

      /* No update file. If there is also no application to run, say so rather
       * than resetting silently into the same state forever. */
      if (!app_is_valid()) {
          hmi_set_page(PAGE_FAIL);
          HAL_Delay(3000);
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
