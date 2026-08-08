/*
 * AT24C_EEPROM.cpp
 * CLURA ENCLOSURE - RevB firmware
 *
 * Implementation of the low-level AT24C EEPROM driver.
 *
 * Key behaviour:
 *   - EEPROM_Read  : single HAL_I2C_Mem_Read call (sequential read spans pages)
 *   - EEPROM_Write : splits writes at 64-byte page boundaries; waits 6 ms after
 *                    each page write for the internal write cycle to complete
 *   - EEPROM_IsReady: polls the device ACK with a short timeout
 *
 * Address handling
 * ────────────────
 * g_eeprom_addr defaults to AT24C_I2C_ADDR (0x50).
 * Call EEPROM_ScanAddress() at startup to auto-detect the real address if
 * the EEPROM doesn't respond at 0x50 (e.g. open/floating address pins that
 * settle at VCC → address becomes 0x57).  EEPROM_ScanAddress() updates
 * g_eeprom_addr automatically so all subsequent calls use the correct address.
 *
 * The hi2c1 handle is defined in main.cpp / stm32f4xx_hal_msp.c and declared
 * as an extern via main.h.
 */

#include "AT24C_EEPROM.h"

extern I2C_HandleTypeDef hi2c1;

/* ── Runtime address (updated by EEPROM_ScanAddress if needed) ─────────── */
uint8_t g_eeprom_addr = AT24C_I2C_ADDR;   /* default: 0x50 */

/* ── Public functions ──────────────────────────────────────────────────── */

bool EEPROM_IsReady(void)
{
    /* Poll with 3 retries, 100 ms timeout — uses the runtime address */
    return (HAL_I2C_IsDeviceReady(&hi2c1,
                                  (uint16_t)(g_eeprom_addr << 1U),
                                  3,
                                  100) == HAL_OK);
}

HAL_StatusTypeDef EEPROM_Read(uint16_t memAddr, uint8_t *pData, uint16_t size)
{
    if (pData == nullptr || size == 0U)
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)(g_eeprom_addr << 1U),
                             memAddr,
                             I2C_MEMADD_SIZE_16BIT,
                             pData,
                             size,
                             HAL_MAX_DELAY);
}

HAL_StatusTypeDef EEPROM_Write(uint16_t memAddr, uint8_t *pData, uint16_t size)
{
    if (pData == nullptr || size == 0U)
        return HAL_ERROR;

    uint16_t written = 0U;

    while (written < size)
    {
        uint16_t currentAddr = memAddr + written;

        /* Number of bytes remaining until the next page boundary */
        uint16_t pageOffset   = currentAddr % AT24C_PAGE_SIZE;
        uint16_t spaceInPage  = AT24C_PAGE_SIZE - pageOffset;

        /* Bytes left to write overall */
        uint16_t remaining    = size - written;

        /* Write at most what fits in the current page */
        uint16_t chunk = (remaining < spaceInPage) ? remaining : spaceInPage;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1,
                                                      (uint16_t)(g_eeprom_addr << 1U),
                                                      currentAddr,
                                                      I2C_MEMADD_SIZE_16BIT,
                                                      pData + written,
                                                      chunk,
                                                      HAL_MAX_DELAY);
        if (status != HAL_OK)
            return status;

        /* Wait for the EEPROM internal write cycle to complete */
        HAL_Delay(AT24C_WRITE_DELAY_MS);

        written += chunk;
    }

    return HAL_OK;
}
