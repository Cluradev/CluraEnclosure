/*
 * AT24C_EEPROM.h
 * CLURA ENCLOSURE - RevB firmware
 *
 * Low-level I²C driver for the AT24C series EEPROM (AT24C256 assumed, 32 KB).
 * Address pins A2/A1/A0 are all open (logic 0) → 7-bit I²C address = 0x50.
 *
 * The AT24C256 uses 16-bit memory addressing and has a 64-byte page size.
 * Writes that cross a page boundary are automatically split into multiple
 * page-write operations with the mandatory 5 ms write-cycle delay between them.
 *
 * All functions share hi2c1 with the BME680 (addresses 0x76 / 0x77).
 * There is no conflict because the EEPROM sits at 0x50.
 */

#ifndef INC_AT24C_EEPROM_H_
#define INC_AT24C_EEPROM_H_

#include "main.h"       /* HAL types, hi2c1 declaration */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Device constants ──────────────────────────────────────────────────── */

/** 7-bit I²C address (A2=A1=A0=0, all pins open → pulled low) */
#define AT24C_I2C_ADDR        0x50U

/**
 * Runtime-adjustable I²C address.
 * Defaults to AT24C_I2C_ADDR (0x50).
 * Call EEPROM_ScanAddress() at boot to auto-detect and update this if the
 * chip doesn't respond at the default address (e.g. when address pins float
 * high → 0x57).
 *
 * Declared extern here; defined in AT24C_EEPROM.cpp.
 */
extern uint8_t g_eeprom_addr;

/** Page size in bytes.
 *
 * AT24C32 / AT24C64  → 32 bytes  (our target)
 * AT24C128 / AT24C256 → 64 bytes  (32-byte writes are still valid — just
 *                                  requires twice as many I²C transactions)
 *
 * Using 32 here is safe for ALL common AT24C variants.
 * Root-cause note: using 64 with a 32-byte-page chip caused a 60-byte write
 * at address 0x0004 to silently wrap back over 0x0000, corrupting the magic
 * word with filamentPageMax (= 4 from the default MachineState).
 */
#define AT24C_PAGE_SIZE       32U

/**
 * Maximum write-cycle time in ms.
 */
#define AT24C_WRITE_DELAY_MS  6U

/** Total EEPROM capacity in bytes (AT24C256 = 32 KB) */
#define AT24C_CAPACITY_BYTES  32768U

/* ── Public API ────────────────────────────────────────────────────────── */

/**
 * @brief  Check whether the EEPROM is present and responding on the bus.
 * @retval true  if the device ACKs its address
 * @retval false if the device does not respond (missing or wiring fault)
 */
bool EEPROM_IsReady(void);

/**
 * @brief  Read a block of bytes from the EEPROM.
 * @param  memAddr  Start byte address within the EEPROM (0x0000 – 0x7FFF)
 * @param  pData    Destination buffer
 * @param  size     Number of bytes to read
 * @retval HAL_OK on success, HAL error code otherwise
 */
HAL_StatusTypeDef EEPROM_Read(uint16_t memAddr, uint8_t *pData, uint16_t size);

/**
 * @brief  Write a block of bytes to the EEPROM.
 *
 * Automatically splits the write across page boundaries and inserts the
 * required write-cycle delay after each page write.
 *
 * @param  memAddr  Start byte address within the EEPROM (0x0000 – 0x7FFF)
 * @param  pData    Source buffer
 * @param  size     Number of bytes to write
 * @retval HAL_OK on success, HAL error code otherwise
 */
HAL_StatusTypeDef EEPROM_Write(uint16_t memAddr, uint8_t *pData, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* INC_AT24C_EEPROM_H_ */
