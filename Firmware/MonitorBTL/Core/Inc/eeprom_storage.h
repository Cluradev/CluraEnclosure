/*
 * eeprom_storage.h
 * CLURA ENCLOSURE - RevB firmware
 *
 * High-level config and filament storage using the AT24C256 EEPROM.
 *
 * ── Storage layout (byte addresses) ─────────────────────────────────────
 *
 *   0x0000  uint16_t magic    (0xC1A4) — validity sentinel
 *   0x0002  uint16_t version  (current = 1)
 *   0x0004  EepromConfig      packed struct of persistent MachineState fields
 *   0x0080  char[24][16]      filament brand strings  (384 bytes)
 *   0x0200  int32_t[24]       filament weights        (96 bytes)
 *   0x0260  (free)
 *
 * SD card is retained ONLY for:
 *   • CSV sensor logging  (csvInit / csvLog / csvDelete)
 *   • Firmware OTA binary (reserved, not yet implemented)
 *
 * ── Persistent fields written to EEPROM ─────────────────────────────────
 *   Settings : mute, alarm, standbyTime, interval, logging, brightness
 *   Baseline : temperatureBaseline, impuritiesBaseline, usage_minutes
 *   Spool    : leftSpoolIndex, rightSpoolIndex, leftSelectFilament,
 *              rightSelectFilament, filamentSelectPage, filamentIndex,
 *              filamentIndexMax, filamentPage, filamentPageMax,
 *              leftFilament, rightFilament
 *   Cal data : leftCalFactor, rightCalFactor, leftOffset, rightOffset
 *   Lighting : Impurities, lightIndex, LedBrightness, red, blue, green, ledBar
 *   Fans     : filterFanSetSpeed, coolingFanSetSpeed, autoCooling, manCooling,
 *              autoFiltering, manFiltering, isFanRegOn
 *   Output   : servoAngle, autoServo, servoPower, fanServoState, relay
 *   Features : features, usePms, useBme, useMq2, useOptical, useLedStrip,
 *              useLedBar, useLoadCell, useMems, useFilterFan, useCoolingFan
 *   Status   : calibrationComplete
 *
 * Runtime-only sensor readings (pm03, pm2, temperature, etc.) are NOT written.
 */

#ifndef INC_EEPROM_STORAGE_H_
#define INC_EEPROM_STORAGE_H_

#include "parameters.h"   /* MachineState, NUM_OF_FILAMENTS, LEN_BUFFER */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── EEPROM memory map constants ────────────────────────────────────────── */
/* All address/magic constants are defined in parameters.h which is included
 * above.  Do not redefine them here.                                         */

/* ── Compact persistent-config struct ───────────────────────────────────── */

/**
 * Packed struct containing only the MachineState fields that must survive
 * a power cycle.  Sensor readings, display state, and other runtime-only
 * values are deliberately excluded to keep writes small and fast.
 *
 * sizeof(EepromConfig) ≈ 73 bytes — fits well within 32 KB.
 */
typedef struct __attribute__((packed))
{
    /* Settings */
    uint8_t  mute;
    uint8_t  alarm;
    int16_t  standbyTime;
    int16_t  interval;
    uint8_t  logging;
    uint8_t  brightness;

    /* Baselines & usage */
    int32_t  temperatureBaseline;
    int32_t  impuritiesBaseline;
    int32_t  usage_minutes;

    /* Spool / filament selection */
    uint8_t  leftSpoolIndex;
    uint8_t  rightSpoolIndex;
    uint8_t  leftSelectFilament;
    uint8_t  rightSelectFilament;
    uint8_t  filamentSelectPage;
    uint8_t  filamentIndex;
    uint8_t  filamentIndexMax;
    uint8_t  filamentPage;
    uint8_t  filamentPageMax;
    int16_t  leftFilament;
    int16_t  rightFilament;

    /* Load-cell calibration */
    int32_t  leftCalFactor;
    int32_t  rightCalFactor;
    int32_t  leftOffset;
    int32_t  rightOffset;

    /* Lighting */
    uint8_t  Impurities;
    uint8_t  lightIndex;
    float    LedBrightness;
    uint8_t  red;
    uint8_t  blue;
    uint8_t  green;
    uint8_t  ledBar;

    /* Fan control */
    uint8_t  filterFanSetSpeed;
    uint8_t  coolingFanSetSpeed;
    uint8_t  autoCooling;
    uint8_t  manCooling;
    uint8_t  autoFiltering;
    uint8_t  manFiltering;
    uint8_t  isFanRegOn;

    /* Output / servo */
    uint8_t  servoAngle;
    uint8_t  autoServo;
    uint8_t  servoPower;
    uint8_t  fanServoState;
    uint8_t  relay;

    /* Feature flags */
    uint8_t  features;
    uint8_t  usePms;
    uint8_t  useBme;
    uint8_t  useMq2;
    uint8_t  useOptical;
    uint8_t  useLedStrip;
    uint8_t  useLedBar;
    uint8_t  useLoadCell;
    uint8_t  useMems;
    uint8_t  useFilterFan;
    uint8_t  useCoolingFan;

    /* Calibration status */
    uint8_t  calibrationComplete;

} EepromConfig;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Check whether the EEPROM contains valid data.
 *         Reads the magic word at address 0x0000 and compares it with
 *         EEPROM_MAGIC_VALUE.
 * @retval true  — EEPROM has been initialised and data is valid
 * @retval false — first boot, erased chip, or corrupt header
 */
bool EEPROM_IsValid(void);

/**
 * @brief  Write the magic word and version, then save default config and
 *         filament data.  Call this on first boot when EEPROM_IsValid()
 *         returns false.
 * @param  ms              Pointer to MachineState with default values already
 *                         populated (i.e. the compile-time initialised struct)
 * @param  filamentBrand   Default brand strings
 * @param  filamentWeight  Default weight values
 * @retval true  all writes succeeded
 * @retval false at least one write failed — EEPROM may not be present/wired
 */
bool EEPROM_Format(MachineState *ms,
                   char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                   int  filamentWeight[NUM_OF_FILAMENTS]);

/**
 * @brief  Scan all possible AT24C I²C addresses (0x50–0x57) and return the
 *         first one that ACKs.  Call this once at startup if EEPROM_IsReady()
 *         fails with the default address to auto-detect the actual address.
 * @retval 7-bit address of the responding device (0x50–0x57)
 * @retval 0x00 if no device found at any address
 */
uint8_t EEPROM_ScanAddress(void);

/**
 * @brief  Save all persistent MachineState fields to EEPROM.
 * @retval true on success
 */
bool EEPROM_SaveConfig(MachineState *ms);

/**
 * @brief  Load all persistent MachineState fields from EEPROM into *ms.
 * @retval true on success
 */
bool EEPROM_LoadConfig(MachineState *ms);

/**
 * @brief  Save the filament brand names and spool weights to EEPROM.
 * @retval true on success
 */
bool EEPROM_SaveFilamentData(char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                             int  filamentWeight[NUM_OF_FILAMENTS]);

/**
 * @brief  Load filament brand names and spool weights from EEPROM.
 * @retval true on success
 */
bool EEPROM_LoadFilamentData(char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                             int  filamentWeight[NUM_OF_FILAMENTS]);

/**
 * @brief  Read the boot counter, increment it, and write it back.
 *
 * Used to timestamp CSV log rows. HAL_GetTick() restarts at 0 on every power
 * cycle, so "minutes since boot" alone cannot order rows across reboots -
 * logging the boot id alongside it makes the whole log sortable by
 * (BootID, MinSinceBoot).
 *
 * Stored at its own EEPROM address so this once-per-boot write never touches
 * the config or tuning blocks. Returns 1 on the first ever call, and 1 again
 * if the EEPROM is missing or unreadable.
 *
 * @retval the boot id for this power-on session (>= 1)
 */
uint32_t EEPROM_NextBootId(void);

/**
 * @brief  Persist the machine-usage counter (minutes of filter-fan runtime).
 *
 * Stored in its own small record at EEPROM_ADDR_USAGE rather than inside
 * EepromConfig, so saving it does not rewrite the whole config block. Written
 * at most every USAGE_LOG_INTERVAL and only when the value has changed.
 * @retval true on success
 */
bool EEPROM_SaveUsage(uint32_t usageMinutes);

/**
 * @brief  Read the machine-usage counter.
 * @param  usageMinutes  receives the stored value on success
 * @retval false if the record is absent or corrupt (first boot with this
 *         firmware) - the caller should seed it from the legacy config field.
 */
bool EEPROM_LoadUsage(uint32_t *usageMinutes);

/**
 * @brief  Persist / read the filament-library revision last applied from
 *         filament.clu (see filament_store.h). Kept in its own tiny record.
 */
bool EEPROM_SaveFilamentRev(uint32_t revision);
bool EEPROM_LoadFilamentRev(uint32_t *revision);

#ifdef __cplusplus
}
#endif

#endif /* INC_EEPROM_STORAGE_H_ */
