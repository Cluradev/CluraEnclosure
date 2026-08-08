/*
 * tuning.h
 * CLURA ENCLOSURE - RevB firmware
 *
 * Runtime-tunable machine parameters, stored in the AT24C256 EEPROM and
 * optionally provisioned from a `data.clu` file on the SD card.
 *
 * ── Why this exists ─────────────────────────────────────────────────────
 * Thresholds, servo endpoints, timing intervals etc. used to be compile-time
 * #defines in parameters.h, so changing one meant rebuilding and reflashing.
 * They now live in this struct, which the firmware reads at runtime.
 *
 * ── Three tiers ─────────────────────────────────────────────────────────
 *   1. Firmware defaults  — the #defines in parameters.h (the `defval`
 *                           column of TUNE_PARAMS). Used when the EEPROM
 *                           block is blank or corrupt.
 *   2. EEPROM             — authoritative at runtime. Survives firmware
 *                           updates (the bootloader only touches internal
 *                           flash, never the external I2C EEPROM).
 *   3. SD `data.clu`      — optional provisioning source, applied ONCE per
 *                           revision (see below).
 *
 * ── Revision control (why the SD file doesn't fight the EEPROM) ─────────
 * `data.clu` carries `configRevision = N`. Each board stores the last N it
 * applied in `appliedRevision`. The file is applied only when
 *
 *      file.configRevision > board.appliedRevision
 *
 * so the card can stay in the machine permanently and is inert on every
 * boot except the one after a technician deliberately bumps the number.
 * Bumping it re-provisions every board the card is inserted into, exactly
 * once each — a single master card can configure a whole fleet.
 *
 * Revisions are monotonic: to roll values back, restore the old values AND
 * bump the number upward. Never reuse a number.
 *
 * ── What is NOT here ────────────────────────────────────────────────────
 * User state (brightness, fan speeds, filament selection, load-cell
 * calibration, feature flags) stays in EepromConfig / eeprom_storage.cpp.
 * That data is written by the HMI constantly; if the SD file described it,
 * every boot with a card inserted would revert the user's settings.
 */

#ifndef INC_TUNING_H_
#define INC_TUNING_H_

#include "parameters.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sentinel proving the EEPROM tuning block has been initialised. */
#define TUNE_MAGIC            0x7551U

/* Layout version of EepromTuning. Bumped by FIRMWARE whenever a field is
 * added/removed/resized. A mismatch triggers Tuning_Migrate(), which must
 * preserve existing values and appliedRevision — never wipe them, or every
 * OTA update would silently reset the machine's tuning. */
#define TUNE_STRUCT_VERSION   5U

/**
 * Persistent tuning block. Stored at EEPROM_ADDR_TUNING.
 * Packed so the on-EEPROM layout is stable across compilers.
 */
typedef struct __attribute__((packed))
{
    uint16_t magic;             /* TUNE_MAGIC                                   */
    uint16_t structVersion;     /* TUNE_STRUCT_VERSION                          */
    uint32_t appliedRevision;   /* configRevision last applied from data.clu    */

    /* ── Servo / flap ─────────────────────────────────────────────────── */
    uint8_t  servoOpenAngle;    /* deg, flap open (also the overtemp position)  */
    uint8_t  servoCloseAngle;   /* deg, flap closed                             */
    uint16_t servoHoldMs;       /* ms driving the pulse before detaching        */
    uint8_t  servoTempHyst;     /* deg C hysteresis on the overtemp vent-open   */

    /* ── Safety ───────────────────────────────────────────────────────── */
    uint8_t  maxTemp;           /* deg C, automatic flap-open trigger           */

    /* ── Smoke / gas ──────────────────────────────────────────────────── */
    uint16_t lpgThreshold;      /* MQ2 raw ADC                                  */
    uint16_t smokeThreshold;    /* MEMS raw ADC                                 */
    uint32_t smokeBeginTime;    /* ms warm-up before alarms arm                 */
    uint8_t  countdownTime;     /* s, "continue" button countdown               */
    uint8_t  opticalAlarmEnable;/* 0 = optical sensor cannot trigger the alarm  */
    uint8_t  smokeFireResponse; /* 1 = on smoke: all fans off AND flap closed   */

    /* ── Auto cooling ─────────────────────────────────────────────────── */
    int16_t  autoCoolTempMin;   /* deg C -> 0 % fan                             */
    int16_t  autoCoolTempMax;   /* deg C -> 100 % fan                           */

    /* ── Auto filtering ───────────────────────────────────────────────── */
    uint16_t autoFilterImpMin;
    uint16_t autoFilterImpMax;
    uint16_t autoFilterP03Min;
    uint16_t autoFilterP03Max;

    /* ── Fans ─────────────────────────────────────────────────────────── */
    uint8_t  extraFanSpeed;     /* %, fixed speed for the regulated "extra" fan */

    /* ── Intervals / display ──────────────────────────────────────────── */
    uint32_t bmeSampleInterval; /* ms between BME680 reads                      */
    uint32_t usageLogInterval;  /* ms between usage-counter saves               */
    uint32_t sensorUsageWarnHours; /* runtime HOURS before the "replace" icon    */
    uint8_t  minBrightness;     /* lowest screen brightness the UI allows       */

    /* ── Load-cell factory seeds ──────────────────────────────────────── */
    /* NOTE: only used for a board that has never been calibrated. An already
     * calibrated unit keeps its real factors in EepromConfig; changing these
     * will NOT re-tune it. */
    int32_t  defaultLeftCal;
    int32_t  defaultRightCal;
    int32_t  defaultLeftOffset;
    int32_t  defaultRightOffset;
    int32_t  tareWeight;        /* g, empty-spool weight                        */

    uint16_t crc16;             /* over all preceding bytes                     */
} EepromTuning;

/** The live tuning values. Read this instead of the old #defines. */
extern EepromTuning g_tune;

/** Number of parameters clamped during the last data.clu apply (0 = clean). */
extern uint8_t g_tuneClamped;

/** Result of the last Tuning_Init(), for diagnostics / display. */
typedef enum {
    TUNE_SRC_DEFAULTS = 0,   /* EEPROM blank or corrupt -> firmware defaults  */
    TUNE_SRC_EEPROM,         /* loaded from EEPROM, SD absent or already applied */
    TUNE_SRC_SD_APPLIED,     /* data.clu had a newer revision and was applied  */
    TUNE_SRC_MIGRATED        /* struct version changed, values migrated       */
} TuneSource;

extern TuneSource g_tuneSource;

/**
 * @brief  Bring g_tune up: load from EEPROM (or defaults / migrate), then
 *         apply data.clu from SD if it carries a newer configRevision.
 *
 * Call once at boot, AFTER the EEPROM address has been resolved
 * (EEPROM_IsReady / EEPROM_ScanAddress) and BEFORE any code reads g_tune.
 * Safe to call with no EEPROM and/or no SD card — falls back to defaults.
 */
void Tuning_Init(void);

/** @brief Populate *t with the firmware defaults from TUNE_PARAMS. */
void Tuning_LoadDefaults(EepromTuning *t);

/** @brief Write g_tune back to EEPROM (recomputes the CRC). */
bool Tuning_Save(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TUNING_H_ */
