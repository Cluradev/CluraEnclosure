/*
 * eeprom_storage.cpp
 * CLURA ENCLOSURE - RevB firmware
 *
 * High-level config and filament storage implementation.
 *
 * Serialisation strategy
 * ─────────────────────
 * Rather than using JSON (which requires heap allocation and is slow on
 * EEPROM), we copy a compact POD struct (EepromConfig) directly in/out of
 * the EEPROM at a fixed base address.  Filament data is stored at a separate
 * fixed address as raw char arrays + int32_t arrays.
 *
 * This gives deterministic write sizes, no fragmentation, and fast load
 * times at boot — important when hi2c1 is also shared with the BME680.
 *
 * Write wear
 * ──────────
 * The AT24C256 is rated for 1,000,000 write cycles per page.
 * sizeof(EepromConfig) ≈ 73 bytes → spans 2 × 64-byte pages.
 * Filament data ≈ 480 bytes → spans 8 pages.
 * Even at one config save per minute the device would last >600 days.
 * The saveFlag debounce in the main loop already batches rapid changes.
 */

#include "eeprom_storage.h"
#include "AT24C_EEPROM.h"
#include <string.h>   /* memcpy, memset */

extern uint8_t g_eeprom_addr;  /* defined in AT24C_EEPROM.cpp */

/* ── Internal helpers ───────────────────────────────────────────────────── */

/**
 * Copy persistent MachineState fields into a compact EepromConfig struct.
 * Fields that are runtime-only (sensor readings, display temps, etc.) are
 * deliberately not copied — they are re-acquired from sensors each boot.
 */
static void ms_to_cfg(const MachineState *ms, EepromConfig *cfg)
{
    cfg->mute                = (uint8_t)ms->mute;
    cfg->alarm               = (uint8_t)ms->alarm;
    cfg->standbyTime         = (int16_t)ms->standbyTime;
    cfg->interval            = (int16_t)ms->interval;
    cfg->logging             = (uint8_t)ms->logging;
    cfg->brightness          = ms->brightness;

    cfg->temperatureBaseline = ms->temperatureBaseline;
    cfg->impuritiesBaseline  = ms->impuritiesBaseline;
    cfg->usage_minutes       = ms->usage_minutes;

    cfg->leftSpoolIndex      = ms->leftSpoolIndex;
    cfg->rightSpoolIndex     = ms->rightSpoolIndex;
    cfg->leftSelectFilament  = ms->leftSelectFilament;
    cfg->rightSelectFilament = ms->rightSelectFilament;
    cfg->filamentSelectPage  = ms->filamentSelectPage;
    cfg->filamentIndex       = ms->filamentIndex;
    cfg->filamentIndexMax    = ms->filamentIndexMax;
    cfg->filamentPage        = ms->filamentPage;
    cfg->filamentPageMax     = ms->filamentPageMax;
    cfg->leftFilament        = (int16_t)ms->leftFilament;
    cfg->rightFilament       = (int16_t)ms->rightFilament;

    cfg->leftCalFactor       = (int32_t)ms->leftCalFactor;
    cfg->rightCalFactor      = (int32_t)ms->rightCalFactor;
    cfg->leftOffset          = (int32_t)ms->leftOffset;
    cfg->rightOffset         = (int32_t)ms->rightOffset;

    cfg->Impurities          = ms->Impurities;
    cfg->lightIndex          = ms->lightIndex;
    cfg->LedBrightness       = ms->LedBrightness;
    cfg->red                 = ms->red;
    cfg->blue                = ms->blue;
    cfg->green               = ms->green;
    cfg->ledBar              = (uint8_t)ms->ledBar;

    cfg->filterFanSetSpeed   = ms->filterFanSetSpeed;
    cfg->coolingFanSetSpeed  = ms->coolingFanSetSpeed;
    cfg->autoCooling         = (uint8_t)ms->autoCooling;
    cfg->manCooling          = (uint8_t)ms->manCooling;
    cfg->autoFiltering       = (uint8_t)ms->autoFiltering;
    cfg->manFiltering        = (uint8_t)ms->manFiltering;
    cfg->isFanRegOn          = (uint8_t)ms->isFanRegOn;

    cfg->servoAngle          = ms->servoAngle;
    cfg->autoServo           = (uint8_t)ms->autoServo;
    cfg->servoPower          = (uint8_t)ms->servoPower;
    cfg->fanServoState       = ms->fanServoState;
    cfg->relay               = (uint8_t)ms->relay;

    cfg->features            = ms->features;
    cfg->usePms              = (uint8_t)ms->usePms;
    cfg->useBme              = (uint8_t)ms->useBme;
    cfg->useMq2              = (uint8_t)ms->useMq2;
    cfg->useOptical          = (uint8_t)ms->useOptical;
    cfg->useLedStrip         = (uint8_t)ms->useLedStrip;
    cfg->useLedBar           = (uint8_t)ms->useLedBar;
    cfg->useLoadCell         = (uint8_t)ms->useLoadCell;
    cfg->useMems             = (uint8_t)ms->useMems;
    cfg->useFilterFan        = (uint8_t)ms->useFilterFan;
    cfg->useCoolingFan       = (uint8_t)ms->useCoolingFan;

    cfg->calibrationComplete = (uint8_t)ms->calibrationComplete;
}

/**
 * Unpack an EepromConfig struct into the persistent MachineState fields.
 * Runtime-only fields are left untouched.
 */
static void cfg_to_ms(const EepromConfig *cfg, MachineState *ms)
{
    ms->mute                = (bool)cfg->mute;
    ms->alarm               = (bool)cfg->alarm;
    ms->standbyTime         = cfg->standbyTime;
    ms->interval            = cfg->interval;
    ms->logging             = (bool)cfg->logging;
    ms->brightness          = cfg->brightness;

    ms->temperatureBaseline = cfg->temperatureBaseline;
    ms->impuritiesBaseline  = cfg->impuritiesBaseline;
    ms->usage_minutes       = cfg->usage_minutes;

    ms->leftSpoolIndex      = cfg->leftSpoolIndex;
    ms->rightSpoolIndex     = cfg->rightSpoolIndex;
    ms->leftSelectFilament  = cfg->leftSelectFilament;
    ms->rightSelectFilament = cfg->rightSelectFilament;
    ms->filamentSelectPage  = cfg->filamentSelectPage;
    ms->filamentIndex       = cfg->filamentIndex;
    ms->filamentIndexMax    = cfg->filamentIndexMax;
    ms->filamentPage        = cfg->filamentPage;
    ms->filamentPageMax     = cfg->filamentPageMax;
    ms->leftFilament        = (int)cfg->leftFilament;
    ms->rightFilament       = (int)cfg->rightFilament;

    ms->leftCalFactor       = (long)cfg->leftCalFactor;
    ms->rightCalFactor      = (long)cfg->rightCalFactor;
    ms->leftOffset          = (long)cfg->leftOffset;
    ms->rightOffset         = (long)cfg->rightOffset;

    ms->Impurities          = cfg->Impurities;
    ms->lightIndex          = cfg->lightIndex;
    ms->LedBrightness       = cfg->LedBrightness;
    ms->red                 = cfg->red;
    ms->blue                = cfg->blue;
    ms->green               = cfg->green;
    ms->ledBar              = (bool)cfg->ledBar;

    ms->filterFanSetSpeed   = cfg->filterFanSetSpeed;
    ms->coolingFanSetSpeed  = cfg->coolingFanSetSpeed;
    ms->autoCooling         = (bool)cfg->autoCooling;
    ms->manCooling          = (bool)cfg->manCooling;
    ms->autoFiltering       = (bool)cfg->autoFiltering;
    ms->manFiltering        = (bool)cfg->manFiltering;
    ms->isFanRegOn          = (bool)cfg->isFanRegOn;

    ms->servoAngle          = cfg->servoAngle;
    ms->autoServo           = (bool)cfg->autoServo;
    ms->servoPower          = (bool)cfg->servoPower;
    ms->fanServoState       = cfg->fanServoState;
    ms->relay               = (bool)cfg->relay;

    ms->features            = cfg->features;
    ms->usePms              = (bool)cfg->usePms;
    ms->useBme              = (bool)cfg->useBme;
    ms->useMq2              = (bool)cfg->useMq2;
    ms->useOptical          = (bool)cfg->useOptical;
    ms->useLedStrip         = (bool)cfg->useLedStrip;
    ms->useLedBar           = (bool)cfg->useLedBar;
    ms->useLoadCell         = (bool)cfg->useLoadCell;
    ms->useMems             = (bool)cfg->useMems;
    ms->useFilterFan        = (bool)cfg->useFilterFan;
    ms->useCoolingFan       = (bool)cfg->useCoolingFan;

    ms->calibrationComplete = (bool)cfg->calibrationComplete;
}

/* ── Public functions ───────────────────────────────────────────────────── */

bool EEPROM_IsValid(void)
{
    uint16_t magic = 0U;
    HAL_StatusTypeDef status = EEPROM_Read(EEPROM_ADDR_MAGIC,
                                           (uint8_t *)&magic,
                                           sizeof(magic));
    return (status == HAL_OK) && (magic == EEPROM_MAGIC_VALUE);
}

uint8_t EEPROM_ScanAddress(void)
{
    /*
     * Try all 8 possible AT24C addresses: 0x50 – 0x57.
     * Updates g_eeprom_addr to the first responding device so that all
     * subsequent EEPROM_Read / EEPROM_Write calls use the correct address.
     *
     * Call this ONCE at startup if EEPROM_IsReady() fails at the default
     * address.  The scan temporarily writes each address into g_eeprom_addr
     * so EEPROM_IsReady() can use the same path as normal operation.
     */
    for (uint8_t addr = 0x50U; addr <= 0x57U; addr++)
    {
        g_eeprom_addr = addr;
        if (EEPROM_IsReady())
            return addr;   /* g_eeprom_addr already updated */
    }
    /* Nothing found — restore default and return 0 */
    g_eeprom_addr = AT24C_I2C_ADDR;
    return 0x00U;
}

bool EEPROM_Format(MachineState *ms,
                   char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                   int  filamentWeight[NUM_OF_FILAMENTS])
{
    bool ok = true;

    /* Write magic + version header */
    uint16_t magic   = EEPROM_MAGIC_VALUE;
    uint16_t version = EEPROM_CONFIG_VERSION;

    if (EEPROM_Write(EEPROM_ADDR_MAGIC,   (uint8_t *)&magic,   sizeof(magic))   != HAL_OK) ok = false;
    if (EEPROM_Write(EEPROM_ADDR_VERSION, (uint8_t *)&version, sizeof(version)) != HAL_OK) ok = false;

    /* Write default config and filament data */
    if (!EEPROM_SaveConfig(ms))                              ok = false;
    if (!EEPROM_SaveFilamentData(filamentBrand, filamentWeight)) ok = false;

    return ok;
}

bool EEPROM_SaveConfig(MachineState *ms)
{
    if (ms == nullptr)
        return false;

    EepromConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    ms_to_cfg(ms, &cfg);

    return EEPROM_Write(EEPROM_ADDR_CONFIG,
                        (uint8_t *)&cfg,
                        sizeof(cfg)) == HAL_OK;
}

bool EEPROM_LoadConfig(MachineState *ms)
{
    if (ms == nullptr)
        return false;

    EepromConfig cfg;
    memset(&cfg, 0, sizeof(cfg));

    HAL_StatusTypeDef status = EEPROM_Read(EEPROM_ADDR_CONFIG,
                                           (uint8_t *)&cfg,
                                           sizeof(cfg));
    if (status != HAL_OK)
        return false;

    cfg_to_ms(&cfg, ms);
    return true;
}

bool EEPROM_SaveFilamentData(char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                             int  filamentWeight[NUM_OF_FILAMENTS])
{
    /* Write brand strings as-is (char[24][16] = 384 bytes) */
    HAL_StatusTypeDef st = EEPROM_Write(EEPROM_ADDR_FILAMENT_BRAND,
                                        (uint8_t *)filamentBrand,
                                        NUM_OF_FILAMENTS * LEN_BUFFER);
    if (st != HAL_OK)
        return false;

    /*
     * Filament weights are int (platform-dependent size).
     * We convert to int32_t for portability before writing.
     */
    int32_t weights[NUM_OF_FILAMENTS];
    for (int i = 0; i < NUM_OF_FILAMENTS; i++)
        weights[i] = (int32_t)filamentWeight[i];

    st = EEPROM_Write(EEPROM_ADDR_FILAMENT_WEIGHT,
                      (uint8_t *)weights,
                      sizeof(weights));
    return (st == HAL_OK);
}

bool EEPROM_LoadFilamentData(char filamentBrand[NUM_OF_FILAMENTS][LEN_BUFFER],
                             int  filamentWeight[NUM_OF_FILAMENTS])
{
    HAL_StatusTypeDef st = EEPROM_Read(EEPROM_ADDR_FILAMENT_BRAND,
                                       (uint8_t *)filamentBrand,
                                       NUM_OF_FILAMENTS * LEN_BUFFER);
    if (st != HAL_OK)
        return false;

    /* Ensure every brand string is null-terminated */
    for (int i = 0; i < NUM_OF_FILAMENTS; i++)
        filamentBrand[i][LEN_BUFFER - 1] = '\0';

    int32_t weights[NUM_OF_FILAMENTS];
    st = EEPROM_Read(EEPROM_ADDR_FILAMENT_WEIGHT,
                     (uint8_t *)weights,
                     sizeof(weights));
    if (st != HAL_OK)
        return false;

    for (int i = 0; i < NUM_OF_FILAMENTS; i++)
        filamentWeight[i] = (int)weights[i];

    return true;
}


/* ── Boot counter ───────────────────────────────────────────────────────── */

#define BOOTID_MAGIC 0xB007U

typedef struct __attribute__((packed))
{
    uint16_t magic;
    uint32_t bootId;
} EepromBootId;

uint32_t EEPROM_NextBootId(void)
{
    EepromBootId rec;

    if (EEPROM_Read(EEPROM_ADDR_BOOTID, (uint8_t *)&rec, sizeof(rec)) != HAL_OK) {
        return 1U;                      /* no EEPROM - still return a usable id */
    }

    if (rec.magic != BOOTID_MAGIC) {
        rec.magic  = BOOTID_MAGIC;      /* blank or corrupt - start over */
        rec.bootId = 1U;
    } else {
        rec.bootId++;
    }

    (void)EEPROM_Write(EEPROM_ADDR_BOOTID, (uint8_t *)&rec, sizeof(rec));
    return rec.bootId;
}


/* ── Machine-usage counter ──────────────────────────────────────────────── */

#define USAGE_MAGIC 0x05A9U

/* The complement of the value acts as the integrity check: cheap, needs no CRC
 * routine, and catches a partial write or an erased/blank record. */
typedef struct __attribute__((packed))
{
    uint16_t magic;
    uint32_t usageMinutes;
    uint32_t check;          /* ~usageMinutes */
} EepromUsage;

bool EEPROM_SaveUsage(uint32_t usageMinutes)
{
    EepromUsage rec;
    rec.magic        = USAGE_MAGIC;
    rec.usageMinutes = usageMinutes;
    rec.check        = ~usageMinutes;

    return EEPROM_Write(EEPROM_ADDR_USAGE,
                        (uint8_t *)&rec, sizeof(rec)) == HAL_OK;
}

bool EEPROM_LoadUsage(uint32_t *usageMinutes)
{
    EepromUsage rec;

    if (usageMinutes == NULL) return false;

    if (EEPROM_Read(EEPROM_ADDR_USAGE,
                    (uint8_t *)&rec, sizeof(rec)) != HAL_OK) return false;

    if (rec.magic != USAGE_MAGIC)          return false;
    if (rec.check != (uint32_t)~rec.usageMinutes) return false;

    *usageMinutes = rec.usageMinutes;
    return true;
}


/* ── Filament library revision ──────────────────────────────────────────── */

#define FILREV_MAGIC 0x0F1AU

typedef struct __attribute__((packed))
{
    uint16_t magic;
    uint32_t revision;
    uint32_t check;          /* ~revision */
} EepromFilamentRev;

bool EEPROM_SaveFilamentRev(uint32_t revision)
{
    EepromFilamentRev rec;
    rec.magic    = FILREV_MAGIC;
    rec.revision = revision;
    rec.check    = ~revision;

    return EEPROM_Write(EEPROM_ADDR_FILAMENT_REV,
                        (uint8_t *)&rec, sizeof(rec)) == HAL_OK;
}

bool EEPROM_LoadFilamentRev(uint32_t *revision)
{
    EepromFilamentRev rec;

    if (revision == NULL) return false;

    if (EEPROM_Read(EEPROM_ADDR_FILAMENT_REV,
                    (uint8_t *)&rec, sizeof(rec)) != HAL_OK) return false;

    if (rec.magic != FILREV_MAGIC)                return false;
    if (rec.check != (uint32_t)~rec.revision)     return false;

    *revision = rec.revision;
    return true;
}
