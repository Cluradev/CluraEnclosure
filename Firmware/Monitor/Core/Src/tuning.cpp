/*
 * tuning.cpp
 * CLURA ENCLOSURE - RevB firmware
 *
 * Implementation of the runtime tuning block. See tuning.h for the design.
 *
 * Everything is driven by one table (TUNE_PARAMS): it supplies the firmware
 * defaults, the data.clu key names, the field offsets, AND the validation
 * bounds. Adding a parameter later means one struct field plus one table row
 * — no new parsing or validation code.
 */

#include "tuning.h"
#include "AT24C_EEPROM.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

#include "ff.h"   /* already carries its own extern "C" guards */

/* ── Globals ────────────────────────────────────────────────────────────── */

EepromTuning g_tune;
uint8_t      g_tuneClamped = 0;
TuneSource   g_tuneSource  = TUNE_SRC_DEFAULTS;

/* Static, NOT stack-local: FatFs keeps this pointer in its drive table for as
 * long as the volume is mounted. The legacy csv helpers mount a stack-local
 * FATFS and get away with it only because they re-mount on every call. */
static FATFS s_tuneFatFs;

/* ── Parameter table ────────────────────────────────────────────────────── */

typedef enum { T_U8, T_U16, T_U32, T_I16, T_I32 } TuneType;

typedef struct {
    const char *key;        /* name used in data.clu           */
    TuneType    type;
    uint16_t    offset;     /* offsetof() into EepromTuning     */
    int32_t     min;
    int32_t     max;
    int32_t     defval;     /* firmware default (parameters.h)  */
} TuneParam;

#define TP(field) offsetof(EepromTuning, field)

static const TuneParam TUNE_PARAMS[] = {
    /* key                   type    offset                     min      max        default */

    /* Servo / flap. Angles are bounded to the mechanism's real travel so a
     * typo cannot drive the servo into a hard stop (a stalled MG996R pulls
     * ~2.5 A continuously). */
    { "servoOpenAngle",     T_U8,  TP(servoOpenAngle),            1,      90,  OPEN_ANGLE            },
    { "servoCloseAngle",    T_U8,  TP(servoCloseAngle),           1,      90,  CLOSE_ANGLE           },
    { "servoHoldMs",        T_U16, TP(servoHoldMs),             100,    3000,  SERVO_HOLD_MS         },
    { "servoTempHyst",      T_U8,  TP(servoTempHyst),             1,      10,  SERVO_TEMP_HYST       },

    /* Safety: maxTemp drives the automatic flap-open. Bounded tightly so a
     * bad value cannot disable thermal protection. */
    { "maxTemp",            T_U8,  TP(maxTemp),                  35,      80,  MAX_TEMP              },

    /* Smoke / gas */
    { "lpgThreshold",       T_U16, TP(lpgThreshold),            100,    4095,  LPG_THRESHOLD         },
    { "smokeThreshold",     T_U16, TP(smokeThreshold),          100,    4095,  SMOKE_THRESHOLD       },
    { "smokeBeginTime",     T_U32, TP(smokeBeginTime),            0,  900000,  SMOKE_BEGIN_TIME      },
    { "countdownTime",      T_U8,  TP(countdownTime),             1,      60,  COUNTDOWN_TIME        },
    { "opticalAlarmEnable", T_U8,  TP(opticalAlarmEnable),        0,       1,  OPTICAL_ALARM_ENABLE  },
    { "smokeFireResponse",  T_U8,  TP(smokeFireResponse),         0,       1,  SMOKE_FIRE_RESPONSE   },

    /* Auto cooling */
    { "autoCoolTempMin",    T_I16, TP(autoCoolTempMin),           0,      60,  AUTOCOOL_TEMP_MIN     },
    { "autoCoolTempMax",    T_I16, TP(autoCoolTempMax),           0,      60,  AUTOCOOL_TEMP_MAX     },

    /* Auto filtering */
    { "autoFilterImpMin",   T_U16, TP(autoFilterImpMin),          0,    1000,  AUTOFILTER_IMPURITIES_MIN },
    { "autoFilterImpMax",   T_U16, TP(autoFilterImpMax),          0,    1000,  AUTOFILTER_IMPURITIES_MAX },
    { "autoFilterP03Min",   T_U16, TP(autoFilterP03Min),          0,   10000,  AUTOFILTER_P03_MIN    },
    { "autoFilterP03Max",   T_U16, TP(autoFilterP03Max),          0,   10000,  AUTOFILTER_P03_MAX    },

    /* Fans. The extra fan has no on-screen speed control, so it runs at this
     * fixed level whenever it is switched on. */
    { "extraFanSpeed",      T_U8,  TP(extraFanSpeed),             0,     100,  EXTRA_FAN_SPEED       },

    /* Intervals / display */
    { "bmeSampleInterval",  T_U32, TP(bmeSampleInterval),      1000,   60000,  BME_SAMPLE_INTERVAL   },
    { "usageLogInterval",   T_U32, TP(usageLogInterval),      60000, 3600000,  USAGE_LOG_INTERVAL    },
    { "sensorUsageWarnHours", T_U32, TP(sensorUsageWarnHours),    0,   20000,  SENSOR_USAGE_WARNING_HOURS },
    { "minBrightness",      T_U8,  TP(minBrightness),             5,     100,  MIN_BRIGHTNESS        },

    /* Load-cell factory seeds (first-boot only) */
    { "defaultLeftCal",     T_I32, TP(defaultLeftCal),            1,  100000,  DEFAULT_LEFT_CAL      },
    { "defaultRightCal",    T_I32, TP(defaultRightCal),           1,  100000,  DEFAULT_RIGHT_CAL     },
    { "defaultLeftOffset",  T_I32, TP(defaultLeftOffset), -2000000000, 2000000000, DEFAULT_LEFT_OFFSET  },
    { "defaultRightOffset", T_I32, TP(defaultRightOffset),-2000000000, 2000000000, DEFAULT_RIGHT_OFFSET },
    { "tareWeight",         T_I32, TP(tareWeight),                0,  100000,  TARE_WEIGHT           },
};

#define TUNE_PARAM_COUNT  (sizeof(TUNE_PARAMS) / sizeof(TUNE_PARAMS[0]))

/* ── Field access helpers ───────────────────────────────────────────────── */

static void writeField(EepromTuning *t, const TuneParam *p, int32_t v)
{
    uint8_t *field = (uint8_t *)t + p->offset;

    switch (p->type) {
        case T_U8:  *(uint8_t  *)field = (uint8_t)v;  break;
        case T_U16: *(uint16_t *)field = (uint16_t)v; break;
        case T_U32: *(uint32_t *)field = (uint32_t)v; break;
        case T_I16: *(int16_t  *)field = (int16_t)v;  break;
        case T_I32: *(int32_t  *)field = v;           break;
        default: break;
    }
}

/* ── CRC ────────────────────────────────────────────────────────────────── */

/** CRC-16/CCITT over every byte of the struct except the trailing crc16. */
static uint16_t tuneCrc(const EepromTuning *t)
{
    const uint8_t *p = (const uint8_t *)t;
    uint16_t crc = 0xFFFFU;
    size_t   len = offsetof(EepromTuning, crc16);

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ── Defaults / migration ───────────────────────────────────────────────── */

void Tuning_LoadDefaults(EepromTuning *t)
{
    memset(t, 0, sizeof(*t));

    for (unsigned i = 0; i < TUNE_PARAM_COUNT; i++) {
        writeField(t, &TUNE_PARAMS[i], TUNE_PARAMS[i].defval);
    }

    t->magic           = TUNE_MAGIC;
    t->structVersion   = TUNE_STRUCT_VERSION;
    t->appliedRevision = 0;
    t->crc16           = tuneCrc(t);
}

/**
 * Bring an older layout forward. Default ONLY the newly added fields and
 * leave everything else (including appliedRevision) intact, so a firmware
 * update never silently resets a machine's tuning.
 */
static void Tuning_Migrate(EepromTuning *t)
{
    /* v1 -> v2: servoOnAngle and servoOffAngle were removed when auto-servo
     * mode was dropped. That SHRANK the struct, so every field after them
     * shifted. A v1 record therefore fails the CRC check in Tuning_Init()
     * before this function is ever reached, and is cleanly replaced by the
     * firmware defaults with appliedRevision reset to 0 - which lets a
     * data.clu on the card re-provision the unit on that same boot.
     * Nothing to do here for that step.
     *
     * v2 -> v3: extraFanSpeed was inserted mid-struct, which likewise shifts
     * every field after it and fails the CRC for the same reason. Also handled
     * by the defaults path.
     *
     * v3 -> v4: opticalAlarmEnable inserted mid-struct - same story, handled by
     * the defaults path.
     *
     * v4 -> v5: smokeFireResponse, likewise.
     *
     * Future in-place migrations (fields APPENDED at the end, where earlier
     * offsets are preserved) go here:
     *   if (t->structVersion < 4) { t->newFieldInV4 = <default>; }
     */

    t->structVersion = TUNE_STRUCT_VERSION;
    t->crc16         = tuneCrc(t);
}

/* ── EEPROM I/O ─────────────────────────────────────────────────────────── */

static bool Tuning_LoadFromEeprom(EepromTuning *t)
{
    return EEPROM_Read(EEPROM_ADDR_TUNING,
                       (uint8_t *)t,
                       (uint16_t)sizeof(EepromTuning)) == HAL_OK;
}

bool Tuning_Save(void)
{
    g_tune.crc16 = tuneCrc(&g_tune);

    return EEPROM_Write(EEPROM_ADDR_TUNING,
                        (uint8_t *)&g_tune,
                        (uint16_t)sizeof(EepromTuning)) == HAL_OK;
}

/* ── data.clu parsing ───────────────────────────────────────────────────── */

/**
 * Split "  key = value   # comment" into key/value.
 * Returns false for blank lines, comments, and anything without '='.
 */
static bool splitKeyValue(char *line, char **outKey, char **outVal)
{
    char *eq;
    char *end;
    char *p = line;

    /* strip a trailing comment */
    for (char *c = line; *c; c++) {
        if (*c == '#' || *c == ';') { *c = '\0'; break; }
    }

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\r' || *p == '\n') return false;

    eq = strchr(p, '=');
    if (eq == NULL) return false;

    *eq = '\0';

    /* trim key's trailing whitespace */
    end = eq - 1;
    while (end >= p && (*end == ' ' || *end == '\t')) *end-- = '\0';
    if (*p == '\0') return false;

    /* trim value's leading whitespace */
    char *v = eq + 1;
    while (*v == ' ' || *v == '\t') v++;

    /* trim value's trailing whitespace / EOL */
    end = v + strlen(v) - 1;
    while (end >= v && (*end == ' ' || *end == '\t' ||
                        *end == '\r' || *end == '\n')) *end-- = '\0';

    *outKey = p;
    *outVal = v;
    return true;
}

/**
 * Look the key up in TUNE_PARAMS, clamp the value to its bounds, store it.
 * Unknown keys are ignored so a newer data.clu still works on older firmware.
 */
static void applyKey(EepromTuning *t, const char *key, const char *val)
{
    for (unsigned i = 0; i < TUNE_PARAM_COUNT; i++) {
        const TuneParam *p = &TUNE_PARAMS[i];

        if (strcmp(key, p->key) != 0) continue;

        int32_t v = (int32_t)strtol(val, NULL, 10);

        if (v < p->min) { v = p->min; if (g_tuneClamped < 255) g_tuneClamped++; }
        if (v > p->max) { v = p->max; if (g_tuneClamped < 255) g_tuneClamped++; }

        writeField(t, p, v);
        return;
    }
}

/**
 * Read data.clu into *out and report its configRevision.
 * Returns false if the card/file is missing, unreadable, or carries no
 * configRevision line — in which case NOTHING is applied. Rejecting an
 * unversioned file wholesale also makes a legacy JSON data.clu a safe no-op
 * (its lines use ':' not '=', so they are all skipped).
 */
static bool readTuningFile(EepromTuning *out, uint32_t *fileRev)
{
    FIL  f;
    char line[96];
    bool haveRev = false;

    if (f_mount(&s_tuneFatFs, "", 1) != FR_OK) return false;

    if (f_open(&f, CONFIG_PATH, FA_READ) != FR_OK) return false;

    while (f_gets(line, sizeof(line), &f) != NULL) {
        char *key;
        char *val;

        if (!splitKeyValue(line, &key, &val)) continue;

        if (strcmp(key, "configRevision") == 0) {
            *fileRev = (uint32_t)strtoul(val, NULL, 10);
            haveRev  = true;
            continue;
        }

        applyKey(out, key, val);
    }

    f_close(&f);
    return haveRev;
}

/**
 * Apply data.clu if it carries a newer configRevision than this board has
 * already consumed. Returns true only when the EEPROM was actually updated.
 */
static bool Tuning_ApplyFromSD(void)
{
    uint32_t     fileRev   = 0;
    EepromTuning candidate = g_tune;   /* start from CURRENT values, so a key
                                        * absent from the file keeps its value
                                        * rather than reverting to a default */

    g_tuneClamped = 0;

    if (!readTuningFile(&candidate, &fileRev)) return false;

    /* The normal every-boot path: already applied, do nothing (no EEPROM
     * write, so the card can stay in the machine permanently). */
    if (fileRev <= g_tune.appliedRevision) return false;

    candidate.appliedRevision = fileRev;
    candidate.magic           = TUNE_MAGIC;
    candidate.structVersion   = TUNE_STRUCT_VERSION;

    g_tune = candidate;
    return Tuning_Save();
}

/* ── Boot entry point ───────────────────────────────────────────────────── */

void Tuning_Init(void)
{
    bool eepromOk = Tuning_LoadFromEeprom(&g_tune);

    if (!eepromOk ||
        g_tune.magic != TUNE_MAGIC ||
        g_tune.crc16 != tuneCrc(&g_tune))
    {
        /* Fresh board, missing EEPROM, or corrupt block. appliedRevision
         * resets to 0, so an inserted card will re-provision this unit. */
        Tuning_LoadDefaults(&g_tune);
        g_tuneSource = TUNE_SRC_DEFAULTS;
        if (eepromOk) Tuning_Save();
    }
    else if (g_tune.structVersion != TUNE_STRUCT_VERSION)
    {
        /* Firmware was updated and the layout changed. Migrate BEFORE the SD
         * step, otherwise new values would be written into an old layout. */
        Tuning_Migrate(&g_tune);
        g_tuneSource = TUNE_SRC_MIGRATED;
        Tuning_Save();
    }
    else
    {
        g_tuneSource = TUNE_SRC_EEPROM;
    }

    if (Tuning_ApplyFromSD()) {
        g_tuneSource = TUNE_SRC_SD_APPLIED;
    }
}
