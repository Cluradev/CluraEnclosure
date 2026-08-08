/*
 * filament_store.cpp
 * CLURA ENCLOSURE - RevB firmware
 *
 * See filament_store.h for the read/write rules.
 */

#include "filament_store.h"
#include "eeprom_storage.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ff.h"   /* carries its own extern "C" guards */

FilamentSource g_filamentSource = FIL_SRC_EEPROM;

/* Static, not stack-local: FatFs keeps this pointer while the volume is
 * mounted (the same trap the legacy csv helpers had). */
static FATFS s_filFatFs;

/* ── Small INI helpers ──────────────────────────────────────────────────── */

/**
 * Split "  key = value   # comment " into key/value.
 * Returns false for blank lines, comments and anything without '='.
 */
static bool splitKeyValue(char *line, char **outKey, char **outVal)
{
    char *eq;
    char *end;
    char *p = line;

    for (char *c = line; *c; c++) {
        if (*c == '#' || *c == ';') { *c = '\0'; break; }
    }

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\r' || *p == '\n') return false;

    eq = strchr(p, '=');
    if (eq == NULL) return false;
    *eq = '\0';

    end = eq - 1;
    while (end >= p && (*end == ' ' || *end == '\t')) *end-- = '\0';
    if (*p == '\0') return false;

    char *v = eq + 1;
    while (*v == ' ' || *v == '\t') v++;

    end = v + strlen(v) - 1;
    while (end >= v && (*end == ' ' || *end == '\t' ||
                        *end == '\r' || *end == '\n')) *end-- = '\0';

    *outKey = p;
    *outVal = v;
    return true;
}

/** Trim leading/trailing spaces in place and return the start. */
static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *end = s + strlen(s) - 1;
    while (end >= s && (*end == ' ' || *end == '\t')) *end-- = '\0';
    return s;
}

/* ── Export ─────────────────────────────────────────────────────────────── */

/**
 * Write a NUL-terminated string to the file.
 *
 * Always uses strlen() of what actually landed in the buffer. snprintf()
 * returns the length it WOULD have produced, so passing that value straight to
 * f_write() makes it read past the end of a truncated buffer and emit garbage
 * - which is exactly what happened when the header comment here outgrew
 * line[96]. Going through strlen() makes over-long text truncate harmlessly.
 */
static void writeLine(FIL *f, const char *text)
{
    UINT bw;
    (void)f_write(f, text, (UINT)strlen(text), &bw);
}

bool Filament_ExportToSD(char brand[NUM_OF_FILAMENTS][LEN_BUFFER],
                         int  weight[NUM_OF_FILAMENTS],
                         uint8_t indexMax)
{
    FIL  f;
    char line[96];
    char safeName[LEN_BUFFER + 1];
    uint32_t rev = 0;

    if (indexMax >= NUM_OF_FILAMENTS) indexMax = NUM_OF_FILAMENTS - 1;

    /* Write back whatever revision we last applied, so the board's own export
     * is never mistaken for a technician edit on the next boot. */
    (void)EEPROM_LoadFilamentRev(&rev);

    if (f_mount(&s_filFatFs, "", 1) != FR_OK) return false;
    if (f_open(&f, FILAMENT_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;

    writeLine(&f, "# CLURA filament library\r\n");
    writeLine(&f, "# Add entries as filament<n> = <name>,<weight g>. The highest n wins.\r\n\r\n");

    snprintf(line, sizeof(line), "filamentRevision = %lu\r\n", (unsigned long)rev);
    writeLine(&f, line);

    snprintf(line, sizeof(line), "filamentCount    = %u\r\n\r\n",
             (unsigned)(indexMax + 1));
    writeLine(&f, line);

    for (uint8_t i = 0; i <= indexMax; i++) {
        /* brand[] is filled with memcpy() elsewhere and is not guaranteed to be
         * null-terminated, so bound the copy and terminate it here. Commas would
         * break the field separator, so they are replaced. */
        memcpy(safeName, brand[i], LEN_BUFFER);
        safeName[LEN_BUFFER] = '\0';
        for (char *c = safeName; *c; c++) {
            if (*c == ',' || *c == '\r' || *c == '\n') *c = ' ';
        }

        snprintf(line, sizeof(line), "filament%-2u = %s,%d\r\n",
                 (unsigned)i, safeName, weight[i]);
        writeLine(&f, line);
    }

    return f_close(&f) == FR_OK;
}

/* ── Import ─────────────────────────────────────────────────────────────── */

/**
 * Parse filament.clu into the caller's tables.
 * Returns false if the file is missing/unreadable or carries no
 * filamentRevision - in which case nothing is touched.
 */
static bool readFilamentFile(char brand[NUM_OF_FILAMENTS][LEN_BUFFER],
                             int  weight[NUM_OF_FILAMENTS],
                             uint8_t *count,
                             uint32_t *fileRev)
{
    FIL  f;
    char line[96];
    bool haveRev = false;
    int  maxSeenIdx = -1;      /* highest filament<n> actually present */

    if (f_open(&f, FILAMENT_PATH, FA_READ) != FR_OK) return false;

    while (f_gets(line, sizeof(line), &f)) {
        char *key;
        char *val;

        if (!splitKeyValue(line, &key, &val)) continue;

        if (strcmp(key, "filamentRevision") == 0) {
            *fileRev = (uint32_t)strtoul(val, NULL, 10);
            haveRev  = true;
            continue;
        }
        if (strcmp(key, "filamentCount") == 0) {
            unsigned long c = strtoul(val, NULL, 10);
            if (c >= 1 && c <= NUM_OF_FILAMENTS) *count = (uint8_t)c;
            continue;
        }

        /* filament<n> = <name>,<weight> */
        if (strncmp(key, "filament", 8) == 0) {
            char *idxTxt = key + 8;
            if (*idxTxt < '0' || *idxTxt > '9') continue;   /* not an entry */

            unsigned long idx = strtoul(idxTxt, NULL, 10);
            if (idx >= NUM_OF_FILAMENTS) continue;

            char *comma = strchr(val, ',');
            if (comma == NULL) continue;
            *comma = '\0';

            char *name = trim(val);
            char *wTxt = trim(comma + 1);

            memset(brand[idx], 0, LEN_BUFFER);
            strncpy(brand[idx], name, LEN_BUFFER - 1);

            long w = strtol(wTxt, NULL, 10);
            if (w < 0)      w = 0;
            if (w > 100000) w = 100000;
            weight[idx] = (int)w;

            if ((int)idx > maxSeenIdx) maxSeenIdx = (int)idx;
        }
    }

    f_close(&f);

    /* The entries themselves define how many slots are live. Deriving it here
     * means adding a "filament16 = ..." line to the file is all that is needed
     * - previously filamentCount had to be bumped by hand as well, and if it
     * was not, the new spool appeared in the list but could not be selected
     * (the list renders up to NUM_OF_FILAMENTS, but selection is bounded by
     * filamentIndexMax). filamentCount is now only a fallback for a file that
     * carries no entries at all. */
    if (maxSeenIdx >= 0) {
        *count = (uint8_t)(maxSeenIdx + 1);
    }

    return haveRev;   /* no revision line -> ignore the file entirely */
}

/* ── Boot reconciliation ────────────────────────────────────────────────── */

void Filament_Init(char brand[NUM_OF_FILAMENTS][LEN_BUFFER],
                   int  weight[NUM_OF_FILAMENTS],
                   uint8_t *indexMax)
{
    uint32_t appliedRev = 0;
    uint32_t fileRev    = 0;
    FIL      probe;

    if (indexMax == NULL) return;

    if (!EEPROM_LoadFilamentRev(&appliedRev)) {
        appliedRev = 0;
        (void)EEPROM_SaveFilamentRev(0);
    }

    if (f_mount(&s_filFatFs, "", 1) != FR_OK) {
        g_filamentSource = FIL_SRC_NO_CARD;
        return;
    }

    /* Missing file -> create it from whatever the board currently holds, so a
     * blank card always ends up with a usable, editable library on it. */
    if (f_open(&probe, FILAMENT_PATH, FA_READ) != FR_OK) {
        if (Filament_ExportToSD(brand, weight, *indexMax)) {
            g_filamentSource = FIL_SRC_CREATED;
        } else {
            g_filamentSource = FIL_SRC_NO_CARD;
        }
        return;
    }
    f_close(&probe);

    /* Parse into scratch copies so a rejected file cannot half-apply. */
    static char scratchBrand[NUM_OF_FILAMENTS][LEN_BUFFER];
    static int  scratchWeight[NUM_OF_FILAMENTS];
    uint8_t     scratchCount = (uint8_t)(*indexMax + 1);

    memcpy(scratchBrand,  brand,  sizeof(scratchBrand));
    memcpy(scratchWeight, weight, sizeof(scratchWeight));

    if (!readFilamentFile(scratchBrand, scratchWeight, &scratchCount, &fileRev)) {
        g_filamentSource = FIL_SRC_EEPROM;
        return;
    }

    if (fileRev <= appliedRev) {
        g_filamentSource = FIL_SRC_EEPROM;   /* normal path - already applied */
        return;
    }

    /* Newer library on the card: adopt it. */
    memcpy(brand,  scratchBrand,  sizeof(scratchBrand));
    memcpy(weight, scratchWeight, sizeof(scratchWeight));
    if (scratchCount < 1)                 scratchCount = 1;
    if (scratchCount > NUM_OF_FILAMENTS)  scratchCount = NUM_OF_FILAMENTS;
    *indexMax = (uint8_t)(scratchCount - 1);

    (void)EEPROM_SaveFilamentData(brand, weight);
    (void)EEPROM_SaveFilamentRev(fileRev);

    /* filamentIndexMax lives in EepromConfig, NOT in the filament data block,
     * so saving the names and weights alone left it un-persisted: the imported
     * spools survived a reboot but the count did not, and on the next boot the
     * extra entries showed in the list yet could not be selected. */
    (void)EEPROM_SaveConfig(&ms);

    g_filamentSource = FIL_SRC_SD_APPLIED;
}
