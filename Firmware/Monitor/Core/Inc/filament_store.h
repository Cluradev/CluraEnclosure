/*
 * filament_store.h
 * CLURA ENCLOSURE - RevB firmware
 *
 * The filament library (spool brand names + empty-spool weights) on the SD card.
 *
 * ── How this differs from data.clu ──────────────────────────────────────
 * data.clu is strictly one-way: a technician writes it, the board reads it,
 * the board never writes back. Filament data is BIDIRECTIONAL - the machine
 * itself creates entries when a user registers a custom spool through the
 * calibration wizard - so the rules have to be tighter:
 *
 *   * The board WRITES filament.clu when the file is missing, and again
 *     whenever the library changes on the device (a spool is registered).
 *     It always writes back the same filamentRevision it last applied, so
 *     its own export can never look like a technician edit.
 *
 *   * The board READS filament.clu only when the file's filamentRevision is
 *     HIGHER than the one it last applied - same apply-once rule as data.clu.
 *     So a card can sit in the machine permanently without fighting it.
 *
 * Net effect: bump the revision in the file to push a library to boards;
 * leave it alone and the file simply tracks whatever the machine has.
 *
 * ⚠ One card shared between boards with DIFFERENT custom spools will end up
 * holding whichever board wrote last. Bump the revision to make one library
 * authoritative again.
 *
 * The EEPROM remains the runtime source of truth; the file is a mirror and a
 * distribution mechanism.
 */

#ifndef INC_FILAMENT_STORE_H_
#define INC_FILAMENT_STORE_H_

#include "parameters.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Outcome of the last Filament_Init(), for diagnostics. */
typedef enum {
    FIL_SRC_EEPROM = 0,   /* file absent or already applied - EEPROM kept    */
    FIL_SRC_CREATED,      /* filament.clu did not exist and was written out  */
    FIL_SRC_SD_APPLIED,   /* file had a newer revision and was imported      */
    FIL_SRC_NO_CARD       /* no SD card / unreadable                         */
} FilamentSource;

extern FilamentSource g_filamentSource;

/**
 * @brief  Reconcile the filament library with the SD card.
 *
 * Call once at boot, AFTER EEPROM_LoadFilamentData() has populated the arrays
 * and ms.filamentIndexMax, and after Tuning_Init().
 *
 *   - no filament.clu on the card  -> writes one from the current library
 *   - file revision > applied      -> imports it into the arrays + EEPROM
 *   - otherwise                    -> leaves everything alone
 *
 * @param brand   filament name table (updated in place on import)
 * @param weight  spool weight table  (updated in place on import)
 * @param indexMax  index of the last valid entry; updated in place on import
 */
void Filament_Init(char brand[NUM_OF_FILAMENTS][LEN_BUFFER],
                   int  weight[NUM_OF_FILAMENTS],
                   uint8_t *indexMax);

/**
 * @brief  Write the current library out to filament.clu, preserving the stored
 *         revision. Call after a spool is registered so the card mirrors the
 *         machine.
 * @retval true if the file was written
 */
bool Filament_ExportToSD(char brand[NUM_OF_FILAMENTS][LEN_BUFFER],
                         int  weight[NUM_OF_FILAMENTS],
                         uint8_t indexMax);

#ifdef __cplusplus
}
#endif

#endif /* INC_FILAMENT_STORE_H_ */
