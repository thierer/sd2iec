/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   fastloader.h: Definitions for the high level fast loader handling

*/

#ifndef FASTLOADER_H
#define FASTLOADER_H

/* these two values are needed in the assembler implementation for AVR */
#define FLCODE_DREAMLOAD     1
#define FLCODE_DREAMLOAD_OLD 2

#ifndef __ASSEMBLER__

#include <stdbool.h>

#define UNUSED_PARAMETER uint8_t __attribute__((unused)) unused__

typedef enum {
  FL_NONE          = 0,
  FL_DREAMLOAD     = FLCODE_DREAMLOAD,
  FL_DREAMLOAD_OLD = FLCODE_DREAMLOAD_OLD,
  FL_TURBODISK,
  FL_FC3_LOAD,
  FL_FC3_SAVE,
  FL_FC3_FREEZED,
  FL_ULOAD3,
  FL_GI_JOE,
  FL_EPYXCART,
  FL_GEOS_S1_64,
  FL_GEOS_S1_128,
  FL_GEOS_S23_1541,
  FL_GEOS_S23_1571,
  FL_GEOS_S23_1581,
  FL_WHEELS_S1_64,
  FL_WHEELS_S1_128,
  FL_WHEELS_S2,
  FL_WHEELS44_S2,
  FL_WHEELS44_S2_1581,
  FL_NIPPON,
  FL_AR6_1581_LOAD,
  FL_AR6_1581_SAVE,
  FL_ELOAD1,
  FL_FC3_OLDFREEZED,
  FL_MMZAK,
  FL_N0SDOS_FILEREAD,
  FL_SAMSJOURNEY,
  FL_ULTRABOOT,
  FL_HYPRALOAD,
  FL_KRILL_SLEEP,
  FL_KRILL_R58PRE,
  FL_KRILL_R58,
  FL_KRILL_R146,
  FL_KRILL_R159,
  FL_KRILL_R164,
  FL_KRILL_R184,
  FL_KRILL_R186,
  FL_KRILL_R192,
  FL_BOOZE,
  FL_SPINDLE_SLEEP,
  FL_SPINDLE_21, // Spindle < 2.1 not supported
  FL_SPINDLE_22,
  FL_SPINDLE_23,
  FL_SPINDLE_3,
  FL_BITFIRE_SLEEP,
  FL_BITFIRE_01,
  FL_BITFIRE_03,
  FL_BITFIRE_04,
  FL_BITFIRE_06,
  FL_BITFIRE_07PRE, // 0.7 without barrier byte in header
  FL_BITFIRE_07DBG, // 0.7 without barrier byte and compiled with BITFIRE_DEBUG
  FL_BITFIRE_07,
  FL_BITFIRE_10,
  FL_BITFIRE_11,
  FL_BITFIRE_12PR1,
  FL_BITFIRE_12PR2,
  FL_BITFIRE_12,
  FL_SPARKLE_10,
  FL_SPARKLE_15,
  FL_SPARKLE_20,
  FL_SPARKLE_21,
} fastloaderid_t;

typedef struct {
  uint16_t  crc;         // crc of the *previous* file
  uint8_t   block_delay; // delay between block transfers; unit up to user
} file_quirks_t;

extern fastloaderid_t detected_loader;
extern volatile uint8_t fl_track;
extern volatile uint8_t fl_sector;
extern uint8_t (*fast_send_byte)(uint8_t byte);
extern uint8_t (*fast_get_byte)(void);

uint8_t check_keys(void);
bool bus_sleep(uint8_t);

# ifdef CONFIG_HAVE_IEC
/* per-loader functions, located in separate fl-*.c files */
bool load_turbodisk(uint8_t);
bool load_fc3(uint8_t freezed);
bool load_fc3oldfreeze(uint8_t);
bool save_fc3(uint8_t);
bool load_dreamload(uint8_t);
bool load_uload3(uint8_t);
bool load_eload1(uint8_t);
bool load_gijoe(uint8_t);
bool load_epyxcart(uint8_t);
bool load_geos(uint8_t);
bool load_geos_s1(uint8_t version);
bool load_wheels_s1(uint8_t version);
bool load_wheels_s2(uint8_t);
bool load_nippon(uint8_t);
bool load_ar6_1581(uint8_t);
bool save_ar6_1581(uint8_t);
bool load_mmzak(uint8_t);
bool load_n0sdos_fileread(uint8_t);
bool load_samsjourney(uint8_t);
bool load_ultraboot(uint8_t);
bool write_ultraboot(uint8_t);
bool format_ultraboot(uint8_t);
bool load_hypraload(uint8_t);
bool drvchkme_krill(uint8_t);
bool bus_sleep_krill(uint8_t);
bool load_krill(uint8_t);
bool load_booze(uint8_t);
bool load_spindle(uint8_t);
bool load_bitfire(uint8_t);
bool load_sparkle(uint8_t);

int16_t dolphin_getc(void);
uint8_t dolphin_putc(uint8_t data, uint8_t with_eoi);
void load_dolphin(void);
void save_dolphin(void);

void burst_fastload(void);

/* functions that are shared between multiple loaders */
/* currently located in fastloader.c                  */
int16_t gijoe_read_byte(void);
uint8_t clocked_read_byte(iec_bus_t clk, iec_bus_t data, uint16_t to);
uint8_t clocked_write_byte(uint8_t b, const uint8_t *enc, uint16_t to);

uint16_t command_crc(const uint8_t start_offset, const uint8_t end_offset);
uint8_t wait_atn_low(uint16_t timeout);
const file_quirks_t *get_file_quirks(const file_quirks_t *, uint16_t);
# endif // CONFIG_HAVE_IEC

# ifdef PARALLEL_ENABLED
extern volatile uint8_t parallel_rxflag;
static inline void parallel_clear_rxflag(void) { parallel_rxflag = 0; }
# else
#  define parallel_rxflag 0
static inline void parallel_clear_rxflag(void) {}
# endif

#endif // not assembler
#endif
