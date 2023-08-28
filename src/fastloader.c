/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>

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


   fastloader.c: High level handling of fastloader protocols

*/

#include <string.h>
#include "config.h"
#include "crc.h"
#include "diskchange.h"
#include "doscmd.h"
#include "fastloader-ll.h"
#include "iec-bus.h"
#include "iec.h"
#include "led.h"
#include "progmem.h"
#include "timer.h"
#include "fastloader.h"

fastloaderid_t detected_loader;

/* Function pointer to the current byte transmit/receive functions */
/* (to simplify loaders with multiple variations of these)         */
uint8_t (*fast_send_byte)(uint8_t byte);
uint8_t (*fast_get_byte)(void);

/* track to load, used as a kind of jobcode */
volatile uint8_t fl_track;

/* sector to load, used as a kind of jobcode */
volatile uint8_t fl_sector;

#ifdef PARALLEL_ENABLED
/* parallel byte received */
volatile uint8_t parallel_rxflag;
#endif

/* Small helper for fastloaders that need to detect disk changes */
uint8_t check_keys(void) {
  /* Check for disk changes etc. */
  if (key_pressed(KEY_NEXT | KEY_PREV | KEY_HOME)) {
    change_disk();
  }
  if (key_pressed(KEY_SLEEP)) {
    reset_key(KEY_SLEEP);
    set_busy_led(0);
    set_dirty_led(1);

    /* wait for release */
    while (key_pressed(IGNORE_KEYS)) ;

    return 1;
  }

  return 0;
}

#ifdef CONFIG_BUS_SILENCE_REQ
/* ATN silence */
bool bus_sleep(UNUSED_PARAMETER) {
  iec_data.bus_state = BUS_SLEEP;

  /* we don't want the detected ATN-responder to persist */
  detected_loader = FL_NONE;

  return true;
}
#endif

#if defined(CONFIG_BUS_SILENCE_REQ) || defined(CONFIG_LOADER_KRILL) || defined(CONFIG_LOADER_BOOZE) || defined(CONFIG_LOADER_SPINDLE)
/* Calculate crc16 of command_buffer between the specified offsets */
uint16_t command_crc(const uint8_t start_offset, const uint8_t end_offset) {
  uint8_t  i;
  uint16_t crc;

  crc = 0xffff;

  for (i = start_offset; i < command_length-end_offset; i++)
    crc = crc16_update(crc, command_buffer[i]);

  return crc;
}
#endif

#if defined(CONFIG_LOADER_KRILL) || defined(CONFIG_LOADER_BOOZE) || defined(CONFIG_LOADER_SPINDLE) || defined(CONFIG_LOADER_BITFIRE)
/**
 * Wait for ATN low with a variable (but not very precise) timeout.
 *
 * @to: if != 0, this is the timeout in [ms]. The minimum timeout is 10ms
 *      and it is rounded down to next 10ms multiple if > 10ms. If == 0,
 *      there is no timeout, but this might be easier to achieve with just
 *      a "while (IEC_ATN);".
 *
 * Returns != 0 if timed out, 0 otherwise.
 */
uint8_t wait_atn_low(uint16_t to) {
  /* botch timeout using multiple 10ms timeouts */
  while (true) {
    start_timeout(10000); // caution: max. duration on AVR is 16000 == 16ms!

    while (to == 0 || !has_timed_out()) {
      if (!IEC_ATN)
        return false;
    }

    if (to <= 10)
      return true; // timed out

    to -= 10;
  }
}

/**
 * Write a byte LSB first using CLK and DATA as data lines. ATN is the
 * clock line and is driven by the host.
 *
 * @to:  if != 0, this is a timeout in [ms] applied when waiting for the
 *       falling ATN edge. The minimum timeout is 10ms and it is rounded
 *       down to next 10ms multiple if > 10ms (see wait_atn_low()).
 * @enc: if != NULL, pointer to a bit shuffle table used while sending.
 *       The table must consist of 8 bitmasks. The resulting byte has a
 *       bit corresponding to a table index set, if the bit as indicated
 *       by the bitmask at that index is set in the original byte.
 *
 * Returns != 0 if timeout occured, 0 otherwise.
 */
uint8_t clocked_write_byte(uint8_t b, const uint8_t *enc, uint16_t to) {
  uint8_t i, o;

  for (i = 0; i < 8; i +=2 ) {
    if (enc != NULL) { // shuffle bits if an encoding has been specified
      if (b & pgm_read_byte(enc+i)) {
        o = 0x01;
      } else {
        o = 0x00;
      }
      if (b & pgm_read_byte(enc+i+1)) {
        o |= 0x02;
      }
    } else {
      o = b;
      b >>= 2;
    }

    if (i&2) {
      if (wait_atn_low(to))
        return 1; // timeout
    } else {
      while (!IEC_ATN);
    }

    set_clock(o&1);
    set_data(o&2);
  }

  return 0;
}
#endif

#if defined(CONFIG_LOADER_KRILL)
/**
 * Read a byte from the "data" line, clocked by the "clk" line.
 * Data is read on both clock edges, LSB first and with bit values
 * inverted (bit = 1 if data line low). The transfer starts at the
 * next transition of the clock line, so the caller has to make
 * sure it is in the correct state when calling this function.
 *
 * @to: if != 0, the transfer is aborted if the clock line does not
 *      change for the specified time in [ms]. The caller has to
 *      check for a possible timeout by calling has_timed_out()
 *      immediately (!) after clocked_read_byte() returned.
 *
 * Returns the byte read or 0, if the timeout occured.
 */
uint8_t clocked_read_byte(iec_bus_t clk, iec_bus_t data, uint16_t to)  {
  uint8_t   i, b = 0;
  uint16_t  tc;
  iec_bus_t bus;

  bus = iec_bus_read();

  for (i = 8; i != 0; i--) {
    tc = to;
timeout_loop:
    start_timeout(10000);  // caution: max. duration on AVR is 16000 == 16ms!

    /* wait for respective clock edge */
    while ((iec_bus_read() & clk) == (bus & clk)) {
      if (tc != 0 && has_timed_out()) {
        /* Abort if the clock line hasn't changed before the timeout */
        if (tc <= 10)
          return 0; // timed out

        tc -= 10;

        goto timeout_loop;
      }
    }

    delay_us(2);
    bus = iec_bus_read();

    b = b >> 1 | (bus & data ? 0 : 0x80);
  }

  /* This is a hack to make it (a lot) less likely that a caller */
  /* mistakenly registers an intermediate timeout as real.       */
  /* TODO: An e.g. stop_timeout() function which both stops the  */
  /* timer and clears the timeout condition would be better.     */
  start_timeout(256);

  return b;
}
#endif

#if defined(CONFIG_LOADER_KRILL) || defined(CONFIG_LOADER_BOOZE) || defined(CONFIG_LOADER_SPINDLE) || defined(CONFIG_LOADER_BITFIRE)
/* Search a (loader-specfic) file quirks table for an entry with the given */
/* crc. Returns the pointer to the entry, if found, or NULL otherwise.     */
const file_quirks_t *get_file_quirks(const file_quirks_t *fq_table, uint16_t crc) {
  uint16_t c;

  while (true) {
    c = pgm_read_word(&fq_table->crc);

    if (c == 0)
      break;

    if (c == crc)
      return fq_table;

    fq_table++;
  }

  return NULL;
}
#endif

/*
 *
 *  GIJoe/EPYX common code
 *
 */
#if defined(CONFIG_LOADER_GIJOE) || defined(CONFIG_LOADER_EPYXCART)
/* Returns the byte read or <0 if the user aborts */
/* Aborting on ATN is not reliable for at least one version */
int16_t gijoe_read_byte(void) {
  uint8_t i;
  uint8_t value = 0;

  for (i=0;i<4;i++) {
    while (IEC_CLOCK)
      if (check_keys())
        return -1;

    value >>= 1;

    delay_us(3);
    if (!IEC_DATA)
      value |= 0x80;

    while (!IEC_CLOCK)
      if (check_keys())
        return -1;

    value >>= 1;

    delay_us(3);
    if (!IEC_DATA)
      value |= 0x80;
  }

  return value;
}
#endif


/*
 *
 *  Generic parallel speeder
 *
 */

#ifdef PARALLEL_ENABLED
/* parallel handshake interrupt handler */
PARALLEL_HANDLER {
  parallel_rxflag = 1;
}
#endif
