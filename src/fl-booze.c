/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   BoozeLoader support:
   Copyright (C) 2023  Martin Thierer <mthierer@gmail.com>

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


   fl-booze.c: Handling of BoozeLoader

*/


#include <string.h>
#include "config.h"
#include "crc.h"
#include "buffers.h"
#include "d64ops.h"
#include "doscmd.h"
#include "errormsg.h"
#include "iec-bus.h"
#include "led.h"
#include "parser.h"
#include "timer.h"
#include "fastloader.h"


#define BOOT_TRACK      18
#define BOOT_SECTOR     0
#define DISK_ID_OFFSET  0xff

typedef struct {
  buffer_t *dir_buf, *buf;
  uint8_t  dir_sector;
  uint16_t file_crc;
} session_t;

/* block delays as hacks to make specific releases work */
static const PROGMEM file_quirks_t file_quirks[] = {
  { 0x3562, 120 }, // the elder scrollers    / file $19 at $1f/$04
  { 0x19b2, 120 }, // uncensored             / disk 2 file $10 at $11/$0e
  { 0xd41b, 240 }, // smart girls hate booze / file at $1b/$02
  { 0xe529, 240 }, // andropolis             / file at $17/$02

  { 0, 0 } // end marker
};

static uint8_t get_block_delay(uint16_t crc) {
  const file_quirks_t *fq;

  fq = get_file_quirks(file_quirks, crc);

  return fq != NULL ? pgm_read_byte(&fq->block_delay) : 0;
}

/* can't use clocked_read_byte() here as that reads on both clock edges */
static uint8_t get_byte_1bit(void) {
  uint8_t i, b = 0;

  for (i = 8; i != 0; i--) {
    set_data(1);

    ATOMIC_BLOCK ( ATOMIC_FORCEON ) {
      while (!IEC_ATN);
      delay_us(2);
      b = b >> 1 | (IEC_CLOCK ? 0 : 0x80);
      set_data(0);
    }

    if (wait_atn_low(1000))
      return 0; // timeout
  }

  /* try to prevent invalid timeouts, see comment in clocked_read_byte() */
  start_timeout(256);

  return b;
}

inline static bool is_valid_ts(uint8_t t, uint8_t s) {
  return t > 0 && t <= 42 && s < d64_sectors_per_track(current_part, t);
}

/* Check for a valid dir sector. If none is found, t/s addressing is used. */
/* "valid" == contains only valid t/s pairs (padded with 0s, if not full). */
static uint8_t find_dir(session_t *s) {
  uint8_t i;

  s->dir_buf = alloc_system_buffer();
  if (!s->dir_buf)
    return 1;

  /* known dir sectors are 12, 9 and 6 */
  for (s->dir_sector = 9;; s->dir_sector = s->dir_sector == 9 ? 12 : 6) {
    read_sector(s->dir_buf, current_part, BOOT_TRACK, s->dir_sector);
    if (current_error != ERROR_OK)
      return 1;

    /* check special case Andropolis (sector looks valid, but is not used) */
    if (s->dir_sector == 6 && s->dir_buf->data[0] != 1)
      break;

    for (i = 0;; i+= 2) {
      if (s->dir_buf->data[i] == 0) { // might be the start of the padding
        if (i < 2) // need at least one valid entry
          break;

        while (s->dir_buf->data[++i] == 0) { // check padding
          if (i == 0xff) // looks like a valid dir sector, use it
            return 0;
        }

        break; // done with this sector
      }

      if (!is_valid_ts(s->dir_buf->data[i], s->dir_buf->data[i+1]))
        break; // invalid entry
    }

    if (s->dir_sector == 6) // this was the last sector to check
      break;
  }

  /* no valid dir sector was found */
  free_buffer(s->dir_buf);
  s->dir_buf = NULL;
  s->dir_sector = 0;

  return 0;
}

/* Wait for a disk with the specified id, then read its */
/* directory. Only used for host-requested disk flips,  */
/* the initial disk's directory is read by find_dir().  */
static uint8_t load_dir(session_t* s, uint8_t disk_id) {
  /* first load the boot sector to check for the requested disk id */
  while (true) {
    dir_changed = 0;

    read_sector(s->dir_buf, current_part, BOOT_TRACK, BOOT_SECTOR);
    if (current_error != ERROR_OK)
      return 1;

    if (s->dir_buf->data[DISK_ID_OFFSET] == disk_id)
      break;

    /* wrong disk; wait for disk change, host reset or user abort */
    while (!dir_changed) {
      if (IEC_ATN || check_keys()) // exit on host reset or long key press
        return 1;
    }

    /* disk changed, check again */
  }

  /* we found the requested disk, load the dir sector */
  read_sector(s->dir_buf, current_part, BOOT_TRACK, s->dir_sector);
  if (current_error != ERROR_OK)
    return 1;

  /* acknowledge requested disk to host */
  set_data(1);
  while (!IEC_ATN);
  set_data(0);

  return 0;
}

/* Used to transfer both file blocks and the disk id for the disk flip */
/* of the t/s protocol, that's why the start offset p is a parameter.  */
static uint8_t send_block(uint8_t *data, uint8_t p, uint16_t *crc) {
  /* make sure the host is ready */
  if (wait_atn_low(1000))
    return 1; // timed out

  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    set_data(1); // we're ready

    do {
      if (clocked_write_byte(data[p], NULL, 4000)) // 4s timeout
        return 1;
      if (crc != NULL && p > 1)
        *crc = crc16_update(*crc, data[p]);
    } while (p++ != (data[0] != 0 ? 0xff : data[1]));

    /* clocked_write_byte() exits with the last bitpair not yet ackowledged */
    while (!IEC_ATN);

    set_clock(1);
    set_data(0); // busy
  }

  return 0;
}

/* send file starting at sector referenced by the first two bytes in s->buf */
static uint8_t send_file(session_t *s) {
  uint8_t  bdel;

  bdel = get_block_delay(s->file_crc);

  /* delay first block; needed at least for Neon, */
  /* Edge of Disgrace and The Elder Scrollers.    */
  delay_ms(60);

  s->file_crc = 0xffff;

  set_busy_led(1);

  while (s->buf->data[0] != 0) {
    if (bdel != 0)
      delay_ms(bdel);

    read_sector(s->buf, current_part, s->buf->data[0], s->buf->data[1]);
    if (current_error != ERROR_OK)
      return 1;

    if (send_block(s->buf->data, 0, &s->file_crc))
      return 1;
  }

  set_busy_led(0);

  return 0;
}

/* Bus lock makes the drive ignore all iec lines until it sees */
/* a L/H/L/H pattern on ATN where each phase is ~18us. The     */
/* drive acknowledges the pattern by setting DATA for 18us.    */
static void bus_locked(void) {
  uint8_t phase;

  phase = 0;

  set_data(1);

  /* We use a timeout of 30us (instead of 18us) to account for an   */
  /* additional delay from a possible interrupt. Otherwise we would */
  /* have to disable IRQs for the whole period (> 40s in "1991").   */
timeout_loop:
  start_timeout(30);

  while (true) {
    if (has_timed_out()) {
      if (phase == 4)
        break;

      phase = 0;
      goto timeout_loop;
    }
    if ((!IEC_ATN) != (phase & 1)) {
      phase++;
      goto timeout_loop;
    }
  }

  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    /* acknowledge the pattern by setting DATA for 18us */
    set_data(0);
    delay_us(18);
    set_data(1);
  }
}

bool load_booze(UNUSED_PARAMETER) {
  session_t session;
  uint8_t   cmd;

  if (detected_loader == FL_NONE) {
    /* possible drivecode install using M-E command, check command CRC */
    datacrc = command_crc(5, 2);

    if ((command_length != 0x29 || (datacrc != 0xe711 && datacrc != 0xab17)) &&
        (command_length != 0x27 || datacrc != 0xf674) &&
        (command_length != 0x26 || datacrc != 0xf700))
      return false;
  }

  set_atn_irq(0);
  set_data(0); // drive busy

  memset(&session, 0, sizeof(session));

  if (find_dir(&session))
    goto exit;

  /* if a valid directory sector was found, it's now in dir_buf */

  session.buf = alloc_system_buffer();
  if (!session.buf)
    goto exit;

  session.file_crc = 0xffff;

  while (true) {
    set_data(1);

    /* wait for host request while checking for abort and diskchange */
    while (IEC_ATN) {
      if (check_keys()) // exit loop on long key press
        goto exit;
    }

    cmd = get_byte_1bit();
    /* get_byte_1bit() exits with DATA set */

    if (has_timed_out() || IEC_ATN)
      goto exit; // probably host reset

    if (session.dir_sector != 0) {
      /* dir sector protocol */
      if ((cmd & 0x80) == 0) {
        cmd <<= 1; // file request; look up start t/s from dir buffer
        session.buf->data[0] = session.dir_buf->data[cmd];
        session.buf->data[1] = session.dir_buf->data[cmd+1];
      } else { // disk request or special command 0xff to lock bus
        if (cmd != 0xff) {
          if (load_dir(&session, cmd & 0x7f))
            goto exit; // error, user abort or timeout
        } else {
          bus_locked();
        }

        continue; // nothing to send, read next command
      }
    } else {
      /* t/s protocol */
      while (cmd == 0) {
        /* disk id check */
        dir_changed = 0;

        /* read boot sector and send disk id (last byte) */
        read_sector(session.buf, current_part, BOOT_TRACK, BOOT_SECTOR);
        if (current_error != ERROR_OK)
          goto exit;

        if (send_block(session.buf->data, DISK_ID_OFFSET, NULL))
          goto exit;

        /* "Let's scroll it" might take "forever" here, as it waits */
        /* for a keypress before continuing after the expected disk */
        /* has been identified. So no timeout waiting for ATN low.  */
        while (IEC_ATN);

        cmd = get_byte_1bit();
        if (has_timed_out())
          goto exit;
        if (cmd != 0)
          break; // host is happy with this disk; cmd is track of first file

        /* wrong disk; wait for disk change or user abort */
        while (!dir_changed) {
          if (IEC_ATN || check_keys()) // exit on host reset or long key press
            goto exit;
        }

        /* disk changed, check again */
      }

      session.buf->data[0] = cmd;             // track
      session.buf->data[1] = get_byte_1bit(); // sector
      if (has_timed_out())
        goto exit;
    }

    if (session.buf->data[0] == 0)
      goto exit; // loader done

    if (send_file(&session))
      goto exit; // error
  }

exit:
  /* buffers will be cleaned up by iec loop */

  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  /* loader no longer active past this point */
  detected_loader = FL_NONE;

  return true;
}
