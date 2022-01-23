/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Krill's Loader support:
   Copyright (C) 2022  Martin Thierer <mthierer@gmail.com>

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


   fl-krill.c: High level handling of Krill's loader

*/

#include "config.h"

#include <stdbool.h>
#include <string.h>
#include "crc.h"
#include "buffers.h"
#include "d64ops.h"
#include "doscmd.h"
#include "errormsg.h"
#include "fastloader.h"
#include "iec-bus.h"
#include "parser.h"
#include "timer.h"
#include "uart.h"
#include "wrapops.h"


typedef struct {
  dh_t   dh;
  path_t path;
} dir_state_t;

static dir_state_t current_dir;

#define DC_CRC_LEN 0x200 /* must be less than the minimum parameter offset */

struct ld_variant_s {
  uint16_t crc;
  uint16_t length;
  uint16_t fn_maxlength_offs;
  uint16_t dir_track_offs;
  uint16_t config;
};

static const PROGMEM struct ld_variant_s ld_variants[] = {
  { 0xa161, 0x797, 0x216, 0x3a1, 0x0001 },
  { 0x562a, 0x797, 0x215, 0x384, 0x0000 },
  { 0x44ef, 0x458, 0x3af, 0xfff, 0x0000 },

  { 0, 0, 0, 0, 0 } // end marker
};

static const struct ld_variant_s *detected_variant;

static uint8_t fn_maxlength;
static uint8_t dir_track;

/* handlers for different custom code blocks */
typedef uint16_t (*cc_handler_t)(void);

static uint16_t cc_read_length(void);
static uint16_t cc_mem_backup(void);
static uint16_t cc_save_plugin(void);

struct cc_handler_s {
  uint16_t     crc;
  cc_handler_t handler;
  uint8_t      params;  // number of bytes to leave for the handler to process
};

#define CC_STAGE0_LEN 0x29

#define CC_CRC_LEN    (CC_STAGE0_LEN-2) // stage 0 loader minus parameters

static const PROGMEM struct cc_handler_s cc_handler_table[] = {
  { 0xf923, cc_read_length, 0x02 }, // custom drivecode loader stage 0
  { 0xab48, cc_mem_backup,  0x07 },
  { 0x5e85, cc_save_plugin, 0x11 },

  { 0, 0, 0 }, // end marker
};

/* number of bytes in the (simulated) drive memory backup */
static uint16_t backup_len;

/* fake M-R values for drive detection */
static magic_value_t mr_magic = { address: 0x300 };

void drive_detect_krill(UNUSED_PARAMETER) {
  uint8_t  i;
  uint16_t crc;

  /* expected command length for drvchkme */
  if (command_length != 0x1a)
    return;

  crc = 0xffff;

  for (i = 5; i < command_length; i++)
    crc = crc16_update(crc, command_buffer[i]);

  if (crc != 0xca5b) /* drvchkme */
    return;

  mr_magic.val[0] = ~(uint8_t)0; /* the first read will have returned 0 */

  custom_magic = &mr_magic;
  detected_loader = FL_KRILL;
}

static uint8_t wait_atn_low(void) {
  int16_t to;

  /* botch a ~1s timeout using multiple 16ms timeouts (max. duration on AVR) */
  for (to = 1000; to > 0; to -= 16) {
    start_timeout(16000);

    while (true) {
      if (!IEC_ATN)
        return 0;
      if (has_timed_out())
        break;
    }
  }

  return 1; /* timed out */
}

static uint8_t receive_byte(void) {
  uint8_t i, b = 0;

  for (i = 8; i != 0; i--) {
    while ((!IEC_DATA) == (i&1))
      if (IEC_ATN)
        return 0;

    delay_us(2);

    b = b >> 1 | (IEC_CLOCK ? 0 : 0x80);
  }

  return b;
}

static uint8_t send_byte_1bit(uint8_t b) {
  uint8_t i;

  for (i = 8; i != 0; i--) {
    set_data(!(b&0x80));
    b <<= 1;

    while ((!IEC_CLOCK) == (i&1))
      if (IEC_ATN)
        return 1;
  }

  return 0;
}

static uint8_t send_byte_2bit(uint8_t b) {
  uint8_t i;

  for (i = 4; i != 0; i--, b >>= 2) {
    if (i&1) { /* wait for ATN low with ~1s timeout */
      if (wait_atn_low())
        break; /* timed out */
    } else {
      while (!IEC_ATN); /* no timeout needed for ATN low -> high transition */
    }

    set_clock(b&1);
    set_data(b&2);
  }

  return IEC_ATN != 0; /* if ATN is not set here, something is wrong */
}

static uint8_t load_drivecode(void) {
  uint16_t i;
  uint16_t len;
  uint16_t crc;
  uint8_t  b;

  dir_track = 255;
  fn_maxlength = CBM_NAME_LENGTH;

  crc = 0xffff;
  len = DC_CRC_LEN;
  detected_variant = NULL;

  set_clock(1);
  set_data(1);

  delay_us(2);

  for (i = 0; i < len; i++) {
    b = receive_byte();
    if (IEC_ATN)
      return 1;
    crc = crc16_update(crc, b);

    /* read DC_CRC_LEN bytes and try to match the crc with a known variant */
    if (detected_variant != NULL) {
      if (i == pgm_read_word(&detected_variant->dir_track_offs)) {
        dir_track = b;
      } else if (i == pgm_read_word(&detected_variant->fn_maxlength_offs)) {
        fn_maxlength = b;
      }
    } else if (i == DC_CRC_LEN-1) {
      for (detected_variant = ld_variants;; detected_variant++) {
        if ( (len = pgm_read_word(&detected_variant->length)) == 0) {
          uart_puts_P(PSTR("Unknown drivecode, CRC "));
          uart_puthex(crc >> 8);
          uart_puthex(crc & 0xff);
          uart_putcrlf();

          detected_variant = NULL;
          return 1;
        }

        if (pgm_read_word(&detected_variant->crc) == crc)
          break;
      }
    }
  }

  return 0;
}

static uint8_t find_file(cbmdirent_t *dent) {
  if (command_buffer[0] != '\0') {
    if (opendir(&current_dir.dh, &current_dir.path))
      return 1;
  } else { /* empty filename -> read next file */
    command_buffer[0] = '*';
    command_buffer[1] = '\0';
  }

  return next_match(&current_dir.dh, command_buffer, NULL, NULL, 0, dent);
}

static buffer_t *get_file_buf(void) {
  buffer_t *buf;
  cbmdirent_t dent;
  uint8_t i;

  /* read filename */
  for (i = 0; i < fn_maxlength; i++) {
    if ( (command_buffer[i] = receive_byte()) == 0 )
      break;
  }
  if (IEC_ATN)
    return NULL;
  command_buffer[i] = '\0';

  set_clock(0);

  if (find_file(&dent))
    return NULL;

  buf = alloc_buffer();
  if (!buf)
    return NULL;

  open_read(&current_dir.path, &dent, buf);

  return buf;
}

static uint8_t send_file(void) {
  buffer_t *buf;
  uint8_t  hd[2];
  uint8_t  v;
  uint8_t  to = 0;
  uint16_t i;

  v = pgm_read_word(&detected_variant->config)&1;
  hd[v]   = 2;
  hd[1-v] = 2-v;

  buf = get_file_buf();
  if (!buf)
    hd[0] = 0xff; /* file not found or other error */

  while (true) {
    if (buf && buf->sendeoi) {
      hd[v]  |= 1;
      hd[1-v] = ~buf->lastused+1;
    }

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      /* data ready */
      set_data(0);
      set_clock(1);

      /* TODO: what does it indicate if the host releases ATN with CLK set? */

      to = send_byte_2bit(hd[0]);
      if (buf != NULL && !to) {
        to = send_byte_2bit(hd[1]);

        for (i = 2; i <= buf->lastused && !to; i++)
          to = send_byte_2bit(buf->data[i]);
      }

      /* send_byte_2bit() exits with ATN low (bitpair not yet ackowledged) */
      while (!IEC_ATN);

      /* busy */
      set_clock(0);
      set_data(1);
    }

    if (to || wait_atn_low() || buf == NULL)
      break;

    if (!buf->sendeoi) {
      if (!buf->refill(buf)) {
        hd[1-v]++;
        continue;
      }

      hd[0] = 0xff; /* error */
    } else {
      /* this was the last block, status ok  */
      hd[0] = 0;
    }

    cleanup_and_free_buffer(buf);
    buf = NULL;
  }

  return to;
}

static uint16_t cc_read_length(void) {
  uint8_t  i;
  uint16_t w;

  w = 0;

  for (i = 0; i < 2; i++) {
    w = w>>8 | receive_byte()<<8;
    if (IEC_ATN)
      return 0;
  }

  return ~w + 1;
}

static uint16_t cc_mem_backup(void) {
  uint16_t l;
  uint16_t i;
  uint8_t  b;
  uint8_t  pg;

  for (i = 0; i < 5; i++) {
    b = receive_byte();
    if (IEC_ATN)
      return 0;

    if (i == 0)
      pg = b;
  }

  l = cc_read_length();
  if (l == 0)
    return 0;

  while (!IEC_DATA || !IEC_CLOCK)
    if (IEC_ATN)
      return 0;

  /* TODO: the start offset (0x1e4) might be configuration-dependant, too */
  backup_len = 0x200 + (pg<<8) - 0x1e4;

  /* simulate drive memory backup; content doesn't seem to matter */
  for (i = 0; i < backup_len; i++)
    if (send_byte_1bit(i & 0xff))
      return 0;

  set_clock(0);
  set_data(0);

  delay_us(20);

  return l;
}

static uint8_t delete_file(path_t *path, cbmdirent_t dent) {
  /* file_delete() returns 255 in case of an error; both 0 and 1 are success */
  return file_delete(path, &dent) == 255;
}

static uint16_t cc_save_plugin(void) {
  buffer_t *buf;
  cbmdirent_t dent;
  uint8_t i;
  uint8_t st;

  for (i = 0; i < fn_maxlength+1; i++) {
    command_buffer[i] = receive_byte();
    if (IEC_ATN)
      return 0;
  }
  command_buffer[fn_maxlength] = '\0';

  while (!IEC_CLOCK)
    if (IEC_ATN)
      break;

  set_clock(0);
  set_data(0);

  st = 0xff;
  if (!IEC_ATN) {
    if (!find_file(&dent)) {
      /* allocate the buffer first to make sure we can create the file later */
      buf = alloc_buffer();

      /* for now just recreate the file; implement modify later, if necessary */
      if (buf && !delete_file(&current_dir.path, dent)) {
        /* used to check if open_write() succeeds */
        buf->lastused = 0;

        open_write(&current_dir.path, &dent, dent.typeflags & TYPE_MASK, buf, 0);
        if (buf->lastused != 0)
          st = dent.blocksize;
      }
    }
  } else {
    st = 0xfe; /* ATN abort */
  }

  set_clock(1);
  delay_us(2);

  if (send_byte_1bit(st) || st >= 0xfe)
    return 0;

  while (true) {
    set_clock(1);
    set_data(1);

    delay_us(2);

    i = receive_byte();
    if (IEC_ATN || i == 0xff) /* just to be safe, should not happen */
      break;

    if (i > 0) {
      mark_buffer_dirty(buf);

      while (i--) {
        buf->data[buf->position++] = receive_byte();
        if (IEC_ATN)
          return 0;
      }
    }

    set_clock(0);
    set_data(0);

    buf->lastused = buf->position-1;
    buf->mustflush = buf->position == 0;

    if (--st == 0) {
      /* clear remaining buffer data */
      memset(buf->data + buf->position, 0, 0x100 - buf->position);
      cleanup_and_free_buffer(buf);
      break;
    }

    if (buf->refill(buf))
      break;
  }

  set_clock(1);
  set_data(1);

  delay_us(2);

  /* simulate drive memory restore */
  while (backup_len--) {
    receive_byte();
    if (IEC_ATN)
      break;
  }

  return 0;
}

static uint8_t custom_code_handler(void) {
  int16_t  len;
  uint16_t crc;
  uint16_t i;
  uint8_t  b;
  const struct cc_handler_s *ptr;
  cc_handler_t handler;

  set_data(0);
  while (!IEC_CLOCK)
    if (IEC_ATN)
      return 1;

  len = CC_STAGE0_LEN;

  while (len != 0) {
    crc = 0xffff;
    handler = NULL;

    set_clock(1);
    set_data(1);

    delay_us(2);

    for (i = 0; i < len; i++) {
      b = receive_byte();
      if (IEC_ATN)
        return 1;

      if (i >= CC_CRC_LEN)
        continue;

      crc = crc16_update(crc, b);

      if (i == CC_CRC_LEN-1) {
        /* try to find a handler */
        for (ptr = cc_handler_table;; ptr++) {
          handler = (cc_handler_t)pgm_read_word(&ptr->handler);

          if (handler == NULL) {
            uart_puts_P(PSTR("Unhandled custom code, CRC "));
            uart_puthex(crc >> 8);
            uart_puthex(crc & 0xff);
            uart_putcrlf();

            return 1;
          }

          if (crc == pgm_read_word(&ptr->crc))
            break;
        }

        /* stop early so the handler can read its parameters */
        len -= pgm_read_byte(&ptr->params);

        if (len < i) { /* should not happen */
          handler = NULL;
          break;
        }
      }
    }

    /* if this happens there's a configuration error in the cc-table */
    if (handler == NULL)
      return 1;

    len = handler();
  }

  return 0;
}

static uint8_t update_dir(void) {
  partition_t *part;

  uart_puts_P(PSTR("dir changed\r\n"));

  part = &partition[current_part];

  current_dir.path.part = current_part;
  current_dir.path.dir  = part->current_dir;

  if (part->fop == &d64ops && dir_track <= part->d64data.last_track) {
    switch (part->imagetype & D64_TYPE_MASK) {
      case D64_TYPE_D41:
      case D64_TYPE_D71:
      case D64_TYPE_D81:
        /* use custom dir track */
        current_dir.path.dir.dxx.track = dir_track;
        break;
      default:
        break;
    }
  }

  dir_changed = 0;

  return opendir(&current_dir.dh, &current_dir.path);
}

void load_krill(UNUSED_PARAMETER) {
  uint8_t i;

  set_atn_irq(0);
  set_clock(0);

  while (IEC_ATN)
    if (check_keys())
      goto exit;

  if (load_drivecode())
    goto exit; /* unknown drive code */

  /* plausibility checks for parameters derived from drivecode */
  if (fn_maxlength < 1 || fn_maxlength > CBM_NAME_LENGTH)
    goto exit;

  if (dir_track < 1)
    dir_track = 255; /* this will fail as invalid and not be used */

  set_clock(0);

  /* force directory update */
  dir_changed = 1;

  while (!IEC_ATN) {
    /* wait for DATA low then high while checking for abort and diskchange */
    for (i = 0; i < 2; i++)
      while ((IEC_DATA != 0) != i)
        if (IEC_ATN || check_keys())
          break;

    /* make sure the dir handle and the path are in a usable state */
    if (dir_changed && update_dir())
      break;

    delay_us(10);
    if (IEC_ATN)
      break;

    set_clock(1);
    delay_us(2);

    if (IEC_CLOCK) {
      if (send_file())
        break;
    } else {
      if (custom_code_handler())
        break;
    }

    set_clock(0);
    set_data(1);

    free_multiple_buffers(FMB_USER_CLEAN);
  }

exit:
  set_clock(1);
  set_data(1);
  set_atn_irq(1);
}
