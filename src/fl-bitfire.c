/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Bitfire support:
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


   fl-bitfire.c: Handling of Bitfire loader

*/


#include <string.h>
#include "config.h"
#include "crc.h"
#include "buffers.h"
#include "d64ops.h"
#include "doscmd.h"
#include "errormsg.h"
#include "iec-bus.h"
#include "parser.h"
#include "timer.h"
#include "fastloader.h"


/* First dir sector on track 18 (next: DIR_START-1, DIR_START-2) */
#define INIT_TRACK      18
#define DIR_START       18

/* 0.x: 6 bytes per entry / 42 entries per sector / 3 sectors max. */
/* 1.x: 4 bytes per entry / 63 entries per sector / 2 sectors max. */
/* All have a maximum of 126 files in total (3*42 == 2*63 == 126). */
#define V0_DIR_ENTRIES  42
#define V1_DIR_ENTRIES  63
#define MAX_FILES       126

typedef struct {
  uint8_t  track, sector;
  uint16_t addr, length;
} dir_entry_v0_t;

typedef struct dir_entry_v1 {
  uint16_t addr, length;
} dir_entry_v1_t;

typedef union {
  dir_entry_v0_t v0;
  dir_entry_v1_t v1;
} dir_entry_t;

/* Nearly all known productions use an interleave of 4 sectors for all   */
/* tracks, except the pre-release loader revision used by 13:37, which   */
/* only uses interleave 4 on tracks <= 17 and 3 for all other tracks.    */
/* For now this exception is handled in iterate_file()/iterate_sector(). */
#define INTERLEAVE      4

#define LOAD_NEXT_CMD   0xef
#define RESET_CMD       0xff

/* block header ("preamble") fields */
typedef enum {
  IMM0, // 0x00 value

  LDLO, // file load address low-byte  - first block only
  LDHI, // file load address high-byte - first block only

  BALO, // block load address low-byte
  BAHI, // block load address high-byte
  BIDX, // zero-based block index

  BARR, // barrier (high-byte of last contiguous block's address + 1)
  BRDT, // barrier delta (shifted << 2)

  BLST, // block status
  BLEN, // block length

  FNUM, // file number (debug builds only)
} hdr_field_t;

#define MAX_HDR_LEN     6

/* The last byte is always the block length and can therefore */
/* (for now) be used as an implicit end marker.               */
static const PROGMEM hdr_field_t hdr_fields[][MAX_HDR_LEN] = {
  {             LDHI, LDLO,       BIDX, BLEN }, // 0.1
  { IMM0,       LDHI, LDLO,       BAHI, BLEN }, // 0.2/0.3
  { BRDT,       LDHI, LDLO,       BAHI, BLEN }, // 0.4/0.5
  { BRDT,       LDLO, LDHI,       BAHI, BLEN }, // 0.6 and 0.7 pre-releases
  { BRDT, FNUM, LDLO, LDHI,       BAHI, BLEN }, // 0.7 pre-release debug builds
  { BRDT,       LDLO, LDHI, BARR, BAHI, BLEN }, // 0.7
  { BLST,             BARR, BAHI, BALO, BLEN }, // 1.x
};

typedef struct {
  buffer_t      *dir_buf;
  uint8_t       dir_sector;     // loaded dir sector
  uint8_t       interleave;     // sector interleave on current track
  uint8_t       next_file;      // file index for "load next"
  uint8_t       track, sector;
  uint8_t       offset;         // byte offset from dir entry, if applicable
  uint16_t      file_crc;
  const uint8_t (*hdr_layout)[MAX_HDR_LEN];
} session_t;

/* block delays as hacks to make specific releases work */
static const PROGMEM file_quirks_t file_quirks[] = {
  { 0x3393,  40 }, // stacked / file $0a at $0b/$0a
  { 0x2b90,  60 }, // beats   / file $0f at $0c/$03

  { 0, 0 } // end marker
};

static uint8_t get_block_delay(uint16_t crc) {
  const file_quirks_t *fq;

  fq = get_file_quirks(file_quirks, crc);

  return fq != NULL ? pgm_read_byte(&fq->block_delay) : 0;
}

/* Can't use clocked_read_byte(), as we need to be able to    */
/* read the first bit immediately after the request line has  */
/* been set (which except for 0.1 is really just the first    */
/* clock edge). Also it's convenient to handle bus lock here. */
static uint8_t get_byte_1bit(iec_bus_t clk, iec_bus_t data) {
  uint8_t tc, i, b;

start:
  /* wait for host request while checking for abort and diskchange */
  while ((iec_bus_read() & (clk|IEC_BIT_ATN)) == (clk|IEC_BIT_ATN)) {
    if (check_keys())
      return RESET_CMD; // will cause the main loop to exit
  }

  if (!IEC_ATN) {
bus_locked:
    tc = 250; // timeout counter for host reset / bus lock detection

atn_timeout_loop:
    start_timeout(10000);

    while (!IEC_ATN) {
      if (tc != 0 && has_timed_out()) {
        if (--tc > 0)
          goto atn_timeout_loop;
      }
    }

    /* If ATN was low for over 2.5s assume it was bus lock and start over */
    if (tc == 0)
      goto start;

    /* ... otherwise enter the receive loop. (Which */
    /* will time out after 90ms if it was a reset). */
  }

  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    i = 8;
    b = 0;

    while (true) {
      tc = 9;
  timeout_loop:
      start_timeout(10000);

      /* wait for expected clock state */
      while (!!((iec_bus_read() & clk)) != (i&1)) {
        if (!IEC_ATN)
          goto bus_locked;

        if (has_timed_out()) {
          /* Abort if the clock line hasn't changed for 90ms (9 * 10ms) */
          if (--tc == 0)
            return 0;

          goto timeout_loop;
        }
      }

      delay_us(1);

      if (iec_bus_read() & data)
        b |= 0x80;

      if (--i == 0)
        break;

      b = b >> 1;
    }
  }

  /* try to prevent invalid timeouts, see comment in clocked_read_byte() */
  start_timeout(256);

  return b;
}

/* used by 1.2 */
uint8_t bitfire_get_byte_clk_data(void) {
  return get_byte_1bit(IEC_BIT_CLOCK, IEC_BIT_DATA);
}

/* used by 0.7 including pre-releases */
uint8_t bitfire_get_byte_clk_data_inv(void) {
  return ~bitfire_get_byte_clk_data();
}

/* used by <0.6 and 1.1 */
uint8_t bitfire_get_byte_data_clk(void) {
  return get_byte_1bit(IEC_BIT_DATA, IEC_BIT_CLOCK);
}

/* used by 0.6 and 1.0 */
uint8_t bitfire_get_byte_data_clk_inv(void) {
  return ~bitfire_get_byte_data_clk();
}

static uint8_t load_drivecode(void) {
  iec_bus_t eob;

  /* which bus line to wait for to be set at the end of each byte */
  eob = detected_loader < FL_BITFIRE_06 ? IEC_BIT_ATN : IEC_BIT_DATA;

  /* <= 0.5 wait for CLK, >= 0.6 for DATA; we just set both */
  set_clock(0);
  set_data(0);

  if (wait_atn_low(1000))
    return 1; // timed out

  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    set_clock(1);
    set_data(1);

    while (true) {
      /* drivecode download always uses CLK for data */
      bitfire_get_byte_data_clk();
      if (has_timed_out())
        goto done;

      start_timeout(150);
      while ((iec_bus_read() & eob) == eob) {
        if (has_timed_out())
          goto done;
      }
    }
  }

done:
  set_data(0);

  return 0;
}

static uint8_t load_dir(session_t *s, uint8_t sector) {
  dir_changed = 0;

  read_sector(s->dir_buf, current_part, INIT_TRACK, DIR_START-sector);
  if (current_error != ERROR_OK)
    return 1;

  s->dir_sector = sector;

  return 0;
}

/* Makes sure the dir sector for "file" is loaded. */
/* Returns "file" adjusted to this dir sector, or  */
/* 0xff if an error occured reading the sector.    */
static uint8_t update_dir(session_t *s, uint8_t file) {
  uint8_t eps, ds;

  /* entries per dir sector differ between versions */
  eps = detected_loader >= FL_BITFIRE_10 ? V1_DIR_ENTRIES : V0_DIR_ENTRIES;

  /* calculate dir sector and index into that sector for given file */
  for (ds = 0; file >= eps; file -= eps, ds++);

  if (s->dir_sector != ds) {
    if (load_dir(s, ds))
      return 0xff;
  }

  /* return file number adjusted to dir sector */
  return file;
}

/* Iterate one sector. Advances track if end of current track was reached. */
static void iterate_sector(session_t *s) {
  s->sector += s->interleave;
  if (s->sector >= d64_sectors_per_track(current_part, s->track)) {
    while (s->sector >= s->interleave) // avoid division for modulo op
      s->sector -= s->interleave;
    s->sector++;
    if (s->sector == s->interleave) { // track done
      s->sector = 0;
      while (++s->track == INIT_TRACK); // advance track but skip dir track
      if (detected_loader >= FL_BITFIRE_12PR3)
        s->interleave = s->track > 17 ? 3 : INTERLEAVE;
    }
  }
}

/* Looks up the entry specified by the index into the current dir sector  */
/* and populates either the v0 or the v1 part of dir_entry_t, depending   */
/* on the loader revision. First t/s and the byte offset for v1 revisions */
/* are then found by a separate call to iterate_file(), if necessary.     */
static void get_dir_entry(uint8_t *dir_buf, uint8_t i, dir_entry_t *e) {
  /* unconditionally initialize v0 part to make gcc happy */
  memcpy(&e->v0, dir_buf + i*sizeof(e->v0), sizeof(e->v0));

  switch (detected_loader) {
    case FL_BITFIRE_12PR3:
    case FL_BITFIRE_12PR2:
      e->v1.addr   = dir_buf[0x04+0*0x3f+i] | dir_buf[0x04+1*0x3f+i] << 8;
      e->v1.length = dir_buf[0x04+2*0x3f+i] | dir_buf[0x04+3*0x3f+i] << 8;
      break;
    case FL_BITFIRE_12PR1:
      i++; // first entry starts at byte offset 4
      /* falls through */
    case FL_BITFIRE_10:
    case FL_BITFIRE_11:
      memcpy(&e->v1, dir_buf + i*sizeof(e->v1), sizeof(e->v1));
      break;
    default: // 0.x; already handled above
      break;
  }
}

/* 1.x directory sectors only hold the start-position (track, logical   */
/* sector and byte-offset into this sector) of the sector's first file. */
/* Every time a random file is requested, its start position has to be  */
/* calculated by iterating over the lengths of the preceding entries.   */
static void iterate_file(session_t *s, uint8_t file) {
  uint8_t  i;
  uint16_t l;

  /* init values start at offset 0x00 (1.2) or 0xfc (1.0/1.1) */
  /* dir entries start at offset 0x04 (1.2) or 0x00 (1.0/1.1) */
  i = detected_loader >= FL_BITFIRE_12PR1 ? 0x00 : 0xfc;

  s->track  = s->dir_buf->data[i];
  s->offset = s->dir_buf->data[i+2];
  if (detected_loader >= FL_BITFIRE_12PR3)
    s->interleave = s->track > 17 ? 3 : INTERLEAVE;

  /* find the *first* file's start sector by iterating */
  /* the specified number of sectors from sector 0...  */
  s->sector = 0;
  i = s->dir_buf->data[i+1];
  while (i--)
    iterate_sector(s);

  /* ... and further iterate to the *requested* file's */
  /* start sector and byte-offset from there.          */
  for (i = 0; i < file; i++) {
    switch (detected_loader) {
      case FL_BITFIRE_12PR3:
      case FL_BITFIRE_12PR2:
        l = s->dir_buf->data[0x04+2*0x3f+i] | s->dir_buf->data[0x04+3*0x3f+i] << 8;
        break;
      case FL_BITFIRE_12PR1:
        l = ((dir_entry_v1_t *)&s->dir_buf->data[0x04+i*sizeof(dir_entry_v1_t)])->length;
        break;
      default: // 1.0 / 1.1
        l = ((dir_entry_v1_t *)&s->dir_buf->data[0x00+i*sizeof(dir_entry_v1_t)])->length;
        break;
    }

    for (l += s->offset + 1; l >= 256; l -= 256)
      iterate_sector(s);

    s->offset = (uint8_t)l;
  }
}

static uint8_t load_file(session_t *s, uint8_t file) {
  buffer_t    *buf;
  dir_entry_t dir_entry;
  uint8_t     bi, i, hlen, bdel;
  uint16_t    addr, blen, flen;
  hdr_field_t hf;
  uint8_t     hd[MAX_HDR_LEN];

  if (file == LOAD_NEXT_CMD)
    file = s->next_file;

  if (file >= MAX_FILES)
    return 1; // invalid file index

  bdel = get_block_delay(s->file_crc);
  s->file_crc = 0xffff;

  /* update_dir() makes sure the correct dir sector for the */
  /* requested file is loaded and returns the file index    */
  /* adjusted to that sector (or 0xff if an error occured). */
  i = update_dir(s, file);
  if (i == 0xff)
    return 1;

  get_dir_entry(s->dir_buf->data, i, &dir_entry);

  if (detected_loader >= FL_BITFIRE_10) {
    addr = dir_entry.v1.addr;
    if (detected_loader >= FL_BITFIRE_12PR1)
      addr += 0x100; // load adress needs an 0x100 offset
    flen = dir_entry.v1.length + 1;

    /* calculate track/sector/offset if either the first or a random file */
    if (file != s->next_file || s->next_file == 0)
      iterate_file(s, i);
  } else { /* pre 1.0 dir layout */
    s->track  = dir_entry.v0.track;
    s->sector = dir_entry.v0.sector;
    s->offset = 0;
    addr      = dir_entry.v0.addr;
    flen      = dir_entry.v0.length + 1;
  }

  buf = alloc_buffer();
  if (!buf)
    return 1;

  delay_ms(30); // needed at least by Incoherent Nightmare

  for (bi = 0;; bi++) {
    read_sector(buf, current_part, s->track, s->sector);
    if (current_error != ERROR_OK)
      return 1;

    if (bdel > 0)
      delay_ms(bdel);

    blen = s->offset + flen > 0x100 ? (uint16_t)0x100 - s->offset : flen;

    /* prepare header fields according to table for active protocol */
    for (i = 0, hf = 0, hlen = 0; hf != BLEN; i++) {
      hf = (hdr_field_t)pgm_read_byte((uint8_t *)s->hdr_layout+i);

      switch (hf) {
        case IMM0:
          hd[hlen++] = 0;
          break;

        case LDLO: // file load address low; only for first block
          if (bi > 0)
            break;
          // falls through
        case BALO: // block load address low
          hd[hlen++] = addr & 0xff;
          break;

        case LDHI: // file load address high; only for first block
          if (bi > 0)
            break;
          // falls through
        case BAHI: // block load address high
        case BARR: // as we send in order, barrier is == block high address
          hd[hlen++] = addr >> 8;
          break;

        case BIDX: // zero-based block index
          hd[hlen++] = bi;
          break;

        case BRDT: // barrier delta; shifted << 2, so bits 0&1 are always 0
          switch (bi) {
            case 0:
              hd[hlen++] = (uint8_t)(0xff << 2);
              break;
            case 1: // compensate for the first block (always 0xff)
              hd[hlen++] = 0x02 << 2;
              break;
            default:
              hd[hlen++] = 0x01 << 2;
              break;
          }
          break;

        case BLST: // block status
          hd[hlen++] = bi > 0 ? 0x80 : 0x00;
          break;

        case BLEN: // block length
          hd[hlen++] = blen & 0xff;
          break;

        case FNUM: // file number (debug builds only)
          if (bi == 0)
            hd[hlen++] = file;
          break;
      }
    }

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      set_clock(0);

      if (detected_loader == FL_BITFIRE_01) {
        /* 0.1 sends an ATN low pulse instead of the shifted first byte */
        if (wait_atn_low(1000))
          return 1; // timeout
        while (!IEC_ATN);
      }

      /* send prepared header */
      for (i = 0; i < hlen; i++) {
        if (clocked_write_byte(hd[i], NULL, 1000))
          return 1; // timeout
      }

      /* payload bytes are sent in reverse order */
      for (i = s->offset + blen - 1;; i--) {
        if (clocked_write_byte(buf->data[i], NULL, 1000))
          return 1; // timeout
        s->file_crc = crc16_update(s->file_crc, buf->data[i]);
        if (i == s->offset)
          break;
      }

      /* clocked_write_byte() exits with the last bitpair not yet ackowledged */
      while (!IEC_ATN);

      set_clock(1);
      set_data(0);
    }

    /* Update byte-position and move to the next sector, if */
    /* all of the current sector's data has been processed. */
    s->offset = (s->offset + blen) & 0xff;
    if (s->offset == 0)
      iterate_sector(s);

    flen -= blen; // remaining file length
    if (flen == 0)
      break; // done with this file

    addr += blen; // update load address for next sector
  }

  s->next_file = file + 1; // prepare for a potential "load next" command

  free_buffer(buf);

  return 0;
}

static uint8_t turn_disk(session_t *s, uint8_t disk_id) {
  uint8_t offs;

  /* disk id is located at offset 0x03 oder 0xff, depending on the revision */
  offs = detected_loader >= FL_BITFIRE_12PR1 ? 0x03 : 0xff;

  while (true) {
    /* load_dir() resets dir_changed */
    if (load_dir(s, 0))
      return 1;

    if (s->dir_buf->data[offs] == disk_id)
      break;

    /* wrong disk; wait for disk change, host reset or user abort */
    while (!dir_changed) {
      if (!IEC_ATN || check_keys()) // exit on host reset or long key press
        return 1;
    }

    /* disk changed, check again */
  }

  s->next_file = 0;

  return 0;
}

bool load_bitfire(uint8_t proto) {
  session_t session;
  uint8_t   cmd;

  memset(&session, 0, sizeof(session));
  session.file_crc = 0xffff;
  session.hdr_layout = &hdr_fields[proto];
  session.interleave = INTERLEAVE;

  session.dir_buf = alloc_system_buffer();
  if (session.dir_buf == NULL)
    goto exit;

  set_atn_irq(0);

  if (load_dir(&session, 0))
    goto exit;

  if (load_drivecode())
    goto exit;

  /* wait for >= 0.7 to release ATN */
  while (!IEC_ATN);

  while (true) {
    set_clock(1);
    set_data(1);
    delay_us(2);

    cmd = fast_get_byte();
    if (has_timed_out())
      goto exit; // timeout during receive; probably host reset
    set_data(0);

    if (cmd < 0xf0) {
      if (cmd == 0x80) // custom drivecode upload (not supported)
        goto exit;
      if (load_file(&session, cmd))
        goto exit; // error
    } else { // cmd >= 0xf0; reset or disk change
      if (cmd == RESET_CMD)
        goto exit;

      if (turn_disk(&session, cmd))
        goto exit; // error

      if (detected_loader <= FL_BITFIRE_03) {
        /* Disk change acknowledge for <= 0.3: Consume */
        /* one (<= 0.2) or two (0.3) ATN low pulses.  */
        set_clock(0); // disk changed

        while (IEC_ATN);
        while (!IEC_ATN);
        if (detected_loader == FL_BITFIRE_03) { // 0.3: two pulses
          if (wait_atn_low(10))
            goto exit; // probably host reset
          while (!IEC_ATN);
        }
      }
    }
  }

exit:
  /* dir buffer will be cleaned up by iec loop */

  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  /* loader no longer active past this point */
  detected_loader = FL_NONE;

  return true;
}
