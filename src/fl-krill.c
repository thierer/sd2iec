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


#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "crc.h"
#include "buffers.h"
#include "d64ops.h"
#include "doscmd.h"
#include "errormsg.h"
#include "fastloader-ll.h"
#include "iec-bus.h"
#include "parser.h"
#include "timer.h"
#include "uart.h"
#include "ustring.h"
#include "wrapops.h"
#include "fastloader.h"


/* magic string used in M-E commands of >= r192 */
#define KRILL_MAGIC PSTR("KRILL")

/* offsets of the various configuration parameters into the ID string */
enum {
  ID_REPO_VER = 12,
  ID_PLATFORM = ID_REPO_VER + 2,
  ID_DRIVE,
  ID_DIRTRACK,
  ID_FN_MAXLEN,
  ID_CONFIG_INT,
};

/* handling of the various loader/protocol variants */

/* session state */
typedef struct {
  dh_t      dh;           /* directory handle (current seek state) */
  path_t    path;
  uint8_t   dir_track;    /* directory track (255 => default) */
  uint8_t   bam_sector;   /* BAM sector on directory track (0 => default) */
  uint8_t   fn_maxlength; /* Max. filename length (<= 16) */
  uint8_t   file_count;   /* number of files loaded in the current session */
  uint16_t  file_crc;     /* crc of the current file, used for file quirks */
  uint16_t  backup_len;   /* length of the (simulated) drive memory backup */
  int       ts_load:1;    /* files are addressed using track and sector    */
} session_t;

/* must be less than the minimum parameter offset in ld_variants[] */
#define DC_CRC_LEN 0xf0

typedef struct {
  uint16_t  crc; /* crc16 of the first DC_CRC_LEN bytes of the drivcode */
  uint16_t  fn_maxlength_offs;
  uint16_t  dir_track_offs;
} ld_variant_t;

/* only variants that have known variations from the defaults are listed */
static const PROGMEM ld_variant_t ld_variants[] = {
  { 0x4a88, 0x216, 0x3a1 }, // r186
  { 0x8ad2, 0x215, 0x384 }, // r184
  { 0xf4a2, 0x435, 0x1c5 }, // r164
  { 0x8d19, 0x435, 0x1c5 }, // r164
  { 0xec9c, 0x435, 0x1c5 }, // r164
  { 0x0519, 0x3b4, 0x3da }, // r146
  { 0x214c, 0x3a5, 0x3c8 }, // r146
  { 0x3154, 0x3a8, 0x3e6 }, // r146
  { 0x3e7a, 0x3c4, 0x3f9 }, // r146
  { 0x4c28, 0x3a3, 0x3c9 }, // r146
  { 0xcba6, 0x3b2, 0x3d8 }, // r146
  { 0xea6c, 0x3a4, 0x3ca }, // r146
  { 0x49e4, 0x318, 0x342 }, //  r58

  { 0, 0, 0 } // end marker
};

/* hacks to make specific releases work */

/* resolution in [ms] for block_delay in file_quirks_t */
#define BDEL_TIME 20

typedef struct {
  uint16_t  crc;         /* crc of the *previous* file */
  uint8_t   block_delay; /* delay between block transfers in ms */
} file_quirks_t;

static const PROGMEM file_quirks_t file_quirks[] = {
  { 0x1ba6,  20 }, /* coma light 13   / "SAMPLE"   */
  { 0xe5ac,  80 }, /* coma light 13   / "PICDAT"   */
  { 0xfe43,  80 }, /* protogeo 100%   / 13th file  */
  { 0x7f19, 120 }, /* pearls for pigs / "03"       */
  { 0x8e1e,  20 }, /* cause of death  / "PLOTBALL" */

  { 0, 0 } // end marker
};

/* handlers for different custom code blocks; only used for r186 */
typedef uint16_t (*cc_handler_t)(session_t *);

static uint16_t cc_read_length(session_t *);
static uint16_t cc_mem_backup(session_t *);
static uint16_t cc_save_plugin(session_t *);

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

/* check if the magic string is present in the command_buffer */
static bool magic_string_matches(void) {
  return ustrncmp_P(command_buffer+5, KRILL_MAGIC, strlen_P(KRILL_MAGIC)) == 0;
}

/* Universal handler for possible drvchkme memexecs */
/* f == 0: drvchkme of r146; detected by M-W, no further checks necessary */
/* f == 1: possible drvchkme of <= r186; check command crc                */
/* f == 2: possible drvchkme of >= r192; check command for magic string   */
bool drvchkme_krill(uint8_t f) {
  uint8_t  i;
  uint16_t crc;

  switch (f) {
  case 0: /* r146: M-W drvchkme crc matched */
    break;

  case 1: /* check command length and crc for drvchkme */
    if (command_length != 0x1a && command_length != 0x17) /* r159 is 0x17 */
      return false;

    crc = 0xffff;

    for (i = 5; i < command_length; i++)
      crc = crc16_update(crc, command_buffer[i]);

    if (crc != 0xca5b && crc != 0xf35b) /* drvchkme; r159 is 0xf35b */
      return false;

    break;

  case 2: /* >= r192 */
    if (!magic_string_matches())
      return false;

    break;

  default:
    return false;
  }

  custom_magic.address = 0x300;
  /* the first read will have returned '0' ("00, OK, .." in error_buffer[]) */
  custom_magic.val[0]  = ~'0';
  custom_magic.val[1]  = 0;
  custom_magic.drives  = 0xff; /* applicable for all drive types */

  return true;
}

bool bus_sleep_krill(uint8_t check_magic) {
  if (check_magic) { /* >= r192; M-W crc not matched, check magic string */
    if (!magic_string_matches())
      return false;
  } else {
    /* we don't want FL_KRILL_SLEEP to persist */
    detected_loader = FL_NONE;
  }

  /* activate bus sleep */
  return bus_sleep(0);
}

static uint8_t wait_atn_low(void) {
  int16_t to;

  /* botch a ~1s timeout using multiple 16ms timeouts (max. duration on AVR) */
  for (to = 1000; to > 0; to -= 16) {
    start_timeout(16000);

    while (!has_timed_out()) {
      if (!IEC_ATN)
        return 0;
    }
  }

  return 1; /* timed out */
}

static uint8_t get_byte_1bit(iec_bus_t clk, iec_bus_t data) {
  uint8_t   tc, i, b = 0;
  iec_bus_t bus;

  bus = iec_bus_read();

  for (i = 8; i != 0; i--) {
    tc = 9;
timeout_loop:
    start_timeout(10000);

    /* wait for respective clock edge */
    while ((iec_bus_read() & clk) == (bus & clk)) {
      if (has_timed_out()) {
        /* Abort if the clock line hasn't changed for 90ms (9 * 10ms) */
        if (--tc == 0)
          return 0;

        goto timeout_loop;
      }
    }

    delay_us(2);
    bus = iec_bus_read();

    b = b >> 1 | (bus & data ? 0 : 0x80);
  }

  return b;
}

/* used by <= r146 */
uint8_t krill_get_byte_clk_data(void) {
  return get_byte_1bit(IEC_BIT_CLOCK, IEC_BIT_DATA);
}

/* used by r164 for filenames */
uint8_t krill_get_byte_clk_atn(void) {
  return get_byte_1bit(IEC_BIT_CLOCK, IEC_BIT_ATN);
}

/* used by >r146 (in r164 only used for drivecode install) */
uint8_t krill_get_byte_data_clk(void) {
  return get_byte_1bit(IEC_BIT_DATA, IEC_BIT_CLOCK);
}

/* used by the save plugin for status byte and drive memory backup */
static uint8_t send_byte_1bit(uint8_t b) {
  uint8_t tc, i;

  for (i = 8; i != 0; i--) {
    set_data(!(b&0x80));
    b <<= 1;

    tc = 9;
timeout_loop:
    start_timeout(10000);

    while ((!IEC_CLOCK) == (i&1)) {
      if (IEC_ATN)
        return 1;
      if (has_timed_out()) {
        /* Abort if the clock line hasn't changed for 90ms (9 * 10ms) */
        if (--tc == 0)
          return 1;

        goto timeout_loop;
      }
    }
  }

  return 0;
}

static void send_bitpair_r58pre(uint8_t *b, uint8_t i) {
  switch (i) {
    case 4:
      set_clock(*b&0x80);
      set_data(*b&0x20);
      break;
    case 3:
      set_clock(*b&0x40);
      set_data(*b&0x10);
      break;
    case 2:
      set_clock(*b&0x08);
      set_data(*b&0x02);
      break;
    case 1:
      set_clock(*b&0x04);
      set_data(*b&0x01);
      break;
  }
}

static void send_bitpair(uint8_t *b, uint8_t i) {
  set_clock(*b&1);
  set_data(*b&2);
  *b >>= 2;
}

uint8_t krill_send_byte_atn(uint8_t b) {
  uint8_t i;
  void (*send_fn)(uint8_t *, uint8_t);

  if (detected_loader >= FL_KRILL_R58) {
    send_fn = send_bitpair;
  } else {
    b = ~b;
    send_fn = send_bitpair_r58pre;
  }

  for (i = 4; i != 0; i--) {
    if (i&1) { /* wait for ATN low with ~1s timeout */
      if (wait_atn_low())
        break; /* timed out */
    } else {
      while (!IEC_ATN); /* no timeout needed for ATN low -> high transition */
    }

    send_fn(&b, i);
  }

  return IEC_ATN != 0; /* if ATN is not set here, something is wrong */
}

static uint8_t load_drivecode(session_t *s) {
  uint16_t  i, len, crc, tmp;
  uint8_t   b, bp;
  iec_bus_t mask;
  const ld_variant_t *ptr, *detected_variant;

  crc = 0xffff;
  ptr = ld_variants;
  len = DC_CRC_LEN + 1;
  detected_variant = NULL;
  bp = 0;

  set_clock(0);

  /* wait for either ATN or DATA low, depending on the protocol version */
  mask = detected_loader >= FL_KRILL_R184 ? IEC_BIT_ATN : IEC_BIT_DATA;

  while ((iec_bus_read() & mask) == mask)
    if (check_keys())
      return 1;

  set_clock(1);
  set_data(1);

  delay_us(2);

  for (i = 0; ; i++) {
    b = fast_get_byte();

    if (has_timed_out()) {
      set_clock(0);
      if (detected_loader < FL_KRILL_R184)
        set_data(0);

      if (detected_variant == NULL) {
        uart_puts_P(PSTR("Unknown drivecode, CRC "));
        uart_puthex(crc >> 8);
        uart_puthex(crc & 0xff);
        uart_puts_P(PSTR(" loader "));
        uart_puthex(detected_loader);
        uart_putcrlf();
        uart_flush();
      }

      /* end of drivecode */
      break;
    }

    if (i > len)
      continue; /* ignore the rest with busy line set and wait for timeout */

    if (i == len) {
      /* set the respective busy line so the first request isn't made early */
      if (detected_loader >= FL_KRILL_R184) set_clock(0);
      else if (detected_loader == FL_KRILL_R164) set_data(0);
    }

    /* read DC_CRC_LEN bytes and try to match the crc with a known variant.  */
    /* versions >= r192 are configured from the id string; will never match. */
    if (detected_variant != NULL) {
      if (i == pgm_read_word(&detected_variant->dir_track_offs)) {
        s->dir_track = b;
      } else if (i == pgm_read_word(&detected_variant->fn_maxlength_offs)) {
        switch (detected_loader) {
          case FL_KRILL_R58PRE: /* should not happen, always 2 */
            tmp = 2;
            break;
          case FL_KRILL_R146:
            tmp = (b + 1) & 0x7f;
            break;
          case FL_KRILL_R164:
            tmp = (~b + 1) & 0xff;
            break;
          default: // r58, r184, r186
            tmp = b;
            break;
        }

        if (tmp > 0 && tmp < s->fn_maxlength)
          s->fn_maxlength = tmp;
      } else if (b < 13 && bp == 0xa0 && detected_loader <= FL_KRILL_R146 &&
                 i == pgm_read_word(&detected_variant->dir_track_offs)+2) {
        /* r58 and r146 support a custom dir start sector */
        s->bam_sector = b;
      }

      bp = b; /* remember previous byte for potential dir sector check */
      continue;
    }

    if (i < DC_CRC_LEN) {
      crc = crc16_update(crc, b);
      continue;
    }

    /* AVR at 8 MHz is too slow to check the whole table between */
    /* 2 bytes, so limit the time spent for searching to ~25 us. */
    for (start_timeout(25);; ptr++) {
      if ( (tmp = pgm_read_word(&ptr->crc)) == 0 ) /* end of table reached */
        break;

      if (has_timed_out()) {
        len++; /* continue searching the table after the next byte */
        break;
      }

      if (tmp != crc)
        continue;

      /* Found! Adjust len if parameter offsets are defined for this variant */
      if ( (tmp = pgm_read_word(&ptr->dir_track_offs) + 2) > len )
        len = tmp;
      if ( (tmp = pgm_read_word(&ptr->fn_maxlength_offs)) > len )
        len = tmp;

      detected_variant = ptr;
      break;
    }
  }

  return wait_atn_low();
}

/* set up the directory state after a disk change */
static void update_path(session_t *s) {
  partition_t *part;
  buffer_t    *buf;

  part = &partition[current_part];

  s->path.part = current_part;
  s->path.dir  = part->current_dir;

  if (part->fop == &d64ops && s->dir_track <= part->d64data.last_track) {
    switch (part->imagetype & D64_TYPE_MASK) {
      case D64_TYPE_D41:
      case D64_TYPE_D71:
        /* use custom dir track and sector*/
        s->path.dir.dxx.track = s->dir_track;

        if (s->bam_sector == 0)
          break; /* use standard dir start sector */

        /* read BAM sector to find the first dir sector */
        buf = alloc_buffer();
        if (!buf)
          break;

        read_sector(buf, current_part, s->dir_track, s->bam_sector);
        if (buf->data[0] == s->dir_track) /* safety check */
          s->path.dir.dxx.sector = buf->data[1];

        free_buffer(buf);
        break;

      case D64_TYPE_D81:
        /* use custom dir track */
        s->path.dir.dxx.track = s->dir_track;
        break;

      default:
        break;
    }
  }

  dir_changed = 0;
}

static uint8_t find_file(session_t *s, cbmdirent_t *dent) {
  if (command_buffer[0] != '*') {
    /* make sure the dir handle and the path are in a usable state */
    if (dir_changed)
      update_path(s);

    if (opendir(&s->dh, &s->path))
      return 1;

    /* force inclusion of entries with type 0 as hidden files for D64 images */
    if (partition[current_part].fop == &d64ops) {
      switch (partition[current_part].imagetype & D64_TYPE_MASK) {
        case D64_TYPE_D41:
        case D64_TYPE_D71:
        case D64_TYPE_D81:
          s->dh.dir.d64.hidden = 1;
          break;
        default:
          break;
      }
    }
  }

  return next_match(&s->dh, command_buffer, NULL, NULL, FLAG_HIDDEN, dent);
}

/* Simple check for possible T/S adressing. Not very */
/* reliable, but works for all known productions.    */
static bool is_valid_ts(void) {
  return detected_loader <= FL_KRILL_R146 &&
    command_buffer[1] <= 20 && command_buffer[0] <= 41 && command_buffer[0] > 0;
}

/* emulate the required minimum of d64_read() */
static uint8_t next_sector(buffer_t *buf) {
  read_sector(buf, current_part, buf->data[0], buf->data[1]);
  buf->sendeoi  = buf->data[0] == 0;
  buf->lastused = buf->sendeoi ? buf->data[1] : 255;

  return current_error != ERROR_OK;
}

/* Read a filename into the command_buffer using */
/* the respective 1 bit receive protocol.        */
/* Returns the number of bytes read (which might */
/* be more than fn_maxlength bytes for >= r192,  */
/* if it's really a custom drivecode upload).    */
static uint8_t read_filename(session_t *s) {
  uint8_t i, max_len;

  /* r192 might send more than 16+1 bytes, if */
  /* it is in fact a custom drivecode upload. */
  if (detected_loader >= FL_KRILL_R192)
    max_len = CBM_NAME_LENGTH+2;
  else
    max_len = s->fn_maxlength;

  set_clock(1);
  set_data(1);

  /* read filename or track/sector */
  for (i = 0; i < max_len; i++) {
    if ( (command_buffer[i] = fast_get_byte()) == 0 ) // could also be timeout
      break;

    /* Stop early if T/S loading or first file and valid T/S for D41.     */
    /* This auto-detection of T/S addressing would fail for one-character */
    /* filenames with a PETSCII code <= 41, but no affected productions   */
    /* are known. Also wouldn't work for valid track and sector ranges of */
    /* productions using D71 or D81 images, but none are known, either.   */
    if (i == 1 && (s->ts_load || (s->file_count == 0 && is_valid_ts())))
      break;
  }

  if (detected_loader != FL_KRILL_R164) set_clock(0);
  set_data(0);

  if (i == 0) {
    command_buffer[0] = '*'; /* match next */
    command_buffer[1] = '\0';
  } else if (s->fn_maxlength < CBM_NAME_LENGTH) {
    /* Only fn_maxlength bytes have to match, even though on >= r192 */
    /* the host could in theory send a longer filename/pattern.      */
    command_buffer[s->fn_maxlength]   = '*'; /* match prefix */
    command_buffer[s->fn_maxlength+1] = '\0';
  }

  return i;
}

/* Open a buffer for the file specified in command_buffer: */
/* Either a string (null-terminated if less than 16 bytes) */
/* or two bytes specifying start-track and -sector.        */
static buffer_t *get_file_buf(session_t *s) {
  buffer_t *buf;
  cbmdirent_t dent;

  buf = alloc_buffer();
  if (!buf)
    return NULL;

  if (!s->ts_load && !find_file(s, &dent)) {
    open_read(&s->path, &dent, buf);
  } else {
    if (!s->ts_load) {
      /* switch to T/S addressing if first file and valid T/S, else error */
      if (s->file_count > 0 || !is_valid_ts()) {
        free_buffer(buf);
        return NULL;
      }

      s->ts_load = 1;
    }

    /* track/sector load */
    buf->data[0] = command_buffer[0];
    buf->data[1] = command_buffer[1];

    /* read first sector */
    if (next_sector(buf))
      return NULL;

    buf->refill = next_sector;
  }

  s->file_count++;

  return buf;
}

static uint8_t get_block_delay(uint16_t crc) {
  uint16_t c;
  const file_quirks_t *ptr;

  for (ptr = file_quirks;; ptr++) {
    c = pgm_read_word(&ptr->crc);

    if (c == 0)
      break;

    if (c == crc)
      return pgm_read_byte(&ptr->block_delay);
  }

  return 0;
}

static uint8_t send_file(session_t *s) {
  buffer_t *buf;
  uint8_t  hd[2] = {0xff, 0};
  uint8_t  bi;
  uint8_t  to = 0;
  uint16_t i;
  uint8_t bdel;

  bdel = get_block_delay(s->file_crc);

  buf = get_file_buf(s);
  /* no "file not found" error for "next file" at the end of the directory */
  if (!buf && !dir_changed && command_buffer[0] == '*')
    hd[0] = detected_loader > FL_KRILL_R146 ? 0 : 0xfe;

  bi = 0;

  s->file_crc = 0xffff;

  while (true) {
    if (buf) {
      /* prepare block metadata */
      switch (detected_loader) {
        case FL_KRILL_R58PRE:
        case FL_KRILL_R58:
        case FL_KRILL_R146:
          /* the EOF marker is the status byte (0xfe) */
          hd[0] = bi;
          hd[1] = buf->lastused-2;
          break;
        case FL_KRILL_R159:
        case FL_KRILL_R164:
          hd[0] = 0x82 - buf->sendeoi;
          hd[1] = !buf->sendeoi ? bi+2 : ~buf->lastused+2;
          break;
        case FL_KRILL_R184:
          hd[0] = 2 | buf->sendeoi;
          hd[1] = !buf->sendeoi ? bi+2: ~buf->lastused+1;
          break;
        default: // >= R186
          hd[0] = !buf->sendeoi ? bi+1: ~buf->lastused+1;
          hd[1] = 2 | buf->sendeoi;
          break;
      }
    }

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      /* data ready */
      set_data(detected_loader == FL_KRILL_R164);
      set_clock(detected_loader != FL_KRILL_R164);

      if (detected_loader <= FL_KRILL_R146)
        while (IEC_ATN);

      /* check for "file exists" test (CLK set by host) */
      if (detected_loader >= FL_KRILL_R192) {
        while (!IEC_ATN && IEC_CLOCK);

        if (!IEC_CLOCK) {
          while (!IEC_ATN);
          set_data(buf != NULL);
          /* buffer will be cleaned up by main loop */
          buf = NULL;
          goto abort;
        }
      }

      to = fast_send_byte(hd[0]);
      if (buf != NULL && !to) {
        to = fast_send_byte(hd[1]);

        for (i = 2; i <= buf->lastused && !to; i++) {
          to = fast_send_byte(buf->data[i]);
          s->file_crc = crc16_update(s->file_crc, buf->data[i]);
        }
      }

      /* fast_send_byte() exits with ATN low (bitpair not yet ackowledged) */
      while (!IEC_ATN);

      /* busy */
      set_clock(buf != NULL && detected_loader == FL_KRILL_R164);
      set_data(buf != NULL && detected_loader != FL_KRILL_R164);

      if (buf != NULL) {
        for (i = 0; i < bdel; i += BDEL_TIME)
          delay_ms(BDEL_TIME);
      }
    }

abort:
    if (buf == NULL && detected_loader == FL_KRILL_R58PRE)
      while (IEC_ATN);

    if (to || ((detected_loader > FL_KRILL_R146 || buf == NULL) && wait_atn_low()) || buf == NULL)
      break;

    if (!buf->sendeoi) {
      if (!buf->refill(buf)) {
        bi++;
        continue;
      }

      hd[0] = 0xff; /* error */
    } else {
      /* this was the last block, status ok */
      hd[0] = detected_loader > FL_KRILL_R146 ? 0 : 0xfe;
    }

    cleanup_and_free_buffer(buf);
    buf = NULL;
  }

  return to;
}

static uint16_t cc_read_length(session_t *s) {
  uint8_t  i;
  uint16_t w;

  w = 0;

  for (i = 0; i < 2; i++) {
    w = w>>8 | fast_get_byte()<<8;
    if (IEC_ATN)
      return 0;
  }

  return ~w + 1;
}

static uint16_t cc_mem_backup(session_t *s) {
  uint16_t i;

  for (i = 0; i < 5; i++) {
    fast_get_byte();
    if (IEC_ATN)
      return 0;
  }

  i = cc_read_length(s);
  if (i == 0)
    return 0;

  while (!IEC_DATA || !IEC_CLOCK)
    if (IEC_ATN)
      return 0;

  /* simulate drive memory backup */
  for (s->backup_len = 0; ; s->backup_len++) {
    /* send 0xff so DATA is kept low and we can wait for the timeout */
    if (send_byte_1bit(0xff)) {
      if (IEC_ATN)
        return 0;

      break;
    }
  }

  set_clock(0);
  set_data(0);

  delay_us(20);

  return i;
}

/* Save file with save plugin protocol, filename in command_buffer */
static uint8_t cc_save_file(session_t *s) {
  uint8_t i;
  buffer_t *buf;
  cbmdirent_t dent;
  uint8_t st;

  if (!find_file(s, &dent)) {
    /* don't rely on file_delete() and open_write() preserving the filesize */
    st = dent.blocksize;

    /* allocate the buffer first to make sure we can create the file later */
    buf = alloc_buffer();

    if (buf != NULL && file_delete(&s->path, &dent) == 1) {
      open_write(&s->path, &dent, dent.typeflags & TYPE_MASK, buf, 0);
    } else {
      st = 0xfe; /* write protect */
    }
  } else {
    st = 0xff; /* not found */
  }

  set_clock(1);
  delay_us(2);

  if (send_byte_1bit(st) || st >= 0xfe) /* send timed out or error */
    return 1;

  while (true) {
    set_clock(1);
    set_data(1);

    delay_us(2);

    i = fast_get_byte();
    if (IEC_ATN || has_timed_out()) /* just to be safe, should not happen */
      return 1;

    if (i > 0) {
      mark_buffer_dirty(buf);

      while (i--) {
        buf->data[buf->position++] = fast_get_byte();
        if (IEC_ATN)
          return 1;
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
      return 1;
  }

  return 0;
}

/* save plugin handler specific to r186 (only Scramble Infinity 1.2) */
static uint16_t cc_save_plugin(session_t *s) {
  uint8_t i;

  for (i = 0; i < CBM_NAME_LENGTH+1; i++) {
    command_buffer[i] = fast_get_byte();
    if (IEC_ATN)
      return 0;
  }

  while (!IEC_CLOCK)
    if (IEC_ATN)
      return 0;

  set_clock(0);
  set_data(0);

  if (cc_save_file(s))
    return 0;

  set_clock(1);
  set_data(1);

  delay_us(2);

  /* simulate drive memory restore */
  while (s->backup_len--) {
    fast_get_byte();
    if (IEC_ATN)
      break;
  }

  return 0;
}

/* Fallback (assuming save plugin) for unknown custom drivecode. Only */
/* works with protocol used by r192 and later. Others will error out. */
static uint8_t cc_fallback(session_t *s) {
  uint8_t i, b;

  set_clock(0);

  /* plugin loader code */
  do {
    fast_get_byte();
  } while (!has_timed_out());

  set_clock(1);
  set_data(0);
  while (IEC_CLOCK);

  /* simulate drive memory backup */
  do {
    /* send 0xff so DATA is kept low and we can wait for the timeout */
    send_byte_1bit(0xff);
  } while (!has_timed_out());

  set_clock(1);
  set_data(1);

  /* save plugin code & filename (last fn_maxlength+1 bytes). */
  /* collect the last fn_maxlength+1 bytes in a ring buffer.  */
  for (i = 0;; i = (i == s->fn_maxlength ? 0 : i+1)) {
    b = fast_get_byte();
    if (has_timed_out())
      break;

    ops_scratch[i] = b;
  } while (!has_timed_out());

  /* handshake before starting payload transfer */
  set_data(0);
  while (IEC_CLOCK && !IEC_ATN);
  set_clock(0);
  set_data(1);
  while (!IEC_DATA && !IEC_ATN);
  if (IEC_ATN)
    return 1;

  /* extract filename from ring buffer */
  for (b = 0; ops_scratch[i] != 0; i = (i == s->fn_maxlength ? 0 : i+1))
    command_buffer[b++] = ops_scratch[i];

  if (s->fn_maxlength < CBM_NAME_LENGTH)
    command_buffer[b++] = '*'; /* match prefix */
  command_buffer[b] = '\0';

  /* receive payload and write to file */
  if (cc_save_file(s))
    return 1;

  set_clock(1);
  set_data(1);

  /* simulate restore drive memory */
  do {
    fast_get_byte();
    set_clock(0); /* set busy line after the first byte to enable timeout */
  } while (!has_timed_out());

  set_data(0);

  return 0;
}

static uint8_t custom_code_handler(session_t *s) {
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
      b = fast_get_byte();
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
            uart_puts_P(PSTR("Unhandled custom drivecode, CRC "));
            uart_puthex(crc >> 8);
            uart_puthex(crc & 0xff);
            uart_putcrlf();

            return cc_fallback(s);
          }

          if (crc == pgm_read_word(&ptr->crc))
            break;
        }

        /* stop early so the handler can read its parameters */
        len -= pgm_read_byte(&ptr->params);

        if (len < i) /* should not happen */
          return 1;
      }
    }

    len = handler(s);
  }

  return 0;
}

bool load_krill(UNUSED_PARAMETER) {
  int8_t    fn_len;
  iec_bus_t req_line;
  session_t session;

  memset(&session, 0, sizeof(session));


  if (detected_loader == FL_NONE && command_length >= 19) {
    if (ustrncmp_P(command_buffer+5, KRILL_MAGIC, strlen_P(KRILL_MAGIC)))
      return false; /* magic string didn't match */

    detected_loader = FL_KRILL_R192;
    fast_get_byte  = krill_get_byte_data_clk;
    fast_send_byte = krill_send_byte_atn;

    /* M-E with ID-string; extract parameters */
    session.dir_track    = command_buffer[ID_DIRTRACK];
    session.fn_maxlength = command_buffer[ID_FN_MAXLEN];
  } else {
    /* set defaults; actual parameters will be extracted from the drivecode */
    session.dir_track    = 255;
    session.fn_maxlength = detected_loader != FL_KRILL_R58PRE ? CBM_NAME_LENGTH : 2;
  }

  set_atn_irq(0);

  if (load_drivecode(&session))
    goto exit;

  /* r164 uses DATA as data line for the drivecode, but ATN for requests */
  if (detected_loader == FL_KRILL_R164)
    fast_get_byte = krill_get_byte_clk_atn;

  session.file_count = 0;
  dir_changed = 1; /* force directory update */

  if (detected_loader == FL_KRILL_R159 || detected_loader >= FL_KRILL_R184) {
    req_line = IEC_BIT_DATA;
  } else {
    req_line = IEC_BIT_ATN;
  }

  while (!IEC_ATN) {
    set_clock(detected_loader == FL_KRILL_R164);
    set_data(detected_loader != FL_KRILL_R164);

    free_multiple_buffers(FMB_USER_CLEAN);

    /* wait for host request while checking for abort and diskchange */
    while (!(iec_bus_read() & req_line))
      if (check_keys()) /* exit loop on long key press */
        goto exit;

    delay_us(10);
    /* abort if both ATN and DATA were released */
    if ((iec_bus_read() & (IEC_BIT_DATA|IEC_BIT_ATN)) == (IEC_BIT_DATA|IEC_BIT_ATN))
      goto exit;

    if (detected_loader >= FL_KRILL_R184) {
      set_clock(1);
      delay_us(2);
    }

    /* versions < r186 also have custom code upload, but we don't support it */
    if (detected_loader < FL_KRILL_R186 || IEC_CLOCK) {
      fn_len = read_filename(&session);

      if (fn_len == -1) /* timed out */
        goto exit;

      if (fn_len <= CBM_NAME_LENGTH) {
        if (send_file(&session))
          break;
      } else {
        /* More than CBM_NAME_LENGTH (16) bytes received. Can only      */
        /* happen with r192 and indicates a custom drivecode upload.    */
        if (cc_fallback(&session))
          break;

        /* The host might have already signaled the next request while  */
        /* cc_fackback() waited for the faked drive memory restore to   */
        /* time out, so skip the wait for the request line to be set.   */
        continue;
      }
    } else {
      /* custom drivecode handler for r186 (only Scramble Infinity 1.2) */
      if (custom_code_handler(&session))
        break;
    }

    /* Wait for the request line to be set */
    while (iec_bus_read() & req_line)
      if (check_keys()) /* exit loop on long key press */
        goto exit;
  }

exit:
  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  /* loader no longer active past this point */
  detected_loader = FL_NONE;

  return true;
}
