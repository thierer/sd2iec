/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Sparkle support:
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


   fl-sparkle.c: Handling of Sparkle loader

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


#define INIT_TRACK    18
#define BAM_SECTOR    0
#define DIR_START     17

#define SAVER_BUNDLE  0x7e
#define SAVE_FILE     0x7f
/* internal dummy bundle number to indicate sequential loading */
#define SEQ_BUNDLE    0x80

/* skew is a configuration parameter, but it's only used */
/* in the two "Median" releases with a value of 2.       */
#define SKEW          2

/* offset of the bundle count offset in the BAM sector (1.x only) */
#define BNDCNT_OFFS   0xfe

/* disc-specific parameters in the BAM sector */
#define PRODID_LEN 3
enum {
  DISKID,
  NEXTID,
  SAVER,
  IL0R,
  IL1R,
  IL2R,
  IL3R,
  PRODID, // offset of PRODID[2] (prod id is stored reversed)
  NUM_PARAMS
};

/* Offsets of parameters into the BAM sector for various loader revisions.   */
/* Currently addressed by get_param() using "detected_loader-FL_SPARKLE_10", */
/* so every defined FL_SPARKLE_xx version needs an entry in this table!      */
/* 0x00 indicates paramter not present in this revision. get_param() returns */
/* 0 in this case, which is a safe default for all parameters.               */
static const PROGMEM uint8_t param_layouts[][NUM_PARAMS] = {
  /* DISKID NEXTID SAVER  IL0R   IL1R   IL2R   IL3R   PRODID */
  {  0xff,  0xfd,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00  }, // FL_SPARKLE_10
  {  0xff,  0xfd,  0x00,  0xf8,  0xfa,  0xfb,  0xfc,  0x00  }, // FL_SPARKLE_15
  {  0xff,  0xfe,  0xf4,  0xf9,  0xfb,  0xfc,  0xfd,  0xf1  }, // FL_SPARKLE_20
  {  0xff,  0xfb,  0xf9,  0xfa,  0xfc,  0xfd,  0xfe,  0xf6  }, // FL_SPARKLE_21
};

/* ids of productions that need special treatment:    */
/* - Median / Median final: using "sector skew" == 2. */
/* - Propaganda 30: sets sector = 0 on track change.  */
static const PROGMEM uint8_t pid_median[]       = { 0xbd, 0xe2, 0x0a };
static const PROGMEM uint8_t pid_median_final[] = { 0xbd, 0x8c, 0xd3 };
static const PROGMEM uint8_t pid_propaganda30[] = { 0x92, 0xd2, 0x6f };

typedef struct session_s {
  buffer_t *dir_buf;
  uint8_t  bundle_len;    // number of blocks in next bundle
  uint8_t  track, sector;

  /* state for the current track */
  uint8_t  current_il;    // interleave
  uint8_t  num_sectors;   // total number of sectors
  uint8_t  used[3];       // bitmap of processed sectors
  uint8_t  remaining;     // number of remaining (not yet processed) sectors

  /* configuration parameters copied from the BAM block (if applicable) */
  uint8_t  interleave[4]; // interleave values for the 4 speed zones
  uint8_t  prod_id[3];    // production-specific id; used for disk flip check
  uint8_t  next_id;       // expected disk id for next flip command
  int      has_saver:1;   // loader uses saver

  /* loader state */
  int current_dir:1;  // index (0/1) of directory sector loaded into dir_buf
  int save_active:1;  // saver bundle was requested, save active

  /* loader configuration quirks */
  int has_skew:1;     // loader version with sector skew (Median, Median final)
  int has_nsreset:1;  // start at sector 0 after track change (Propaganda 30)
  int bundle_inv:1;   // bundle number is sent inverted (Memento Mori, reMETA)

  /* function pointers */
  uint8_t (*decode_byte)(uint8_t);
  void (*decode_block)(struct session_s *, uint8_t *);
} session_t;

typedef struct {
  uint8_t track;
  uint8_t sector;
  uint8_t sctr;
  uint8_t bptr;
} dir_entry_t;

static uint8_t decode_low(uint8_t v) {
  switch (v & 0x09) {
    case 0x00:
    case 0x09:
      v ^= 0x0f;
      break;
    default:
      v ^= 0x06;
      break;
  }

  return v;
}

static uint8_t decode_high(uint8_t v) {
  switch (v & 0x90) {
    case 0x00:
    case 0x90:
      v ^= 0xf0;
      break;
    default:
      v ^= 0x60;
      break;
  }

  return v;
}

/* 1.x (no encoding) */
static uint8_t identity(uint8_t v) {
  return v;
}

/* 2.0 encoding */
static uint8_t decode_byte_20(uint8_t v) {
  return decode_high(decode_low(v));
}

/* 2.1 encoding */
static uint8_t decode_byte_21(uint8_t v) {
  return decode_low(v) ^ 0x70;
}

/* only known production using this is "Padawan's Awakening" */
static uint8_t decode_byte_21ff(uint8_t v) {
  return decode_low(v) ^ 0xf0;
}

/* 1.x plain layout */
static void decode_block(session_t *s, uint8_t *data) {
  uint8_t i;

  for (i = 0;; i++) {
    data[i] = s->decode_byte(data[i]);
    if (i == 0xff)
      break;
  }
}

/* 2.x partially reversed layout */
static void decode_block_rev(session_t *s, uint8_t *data) {
  uint8_t i, tmp;

  /* decode block; swap not necessary for offsets 0x00 and 0x80, but simpler */
  for (i = 0; i <= 0x80; i++) {
    tmp = s->decode_byte(data[i]);
    data[i] = s->decode_byte(data[(uint8_t)-i]);
    data[(uint8_t)-i] = tmp;
  }
}

/* Make sure the dir sector indicated by dir_index is loaded    */
/* and decoded. Also sets up the decode function, if necessary. */
static uint8_t load_dir(session_t *s, uint8_t dir_index) {
  if (dir_index > 1) // only 0 or 1 are valid
    return 1;
  if (s->current_dir == dir_index)
    return 0;

  read_sector(s->dir_buf, current_part, INIT_TRACK, DIR_START + dir_index);
  if (current_error != ERROR_OK)
    return 1;

  s->current_dir = dir_index;

  if (s->decode_byte == identity)
    return 0; // 1.x; no decoding necessary

  if (s->decode_block == NULL) { //
    /* Block decode function not yet initialized. Check the sector  */
    /* layout and set up the correct decode function. We expect the */
    /* sector of the first entry to be 0 with a sector count of 21. */
    if (s->decode_byte(s->dir_buf->data[1]) == 0 && s->decode_byte(s->dir_buf->data[2]) == 21) {
      /* Probably the old, plain layout still used by some    */
      /* 2.0 pre-release versions ("Memento Mori", "reMETA"). */
      s->decode_block = decode_block;
      s->bundle_inv = 1; // these versions also send the bundle number inverted
    } else {
      /* otherwise assume the new layout */
      s->decode_block = decode_block_rev;
    }
  }

  s->decode_block(s, s->dir_buf->data);

  return 0;
}

/* Advance sector by the given number of sectors */
static void advance_sector(session_t *s, uint8_t ds) {
  s->sector += ds;

  if (s->sector >= s->num_sectors) { // overflow
    s->sector -= s->num_sectors;

    if (s->track < 18 && s->sector > 0) // special case tracks 1 to 17
      s->sector--;
  }
}

/* Find the next useable sector on the current track.  */
/* Skips sectors which were already used, except if we */
/* reached the end of this track (remaining == 0). If  */
/* so, track and sector are updated in send_bundle().  */
static uint8_t iterate_sector(session_t *s) {
  s->used[s->sector >> 3] |= 1 << (s->sector & 7);

  /* don't update sector if no remaining sectors on track and skew is used */
  if (--s->remaining > 0 || !s->has_skew)
    advance_sector(s, s->current_il);

  if (s->remaining > 0) { // skip over used sectors
    while ((s->used[s->sector >> 3] & (1 << (s->sector & 7))))
      advance_sector(s, 1);
  }

  return s->remaining;
}

/* initialize some track-specific parameters */
static void track_changed(session_t *s) {
  if (s->track < 18) {
    s->num_sectors = 21;
    s->current_il  = s->interleave[0];
  } else if (s->track < 25) {
    s->num_sectors = 19;
    s->current_il  = s->interleave[1];
  } else if (s->track < 31) {
    s->num_sectors = 18;
    s->current_il  = s->interleave[2];
  } else { // track >= 31
    s->num_sectors = 17;
    s->current_il  = s->interleave[3];
  }

  s->remaining = s->num_sectors;
  memset(s->used, 0, sizeof(s->used));
}

/* Helper to a get parameter value from the BAM sector (expected in */
/* dir_buf). Returns 0 if parameter is not present (offset == 0).   */
static uint8_t get_param(const session_t *s, uint8_t pm) {
  uint8_t i;

  /* get offset from parameter layout table for detected loader version */
  i = pgm_read_byte(&param_layouts[detected_loader-FL_SPARKLE_10][pm]);

  return i != 0 ? s->dir_buf->data[i] : 0;
}

/* Compare prod ids. Returns true if equal. */
static bool pidcmp(const session_t *s, const uint8_t *pid) {
  return !memcmp_P(s->prod_id, pid, PRODID_LEN);
}

/* Load the BAM sector and initialize disk-specific parameters. */
/* On first invocation (detected_loader is still FL_NONE), any  */
/* disk is accepted and the loader version is determined from   */
/* the parameter layout. Otherwise (not the initial disk) first */
/* wait for a disk with the expected prod and side ids.         */
static uint8_t init_disk(session_t *s) {
  uint8_t i;

  while (true) {
    read_sector(s->dir_buf, current_part, INIT_TRACK, BAM_SECTOR);
    if (current_error != ERROR_OK)
      return 1;

    dir_changed = 0;

    /* If no loader version is set yet, this is the BAM sector   */
    /* of the initial disk. In this case both the loader version */
    /* (controls the parameter layout) and the byte encoding are */
    /* are deduced from suitable byte values in this sector.     */
    if (detected_loader == FL_NONE) {
      switch (s->dir_buf->data[0xf9] & 0xc0) {
        case 0x00: // <= 2.0
          /* for 1.x [0xf8] == -[0xf9] (IL0R / IL0) (or == 0 for 1.0) */
          i = s->dir_buf->data[0xf8];

          if (i == (uint8_t)-s->dir_buf->data[0xf9]) {
            /* 1.x layout, no encoding */
            s->decode_byte = identity;

            if (i != 0) { // valid interleave?
              detected_loader = FL_SPARKLE_15;
            } else {
              /* 1.0 has no custom interleave, use default values */
              detected_loader = FL_SPARKLE_10;
              s->interleave[0] = 4;
              s->interleave[1] = 3;
              s->interleave[2] = 3;
              s->interleave[3] = 3;
            }
          } else { // 2.0 layout
            detected_loader = FL_SPARKLE_20;
            if ((s->dir_buf->data[0xfe] & 0xc0) == 0xc0) { // 2.0 encoding
              /* we don't expect a side id >= 0x10 */
              s->decode_byte = decode_byte_20;
            } else { // 2.1FF encoding (Padawan's Awakening)
              s->decode_byte = decode_byte_21ff;
            }
          }
          break;
        case 0x80: // 2.0 layout, 2.1 encoding
          detected_loader = FL_SPARKLE_20;
          s->decode_byte  = decode_byte_21;
          break;
        case 0x40: // 2.1 layout, 2.1 encoding
          detected_loader = FL_SPARKLE_21;
          s->decode_byte  = decode_byte_21;
          break;
        default:
          return 1;
      }

      i = pgm_read_byte(&param_layouts[detected_loader-FL_SPARKLE_10][PRODID]);
      if (i != 0) { // loader version has prod id
        /* no need to decode the prod id, it's only tested for equality */
        memcpy(s->prod_id, &s->dir_buf->data[i], PRODID_LEN);

        /* check for the two known productions which use sector skew */
        if (pidcmp(s, pid_median) || pidcmp(s, pid_median_final)) {
          s->has_skew = 1;
        } else {
          s->has_skew = 0;

          /* Propaganda 30 resets the sector to 0 on every track change */
          s->has_nsreset = pidcmp(s, pid_propaganda30);
        }
      }

      break; // this is the initial disk; skip disk and prod id checks
    }

    /* check disk and production ids */
    if (s->next_id == s->decode_byte(get_param(s, DISKID))) {
      i = pgm_read_byte(&param_layouts[detected_loader-FL_SPARKLE_10][PRODID]);

      if (i == 0 || !memcmp(s->prod_id, &s->dir_buf->data[i], PRODID_LEN))
        break; // id matched (or loader version without prod id)
    }

    /* wrong disk; wait for disk change, host reset or user abort */
    while (!dir_changed) {
      if (IEC_ATN || check_keys()) // exit on host reset or long key press
        return 1;
    }

    /* disk changed, check again */
  }

  /* copy interleave values for disk (unless it's 1.0 with fixed interleave) */
  if (detected_loader != FL_SPARKLE_10) {
    /* interleave is stored as 2s complement, convert to positive value */
    for (i = IL0R; i <= IL3R; i++)
      s->interleave[i-IL0R] = ~s->decode_byte(get_param(s, i))+1;
  }

  s->next_id   = s->decode_byte(get_param(s, NEXTID));
  s->has_saver = s->decode_byte(get_param(s, SAVER)) == 2;

  if (detected_loader >= FL_SPARKLE_20) {
    /* load the first dir sector */
    s->current_dir = 1; // force update
    if (load_dir(s, 0))
      return 1;
  } else {
    /* 1.x versions have no directory and always start at 1/0 */
    s->track  = 1;
    s->sector = 0;
    track_changed(s); // update track-specific parameters
  }

  return 0;
}

/* Called while the highscore saver is active. Receives one block of */
/* payload data, writes it to disk and iterates to the next sector.  */
static uint8_t handle_save(session_t *s) {
  buffer_t *buf;
  uint8_t  i;

  if (s->remaining == 0) // no blocks left on track; should not happen
    return 1;

  buf = alloc_buffer();
  if (!buf)
    return 1;

  mark_buffer_dirty(buf);

  set_data(0);

  /* payload bytes are sent in reverse order, starting at offset 0 */
  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    for (i = 0;; i--) {
      buf->data[i] = clocked_read_byte(IEC_BIT_CLOCK, IEC_BIT_ATN, 90);
      if (has_timed_out())
        return 1;
      if (i == 0x01)
        break;
    }

    set_data(1);
  }

  write_sector(buf, current_part, s->track, s->sector);

  free_buffer(buf);

  /* The saver only operates on one (the last) track, */
  /* so no need to deal with track changes here.      */
  iterate_sector(s);

  return current_error != ERROR_OK;
}

/* Find directory entry for bundle and iterate to the first sector. */
/* Returns pointer to the entry or NULL in case of an error.        */
static dir_entry_t *find_dir_entry(session_t *s, uint8_t bundle) {
  dir_entry_t *entry;

  /* make sure the correct dir sector is loaded */
  if (load_dir(s, bundle >> 6))
    return NULL;

  entry = (dir_entry_t *)(s->dir_buf->data) + (bundle & 0x3f);
  s->track  = entry->track;
  s->sector = entry->sector;
  track_changed(s);

  /* track_changed() updated s->remaining for the new track */
  while (s->remaining > entry->sctr)
    iterate_sector(s);

  return entry;
}

static uint8_t send_bundle(session_t *s, uint8_t bundle) {
  uint8_t  i;
  buffer_t *buf;
  dir_entry_t *entry = NULL;
  bool eob = false;

  if (bundle != SEQ_BUNDLE) {
    /* random, or first, auto-loaded bundle 0 */
    if (detected_loader >= FL_SPARKLE_20) { // only 2.x have a directory
      entry = find_dir_entry(s, bundle);
      if (entry == NULL)
        return 1; // error
    }

    /* load first block to find the actual length */
    s->bundle_len = 1;
  }

  buf = alloc_buffer();
  if (!buf)
    return 1;

  do {
    read_sector(buf, current_part, s->track, s->sector);
    if (current_error != ERROR_OK)
      return 1;

    if (detected_loader != FL_SPARKLE_10) {
      if (--s->bundle_len == 0) {
        eob = true; // exit to main loop after this block

        s->bundle_len = s->decode_byte(buf->data[1]);
        buf->data[1] = 0;

        if (bundle & 0x7f) { // first block of a random bundle != 0
          buf->data[0x00] = 0;
          buf->data[0xff] = s->decode_byte(entry->bptr);
        }
      }
    } else { // 1.0; only OMG Got Balls!
      switch (--s->bundle_len) {
        case 0:
          s->bundle_len = buf->data[0xff];
          break;
        case 1:
          eob = true;
          break;
        default:
          break;
      }
    }

    if (wait_atn_low(1000))
      return 1;

    if (detected_loader != FL_SPARKLE_10) {
      set_data(1);
      while (!IEC_DATA); // wait for host ready
      delay_us(2);
      if (IEC_ATN)
        return 1; // host reset
    }

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      set_clock(0);
      set_data(1); // needed for 1.0
      while (!IEC_ATN);

      for (i = 0;; i++) {
        if (clocked_write_byte(buf->data[i], NULL, 1000))
          return 1; // timeout
        if (i == 0xff)
          break;
      }

      /* clocked_write_byte() exits with the last bitpair not yet ackowledged */
      while (!IEC_ATN);
    }

    set_clock(1);
    set_data(detected_loader != FL_SPARKLE_10);

    if (iterate_sector(s) == 0) {
      /* end of track; continue on next track but skip dir track */
      while (++s->track == INIT_TRACK)
        s->sector += 2; // could this overflow? original loader doesn't check?

      track_changed(s); // update number of sectors per track and interleave

      /* handle special cases */
      if (s->has_skew) {
        /* sector skew; only "Median" / "Median Final" */
        s->sector -= SKEW;
        if (s->track == 19)
          s->sector -= 8; // -6 minus 2 added above
        if (s->sector & 0x80) // sector < 0
          s->sector += s->num_sectors;
      } else if (s->has_nsreset) {
        /* reset sector on track change; only "Propaganda #30" */
        s->sector = 0;
      }
    }
  } while (!eob);

  free_buffer(buf);

  while (IEC_ATN); // takes a long time on 1.0 ("OMG got balls!")

  return 0;
}

bool load_sparkle(UNUSED_PARAMETER) {
  session_t session;
  uint8_t   bundle;

  datacrc = command_crc(5, 0);
  if ((command_length != 0x22 || datacrc != 0x1874) && // 2.x
      (command_length != 0x28 || datacrc != 0x36fe) && // 1.5
      (command_length != 0x23 || datacrc != 0x656f)) { // 1.0
      return false;
  }

  memset(&session, 0, sizeof(session));

  set_atn_irq(0);

  /* dir_buf is used for both directory and BAM sectors, that's why it's  */
  /* also needed for 1.x loader revisions (which don't have a directory). */
  session.dir_buf = alloc_system_buffer();
  if (session.dir_buf == NULL)
    goto exit;

  /* load BAM sector, detect loader version/parameters and load dir sector */
  if (init_disk(&session))
    goto exit;
  bundle = 0;

  set_data(0); // drive ready

  while (true) {
    if (wait_atn_low(1000))
      goto exit;

    if (detected_loader != FL_SPARKLE_10) {
      set_data(1);

      /* wait for host ready to receive */
      while (!IEC_DATA) {
        if (IEC_ATN) // exit on host reset
          goto exit;
      }

      /* 2.x only: check if the hosts requests a random bundle */
      if (detected_loader >= FL_SPARKLE_20) {
        delay_us(2);

        if (!IEC_CLOCK) {
          /* host didn't release CLK -> random load */
          set_clock(1);
          set_data(0);

          bundle = clocked_read_byte(IEC_BIT_CLOCK, IEC_BIT_ATN, 90);
          if (has_timed_out())
            goto exit;

          set_data(1);

          /* 2.0 pre-release versions used by "Memento Mori" and all */
          /* known "reMETA" issues send the bundle number inverted.  */
          if (session.bundle_inv)
            bundle = ~bundle;

          if (bundle & 0x80) { // flip or reset
            if (bundle == 0xff) // reset
              goto exit;

            /* disk flip; update next_id from "bundle" number */
            session.next_id = bundle & 0x7f;
            goto disk_flip;
          }
        }
      }
    }

    if (session.save_active) {
      if (bundle != 0) {
        /* "bundle" is really just a flag here; != 0 => more payload */
        if (handle_save(&session))
          goto exit;
      } else { // done with saver
        /* The host *must* follow up with a request for a regular bundle. */
        session.save_active = 0;
      }
    } else if (session.bundle_len == 0 && bundle == SEQ_BUNDLE) { // end of disk
      if (session.next_id & 0x80)
        goto exit; // no more disks, loader done

disk_flip:
      if (init_disk(&session))
        goto exit;

      bundle = 0; // auto-load bundle 0
    } else { // standard bundle
      if (send_bundle(&session, bundle))
        goto exit; // timeout

      /* 1.x only: disk flip (or done) if all bundles are transferred */
      if (detected_loader < FL_SPARKLE_20) {
        if (--session.dir_buf->data[BNDCNT_OFFS] == 0) {
          if (session.next_id != 0)
            goto disk_flip;

          goto exit; // no more disks, loader done
        }
      }

      /* enter save mode if the saver bundle was requested */
      if (bundle == SAVER_BUNDLE && session.has_saver) {
        /* Look up the first sector of the save file. This probably  */
        /* isn't necessary, as it should always start at the sector  */
        /* following the code bundle, so we could just iterate       */
        /* instead, but better not risk writing to the wrong blocks. */
        if (find_dir_entry(&session, SAVE_FILE) == NULL)
          goto exit; // error

        session.save_active = 1;
      }

      bundle = SEQ_BUNDLE; // default unless a command is sent
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
