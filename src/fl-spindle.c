/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Spindle support:
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


   fl-spindle.c: Handling of Spindle loader

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


#define INIT_TRACK      18
#define INIT_SECTOR     17 // sector with the disk id and the initial command
#define FLIP_SECTOR     5  // 3.x only: sector with next side id and retry unit
#define ASYNC_SECTOR    6  // 3.x only: sector with t/s table for async jobs
#define MAX_SECTORS     21

#define CMD_LEN         3
#define PP_LEN          (0x60-3) // 3.x only: maximum total length of pp units
#define SIDE_ID_LEN     3

/* 2.x command flags */
#define CMD2_COMMAND    (1<<7)
#define CMD2_EOF1       (1<<6)
#define CMD2_NEXTTRACK  (1<<5)
/* "reset" only if no sector flags are set ("special command") */
#define CMD2_RESET      (1<<5)

/* 3.x command flags */
#define CMD3_NEWJOB     (1<<7)
#define CMD3_NEXTTRACK  (1<<6)
#define CMD3_ONDEMAND   (1<<5)

/* 3.x sector flags */
#define FLAG_FULLSECT   (1<<7)
#define FLAG_CONTREC    (1<<6)

typedef struct {
  buffer_t *buf;
  uint8_t  track;
  uint8_t  cmd[CMD_LEN];
  uint8_t  next_cmd[CMD_LEN];
  uint8_t  next_id[SIDE_ID_LEN];
  uint8_t  pp_units[PP_LEN];
  uint8_t  bdel;    // extra delay between blocks/units
  uint16_t job_crc; // used for file quirks (block delays)
  int init_done:1;  // the init sector of the initial disk has been read
} session_t;

/* block delays as hacks to make specific releases work */
static const PROGMEM file_quirks_t file_quirks[] = {
  { 0xebd1,  40 }, // mojo / 26th job on side 4 (CR at 0x19/0x06)

  { 0, 0 } // end marker
};

static uint8_t get_block_delay(uint16_t crc) {
  const file_quirks_t *fq;

  fq = get_file_quirks(file_quirks, crc);

  return fq != NULL ? pgm_read_byte(&fq->block_delay) : 0;
}

/* Calculate the hash of the init sector in "buf" to find   */
/* the exakt 2.x version of the loader. They share the same */
/* M-E code, so it can't be used to differentiate between   */
/* them. The side id, next side id and the initial command  */
/* might differ between productions, so they are excluded.  */
static fastloaderid_t detect_v2_dotversion(uint8_t *buf) {
  uint8_t  i;
  uint16_t crc;

  crc = 0xffff;

  /* exclude sideid, nextsideid and initial command from hash */
  for (i = 0; i < 0xf7; i++)
    crc = crc16_update(crc, buf[i]);

  switch (crc) {
    case 0x889e:
      return FL_SPINDLE_21;
    case 0xd126:
      return FL_SPINDLE_22;
    case 0x7ee2:
      return FL_SPINDLE_23;
    default:
      return FL_NONE;
  }
}

/* Construct a command to load the specified sector */
static inline void fake_command(uint8_t cmd[], uint8_t sector) {
  /* don't use memset(); the compiler might be able to optimize this */
  cmd[0] = 0;
  cmd[1] = 0;
  cmd[2] = 0;
  cmd[(sector+3) >> 3] |= 0x80 >> ((sector+3)&7);
}

/* Find the next requested sector in the command,      */
/* starting with s. Returns MAX_SECTORS if none found. */
static uint8_t next_sector(uint8_t cmd[], uint8_t s) {
  while (s < MAX_SECTORS) {
    if (cmd[(s+3) >> 3] & (0x80 >> ((s+3)&7)))
      break;
    s++;
  }

  return s;
}

/* 2.x bit shuffle table 57460213 */
static const PROGMEM uint8_t encoding_v2[] = {
  1<<3, 1<<1, 1<<2, 1<<0, 1<<6, 1<<4, 1<<7, 1<<5
};

/* 2.x only: send current block and checksum, update next_cmd if applicable */
static uint8_t send_block(session_t *s) {
  uint16_t i;
  uint8_t  b, cs;

  while (true) {
    set_clock(detected_loader == FL_SPINDLE_21);
    set_data(detected_loader != FL_SPINDLE_21);

    while (IEC_ATN); // This can stall for a long time, so no timeout
    set_busy_led(1);

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      for (i = 0, cs = 0; i <= 0x100; i++) {
        if (i < 0x100) {
          b = s->buf->data[i];
          cs ^= b;
        } else { // checksum
          b = cs;
        }

        if (clocked_write_byte(~b, encoding_v2, 1000))
          return 1; // timeout
      }
    }

    /* clocked_write_byte() exits with the last bitpair not yet ackowledged */
    while (!IEC_ATN);

    set_clock(1);
    set_data(1);

    /* wait for host acknowledge */
    start_timeout(10);
    while (!(iec_bus_read() & (IEC_BIT_CLOCK|IEC_BIT_DATA)));

    if (!has_timed_out()) // host ack received
      break;

    /* no host ack, send again */
  }

  if (cs & 0x80) { // next command
    s->next_cmd[0] = cs;
    s->next_cmd[1] = s->buf->data[0];
    s->next_cmd[2] = s->buf->data[0] ^ s->buf->data[1];
  }

  return 0;
}

static void load_spindle_v2(session_t *s) {
  uint8_t i;
  uint8_t sector;

  while (true) {
    sector = 0;

    while (true) {
      sector = next_sector(s->cmd, sector);
      if (sector == MAX_SECTORS)
        break; // no more sectors for this command

      dir_changed = 0;

      read_sector(s->buf, current_part, s->track, sector);
      if (current_error != ERROR_OK)
        return;

      /* 2.1 uses CLK for EOF1 and DATA for EOF2, for 2.2/3.x */
      /* the lines are swapped. We just check if either line  */
      /* is set and then set both to signal EOF2.             */
      if ((iec_bus_read() & (IEC_BIT_CLOCK|IEC_BIT_DATA)) != (IEC_BIT_CLOCK|IEC_BIT_DATA)) {
        /* we're in EOF1 */
        set_clock(0);
        set_data(0);

        while (IEC_ATN); // can stall for a long time, so no timeout
        while (!IEC_ATN) {
          /* "Expand" prompts for disk change before acknowledging EOF2. */
          /* Call check_keys() here so the LED confirmation blink works. */
          if (check_keys())
            return;
        }

        set_clock(1);
        set_data(1);

        /* This reset detection unfortunately doesn't work with 2.1, because  */
        /* the host always releases both CLK and DATA after EOF2 acknowledge. */
        /* So with this version a reset during EOF1 might result in a hang in */
        /* send_block() later, which then requires a second reset to solve.   */
        if (detected_loader != FL_NONE && detected_loader != FL_SPINDLE_21) {
          delay_us(2);
          if (IEC_DATA)
            return; // host reset
        }
      }

      if (s->track == INIT_TRACK) {
        if (sector != INIT_SECTOR)
          return; // unknown sector

        if (s->init_done) { // not the first init sector; check side id
          if (memcmp(s->next_id, s->buf->data + 0xf7, SIDE_ID_LEN)) {
            /* wrong disk; wait for disk change, host reset or user abort */
            while (!dir_changed) {
              if (!IEC_ATN || check_keys())
                return;
            }

            /* load the init sector of the new disk, then check again */
            continue;
          }

          /* 2.1: We don't special-case the EOF1/2 handling for the flip case */
          /* and immediately wait for the host's second ATN pulse, like the   */
          /* original loader does, but instead force another EOF1/2 for the   */
          /* first command of the "new" disk (the only known user is "Aerial  */
          /* Core", which doesn't even use it for flip, but for looping).     */

          /* force EOF1 to make the host's flip call return */
          s->buf->data[0xfd] |= CMD2_EOF1;
        } else { // initial disk
          if (detected_loader == FL_NONE) { // check exact loader version
            detected_loader = detect_v2_dotversion(s->buf->data);
            if (detected_loader == FL_NONE)
              return; // unknown version
          }

          /* all 2.x versions but 2.1 start with EOF1 on the initial disk */
          if (detected_loader != FL_SPINDLE_21)
            s->buf->data[0xfd] |= CMD2_EOF1;
          s->init_done = 1;
        }

        /* copy initial command and next side id */
        for (i = 0; i < 3; i++) {
          s->next_cmd[i] = s->buf->data[0xfd+i];

          if (detected_loader < FL_SPINDLE_23) {
            s->next_id[i] = s->buf->data[0xfa+i];
          } else {
            /* 2.3 has the next side id stored backwards */
            s->next_id[i] = s->buf->data[0xfc-i];
          }
        }

        s->track = 1; // start command on track 1

        break; // don't send anything for the init sector
      } // end of flip handling

      if (send_block(s))
        return; // timeout

      sector++; // check next sector
    }

    set_busy_led(0);

    /* If no sector flags are set, it's a "special command" (reset or flip). */
    if (!(s->next_cmd[0]&0x1f || s->next_cmd[1] || s->next_cmd[2])) {
      if (s->next_cmd[0] & CMD2_RESET) { // reset
        /* In the 2.3 variant which supports custom drivecode, bit 5 */
        /* does not trigger a reset, but loading the drivecode.      */
        /* As we don't support that, we can just as well still exit. */
        set_clock(0);
        set_data(0);

        wait_atn_low(1000); // wait for acknowledge
        /* the original loader now waits for ATN release, but we don't */

        return;
      }

      /* flip; fake a command to load the init sector and force EOF1 */
      s->track = INIT_TRACK;
      fake_command(s->next_cmd, INIT_SECTOR);
      s->next_cmd[0] |= CMD2_EOF1;
    }

    /* change track and/or enter EOF1 as indicated by the new command flags */
    if (s->next_cmd[0] & CMD2_NEXTTRACK) // advance track
      while (++s->track == INIT_TRACK);  // but skip track 18

    if (s->next_cmd[0] & CMD2_EOF1) {
      if (detected_loader != FL_SPINDLE_21) {
        set_data(0);
        if (wait_atn_low(2000))
          return; // timeout
      } else { // 2.1
        set_clock(0);
      }
    }

    /* make next_cmd the active command */
    memcpy(s->cmd, s->next_cmd, sizeof(s->cmd));
  }
}

/* 3.x only: Receive a 7-bit (!) job number for an async transfer.  */
/* ATN is used as clock, CLK as data line. Bits are sent MSB first. */
static uint8_t receive_job_no(void) {
  uint8_t i, b;

  b = 0;

  ATOMIC_BLOCK( ATOMIC_FORCEON ) {
    for (i = 7; i != 0; i--) {
      set_data(1);

      while (!IEC_ATN);
      delay_us(2);

      b = b << 1 | !IEC_CLOCK;
      set_data(0);

      if (wait_atn_low(1000))
        return 0x80; // timeout
    }
  }

  return b;
}

/* 3.x only: Process a sector with a continuation record. The command   */
/* is copied to "next_cmd", postponed_units to "pp_units". If there are */
/* no postponed units, a dummy unit of length 3 is added. Returns the   */
/* index of the length byte of the the first non-postponed unit in the  */
/* block (this length byte might be zero if there is no regular unit).  */
static uint8_t copy_cr(session_t *s) {
  uint8_t i;
  uint8_t pos;  // current position in sector buffer
  uint8_t dest; // current position in pp_units

  pos = 0xff-2;
  for (i = 0; i < 3; i++)
    s->next_cmd[i] = s->buf->data[pos + i];
  pos--;

  dest = PP_LEN-1;

  /* postponed units */
  while (true) {
    i = s->buf->data[pos]; // length of next unit (excluding length byte!)
    if (i == 0 || i > 4 || i >= dest) // end of postponed units
      break;

    i++; // include length byte in copy

    while (i--)
      s->pp_units[dest--] = s->buf->data[pos--];
  }

  if (dest == PP_LEN-1) {
    /* no postponed units, add dummy unit (content doesn't matter) */
    s->pp_units[dest] = 3;
    dest -= 4;
  }

  s->pp_units[dest] = 0; // end marker

  /* Start position of the first non-postponed unit in current buffer. */
  /* If there is a zero byte at this position there are no such units. */
  return pos;
}

/* 3.x bit shuffle table 76540213 */
static const PROGMEM uint8_t encoding_v3[] = {
  1<<3, 1<<1, 1<<2, 1<<0, 1<<4, 1<<5, 1<<6, 1<<7
};

/* 3.x only: Send either all regular units (from sector buffer) */
/* or the postponed units (from pp_units), as indicated by the  */
/* pp parameter. Returns early if an async command is detected. */
/* pos: offset of first unit's length byte or 0 if full buffer. */
/* pp:  send regular (false) or postponed (true) units.         */
static uint8_t send_units(session_t *s, uint8_t pos, bool pp) {
  uint8_t b, unit_len;
  uint8_t *data;
  bool chain;

  data = !pp ? s->buf->data : s->pp_units; // pointer to source data
  unit_len = pos != 0 ? data[pos] : 0xff;

  while (unit_len > 0) {
    /* check for underflow; pos == 0 is special case full sector */
    if (pos > 0 && unit_len >= pos)
      return 1; // invalid length

    pos -= unit_len; // set pos to the first byte of the unit

    if (pp) {
      while (IEC_CLOCK); // wait for host
    }

    set_data(1);
    while (!IEC_ATN);
    delay_us(2);

    if (IEC_DATA)
      return 0; // async command or reset

    set_busy_led(1);

    /* If it's a "chain head" (pp unit of length 2) or the   */
    /* host released CLK, we have to set CLK for the status. */
    chain = (pp && unit_len == 2) || IEC_CLOCK;

    ATOMIC_BLOCK( ATOMIC_FORCEON ) {
      if (clocked_write_byte(unit_len, NULL, 1000))
        return 1; // timeout

      do {
        unit_len--;
        b = data[pos + unit_len];
        s->job_crc = crc16_update(s->job_crc, b);
        if (clocked_write_byte(b ^ 0x7f, encoding_v3, 1000))
          return 1; // timeout
      } while (unit_len > 0);

      /* clocked_write_byte() exits with the last bitpair not yet ackowledged */
      while (!IEC_ATN);

      unit_len = pos > 1 ? data[--pos] : 0;

      /* indicate status by setting CLK and DATA accordingly */
      set_clock(!chain);
      /* Set DATA ("more"), unless this was the last unit of this job */
      set_data(unit_len == 0 && pp && (s->next_cmd[0] & CMD3_NEWJOB));
    }

    if (wait_atn_low(1000))
      return 1; // timeout

    set_data(0);
    set_clock(1);

    if (s->bdel > 0)
      delay_ms(s->bdel);
  }

  return 0;
}

static void load_spindle_v3(session_t *s) {
  uint8_t job;
  uint8_t sector;
  uint8_t unit_start;

  set_data(0);

  if (wait_atn_low(1000))
    return; // timeout

  while (true) {
cmd_loop:
    sector = 0;

    while (true) {
      sector = next_sector(s->cmd, sector);
      if (sector == MAX_SECTORS)
        break;

      read_sector(s->buf, current_part, s->track, sector);
      if (current_error != ERROR_OK)
        return;

      if (s->track == INIT_TRACK) {
        s->job_crc = 0xffff;

        switch (s->buf->data[0] & 0x1f) {
          case INIT_SECTOR:
            if (s->init_done) { // not the initial disk; potential disk flip
              if (memcmp(s->next_id, s->buf->data+0xf9, SIDE_ID_LEN)) {
                /* disk id doesn't match, wait for disk change */
                dir_changed = 0;
                while (!dir_changed) {
                  if (IEC_ATN || check_keys())
                    return; // exit on host reset or long key press
                }

                continue; // load the init sector of the new disk
              }

              /* disk id matches; set "new job" flag for the first command */
              s->buf->data[0xff-2] |= CMD3_NEWJOB;
            } else { // this is the inital disk
              s->init_done = 1;
              /* initialize the next side id with the current side id, */
              /* so a potential async command for the first job works. */
              memcpy(s->next_id, s->buf->data+0xf9, SIDE_ID_LEN);
            }

            s->track = 1;
            break;

          case FLIP_SECTOR:
            /* Even though we can load sectors without doing any transfer,  */
            /* we have to send the retry sector unit at least once, so the  */
            /* host returns from its loadercall if this isn't really a disk */
            /* flip but the end of the chain. Waiting for a disk change is  */
            /* then done in the init sector handler, above.                 */

            /* copy next side id, fake retry sector flags/unit  */
            /* and the command which will load the init sector. */
            memcpy(s->next_id, s->buf->data+1, SIDE_ID_LEN);
            s->buf->data[0x00] = s->buf->data[0x0e];
            memcpy(s->buf->data+(0xff-9), s->buf->data+4, 10);
            break;

          case ASYNC_SECTOR:
            /* Async job request. Read the job number from the host and   */
            /* fake a command to read this job's CR sector. For the first */
            /* job, this is the init sector, which will be handled by the */
            /* respective case above. Otherwise the track is != 18 and    */
            /* will run into the else branch, below. The job number is    */
            /* 7 bits, but as of 3.1 the t/s table only holds 64 entries. */

            if (wait_atn_low(1000)) // make sure the host has set ATN
              return; // timeout

            job = receive_job_no();
            if (job & 0x80)
              return; // receive timeout

            /* the stored track number is a half-track, so divide by 2 */
            s->track = s->buf->data[0x80-job] >> 1;
            fake_command(s->cmd, s->buf->data[0x40-job]);
            s->cmd[0] |= CMD3_ONDEMAND;

            goto cmd_loop; // no units to send

          default: // unknown sector
            return;
        }
      } else if (s->cmd[0] & CMD3_ONDEMAND) {
        /* First sector of an async job != the first job (if it was  */
        /* the first job, it would have already been handled by the  */
        /* if() branch above, as its CR sector is the init sector).  */
        s->buf->data[0xff-3]  = 0; // ignore all units from this sector
        s->buf->data[0xff-2] &= ~CMD3_NEWJOB; // clear the "new job" flag
        s->job_crc = 0xffff;
      }

      /* check sector flags */
      switch (s->buf->data[0] & (FLAG_FULLSECT|FLAG_CONTREC)) {
        case FLAG_FULLSECT: // 255 bytes payload, no unit length byte
          unit_start = 0;
          break;
        case FLAG_CONTREC: // continuation record present
          /* Copy next command and pp units. copy_cr() returns the */
          /* position of the length byte of the first regular unit */
          /* in this sector (length might be 0 if there is none).  */
          unit_start = copy_cr(s);
          break;
        default: // Not full and no CR. First unit length at offset 0xff.
          unit_start = 0xff;
          break;
      }

      /* Send regular units from the current sector. send_units() */
      /* returns early if it detects an async command or a reset. */
      if (send_units(s, unit_start, false))
        return; // timeout or error

      if (IEC_DATA) { // async command or reset
        if (IEC_CLOCK)
          return; // reset

        /* abort the current transfer and read the async sector */
        set_data(0);

        s->track = INIT_TRACK;
        fake_command(s->cmd, ASYNC_SECTOR);

        goto cmd_loop;
      }

      sector++; // check next sector
    }

    /* Done with command, send postponed units. If there are none, copy_cr() */
    /* already added a dummy unit, so no special handling needed here.       */
    if (send_units(s, PP_LEN-1, true))
      return; // timeout or error

    set_busy_led(0);

    /* check command flags */
    switch (s->next_cmd[0] & (CMD3_NEXTTRACK|CMD3_ONDEMAND)) {
      case 0:
        break;
      case CMD3_NEXTTRACK: // advance track but skip track 18
        while (++s->track == INIT_TRACK);
        break;
      case CMD3_ONDEMAND: // handle ondemand as indicated by sector flags
        s->track = INIT_TRACK;
        break;
      default:
        return; // unknown command flags
    }

    /* check if extra block delays are needed for the next bundle */
    if (s->next_cmd[0] & CMD3_NEWJOB) {
      s->bdel = get_block_delay(s->job_crc);
      s->job_crc = 0xffff; // reset crc for next bundle
    }

    /* make next_cmd the active command */
    memcpy(s->cmd, s->next_cmd, sizeof(s->cmd));
  }
}

bool load_spindle(UNUSED_PARAMETER) {
  session_t session;

  if (command_length != 0x17 && command_length != 0x29)
    return false;

  switch (command_crc(5, 2)) {
    case 0x6027: // 3.x
      detected_loader = FL_SPINDLE_3;
      break;
    case 0xe438: // 2.x (exact version detected later from init sector hash)
      break;
    case 0x2c76: // ES1RA & Amanita 80% (2.3 with custom drivecode support)
      detected_loader = FL_SPINDLE_23;
      break;
    default:
      return false;
  }

  memset(&session, 0, sizeof(session));

  session.buf = alloc_system_buffer();
  if (!session.buf)
    return true;

  set_atn_irq(0);

  /* fake command to load the init sector */
  session.track = INIT_TRACK;
  fake_command(session.cmd, INIT_SECTOR);

  if (detected_loader == FL_SPINDLE_3) {
    load_spindle_v3(&session);
  } else {
    load_spindle_v2(&session);
  }

  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  /* Don't wait for the main loop to clean up the buffer, as if */
  /* the loader wasn't detected, other handlers might need it.  */
  free_buffer(session.buf);

  if (detected_loader == FL_NONE)
    return false;

  /* loader no longer active past this point */
  detected_loader = FL_NONE;

  return true;
}

