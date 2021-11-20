/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2021  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Ultraboot support:
   Copyright (C) 2021  Martin Thierer <mthierer@gmail.com>

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


   fl-ultraboot.c: High level handling of Ultraboot

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
#include "iec.h"
#include "led.h"
#include "parser.h"
#include "timer.h"
#include "wrapops.h"
#include "fastloader.h"

// Number of sectors per track for a given speedzone
static const PROGMEM uint8_t sectors_per_track[] = { 17, 18, 19, 21 };
static uint8_t speedzone;

// Check if a D41 image is mounted.
static bool check_d41_image(void) {
  partition_t *p = &partition[current_part];

  if (p->fop != &d64ops || (p->imagetype & D64_TYPE_MASK) != D64_TYPE_D41) {
    set_error(ERROR_IMAGE_INVALID);
    return false;
  }

  return true;
}

static bool detect_ultraboot(void) {
  uint16_t crc;
  buffer_t *buf;
  uint8_t  i;

  crc = 0xffff;

  switch (*(uint16_t *)(command_buffer+3)) { /* M-E address */
    case 0x0205: // Ultraboot or Ultraboot Menue
      for (i = 5; i < command_length; i++) {
        // Bytes at offsets 6 and 11 are variable: track, sector
        if (i == 6 || i == 11) continue;
        crc = crc16_update(crc, command_buffer[i]);
      }

      switch (crc) {
        case 0xd75a: // Ultraboot
          fl_track  = 36;
          fl_sector = 0;
          // avoid mapping for 36/0; the correct speedzone is applied later
          speedzone = 0;
          return true;
        case 0x3e82: // Ultraboot Menue
          fl_track  = command_buffer[6];
          fl_sector = command_buffer[11];
          return true;
        default:
          break;
      }
      break;

    case 0x0417: // Ultraboot Maker's format code
      buf = find_buffer(2);
      if (!buf)
        return false;

      // Channel 2 contains the format drivecode. The last two bytes are
      // variable (speedzone and sectors/track), so ignore them for the hash.
      for (i = 0; i < buf->position-2; i++)
        crc = crc16_update(crc, buf->data[i]);

      if (crc == 0x60db && (buf->data[buf->position-2] & ~0x60) == 0) {
        fl_track = command_buffer[5];
        speedzone = buf->data[buf->position-2] >> 5;
        return true;
      }
      break;

    default:
      break;
  }

  return false;
}

// Map track and sector numbers according to the current speedzone
static void map_sector(volatile uint8_t *track, volatile uint8_t *sector) {
  uint8_t index;

  if (speedzone == 0 || fl_track < 36)
    return;

  index = pgm_read_byte(&sectors_per_track[speedzone]) * (*track-36) + *sector;
  *track = 36 + index / 17;
  *sector = index % 17;
}

bool format_ultraboot(UNUSED_PARAMETER) {
  uint8_t t;
  uint8_t s;

  if (!detect_ultraboot())
    return false;

  if (!check_d41_image())
    return true;

  if (fl_track > 40) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND, fl_track, 0);
    return true;
  }

  if (fl_track < 36)
    return true;

  if (fl_track == 36) {
    if (d64_extend_image(current_part, speedzone > 0 ? 42 : 40))
      return true;
  }

  fl_sector = 0;
  map_sector(&fl_track, &fl_sector);

  t = fl_track;
  s = pgm_read_byte(&sectors_per_track[speedzone]) - 1;
  map_sector(&t, &s);

  // mark all (mapped) sectors of the given track as valid
  while (t > fl_track || (t == fl_track && s >= fl_sector)) {
    d64_set_error(current_part, t, s, 1);

    if (s-- == 0) {
      t--;
      s = 16;
    }
  }

  // set loader for write_ultraboot() detection
  detected_loader = FL_ULTRABOOT;

  return true;
}

bool write_ultraboot(UNUSED_PARAMETER) {
  buffer_t *buf;

  fl_track = command_buffer[5];
  fl_sector = command_buffer[6];

  if (fl_track > 40) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND, fl_track, fl_sector);
    return true;
  }

  map_sector(&fl_track, &fl_sector);

  // Channel 2 has the sector data
  buf = find_buffer(2);
  if (!buf)
    return true;

  write_sector(buf, current_part, fl_track, fl_sector);

  return true;
}

static uint8_t ultraboot_send_block(const uint8_t* data) {
  uint16_t i;

  for (i = 0; i < 256; i++) {
    if (ultraboot_send_byte(data[i]))
      return 1;
  }

  return 0;
}

bool load_ultraboot(UNUSED_PARAMETER) {
  buffer_t *buf;

  if (!detect_ultraboot())
    return false;

  set_atn_irq(0);
  set_clock(0);
  set_data(1);

  buf = alloc_buffer();
  if (!buf)
    return true;

  while (!IEC_DATA)
    if (!IEC_ATN)
      goto exit;

  // Wait one frame after the host released data to make sure its screen is off
  delay_ms(20);

  while (fl_track > 0 && fl_track <= 40) {
    map_sector(&fl_track, &fl_sector);

    // UB requires an extended D41 image, so an error here is not unlikely
    read_sector(buf, current_part, fl_track, fl_sector);
    if (current_error != ERROR_OK)
      break;

    // Don't send the loader. We only read it to find the speedzone setting
    // and the start sector of the actual payload.
    if (fl_sector != 0 || fl_track != 36) {
      if (ultraboot_send_block(buf->data))
        break;
    } else {
      // get speedzone from offset 207 in loader
      if (buf->data[207] & ~0x60) // plausibility check for speedzone byte
        break;
      speedzone = buf->data[207] >> 5;
    }

    fl_sector = buf->data[254];
    fl_track  = buf->data[255];
  }

exit:
  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  return true;
}
