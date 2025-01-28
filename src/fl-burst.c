/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Fast Serial / Burst support:
   Copyright (C) 2024  Martin Thierer <mthierer@gmail.com>

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


   fl-burst.c: Handling of burst fastload command

*/

#include "config.h"
#include "buffers.h"
#include "display.h"
#include "doscmd.h"
#include "errormsg.h"
#include "iec-bus.h"
#include "parser.h"
#include "wrapops.h"
#include "fastloader-ll.h"

#define BURST_STATUS_OK               0b00000
#define BURST_STATUS_HEADER_NOT_FOUND 0b00010
#define BURST_STATUS_FILE_NOT_FOUND   0b00010 // fastload only
#define BURST_STATUS_DRIVE_NOT_READY  0b01111
#define BURST_STATUS_EOI              0b11111 // fastload only

static uint8_t clk_state;

/* send fast serial byte with handshake */
static void burst_send_byte(uint8_t b) {
  while ((!IEC_CLOCK) == clk_state) {
    if (!IEC_ATN)
      return;
  }

  fs_send_byte(b);
  clk_state = !clk_state;
}

/* map dos error code to back to job error for use in burst status */
static uint8_t translate_error(uint8_t error) {
  if (error >= ERROR_READ_NOHEADER && error <= ERROR_DISK_ID_MISMATCH) {
    error -= ERROR_READ_NOHEADER - BURST_STATUS_HEADER_NOT_FOUND;
  } else {
    /* invalid or unexpected error code */
    error = BURST_STATUS_DRIVE_NOT_READY;
  }

  return error;
}

void burst_fastload(void) {
  cbmdirent_t dent;
  uint8_t *fname;
  path_t path;
  buffer_t *buf;
  uint8_t i, first;

  command_buffer[command_length] = '\0';
  clk_state = 0;

  if (command_buffer[3] != '*' || !previous_file_dirent.name[0]) {
    /* Parse path and file name */
    if (parse_path(command_buffer+3, &path, &fname, 0)) {
      burst_send_byte(BURST_STATUS_FILE_NOT_FOUND);
      return;
    }

    uint8_t match_type = command_buffer[2] & 0x80 ? 0 : TYPE_PRG;

    if (first_match(&path, fname, match_type, &dent) != 0) {
      burst_send_byte(BURST_STATUS_FILE_NOT_FOUND);
      return;
    }

    previous_file_path   = path;
    previous_file_dirent = dent;
  } else { // open previous file
    path = previous_file_path;
    dent = previous_file_dirent;
  }

  buf = alloc_buffer();
  if (!buf)
    return;

  buf->secondary = 0;

  display_filename_read(path.part, CBM_NAME_LENGTH, dent.name);
  open_read(&path, &dent, buf, 0);
  unstick_buffer(buf);

  first = 1;

  while (1) {
    if (current_error != ERROR_OK) {
      burst_send_byte(translate_error(current_error));
      return;
    }

    if (!buf->sendeoi) {
      burst_send_byte(BURST_STATUS_OK);
    } else {
      burst_send_byte(BURST_STATUS_EOI);
      burst_send_byte(buf->lastused - (first ? 3 : 1));
    }

    i = 2;
    do {
      burst_send_byte(buf->data[i]);
      if (!IEC_ATN)
        goto abort;
    } while (i++ < buf->lastused);

    if (buf->sendeoi)
      break;

    first = 0;
    buf->refill(buf);
  }

abort:
  cleanup_and_free_buffer(buf);
}
