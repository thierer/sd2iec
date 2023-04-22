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


   fl-samsjourney.c: Fastloader of Sam's Journey

*/

#include <stdio.h>

#include <string.h>
#include "config.h"
#include "buffers.h"
#include "diskchange.h"
#include "doscmd.h"
#include "fastloader-ll.h"
#include "fileops.h"
#include "iec-bus.h"
#include "iec.h"
#include "parser.h"
#include "system.h"
#include "timer.h"
#include "uart.h"
#include "wrapops.h"
#include "fastloader.h"


static const PROGMEM uint8_t hexchars[16] = "0123456789ABCDEF";

static uint8_t hex2bin(uint8_t *ch) {
  uint8_t result;

  if (*ch >= '0' && *ch <= '9') {
    result = (*ch - '0') << 4;
  } else if (*ch >= 'A' && *ch <= 'F') {
    result = (*ch - 'A' + 10) << 4;
  } else {
    return 0xff;
  }

  ch++;
  if (*ch >= '0' && *ch <= '9') {
    result |= (*ch - '0');
  } else if (*ch >= 'A' && *ch <= 'F') {
    result |= (*ch - 'A' + 10);
  } else {
    return 0xff;
  }

  return result;
}

/* note: similar, but not identical to fl-nosdos.c:getbyte */
static int16_t getbyte(void) {
  uint8_t byte = 0;
  iec_bus_t bus;

  for (uint8_t i = 0; i < 8; i++) {
    /* release bus */
    set_clock(1);
    set_data(1);
    delay_us(2);

    /* wait for bit */
    do {
      check_keys();

      bus = iec_bus_read();

      /* immediately abort if ATN is low */
      if (!(bus & IEC_BIT_ATN))
        return -1;
    } while ((bus & (IEC_BIT_CLOCK | IEC_BIT_DATA)) ==
                    (IEC_BIT_CLOCK | IEC_BIT_DATA));

    byte >>= 1;
    if (!(bus & IEC_BIT_DATA))
      byte |= 0x80;

    /* acknowledge it */
    if (bus & IEC_BIT_DATA)
      set_data(0);
    else
      set_clock(0);
    delay_us(2);

    /* wait for C64's acknowledge */
    do {
      bus = iec_bus_read();

      if (!(bus & IEC_BIT_ATN))
        return -1;
    } while ((bus & (IEC_BIT_CLOCK | IEC_BIT_DATA)) == 0);
  }

  return byte;
}

static void transmit_byte(uint8_t byte) {
  byte = ~byte;

  while (!IEC_ATN) ;
  set_clock(byte & 0x80);
  set_data(byte & 0x20);

  while (IEC_ATN) ;
  set_clock(byte & 0x40);
  set_data(byte & 0x10);

  while (!IEC_ATN) ;
  set_clock(byte & 8);
  set_data(byte & 2);

  while (IEC_ATN) ;
  set_clock(byte & 4);
  set_data(byte & 1);
}

static void transmit_block(uint8_t continue_marker, uint8_t length, uint8_t *data) {
  disable_interrupts();

  set_clock(1);
  set_data(1);
  while (IEC_ATN) ;

  set_clock(0);
  set_data(0);

  transmit_byte((length + 2) & 0xff);
  transmit_byte(continue_marker);

  for (uint8_t i = 0; i < length; i++) {
    transmit_byte(data[i]);
  }

  while (!IEC_ATN) ;

  set_clock(0);
  set_data(0);

  enable_interrupts();
}

static void send_error(void) {
  transmit_block(0xff, 0, NULL);
}

static void scan_directory(void) {
  path_t path;
  dh_t dh;
  int8_t res;
  cbmdirent_t dent;
  uint8_t entry[3] = {0xff, 0, 0};

  path.part = current_part;
  path.dir  = partition[path.part].current_dir;

  if (opendir(&dh, &path)) {
    send_error();
  }

  while (1) {
    res = readdir(&dh, &dent);

    if (res > 0) {
      send_error();
    }

    if (res < 0) {
      /* end of directory, send final entry */
      /* (assumes that there is always at least one entry) */
      transmit_block(1, sizeof(entry), entry);
      return;
    }

    /* res == 0, successfully read a directory entry */
    if ((dent.typeflags & TYPE_MASK) != TYPE_PRG)
      /* skip non-PRG entries */
      continue;

    if (entry[1] != 0) {
      /* delayed transmit to avoid sending dummy entry at the end */
      transmit_block(0, sizeof(entry), entry);
    }

    entry[0] = hex2bin(dent.name);
    entry[1] = (entry[0] >> 4) + 1;
    entry[2] = entry[0] & 0xf;
  }
}

static void read_file_by_name(uint8_t name) {
  buffer_t *buf;

  /* try to open the actual file */
  command_buffer[0] = pgm_read_byte(hexchars + (name >> 4));
  command_buffer[1] = pgm_read_byte(hexchars + (name & 0xf));
  command_length = 2;

  file_open(0);
  buf = find_buffer(0);
  if (!buf) {
    send_error();
    return;
  }

  /* transfer data */
  while (1) {
    transmit_block(!!buf->sendeoi, buf->lastused - 1, buf->data + 2);

    if (buf->sendeoi)
      break;

    if (buf->refill(buf)) {
      send_error();
      return;
    }
  }

  uart_putcrlf();
  cleanup_and_free_buffer(buf);
}

static uint8_t write_file_by_name(uint8_t name) {
  buffer_t *buf;
  int16_t res, length;
  uint8_t retval = 1;

  /* open with replace */
  command_buffer[0] = '@';
  command_buffer[1] = ':';
  command_buffer[2] = pgm_read_byte(hexchars + (name >> 4));
  command_buffer[3] = pgm_read_byte(hexchars + (name & 0xf));
  command_length = 4;

  file_open(1);
  buf = find_buffer(1);
  if (!buf) {
    send_error();
    return 0;
  }

  /* send success marker */
  transmit_block(0, 0, NULL);

  while (1) {
    /* receive length byte */
    length = getbyte();
    if (length < 0)
      break;

    if (length == 0) {
      retval = 0;
      break;
    }

    /* receive the actual data */
    while (length-- > 0) {
      if (buf->mustflush) {
        buf->refill(buf);
      }

      res = getbyte();
      if (res < 0)
        goto fail;
      buf->data[buf->position] = res;

      if (buf->lastused < buf->position)
        buf->lastused = buf->position;
      buf->position++;

      if (buf->position == 0)
        buf->mustflush = 1;
    }
  }

 fail:
  cleanup_and_free_buffer(buf);
  return retval;
}

static uint8_t ts_to_name(const uint8_t track, const uint8_t sector) {
  if (!track || track > 16 || sector > 15)
    return 0xff;

  return ((track - 1) << 4) | sector;
}

static void read_file_by_ts(uint8_t track, uint8_t sector) {
  read_file_by_name(ts_to_name(track, sector));
}

static uint8_t write_file_by_ts(uint8_t track, uint8_t sector) {
  return write_file_by_name(ts_to_name(track, sector));
}

bool load_samsjourney(UNUSED_PARAMETER) {
  int16_t tmp;
  uint8_t command, cmd_len, cmd_buffer[4];
  uint8_t abort = 0;

  memset(cmd_buffer, 0, sizeof(cmd_buffer));

  /* avoid interference from the preceeding IEC transaction */
  delay_ms(1);

  while (!abort) {
    /* receive command byte and argument block */
    tmp = getbyte();
    if (tmp < 0)
      break;
    command = tmp;

    tmp = getbyte();
    if (tmp < 0)
      break;
    cmd_len = tmp;

    if (cmd_len > 0) {
      /* receive argument block, discard anything beyond the buffer size */
      for (uint8_t i = 0; i < cmd_len; i++) {
        tmp = getbyte();
        if (tmp < 0)
          goto abort;

        if (i < sizeof(cmd_buffer))
          cmd_buffer[i] = tmp;
      }
    }

    /* dispatch */
    switch (command) {
    case 1:
      scan_directory();
      break;

    case 2:
      read_file_by_name(cmd_buffer[0]);
      break;

    case 3:
      if (write_file_by_name(cmd_buffer[0]))
        goto abort;
      break;

    case 0x82:
      read_file_by_ts(cmd_buffer[0], cmd_buffer[1]);
      break;

    case 0x83:
      if (write_file_by_ts(cmd_buffer[0], cmd_buffer[1]))
        goto abort;
      break;

    default:
      send_error();
      break;
    }
  }

 abort:
  /* release bus lines and return */
  set_data(1);
  set_clock(1);

  return true;
}
