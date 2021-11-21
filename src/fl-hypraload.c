/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2021  Ingo Korb <ingo@akana.de>
   Final Cartridge III, DreamLoad, ELoad fastloader support:
   Copyright (C) 2008-2011  Thomas Giesel <skoe@directbox.com>
   Nippon Loader support:
   Copyright (C) 2010  Joerg Jungermann <abra@borkum.net>
   Hypra-Load support:
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


   fl-hypraload.c: High level handling of Hypra-Load fastloader

*/

#include <stdbool.h>
#include "config.h"
#include "buffers.h"
#include "fastloader-ll.h"
#include "fastloader.h"
#include "timer.h"

bool load_hypraload(UNUSED_PARAMETER) {
  buffer_t *buf;
  uint16_t i;
  uint8_t  status;

  buf = find_buffer(0);

  status = buf ? 0x55 : 0xff;

  // Wait one frame to make sure the host's screen is off
  delay_ms(20);

  set_atn_irq(0);

  while (1) {
    hypraload_send_byte(status);
    if (status == 0xff) // error
      break;

    hypraload_send_byte(!buf->sendeoi);
    hypraload_send_byte(buf->lastused);

    for (i = 2; i < 256; i++)
      hypraload_send_byte(buf->data[i]);

    if (buf->sendeoi)
      break;

    /* next sector */
    if (buf->refill(buf)) {
      status = 0xff;
    }
  }

  set_clock(1);
  set_data(1);
  set_atn_irq(1);

  return true;
}
