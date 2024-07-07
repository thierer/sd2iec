/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   Hypra-Load support:
   Copyright (C) 2021-2024  Martin Thierer <mthierer@gmail.com>

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


   llfl-hypraload.c: Low level handling of Hypra-Load transfers

*/

#include "config.h"
#include "bitband.h"
#include "iec-bus.h"
#include "llfl-common.h"
#include "system.h"
#include "timer.h"
#include "fastloader-ll.h"


static const generic_2bit_t hypraload_send_def = {
  .pairtimes = {300, 550, 800, 1050},
  .clockbits = {0, 2, 4, 6},
  .databits  = {1, 3, 5, 7},
  .eorvalue  = 0xff
};

uint8_t hypraload10_send_byte(uint8_t byte) {
  llfl_setup();
  disable_interrupts();

  while (IEC_ATN);
  set_data(1);

  /* wait for start signal */
  llfl_wait_atn(1);

  /* transmit data */
  llfl_generic_load_2bit(&hypraload_send_def, byte);

  /* data hold time */
  delay_us(25);

  set_data(0);

  enable_interrupts();
  llfl_teardown();

  return 0;
}

uint8_t hypraload21_send_byte(uint8_t byte) {
  llfl_setup();
  disable_interrupts();

  while (IEC_DATA);
  set_clock(1);

  /* wait for start signal */
  llfl_wait_data(1, NO_ATNABORT);

  /* transmit data */
  llfl_generic_load_2bit(&hypraload_send_def, byte);

  /* data hold time */
  delay_us(25);

  set_clock(0);
  set_data(1);

  enable_interrupts();
  llfl_teardown();

  return 0;
}
