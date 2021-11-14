/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2021  Ingo Korb <ingo@akana.de>
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


   llfl-ultraboot.c: Low level handling of Ultraboot transfers

*/

#include "config.h"
#include <arm/NXP/LPC17xx/LPC17xx.h>
#include <arm/bits.h>
#include "iec-bus.h"
#include "llfl-common.h"
#include "system.h"
#include "timer.h"
#include "fastloader-ll.h"


static const generic_2bit_t ultraboot_send_def = {
  .pairtimes = {190, 270, 350, 430},
  .clockbits = {7, 6, 3, 2},
  .databits  = {5, 4, 1, 0},
  .eorvalue  = 0xff
};

uint8_t ultraboot_send_byte(uint8_t byte) {
  uint8_t result;

  llfl_setup();
  disable_interrupts();

  while (!IEC_DATA)
    if (!IEC_ATN) {
      result = 1;
      goto exit;
    }

  /* start in 1 us */
  llfl_reference_time = llfl_now() + 10;
  llfl_set_data_at (  0, 0, WAIT);
  llfl_set_data_at (150, 1, WAIT);

  /* transmit data */
  llfl_generic_load_2bit(&ultraboot_send_def, byte);

  llfl_set_data_at (510, 1, WAIT);
  result = (llfl_read_bus_at(560) & IEC_BIT_DATA) != 0;

exit:
  enable_interrupts();
  llfl_teardown();
  return result;
}
