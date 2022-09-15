/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2021  Ingo Korb <ingo@akana.de>
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


   llfl-krill.c: Low level handling of Krill's loader "resend" transfers

*/

#include "config.h"
#include <arm/NXP/LPC17xx/LPC17xx.h>
#include <arm/bits.h>
#include "llfl-common.h"
#include "system.h"
#include "fastloader-ll.h"

static const generic_2bit_t krill_resend_def = {
  .pairtimes = {100, 180, 260, 340},
  .clockbits = {0, 2, 4, 6},
  .databits  = {1, 3, 5, 7},
  .eorvalue  = 0x00
};

/* Send a byte in the "resend" protocol used in Krill's loader r146 */
uint8_t krill_send_byte_resend(uint8_t byte) {
  llfl_setup();
  disable_interrupts();

  for (;;) {
    llfl_wait_atn(1);

    /* transmit data */
    llfl_generic_load_2bit(&krill_resend_def, byte);
    llfl_set_clock_at(420, 1, NO_WAIT);

    /* We are done if host sets ATN in time; otherwise send same byte again */
    if ((llfl_read_bus_at(500) & IEC_BIT_ATN) == 0)
      break;

    /* signal to host that we are going to resend */
    set_clock(0);
  }

  enable_interrupts();
  llfl_teardown();

  return 0;
}
