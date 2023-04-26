/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>

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


   arch-timer.h: Architecture-specific system timer definitions

*/

#ifndef ARCH_TIMER_H
#define ARCH_TIMER_H

#include <util/delay.h>

/* use an approximated delay loop if the time isn't constant at compile time */
static inline __attribute__((always_inline)) void delay_ms(uint16_t delay) {
  if (__builtin_constant_p(delay)) {
    _delay_ms(delay);
  } else {
    while (delay--)
      delay_ms(1);
  }
}

#define delay_us(x) _delay_us(x)

/* Types for unsigned and signed tick values */
typedef uint16_t tick_t;
typedef int16_t stick_t;

/**
 * start_timeout - start a timeout using timer2
 * @usecs: number of microseconds before timeout (maximum 16384 for 16MHz clock)
 *
 * This function sets timer 2 so it will time out after the specified number
 * of microseconds. DON'T use a variable as parameter because it would cause
 * run-time floating point calculations (slow and huge).
 *
 * As timer 2 is 8 bit, it can count a maximum of 256 timer ticks, which is
 * 256us with a /8 prescaler and a 8MHz clock speed (128us at 16MHz).
 *
 * For longer timeouts the /1024 prescaler is used at the cost of a reduced
 * resolution. This always rounds up, so the timeout doesn't get shorter
 * than expected.
 */
static inline __attribute__((always_inline)) void start_timeout(uint16_t usecs) {
  tick_t ticks = ((float)F_CPU/8000000.0) * usecs;

  if (ticks == 0)
    return;

  if (ticks <= 256) {
    TCCR2B = _BV(CS21); /* F_CPU / 8 */
  } else {
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); /* F_CPU / 1024 */
    ticks = (ticks+127) >> 7; /* adjust timer ticks, rounding up */
  }

  if (ticks > 256) /* at least prevent an overflow if timeout period too long*/
    ticks = 256;

  GTCCR  = _BV(PSRASY); /* reset timer 2 prescaler */
  TCNT2  = 256 - ticks;
  TIFR2 |= _BV(TOV2);   /* reset overflow flag */
}

/**
 * has_timed_out - returns true if timeout was reached
 *
 * This function returns true if the overflow flag of timer 2 is set which
 * (together with start_timeout and TIMEOUT_US) will happen when the
 * specified time has elapsed.
 */
static inline uint8_t has_timed_out(void) {
  return TIFR2 & _BV(TOV2);
}

#endif
