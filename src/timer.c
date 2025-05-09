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


   timer.c: System timer (and button debouncer)

*/

#include "config.h"
#include "diskchange.h"
#include "display.h"
#include "led.h"
#include "time.h"
#include "rtc.h"
#include "softrtc.h"
#include "timer.h"

#define DEBOUNCE_TICKS 4
#define SLEEP_TICKS    2*HZ

static tick_t ticks;

tick_t getticks(void) {
  tick_t tmp;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    tmp = ticks;
  }
  return tmp;
}

// Logical buttons
static uint8_t active_keys;

// Physical buttons
rawbutton_t buttonstate;
tick_t      lastbuttonchange;

uint8_t key_pressed(uint8_t mask) {
  return active_keys & mask;
}

void reset_key(uint8_t mask) {
  active_keys &= (uint8_t)~mask;
}

void set_key(uint8_t mask) {
  active_keys |= mask;
}

/* Called by the timer interrupt when the button state has changed */
static void buttons_changed(rawbutton_t new_state) {
  /* Check if the previous state was stable for two ticks */
  if (time_after(ticks, lastbuttonchange + DEBOUNCE_TICKS)) {
    if (key_pressed(IGNORE_KEYS)) {
      reset_key(IGNORE_KEYS);
    } else if (!(buttonstate & BUTTON_RESET) &&
               (new_state & BUTTON_RESET)) {
      set_key(KEY_RESET);
    } else if (BUTTON_PREV && /* match only if PREV exists */
               !(buttonstate & (BUTTON_PREV|BUTTON_NEXT))) {
      /* Both buttons held down */
      set_key(KEY_HOME);
    } else if (!(buttonstate & BUTTON_NEXT) &&
               (new_state & BUTTON_NEXT)) {
      /* "Next" button released */
      set_key(KEY_NEXT);
    } else if (BUTTON_PREV && /* match only if PREV exists */
               !(buttonstate & BUTTON_PREV) &&
               (new_state & BUTTON_NEXT)) {
      set_key(KEY_PREV);
    }
  }

  lastbuttonchange = ticks;
  buttonstate = new_state;
}

/* The main timer interrupt */
SYSTEM_TICK_HANDLER {
  set_tick_irq(0);

  rawbutton_t tmp = buttons_read();

  if (tmp != buttonstate) {
    buttons_changed(tmp);
  }

  ticks++;

#ifdef SINGLE_LED
  if (led_state & LED_ERROR) {
    if ((ticks & 15) == 0)
      toggle_led();
  } else {
    set_led((led_state & LED_BUSY) || (led_state & LED_DIRTY));
  }
#else
  if (led_state & LED_ERROR)
    if ((ticks & 15) == 0)
      toggle_dirty_led();
#endif

  /* Sleep button triggers when held down for 2sec */
  if (time_after(ticks, lastbuttonchange + DEBOUNCE_TICKS)) {
    if (!(buttonstate & BUTTON_NEXT) &&
        (!BUTTON_PREV || (buttonstate & BUTTON_PREV)) &&
        time_after(ticks, lastbuttonchange + SLEEP_TICKS) &&
        !key_pressed(KEY_SLEEP)) {
      /* Set ignore flag so the release doesn't trigger KEY_NEXT */
      set_key(KEY_SLEEP | IGNORE_KEYS);
      /* Avoid triggering for the next two seconds */
      lastbuttonchange = ticks;
    }
  }

  /* send tick to the software RTC emulation */
  if (rtc_state == RTC_OK)
    softrtc_tick();

#ifdef CONFIG_REMOTE_DISPLAY
  /* Check if the display wants to be queried */
  if (display_intrq_active()) {
    set_key(KEY_DISPLAY);
  }
#endif

  set_tick_irq(1);
}
