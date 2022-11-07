/* A simple macro for bitbanding on a Cortex M3 */
/* Written in 2010 by Ingo Korb                 */
#ifndef BITBAND_H
#define BITBAND_H

/* CM3 bit-band access macro */
#define BITBAND(addr,bit) \
  (*((volatile unsigned long *)( \
     ((unsigned long)&(addr) & 0x01ffffff)*32 + \
     (bit)*4 + 0x02000000 + ((unsigned long)&(addr) & 0xfe000000)       \
  )))

#endif
