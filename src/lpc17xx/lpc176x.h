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


   lpc17xx.h: Register definitions for LPC176x (to avoid licensing annoyances)

*/

#ifndef LPC176X_H
#define LPC176X_H

#include <stdint.h>

#define REG_R8   const volatile uint8_t
#define REG_R16  const volatile uint16_t
#define REG_R    const volatile uint32_t
#define REG_W8   volatile uint8_t
#define REG_W16  volatile uint16_t
#define REG_W    volatile uint32_t
#define REG_RW8  volatile uint8_t
#define REG_RW16 volatile uint16_t
#define REG_RW   volatile uint32_t

#define LPCREGS_CONCAT2(x, y) x ## y
#define LPCREGS_CONCAT(x, y) LPCREGS_CONCAT2(x, y)

#define UNUSED8(len)  const uint8_t  LPCREGS_CONCAT(__unused_, __LINE__) [len]
#define UNUSED16(len) const uint16_t LPCREGS_CONCAT(__unused_, __LINE__) [len]
#define UNUSED(len)   const uint32_t LPCREGS_CONCAT(__unused_, __LINE__) [len]

struct SysTick_Regs {
  REG_RW CTRL;        // 10
  REG_RW LOAD;        // 14
  REG_RW VAL;         // 18
  REG_R  CALIB;       // 1c
};
_Static_assert(__builtin_offsetof(struct SysTick_Regs, CALIB) == 0x1c - 0x10, "struct SysTick_Regs has incorrect size");

#define SysTick_CTRL_COUNTFLAG (1U << 16)
#define SysTick_CTRL_CLKSOURCE (1U << 2)
#define SysTick_CTRL_TICKINT   (1U << 1)
#define SysTick_CTRL_ENABLE    (1U << 0)


struct SCB_Regs {
  REG_R  CPUID;        // 00
  REG_RW ICSR;         // 04
  REG_RW VTOR;         // 08
  REG_RW AIRCR;        // 0c
  REG_RW SCR;          // 10
  REG_RW CCR;          // 14
  union {
    struct {
      REG_RW SHPR1;    // 18
      REG_RW SHPR2;    // 1c
      REG_RW SHPR3;    // 20
    };
    REG_RW8 SHP[12];   // 18..23
  };
  REG_RW SHCRS;        // 24
  union {              // 28
    REG_RW CFSR;
    struct {
      REG_RW8  MMSR;   // 28
      REG_RW8  BFSR;   // 29
      REG_RW16 UFSR;   // 2a
    };
  };
  REG_RW HFSR;         // 2c
  UNUSED(1);           // 30
  REG_RW MMFAR;        // 34
  REG_RW BFAR;         // 38;
};
_Static_assert(__builtin_offsetof(struct SCB_Regs, BFAR) == 0x38, "struct SCB_Regs has incorrect size");


struct NVIC_Regs {
  // struct starts at address 0x...100
  union {
    struct {
      REG_RW ISER0;    // 100
      REG_RW ISER1;    // 104
      REG_RW ISER2;    // 108
      REG_RW ISER3;    // 10c
    };
    REG_RW ISER[4];    // 100..10c
  };

  UNUSED(28);          // 110..17c

  union {
    struct {
      REG_RW ICER0;    // 180
      REG_RW ICER1;    // 184
      REG_RW ICER2;    // 188
      REG_RW ICER3;    // 18c
    };
    REG_RW ICER[4];    // 180..18c
  };

  UNUSED(28);          // 190..1fc

  union {
    struct {
      REG_RW ISPR0;    // 200
      REG_RW ISPR1;    // 204
      REG_RW ISPR2;    // 208
      REG_RW ISPR3;    // 20c
    };
    REG_RW ISPR[4];    // 200.20c
  };

  UNUSED(28);          // 210..27f

  union {
    struct {
      REG_RW ICPR0;    // 280
      REG_RW ICPR1;    // 284
      REG_RW ICPR2;    // 288
      REG_RW ICPR3;    // 28c
    };
    REG_RW ICPR[4];    // 280..28c
  };

  UNUSED(28);          // 290..2fc

  union {
    struct {
      REG_R  IABR0;    // 300
      REG_R  IABR1;    // 304
      REG_R  IABR2;    // 308
      REG_R  IABR3;    // 30c
    };
    REG_R  IABR[4];    // 300..30c
  };

  UNUSED(60);          // 310..3fc

  union {
    struct {
      REG_RW IPR0;     // 400
      REG_RW IPR1;     // 404
      REG_RW IPR2;     // 408
      REG_RW IPR3;     // 40c
      REG_RW IPR4;     // 410
      REG_RW IPR5;     // 414
      REG_RW IPR6;     // 418
      REG_RW IPR7;     // 41c
      REG_RW IPR8;     // 420
      REG_RW IPR9;     // 424
      REG_RW IPR10;    // 428
      REG_RW IPR11;    // 42c
      REG_RW IPR12;    // 430
      REG_RW IPR13;    // 434
      REG_RW IPR14;    // 438
      REG_RW IPR15;    // 43c
      REG_RW IPR16;    // 440
      REG_RW IPR17;    // 444
      REG_RW IPR18;    // 448
      REG_RW IPR19;    // 44c
      REG_RW IPR20;    // 450
      REG_RW IPR21;    // 454
      REG_RW IPR22;    // 458
      REG_RW IPR23;    // 45c
      REG_RW IPR24;    // 460
      REG_RW IPR25;    // 464
      REG_RW IPR26;    // 468
      REG_RW IPR27;    // 46c
    };
    REG_RW  IPR[28];   // 400..46c
    REG_RW8 IP[112];
  };

  UNUSED(676);         // 470..efc (actually not fully unused, System Control Block lives here)

  REG_W STIR;          // f00
};
_Static_assert(__builtin_offsetof(struct NVIC_Regs, STIR) == 0xf00 - 0x100, "struct NVUC_Regs has incorrect size");

#define NVIC_PRIORITY_HIGHEST 0
#define NVIC_PRIORITY_LOWEST  ((1 << 5) - 1)


struct LPC_GPDMA_Regs {
  REG_R  DMACIntStat;       // 00
  REG_R  DMACIntTCStat;     // 04
  REG_W  DMACIntTCClear;    // 08
  REG_R  DMACIntErrStat;    // 0c
  REG_W  DMACIntErrClr;     // 10
  REG_R  DMACRawIntTCStat;  // 14
  REG_R  DMACRawIntErrStat; // 18
  REG_R  DMACEnbldChns;     // 1c
  REG_RW DMACSoftBReq;      // 20
  REG_RW DMACSoftSReq;      // 24
  REG_RW DMACSoftLBReq;     // 28
  REG_RW DMACSoftLSReq;     // 2c
  REG_RW DMACConfig;        // 30
  REG_RW DMACSync;          // 34
};
_Static_assert(__builtin_offsetof(struct LPC_GPDMA_Regs, DMACSync) == 0x34, "struct LPC_GPDMA_Regs has incorrect size");


struct LPC_GPDMACH_Regs {
  REG_RW DMACCSrcAddr;      // 00
  REG_RW DMACCDestAddr;     // 04
  REG_RW DMACCLLI;          // 08
  REG_RW DMACCControl;      // 0c
  REG_RW DMACCConfig;       // 10
};
_Static_assert(__builtin_offsetof(struct LPC_GPDMACH_Regs, DMACCConfig) == 0x10, "struct LPC_GPDMACH_Regs has incorrect size");


struct LPC_GPIO_Regs {
  REG_RW FIODIR;     // 00
  UNUSED(3);         // 04..0c
  REG_RW FIOMASK;    // 10
  REG_RW FIOPIN;     // 14
  REG_RW FIOSET;     // 18
  REG_RW FIOCLR;     // 1c
};
_Static_assert(__builtin_offsetof(struct LPC_GPIO_Regs, FIOCLR) == 0x1c, "struct LPC_GPIO_Regs has incorrect size");


struct LPC_GPIOINT_Regs {
  REG_R  IntStatus;    // 80
  REG_R  IO0IntStatR;  // 84
  REG_R  IO0IntStatF;  // 88
  REG_W  IO0IntClr;    // 8c
  REG_RW IO0IntEnR;    // 90
  REG_RW IO0IntEnF;    // 94
  UNUSED(3);           // 98..a0
  REG_R  IO2IntStatR;  // a4
  REG_R  IO2IntStatF;  // a8
  REG_W  IO2IntClr;    // ac
  REG_RW IO2IntEnR;    // b0
  REG_RW IO2IntEnF;    // b4
};
_Static_assert(__builtin_offsetof(struct LPC_GPIOINT_Regs, IO2IntEnF) == 0xb4 - 0x80, "struct LPC_GPIOINT_Regs has incorrect size");


struct LPC_I2C_Regs {
  REG_RW I2CONSET;      // 00
  REG_R  I2STAT;        // 04
  REG_RW I2DAT;         // 08
  REG_RW I2ADR0;        // 0c
  REG_RW I2SCLH;        // 10
  REG_RW I2SCLL;        // 14
  REG_W  I2CONCLR;      // 18
  REG_RW MMCTRL;        // 1c
  REG_RW I2ADR1;        // 20
  REG_RW I2ADR2;        // 24
  REG_RW I2ADR3;        // 28
  REG_R  I2DATA_BUFFER; // 2c
  REG_RW I2MASK0;       // 30
  REG_RW I2MASK1;       // 34
  REG_RW I2MASK2;       // 38
  REG_RW I2MASK3;       // 3c
};
_Static_assert(__builtin_offsetof(struct LPC_I2C_Regs, I2MASK3) == 0x3c, "struct LPC-I2C_Regs has incorrect size");


struct LPC_PINCON_Regs {
  REG_RW PINSEL0;     // 00
  REG_RW PINSEL1;     // 04
  REG_RW PINSEL2;     // 08
  REG_RW PINSEL3;     // 0c
  REG_RW PINSEL4;     // 10
  REG_RW PINSEL5;     // 14
  REG_RW PINSEL6;     // 18
  REG_RW PINSEL7;     // 1c
  REG_RW PINSEL8;     // 20
  REG_RW PINSEL9;     // 24
  REG_RW PINSEL10;    // 28
  UNUSED(5);          // 2c..3c
  REG_RW PINMODE0;    // 40
  REG_RW PINMODE1;    // 44
  REG_RW PINMODE2;    // 48
  REG_RW PINMODE3;    // 4c
  REG_RW PINMODE4;    // 50
  REG_RW PINMODE5;    // 54
  REG_RW PINMODE6;    // 58
  REG_RW PINMODE7;    // 5c
  UNUSED(1);          // 60
  REG_RW PINMODE9;    // 64
  REG_RW PINMODE_OD0; // 68
  REG_RW PINMODE_OD1; // 6c
  REG_RW PINMODE_OD2; // 70
  REG_RW PINMODE_OD3; // 74
  REG_RW PINMODE_OD4; // 78
  REG_RW I2CPADCFG;   // 7c
};
_Static_assert(__builtin_offsetof(struct LPC_PINCON_Regs, I2CPADCFG) == 0x7c, "struct LPC_PINCON_Regs has incorrect size");


struct LPC_RIT_Regs {
  REG_RW  RICOMPVAL;    // 00
  REG_RW  RIMASK;       // 04
  REG_RW8 RICTRL;       // 08
  UNUSED8(3);           // 09..0b
  REG_RW  RICOUNTER;    // 0c
};
_Static_assert(__builtin_offsetof(struct LPC_RIT_Regs, RICOUNTER) == 0xc, "struct LPC_RIT_Regs has incorrect size");


struct LPC_RTC_Regs {
  REG_RW8 ILR;          // 00
  UNUSED8(7);           // 01..07
  REG_RW8 CCR;          // 08
  UNUSED8(3);           // 09..0b
  REG_RW8 CIIR;         // 0c
  UNUSED8(3);           // 0d..0f
  REG_RW8 AMR;          // 10
  UNUSED8(3);           // 11..13
  REG_R  CTIME0;        // 14
  REG_R  CTIME1;        // 18
  REG_R  CTIME2;        // 1c
  REG_RW8 SEC;          // 20
  UNUSED8(3);           // 21..23
  REG_RW8 MIN;          // 24
  UNUSED8(3);           // 25..27
  REG_RW8 HOUR;         // 28
  UNUSED8(3);           // 29..2b
  REG_RW8 DOM;          // 2c
  UNUSED8(3);           // 2d..2f
  REG_RW8 DOW;          // 30
  UNUSED8(3);           // 31..33
  REG_RW16 DOY;         // 34
  UNUSED16(1);          // 36..37
  REG_RW8 MONTH;        // 38
  UNUSED8(3);           // 39..3b
  REG_RW16 YEAR;        // 3c
  UNUSED16(1);          // 3e..3f
  REG_RW CALIBRATION;   // 40
  REG_RW GPREG0;        // 44
  REG_RW GPREG1;        // 48
  REG_RW GPREG2;        // 4c
  REG_RW GPREG3;        // 50
  REG_RW GPREG4;        // 54
  REG_RW8 RTC_AUXEN;    // 58
  UNUSED8(3);           // 59..5b
  REG_RW8 RTC_AUX;      // 5c
  UNUSED8(3);           // 5d..5f
  REG_RW8 ALSEC;        // 60
  UNUSED8(3);           // 61..63
  REG_RW8 ALMIN;        // 64
  UNUSED8(3);           // 65..67
  REG_RW8 ALHOUR;       // 68
  UNUSED8(3);           // 69..6b
  REG_RW8 ALDOM;        // 6c
  UNUSED8(3);           // 6d..6f
  REG_RW8 ALDOW;        // 70
  UNUSED8(3);           // 71..73
  REG_RW16 ALDOY;       // 74
  UNUSED16(1);          // 76..77
  REG_RW8 ALMON;        // 78
  UNUSED8(3);           // 79..7b
  REG_RW16 ALYEAR;      // 7c
};
_Static_assert(__builtin_offsetof(struct LPC_RTC_Regs, ALYEAR) == 0x7c, "struct LPC_RTC_Regs has incorrect size");


struct LPC_SC_Regs {
  REG_RW FLASHCFG;      // 00
  UNUSED(31);           // 04..7c
  REG_RW PLL0CON;       // 080
  REG_RW PLL0CFG;       // 084
  REG_R  PLL0STAT;      // 088
  REG_W  PLL0FEED;      // 08c
  UNUSED(4);            // 090..09c
  REG_RW PLL1CON;       // 0a0
  REG_RW PLL1CFG;       // 0a4
  REG_R  PLL1STAT;      // 0a8
  REG_W  PLL1FEED;      // 0ac
  UNUSED(4);            // 0b0..0bc
  REG_RW PCON;          // 0c0
  REG_RW PCONP;         // 0c4
  UNUSED(15);           // 0c8..100
  REG_RW CCLKCFG;       // 104
  REG_RW USBCLKCFG;     // 108
  REG_RW CLKSRCSEL;     // 10c
  UNUSED(12);           // 110..13c
  REG_RW EXTINT;        // 140
  REG_R  _unused6;      // 144
  REG_RW EXTMODE;       // 148
  REG_RW EXTPOLAR;      // 14c
  UNUSED(12);           // 150..17c
  REG_RW RSID;          // 180
  UNUSED(7);            // 184..19c
  REG_RW SCS;           // 1a0
  UNUSED(1);            // 1a4
  REG_RW PCLKSEL0;      // 1a8
  REG_RW PCLKSEL1;      // 1ac
  UNUSED(5);            // 1b0..1c0
  REG_RW DMAREQSEL;     // 1c4
};
_Static_assert(__builtin_offsetof(struct LPC_SC_Regs, DMAREQSEL) == 0x1c4, "struct LPC_SC_Regs has incorrect size");


struct LPC_SSP_Regs {
  REG_RW CR0;           // 00
  REG_RW CR1;           // 04
  REG_RW DR;            // 08
  REG_R  SR;            // 0c
  REG_RW CPSR;          // 10
  REG_RW IMSC;          // 14
  REG_RW RIS;           // 18
  REG_RW MIS;           // 1c
  REG_RW ICR;           // 20
  REG_RW DMACR;         // 24
};
_Static_assert(__builtin_offsetof(struct LPC_SSP_Regs, DMACR) == 0x24, "struct LPC_SSP_Regs has incorrect size");


struct LPC_TIMER_Regs {
  REG_RW IR;           // 00
  REG_RW TCR;          // 04
  REG_RW TC;           // 08
  REG_RW PR;           // 0c
  REG_RW PC;           // 10
  REG_RW MCR;          // 14
  REG_RW MR0;          // 18
  REG_RW MR1;          // 1c
  REG_RW MR2;          // 20
  REG_RW MR3;          // 24
  REG_RW CCR;          // 28
  REG_R  CR0;          // 2c
  REG_R  CR1;          // 30
  UNUSED(2);           // 34..3c
  REG_RW EMR;          // 3c
  UNUSED(12);          // 40..6c
  REG_RW CTCR;         // 70
};
_Static_assert(__builtin_offsetof(struct LPC_TIMER_Regs, CTCR) == 0x70, "struct LPC_Timer_Regs has incorrect size");


struct LPC_UART0_Regs {
  union {              // 00
    REG_R8  RBR;
    REG_W8  THR;
    REG_RW8 DLL;
    UNUSED(1);
  };

  union {              // 04
    REG_RW8  DLM;
    REG_RW16 IER;
    UNUSED(1);
  };

  union {              // 08
    REG_RW IIR;
    REG_W8 FCR;
    UNUSED(1);
  };

  REG_RW8  LCR;        // 0c
  UNUSED8(7);          // 0d..13
  REG_R8   LSR;        // 14
  UNUSED8(7);          // 15..1b
  REG_RW8  SCR;        // 1c
  UNUSED8(3);          // 1d..1f
  REG_RW16 ACR;        // 20
  UNUSED16(1);         // 22
  REG_RW8  ICR;        // 24
  UNUSED8(3);          // 25..27
  REG_RW8  FDR;        // 28
  UNUSED8(7);          // 29..2f
  REG_RW8  TER;        // 30
};
_Static_assert(__builtin_offsetof(struct LPC_UART0_Regs, TER) == 0x30, "struct LPC_UART0_Regs has incorrect size");


struct LPC_UART1_Regs {
  union {              // 00
    REG_R  RBR;
    REG_W  THR;
    REG_RW DLL;
  };

  union {              // 04
    REG_RW DLM;
    REG_RW IER;
  };

  union {              // 08
    REG_R  IIR;
    REG_W  FCR;
  };

  REG_RW LCR;          // 0c
  REG_R  MCR;          // 10
  REG_R  LSR;          // 14
  REG_R  MSR;          // 18
  REG_RW SCR;          // 1c
  REG_RW ACR;          // 20
  UNUSED(1);           // 24
  REG_RW FDR;          // 28
  UNUSED(1);           // 2c
  REG_RW TER;          // 30
  UNUSED(6);           // 34..48
  REG_RW RS485CTRL;    // 4c
  REG_RW ADRMATCH;     // 50
  REG_RW RS485DLY;     // 54
};
_Static_assert(__builtin_offsetof(struct LPC_UART1_Regs, RS485DLY) == 0x54, "struct LPC_UART1_Regs has incorrect size");



struct LPC_WDT_Regs {
  REG_RW8 WDMOD;       // 00
  UNUSED8(3);          // 01..03
  REG_RW  WDTC;        // 04
  REG_W8  WDFEED;      // 08
  UNUSED8(3);          // 09..0b
  REG_R   WDTV;        // 0c
  REG_RW  WDCLKSEL;    // 10
};
_Static_assert(__builtin_offsetof(struct LPC_WDT_Regs, WDCLKSEL) == 0x10, "struct LPC_WDT_Regs has incorrect size");


#define LPC_GPIO_BASE     0x2009c000
#define LPC_APB0_BASE     0x40000000
#define LPC_APB1_BASE     0x40080000

#define SYSTICK_BASE      0xe000e010
#define NVIC_BASE         0xe000e100
#define SCB_BASE          0xe000ed00

#define LPC_GPDMA_BASE    0x50004000
#define LPC_GPDMACH0_BASE (LPC_GPDMA_BASE + 0x100)
#define LPC_GPDMACH1_BASE (LPC_GPDMA_BASE + 0x120)
#define LPC_GPDMACH2_BASE (LPC_GPDMA_BASE + 0x140)
#define LPC_GPDMACH3_BASE (LPC_GPDMA_BASE + 0x160)
#define LPC_GPDMACH4_BASE (LPC_GPDMA_BASE + 0x180)
#define LPC_GPDMACH5_BASE (LPC_GPDMA_BASE + 0x1a0)
#define LPC_GPDMACH6_BASE (LPC_GPDMA_BASE + 0x1c0)
#define LPC_GPDMACH7_BASE (LPC_GPDMA_BASE + 0x1e0)

#define LPC_GPIO0_BASE    LPC_GPIO_BASE
#define LPC_GPIO1_BASE    (LPC_GPIO_BASE + 0x20)
#define LPC_GPIO2_BASE    (LPC_GPIO_BASE + 0x40)
#define LPC_GPIO3_BASE    (LPC_GPIO_BASE + 0x60)
#define LPC_GPIO4_BASE    (LPC_GPIO_BASE + 0x80)

#define LPC_I2C0_BASE     (LPC_APB0_BASE + 0x1c000)
#define LPC_I2C1_BASE     (LPC_APB0_BASE + 0x5c000)
#define LPC_I2C2_BASE     (LPC_APB1_BASE + 0x20000)

#define LPC_SSP0_BASE     (LPC_APB1_BASE + 0x08000)
#define LPC_SSP1_BASE     (LPC_APB0_BASE + 0x30000)

#define LPC_TIM0_BASE     (LPC_APB0_BASE + 0x4000)
#define LPC_TIM1_BASE     (LPC_APB0_BASE + 0x8000)
#define LPC_TIM2_BASE     (LPC_APB1_BASE + 0x10000)
#define LPC_TIM3_BASE     (LPC_APB1_BASE + 0x14000)

#define LPC_UART0_BASE    (LPC_APB0_BASE + 0xc000)
#define LPC_UART1_BASE    (LPC_APB0_BASE + 0x10000)
#define LPC_UART2_BASE    (LPC_APB1_BASE + 0x18000)
#define LPC_UART3_BASE    (LPC_APB1_BASE + 0x1c000)

#define LPC_GPIOINT_BASE  (LPC_APB0_BASE + 0x28080)
#define LPC_PINCON_BASE   (LPC_APB0_BASE + 0x2c000)
#define LPC_RIT_BASE      (LPC_APB1_BASE + 0x30000)
#define LPC_RTC_BASE      (LPC_APB0_BASE + 0x24000)
#define LPC_SC_BASE       (LPC_APB1_BASE + 0x7c000)
#define LPC_WDT_BASE      (LPC_APB0_BASE)

#define LPC_GPDMA    ((struct LPC_GPDMA_Regs*)LPC_GPDMA_BASE)
#define LPC_GPDMACH0 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH0_BASE)
#define LPC_GPDMACH1 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH1_BASE)
#define LPC_GPDMACH2 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH2_BASE)
#define LPC_GPDMACH3 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH3_BASE)
#define LPC_GPDMACH4 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH4_BASE)
#define LPC_GPDMACH5 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH5_BASE)
#define LPC_GPDMACH6 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH6_BASE)
#define LPC_GPDMACH7 ((struct LPC_GPDMACH_Regs*)LPC_GPDMACH7_BASE)

#define LPC_GPIO0    ((struct LPC_GPIO_Regs*)LPC_GPIO0_BASE)
#define LPC_GPIO1    ((struct LPC_GPIO_Regs*)LPC_GPIO1_BASE)
#define LPC_GPIO2    ((struct LPC_GPIO_Regs*)LPC_GPIO2_BASE)
#define LPC_GPIO3    ((struct LPC_GPIO_Regs*)LPC_GPIO3_BASE)
#define LPC_GPIO4    ((struct LPC_GPIO_Regs*)LPC_GPIO4_BASE)

#define LPC_I2C0     ((struct LPC_I2C_Regs*)LPC_I2C0_BASE)
#define LPC_I2C1     ((struct LPC_I2C_Regs*)LPC_I2C1_BASE)
#define LPC_I2C2     ((struct LPC_I2C_Regs*)LPC_I2C2_BASE)

#define LPC_SSP0     ((struct LPC_SSP_Regs*)LPC_SSP0_BASE)
#define LPC_SSP1     ((struct LPC_SSP_Regs*)LPC_SSP1_BASE)

#define LPC_TIM0     ((struct LPC_TIMER_Regs*)LPC_TIM0_BASE)
#define LPC_TIM1     ((struct LPC_TIMER_Regs*)LPC_TIM1_BASE)
#define LPC_TIM2     ((struct LPC_TIMER_Regs*)LPC_TIM2_BASE)
#define LPC_TIM3     ((struct LPC_TIMER_Regs*)LPC_TIM3_BASE)

#define LPC_UART0    ((struct LPC_UART0_Regs*)LPC_UART0_BASE)
#define LPC_UART1    ((struct LPC_UART1_Regs*)LPC_UART1_BASE)
#define LPC_UART2    ((struct LPC_UART0_Regs*)LPC_UART2_BASE)
#define LPC_UART3    ((struct LPC_UART0_Regs*)LPC_UART3_BASE)

#define LPC_GPIOINT  ((struct LPC_GPIOINT_Regs*)LPC_GPIOINT_BASE)
#define LPC_PINCON   ((struct LPC_PINCON_Regs*)LPC_PINCON_BASE)
#define LPC_RIT      ((struct LPC_RIT_Regs*)LPC_RIT_BASE)
#define LPC_RTC      ((struct LPC_RTC_Regs*)LPC_RTC_BASE)
#define LPC_SC       ((struct LPC_SC_Regs*)LPC_SC_BASE)
#define LPC_WDT      ((struct LPC_WDT_Regs*)LPC_WDT_BASE)

#define SysTick      ((struct SysTick_Regs*)SYSTICK_BASE)
#define NVIC         ((struct NVIC_Regs*)NVIC_BASE)
#define SCB          ((struct SCB_Regs*)SCB_BASE)

typedef enum {
  NMI_IRQn = -14,
  HardFault_IRQn = -13,
  MemoryManage_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SVC_IRQn = -5,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,
  WDT_IRQn = 0,
  TIMER0_IRQn,
  TIMER1_IRQn,
  TIMER2_IRQn,
  TIMER3_IRQn,
  UART0_IRQn,
  UART1_IRQn,
  UART2_IRQn,
  UART3_IRQn,
  PWM1_IRQn,
  I2C0_IRQn,
  I2C1_IRQn,
  I2C2_IRQn,
  SPI_IRQn,
  SSP0_IRQn,
  SSP1_IRQn,
  PLL0_IRQn,
  RTC_IRQn,
  EINT0_IRQn,
  EINT1_IRQn,
  EINT2_IRQn,
  EINT3_IRQn,
  ADC_IRQn,
  BOD_IRQn,
  USB_IRQn,
  CAN_IRQn,
  DMA_IRQn,
  I2S_IRQn,
  ENET_IRQn,
  RIT_IRQn,
  MCPWM_IRQn,
  QEI_IRQn,
  PLL1_IRQn,
} LPCInterruptNum;


static inline void NVIC_EnableIRQ(LPCInterruptNum irqnum) {
  NVIC->ISER[irqnum / 32] = 1U << (irqnum & 31);
}

static inline void NVIC_DisableIRQ(LPCInterruptNum irqnum) {
  NVIC->ICER[irqnum / 32] = 1U << (irqnum & 31);
}

static inline void NVIC_SetPriority(LPCInterruptNum irqnum, uint32_t priority) {
  if (irqnum < 0) {
    SCB->SHP[(irqnum & 0xf) - 4] = priority << 3;
  } else {
    NVIC->IP[irqnum] = priority << 3;
  }
}

static inline void __WFI(void) {
  asm volatile ("wfi" ::: "memory");
}

static inline void __disable_irq(void) {
  asm volatile ("cpsid i");
}

static inline void __enable_irq(void) {
  asm volatile ("cpsie i");
}

#endif
