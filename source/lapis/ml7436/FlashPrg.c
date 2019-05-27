/********************************************************************************/
/*  This file is part of the ARM Toolchain package                              */
/*  Copyright (c) 2013-2018 LAPIS Semiconductor Co., Ltd. All rights reserved.  */
/********************************************************************************/
/*                                                                              */
/*  FlashPrg.C: Device Description for ML7436 512KB Flash                       */
/*    Version:1.00                                                              */
/*    Data   :2018.05.30                                                        */
/*                                                                              */
/********************************************************************************/

#include "FlashOS.h"        // FlashOS Structures
#include "ML7436N.h"

#define FLC_ACP_1st          (0x000000FA)
#define FLC_ACP_2nd          (0x000000F5)
#define FLC_ACP_lock         (0x00000000)
#define ERR_COUNTER_LIMIT    (0x00600000)
#define FLC_ERA_CHIP_ERASE   (0x00000001)
#define FLC_ERA_BLOCK_ERASE  (0x00000002)
#define FLC_ERA_SECTOR_ERASE (0x00000003)
//#define FLC_CTR_0_WAIT       (0x00000000)
//#define FLC_CTR_1_WAIT       (0x00000001)

#define __ERR_COUNTER_LIMIT  (0x00600000*3)		/* CPU‘¬“x‚ª2”{ˆÈã‘¬‚­‚È‚Á‚Ä‚¢‚é‚Ì‚Å3”{‚É‚µ‚½B */

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

//  UX_FLC0->FLCn_CTR = FLC_CTR_1_WAIT;
  UX_FLC0->FLCn_ACP = FLC_ACP_lock;

  return (0);                                  // Finished without Errors
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

  UX_FLC0->FLCn_ACP = FLC_ACP_lock;

  return (0);                                  // Finished without Errors
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {

  unsigned int err_counter=0;

  UX_FLC0->FLCn_ACP = FLC_ACP_1st;
  UX_FLC0->FLCn_ACP = FLC_ACP_2nd;

  /* start erase chip */
  UX_FLC0->FLCn_ERA = FLC_ERA_CHIP_ERASE;

  while(1)
  {
    if((UX_FLC0->FLCn_STA & 0x0000000f) == 0) break;  /*  ML7436_Add  status mask */
    err_counter++;
    if(err_counter > __ERR_COUNTER_LIMIT)
      return (1);
  }

  return (0);                                  // Finished without Errors
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

  unsigned int err_counter=0;

  UX_FLC0->FLCn_ACP = FLC_ACP_1st;
  UX_FLC0->FLCn_ACP = FLC_ACP_2nd;

  /* start erase sector */
  UX_FLC0->FLCn_ADR = adr;
  UX_FLC0->FLCn_ERA = FLC_ERA_SECTOR_ERASE;

  while(1)
  {
    if((UX_FLC0->FLCn_STA & 0x0000000f) == 0) break;  /*  ML7436_Add  status mask */
    err_counter++;
    if(err_counter > __ERR_COUNTER_LIMIT)
      return (1);
  }

  return (0);                                  // Finished without Errors
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  unsigned int err_counter;
  unsigned long *ptr=(unsigned long *)buf;

//AHB_GPIO->AHBGPIODATA=0;
//AHB_GPIO->AHBGPIODIR=1;
//AHB_GPIO->AHBGPIODATA=1;

  /* Adjust size */
  sz = sz + 0x03;
  sz = sz & (~0x03);

  while(sz > 0)
  {
    err_counter = 0;

    UX_FLC0->FLCn_ACP = FLC_ACP_1st;
    UX_FLC0->FLCn_ACP = FLC_ACP_2nd;

    UX_FLC0->FLCn_ADR = adr;

    /* start write */
    UX_FLC0->FLCn_WDA = *ptr++;
   
    while(1)
    {
      if((UX_FLC0->FLCn_STA & 0x0000000f) == 0) break;  /*  ML7436_Add  status mask */
      err_counter++;
      if(err_counter > __ERR_COUNTER_LIMIT)
        return (1);
    }

    /* next write */
    adr += 4;
    sz -= 4;
  }

//AHB_GPIO->AHBGPIODATA=0;

  return (0);                                  // Finished without Errors
}
