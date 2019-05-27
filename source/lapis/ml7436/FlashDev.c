/********************************************************************************/
/*  This file is part of the ARM Toolchain package                              */
/*  Copyright (c) 2013-2019 LAPIS Semiconductor Co., Ltd. All rights reserved.  */
/********************************************************************************/
/*                                                                              */
/*  FlashDev.C: Device Description for ML7436 512KB Flash                       */
/*    Version:1.00                                                              */
/*    Data   :2019.02.07                                                        */
/*                                                                              */
/********************************************************************************/
#include "FlashOS.h"        // FlashOS Structures

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "ML7436 512KB Flash",       // Device Name 
   ONCHIP,                     // Device Type
   0x00000000,                 // Device Start Address
   0x00080000,                 // Device Size in Bytes (512KB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   1000,                       // Program Page Timeout 1000 mSec
   1000,                       // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   {{0x00000400, 0x00000000},  // Sector Size  1024BByte
   {SECTOR_END}}
};
