/**************************************************************************//**
 * file     system_ML7436N.h
 * brief    CMSIS Cortex-M3 Device Peripheral Access Layer Header File for 
 *           ML7436N.
 * version  V0.07
 * date     04 Mar 2019
 *
 * note
 * Copyright (C) 2019 LAPIS Semiconductor Co., Ltd.  All rights reserved.
 *
 * par
 * This software is provided "as is" and any expressed or implied warranties,
 * including, but not limited to, the implied warranties of merchantability
 * and fitness for a particular purpose are disclaimed.
 * LAPIS Semiconductor shall not be liable for any direct, indirect,
 * consequential or incidental damages arising from using or modifying
 * this software.
 * You can modify and use this software in whole or part on
 * your own responsibility, only for the purpose of developing the software
 * for use with microcontroller manufactured by LAPIS Semiconductor.
 *
 ******************************************************************************/
#ifndef __SYSTEM_ML7436N_H
#define __SYSTEM_ML7436N_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

/**
 * Configure system clock
 *
 * @param  uint32_t sclk
 *             value for system clock setting
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
extern void syscon_configSYSCLK (uint32_t sclk);


#ifdef __cplusplus
}
#endif

#endif


