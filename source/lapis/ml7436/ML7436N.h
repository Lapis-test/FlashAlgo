
/****************************************************************************************************//**
 * @file     ML7436N.h
 *
 * @brief    CMSIS Cortex-M3 Peripheral Access Layer Header File for
 *           ML7436N from LAPIS Semiconductor Co.,Ltd..
 *
 * @version  V0.07
 * @date     4. March 2019
 *
 * @note     Generated with SVDConv V2.83a 
 *           from CMSIS SVD File 'ML7436N.svd' Version 0.07,
 *
 * @par      This software is provided "as is" and any expressed or implied
 *           warranties, including, but not limited to, the implied warranties of
 *           merchantability and fitness for a particular purpose are disclaimed.
 *           LAPIS Semiconductor shall not be liable for any direct, indirect, consequential or 
 *           incidental damages arising from using or modifying this software.
 *           You (customer) can modify and use this software in whole or part on
 *           your own responsibility, only for the purpose of developing the software
 *           for use with microcontroller manufactured by LAPIS Semiconductor.
 *           
 *
 *******************************************************************************************************/



/** @addtogroup LAPIS Semiconductor Co.,Ltd.
  * @{
  */

/** @addtogroup ML7436N
  * @{
  */

#ifndef ML7436N_H
#define ML7436N_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M3 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ---------------------  ML7436N Specific Interrupt Numbers  --------------------- */
  WDT_IRQn                      =   0,              /*!<   0  WDT                                                              */
  EXTINT_IRQn                   =   1,              /*!<   1  EXTINT                                                           */
  TMRA_TIMER1_IRQn              =   3,              /*!<   3  TMRA_TIMER1                                                      */
  RTC_IRQn                      =   5,              /*!<   5  RTC                                                              */
  TMRB_TIMER1_IRQn              =   6,              /*!<   6  TMRB_TIMER1                                                      */
  TMRC_TIMER1_IRQn              =   7,              /*!<   7  TMRC_TIMER1                                                      */
  FTMA_FTM0_IRQn                =   8,              /*!<   8  FTMA_FTM0                                                        */
  UART0_IRQn                    =  10,              /*!<  10  UART0                                                            */
  ADC_CNT0_IRQn                 =  12,              /*!<  12  ADC_CNT0                                                         */
  AES_IRQn                      =  13,              /*!<  13  AES                                                              */
  UART1_IRQn                    =  14,              /*!<  14  UART1                                                            */
  UART2_IRQn                    =  15,              /*!<  15  UART2                                                            */
  TMRD_TIMER1_IRQn              =  16,              /*!<  16  TMRD_TIMER1                                                      */
  FLASH_CNT0_IRQn               =  17,              /*!<  17  FLASH_CNT0                                                       */
  I2C0_IRQn                     =  20,              /*!<  20  I2C0                                                             */
  DMAC0_IRQn                    =  21,              /*!<  21  DMAC0                                                            */
  SPI0_IRQn                     =  22,              /*!<  22  SPI0                                                             */
  SPI1_IRQn                     =  23,              /*!<  23  SPI1                                                             */
  SPI2_IRQn                     =  26,              /*!<  26  SPI2                                                             */
  DIO0_IRQn                     =  27,              /*!<  27  DIO0                                                             */
  LVD_CNT0_IRQn                 =  28,              /*!<  28  LVD_CNT0                                                         */
  CLK_TIMER0_IRQn               =  30,              /*!<  30  CLK_TIMER0                                                       */
  MODE0_IRQn                    =  31,              /*!<  31  MODE0                                                            */
  I2C1_IRQn                     =  32,              /*!<  32  I2C1                                                             */
  DMAC1_IRQn                    =  34,              /*!<  34  DMAC1                                                            */
  FTMB_FTM0_IRQn                =  35,              /*!<  35  FTMB_FTM0                                                        */
  FTMC_FTM0_IRQn                =  36,              /*!<  36  FTMC_FTM0                                                        */
  FTMD_FTM0_IRQn                =  37,              /*!<  37  FTMD_FTM0                                                        */
  FTME_FTM0_IRQn                =  38,              /*!<  38  FTME_FTM0                                                        */
  FTMF_FTM0_IRQn                =  39,              /*!<  39  FTMF_FTM0                                                        */
  ECC_SRAM_IRQn                 =  40,              /*!<  40  ECC_SRAM                                                         */
  HASH_IRQn                     =  41,              /*!<  41  HASH                                                             */
  TRNG_IRQn                     =  42               /*!<  42  TRNG                                                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M3 Processor and Core Peripherals---------------- */
#define __CM3_REV                 0x0201            /*!< Cortex-M3 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               2            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm3.h"                               /*!< Cortex-M3 processor and core peripherals                              */
#include "system_ML7436N.h"                         /*!< ML7436N System                                                        */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                    UX_SYSCON                   ================ */
/* ================================================================================ */


/**
  * @brief System control (UX_SYSCON)
  */

typedef struct {                                    /*!< UX_SYSCON Structure                                                   */
  __I  uint32_t  SYSCON_ID0;                        /*!< Show device information 0.                                            */
  __I  uint32_t  SYSCON_ID1;                        /*!< Show device information 1.                                            */
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  SYSCON_REMAP_CON;                  /*!< Control REMAP.                                                        */
  __IO uint32_t  SYSCON_REMAP_BASE;                 /*!< Control REMAP.                                                        */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  SYSCON_CPU_CON;                    /*!< Operation when the lock-up of CPU occurs is set up.                   */
  __IO uint32_t  SYSCON_CPU_ST;                     /*!< Show CPU State.                                                       */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  SYSCON_STCALIB;                    /*!< Adjust clock of a SysTick timer.                                      */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  SYSCON_PCLK_EN;                    /*!< The clock supply to each peripheral is enabled.                       */
  __IO uint32_t  SYSCON_PCLK_DIS;                   /*!< The clock supply to each peripheral is disabled.                      */
  __IO uint32_t  SYSCON_PPM1;                       /*!< Peripheral power management on SLEEPDEEP.                             */
  __IO uint32_t  SYSCON_PPM2;                       /*!< Peripheral power management on SLEEP.                                 */
  __IO uint32_t  SYSCON_PRST_CON;                   /*!< Reset control of each peripheral.                                     */
  __I  uint32_t  RESERVED4[3];
  __IO uint32_t  SYSCON_EPCLK_EN;                   /*!< The clock supply to each Ex-peripheral is enabled.                    */
  __IO uint32_t  SYSCON_EPCLK_DIS;                  /*!< The clock supply to each Ex-peripheral is disabled.                   */
  __IO uint32_t  SYSCON_EPPM1;                      /*!< Ex-Peripheral power management on SLEEPDEEP.                          */
  __IO uint32_t  SYSCON_EPPM2;                      /*!< Ex-Peripheral power management on SLEEP.                              */
  __IO uint32_t  SYSCON_EPRST_CON;                  /*!< Reset control of each Ex-peripheral.                                  */
  __I  uint32_t  RESERVED5[3];
  __IO uint32_t  SYSCON_SPCLK_EN;                   /*!< The clock supply to each Secure-peripheral is enabled.                */
  __IO uint32_t  SYSCON_SPCLK_DIS;                  /*!< The clock supply to each Secure-peripheral is disabled.               */
  __IO uint32_t  SYSCON_SPPM1;                      /*!< Secure-Peripheral power management on SLEEPDEEP.                      */
  __IO uint32_t  SYSCON_SPPM2;                      /*!< Secure-Peripheral power management on SLEEP.                          */
  __IO uint32_t  SYSCON_SPRST_CON;                  /*!< Reset control of each Secure-peripheral.                              */
} UX_SYSCON_Type;


/* ================================================================================ */
/* ================                     UX_FLCn                    ================ */
/* ================================================================================ */


/**
  * @brief Flash-ROM Controller (UX_FLCn)
  */

typedef struct {                                    /*!< UX_FLCn Structure                                                     */
  __I  uint32_t  FLCn_STA;                          /*!< Flash-ROM status                                                      */
  __O  uint32_t  FLCn_ACP;                          /*!< Flash-ROM accepter                                                    */
  __IO uint32_t  FLCn_ADR;                          /*!< Flash-ROM address                                                     */
  __O  uint32_t  FLCn_WDA;                          /*!< Flash-ROM write data                                                  */
  __IO uint32_t  FLCn_ERA;                          /*!< Flash-ROM erase                                                       */
  __I  uint32_t  FLCn_CTR;                          /*!< Flash-ROM control                                                     */
  __I  uint32_t  RESERVED0;
  __I  uint32_t  FLCn_PSTA;                         /*!< Flash-ROM Protect status                                              */
  __I  uint32_t  FLCn_RSIZ;                         /*!< Flash-ROM size                                                        */
  __I  uint32_t  FLCn_BADR;                         /*!< boot program address                                                  */
  __IO uint32_t  FLCn_INTMSK;                       /*!< Interrupt Mask                                                        */
  __IO uint32_t  FLCn_INTSTA;                       /*!< Interrupt Status                                                      */
  __IO uint32_t  FLCn_RINTSTA;                      /*!< Raw interrupt status                                                  */
  __I  uint32_t  RESERVED1[36];
  __IO uint32_t  FLCn_ERRSTA;                       /*!< Error status                                                          */
  __I  uint32_t  RESERVED2[2];
  __O  uint32_t  FLCn_PROTUNLOCK;                   /*!< Protect UnLock                                                        */
  __I  uint32_t  FLCn_PROTLOCKSTA;                  /*!< Protect Lock status                                                   */
  __O  uint32_t  FLCn_PROTWDA;                      /*!< Protect Set Write Data                                                */
  __IO uint32_t  FLCn_PROTERA;                      /*!< Protect Set Erase                                                     */
  __I  uint32_t  RESERVED3[8];
  __I  uint32_t  FLCn_CONFIG;                       /*!< Flash Config                                                          */
  __I  uint32_t  RESERVED4[3];
  __I  uint32_t  FLCn_CU0ADR;                       /*!< Code Usr app0 address                                                 */
  __I  uint32_t  FLCn_CP0ADR;                       /*!< Code Protect Set0 address                                             */
  __I  uint32_t  FLCn_CB0ADR;                       /*!< Code Boot Program0 address                                            */
  __I  uint32_t  FLCn_C0SIZ;                        /*!< Code Area0 size                                                       */
  __I  uint32_t  FLCn_DU0ADR;                       /*!< Data Usr app0 address                                                 */
  __I  uint32_t  FLCn_DP0ADR;                       /*!< Data Protect Set0 address                                             */
  __I  uint32_t  FLCn_DB0ADR;                       /*!< Data Boot Program0 address                                            */
  __I  uint32_t  FLCn_D0SIZ;                        /*!< Data Area0 size                                                       */
  __I  uint32_t  RESERVED5[4];
  __I  uint32_t  FLCn_CU1ADR;                       /*!< Code Usr app1 address                                                 */
  __I  uint32_t  FLCn_CP1ADR;                       /*!< Code Protect Set1 address                                             */
  __I  uint32_t  FLCn_CB1ADR;                       /*!< Code Boot Program1 address                                            */
  __I  uint32_t  FLCn_C1SIZ;                        /*!< Code Area1 size                                                       */
  __I  uint32_t  FLCn_DU1ADR;                       /*!< Data Usr app1 address                                                 */
  __I  uint32_t  FLCn_DP1ADR;                       /*!< Data Protect Set1 address                                             */
  __I  uint32_t  FLCn_DB1ADR;                       /*!< Data Boot Program1 address                                            */
  __I  uint32_t  FLCn_D1SIZ;                        /*!< Data Area1 size                                                       */
} UX_FLCn_Type;


/* ================================================================================ */
/* ================                 UX_TMRm_TIMERn                 ================ */
/* ================================================================================ */


/**
  * @brief 32bit Timer (UX_TMRm_TIMERn)
  */

typedef struct {                                    /*!< UX_TMRm_TIMERn Structure                                              */
  __IO uint32_t  TMRm_TIMERnLOADCOUNT;              /*!< Value to be loaded into TIMERn                                        */
  __I  uint32_t  TMRm_TIMERnCURRENTVALUE;           /*!< Current Value of TIMERn                                               */
  __IO uint32_t  TMRm_TIMERnCONTROLREG;             /*!< Control Register for TIMERn                                           */
  __I  uint32_t  TMRm_TIMERnEOI;                    /*!< Clears the interrupt from TIMERn                                      */
  __I  uint32_t  TMRm_TIMERnINTSTATUS;              /*!< Contains the interrupt status for TIMERn                              */
  __I  uint32_t  RESERVED0[37];
  __I  uint32_t  TMRm_TIMERSRAWINTSTATUS;           /*!< Timers Raw Interrupt Status Register                                  */
} UX_TMRm_TIMERn_Type;


/* ================================================================================ */
/* ================                  UX_FTMm_FTMn                  ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible Timer (UX_FTMm_FTMn)
  */

typedef struct {                                    /*!< UX_FTMm_FTMn Structure                                                */
  __IO uint32_t  FTMm_FTMnCON;                      /*!< Configure Timer n                                                     */
  __IO uint32_t  FTMm_FTMnST;                       /*!< Timer n status                                                        */
  __IO uint32_t  FTMm_FTMnC;                        /*!< Timer n counter                                                       */
  __IO uint32_t  FTMm_FTMnR;                        /*!< Timer n reload value                                                  */
  __IO uint32_t  FTMm_FTMnGR;                       /*!< Timer n general register                                              */
  __IO uint32_t  FTMm_FTMnIOLV;                     /*!< Timer n I/O level register                                            */
  __IO uint32_t  FTMm_FTMnOUT;                      /*!< Timer n output register                                               */
  __IO uint32_t  FTMm_FTMnIER;                      /*!< Timer n interrupt enable                                              */
  __IO uint32_t  FTMm_FTMnCKCON;                    /*!< Timer n clock control                                                 */
} UX_FTMm_FTMn_Type;


/* ================================================================================ */
/* ================                     UX_FTMA                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerA (UX_FTMA)
  */

typedef struct {                                    /*!< UX_FTMA Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTMA_Type;


/* ================================================================================ */
/* ================                     UX_FTMB                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerB (UX_FTMB)
  */

typedef struct {                                    /*!< UX_FTMB Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTMB_Type;


/* ================================================================================ */
/* ================                     UX_FTMC                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerC (UX_FTMC)
  */

typedef struct {                                    /*!< UX_FTMC Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTMC_Type;


/* ================================================================================ */
/* ================                     UX_FTMD                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerD (UX_FTMD)
  */

typedef struct {                                    /*!< UX_FTMD Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTMD_Type;


/* ================================================================================ */
/* ================                     UX_FTME                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerE (UX_FTME)
  */

typedef struct {                                    /*!< UX_FTME Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTME_Type;


/* ================================================================================ */
/* ================                     UX_FTMF                    ================ */
/* ================================================================================ */


/**
  * @brief 32bit Flexible TimerF (UX_FTMF)
  */

typedef struct {                                    /*!< UX_FTMF Structure                                                     */
  __IO uint32_t  FTMm_FTMEN;                        /*!< Timer enable                                                          */
  __O  uint32_t  FTMm_FTMDIS;                       /*!< Timer disable                                                         */
} UX_FTMF_Type;


/* ================================================================================ */
/* ================                     UX_RTC                     ================ */
/* ================================================================================ */


/**
  * @brief Real Time Clock (UX_RTC)
  */

typedef struct {                                    /*!< UX_RTC Structure                                                      */
  __IO uint32_t  RTC_S1;                            /*!< 1-second unit register                                                */
  __IO uint32_t  RTC_S10;                           /*!< 10-second unit register                                               */
  __IO uint32_t  RTC_MI1;                           /*!< 1-minute unit register                                                */
  __IO uint32_t  RTC_MI10;                          /*!< 10-minute unit register                                               */
  __IO uint32_t  RTC_H1;                            /*!< 1-hour unit register                                                  */
  __IO uint32_t  RTC_H10;                           /*!< PM/AM 10-hour unit register                                           */
  __IO uint32_t  RTC_D1;                            /*!< 1-day unit register                                                   */
  __IO uint32_t  RTC_D10;                           /*!< 10-day unit register                                                  */
  __IO uint32_t  RTC_MO1;                           /*!< 1-month unit register                                                 */
  __IO uint32_t  RTC_MO10;                          /*!< 10-month unit register                                                */
  __IO uint32_t  RTC_Y1;                            /*!< 1-year unit register                                                  */
  __IO uint32_t  RTC_Y10;                           /*!< 10-year unit register                                                 */
  __IO uint32_t  RTC_W;                             /*!< Week register                                                         */
  __IO uint32_t  RTC_CD;                            /*!< RTC Control                                                           */
  __IO uint32_t  RTC_CE;                            /*!< RTC Control                                                           */
  __IO uint32_t  RTC_CF;                            /*!< RTC Control                                                           */
  __IO uint32_t  RTC_FT_S1;                         /*!< Designate 1-second unit register                                      */
  __IO uint32_t  RTC_FT_S10;                        /*!< Designate 10-second unit register                                     */
  __IO uint32_t  RTC_FT_MI1;                        /*!< Designate 1-minute unit register                                      */
  __IO uint32_t  RTC_FT_MI10;                       /*!< Designate 10-minute unit register                                     */
  __IO uint32_t  RTC_FT_H1;                         /*!< Designate 1-hour unit register                                        */
  __IO uint32_t  RTC_FT_H10;                        /*!< Designate PM/AM 10-hour unit register                                 */
  __IO uint32_t  RTC_FT_D1;                         /*!< Designate 1-day unit register                                         */
  __IO uint32_t  RTC_FT_D10;                        /*!< Designate 10-day unit register                                        */
  __IO uint32_t  RTC_FT_MO1;                        /*!< Designate 1-month unit register                                       */
  __IO uint32_t  RTC_FT_MO10;                       /*!< Designate 10-month unit register                                      */
  __IO uint32_t  RTC_FT_Y1;                         /*!< Designate 1-year unit register                                        */
  __IO uint32_t  RTC_FT_Y10;                        /*!< Designate 10-year unit register                                       */
} UX_RTC_Type;


/* ================================================================================ */
/* ================                    UX_UARTn                    ================ */
/* ================================================================================ */


/**
  * @brief Standard 16550 UART. (UX_UARTn)
  */

typedef struct {                                    /*!< UX_UARTn Structure                                                    */
  
  union {
    __IO uint32_t  UARTn_DLL;                       /*!< Low value of baud rate divisor.                                       */
    __O  uint32_t  UARTn_THR;                       /*!< Transmit Holding Register                                             */
    __I  uint32_t  UARTn_RBR;                       /*!< Receive Buffer Register                                               */
  };
  
  union {
    __IO uint32_t  UARTn_DLH;                       /*!< High value of baud rate divisor.                                      */
    __IO uint32_t  UARTn_IER;                       /*!< Interrupt enable register                                             */
  };
  
  union {
    __O  uint32_t  UARTn_FCR;                       /*!< FIFO control register                                                 */
    __I  uint32_t  UARTn_IIR;                       /*!< Interrupt identification register                                     */
  };
  __IO uint32_t  UARTn_LCR;                         /*!< Line Control Register                                                 */
  __IO uint32_t  UARTn_MCR;                         /*!< MODEM control register                                                */
  __I  uint32_t  UARTn_LSR;                         /*!< Line Status Register                                                  */
  __I  uint32_t  UARTn_MSR;                         /*!< MODEM status register                                                 */
  __IO uint32_t  UARTn_SCR;                         /*!< Scratchpad Register                                                   */
  __IO uint32_t  UARTn_LPDLL;                       /*!< Low Power Divisor Latch Low Register                                  */
  __IO uint32_t  UARTn_LPDLH;                       /*!< Low Power Divisor Latch High Register                                 */
  __I  uint32_t  RESERVED0[2];
  
  union {
    __O  uint32_t  UARTn_STHR;                      /*!< Shadow Transmit Holding Register                                      */
    __I  uint32_t  UARTn_SRBR;                      /*!< Shadow Receive Buffer Register                                        */
  };
  __I  uint32_t  RESERVED1[15];
  __IO uint32_t  UARTn_FAR;                         /*!< FIFO Access Register                                                  */
  __I  uint32_t  UARTn_TFR;                         /*!< Transmit FIFO Read                                                    */
  __O  uint32_t  UARTn_RFW;                         /*!< Receive FIFO Write                                                    */
  __I  uint32_t  UARTn_USR;                         /*!< UART Status Register                                                  */
  __I  uint32_t  UARTn_TFL;                         /*!< Transmit FIFO Level                                                   */
  __I  uint32_t  UARTn_RFL;                         /*!< Receive FIFO Level                                                    */
  __O  uint32_t  UARTn_SRR;                         /*!< Software Reset Register                                               */
  __IO uint32_t  UARTn_SRTS;                        /*!< Shadow Request to Send                                                */
  __IO uint32_t  UARTn_SBCR;                        /*!< Shadow Break Control Register                                         */
  __IO uint32_t  UARTn_SDMAM;                       /*!< Shadow DMA Mode                                                       */
  __IO uint32_t  UARTn_SFE;                         /*!< Shadow FIFO Enable                                                    */
  __IO uint32_t  UARTn_SRT;                         /*!< Shadow RCVR Trigger                                                   */
  __IO uint32_t  UARTn_STET;                        /*!< Shadow TX Empty Trigger                                               */
  __IO uint32_t  UARTn_HTX;                         /*!< Halt TX                                                               */
  __O  uint32_t  UARTn_DMASA;                       /*!< DMA Software Acknowledge                                              */
  __I  uint32_t  RESERVED2[18];
  __I  uint32_t  UARTn_CPR;                         /*!< Component Parameter Register                                          */
} UX_UARTn_Type;


/* ================================================================================ */
/* ================                     UX_I2Cn                    ================ */
/* ================================================================================ */


/**
  * @brief Two-wire serial interface (UX_I2Cn)
  */

typedef struct {                                    /*!< UX_I2Cn Structure                                                     */
  __IO uint32_t  I2Cn_CON;                          /*!< I2C Control Register                                                  */
  __IO uint32_t  I2Cn_TAR;                          /*!< I2C Target Address Register                                           */
  __IO uint32_t  I2Cn_SAR;                          /*!< I2C Slave Address Register                                            */
  __IO uint32_t  I2Cn_HS_MADDR;                     /*!< High speed mode master code address Register                          */
  __IO uint32_t  I2Cn_DATA_CMD;                     /*!< I2C Rx/Tx Data Buffer and Command Register                            */
  __IO uint32_t  I2Cn_SS_SCL_HCNT;                  /*!< Standard Speed I2C Clock SCL High Count Register                      */
  __IO uint32_t  I2Cn_SS_SCL_LCNT;                  /*!< Standard Speed I2C Clock SCL Low Count Register                       */
  __IO uint32_t  I2Cn_FS_SCL_HCNT;                  /*!< Fast Speed I2C Clock SCL High Count Register                          */
  __IO uint32_t  I2Cn_FS_SCL_LCNT;                  /*!< Fast Speed I2C Clock SCL Low Count Register                           */
  __IO uint32_t  I2Cn_HS_SCL_HCNT;                  /*!< High Speed I2C Clock SCL High Count Register                          */
  __IO uint32_t  I2Cn_HS_SCL_LCNT;                  /*!< High Speed I2C Clock SCL Low Count Register                           */
  __I  uint32_t  I2Cn_INTR_STAT;                    /*!< I2C Interrupt Status Register                                         */
  __IO uint32_t  I2Cn_INTR_MASK;                    /*!< I2C Interrupt Mask Register                                           */
  __I  uint32_t  I2Cn_RAW_INTR_STAT;                /*!< I2C Raw Interrupt Status Register                                     */
  __IO uint32_t  I2Cn_RX_TL;                        /*!< I2C Receive FIFO Threshold Register                                   */
  __IO uint32_t  I2Cn_TX_TL;                        /*!< I2C Transmit FIFO Threshold Register                                  */
  __I  uint32_t  I2Cn_CLR_INTR;                     /*!< Clear Combined and Individual Interrupt Register                      */
  __I  uint32_t  I2Cn_CLR_RX_UNDER;                 /*!< Clear RX_UNDER Interrupt Register                                     */
  __I  uint32_t  I2Cn_CLR_RX_OVER;                  /*!< Clear RX_OVER Interrupt Register                                      */
  __I  uint32_t  I2Cn_CLR_TX_OVER;                  /*!< Clear TX_OVER Interrupt Register                                      */
  __I  uint32_t  I2Cn_CLR_RD_REQ;                   /*!< Clear RD_REQ Interrupt Register                                       */
  __I  uint32_t  I2Cn_CLR_TX_ABRT;                  /*!< Clear TX_ABRT Interrupt Register                                      */
  __I  uint32_t  I2Cn_CLR_RX_DONE;                  /*!< Clear RX_DONE Interrupt Register                                      */
  __I  uint32_t  I2Cn_CLR_ACTIVITY;                 /*!< Clear ACTIVITY Interrupt Register                                     */
  __I  uint32_t  I2Cn_CLR_STOP_DET;                 /*!< Clear STOP_DET Interrupt Register                                     */
  __I  uint32_t  I2Cn_CLR_START_DET;                /*!< Clear START_DET Interrupt Register                                    */
  __I  uint32_t  I2Cn_CLR_GEN_CALL;                 /*!< Clear GEN_CALL Interrupt Register                                     */
  __IO uint32_t  I2Cn_ENABLE;                       /*!< I2C Enable Register                                                   */
  __I  uint32_t  I2Cn_STATUS;                       /*!< I2C Status Register                                                   */
  __I  uint32_t  I2Cn_TXFLR;                        /*!< I2C Transmit FIFO Level Register                                      */
  __I  uint32_t  I2Cn_RXFLR;                        /*!< I2C Receive FIFO Level Register                                       */
  __IO uint32_t  I2Cn_SDA_HOLD;                     /*!< I2C SDA Hold Time Length Register                                     */
  __I  uint32_t  I2Cn_TX_ABRT_SOURCE;               /*!< I2C Transmit Abort Source Register                                    */
  __IO uint32_t  I2Cn_SLV_DATA_NACK_ONLY;           /*!< Generate Slave Data NACK Register                                     */
  __IO uint32_t  I2Cn_DMA_CR;                       /*!< I2C DMA Control Register                                              */
  __IO uint32_t  I2Cn_DMA_TDLR;                     /*!< I2C DMA TX Data Level Register                                        */
  __IO uint32_t  I2Cn_DMA_RDLR;                     /*!< I2C DMA RX Data Level Register                                        */
  __IO uint32_t  I2Cn_SDA_SETUP;                    /*!< I2C SDA Setup Register                                                */
  __IO uint32_t  I2Cn_ACK_GENERAL_CALL;             /*!< I2C ACK General Call Register                                         */
  __I  uint32_t  I2Cn_ENABLE_STATUS;                /*!< I2C Enable Status Register                                            */
  __IO uint32_t  I2Cn_FS_SPKLEN;                    /*!< I2C SS and FS Spike Suppression Limit Register                        */
  __IO uint32_t  I2Cn_HS_SPKLEN;                    /*!< I2C HS Spike Suppression Limit Register                               */
  __I  uint32_t  I2Cn_CLR_RESTART_DET;              /*!< Read this register to clear the RESTART_DET interrupt.                */
  __IO uint32_t  I2Cn_SCL_STUCK_AT_LOW_TIMEOUT;     /*!< SCL Detect time of L stack                                            */
  __IO uint32_t  I2Cn_SDA_STUCK_AT_LOW_TIMEOUT;     /*!< SDA Detect time of L stack                                            */
  __I  uint32_t  I2Cn_CLR_SCL_STUCK_DET;            /*!< Read this register to clear the RESTART_DET interrupt.                */
} UX_I2Cn_Type;


/* ================================================================================ */
/* ================                    UX_GPIOm                    ================ */
/* ================================================================================ */


/**
  * @brief General Purpose Programming I/O (UX_GPIOm)
  */

typedef struct {                                    /*!< UX_GPIOm Structure                                                    */
  __IO uint32_t  GPIOn_SWPORTA_DR;                  /*!< Port A Data Register                                                  */
  __IO uint32_t  GPIOn_SWPORTA_DDR;                 /*!< Port A Data Direction Register                                        */
  __I  uint32_t  RESERVED0[16];
  __IO uint32_t  GPIOn_DEBOUNCE;                    /*!< Port A Debounce enable                                                */
  __I  uint32_t  RESERVED1;
  __I  uint32_t  GPIOn_EXT_PORTA;                   /*!< External Port A                                                       */
} UX_GPIOm_Type;


/* ================================================================================ */
/* ================                     EXTINT                     ================ */
/* ================================================================================ */


/**
  * @brief Extra interrupt module for GPIO (EXTINT)
  */

typedef struct {                                    /*!< EXTINT Structure                                                      */
  __I  uint32_t  RESERVED0[12];
  __IO uint32_t  EXTINTn_INTEN;                     /*!< Port A Interrupt enable                                               */
  __IO uint32_t  EXTINTn_INTMASK;                   /*!< Port A Interrupt mask                                                 */
  __IO uint32_t  EXTINTn_INTTYPE_LEVEL;             /*!< Port A Interrupt level                                                */
  __IO uint32_t  EXTINTn_INT_POLARITY;              /*!< Port A Interrupt polarity                                             */
  __I  uint32_t  EXTINTn_INTSTATUS;                 /*!< Port A Interrupt status                                               */
  __I  uint32_t  EXTINTn_RAW_INTSTATUS;             /*!< Port A Raw interrupt status                                           */
  __IO uint32_t  EXTINTn_DEBOUNCE;                  /*!< Port A Debounce enable                                                */
  __O  uint32_t  EXTINTn_PORTA_EOI;                 /*!< Port A Clear interrupt                                                */
  __I  uint32_t  EXTINTn_EXT_PORTA;                 /*!< External Port A                                                       */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  EXTINTn_LS_SYNC;                   /*!< Port A Synchronization level                                          */
  __I  uint32_t  RESERVED2[7];
  __IO uint32_t  EXTINTn_PORT_SEL0;                 /*!< Port sorce selection 0                                                */
  __IO uint32_t  EXTINTn_PORT_SEL1;                 /*!< Port sorce selection 1                                                */
} EXTINT_Type;


/* ================================================================================ */
/* ================                     UX_WDT                     ================ */
/* ================================================================================ */


/**
  * @brief 32bit Watch Dog Timer (UX_WDT)
  */

typedef struct {                                    /*!< UX_WDT Structure                                                      */
  __IO uint32_t  WDT_CR;                            /*!< Control Register                                                      */
  __IO uint32_t  WDT_TORR;                          /*!< Timeout Range Register                                                */
  __I  uint32_t  WDT_CCVR;                          /*!< Current Counter Value Register                                        */
  __O  uint32_t  WDT_CRR;                           /*!< Counter Restart Register                                              */
  __I  uint32_t  WDT_STAT;                          /*!< Interrupt Status Register                                             */
  __I  uint32_t  WDT_EOI;                           /*!< Interrupt Clear Register                                              */
} UX_WDT_Type;


/* ================================================================================ */
/* ================                   UX_STDPORTm                  ================ */
/* ================================================================================ */


/**
  * @brief Port Configuration (UX_STDPORTm)
  */

typedef struct {                                    /*!< UX_STDPORTm Structure                                                 */
  __IO uint32_t  STDPORTm_SEL2;                     /*!< 2nd function selection register                                       */
  __IO uint32_t  STDPORTm_SEL3;                     /*!< 3rd function selection register                                       */
  __IO uint32_t  STDPORTm_SEL4;                     /*!< 4th function selection register                                       */
  __IO uint32_t  STDPORTm_SEL5;                     /*!< 5th function selection register                                       */
  __IO uint32_t  STDPORTm_SEL6;                     /*!< 6th function selection register                                       */
  __IO uint32_t  STDPORTm_SEL7;                     /*!< 7th function selection register                                       */
} UX_STDPORTm_Type;


/* ================================================================================ */
/* ================                     UX_SPIn                    ================ */
/* ================================================================================ */


/**
  * @brief Synchronous Serial Interface (UX_SPIn)
  */

typedef struct {                                    /*!< UX_SPIn Structure                                                     */
  __IO uint32_t  SPIn_CR;                           /*!< SPI Control Register                                                  */
  __IO uint32_t  SPIn_BRR;                          /*!< SPI Baud Rate control Register                                        */
  __IO uint32_t  SPIn_SR;                           /*!< SPI Status Register                                                   */
  __IO uint32_t  SPIn_DWR;                          /*!< SPI Data Write Register                                               */
  __I  uint32_t  SPIn_DRR;                          /*!< SPI Data Read Register                                                */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  SPIn_DMA;                          /*!< SPI DMA register                                                      */
} UX_SPIn_Type;


/* ================================================================================ */
/* ================                      DIOn                      ================ */
/* ================================================================================ */


/**
  * @brief DIO (DIOn)
  */

typedef struct {                                    /*!< DIOn Structure                                                        */
  __IO uint32_t  DIOn_EN;                           /*!< DIO Enable                                                            */
  __IO uint32_t  DIOn_CTRL;                         /*!< DIO Control                                                           */
  __IO uint32_t  DIOn_MATCH_LEN;                    /*!< RX data match pattern length                                          */
  __IO uint32_t  DIOn_MATCH_PAT;                    /*!< RX data match pattern                                                 */
  __IO uint32_t  DIOn_F_FULL_TRG;                   /*!< FIFO threshold                                                        */
  __IO uint32_t  DIOn_F_EMPTY_TRG;                  /*!< FIFO empty threshold                                                  */
  __I  uint32_t  DIOn_F_LEV;                        /*!< FIFO valid data count                                                 */
  __IO uint32_t  DIOn_IMSK;                         /*!< DIO Interrupt Mask                                                    */
  __I  uint32_t  DIOn_IST;                          /*!< DIO Interrupt Status                                                  */
  __I  uint32_t  DIOn_RIST;                         /*!< DIO RAW Interrupt Status                                              */
  __I  uint32_t  DIOn_F_O_CLR;                      /*!< FIFO over flow interrupt clear                                        */
  __I  uint32_t  DIOn_F_U_CLR;                      /*!< FIFO under flow interrupt clear                                       */
  __I  uint32_t  DIOn_MATCH_CLR;                    /*!< Match Data interrupt clear                                            */
  __I  uint32_t  DIOn_ICLR;                         /*!< interrupt clear                                                       */
  __IO uint32_t  DIOn_FIFO;                         /*!< FIFO Data                                                             */
} DIOn_Type;


/* ================================================================================ */
/* ================                      TRNG                      ================ */
/* ================================================================================ */


/**
  * @brief True Random Number Generator (TRNG)
  */

typedef struct {                                    /*!< TRNG Structure                                                        */
  __IO uint32_t  TRNG_CTRL;                         /*!< MODE selection                                                        */
  __I  uint32_t  TRNG_DATA0;                        /*!< TRNG DATA0                                                            */
  __I  uint32_t  TRNG_DATA1;                        /*!< TRNG DATA1                                                            */
  __IO uint32_t  TRNG_INTMASK;                      /*!< Interrupt mask                                                        */
  __IO uint32_t  TRNG_INTSTATUS;                    /*!< Interrupt status                                                      */
  __IO uint32_t  TRNG_RAWINTSTATUS;                 /*!< Interrupt raw status                                                  */
} TRNG_Type;


/* ================================================================================ */
/* ================                   CLK_TIMERn                   ================ */
/* ================================================================================ */


/**
  * @brief Clock timer (CLK_TIMERn)
  */

typedef struct {                                    /*!< CLK_TIMERn Structure                                                  */
  __IO uint32_t  CT_CR;                             /*!< Clock timer control                                                   */
  __IO uint32_t  CT_TR;                             /*!< Clock timer                                                           */
  __I  uint32_t  CT_SR;                             /*!< Clock timer status                                                    */
  __I  uint32_t  CT_TCR;                            /*!< Clock timer count                                                     */
  __I  uint32_t  CT_TCL;                            /*!< Clock timer clear                                                     */
} CLK_TIMERn_Type;


/* ================================================================================ */
/* ================                    MODE_CNTn                   ================ */
/* ================================================================================ */


/**
  * @brief Mode Control (MODE_CNTn)
  */

typedef struct {                                    /*!< MODE_CNTn Structure                                                   */
  __IO uint32_t  MODE_CNT_WDT;                      /*!< Mode Control WDT Setting                                              */
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  MODE_CNT_PLL0;                     /*!< Mode Control PLL Setting                                              */
  __IO uint32_t  MODE_CNT_PLL1;                     /*!< Mode Control PLL Setting                                              */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  MODE_CNT_CLKGEN0;                  /*!< Mode Control CLK Source Setting                                       */
  __IO uint32_t  MODE_CNT_CLKGEN1;                  /*!< Mode Control CLK Source Setting(Peripheral)                           */
  __IO uint32_t  MODE_CNT_CLKGEN2;                  /*!< Mode Control CLK Timer Source Setting                                 */
  __IO uint32_t  MODE_CNT_CLKGEN3;                  /*!< Mode Control CLK Dividing setup(Peripheral)                           */
  __IO uint32_t  MODE_CNT_CLKGEN4;                  /*!< Mode Control CLK Dividing setup(Peripheral)                           */
  __IO uint32_t  MODE_CNT_CLKGEN5;                  /*!< Mode Control CLK Dividing setup(Peripheral)                           */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  MODE_CNT_CLKGEN8;                  /*!< CLK Enable                                                            */
  __IO uint32_t  MODE_CNT_CLKGEN9;                  /*!< CLK stable latency time                                               */
  __IO uint32_t  MODE_CNT_DPSLP;                    /*!< Deep sleep control                                                    */
  __IO uint32_t  MODE_CNT_FLDPSTB;                  /*!< FLASH-ROM deep standby control                                        */
  __IO uint32_t  MODE_CNT_FLWKUPT;                  /*!< FLASH WakeUp time setup                                               */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  MODE_CNT_IMSK;                     /*!< MODE_CNT interrupt mask                                               */
  __I  uint32_t  MODE_CNT_IST;                      /*!< MODE_CNT interrupt status                                             */
  __I  uint32_t  MODE_CNT_ISTRAW;                   /*!< MODE_CNT interrupt status raw                                         */
  __I  uint32_t  MODE_CNT_RCH0CLR;                  /*!< RCOSC_H0 stability done interrupt clear                               */
  __I  uint32_t  MODE_CNT_RCL0CLR;                  /*!< RCOSC_L0 stability done interrupt clear                               */
  __I  uint32_t  MODE_CNT_XTAL32KCLR;               /*!< XTAL32K stability done interrupt clear                                */
  __I  uint32_t  MODE_CNT_PLLCLR;                   /*!< PLL stability done interrupt clear                                    */
  __I  uint32_t  MODE_CNT_ALLCLR;                   /*!< MODE_CNT interrupt all clear                                          */
  __I  uint32_t  MODE_CNT_CLKST;                    /*!< CLK Status                                                            */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  MODE_CNT_RF;                       /*!< RF control                                                            */
  __I  uint32_t  RESERVED5[3];
  __IO uint32_t  MODE_TEST2;                        /*!< Mode TEST2                                                            */
  __I  uint32_t  RESERVED6[10];
  __IO uint32_t  MODE_CNT_RCOSC_H0_CTRL;            /*!< RCOSC_H0 Control                                                      */
  __IO uint32_t  MODE_CNT_RSOSC_L0_CTRL;            /*!< RCOSC_L0 Control                                                      */
  __IO uint32_t  MODE_CNT_PLL2;                     /*!< Mode Control PLL Setting                                              */
  __I  uint32_t  RESERVED7[13];
  __IO uint32_t  MODE_CNT_EXTCLKEN;                 /*!< EXTCLK clock enable                                                   */
  __IO uint32_t  MODE_CNT_ICLK_DIV;                 /*!< ICLK clock DIV                                                        */
  __I  uint32_t  RESERVED8[6];
  __IO uint32_t  MODE_CNT_SRAM_LP0;                 /*!< SRAM power configuration                                              */
  __I  uint32_t  RESERVED9[3];
  __IO uint32_t  MODE_CNT_IOSET_SWCK;               /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_SWD;                /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED10[3];
  __IO uint32_t  MODE_CNT_IOSET_RF_GPIO0;           /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_RF_GPIO1;           /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED11[2];
  __IO uint32_t  MODE_CNT_IOSET_MOSI2;              /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_MISO2;              /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_SSN2;               /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_SCK2;               /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED12[7];
  __IO uint32_t  MODE_CNT_IOSET_GPIOA0;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA1;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA2;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA3;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA4;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA5;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOA6;             /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED13[9];
  __IO uint32_t  MODE_CNT_IOSET_GPIOB0;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB1;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB2;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB3;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB4;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB5;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB6;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOB7;             /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED14[8];
  __IO uint32_t  MODE_CNT_IOSET_GPIOC0;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC1;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC2;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC3;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC4;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC5;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC6;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOC7;             /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED15[8];
  __IO uint32_t  MODE_CNT_IOSET_GPIOD0;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOD1;             /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED16[14];
  __IO uint32_t  MODE_CNT_IOSET_GPIOE0;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOE1;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOE2;             /*!< IO configuration                                                      */
  __IO uint32_t  MODE_CNT_IOSET_GPIOE3;             /*!< IO configuration                                                      */
  __I  uint32_t  RESERVED17[156];
  __I  uint32_t  MODE_CNT_RFTRIM0;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM1;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM2;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM3;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM4;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM5;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM6;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM7;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM8;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM9;                  /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM10;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM11;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM12;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM13;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM14;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM15;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM16;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM17;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM18;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM19;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM20;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM21;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM22;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM23;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM24;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM25;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM26;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM27;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM28;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM29;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM30;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM31;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM32;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM33;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM34;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM35;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM36;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM37;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM38;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM39;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM40;                 /*!< RF trimming value                                                     */
  __I  uint32_t  MODE_CNT_RFTRIM41;                 /*!< RF trimming value                                                     */
} MODE_CNTn_Type;


/* ================================================================================ */
/* ================                    LVD_CNTn                    ================ */
/* ================================================================================ */


/**
  * @brief LVD Control (LVD_CNTn)
  */

typedef struct {                                    /*!< LVD_CNTn Structure                                                    */
  __I  uint32_t  LVD0CON;                           /*!< LVD Control                                                           */
  __IO uint32_t  LVD0W;                             /*!< LVD wait Control                                                      */
  __IO uint32_t  ENLVD0;                            /*!< LVD0 Control                                                          */
} LVD_CNTn_Type;


/* ================================================================================ */
/* ================                    ADC_CNTn                    ================ */
/* ================================================================================ */


/**
  * @brief ADC Control (ADC_CNTn)
  */

typedef struct {                                    /*!< ADC_CNTn Structure                                                    */
  __IO uint32_t  ADCON;                             /*!< ADC Control                                                           */
  __IO uint32_t  ADINT;                             /*!< ADC interrupt status                                                  */
  __IO uint32_t  ADINTEN;                           /*!< ADC interrupt enable                                                  */
  __I  uint32_t  ADRST0;                            /*!< ADC result ch0                                                        */
  __I  uint32_t  ADRST1;                            /*!< ADC result ch1                                                        */
  __I  uint32_t  ADRST2;                            /*!< ADC result ch2                                                        */
  __I  uint32_t  ADRST3;                            /*!< ADC result ch3                                                        */
  __IO uint32_t  ADDT0;                             /*!< ADC result temp0                                                      */
  __IO uint32_t  ADDT1;                             /*!< ADC result temp1                                                      */
  __IO uint32_t  ADDT2;                             /*!< ADC result temp2                                                      */
  __IO uint32_t  ADDT3;                             /*!< ADC result voltage                                                    */
} ADC_CNTn_Type;


/* ================================================================================ */
/* ================                  UX_DMACm_CHn                  ================ */
/* ================================================================================ */


/**
  * @brief Direct Memory Access Controller (UX_DMACm_CHn)
  */

typedef struct {                                    /*!< UX_DMACm_CHn Structure                                                */
  __IO uint32_t  DMACm_SARn;                        /*!< Channel x Source Address Register.                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DMACm_DARn;                        /*!< Channel x Destination Address Register.                               */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  DMACm_LLPn;                        /*!< Channel x Linked List Pointer Register.                               */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  DMACm_CTLn_L;                      /*!< Channel x Control Register. (64bit:31:0)                              */
  __IO uint32_t  DMACm_CTLn_H;                      /*!< Channel x Control Register. (64bit:63:32)                             */
  __I  uint32_t  RESERVED3[8];
  __IO uint32_t  DMACm_CFGn_L;                      /*!< Channel x Configuration Register.(64bit:31:0)                         */
  __IO uint32_t  DMACm_CFGn_H;                      /*!< Channel x Configuration Register.(64bit:63:32)                        */
} UX_DMACm_CHn_Type;


/* ================================================================================ */
/* ================                  UX_DMACm_COMM                 ================ */
/* ================================================================================ */


/**
  * @brief Direct Memory Access Controller (UX_DMACm_COMM)
  */

typedef struct {                                    /*!< UX_DMACm_COMM Structure                                               */
  __I  uint32_t  DMACm_RAW_TFR;                     /*!< Raw Status for IntTfr Interrupt.                                      */
  __I  uint32_t  RESERVED0;
  __I  uint32_t  DMACm_RAW_BLOCK;                   /*!< Raw Status for IntBlock Interrupt.                                    */
  __I  uint32_t  RESERVED1;
  __I  uint32_t  DMACm_RAW_SRC_TRAN;                /*!< Raw Status for IntSrcTran Interrupt.                                  */
  __I  uint32_t  RESERVED2;
  __I  uint32_t  DMACm_RAW_DST_TRAN;                /*!< Raw Status for IntDstTran Interrupt.                                  */
  __I  uint32_t  RESERVED3;
  __I  uint32_t  DMACm_RAW_ERR;                     /*!< Raw Status for IntErr Interrupt.                                      */
  __I  uint32_t  RESERVED4;
  __I  uint32_t  DMACm_STAT_TFR;                    /*!< Status for IntTfr Interrupt.                                          */
  __I  uint32_t  RESERVED5;
  __I  uint32_t  DMACm_STAT_BLOCK;                  /*!< Status for IntBlock Interrupt.                                        */
  __I  uint32_t  RESERVED6;
  __I  uint32_t  DMACm_STAT_SRC_TRAN;               /*!< Status for IntSrcTran Interrupt.                                      */
  __I  uint32_t  RESERVED7;
  __I  uint32_t  DMACm_STAT_DST_TRAN;               /*!< Status for IntDstTran Interrupt.                                      */
  __I  uint32_t  RESERVED8;
  __I  uint32_t  DMACm_STAT_ERR;                    /*!< Status for IntErr Interrupt.                                          */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  DMACm_MASK_TFR;                    /*!< Mask for IntTfr Interrupt.                                            */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  DMACm_MASK_BLOCK;                  /*!< Mask for IntBlock Interrupt.                                          */
  __I  uint32_t  RESERVED11;
  __IO uint32_t  DMACm_MASK_SRC_TRAN;               /*!< Mask for IntSrcTran Interrupt.                                        */
  __I  uint32_t  RESERVED12;
  __IO uint32_t  DMACm_MASK_DST_TRAN;               /*!< Mask for IntDstTran Interrupt.                                        */
  __I  uint32_t  RESERVED13;
  __IO uint32_t  DMACm_MASK_ERR;                    /*!< Mask for IntErr Interrupt.                                            */
  __I  uint32_t  RESERVED14;
  __O  uint32_t  DMACm_CLEAR_TFR;                   /*!< Clear for IntTfr Interrupt.                                           */
  __I  uint32_t  RESERVED15;
  __O  uint32_t  DMACm_CLEAR_BLOCK;                 /*!< Clear for IntBlock Interrupt.                                         */
  __I  uint32_t  RESERVED16;
  __O  uint32_t  DMACm_CLEAR_SRC_TRAN;              /*!< Clear for IntSrcTran Interrupt.                                       */
  __I  uint32_t  RESERVED17;
  __O  uint32_t  DMACm_CLEAR_DST_TRAN;              /*!< Clear for IntDstTran Interrupt.                                       */
  __I  uint32_t  RESERVED18;
  __O  uint32_t  DMACm_CLEAR_ERR;                   /*!< Clear for IntErr Interrupt.                                           */
  __I  uint32_t  RESERVED19;
  __I  uint32_t  DMACm_STAT_INT;                    /*!< Status for each interrupt type.                                       */
  __I  uint32_t  RESERVED20;
  __IO uint32_t  DMACm_REQ_SRC;                     /*!< Source Software Transaction Request Register.                         */
  __I  uint32_t  RESERVED21;
  __IO uint32_t  DMACm_REQ_DST;                     /*!< Destination Software Transaction Request Register.                    */
  __I  uint32_t  RESERVED22;
  __IO uint32_t  DMACm_SGL_REQ_SRC;                 /*!< Single Source Transaction Request Register.                           */
  __I  uint32_t  RESERVED23;
  __IO uint32_t  DMACm_SGL_REQ_DST;                 /*!< Single Destination Transaction Request Register.                      */
  __I  uint32_t  RESERVED24;
  __IO uint32_t  DMACm_LST_SRC;                     /*!< Last Source Transaction Request Register.                             */
  __I  uint32_t  RESERVED25;
  __IO uint32_t  DMACm_LST_DST;                     /*!< Last Destination Transaction Request Register.                        */
  __I  uint32_t  RESERVED26;
  __IO uint32_t  DMACm_CFG;                         /*!< DMA Configuration Register.                                           */
  __I  uint32_t  RESERVED27;
  __IO uint32_t  DMACm_CH_EN;                       /*!< DMA Channel Enable Register                                           */
  __I  uint32_t  RESERVED28[275];
  __IO uint32_t  DMACm_HSIF_SELn;                   /*!< Handshake interface selection Register.                               */
} UX_DMACm_COMM_Type;


/* ================================================================================ */
/* ================                      AESn                      ================ */
/* ================================================================================ */


/**
  * @brief AES (AESn)
  */

typedef struct {                                    /*!< AESn Structure                                                        */
  __IO uint32_t  AES_CTL;                           /*!< AES Control                                                           */
  __IO uint32_t  AES_MOD;                           /*!< AES Mode                                                              */
  __IO uint32_t  AES_GCCM_CTL;                      /*!< GCM CCM Control                                                       */
  __IO uint32_t  AES_ST;                            /*!< AES Status                                                            */
  __I  uint32_t  AES_RIST;                          /*!< AES Raw Interrupt Status                                              */
  __I  uint32_t  AES_IST;                           /*!< AES Interrupt Status                                                  */
  __IO uint32_t  AES_IMSK;                          /*!< AES Interrupt Mask                                                    */
  __I  uint32_t  AES_ICLR;                          /*!< AES Interrupt Clear                                                   */
  __I  uint32_t  AES_CLR_BLKDONE;                   /*!< AES Block Interrupt Clear                                             */
  __I  uint32_t  AES_CLR_DONE;                      /*!< AES Done Interrupt Clear                                              */
  __O  uint32_t  AES_KEY0;                          /*!< Key[31:0]                                                             */
  __O  uint32_t  AES_KEY1;                          /*!< Key[63:32]                                                            */
  __O  uint32_t  AES_KEY2;                          /*!< Key[95:64]                                                            */
  __O  uint32_t  AES_KEY3;                          /*!< Key[127:96]                                                           */
  __O  uint32_t  AES_KEY4;                          /*!< Key[159:128]                                                          */
  __O  uint32_t  AES_KEY5;                          /*!< Key[191:160]                                                          */
  __O  uint32_t  AES_KEY6;                          /*!< Key[223:192]                                                          */
  __O  uint32_t  AES_KEY7;                          /*!< Key[255:224]                                                          */
  __IO uint32_t  AES_HKEY0;                         /*!< HASH Sub Key[31:0]                                                    */
  __IO uint32_t  AES_HKEY1;                         /*!< HASH Sub Key[63:32]                                                   */
  __IO uint32_t  AES_HKEY2;                         /*!< HASH Sub Key[95:64]                                                   */
  __IO uint32_t  AES_HKEY3;                         /*!< HASH Sub Key[127:96]                                                  */
  __IO uint32_t  AES_IV0;                           /*!< IV[31:0]                                                              */
  __IO uint32_t  AES_IV1;                           /*!< IV[63:32]                                                             */
  __IO uint32_t  AES_IV2;                           /*!< IV[95:64]                                                             */
  __IO uint32_t  AES_IV3;                           /*!< IV[127:96]                                                            */
  __IO uint32_t  AES_CTRIV0;                        /*!< CTRIV[31:0]                                                           */
  __IO uint32_t  AES_CTRIV1;                        /*!< CTRIV[63:32]                                                          */
  __IO uint32_t  AES_CTRIV2;                        /*!< CTRIV[95:64]                                                          */
  __IO uint32_t  AES_CTRIV3;                        /*!< CTRIV[127:96]                                                         */
  __IO uint32_t  AES_IDATA;                         /*!< IDATA                                                                 */
  __I  uint32_t  AES_ODATA;                         /*!< ODATA                                                                 */
  __I  uint32_t  AES_ODATA20;                       /*!< ODATA2[31:0]                                                          */
  __I  uint32_t  AES_ODATA21;                       /*!< ODATA2[63:32]                                                         */
  __I  uint32_t  AES_ODATA22;                       /*!< ODATA2[95:64]                                                         */
  __I  uint32_t  AES_ODATA23;                       /*!< ODATA2[127:96]                                                        */
} AESn_Type;


/* ================================================================================ */
/* ================                      RAMC                      ================ */
/* ================================================================================ */


/**
  * @brief ECC SRAM CONTROLER (RAMC)
  */

typedef struct {                                    /*!< RAMC Structure                                                        */
  __IO uint32_t  ECC_CON;                           /*!< ECC control                                                           */
  __IO uint32_t  ECC_INT_MASK;                      /*!< ECC Interrupt mask                                                    */
  __IO uint32_t  ECC_ST;                            /*!< ECC Interrupt status                                                  */
  __IO uint32_t  ECC_RAWST;                         /*!< ECC RAW Interrupt status                                              */
  __I  uint32_t  ECC_SECADR;                        /*!< ECC 1bit error address                                                */
  __I  uint32_t  ECC_DEDADR;                        /*!< ECC 2bit error address                                                */
  __I  uint32_t  ECC_DATA_POSITION;                 /*!< ECC DATA bit error position                                           */
  __I  uint32_t  ECC_CHECK_POSITION;                /*!< ECC CHECK bit error position                                          */
} RAMC_Type;


/* ================================================================================ */
/* ================                      HASH                      ================ */
/* ================================================================================ */


/**
  * @brief SECURE HASH (HASH)
  */

typedef struct {                                    /*!< HASH Structure                                                        */
  __IO uint32_t  HASH_IRQ;                          /*!< IRQ cause                                                             */
  __IO uint32_t  HASH_IMSK;                         /*!< IRQ mask                                                              */
  __IO uint32_t  HASH_STRT;                         /*!< Number of starts                                                      */
  __IO uint32_t  HASH_CCNT;                         /*!< Number of completions                                                 */
  __I  uint32_t  HASH_STAT;                         /*!< Status                                                                */
  __I  uint32_t  RESERVED0;
  __O  uint32_t  HASH_MBCL;                         /*!< Message block clear                                                   */
  __IO uint32_t  HASH_CTRL;                         /*!< Control                                                               */
  __I  uint32_t  RESERVED1[10];
  __IO uint32_t  HASH_MD2;                          /*!< MD2                                                                   */
  __IO uint32_t  HASH_MD3;                          /*!< MD3                                                                   */
  __IO uint32_t  HASH_MD4;                          /*!< MD4                                                                   */
  __IO uint32_t  HASH_MD5;                          /*!< MD5                                                                   */
  __IO uint32_t  HASH_MD6;                          /*!< MD6                                                                   */
  __IO uint32_t  HASH_MD7;                          /*!< MD7                                                                   */
  __IO uint32_t  HASH_MD8;                          /*!< MD8                                                                   */
  __IO uint32_t  HASH_MD9;                          /*!< MD9                                                                   */
  __I  uint32_t  RESERVED2[6];
  __O  uint32_t  HASH_MB0;                          /*!< MB0                                                                   */
  __O  uint32_t  HASH_MB1;                          /*!< MB1                                                                   */
  __O  uint32_t  HASH_MB2;                          /*!< MB2                                                                   */
  __O  uint32_t  HASH_MB3;                          /*!< MB3                                                                   */
  __O  uint32_t  HASH_MB4;                          /*!< MB4                                                                   */
  __O  uint32_t  HASH_MB5;                          /*!< MB5                                                                   */
  __O  uint32_t  HASH_MB6;                          /*!< MB6                                                                   */
  __O  uint32_t  HASH_MB7;                          /*!< MB7                                                                   */
  __O  uint32_t  HASH_MB8;                          /*!< MB8                                                                   */
  __O  uint32_t  HASH_MB9;                          /*!< MB9                                                                   */
  __O  uint32_t  HASH_MB10;                         /*!< MB10                                                                  */
  __O  uint32_t  HASH_MB11;                         /*!< MB11                                                                  */
  __O  uint32_t  HASH_MB12;                         /*!< MB12                                                                  */
  __O  uint32_t  HASH_MB13;                         /*!< MB13                                                                  */
  __O  uint32_t  HASH_MB14;                         /*!< MB14                                                                  */
  __O  uint32_t  HASH_MB15;                         /*!< MB15                                                                  */
  __O  uint32_t  HASH_MBFIFO;                       /*!< MBFIFO                                                                */
} HASH_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define UX_SYSCON_BASE                  0x40000000UL
#define UX_FLC0_BASE                    0x40000400UL
#define UX_TMRA_TIMER1_BASE             0x40001000UL
#define UX_FTMA_FTM0_BASE               0x40002000UL
#define UX_FTMA_BASE                    0x40002200UL
#define UX_FTMB_BASE                    0x40002600UL
#define UX_FTMC_BASE                    0x40002A00UL
#define UX_FTMD_BASE                    0x40002E00UL
#define UX_FTME_BASE                    0x40003200UL
#define UX_FTMF_BASE                    0x40003600UL
#define UX_RTC_BASE                     0x40007C00UL
#define UX_UART0_BASE                   0x40004000UL
#define UX_I2C0_BASE                    0x40008000UL
#define UX_GPIOA_BASE                   0x4000A000UL
#define EXTINT_BASE                     0x4000D000UL
#define UX_WDT_BASE                     0x40010000UL
#define UX_STDPORTA_BASE                0x40018000UL
#define UX_SPI0_BASE                    0x40040000UL
#define DIO0_BASE                       0x40043000UL
#define TRNG_BASE                       0x40044000UL
#define CLK_TIMER0_BASE                 0x40045000UL
#define MODE_CNT0_BASE                  0x40050000UL
#define LVD_CNT0_BASE                   0x40090000UL
#define ADC_CNT0_BASE                   0x40070000UL
#define UX_DMAC0_CH0_BASE               0x400C0000UL
#define UX_DMAC0_COMM_BASE              0x400C02C0UL
#define AES0_BASE                       0x400D0000UL
#define RAMC_BASE                       0x400E0000UL
#define HASH_BASE                       0x40091000UL
#define UX_I2C1_BASE                    0x40008400UL
#define UX_UART1_BASE                   0x40004400UL
#define UX_UART2_BASE                   0x40004800UL
#define UX_GPIOB_BASE                   0x4000A400UL
#define UX_GPIOC_BASE                   0x4000A800UL
#define UX_GPIOD_BASE                   0x4000AC00UL
#define UX_STDPORTB_BASE                0x40018400UL
#define UX_STDPORTC_BASE                0x40018800UL
#define UX_STDPORTD_BASE                0x40018C00UL
#define UX_SWDSTDPORT_BASE              0x4001A000UL
#define UX_RFSTDPORT_BASE               0x4001A400UL
#define UX_SPI1_BASE                    0x40040400UL
#define UX_SPI2_BASE                    0x40040800UL
#define UX_TMRB_TIMER1_BASE             0x40041000UL
#define UX_TMRC_TIMER1_BASE             0x40041400UL
#define UX_TMRD_TIMER1_BASE             0x40041800UL
#define UX_FTMB_FTM0_BASE               0x40002400UL
#define UX_FTMC_FTM0_BASE               0x40002800UL
#define UX_FTMD_FTM0_BASE               0x40002C00UL
#define UX_FTME_FTM0_BASE               0x40003000UL
#define UX_FTMF_FTM0_BASE               0x40003400UL
#define UX_DMAC0_CH1_BASE               0x400C0058UL
#define UX_DMAC0_CH2_BASE               0x400C00B0UL
#define UX_DMAC0_CH3_BASE               0x400C0108UL
#define UX_DMAC0_CH4_BASE               0x400C0160UL
#define UX_DMAC0_CH5_BASE               0x400C01B8UL
#define UX_DMAC0_CH6_BASE               0x400C0210UL
#define UX_DMAC0_CH7_BASE               0x400C0268UL
#define UX_DMAC1_CH0_BASE               0x400C0800UL
#define UX_DMAC1_CH1_BASE               0x400C0858UL
#define UX_DMAC1_COMM_BASE              0x400C0AC0UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define UX_SYSCON                       ((UX_SYSCON_Type          *) UX_SYSCON_BASE)
#define UX_FLC0                         ((UX_FLCn_Type            *) UX_FLC0_BASE)
#define UX_TMRA_TIMER1                  ((UX_TMRm_TIMERn_Type     *) UX_TMRA_TIMER1_BASE)
#define UX_FTMA_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTMA_FTM0_BASE)
#define UX_FTMA                         ((UX_FTMA_Type            *) UX_FTMA_BASE)
#define UX_FTMB                         ((UX_FTMB_Type            *) UX_FTMB_BASE)
#define UX_FTMC                         ((UX_FTMC_Type            *) UX_FTMC_BASE)
#define UX_FTMD                         ((UX_FTMD_Type            *) UX_FTMD_BASE)
#define UX_FTME                         ((UX_FTME_Type            *) UX_FTME_BASE)
#define UX_FTMF                         ((UX_FTMF_Type            *) UX_FTMF_BASE)
#define UX_RTC                          ((UX_RTC_Type             *) UX_RTC_BASE)
#define UX_UART0                        ((UX_UARTn_Type           *) UX_UART0_BASE)
#define UX_I2C0                         ((UX_I2Cn_Type            *) UX_I2C0_BASE)
#define UX_GPIOA                        ((UX_GPIOm_Type           *) UX_GPIOA_BASE)
#define EXTINT                          ((EXTINT_Type             *) EXTINT_BASE)
#define UX_WDT                          ((UX_WDT_Type             *) UX_WDT_BASE)
#define UX_STDPORTA                     ((UX_STDPORTm_Type        *) UX_STDPORTA_BASE)
#define UX_SPI0                         ((UX_SPIn_Type            *) UX_SPI0_BASE)
#define DIO0                            ((DIOn_Type               *) DIO0_BASE)
#define TRNG                            ((TRNG_Type               *) TRNG_BASE)
#define CLK_TIMER0                      ((CLK_TIMERn_Type         *) CLK_TIMER0_BASE)
#define MODE_CNT0                       ((MODE_CNTn_Type          *) MODE_CNT0_BASE)
#define LVD_CNT0                        ((LVD_CNTn_Type           *) LVD_CNT0_BASE)
#define ADC_CNT0                        ((ADC_CNTn_Type           *) ADC_CNT0_BASE)
#define UX_DMAC0_CH0                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH0_BASE)
#define UX_DMAC0_COMM                   ((UX_DMACm_COMM_Type      *) UX_DMAC0_COMM_BASE)
#define AES0                            ((AESn_Type               *) AES0_BASE)
#define RAMC                            ((RAMC_Type               *) RAMC_BASE)
#define HASH                            ((HASH_Type               *) HASH_BASE)
#define UX_I2C1                         ((UX_I2Cn_Type            *) UX_I2C1_BASE)
#define UX_UART1                        ((UX_UARTn_Type           *) UX_UART1_BASE)
#define UX_UART2                        ((UX_UARTn_Type           *) UX_UART2_BASE)
#define UX_GPIOB                        ((UX_GPIOm_Type           *) UX_GPIOB_BASE)
#define UX_GPIOC                        ((UX_GPIOm_Type           *) UX_GPIOC_BASE)
#define UX_GPIOD                        ((UX_GPIOm_Type           *) UX_GPIOD_BASE)
#define UX_STDPORTB                     ((UX_STDPORTm_Type        *) UX_STDPORTB_BASE)
#define UX_STDPORTC                     ((UX_STDPORTm_Type        *) UX_STDPORTC_BASE)
#define UX_STDPORTD                     ((UX_STDPORTm_Type        *) UX_STDPORTD_BASE)
#define UX_SWDSTDPORT                   ((UX_STDPORTm_Type        *) UX_SWDSTDPORT_BASE)
#define UX_RFSTDPORT                    ((UX_STDPORTm_Type        *) UX_RFSTDPORT_BASE)
#define UX_SPI1                         ((UX_SPIn_Type            *) UX_SPI1_BASE)
#define UX_SPI2                         ((UX_SPIn_Type            *) UX_SPI2_BASE)
#define UX_TMRB_TIMER1                  ((UX_TMRm_TIMERn_Type     *) UX_TMRB_TIMER1_BASE)
#define UX_TMRC_TIMER1                  ((UX_TMRm_TIMERn_Type     *) UX_TMRC_TIMER1_BASE)
#define UX_TMRD_TIMER1                  ((UX_TMRm_TIMERn_Type     *) UX_TMRD_TIMER1_BASE)
#define UX_FTMB_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTMB_FTM0_BASE)
#define UX_FTMC_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTMC_FTM0_BASE)
#define UX_FTMD_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTMD_FTM0_BASE)
#define UX_FTME_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTME_FTM0_BASE)
#define UX_FTMF_FTM0                    ((UX_FTMm_FTMn_Type       *) UX_FTMF_FTM0_BASE)
#define UX_DMAC0_CH1                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH1_BASE)
#define UX_DMAC0_CH2                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH2_BASE)
#define UX_DMAC0_CH3                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH3_BASE)
#define UX_DMAC0_CH4                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH4_BASE)
#define UX_DMAC0_CH5                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH5_BASE)
#define UX_DMAC0_CH6                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH6_BASE)
#define UX_DMAC0_CH7                    ((UX_DMACm_CHn_Type       *) UX_DMAC0_CH7_BASE)
#define UX_DMAC1_CH0                    ((UX_DMACm_CHn_Type       *) UX_DMAC1_CH0_BASE)
#define UX_DMAC1_CH1                    ((UX_DMACm_CHn_Type       *) UX_DMAC1_CH1_BASE)
#define UX_DMAC1_COMM                   ((UX_DMACm_COMM_Type      *) UX_DMAC1_COMM_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group ML7436N */
/** @} */ /* End of group LAPIS Semiconductor Co.,Ltd. */

#ifdef __cplusplus
}
#endif


#endif  /* ML7436N_H */

