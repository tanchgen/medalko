//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <measur.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "main.h"
#include "statefunc.h"
#include "usb_vcp.h"

#define TALLOC_ARRAY_SIZE   8192

//uint8_t tallocArray[TALLOC_ARRAY_SIZE] __aligned(4);

static inline void stateProcess( void );

// ----- main() ---------------------------------------------------------------
#if SWO_ENABLE
static void swoCfg( uint32_t cpuCoreFreqHz );
#endif // SWO_ENABLE

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;

  // Tiny memory allocated init
//  ta_init( tallocArray, tallocArray+TALLOC_ARRAY_SIZE, 256, 16, sizeof(int) );

  /*
   *  Инициализация периферии.
   */

  /* Инициализация Reset and Clock Control (RCC). */
  rccInit();

#if SWO_ENABLE
  swoCfg( rccClocks.HCLK_Frequency );
#endif

#ifdef DEBUG
  trace_puts("\nHello, Medic!");
#endif
  /* Инициализация интерфейсов. */
//  mDelay( 100 );
  ifaceInit();

//  __enable_irq();

  mDelay( 100 );

  /*
   *  Разрешение работы периферии.
   */

  ifaceEnable();

  mDelay(INIT_TOUT);
#ifdef DEBUG
  trace_puts("ALKO 2 Run");
#endif

  // Infinite loop
  while (1) {
    stateProcess();
    ifaceClock();
  }
}


static inline void stateProcess( void ){
  // Обработаем Машину состояний

  // Обработаем Машину состояний
  switch( measState ){
    case MEASST_OFF:
      stateOff();
      break;
//    case MEASST_REL_EN:
//      break;
    case MEASST_START_PROB:
      stateStart();
      break;
    case MEASST_FLOW_PROB:
      stateFlow();
      break;
    case MEASST_END_PROB:
      stateEnd();
      break;
//    case MEASST_PROC:
//      stateProc();
//      break;
    case MEASST_FIN:
      stateFin();
      break;
    case MEASST_FAULT:
      stateFault();
      break;
    default:
      break;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
//  errHandler( ERR_UNKNOWN );
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line
  *     number where the assert_param error has occurred.
  *
  * @param[in]  file  pointer to the source file name
  * @param[in]  line  assert_param error line source number
  *
  * @retval none
  */
void assert_failed(uint8_t *file, uint32_t line) {
  trace_printf("Wrong parameters value: file %s on line %u\n", file, (uint)line);

  /* Infinite loop. */
  while (1)
  {}
}
#endif

#if SWO_ENABLE
static void swoCfg( uint32_t cpuCoreFreqHz ) {
    uint32_t SWOSpeed = 2000000; /* default 64k baud rate */
    uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1;

    // PB3
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3)) | (((uint32_t)GPIO_MODE_AFPP_50) << (3*4));
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins

    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }

    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = SWOPrescaler; // Trace clock = HCLK/(x+1) = 8MHz
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    TPI->FFCR = 0x100; // TPIU packet framing enabled when bit 2 is set.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.

//    /* Configure PC sampling and exception trace  */
//    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
//                                           // 0 = x32, 1 = x512
//              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
//                                                // Divider = value + 1
//              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
//              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
//                                               // 0 = Off, 1 = Every 2^23 cycles,
//                                               // 2 = Every 2^25, 3 = Every 2^27
//              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
//              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
//
    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
               | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
               | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
               | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports

}
#endif //SWO_ENABLE

// ----------------------------------------------------------------------------
