#include "main.h"

RCC_ClocksTypeDef  rccClocks;


void rccInit(void){
  RCC_DeInit();

  /* Enable HSE. */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready and if Time out is reached exit. */
  if (RCC_WaitForHSEStartUp() == SUCCESS) {
    /* HCLK = SYSCLK / 1 = 120 MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK = 72 MHz */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK / 4 = 36 MHz */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /*
     * Configure the main PLL:
     *  PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN = 240 MHz
     *  SYSCLK = PLL_VCO / PLLP = 120 MHz
     *  USB OTG FS, SDIO and RNG Clock = PLL_VCO / PLLQ = 48 MHz
     */
#ifdef STM32F4XX
    RCC_PLLConfig(RCC_PLLSource_HSE, HSE_VALUE/1000000, 250, 2, 5);
#else
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
#endif // STM32F4XX

    BIT_BAND_PER(RCC->CFGR, RCC_CFGR_USBPRE) = RESET;

    /* Enable the main PLL. */
    RCC_PLLCmd(ENABLE);

    /* Wait till the main PLL is ready. */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    /* Configure Flash prefetch, Instruction cache, Data cache and wait states. */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
//    FLASH_InstructionCacheCmd(ENABLE);
//    FLASH_DataCacheCmd(ENABLE);
    FLASH_SetLatency(FLASH_Latency_2);

    /* Select the main PLL as system clock source. */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till the main PLL is used as system clock source. */
    while (RCC_GetSYSCLKSource() != RCC_CFGR_SWS_PLL)
    {}
  }
  // Select ADC clock = APB2 / 6
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_1;

  RCC_GetClocksFreq(&rccClocks);

  /* Настраиваем системный таймер на заданную частоту. */
  SysTick_Config(rccClocks.SYSCLK_Frequency / TICK_HZ);
}
