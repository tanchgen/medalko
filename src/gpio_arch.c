#include <stddef.h>
#include <stdbool.h>
#include <limits.h>

#include "main.h"
#include "statefunc.h"

#include "gpio_arch.h"

/** Флаг направления включения-выключения системы:
 * SET - направление off->on
 * RESET - направление on->off
 */
FlagStatus mesurRun = RESET;

FlagStatus onCan = RESET;

/** Состояние уровня запуска системы */
eMcuState mesurState = MCUSTATE_SYS_OFF;

sGpioPin gpioPinRelEn = {GPIOA, 0, GPIO_Pin_5, 5, GPIO_MODE_OPP_10, GPIO_PULLDOWN, Bit_RESET, Bit_RESET, RESET };
sGpioPin gpioPinRelOn = {GPIOA, 0, GPIO_Pin_4, 4, GPIO_MODE_OPP_10, GPIO_PULLDOWN, Bit_RESET, Bit_RESET, RESET };

FlagStatus relOnSet;
uint8_t relOnCount;

// ------------------- LEDS -----------------------
// Определено в led.c

// ===========================================================================================================
/** Структура дескриптора таймера таймаута изменения состояния системы при включении */
struct timer_list  pwrOnToutTimer;
/** Структура дескриптора таймера таймаута изменения состояния системы при выключении */
struct timer_list  pwrOffToutTimer;
/** Структура дескриптора таймера изменения состояния системы при включении*/
struct timer_list  pwrOnUpdateTimer;
/** Структура дескриптора таймера изменения состояния системы при выключении */
struct timer_list  pwrOffUpdateTimer;

/** Структура дескриптора таймера таймаута восстановления после ошибки */
struct timer_list  pwrOnCanTimer;
/** Структура дескриптора таймера таймаута восстановления после ошибки FPGA*/
struct timer_list  fpgaOnCanTimer;

/** Структура дескриптора таймера таймаута ожидания отклика на FPGA_RST*/
struct timer_list  mzuRstTestTimer;

// =================== Прототипы функций ====================================
// ==========================================================================


/**
  * @brief  Обработчик таймаута восстановления после ошибки
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void mesurOnCan(uintptr_t arg){
  (void)arg;
  // Система выключена полностью и готова к повторному включению
  // Восстанавливаем моргание зеленого светодиода
  ledSysSndby();
  onCan = SET;
}


///**
//  * @brief  Обработчик таймаута ожидания условия при вкл/выкл питания
//  *
//  * @param[in]  arg  NULL
//  *
//  * @retval none
//  */
//static void pwrTimeout(uintptr_t arg){
//  (void)arg;
//  // Включаем на постоянно светодиод LED_PWR_FAULE
//  trace_puts("PWR_TOUT");
//  ledOn( LED_R, 0 );
//  ledToggleSet( LED_ERR, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0 );
//}
//

/**
  * @brief  Обработчик тайм-аута включения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void pwrOnTout(uintptr_t arg){

//  // Не возникло событие успешного завершения очередного этапа включения системы
//  trace_printf("PWR ON tout State: %d\n", arg);
//
//  switch( arg ){
//    case MCUSTATE_SYS_START:
//#if DEBUG_ALARM
//        trace_puts("  - no PG_PSU ." );
//#endif
//        ledToggleSet( LED_R, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0 );
//      break;
//    case MCUSTATE_PSU_ON:
//#if DEBUG_ALARM
//        trace_puts("  - Not all I2C_Devices is ready." );
//#endif
//        ledToggleSet( LED_R, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0 );
//      break;
//    case MCUSTATE_FAN_ON:
//#if DEBUG_ALARM
//      trace_puts("  - no PG_RUN_1" );
//#endif
//      sysAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_EN_1, 1 );
//      return;
//      break;
//    case MCUSTATE_RUN_1:
//#if DEBUG_ALARM
//      trace_puts("  - no PG_RUN_2" );
//#endif
//      sysAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_EN_2, 1 );
//      return;
//      break;
//    case MCUSTATE_RUN_2:
//#if DEBUG_ALARM
//      trace_puts("  - no PG_RUN_3" );
//#endif
//      sysAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_EN_3, 1 );
//      return;
//      break;
//    case MCUSTATE_PLL_LOCK:
//#if DEBUG_ALARM
//      trace_puts("  - MZU_DONE timeout" );
//#endif
//// Переходим с ошибкой дальше
//      ledOn( LED_BM_ERR, 0 );
//      mcuRunWait = MSTATE_NON;
//      mcuState = MCUSTATE_MZU_DONE;
//      return;
//      break;
//    case MCUSTATE_ISTART:
//#if DEBUG_ALARM
//        trace_puts("  - Intel not wake" );
//#endif
//        // INTEL не включился - выключаемся
//        intel.iErr = I_ERR_SLP;
//        sysFailOffQuery = SET;
//        logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_EN_CPU, 1 );
//        return;
//      break;
////    case MCUSTATE_FAN_ON:
////#if DEBUG_ALARM
////        trace_puts("  - PLL_LOCK timeout" );
////#endif
////        ledOn( LED_R, 0 );
////        errHandler(ERR_SYS_ON_TOUT);
////// Переходим с ошибкой дальше
////        mcuRunWait = MSTATE_NON;
////        mcuState++;
////        return;
////      break;
//    case MCUSTATE_FPGA_START: {
//#if DEBUG_ALARM
//      trace_puts("  - no PG_FPGA_RUN_1" );
//#endif
//      fpgaAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_FPGA_EN_1, 1 );
//      return;
//    }
//    case MCUSTATE_FPGA_1:
//#if DEBUG_ALARM
//      trace_puts("  - no PG_FPGA_RUN_2" );
//#endif
//      fpgaAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_FPGA_EN_2, 1 );
//      return;
//    case MCUSTATE_FPGA_2:
//#if DEBUG_ALARM
//      trace_puts("  - no PG_FPGA_RUN_3" );
//#endif
//      fpgaAlrtOffQuery = SET;
//      logger( DEVID_OTHER, 0, LOG_ERR_FLAG, PRM_GPIO_PWR_FPGA_EN_3, 1 );
//      return;
//    case MCUSTATE_FPGA_3:
//#if DEBUG_ALARM
//        trace_puts("  - FPGA_DONE timeout" );
//#endif
//        errHandler(ERR_SYS_ON_TOUT);
//// Переходим с ошибкой дальше
//        mcuRunWait = MSTATE_NON;
//        mcuState++;
//        return;
//      break;
//    default:
//      break;
//  }
//  sysFailOffQuery = SET;
}


/**
  * @brief  Обработчик тайм-аута выключения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void pwrOffTout(uintptr_t arg){
}

/**
  * @brief  Обработчик тайм-аута включения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void mesurOnUpdate(uintptr_t arg){
  (void)arg;
  // Пауза закончилась - переходим в предыдущее состояние системы
  mesurState++;
  mesurRunWait = MSTATE_NON;
}

/**
  * @brief  Обработчик тайм-аута выключения системы
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void mesurOffUpdate(uintptr_t arg){
  (void)arg;
  // Пауза закончилась - переходим в предыдущее состояние системы
  mesurRunWait = MSTATE_NON;
  if( mesurState > MCUSTATE_SYS_OFF ){
    mesurState--;
  }
  else {
    mesurOnCan( (uintptr_t)NULL );
  }
}


/**
  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
static void debounceTimeout(uintptr_t arg){
  sGpioPin *pin = (sGpioPin*)arg;
  bool st;

  // Нынешнее состояния пина
  st = ((((pin->gpio)->IDR & (pin->pin))) == pin->pin);

  if( pin->newstate != st ){
    // Состояние сохранилось - НЕ ложное срабатывание
    pin->newstate = pin->state = st;
    pin->change = SET;
  }

}


/*
void KEY_TIM_IRQH( void ) {
  static sGpioPin * key;

  if( TIM_GetITStatus(KEY_TIM, TIM_IT_Update) != RESET ){
    // Прошло ~30мс
    debounceTimeout( (uintptr_t)key );
    KEY_TIM->DIER = (KEY_TIM->DIER & ~TIM_DIER_UIE) | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
  }
  else if( TIM_GetITStatus(KEY_TIM, TIM_IT_CC1) != RESET ){
    // Получили событие от кнопки RstKey- запускаем таймер дебонса
    KEY_TIM->EGR = TIM_EGR_UG;
    KEY_TIM->DIER |= TIM_DIER_UIE;
    key = &gpioPinRstKey;
  }
  else if( TIM_GetITStatus(KEY_TIM, TIM_IT_CC2) != RESET ){
    // Получили событие от кнопки PwrKey- запускаем таймер дебонса
    KEY_TIM->EGR = TIM_EGR_UG;
    KEY_TIM->DIER |= TIM_DIER_UIE;
    key = &gpioPinPwrKey;
  }
  else if( TIM_GetITStatus(KEY_TIM, TIM_IT_CC3) != RESET ){
    // Получили событие от кнопки FpgaRstKey- запускаем таймер дебонса
    KEY_TIM->EGR = TIM_EGR_UG;
    KEY_TIM->DIER |= TIM_DIER_UIE;
    key = &gpioPinFpgaRstKey;
  }
  else if( TIM_GetITStatus(KEY_TIM, TIM_IT_CC4) != RESET ){
    // Получили событие от кнопки FpgaPwrKey- запускаем таймер дебонса
    KEY_TIM->EGR = TIM_EGR_UG;
    KEY_TIM->DIER |= TIM_DIER_UIE;
    key = &gpioPinFpgaPwrKey;
  }

  KEY_TIM->SR = 0;
}


void CLR_KEY_TIM_IRQH( void ) {
  static sGpioPin * key;

  if( TIM_GetITStatus(CLR_KEY_TIM, TIM_IT_Update) != RESET ){
    // Прошло ~30мс
    debounceTimeout( (uintptr_t)key );
    CLR_KEY_TIM->DIER = (CLR_KEY_TIM->DIER & ~TIM_DIER_UIE) | TIM_DIER_CC1IE;
  }
  else if( TIM_GetITStatus(CLR_KEY_TIM, TIM_IT_CC1) != RESET ){
    // Получили событие от кнопки PwrKey- запускаем таймер дебонса
    CLR_KEY_TIM->EGR = TIM_EGR_UG;
    CLR_KEY_TIM->DIER |= TIM_DIER_UIE;
    key = &gpioPinClrKey;
  }

  CLR_KEY_TIM->SR = 0;
}
*/

/*
void FRST_KEY_TIM_IRQH( void ) {
  FRST_KEY_TIM->SR = 0;
  debounceTimeout( (uintptr_t)&extiPinFpgaRstKey );
}
*/

void REL_PULSE_TIM_IRQH( void ) {
  REL_PULSE_TIM->SR = 0;
  // Снимаем сброс по питанию MZU_RST
  gpioPinResetNow( &gpioPinRelOn );
}


/**
  * @brief  Настройка таймера сброса сигнала MZU_RST, FPGA_RST.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  * @param[in]  tout длина пульса в 100мкс
  *
  * @retval none
  */
void pulseTimInit( TIM_TypeDef * portim, uint16_t tout ){
  FlagStatus rc;
  uint16_t psc;

  if( portim == REL_PULSE_TIM ){
    RCC->APB1ENR |= REL_PULSE_TIM_CLK_EN;
  }

  rc = timPscSet( portim, 100000, &psc );
#ifdef  USE_FULL_ASSERT
  assert_param( rc == RESET );
#else
  (void)rc;
#endif /* USE_FULL_ASSERT */
  portim->PSC = psc;
  // Время работы таймера 100мкс .
  portim->ARR = ( 10 * tout ) -1;
  portim->CR1 |= TIM_CR1_OPM | TIM_CR1_URS;
  portim->EGR |= TIM_EGR_UG;
  portim->DIER |= TIM_DIER_UIE;

  if( portim == REL_PULSE_TIM ){
    NVIC_EnableIRQ( REL_PULSE_TIM_IRQn );
    NVIC_SetPriority( REL_PULSE_TIM_IRQn, 6 );
  }
}



/**
  * @brief  Настройка таймера антидребезга кнопки.
  *
  * @param[in]  arg данные таймера (дескриптор таймера)
  *
  * @retval none
  */
void keyTimInit( TIM_TypeDef * keytim ){
  FlagStatus rc;
  uint16_t psc;

  rc = timPscSet( keytim, 100000, &psc );
#ifdef  USE_FULL_ASSERT
  assert_param( rc == RESET );
#else
  (void)rc;
#endif /* USE_FULL_ASSERT */
  keytim->PSC = psc;
  // Время работы таймера 30мс = 0.030с / (1/10000Гц) .
  keytim->ARR = ( 100000/1000 * 30  ) -1;
  keytim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  keytim->CCMR2 |=  TIM_CCMR2_CC3S_0 | TIM_CCMR2_CC4S_0;
  keytim->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E
                  | TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2E
                  | TIM_CCER_CC3P | TIM_CCER_CC3NP | TIM_CCER_CC3E
                  |TIM_CCER_CC4P | TIM_CCER_CC4NP | TIM_CCER_CC4E;
  keytim->CR1 |= TIM_CR1_URS;
  keytim->EGR |= TIM_EGR_UG;
  keytim->CR1 |= TIM_CR1_CEN;
}


///**
//  * @brief  Настройка таймера антидребезга кнопки.
//  *
//  * @param[in]  arg данные таймера (дескриптор таймера)
//  *
//  * @retval none
//  */
//void keyClrTimInit( TIM_TypeDef * keytim ){
//  FlagStatus rc;
//  uint16_t psc;
//
//  rc = timPscSet( keytim, 100000, &psc );
//#ifdef  USE_FULL_ASSERT
//  assert_param( rc == RESET );
//#else
//  (void)rc;
//#endif /* USE_FULL_ASSERT */
//  keytim->PSC = psc;
//  // Время работы таймера 30мс = 0.030с / (1/10000Гц) .
//  keytim->ARR = ( 100000/1000 * 30  ) -1;
//  keytim->CCMR1 |= TIM_CCMR1_CC1S_0;
//  keytim->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E;
//  keytim->CR1 |= TIM_CR1_URS;
//  keytim->EGR |= TIM_EGR_UG;
//  keytim->CR1 |= TIM_CR1_CEN;
//}
//
//void keyInit( void ){
//  RCC->APB1ENR |= KEY_TIM_CLK_EN;
//  RCC->APB2ENR |= CLR_KEY_TIM_CLK_EN;
//
//  // AF конфигурация вывода кнопок
//  GPIO_PinAFConfig( gpioPinPwrKey.gpio, gpioPinNum(gpioPinPwrKey.pin), KEY_AF_TIM);
//  GPIO_PinAFConfig( gpioPinRstKey.gpio, gpioPinNum(gpioPinRstKey.pin), KEY_AF_TIM);
//  GPIO_PinAFConfig( gpioPinFpgaPwrKey.gpio, gpioPinNum(gpioPinFpgaPwrKey.pin), KEY_AF_TIM);
//  GPIO_PinAFConfig( gpioPinFpgaRstKey.gpio, gpioPinNum(gpioPinFpgaRstKey.pin), KEY_AF_TIM);
//  GPIO_PinAFConfig( gpioPinClrKey.gpio, gpioPinNum(gpioPinClrKey.pin), CLR_KEY_AF_TIM);
//
//  keyTimInit( KEY_TIM );
//  keyClrTimInit( CLR_KEY_TIM );
////  keyTimInit( RST_KEY_TIM );
////  keyTimInit( FRST_KEY_TIM );
//
//  enable_nvic_irq( KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
//  enable_nvic_irq( CLR_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
////  enable_nvic_irq( RST_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
////  enable_nvic_irq( FRST_KEY_TIM_IRQn, KEY_IRQ_PRIORITY);
//}
//
//
//void keyEnable( void ){
//
//  KEY_TIM->SR = 0;
//  KEY_TIM->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
//  CLR_KEY_TIM->SR = 0;
//  CLR_KEY_TIM->DIER |= TIM_DIER_CC1IE;
//
//}


void gpioIrqHandler5_9( uint32_t pin ){
  EXTI->PR = pin;
}


void gpioIrqHandler10_15( uint32_t pin ){
EXTI->PR = pin;
}


/**
  * @brief  Запуск пульса FPGA_RESET.
  *
  * @param[in]  none
  *
  * @retval none
  */
void gpioPulse( sGpioPin * pin ){
  if( pin == &gpioPinRelOn ){
    gpioPinSetNow( pin );
    REL_PULSE_TIM->EGR = TIM_EGR_UG;
    REL_PULSE_TIM->CR1 |= TIM_CR1_CEN;
    relOnSet = RESET;
    relOnCount++;
  }
}


/**
  * @brief  Обработка данных интерфейса GPIO.
  *
  * @param[in]  none
  *
  * @retval none
  */
void gpioClock( void ){

#if DEBUG_TRACE
  trace_puts("Function: Clock FPGA");
#endif

  //  Обработаем кнопки.
  if( gpioPinClrKey.change  \
      && (gpioPinClrKey.state == Bit_SET) \
      && (adcHandle.batOff == RESET) ){
    // Нажата кнопка CMOS-Clear - выставим флаг выключения батареи
    gpioPinClrKey.change = RESET;
    *(adcHandle.clrBatFlag) = SET;
    if( (ledHandle[LED_R].ledOnTout == 0) & (ledHandle[LED_R].ledPin.state == 0)){
      ledOn( LED_R, LED_BLINK_ON_TOUT );
    }
  }

  // UP_PWRKey
  if (gpioPinPwrKey.change) {
    // Система не включена (или не включена полностью) - меняем направление запуска
    gpioPinPwrKey.change = RESET;
    if( mcuState >= MCUSTATE_SYS_ON ){
      // Система включена - транслируем кнопку на CPU (INTEL) с инверсией
      transBtn( &gpioPinPwrOut, !gpioPinPwrKey.state );
    }
    else if (gpioPinPwrKey.state == Bit_RESET){
      if( mcuRun ){
        sysOffQuery = SET;
      }
      else if( onCan ){
        mcuOnNeed = SET;
      }
    }
  }

  // FPGA_PWRKey
  if (gpioPinFpgaPwrKey.change) {
    // VP1, VP2 не включены (или не включена полностью) - меняем направление запуска
    gpioPinFpgaPwrKey.change = RESET;
    if( (gpioPinFpgaPwrKey.state == Bit_RESET) && (mcuState >= MCUSTATE_SYS_ON) ){
      if( fpgaRun ){
        fpgaOffQuery = SET;
      }
      else if( fpgaOnCan ){
        fpgaOnNeed = SET;
      }
    }
  }


  // I_PwrBtnOut pulse
  if( sysFlags.iPwrOutSet ){
    gpioPulse( &gpioPinPwrOut );
  }

  if( extiPinMzuRsrv1.change ){
    // Пришел отклик на сигнал FPGA_RST
    sysFlags.mzuRsrv1Evnt = SET;
    timerDel( &mzuRstTestTimer );
  }

/*
  // --------- MultyBoot --------------
  if( (mcuRun != RESET) && (mcuState >= MCUSTATE_RUN_3) ){
    // MZU включен
    gpioPinReadNow( &gpioPinMzumbFin );
    if( gpioPinMzumbFin.change && (gpioPinMzumbFin.state == Bit_SET) ){
      // Multiboot закончен - проверяем
      if( gpioPinReadNow( &gpioPinMzuMbId ) != gpioPinMzuMb.state ){
        sysFlags.mbErr = SET;
      }
    }
  }
  // ----------------------------------
*/

  // INTEL запущен
  if( (mcuRun != RESET) && (mcuState >= MCUSTATE_SYS_ON) ){
    // RST_Key
    if (gpioPinRstKey.change) {
      gpioPinRstKey.change = RESET;
      if( gpioPinRstKey.state == Bit_RESET ){
        sysFlags.intelRstSet = SET;
      }
    }
    if(  sysFlags.intelRstSet == SET ){
      // PS_SRST сработало от кнопки KEY_SRST или от UART
      gpioPulse( &gpioPinIRst );
    }

    // FPGA_RST_Key
    if (gpioPinFpgaRstKey.change) {
      gpioPinFpgaRstKey.change = RESET;
      if( gpioPinFpgaRstKey.state == Bit_RESET ){
        sysFlags.mzuRstSet = SET;
      }
    }
    if(  sysFlags.mzuRstSet == SET ){
      // MZU_RST сработало от кнопки FPGA_RST или от UART
      gpioPulse( &gpioPinMzuRst );
      timerMod( &mzuRstTestTimer, 3 );
    }
    // MZU_PROG pulse
    if(  sysFlags.mzuProgSet == SET ){
      // MZU_PROG от UART
      gpioPulse( &gpioPinMzuProg );
    }
    // FPGA_PROG pulse
    if(  sysFlags.fpgaProgSet == SET ){
      // FPGA_PROG сработало от UART
      gpioPulse( &gpioPinFpgaProg );
    }

    // Отслеживаем состояние INTEL - SlpS3
    gpioPinReadNow( &gpioPinSlpS[SLP_S3] );
    if( gpioPinSlpS[SLP_S3].change && (gpioPinSlpS[SLP_S3].state == Bit_RESET) ){
      // INTEL начал засыпать или выключаться - выключаем систему
      gpioPinSlpS[SLP_S3].change = RESET;
      extiPgCoreOff();
      sysOffQuery = SET;
    }
    else {
      // ================== INTEL_CATERROR =========================
      gpioPinReadNow( &gpioPinCaterr );
      if( gpioPinCaterr.change && (gpioPinCaterr.state == Bit_RESET) ){
        // INTEL CATERR;
        gpioPinCaterr.change = RESET;
        sysFlags.caterr = SET;
        intel.caterrBatFlag = SET;

        logger( DEVID_CPU, 0, LOG_ERR_FLAG, PRM_GPIO_CATERR, intel.caterrBatFlag );

#if LOGGER_ENABLE
        logCaterrSave( &logDev, sysFlags.caterr );
#endif // LOGGER_ENABLE

        ledOn( LED_BM_ERR, 0 );
      }
      // ===========================================================
    }

  }
}

/**
  * @brief	Разрешение работы интерфейса GPIO.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void gpioEnable( void ) {
#if DEBUG_TRACE
  trace_puts("Function: Enable FPGA");
#endif

  // Включаем прерывания для кнопки PS_SRST
//  EXTI->IMR |= extiPinRstKey.pin;
  timerMod( &pwrOnCanTimer, TOUT_1500 );
  keyEnable();
}

/**
  * @brief	Инициализация интерфейса GPIO.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void gpioInit( void ){

#if DEBUG_TRACE
  trace_puts("Function: Init GPIO");
#endif

  // --------------------- KEYS ------------------------
  extiPinSetup( &extiPinBcmWkup );
  /** Структура дескриптора вывода GPIO RSTKey. */
  gpioPinSetup( &gpioPinRstKey );
  /** Структура дескриптора вывода GPIO PWRKey. */
  gpioPinSetup( &gpioPinPwrKey );
  /** Структура дескриптора вывода GPIO FPGA_RSTKey. */
  gpioPinSetup( &gpioPinFpgaRstKey );
  /** Структура дескриптора вывода GPIO FPGA_PWRKey. */
  gpioPinSetup( &gpioPinFpgaPwrKey );
  /** Структура дескриптора вывода GPIO UP_CLRKey. */
  gpioPinSetup( &gpioPinClrKey );
  keyInit();

  // -------------- POWER ------------------------------
  // RUNs
  for( eRun run = 0; run < RUN_NUM; run++ ){
    gpioPinSetup( &gpioPinRun[run] );
  }

  gpioPinSetup( &gpioPinPg4v );  //PG_4V0
  gpioPinSetup( &gpioPinPgCore );  //PG_CORE

  // PGOOD
  for( ePgood pg = 0; pg < PG_NUM; pg++ ){
    extiPinSetup( &extiPinPg[pg] );
  }

  // --------------- MZU --------------------------
  /** Структуры дескрипторов выводов GPIO MZU_XXX */
  gpioPinSetup( &gpioPinMzuRst );
  // Длительность пульса 100мкс
  pulseTimInit( MZU_RST_TIM, 1 );
  gpioPinSetup( &gpioPinMzuPllLock );
  gpioPinSetup( &gpioPinMzuDone );
  gpioPinSetup( &gpioPinMzuWork );
  gpioPinSetup( &gpioPinMzuMbFin );
  gpioPinSetup( &gpioPinMzuMbId );
  gpioPinSetup( &gpioPinMzuMbSel );

  gpioPinSetup( &gpioPinMzuMb );
  gpioPinSetup( &gpioPinMzuProg );

  extiPinSetup( &extiPinMzuRsrv1 );
  gpioPinSetup( &gpioPinMzuRsrv2 );

  gpioPinSetup( &gpioPinFpgaProg );

//  gpioPinSetup( &gpioPinFpgaDone );
  gpioPinSetup( &gpioPinJtag1Sens );
  gpioPinSetup( &gpioPinJtag2Sens );

  // ---------------- DSW ---------------------------
  /** Структура дескриптора вывода GPIO UP_DSW_+3.3V. */
  gpioPinSetup( &gpioPinDsw );

  // ---------------- INTEL -------------------------
  // INTEL POWER
  gpioPinSetup( &gpioPinPwrOut );
  /** Структура дескриптора вывода GPIO UP_I_RST. */
  // Длительность пульса 100мс
  pulseTimInit( INTEL_POUT_TIM, 1000 );

  gpioPinSetup( &gpioPinIRst );
  // Длительность пульса 100мс
  pulseTimInit( INTEL_RST_TIM, 1000 );

  for( eIntelSlp slp = 0; slp < SLP_NUM; slp++ ){
    gpioPinSetup( &gpioPinSlpS[slp] );
  }
  /** Структура дескриптора вывода GPIO UP_I_CATERR. */
  gpioPinSetup( &gpioPinCaterr );

  // --------------- PLL ---------------------------
  gpioPinSetup( &gpioPinPllPd );
  gpioPinSetup( &gpioPinPllSync );
  extiPinSetup( &extiPinPllLock );
  EXTI->IMR &= ~extiPinPllLock.pin;

  // --------------- I2C ----------------------------
  //  Выводы I2Cx_RST_X (Сброс MAX7357)
  gpioPinSetup( &gpioPinI2cRst[I2C_RST_1] );
  gpioPinSetup( &gpioPinI2cRst[I2C_RST_2] );
  gpioPinSetup( &gpioPinFault[FPGA_VP1] );
  gpioPinSetup( &gpioPinFault[FPGA_VP2] );

  // --------------------- CMOS_BAT -----------------------------------------------------
  /** Структуры дескрипторов выводов GPIO MCU_BAT_OFF */
  gpioPinSetup( &gpioPinBatOff );
  /** Структуры дескрипторов выводов GPIO BAT_DISCHARGE */
  gpioPinSetup( &gpioPinBatDisch );

  // -------------------- DBG_JMP -------------------
  gpioPinSetup( &gpioPinDbgJmp1 );
  gpioPinSetup( &gpioPinDbgJmp2 );

  // ------------------- LEDS -----------------------
  ledInit();

  // ----------- TIMERS ---------------------------
  timerSetup( &pwrOnToutTimer, pwrOnTout, (uintptr_t)NULL);
  timerSetup( &pwrOffToutTimer, pwrOffTout, (uintptr_t)NULL);
  timerSetup( &pwrOnUpdateTimer, pwrOnUpdate, (uintptr_t)NULL);
  timerSetup( &pwrOffUpdateTimer, pwrOffUpdate, (uintptr_t)NULL);

  timerSetup( &pwrOnCanTimer, pwrOnCan, (uintptr_t)NULL );
  timerSetup( &fpgaOnCanTimer, fpgaOnCanTout, (uintptr_t)NULL );

  timerSetup( &mzuRstTestTimer, rstTestTout, (uintptr_t)&gpioPinMzuRst );
}


