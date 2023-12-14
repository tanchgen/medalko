#include <stddef.h>
#include <stdbool.h>
#include <limits.h>

#include "main.h"
#include "statefunc.h"
#include "measur.h"
#include "gpio_arch.h"

#if SIMUL
  extern uint32_t simulStart;
#endif // SIMUL

sGpioPin gpioPinAlcoRst = {GPIOA, 0, GPIO_Pin_3, 3, GPIO_MODE_OPP_10, GPIO_NOPULL, Bit_SET, Bit_SET, RESET };
sGpioPin gpioPinRelEn = {GPIOA, 0, GPIO_Pin_5, 5, GPIO_MODE_OPP_10, GPIO_NOPULL, Bit_RESET, Bit_RESET, RESET };
sGpioPin gpioPinRelOn = {GPIOA, 0, GPIO_Pin_4, 4, GPIO_MODE_OPP_10, GPIO_NOPULL, Bit_RESET, Bit_RESET, RESET };
sGpioPin gpioPinZoom = {GPIOA, 0, GPIO_Pin_8, 8, GPIO_MODE_AFPP_10, GPIO_NOPULL, Bit_RESET, Bit_RESET, RESET };

#if PIN_TEST_EN
sGpioPin gpioPinTest = {GPIOB, 1, GPIO_Pin_0, 0, GPIO_MODE_OPP_10, GPIO_NOPULL, Bit_RESET, Bit_RESET, RESET };
#endif // PIN_TEST_EN

FlagStatus relOnSet;
uint8_t relOnCount;

// ------------------- LEDS -----------------------
// Определено в led.c

// ===========================================================================================================
///** Структура дескриптора таймера таймаута изменения состояния системы при включении */
//struct timer_list  pwrOnToutTimer;
///** Структура дескриптора таймера таймаута изменения состояния системы при выключении */
//struct timer_list  pwrOffToutTimer;
///** Структура дескриптора таймера изменения состояния системы при включении*/
//struct timer_list  pwrOnUpdateTimer;
/** Структура дескриптора таймера сброса АЛКОМЕТРА */
struct timer_list  alcoOffTimer;

/** Структура дескриптора таймера таймаута восстановления после ошибки */
struct timer_list  measOnCanTimer;

// =================== Прототипы функций ====================================
// ==========================================================================


/**
  * @brief  Обработчик таймаута восстановления после ошибки
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
static void measOnCan(uintptr_t arg){
  (void)arg;
  // Система выключена полностью и готова к повторному включению
  if( measDev.prmContinuous ){
    continueStart();
  }
  else {
    onCan = SET;
  }
#if SIMUL
  simulStart = mTick + 2000;
#endif // SIMUL
}


/**
  * @brief  Обработчик таймаута работы АЛКОМЕТРА
  *
  * @param[in]  arg  NULL
  *
  * @retval none
  */
//static void alcoOffTout(uintptr_t arg){
//  (void)arg;
//  gpioPinSetNow( &gpioPinAlcoRst );
//}


///**
//  * @brief  Обработчик тайм-аута антидребезга выводов GPIO.
//  *
//  * @param[in]  arg данные таймера (дескриптор таймера)
//  *
//  * @retval none
//  */
//static void debounceTimeout(uintptr_t arg){
//  sGpioPin *pin = (sGpioPin*)arg;
//  bool st;
//
//  // Нынешнее состояния пина
//  st = ((((pin->gpio)->IDR & (pin->pin))) == pin->pin);
//
//  if( pin->newstate != st ){
//    // Состояние сохранилось - НЕ ложное срабатывание
//    pin->newstate = pin->state = st;
//    pin->change = SET;
//  }
//
//}
//

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
  // Выключаем соленоид
  gpioPinResetNow( &gpioPinRelOn );
  measDev.status.relEnd = SET;
  measDev.rel = RESET;
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


/* ZOOMER_TIM init function */
void zoomTimInit( void ){
  ZOOM_TIM_CLK_EN;
  gpioPinSetup( &gpioPinZoom );

  ZOOM_TIM->PSC = (720-1);
  ZOOM_TIM->ARR = ((rccClocks.PCLK2_Frequency/(ZOOM_TIM->PSC + 1)) / 1000) - 1;      // Частота ШИМ 1кГц
  ZOOM_TIM->CCR1 = (ZOOM_TIM->ARR + 1) / 2;

  ZOOM_TIM->CR2 |= TIM_CR2_OIS1;
  // ШИМ режим 110, CCR1 - preload
  ZOOM_TIM->CCMR1 = (ZOOM_TIM->CCMR1 & ~TIM_CCMR1_CC1S) | TIM_CCMR1_OC1M_2  | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
  ZOOM_TIM->CCER = TIM_CCER_CC1E;
  // Контроль выводов при выключении таймера:
  ZOOM_TIM->BDTR = TIM_BDTR_AOE | TIM_BDTR_MOE | TIM_BDTR_OSSI | TIM_BDTR_OSSR;
}


void zoomOn( void ){
  // ZOOM_TIM->CR1 |= TIM_CR1_CEN;
}

void zoomOff( void ){
  ZOOM_TIM->CR1 &= ~TIM_CR1_CEN;
}


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
    measDev.status.relStart = RESET;
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
  if( measDev.status.relStart ){
    measDev.tout = mTick + measDev.relPulse;
    // Отключаем источник питания от соленоида
    gpioPinResetNow( &gpioPinRelEn );
    gpioPulse( &gpioPinRelOn );
    measDev.status.relStart = RESET;
  }
//  else if( measDev.status.relEnd ){
//    measDev.tout = mTick + measDev.relPulse;
//    gpioPulse( &gpioPinRelOn );
//  }

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

  timerMod( &measOnCanTimer, TOUT_1500 );
  gpioPinSetNow( &gpioPinRelEn );
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

  gpioPinSetup( &gpioPinRelEn );
  gpioPinSetup( &gpioPinRelOn );

#if PIN_TEST_EN
  gpioPinSetup( &gpioPinTest );
#endif // PIN_TEST_EN

  pulseTimInit( REL_PULSE_TIM, measDev.relPulse * 10 );
  zoomTimInit();

  // ----------- TIMERS ---------------------------
  timerSetup( &measOnCanTimer, measOnCan, (uintptr_t)NULL );
//  timerSetup( &alcoOffTimer, alcoOffTout, (uintptr_t)NULL);
//  timerSetup( &pwrOffToutTimer, pwrOffTout, (uintptr_t)NULL);
}


