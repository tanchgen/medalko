#include <stddef.h>

//#include "main.h"
#ifdef STM32F4XX
#include "stm32f4xx.h"
#else
#include "stm32f2xx.h"
#endif // STM32F4XX

#include "times.h"

volatile uint32_t  mTick = 0;

const uint8_t SynchPrediv = 0xFF;
const uint8_t AsynchPrediv = 0x7F;

/** Голова очереди таймеров на исполнение. */
static struct list_head  _timers_queue = LIST_HEAD_INIT(_timers_queue);

#define RTC_RSF_MASK            ((uint32_t)0xFFFFFF5F)
/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
/* --- BDCR Register ---*/
/* Alias word address of RTCEN bit */
#define BDCR_OFFSET               (RCC_OFFSET + 0x70)
#define RTCEN_BitNumber           0x0F
#define BDCR_RTCEN_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (RTCEN_BitNumber * 4))
/* Alias word address of BDRST bit */
#define BDRST_BitNumber           0x10
#define BDCR_BDRST_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (BDRST_BitNumber * 4))
/* Alias word address of LSEON bit */
#define LSEON_BitNumber           0x0
#define BDCR_LSEON_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (LSEON_BitNumber * 4))

#define BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))
#define BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

volatile tRtc rtc;
volatile tUxTime uxTime;
volatile uint8_t sendToutFlag = SET;
volatile uint8_t minTout;
volatile uint8_t minToutRx;
volatile uint8_t uxSecTout;

static void RTC_SetTime( volatile tRtc * prtc );
//static void RTC_GetTime( volatile tRtc * prtc );
static void RTC_SetDate( volatile tRtc * prtc );
static void RTC_GetDate( volatile tRtc * prtc );
static void RTC_SetAlrm( tRtc * prtc );
static void RTC_GetAlrm( tRtc * prtc );
static void RTC_CorrAlrm( tRtc * prtc );

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void){
  // **************** RTC Clock configuration ***********************
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;
  RCC_BackupResetCmd( SET );
  RCC_BackupResetCmd( RESET );
//  RCC->BDCR |= RCC_BDCR_BDRST;
//  RCC->BDCR &= ~RCC_BDCR_BDRST;

  RCC->BDCR |= RCC_BDCR_LSEON;

  while( ( PWR->CR & PWR_CR_DBP) == 0 )
  {}
  while((RCC->BDCR & RCC_BDCR_LSERDY)!=RCC_BDCR_LSERDY)
  {}

  RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0;
  RCC->APB1ENR &=~ RCC_APB1ENR_PWREN;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}
  RTC->PRER = 0x007F00FF;
  RTC->ISR &= ~RTC_ISR_INIT;

  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;

//  EXTI->IMR |= EXTI_IMR_MR17;
//  EXTI->RTSR |= EXTI_RTSR_TR17;
//  NVIC_SetPriority(RTC_WKUP_IRQn, 0);
//  NVIC_EnableIRQ(RTC_WKUP_IRQn);

  /* Enable the RTC Clock */
  *(__IO uint32_t *) BDCR_RTCEN_BB = SET;
}

void timeInit( void ) {
  //Инициализируем RTC
  rtcInit();

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Saturday February 15th 2018 */
  rtc.year = 18;
  rtc.month = 2;
  rtc.date = 15;
  rtc.wday = 4;
  rtc.hour = 12;
  rtc.min = 0;
  rtc.sec = 0;;

  while( (RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  RTC_SetDate( &rtc );
  RTC_SetTime( &rtc );
  // Интервал будильника - измерение минуты
  minTout = 1;
  // Интервал будильника - передача минуты
  minToutRx = 1;
  // Интервал будильника - секунды
  uxSecTout = 10;
  while( RTC->DR == 0x2101 )
  {}

  /* Disable the write protection for RTC registers */
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  /* Clear RSF flag */
  RTC->ISR &= (uint32_t)RTC_RSF_MASK;

  /* Wait the registers to be synchronised */
  do
  {
  } while( (RTC->ISR & RTC_ISR_RSF) == RESET );

  /* Enable the write protection for RTC registers */
  RTC->WPR = 0xFF;

  uxTime = getRtcTime();
}

// Получение системного мремени
uint32_t getTick( void ) {
  // Возвращает количество тиков
  return mTick;
}

#define _TBIAS_DAYS   ((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS   (_TBIAS_DAYS * (uint32_t)86400)
#define _TBIAS_YEAR   0
#define MONTAB(year)    ((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define Daysto32(year, mon) (((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

tUxTime xTm2Utime( volatile tRtc * prtc ){
  /* convert time structure to scalar time */
int32_t   days;
int32_t   secs;
int32_t   mon, year;

  /* Calculate number of days. */
  mon = prtc->month - 1;
  // Годы считаем от 1900г.
  year = (prtc->year + 100) - _TBIAS_YEAR;
  days  = Daysto32(year, mon) - 1;
  days += 365 * year;
  days += prtc->date;
  days -= _TBIAS_DAYS;

  /* Calculate number of seconds. */
  secs  = 3600 * prtc->hour;
  secs += 60 * prtc->min;
  secs += prtc->sec;

  secs += (days * (tUxTime)86400);

  return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( volatile tRtc * prtc, tUxTime secsarg){
  uint32_t    secs;
  int32_t   days;
  int32_t   mon;
  int32_t   year;
  int32_t   i;
  const int16_t * pm;

  #ifdef  _XT_SIGNED
  if (secsarg >= 0) {
      secs = (uint32_t)secsarg;
      days = _TBIAS_DAYS;
    } else {
      secs = (uint32_t)secsarg + _TBIAS_SECS;
      days = 0;
    }
  #else
    secs = secsarg;
    days = _TBIAS_DAYS;
  #endif

    /* days, hour, min, sec */
  days += secs / 86400;
  secs = secs % 86400;
  prtc->hour = secs / 3600;
  secs %= 3600;
  prtc->min = secs / 60;
  prtc->sec = secs % 60;

  prtc->wday = (days + 1) % 7;

  /* determine year */
  for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
  days -= i;
  // Годы выставляем от эпохи 2000г., а не 1900г., как в UNIX Time
  prtc->year = (year - 100) + _TBIAS_YEAR;

    /* determine month */
  pm = MONTAB(year);
  for (mon = 12; days < pm[--mon]; );
  prtc->month = mon + 1;
  prtc->date = days - pm[mon] + 1;
}

void setRtcTime( tUxTime xtime ){

  xUtime2Tm( &rtc, xtime);
  RTC_SetTime( &rtc );
  RTC_SetDate( &rtc );
}

tUxTime getRtcTime( void ){

  RTC_GetTime( &rtc );
  RTC_GetDate( &rtc );
  return xTm2Utime( &rtc );
}

uint8_t getRtcMin( void ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  return BCD2BIN( (RTC->TR >> 8) & 0x7F );
}

/* Установка будильника
 *  xtime - UNIX-времени
 *  alrm - номере будильника
 */
void setAlrm( tUxTime xtime){
  tRtc tmpRtc;

  xUtime2Tm( &tmpRtc, xtime);
  RTC_SetAlrm( &tmpRtc );
}

/* Получениевремени будильника
 *  alrm - номере будильника
 *  Возвращает - UNIX-время
 */
tUxTime getAlrm( void ){
  tRtc tmpRtc;

  RTC_GetAlrm( &tmpRtc );
  return xTm2Utime( &tmpRtc );
}

/* Коррекция будильника в соответствии с реалиями занятости канала:
 * В следующий раз будем пробовать отправлять данные именно в это значение секунд,
 * раз именно сейчас канал свободен.
 */
void correctAlrm( void ){
  tRtc tmpRtc;

  // Получим текущее время, заодно обновим глобальное значение
  uxTime = getRtcTime();
  xUtime2Tm( &tmpRtc, uxTime);
  RTC_CorrAlrm( &tmpRtc );
}

void setAlrmSecMask( uint8_t secMask ){
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &=~ RTC_CR_ALRAE;
  while ((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
  {}
  // Alarm A every day, every hour, every minute, every second
  if( secMask ){
    RTC->ALRMAR |= RTC_ALRMAR_MSK1;
  }
  else {
    RTC->ALRMAR &= ~RTC_ALRMAR_MSK1;
  }
  RTC->CR = RTC_CR_ALRAIE | RTC_CR_ALRAE;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void timersHandler( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 > 1) {
    timerCount1--;
  }
  // Таймаут timerCount2
  if ( timerCount2 > 1) {
    timerCount2--;
  }
  // Таймаут timerCount3
  if ( timerCount3 > 1) {
    timerCount3--;
  }
  if ( !(mTick % 1000) ){
    secondFlag = SET;
  }
#endif
}

#if 0
void timersProcess( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 == 0) {
    timerCount1 = TOUTCOUNT1;
  }
  // Таймаут timerCount2
  if ( timerCount2 == 0) {
    timerCount2 = TOUTCOUNT2;
  }
  // Таймаут timerCount3
  if ( timerCount3 == 3) {
    timerCount3 = TOUTCOUNT3;
  }
#endif
  if (secondFlag) {
    secondFlag = RESET;
  }
}
#endif

#if 0
// Задержка по SysTick без прерывания
void mDelay( uint32_t t_ms ){
  SysTick->VAL = 0;
  while ( t_ms > 0 ){
    while ( !( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) ) // wait for underflow
    {}
    t_ms--;
  }
}
#else
// Задержка в мс
void mDelay( uint32_t del ){
  uint32_t finish = mTick + del;
  while ( mTick < finish)
  {}
}
#endif

static void RTC_SetTime( volatile tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->hour ) << 16 |
          BIN2BCD( prtc->min ) << 8 |
          BIN2BCD( prtc->sec ) );
  // Time reserved mask
  temp &= (uint32_t)0x007F7F7F;
  RTC->TR = ( RTC->TR & (RTC_TR_PM | RTC_TR_HT | RTC_TR_HU | RTC_TR_MNT | RTC_TR_MNU | RTC_TR_ST | RTC_TR_SU)) | temp;
  // Сбрасываем флаг синхронизации часов
  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


static void RTC_SetDate( volatile tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->year ) << 16 |
          BIN2BCD( prtc->month ) << 8 |
          BIN2BCD( prtc->date ) |
          BIN2BCD( prtc->wday ) << 13 );
  // Date reserved mask
  temp &= (uint32_t)0x00FFFF3F;
  RTC->DR = ( RTC->DR & ~(RTC_DR_YT | RTC_DR_YU | RTC_DR_MT | RTC_DR_MU | RTC_DR_DT | RTC_DR_DU | RTC_DR_WDU)) | temp;
  // Сбрасываем флаг синхронизации часов
  RTC->ISR &= ~RTC_ISR_RSF;
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void RTC_GetTime( volatile tRtc * prtc ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  uint32_t tmpTr = RTC->TR;
  prtc->hour = BCD2BIN( tmpTr >> 16 );
  prtc->min = BCD2BIN( tmpTr >> 8 );
  prtc->sec = BCD2BIN( tmpTr  );
}

static void RTC_GetDate( volatile tRtc * prtc ){
  while((RTC->ISR & RTC_ISR_RSF) == 0)
  {}
  uint32_t tmpDr = RTC->DR;
  prtc->year = BCD2BIN( tmpDr >> 16 );
  prtc->month = BCD2BIN( (tmpDr >> 8) & 0x1f );
  prtc->date = BCD2BIN( tmpDr );
  prtc->wday = ( tmpDr >> 13 ) & 0x7;
}

static void RTC_SetAlrm( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->date ) << 24 |
          BIN2BCD( prtc->hour ) << 16 |
          BIN2BCD( prtc->min ) << 8 |
          BIN2BCD( prtc->sec ) );
  RTC->ALRMAR = ( RTC->ALRMAR & (RTC_ALRMAR_PM | RTC_ALRMAR_DT | RTC_ALRMAR_DU | RTC_ALRMAR_HT | RTC_ALRMAR_HU | RTC_ALRMAR_MNT | RTC_ALRMAR_MNU | RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

static void RTC_GetAlrm( tRtc * prtc ){
  prtc->date = BCD2BIN( RTC->ALRMAR >> 24 );
  prtc->hour = BCD2BIN( RTC->ALRMAR >> 16 );
  prtc->min = BCD2BIN( RTC->ALRMAR >> 8);
  prtc->sec = BCD2BIN( RTC->ALRMAR );
}

static void RTC_CorrAlrm( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR |= RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = BIN2BCD( prtc->sec );
  RTC->ALRMAR = ( RTC->ALRMAR & (RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void usTimStop( void ){
  // Остонавливаем
  TIM2->CR1 &= ~TIM_CR1_CEN;
  // Стираем все флаги
  TIM2->SR = 0;
}

/* Установка и запуск wakeup-таймера
 * us - время в мкс.
 */
void usTimSet( uint32_t us ){

  TIM2->CNT = us;
  //Запускаем
  TIM2->CR1 |= TIM_CR1_CEN;
}

void usTimInit( void ){
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // Не включаем
  TIM2->CR1 &= ~TIM_CR1_CEN;
  // Прескалер - для 1МГц (1мкс)
  TIM2->PSC = rccClocks.PCLK1_Frequency/1000000 - 1;
  // Обратный отсчет
  TIM2->CR1 |= TIM_CR1_DIR;
  // Прерывание по обнулению
  TIM2->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ( TIM2_IRQn);
}


void timerSetup(struct timer_list *timer, void (*function)(uintptr_t), uintptr_t data)
{
	timer->function = function;
	timer->data     = data;

  timer->entry.next = NULL;
}

///**
//  * @brief  Проверка таймера на ожидание исполнения.
//  *
//  * @param[in]  timer дескриптор таймера
//  *
//  * @retval true/false
//  */
//inline bool timerPending(const struct timer_list *timer){
//  return (timer->entry.next != NULL);
//}

/**
  * @brief	Непосредственное удаление таймера из очереди.
  *
  * @param[in]	timer		дескриптор таймера
  * @param[in]	clear_pending	удаление признака размещения в очереди на исполнение
  *
  * @retval	none
  */
static void timerDetach(struct timer_list *timer, bool clear_pending)
{
	list_del(&timer->entry);

	if (clear_pending)
		timer->entry.next = NULL;
}

bool timerMod(struct timer_list *timer, uint32_t expires) {
	bool  retval = false;

	expires += mTick;

	if (timerPending(timer)) {
		/*
		 * This is a common optimization triggered by the
		 * networking code - if the timer is re-modified
		 * to be the same thing then just return:
		 */
		if (timer->expires == expires)
			return true;

		timerDetach(timer, false);

		retval = true;
	}

	timer->expires = expires;

	list_add_tail(&timer->entry, &_timers_queue);

	return retval;
}

bool timerModArg(struct timer_list *timer, uint32_t expires, uintptr_t arg) {
  bool  retval = false;

  timer->data = arg;

  expires += mTick;

  if (timerPending(timer)) {
    /*
     * This is a common optimization triggered by the
     * networking code - if the timer is re-modified
     * to be the same thing then just return:
     */
    if (timer->expires == expires)
      return true;

    timerDetach(timer, false);

    retval = true;
  }

  timer->expires = expires;

  list_add_tail(&timer->entry, &_timers_queue);

  return retval;
}

void timerAdd(struct timer_list *timer)
{
	timerMod(timer, timer->expires - mTick);
}

bool timerDel(struct timer_list *timer)
{
	if (!timerPending(timer))
		return false;

	timerDetach(timer, true);

	return true;
}

/**
  * @brief Установка делителя "Prescaler" аппаратного таймера для нужной частоты счета
  *         Если делитель больше 0xFFFF, выставляется 0xFFFF и возвращается -1
  *
  * @param[in]  tim устанавливаемый таймер
  * @param[in]  tim_frequency частота срабатывания таймера, Гц
  * @param[out]  psc Значение Далителя таймера TIM_PSC
  *
  * @retval если без ошибок - возвращает "0", иначе, если переполнение, возвращает "-1"
  */
uint8_t timPscSet( TIM_TypeDef * tim, uint32_t tim_frequency, uint16_t * psc){
  uint32_t tmp;
  uint32_t timClk;
  uint32_t prescaler;
  uint8_t rc = SET;

  assert_param(IS_TIM_ALL_PERIPH(tim));
  if( IS_TIM_PCLK1_PERIPH(tim) ){
    // PCLK1 TIMs
    tmp = RCC->CFGR & RCC_CFGR_PPRE1;
    tmp = ((tmp >> 8) & 0x3);
    timClk = rccClocks.PCLK1_Frequency;
  }
  else {
    // PCLK1 TIMs
    tmp = RCC->CFGR & RCC_CFGR_PPRE2;
    tmp = ((tmp >> 11) & 0x3);
    timClk = rccClocks.PCLK2_Frequency;
  }
  if( tmp == 0 ){
    timClk *= 2;
  }
  prescaler = timClk / (tim_frequency) - 1;

  if(prescaler <= 0xFFFF){
    rc = RESET;
  }
  else {
    prescaler = 0xFFFF;
  }
  *psc = (uint16_t)prescaler;

  return rc;
}

/**
  * @brief	Обработчик прерываний SysTick.
  *
  * @param	none
  *
  * @retval	none
  */
void SysTick_Handler(void)
{
	++mTick;
}

/**
  * @brief	Обработка данных подсистемы таймеров.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void timersClock( void ){

	static uint32_t     _prev_jiffies;
	struct list_head    work_list;
	struct list_head   *curr, *next;
	struct timer_list  *timer;

	if (time_after(mTick, _prev_jiffies)) {
		_prev_jiffies = mTick;

		INIT_LIST_HEAD(&work_list);

		list_for_each_safe(curr, next, &_timers_queue) {
			timer = list_entry(curr, struct timer_list, entry);

			if (time_after(_prev_jiffies, timer->expires))
				list_move_tail(&timer->entry, &work_list);
		}

		while (!list_empty(&work_list)) {
			timer = list_first_entry(&work_list, struct timer_list, entry);

			timerDetach(timer, true);

			if (timer->function != NULL)
				timer->function(timer->data);
		}
	}
}

/**
  * @brief	Инициализация подсистемы таймеров.
  *
  * @param[in]	self	дескриптор интерфейса
  *
  * @retval	none
  */
void timersInit( void ) {

	/* Настраиваем системный таймер на заданную частоту. */
	SysTick_Config(rccClocks.SYSCLK_Frequency / TICK_HZ);
}


