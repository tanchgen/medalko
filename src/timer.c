#include <stddef.h>
#include <math.h>

#include "main.h"

/** Число миллисекунд в секунде. */
#define _MSEC_PER_SEC	(1000)

volatile uint32_t  mTick = 0;
volatile uint32_t  secs = 0;
volatile uint16_t  msecs = 0;
FlagStatus secsFlag = RESET;

/** Голова очереди таймеров на исполнение. */
static struct list_head  _timers_queue = LIST_HEAD_INIT(_timers_queue);



//uint32_t msecs_to_jiffies(const uint32_t msecs)
//{
//#if CONFIG_HZ <= _MSEC_PER_SEC && !(_MSEC_PER_SEC % CONFIG_HZ)
//	/*
//	 * HZ is equal to or smaller than 1000, and 1000 is a nice
//	 * round multiple of HZ, divide with the factor between them,
//	 * but round upwards:
//	 */
//	return (msecs + (_MSEC_PER_SEC / CONFIG_HZ) - 1) / (_MSEC_PER_SEC / CONFIG_HZ);
//
//#elif CONFIG_HZ > _MSEC_PER_SEC && !(CONFIG_HZ % _MSEC_PER_SEC)
//	/*
//	 * HZ is larger than 1000, and HZ is a nice round multiple of
//	 * 1000 - simply multiply with the factor between them:
//	 */
//	return msecs * (CONFIG_HZ / _MSEC_PER_SEC);
//
//#else
//	/*
//	 * Generic case - multiply, round and divide:
//	 */
//	return (uint32_t)ceilf((float)msecs * CONFIG_HZ / 1000.0f);
//#endif
//}

#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC


void dwtInit(void) {
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
    DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void uDelay( const uint32_t usecs) {
    uint32_t usTick =  usecs * (SystemCoreClock / 1000000); // получаем кол-во тактов за 1 мкс и умножаем на наше значение
    DWT->CYCCNT = 0U; // обнуляем счётчик
    while(DWT->CYCCNT < usTick);
}

void mDelay(const uint32_t msecs) {
	uint32_t  tout = mTick + msecs;

	while( tout > mTick )
  {}
}


void timerSetup(struct timer_list *timer, void (*function)(uintptr_t), uintptr_t data) {
	timer->function = function;
	timer->data     = data;
	timer->entry.next = NULL;
}

/**
  * @brief	Непосредственное удаление таймера из очереди.
  *
  * @param[in]	timer		дескриптор таймера
  * @param[in]	clear_pending	удаление признака размещения в очереди на исполнение
  *
  * @retval	none
  */
static void _detach_timer(struct timer_list *timer, bool clear_pending)
{
	list_del(&timer->entry);

	if (clear_pending)
		timer->entry.next = NULL;
}

bool timerStart( struct timer_list *timer, uint32_t expires) {
  if (timerPending(timer) == RESET ) {
    timerMod(timer, expires );
    return false;
  }
  return true;
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

		_detach_timer(timer, false);

		retval = true;
	}

	timer->expires = expires;

	list_add_tail(&timer->entry, &_timers_queue);

	return retval;
}

bool timerModArg(struct timer_list *timer, uint32_t expires, uintptr_t data){
  bool  retval = false;

  timer->data = data;

  expires += mTick;

  if (timerPending(timer)) {
    /*
     * This is a common optimization triggered by the
     * networking code - if the timer is re-modified
     * to be the same thing then just return:
     */
    if (timer->expires == expires)
      return true;

    _detach_timer(timer, false);

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

	_detach_timer(timer, true);

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
void SysTick_Handler(void) {
	++mTick;
  if( (msecs = mTick % _MSEC_PER_SEC) == 0 ){
    secs++;
    secsFlag = SET;
  }
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

			_detach_timer(timer, true);

			if (timer->function != NULL)
				timer->function(timer->data);
		}
	}
}


void timersInit( void ){
  dwtInit();
}
