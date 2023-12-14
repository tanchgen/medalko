#ifndef _TIMES_H
#define _TIMES_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "list.h"

#define IS_TIM_PCLK1_PERIPH(PERIPH) (((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM5) || \
                                      ((PERIPH) == TIM6) || \
                                      ((PERIPH) == TIM7) || \
                                      ((PERIPH) == TIM12) || \
                                      ((PERIPH) == TIM13) || \
                                      ((PERIPH) == TIM14))

#define IS_TIM_PCLK2_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM9) || \
                                      ((PERIPH) == TIM10) || \
                                      ((PERIPH) == TIM11))

#define ALARM_UPDATE_TOUT   100

#define ALRM_A      0   // НЕ МЕНЯТЬ !!!!
#define ALRM_B      1   // НЕ МЕНЯТЬ !!!!

#define RTCCLOCK     32768

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} tTime;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
} tDate;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} tRtc;

#define TIMEZONE_MSK      (+3)

  // DEF: standard signed format
  // UNDEF: non-standard unsigned format
  #define _XT_SIGNED

#ifdef  _XT_SIGNED
  typedef int32_t                           tUxTime;
#else
  typedef uint32                          tUxTime;
#endif

extern volatile tRtc rtc;
extern volatile tUxTime uxTime;
extern volatile uint8_t sendToutFlag;
extern volatile uint8_t minTout;
extern volatile uint8_t minToutRx;
extern volatile uint8_t uxSecTout;


extern volatile uint32_t mTick;
extern volatile uint32_t secs;
extern volatile uint16_t msecs;

extern RCC_ClocksTypeDef  rccClocks;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void);
//void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
tUxTime xTm2Utime( volatile tRtc * prtc );

void timersProcess( void );

void timersHandler( void );
void mDelay( uint32_t del );

void setRtcTime( tUxTime xtime );
tUxTime getRtcTime( void );

void RTC_GetTime( volatile tRtc * prtc );

void setAlrm( tUxTime xtime );
tUxTime getAlrm( void );
void correctAlrm( void );
void setAlrmSecMask( uint8_t secMask );

void usTimInit( void );
void usTimStop( void );
/* Установка и запуск таймера микосекунд
 * us - время в мкс.
 */
void usTimSet( uint32_t us );
// Таймер мигания светодиодом
void errTimInit( void );


/** Инициализатор таймера (аналогичен функции ::timerSetup). */
#define TIMER_INITIALIZER(_function, _data) {	\
		.entry = {NULL, NULL},		\
		.expires = 0,			\
		.function = (_function),	\
		.data = (_data),		\
	}

/** Структура дескриптора таймера. */
struct timer_list {
	/** Точка включения в очередь таймеров. */
	struct list_head  entry;

	/** Время исполнения таймера. */
	uint32_t  expires;

	/** Обработчик тайм-аута. */
	void (*function)(uintptr_t);

	/** Данные таймера (аргумент обработчика тайм-аута). */
	uintptr_t  data;
};

/** Счетчик тиков системного таймера. */
extern volatile uint32_t  mTick;

/*
 * These inlines deal with timer wrapping correctly. You are 
 * strongly encouraged to use them
 *	1. Because people otherwise forget
 *	2. Because if the timer wrap changes in future you
 *	   won't have to alter your code.
 *
 * time_after(a,b) returns true if the time a is after time b.
 */
#define time_after(a, b)	((int32_t)(b) - (int32_t)(a) < 0)
#define time_before(a, b)	time_after(b, a)

#define time_after_eq(a, b)	((int32_t)(a) - (int32_t)(b) >= 0)
#define time_before_eq(a, b)	time_after_eq(b, a)

void uDelay( const uint32_t usecs);

/**
  * @brief	Задержка исполнения на заданное число миллисекунд.
  *
  * @param[in]	msecs	число миллисекунд
  *
  * @retval	none
  */
void mDelay(const uint32_t msecs);

/**
  * @brief	Инициализация системы таймеров.
  *
  * @param[in]	none
  *
  * @retval	none
  */
void timersInit( void );

/**
  * @brief	Настройка таймера.
  *
  * @param[in]	timer		дескриптор таймера
  * @param[in]	function	функция-обработчик тайм-аута
  * @param[in]	data		данные таймера (аргумент функции-обработчика тайм-аута)
  *
  * @retval	none
  */
void timerSetup(struct timer_list *timer, void (*function)(uintptr_t), uintptr_t data);

/**
  * @brief	Модификация таймера.
  *
  * @param[in]	timer	дескриптор таймера
  * @param[in]	expires	время исполнения таймера
  *
  * @retval	признак модификации таймера из очереди на ожидание исполнения
  */
bool timerMod(struct timer_list *timer, uint32_t expires);

bool timerModArg(struct timer_list *timer, uint32_t expires, uintptr_t arg);
/**
  * @brief	Запуск таймера.
  *
  * @param[in]	timer	дескриптор таймера
  *
  * @retval	none
  */
void timerAdd(struct timer_list *timer);

/**
  * @brief	Удаление таймера.
  *
  * @param[in]	timer	дескриптор таймера
  *
  * @retval	признак удаления таймера из очереди на ожидание исполнения
  */
bool timerDel(struct timer_list *timer);

/**
  * @brief  Проверка таймера на ожидание исполнения.
  *
  * @param[in]  timer дескриптор таймера
  *
  * @retval true/false
  */
static inline bool timerPending(const struct timer_list *timer){
  return (timer->entry.next != NULL);
}


uint8_t timPscSet( TIM_TypeDef * tim, uint32_t tim_frequency, uint16_t * psc);

#endif /* _TIMES_H */

