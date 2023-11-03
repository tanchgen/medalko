#ifndef _NVIC_H
#define _NVIC_H

#include <stdint.h>

#ifdef STM32F4XX
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#endif // STM32F4XX

/**
  * @brief	Разрешение работы периферии NVIC.
  *
  * @param[in]	irq		номер прерывания периферии
  * @param[in]	priority	приоритет прерывания периферии
  *
  * @retval	none
  */
void enable_nvic_irq(IRQn_Type irq, uint8_t priority);

/**
  * @brief	Инициализация подсистемы NVIC.
  *
  * @param	none
  *
  * @retval	none
  */
void init_nvic(void);

#endif /* _NVIC_H */

