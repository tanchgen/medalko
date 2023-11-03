#ifndef _RCC_H
#define _RCC_H

#include <stddef.h>

#define TICK_HZ     1000

extern RCC_ClocksTypeDef  rccClocks;


/**
  * @brief	Инициализация подсистемы RCC.
  *
  * @param	none
  *
  * @retval	none
  */
void rccInit(void);


#endif /* _RCC_H */

