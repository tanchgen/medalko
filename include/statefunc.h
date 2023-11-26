/*
 * statefunc.h
 *
 *  Created on: 11 авг. 2020 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef STATEFUNC_H_
#define STATEFUNC_H_

#include "gpio_arch.h"

typedef enum {
  MSTATE_NON,
  MSTATE_NON_2,
  MSTATE_OFF_OK,
  MSTATE_OFF,
  MSTATE_ON,
  MSTATE_ON_OK,

} eMiniState;

extern eMiniState measRunWait;

void stateOff( void );

/**
 * @brief Функции обработки состояния MCUSTATE_SYS_START системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateStart( void );
void stateFlow( void );
void stateEnd( void );
//void stateProc( void );
void stateFin( void );
void stateFault( void );


#endif /* STATEFUNC_H_ */
