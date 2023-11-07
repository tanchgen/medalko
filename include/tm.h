/*
 * tm.h
 *
 *  Created on: 17 февр. 2020 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef TM_H_
#define TM_H_

//#include <stddef.h>
//#include <math.h>
//
//#ifdef STM32F4XX
//#include "stm32f4xx.h"
//#else
//#include "stm32f10x.h"
//#endif // STM32F4XX
//
//#include "types.h"
//
//typedef enum {
//  INT,
//  FLOAT
//} ePtype;
//
//typedef struct {
//  uint8_t prmType: 1;      // Тип значений параметра (см. "Значение параметра"):
//                           // 0 - Min / Max.
//                           // 1 - MinLimit / MaxLimit.
//  uint8_t alrmOffLow: 1;   // Флаг АВАРИИ по нижнему порогу в выключеном состоянии
//  uint8_t alrmOffHi: 1;    // Флаг АВАРИИ по верхнему порогу в выключеном состоянии
//  uint8_t alrmOnLow: 1;   // Флаг АВАРИИ по нижнему порогу в включеном состоянии
//  uint8_t alrmOnHi: 1;    // Флаг АВАРИИ по верхнему порогу в включеном состоянии
//
//} sParamFlags;
//
//typedef enum {
//  PRM_TYPE_PEAK,
//  PRM_TYPE_OFFLIM,
//  PRM_TYPE_ONLIM,
//  PRM_TYPE_STATUS,
//  PRM_TYPE_NUM
//} ePrmType;
//
//
//
////typedef struct {
////  float param;
////
////  float peakMin;
////  float peakMax;
////
////  float limitOffLow;
////  float limitOffHi;
////
////  float limitOnLow;
////  float limitOnHi;
////} sPrmDt;
////
////typedef struct {
////  union {
////    struct {
////      uint8_t offLowAlrm: 1;     // Флаг срабатывания ошибки по порогу OFF_MIN_LIMIT
////      uint8_t offHiAlrm: 1;     // Флаг срабатывания ошибки по порогу OFF_MAX_LIMIT
////      uint8_t onLowAlrm: 1;      // Флаг срабатывания ошибки по порогу ON_MIN_LIMIT
////      uint8_t onHiAlrm: 1;      // Флаг срабатывания ошибки по порогу ON_MAX_LIMIT
////    };
////    uint8_t u8stat;
////  } status;
////
////  // Реакция на события (Limits Reaction)
////  eStatusAct offLowAct;
////  eStatusAct offHiAct;
////  eStatusAct onLowAct;
////  eStatusAct onHiAct;
////
////  sPrmDt now;                      // Актуальные данные
////  sPrmDt prev;                  // Данные последнего исходящего сообщения
////  uint32_t sendTime;             // Время последней отправки данных
////} sParamData;
//
//extern volatile uint32_t  sendTick;
//extern volatile FlagStatus  sendTickFlag;
//
//// Таймер отложенного сброса пиковых значений
//extern struct timer_list peakResetTimer;
//// Таймер отложенного сброса пиковых значений
//extern struct timer_list vpPeakResetTimer;
//
//
//void tmClock( void );
//void tmEnable( void );
//void tmInit( void );
//FlagStatus tmDeviation( int32_t op, int32_t np, ePtype type );
//void tmPrmInit( sParamData * prm, FlagStatus off, FlagStatus on );
//void tmParamProcF( sParamData * prm, FlagStatus pwroff, FlagStatus pwron );
//
//static inline void paramPeakReset( sParamData * prm ){
//  prm->now.peakMax = prm->now.peakMin = NAN;
////  prm->prev.peakMax = prm->prev.peakMin = NAN;
//}
//
//static inline void paramStatusReset( sParamData * prm ){
//  prm->status.u8stat = 0;
//}


#endif /* TM_H_ */
