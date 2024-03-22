/*
 * measur.h
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */

#ifndef MEASUR_H_
#define MEASUR_H_

#include <stddef.h>
#include "stm32f10x.h"

// Давление измеряет ADC или I2C
#define PRESS_ADC         1

#define MEAS_TIME_MAX     3000
#define MEAS_TIME_MIN     1000

#define MEAS_SEQ_NUM_MAX  1024

#define REL_PULSE_DEF     200     // Длина импульса реле по умолчанию
#define ALCO_TOUT_MIN     500     // Время ожидания срабатывания порога ALCO после выключения соленоида
#define ALCO_TOUT_MAX     10000   // Максимальное время измерения ALCO
#define PRESS_LIMIT_MIN   50

typedef enum {
  PROTO_JSON,
  PROTO_CSV,
} eSendProto;

typedef enum _eMesState {
  MEASST_OFF,
//  MEASST_REL_EN,
  MEASST_START_PROB,
  MEASST_FLOW_PROB,
  MEASST_END_PROB,
//  MEASST_PROC,
  MEASST_FIN,
  MEASST_FAULT,
} eMeasState;

typedef enum {
  SEND_START,
  SEND_GOON,
  SEND_FIN,
  SEND_END,
  SEND_CONT,
  SEND_NULL
} eSendState;

typedef enum {
//  RX_PRM_NULL,
  RX_PRM_PRESS,
//  RX_PRM_PERIOD,
  RX_PRM_CONTINUE,
  RX_PRM_NUM
} eRxPrm;

typedef struct {
  float alcoKx;       // Коэфф. "X" К-Л аппроксимации
  float alcoKy;       // Коэфф. "Y" К-Л аппроксимации
} sAlcoKn;

#define ALCO_KN_NUM     7   // Количество интервалов для К-Л аппроксимации датчика алко

typedef union _sMeasStatus {
  struct {
    uint32_t measStart: 1;      // 0
    uint32_t pressOk: 1;        // 1
    uint32_t pressFaultLow: 1;  // 2
    uint32_t alcoLow: 1;        // 3
    uint32_t alcoHi: 1;         // 4
    uint32_t relStart: 1;       // 5
    uint32_t relEnd: 1;         // 6
#define STATUS_CONT_MASK        (0x3 << 7)
    uint32_t cont: 1;           // 7 Флаг постоянной передачи
    uint32_t sendStart: 1;      // 8
    uint32_t sent: 1;           // 9
#if SIMUL
    uint32_t alcoSimOn: 1;      // 10
#endif //SIMUL
  };
  uint32_t u32stat;
} uMeasStatus;

typedef struct {
  volatile float param;

  float peakMin;
  float peakMax;

} sPrmDt;

typedef struct _sAlcoData {
  int16_t press;
  int16_t alco;
  int16_t temp;
} sMeasRec;

typedef struct _sMeasur {
  uMeasStatus status;
  FlagStatus rel;           // Работа соленоида ДЛЯ МОНИТОРА
  uint32_t secs;
  uint32_t msec;

  uint32_t tout0;
  uint32_t tout;
  uint32_t count;

  uint32_t alcoTout;
  uint32_t buzzTout;

  uint32_t secsStart;
  uint32_t msecStart;

  uint32_t secsStart2;
  uint32_t msecStart2;

  uint32_t secsStop;
  uint32_t msecStop;

  sMeasRec alcoData;       // Указатель на массив собранных данных
//  uint32_t dataNum;           // Количество собранных данных
  eSendProto sendProto;

  uint32_t relPulse;

  float pressPeakMin ;
  float pressPeakMax;

  float alcoPeakMin;
  float alcoPeakMax;

  float tempPeakMin;
  float tempPeakMax;

  float pressLimMinStart ;
  float pressLimMinStop ;

#if SIMUL
  float pressLimMax;
#endif // SIMUL

  float alcoLimMinStart;
  float alcoLimMinStop;
  float alcoLimMax;

  float tempLimMin;
  float tempLimMax;

  uint16_t prmPressMin;             // Нижний порог давления
  uint16_t prmContinuous;             // Флаг непрерывной

  uint32_t contRelTout;
  uint16_t alcoK0;              // Постоянный коэфф. датчика алкоголя (k * 10000)
  sAlcoKn alcoKn[ALCO_KN_NUM];            // Набор данных для Кусочно-Линейной аппроксимации
} sMeasur;

extern sMeasur measDev;
extern eMeasState measState;
extern FlagStatus measOnNeed;
extern FlagStatus measRun;
extern FlagStatus onCan;

uint8_t my_itoa(int32_t value, uint8_t * buf, int8_t base);

void pressProc( int32_t press, uint16_t * count );
void alcoProc( int32_t alco, int32_t temp );
void termProc( int32_t term );
void totalProc( void );

void measPrmClean( void );
void measStartClean( void );
void measInit( void );

void continueStart( void );
void continueStop( void );
//void continueProc( void );

#endif /* MEASUR_H_ */
