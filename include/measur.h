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

#define MEAS_TIME_MAX     3000
#define MEAS_TIME_MIN     1000

#define MEAS_SEQ_NUM_MAX  500

typedef enum _eMesState {
  MEASST_OFF,
  MEASST_REL_EN,
  MEASST_START_PROB,
  MEASST_END_PROB,
  MEASST_PROC,
  MEASST_FIN,
  MEASST_FAULT,
} eMeasState;

typedef union _sMeasStatus {
  struct {
    uint32_t measStart: 1;
    uint32_t pressOk: 1;
    uint32_t pressFaultLow: 1;
    uint32_t alcoLow: 1;
    uint32_t relStart: 1;
    uint32_t relEnd: 1;
    uint32_t sent: 1;
  };
  uint32_t u32stat;
} uMeasStatus;

typedef struct {
  volatile float param;

  float peakMin;
  float peakMax;

} sPrmDt;

typedef struct _sAlcoData {
  int32_t press;
  int32_t alco;
  int32_t temp;
} sAlcoData;

typedef struct _sMeasur {
  uMeasStatus status;
  uint32_t secs;
  uint32_t msec;

  uint32_t tout;
  uint32_t count;

  uint32_t secsStart;
  uint32_t msecStart;

  uint32_t secsStop;
  uint32_t msecStop;

  sAlcoData * alcoData;       // Указатель на массив собранных данных
  uint32_t dataNum;           // Количество собранных данных

  float pressPeakMin ;
  float pressPeakMax;

  float alcoPeakMin;
  float alcoPeakMax;

  float tempPeakMin;
  float tempPeakMax;

  float pressLimMinStart ;
  float pressLimMinStop ;
  float pressLimMax;

  float alcoLimMinStart;
  float alcoLimMinStop;
  float alcoLimMax;

  float tempLimMin;
  float tempLimMax;

} sMeasur;

extern sMeasur measDev;
extern eMeasState measState;
extern FlagStatus measOnNeed;
extern FlagStatus measRun;
extern FlagStatus onCan;

uint8_t my_itoa(int32_t value, uint8_t * buf, int8_t base);

void pressProc( int32_t press );
void alcoProc( int32_t alco );
void termProc( int32_t term );
void totalProc( void );

void measStartClean( void );
void measInit( void );

#endif /* MEASUR_H_ */