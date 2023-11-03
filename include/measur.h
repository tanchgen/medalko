/*
 * measur.h
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */

#ifndef MEASUR_H_
#define MEASUR_H_

#include "stm32f10x.h"

#define MEAS_TIME_MAX     3
#define MEAS_TIME_MIN     1

typedef enum _eMesState {
  MEASST_OFF,
  MEASST_REL_EN,
  MEASST_START_PROB,
  MEASST_END_PROB,
  MEASST_PROC,
  MEASST_FIN,
  MEASST_FAULT,
} eMeasState;

extern eMeasState measState;
extern eMeasState measOnNeed;

typedef struct _sMeasStatus {
  uint8_t measStart: 1;
  uint8_t pressStartOk: 1;
  uint8_t pressStartLow: 1;
  uint8_t alcoLow: 1;

} sMeasStatus;

typedef struct {
  volatile float param;

  float peakMin;
  float peakMax;

} sPrmDt;


typedef struct _sAlcoData {
  float press;
  float alko;
  float temp;
} sAlcoData;

typedef struct _sMeasur {

  uint32_t uxTime;
  uint32_t msec;

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

#endif /* MEASUR_H_ */
