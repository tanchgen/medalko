/*
 * measur.c
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */
#include "tinyalloc.h"
#include "times.h"
#include "measur.h"

eMeasState measState = MEASST_OFF;
FlagStatus measOnNeed = RESET;
FlagStatus measRun = RESET;
FlagStatus onCan;


const uint16_t measPressLimMin = 150;
const float measPressLimMax = 1500;
const float measAlkoLimMin = 15.0;

sMeasur measDev;


uint8_t my_itoa(int32_t value, uint8_t * buf, int8_t base){
  uint8_t i = 30;

  buf = '\0';

  for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;

  return 30 - i;
}


// Обработка результата ADC_PRESS
void pressProc( int32_t press ){
  if( measState == MEASST_OFF ){
      if( (press > measPressLimMin) && onCan ){
        // Давление выше минимального порога - Запускаем процесс забора проб
        measOnNeed = SET;
      }
  }
  else {
    if( (press < measPressLimMin) && (measDev.status.pressFaultLow == RESET) ){
      measDev.status.pressFaultLow = SET;
      measState = MEASST_FAULT;
    }
    else {
      if( measState == MEASST_START_PROB ){
        if( measDev.status.measStart == RESET ){
          // Определяем начальный таймаут до начала забора проб
          if( press >= measPressLimMax ){
            measDev.tout = mTick + MEAS_TIME_MIN;
          }
          else {
            float tmp;

            tmp = (((press - measPressLimMin) * 1000)/(measPressLimMax - measPressLimMin));
            tmp *= (MEAS_TIME_MAX - MEAS_TIME_MIN);
            tmp /= 1000;
            tmp += MEAS_TIME_MIN;
          }
        }
        else {
          // Забор проб: созраняем полученое значение
          measDev.alcoData[measDev.dataNum].press = press;
        }
      }
    }
  }
}


// Обработка результатов ADC_TEMPERATURE
void termProc( int32_t term ){
  if( measDev.status.measStart ){
    // Забор проб: созраняем полученое значение
    measDev.alcoData[measDev.dataNum].temp = term;
  }
}


// Обработка результатов ADC_ALCO
void alcoProc( int32_t alco ){
  if( measDev.status.measStart ){
    // Забор проб: созраняем полученое значение
    measDev.alcoData[measDev.dataNum].alco = alco;
    if( alco < measAlkoLimMin ){
      // Значение ALCO упало ниже порога - будем завершать данный цикл
      measDev.status.alcoLow = SET;
    }
  }
}


// Финальная обработка результатов ADC
void totalProc( void ){
  if( measDev.status.measStart ){
    // Все данные сохранили
    measDev.dataNum++;
  }
}


// Периодически выполняемая функция 
void measClock( void ){
}


void measStartClean( void ){
  measDev.dataNum = 0;
  measDev.status.u32stat = 0;
}


void measInit( void ){
  sAlcoData * tmpAd;
  if( (tmpAd = (sAlcoData*)ta_alloc( sizeof( sAlcoData ) * MEAS_SEQ_NUM_MAX) ) == NULL) {
    // Ошибка выделения памяти
    assert_param( tmpAd != NULL );
  }
  measDev.alcoData = tmpAd;
}
