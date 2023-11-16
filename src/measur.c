/*
 * measur.c
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */
#include <stdio.h>

#include "tinyalloc.h"
#include "times.h"
#include "usb_vcp.h"
#include "statefunc.h"
#include "measur.h"

eMeasState measState = MEASST_OFF;
FlagStatus measOnNeed = RESET;
FlagStatus measRun = RESET;
FlagStatus onCan;

uint32_t sendTout;
uint32_t sendCount;
eSendState sendState;
uint8_t sendBuf[96];

uint32_t tmpTout = 0;

const uint16_t measPressLimMin = 90;
const float measPressLimMax = 2000;
const float measAlkoLimMin = 100.0;

sMeasur measDev;
// Определяем начальный таймаут до начала забора проб
int32_t pressAvg;

// ================ Private Function =============================
// ==================================================================

// ----------------------- USB transfer functions ----------------------
size_t sendTmPrep( uint8_t * buf ){
  size_t sz = 0;

  sz = sprintf( (char*)buf, "\"alcoData\":{\"startTime\":%ld.%ld,\"start2Time\":%ld.%ld,\"measData\":{", \
                measDev.secsStart, measDev.msecStart, measDev.secsStart2, measDev.msecStart2 );
  return sz;
}

size_t sendTmCont( uint8_t * buf ){
  uint32_t i = sendCount;
  uint32_t sz = 0;

  if( i < measDev.dataNum ){
    if( sendCount ){
      // Не первая запись - добавим запяту, разделяющую записи
      *buf++ = ',';
      sz = 1;
    }
    // Давление
    sz += sprintf( (char*)buf, "{\"press\":%ld,\"alco\":%ld, \"temp\":%ld}", \
                    measDev.alcoData[i].press,
                    measDev.alcoData[i].alco,
                    measDev.alcoData[i].temp );
  }

  return sz;
}

size_t sendTmEnd( uint8_t * buf ){
  uint32_t sz = 0;

  // Закрывающие скобки
  sz = sprintf( (char*)buf, "},\"stopTime\":%ld.%ld}", measDev.secsStop, measDev.msecStop );

  return sz;
}

// ---------------------------------------------------------------------------------------------


uint8_t my_itoa(int32_t value, uint8_t * buf, int8_t base){
  uint8_t i = 30;

  buf = '\0';

  for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;

  return 30 - i;
}


// Обработка результата ADC_PRESS
void pressProc( int32_t press, uint16_t * count ){
  if( measState == MEASST_OFF ){
      if( (press > measPressLimMin) && onCan ){
        // Давление выше минимального порога - Запускаем процесс забора проб
        measOnNeed = SET;
        measDev.tout0 = mTick;
      }
  }
  else {
    if( press < measPressLimMin ){
      if( measDev.status.pressFaultLow == RESET ){
        measDev.status.pressFaultLow = SET;
        measRunWait = MSTATE_NON;
        measState = MEASST_FAULT;
      }
    }
    else {
      if( measDev.status.measStart ){
        // Забор проб: созраняем полученое значение
        measDev.alcoData[measDev.dataNum].press = press;
      }
      else {
        if (measState == MEASST_START_PROB ){
          // Период ПЕРЕД забором проб
          (*count)++;
          pressAvg += press;
          press = pressAvg / *count;
          if( press >= measPressLimMax ){
            measDev.tout = measDev.tout0 + MEAS_TIME_MIN;
          }
          else {
            float tmp;

            tmp = (((press - measPressLimMin) * 1000)/(measPressLimMax - measPressLimMin));
            tmp *= (MEAS_TIME_MAX - MEAS_TIME_MIN);
            tmp /= 1000;
            tmp = MEAS_TIME_MAX - tmp;
            // Корректируем время
            measDev.tout = measDev.tout0 + tmp;
          }
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
    else {
      measDev.status.alcoHi = SET;
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
  uint32_t size;
  static uint8_t errCount;

#define USB_SEND_TOUT         5000

#if SIMUL
  if( tmpTout > mTick ){
    return;
  }
#endif // SIMUL

  if( measDev.status.sendStart ){
    if( sendTout && (sendTout <= mTick) ){
      if( ++errCount == 2 ){
        // Неудалось отправить
        measDev.status.sendStart = RESET;
        measState = MEASST_FAULT;
      }
      else {
        sendTout += USB_SEND_TOUT;
      }
    }

    if( VCP_Transmitted ){
      switch (sendState ){
        case SEND_START:
          size = sendTmPrep( sendBuf );
          sendState++;
          break;
        case SEND_CONT:
          size = sendTmCont( sendBuf );
          if( ++sendCount == measDev.dataNum ){
            sendState++;
          }
          break;
        case SEND_FIN:
          size = sendTmEnd( sendBuf );
          sendState++;
          break;
        case SEND_END:
          size = 0;
          measDev.status.sendStart = RESET;
          measDev.status.sent = SET;
          sendState++;
          break;
        default:
          size = 0;
          sendTout = 0;
          break;

      }

      if( size != 0 ){
        trace_write( (char*)sendBuf, size );
        trace_write("\n", 1);

        assert_param( size <= 96 );
#if !SIMUL
        Write_VCP( sendBuf, size );
#else
        N_JUMBO_SUBPACKETS = 0;
        tmpTout = mTick + 10;
#endif // SIMUL
        sendTout = mTick + USB_SEND_TOUT;
      }
      else {
        assert_param( sendState <= SEND_FIN );
      }
    }
  }
}


void measStartClean( void ){
  measDev.dataNum = 0;
  measDev.status.u32stat = 0;
}


void measInit( void ){
//  sAlcoData * tmpAd;
//  if( (tmpAd = (sAlcoData*)ta_alloc( sizeof( sAlcoData ) * MEAS_SEQ_NUM_MAX) ) == NULL) {
//    // Ошибка выделения памяти
//    assert_param( tmpAd != NULL );
//  }
//  measDev.alcoData = tmpAd;
  measDev.relPulse = REL_PULSE_DEF;

}
