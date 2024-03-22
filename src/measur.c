/*
 * measur.c
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */
#include <stdio.h>
#include <string.h>

#include "tinyalloc.h"
#include "times.h"
#include "usb_vcp.h"
#include "statefunc.h"
#include "buffer.h"
#include "measur.h"
#include "buffer.meas.h"


FlagStatus alcoKnFlag = RESET;
const sAlcoKn _alcoKn[ALCO_KN_NUM] __attribute__ ((section(".constdata"))) = {
    {0,0},      // x0,y0
    {0,0},      // x1,y1
    {0,0},
    {0,0},
    {0,0},
    {0,0},
    {0,0}
};
//#if SIMUL
//  extern uint32_t simulStart;
//#endif // SIMUL

eMeasState measState = MEASST_OFF;
FlagStatus measOnNeed = RESET;
FlagStatus measRun = RESET;
FlagStatus onCan;

uint32_t sendTout;
uint32_t sendCount;
eSendState sendState;

uint8_t sendBuf[96];

uint32_t tmpTout = 0;

//const uint16_t measPressLimMin = PRESS_LIMIT_MIN;
const float measPressLimMax = PRESS_LIMIT_MIN * 3;
const float measAlkoLimMin = 15.0;

sMeasur measDev;
// Определяем начальный таймаут до начала забора проб
int32_t pressAvg;

// ================ Private Function =============================
void pulseTimInit( TIM_TypeDef * portim, uint16_t tout );
// ==================================================================

// ----------------------- USB transfer functions ----------------------
size_t sendTmPrep( uint8_t * buf ){
  size_t sz = 0;

  if( measDev.sendProto == PROTO_JSON ){
    sz = sprintf( (char*)buf, "{\"alcoData\":{\"startTime\":%ld.%ld,\"start2Time\":%ld.%ld,\"measData\":[\n", \
                  measDev.secsStart, measDev.msecStart, measDev.secsStart2, measDev.msecStart2 );
  }
  else {
    memcpy( (char*)buf, "\"startTime\",\"start2Time\",\"press\",\"alco\",\"temp\",\"stopTime\"\n", 58 );
    sz = 58;
  }
  return sz;
}

size_t sendTmCont( uint8_t * buf ){
//  uint32_t i = sendCount;
  uint32_t sz = 0;
  sMeasRec rec;
  eSendProto proto;

  if( measBuf_Read( &measBuf, &rec, 1 ) != 0 ){
#ifdef  USE_FULL_ASSERT
    if( measDev.status.cont ){
//      proto = PROTO_JSON;
      assert_param( sendCount == 0 );
    }
//    else {
      proto = measDev.sendProto;
//    }
#endif // USE_FULL_ASSERT

    if( proto == PROTO_JSON ){
      if( sendCount ){
        // Не первая запись - добавим запяту, разделяющую записи
        *buf++ = ',';
        sz = 1;
      }
      // Давление
      sz += sprintf( (char*)buf, "{\"press\":%d,\"alco\":%d, \"temp\":%d}\n", \
                      rec.press,
                      rec.alco,
                      rec.temp );
    }
    else {
      if( sendCount == 0 ){
        sz = sprintf( (char*)buf, "%ld.%03ld,%ld.%03ld,", \
                        measDev.secsStart,measDev.msecStart,
                        measDev.secsStart2,measDev.msecStart2 );
      }
      else {
        memcpy( (char*)buf, "0.0,0.0,", 8 );
        sz = 8;
      }
      if( measDev.status.alcoLow == RESET ){
        sz += sprintf( ((char*)buf + sz), "%d,%d,%d,0.0\n", \
                        rec.press, rec.alco, rec.temp );
      }
      else {
        sz += sprintf( ((char*)buf + sz), "%d,%d,%d,", \
                        rec.press, rec.alco, rec.temp );

      }
    }
  }

  return sz;
}

size_t sendTmEnd( uint8_t * buf ){
  uint32_t sz;

  // Закрывающие скобки
  if( measDev.sendProto == PROTO_JSON ){
    sz = sprintf( (char*)buf, "],\"stopTime\":%ld.%ld}\n}\n\n", measDev.secsStop, measDev.msecStop );
  }
  else {
    sz = sprintf( (char*)buf, "%ld.%03ld\n\n", \
                    measDev.secsStop,measDev.msecStop );

//    *buf = '\n';
//    sz = 1;
  }
  return sz;
}

// -------------------------------------------------------------------------------------
// -------------------- Received parcing -----------------------------------------------
uint32_t receivParse( uint8_t * rxBuf, uint32_t rxSizeMax ){
  uint32_t rxlen;
  uint press;
  uint pulse;
  uint cont;

#if 0     // ------------------- Низкий уровень -----------------------------------
  eRxPrm rxPrmFlag = RX_PRM_NUM;
  uint8_t * bufEnd = rxBuf + rxSizeMax;
  uint8_t * prmStr[RX_PRM_NUM] = {"pressure_limit", "pump_period", "broadcast_mode"};

  for( rxlen = 0; rxBuf != rxSizeMax; rxBuf++ ) {
    uint8_t ch = *rxBuf;
    assert_param( ch != '\0' );
    if( ch == '}' ){
//      rxlen = i;
      break;
    }
    if( (rxPrmFlag == RX_PRM_NUM) && (ch != '"') ){
      continue;
    }
    rxBuf++;
    for( rxPrmFlag = 0; rxPrmFlag < RX_PRM_NUM; rxPrmFlag++ ){
      if( strcmp( (char*)rxBuf, (char*)prmStr[rxPrmFlag] ) == 0 ){
        break;
      }
    }
    if( rxPrmFlag == RX_PRM_NUM ){
      trace_write( "\t!!!Rx prm fault!!!\n", 20);
    }
    else {
      // Парсим значение параметра
      while( (*rxBuf < '0') || (*rxBuf > '9') )
      {}
      // Принят нормальный параметр
      measDev.receivPrm[rxPrmFlag] = atoi((char*)rxBuf);
    }
  }
#else // ------------------- Высокий уровень -----------------------------------
  (void)rxSizeMax;
  while( *rxBuf != '{' ){
    rxBuf++;
  }
  rxlen = sscanf( (char*)rxBuf, "{\"pressure_limit\":%u,\"pump_period\":%u,\"broadcast_mode\":%u}", \
                &press, &pulse, &cont );
  measDev.prmPressMin = max( press, 40 );  // Нижний порог = 30
  measDev.relPulse = max(pulse, 5000);
  measDev.prmContinuous = (cont)? SET : RESET;  // Или 0, или 1
  measDev.pressLimMinStart = measDev.prmPressMin;
  measDev.pressLimMinStop = measDev.prmPressMin - 10;
#if SIMUL
  measDev.pressLimMax = measDev.prmPressMin * 3;
#endif // SIMUL

#endif  // ---------------------------------------------------------------------
  if(measDev.prmContinuous &&
      ((measState == MEASST_OFF) && (measRun == RESET) && (measRunWait != MSTATE_NON)))
  {
    timerMod( &measOnCanTimer, TOUT_100);
  }

  return rxlen;
}
// -------------------------------------------------------------------------------------

uint8_t my_itoa(int32_t value, uint8_t * buf, int8_t base){
  uint8_t i = 30;

  buf = '\0';

  for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;

  return 30 - i;
}


// Обработка результата ADC_PRESS
void pressProc( int32_t press, uint16_t * count ){
  if( measDev.status.cont ){
    // Постоянный сбор данных: созраняем полученое значение
    measDev.alcoData.press = press;
  }
  if( measState == MEASST_OFF ){
    if( (press > measDev.pressLimMinStart) && onCan ){
      // Давление выше минимального порога - Запускаем процесс забора проб
      measOnNeed = SET;
      measDev.tout0 = mTick;
    }
  }
  else if( measState < MEASST_END_PROB ){
    if( (measDev.status.relEnd == RESET) && (press < measDev.pressLimMinStop) ){
      if( measDev.status.pressFaultLow == RESET ){
        measDev.status.pressFaultLow = SET;
        measRunWait = MSTATE_NON;
        measState = MEASST_FAULT;
      }
    }
    else {
      if( measDev.status.measStart ){
        // Забор проб: созраняем полученое значение
        measDev.alcoData.press = press;
      }
      else {
        if ( measDev.status.pressOk == RESET ){
          // Период ПЕРЕД забором проб
          (*count)++;
          pressAvg += press;
          press = pressAvg / *count;
          if( press >= measPressLimMax ){
            measDev.tout = measDev.tout0 + MEAS_TIME_MIN;
          }
          else {
            float tmp;

            tmp = (((press - measDev.pressLimMinStop) * 1000)/(measPressLimMax - measDev.pressLimMinStop));
            tmp *= (MEAS_TIME_MAX - MEAS_TIME_MIN);
            tmp /= 1000;
            tmp = MEAS_TIME_MAX - tmp;
            // Корректируем время
            measDev.tout = measDev.tout0 + tmp;
          }
          if( measDev.tout < mTick ){
            // Набрали достаточно воздуха
            measDev.status.pressOk = SET;
          }
        }
      }
    }
  }
}


// Обработка результатов ADC_TEMPERATURE
void termProc( int32_t term ){
  if( measDev.status.measStart || measDev.status.cont ){
    // Забор проб ИЛИ постоянный сбор данных: созраняем полученое значение
    measDev.alcoData.temp = term;
  }
}


// Обработка результатов ADC_ALCO
// temp - Температура в 0.1гр.Ц
void alcoProc( int32_t alco, int32_t temp ){
  uint16_t termK;
  float b, k;
  uint8_t pakidx, pakidx2 = 0;       // Индекс массива поправочных коэффициентов
  sAlcoKn * palcok0;
  sAlcoKn * palcok1 = 0;

  // ----- Поправочные коеффициенты --------
  if( alco > (measAlkoLimMin / 2) ){
    // Постоянный и Температурный
    if( temp >= 250 ){
      termK = 11625 - ((65 * temp) / 10);
      alco = (alco * measDev.alcoK0) / termK;
    }
    else if( temp <= 200 ){
      termK = 8720 + ((64 * temp) / 10);
      alco = (alco * measDev.alcoK0) / termK;
    }
    else if( measDev.alcoK0 != 10000) {
      alco = (alco * measDev.alcoK0) / 10000;
    }

    if( alcoKnFlag ){
      // К-Л аппроксимация
      pakidx = 0;
      if( alco >= measDev.alcoKn[pakidx].alcoKx ){
        while( pakidx2 == 0 ){
          if( alco < measDev.alcoKn[pakidx + 1].alcoKx ) {
            pakidx2 = pakidx + 1;
            palcok0 = &measDev.alcoKn[pakidx];
            palcok1 = &measDev.alcoKn[pakidx2];
            break;
          }
          else {
            // Еще не нашли интервал
            pakidx++;
          }
        }

        if( pakidx2 != 0 ){
          // k = (Y1 - Y0)/(X1 - X0);
          // b = Y0 - k * X0
          // lineB = (y1(x2-x1)-x1(y2-y1))/(x2-x1)
          // lineK = (y1-B)/x1
          k = (float)(palcok0->alcoKy - palcok0->alcoKy) / (float)(palcok1->alcoKx - palcok0->alcoKx);
          b = (float)(palcok0->alcoKy) - (float)palcok0->alcoKx * k;
          alco = alco * k + b;
        }
        else {
          // Выше максимума диапозона К-Л аппроксимации
        }

      }
      else {
        // Ниже минимума диапозона К-Л аппроксимации
      }

    //  alco /= 10000;
    }
  }

  if( measDev.status.measStart ){
    // Забор проб: созраняем полученое значение
    measDev.alcoData.alco = alco;
    if( alco < measAlkoLimMin ){
      // Значение ALCO упало ниже порога - будем завершать данный цикл
      measDev.status.alcoLow = SET;
#if SIMUL
      measDev.status.alcoSimOn = RESET;
#endif //SIMUL
    }
  }
  else {
    if( measDev.status.cont ){
      // Постоянный сбор данных: созраняем полученое значение
      measDev.alcoData.alco = alco;
    }
    if( alco > measAlkoLimMin ){
      measDev.status.alcoHi = SET;
    }
  }
}


// Финальная обработка результатов ADC
void totalProc( void ){
  if( measDev.status.measStart || measDev.status.cont ){
    // Все данные сохранили
    if( measBuf_Write( &measBuf, &measDev.alcoData, 1 ) == 0 ){
      trace_puts("\t=== Buffer is FULL ===");
    }
  }
}


void continueStart( void ){
  measDev.status.cont = SET;
  sendState = SEND_CONT;
//  onCan = RESET;
//  measOnNeed = RESET;
//  measState = MEASST_OFF;
//  measRun = RESET;
//  measRunWait = MSTATE_OFF;
//  timerDel( &measOnCanTimer );
//  measBuf_Reset( &measBuf );
  measDev.status.sendStart = SET;
  pulseTimInit( REL_PULSE_TIM, measDev.relPulse * 10 );
}


void continueStop( void ){
  measDev.status.cont = RESET;
  sendState = SEND_END;
  timerMod( &measOnCanTimer, TOUT_1500 );
  // Очистка буфера
  measBuf_Reset( &measBuf );
}


//void continueProc( void ){
//  if( measDev.tout && (measDev.tout < mTick) ){
//    // Включаем соленоид
//    measDev.status.relStart = SET;
//    measDev.tout = mTick + measDev.prmPumpPeriod;
//#if SIMUL
//    simulStart = mTick;
//#endif // SIMUL
//  }
//}


// Периодически выполняемая функция 
void measClock( void ){
  uint32_t size;
  static uint8_t errCount;

#define USB_SEND_TOUT         5000

#ifdef TRACE
  if( tmpTout > mTick ){
    return;
  }
#endif // defined(TRACE)
  if( measDev.status.cont ){
    if( (measDev.prmContinuous == RESET) && \
        ((measRun == RESET) && (measState == MEASST_OFF)) ){
      continueStop();
    }
//    else {
//      continueProc();
//    }
  }

  if( measDev.status.sendStart ){
    if( sendTout && (sendTout <= mTick) ){
      if( ++errCount == 2 ){
        // Неудалось отправить
        errCount = 0;
        measDev.status.sendStart = RESET;
        measDev.prmContinuous = RESET;
        sendState = SEND_START;
        measRunWait = MSTATE_NON;
        measState = MEASST_FAULT;
        // Очистка буфера
        measBuf_Reset( &measBuf );
        N_JUMBO_SUBPACKETS = 0;
      }
      else {
        sendTout = mTick + USB_SEND_TOUT;
      }
    }

    if( VCP_Transmitted ){
      switch (sendState ){
        case SEND_START:
          size = sendTmPrep( sendBuf );
          sendState++;
          break;
        case SEND_GOON:
          if( (size = sendTmCont( sendBuf )) ){
            sendCount++;
          }
          else if( measDev.status.measStart == RESET ){
            sendState++;
          }
          break;
        case SEND_FIN:
          size = sendTmEnd( sendBuf );
          sendState++;
          break;
        case SEND_END:
          size = 0;
          break;
        case SEND_CONT:
          size = sendTmCont( sendBuf );
          break;
        default:
          size = 0;
          break;

      }

      if( size != 0 ){
#ifdef TRACE
//        trace_write( (char*)sendBuf, size );
//        tmpTout = mTick + 10;
#endif // TRACE
        assert_param( size <= 96 );
#if USB_SIMUL
        N_JUMBO_SUBPACKETS = 0;
#else
        Write_VCP( sendBuf, size );
#endif // SIMUL
        sendTout = mTick + USB_SEND_TOUT;
      }
      else if( sendState == SEND_END ) {
        if( measDev.status.cont == RESET ){
          measDev.status.sendStart = RESET;
        }
        measDev.status.sent = SET;
        sendState = SEND_START;
        sendCount = 0;
        sendTout = 0;
      }
    }
  }

  // Прием сообщений
  if( VCP_Received ){
    uint8_t rxbuf[RX_BUFF_SIZE/4];
    uint16_t len;
    uint8_t parsbuf[RX_BUFF_SIZE/2];
    int16_t parslen;

    Read_VCP( rxbuf, &len );
    assert_param( len <= ARRAY_SIZE(rxbuf) );
    while( len ){
      // Что-то приняли
      if( buffer_GetFree( &rxBuf ) >= len ){
        len -= buffer_Write( &rxBuf, rxbuf, len );
      }
      while( (parslen = buffer_FindChar( &rxBuf, '}')) >= 0 ){
        buffer_Read( &rxBuf, parsbuf, parslen + 1 );
        // Будем парсить принятое сообщение
        receivParse( parsbuf, parslen );
      }
    }

  }
//  else if( measDev.status.cont ){
//    measDev.status.sendStart = SET;
//  }

}


void measPrmClean( void ){
  measDev.prmContinuous = 0;
  measDev.prmPressMin = PRESS_LIMIT_MIN;
}


void measStartClean( void ){
  // Очищаем все флаги, кроме status.cont
  measDev.status.u32stat &= STATUS_CONT_MASK;
//  measDev.prmContinuous = 0;
//  measDev.prmPumpPulse = 3000;
  measDev.sendProto = PROTO_CSV;
  measDev.secsStart = 0;
  measDev.msecStart = 0;
  measDev.secsStart2 = 0;
  measDev.msecStart2 = 0;
  sendCount = 0;
}


void measInit( void ){
//  sAlcoData * tmpAd;
//  if( (tmpAd = (sAlcoData*)ta_alloc( sizeof( sAlcoData ) * MEAS_SEQ_NUM_MAX) ) == NULL) {
//    // Ошибка выделения памяти
//    assert_param( tmpAd != NULL );
//  }
//  measDev.alcoData = tmpAd;
  measDev.sendProto = PROTO_CSV;
  measDev.relPulse = REL_PULSE_DEF;
  pulseTimInit( REL_PULSE_TIM, measDev.relPulse * 10 );
  measBuf_Init( &measBuf, measRecBuff, MEAS_SEQ_NUM_MAX );
  buffer_Init( &rxBuf, receivBuff, RX_BUFF_SIZE );
  measDev.pressLimMinStart = PRESS_LIMIT_MIN;
  measDev.pressLimMinStop = PRESS_LIMIT_MIN - 10;
#if SIMUL
  measDev.pressLimMax = PRESS_LIMIT_MIN * 3;
#endif // SIMUL
  measPrmClean();
  if( _alcoKn[0].alcoKx != 0 ){
    alcoKnFlag = SET;
    for( uint8_t i = 0; i < ALCO_KN_NUM; i++ ){
        measDev.alcoKn[i] = _alcoKn[i];
    }
  }
  measDev.alcoK0 = 10000;
}
