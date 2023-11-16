/*
 * statefunc.h
 *
 *  Created on: 05 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef STATEFUNC_C_
#define STATEFUNC_C_

#include "main.h"
#include "adc.h"
#include "gpio_arch.h"
#include "measur.h"
#include "statefunc.h"

uint32_t tmptick = -1;
size_t idx;

// Состояние выполнения данного этапа
eMiniState measRunWait = MSTATE_NON_2;


/** Тайм-аут разрешения повторного включения системы при аварийном выключении. */
#define SYS_OFF_ALERT_REPAIR_TOUT   10000
/** Тайм-аут разрешения повторного включения системы при штатном выключении. */
#define SYS_OFF_REPAIR_TOUT   3000


/* @brief Очистка флагов статуса при включении системы
 *
 * @param none
 *
 * @retval none
 */
inline void sysOnClear( void ){
//  sysFlags.sysOnSet = RESET;
//  sysFlags.fpgaOnSet = RESET;
//  sysFlags.logTm = RESET;
//  sysFlags.mbErr = RESET;
//  sysFlags.caterr = RESET;
//  alertFlag = RESET;
//  alrmFlag = RESET;
//  intel.iErr = RESET;

}


/**
 * @brief Функции обработки состояния MCUSTATE_SYS_OFF системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateOff( void ){
//    case MESST_OFF:  // Система выключена
  if( measOnNeed ){
    onCan = RESET;
    measOnNeed = RESET;
    measRunWait = MSTATE_NON;
    measRun = SET;
  }

  if( measRun == SET ){
    // Направление - off->on
    gpioPinSet( &gpioPinRelEn );
    zoomOn();
    measDev.tout = mTick + MEAS_TIME_MAX;
    measStartClean();
    measRunWait = MSTATE_NON;
    trace_printf("STATE_OFF\n");
    measState++;
  }
  else {
    if( measRunWait == MSTATE_NON ){
      timerMod( &measOnCanTimer, TOUT_1500 );
      measRunWait = MSTATE_OFF;
    }
  }
}


/**
 * @brief Функции обработки состояния MEASST_START_PROB системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateStart( void ){
// case MEASST_START_PROB:   // Система готова к измерениям
  if( measRun == SET ){
    // Направление - off->on
    switch( measRunWait ){
      case MSTATE_NON:
        // Проверка по времени
        if( measDev.tout < mTick ){
          measDev.secsStart = secs;
          measDev.msecStart = msecs;
          // Запуск соленоида
          measDev.status.relStart = SET;
          // Запуск замеров
          measDev.status.measStart = SET;
          measRunWait = MSTATE_ON;
        }
        break;
      case MSTATE_ON:
        if( measDev.status.relEnd ){
          measDev.secsStart2 = secs;
          measDev.msecStart2 = msecs;
          measRunWait = MSTATE_ON_OK;
#if DEBUG_TRACE_RUN
          trace_puts(" Solenoid off");
#endif
        }
        break;
      case MSTATE_ON_OK:
        if( measDev.status.alcoLow ){
          // Значение ALCO меньше порогового - закончили забор проб
          measDev.secsStop = secs;
          measDev.msecStop = msecs;
          measDev.status.measStart = RESET;
          measRunWait = MSTATE_NON;
          measState++;
        }
        break;
      default:
        break;
    }
  }
}


/**
 * @brief Функции обработки состояния MEASST_END_PROBE системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateEnd( void ){
  // case MEASST_END_PROB:   // Система закончила забор проб
  if( measRun == SET ){
    switch( measRunWait ){
      case MSTATE_NON:
        zoomOff();
        // TODO: Запуск оптавки данных
        measDev.status.sendStart = SET;
        measRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( measDev.status.sent ){
          measRunWait = MSTATE_NON;
          measState++;
        }
        break;
      default:
        break;
    }
  }
}


/**
 * @brief Функции обработки состояния MEASST_PROC системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateProc( void ){
  //    case MEASST_PROC:  // Данные
  if( measRun == SET ){
    switch( measRunWait ){
      case MSTATE_NON:
#if DEBUG_TRACE_RUN
        trace_puts(" MEAS data sent");
#endif
        measRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        measRunWait = MSTATE_NON;
        measState = MEASST_FIN;
        break;
      default:
        break;
    }
  }
}


/**
 * @brief Функции обработки состояния MEASST_FIN системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateFin( void ){
  //    case MEASST_FIN
  if( measRun == SET ){
    zoomOff();
    measRunWait = MSTATE_NON;
    measState = MEASST_OFF;
#if DEBUG_TRACE_RUN
    trace_puts(" MEAS fin");
#endif
  }
}


/**
 * @brief Функции обработки состояния MEASST_FAULT системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateFault( void ){
// case MEASST_FAULT:   // Ошибка выполнения измерения
  if( measRun == SET ){
    // Включаем систему
    switch( measRunWait ){
      case MSTATE_NON:
        // Запуск троекратного зума
        zoomOff();
        measDev.tout = mTick + 1000;
        measDev.count = 0;
        measRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( measDev.tout < mTick ){
          zoomOn();
          measDev.tout = mTick + 300;
          measRunWait = MSTATE_ON_OK;
        }
        break;
      case MSTATE_ON_OK:
        if( measDev.tout < mTick ){
          zoomOff();
          if( ++measDev.count == 3 ){
            // Пропикало Третий раз
            measState = MEASST_OFF;
            measRun = RESET;
            measRunWait = MSTATE_NON;
#if DEBUG_TRACE_RUN
            trace_puts("Measure fault");
#endif
          }
          else {
            measDev.tout = mTick + 300;
            measRunWait = MSTATE_ON;
          }
        }
        break;
      default:
        break;
    }
  }
}


#endif /* STATEFUNC_C_ */
