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
#include "usb_vcp.h"
#include "statefunc.h"

// static uint32_t tmptick = -1;
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
    trace_printf("cont0:%d cont:%d\n", measDev.prmContinuous, measDev.status.cont);
  }

  if( measRun == SET ){
    // Направление - off->on

    zoomOn();
    assert_param( measDev.status.pressOk == RESET );
    measDev.tout = mTick + MEAS_TIME_MAX;
    measStartClean();
    measRunWait = MSTATE_NON;
    trace_printf(":On begin\n");
    trace_printf("1 cont0:%d cont:%d\n", measDev.prmContinuous, measDev.status.cont);
    measState++;
  }
  else {
    if( measRunWait == MSTATE_NON ){
#if SIMUL
      if( adcHandle.adcData[ADC_PRM_PRESS].prm > 1 ){
        return;
      }
#endif // SIMUL
      timerMod( &measOnCanTimer, TOUT_1000 * 10 );
      gpioPinSetNow( &gpioPinRelEn );
      measRunWait = MSTATE_OFF;
#if DEBUG_TRACE_RUN
      trace_write(":SYS OFF\n", 9);
#endif
      trace_printf("0 cont0:%d cont:%d\n", measDev.prmContinuous, measDev.status.cont);
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
          measDev.status.pressOk = SET;
          // Запуск соленоида
          measDev.status.relStart = SET;
          trace_printf(":Solenoid start\n");
          measRunWait = MSTATE_NON_2;
        }
        break;
      case MSTATE_NON_2:
        if( measDev.status.relEnd ){
          measDev.secsStart2 = secs;
          measDev.msecStart2 = msecs;
#if DEBUG_TRACE_RUN
          trace_printf(":Solenoid stop\n");
#endif
#if SIMUL
          measDev.status.alcoSimOn = SET;
#endif //SIMUL

          measDev.tout = mTick + ALCO_TOUT_MIN;
          measRunWait = MSTATE_ON;
        }
        break;
      case MSTATE_ON:
        if( measDev.status.alcoHi || (measDev.tout < mTick)){
          // Alco превысил порог - можно измерять. Или таймаут превышения порога
          measDev.status.measStart = SET;
          measDev.status.alcoLow = RESET;
#if DEBUG_TRACE_RUN
          trace_printf(":Meas start\n");
#endif
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
 * @brief Функции обработки состояния MEASST_FLOW_PROBE системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateFlow( void ){
  // case MEASST_FLOW_PROB:   // Система закончила забор проб
  if( measRun == SET ){
    if( measDev.status.alcoLow /*|| (measDev.dataNum == MEAS_SEQ_NUM_MAX)*/ ){
      // Значение ALCO меньше порогового - закончили забор проб
      measDev.secsStop = secs;
      measDev.msecStop = msecs;
      measDev.status.measStart = RESET;
      measDev.status.pressOk = RESET;
      measDev.status.alcoHi = RESET;
#if DEBUG_TRACE_RUN
      trace_printf(":Alko low. Meas stop\n");
#endif
      assert_param( measRunWait == MSTATE_NON );
      measState++;
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
        measDev.status.relEnd = RESET;
        // TODO: Запуск оптавки данных
#if DEBUG_TRACE_RUN
        trace_printf(":Send start\n");
#endif
        measDev.status.sendStart = SET;
        measRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( measDev.status.sent || measDev.status.cont ){
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
//inline void stateProc( void ){
//  //    case MEASST_PROC:  // Данные
//  if( measRun == SET ){
//    switch( measRunWait ){
//      case MSTATE_NON:
//#if DEBUG_TRACE_RUN
//        trace_puts(":Meas data sent");
//#endif
//        measRunWait = MSTATE_NON;
//        measState = MEASST_FIN;
//        break;
//      default:
//        break;
//    }
//  }
//}


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
#if DEBUG_TRACE_RUN
    trace_puts(":Meas data sent - fin");
#endif
    zoomOff();
    measDev.status.relEnd = RESET;
    measRunWait = MSTATE_NON;
    measState = MEASST_OFF;
    measRun = RESET;
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
#if DEBUG_TRACE_RUN
        trace_puts("Measure fault");
#endif
        break;
      case MSTATE_ON:
        if( measDev.tout < mTick ){
          zoomOn();
          measDev.tout = mTick + 300;
          measRunWait = MSTATE_ON_OK;
#if DEBUG_TRACE_RUN
          trace_printf(":Fault on %d\n", measDev.count );
#endif
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
