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


// Включение линии питания
void pwrEnOn( eRun runnum, FlagStatus autorun ){
  // Выключаем линию питания
  gpioPinSetNow( &(gpioPinRun[runnum]) );
//  i2cSetRelevant( false, runnum );
  mcuRunWait = MSTATE_ON;
  if( autorun ){
    // Автоматическая последовательность включения (по таймауту)
    timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT );
  }
}

// Выключение линии питания
void pwrEnOff( eRun runnum ){
  // Выключаем питание ПЛИС FPGA

  timerDel( &pwrOnUpdateTimer );
  gpioPinResetNow( &(gpioPinRun[runnum]) );
//  i2cSetRelevant( false, runnum );
  mcuRunWait = MSTATE_OFF;
  timerMod(&pwrOffUpdateTimer, PWR_EN_STATE_TOUT );
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
    measState++;
  }
}


/**
 * @brief Функции обработки состояния MEASST_SYS_START системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateStart( void ){
// case MCUSTATE_SYS_START:   // Блок питания включен
  if( measRun == SET ){
    // Направление - off->on
    switch( measRunWait ){
      case MSTATE_NON:
        if( psuDev.stReady ){
          // Статистика готова
          if( ((psuDev.u16status & ~0x0820) == 0) || (psuDev.u16status == 0) ){
            // Статус PSU = off + nPgood, остальное - ОК
            extiPgOn( PG_PSU );
            psuCmd( SET );
            timerModArg(&pwrOnToutTimer, PWR_EN_STATE_TOUT5, mcuState );
            mcuRunWait = MSTATE_ON;
    #if DEBUG_TRACE_RUN
          trace_puts(" PSU_ON on");
    #endif
          }
          else {
            ledToggleSet( LED_BM_ERR, LED_FAST_TOGGLE_TOUT, LED_FAST_TOGGLE_TOUT, 0);
            mcuRunWait = MSTATE_NON_2;
          }
        }
        else if( timerPending( &pwrOnToutTimer ) == RESET) {
          timerModArg(&pwrOnToutTimer, PWR_EN_STATE_TOUT5, mcuState );
        }
        break;
      case MSTATE_ON:
        if( pgFlags.pgPsuState ){
          // Включаем цепи I2C для устройств +12V
          if( i2cDevUpdate( mcuRun, RUN_NUM ) ){
            return;
          }
          // Включаем прерывание по линии PG_PSU
          timerDel( &pwrOnToutTimer );
          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT1 );
          mcuRunWait = MSTATE_ON_OK;
        }
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем PSU
    switch( mcuRunWait ){
      case MSTATE_NON:
        if( i2cDevUpdate( mcuRun, RUN_NUM ) ){
          return;
        }
        // PSU_ON Выключаем
        extiPgOff( PG_PSU );
        psuCmd( RESET );
        mcuRunWait = MSTATE_OFF;
        tmptick = mTick + PWR_EN_STATE_TOUT5;
#if DEBUG_TRACE_RUN
        trace_puts(" PSU_ON off");
#endif
        break;
      case MSTATE_OFF:
        if( psuDev.status.psuOff == SET ){
          timerDel( &pwrOnToutTimer );
          if( alertFlag == SET ){
            mcuRunWait = MSTATE_NON;
            mcuState--;
          }
          else {
            timerMod(&pwrOffUpdateTimer, PWR_EN_STATE_TOUT2 );
            mcuRunWait = MSTATE_OFF_OK;
          }
          timerMod( &peakResetTimer, TOUT_100 );
        }
        else if (tmptick < mTick ){
          // Время ожидания снятия напряжения +12V вышло - "Начинай сказку сначала!"
          mcuRunWait = MSTATE_NON;
        }
        break;
      default:
        break;
    }
  }
}

/**
 * @brief Функции обработки состояния MCUSTATE_PSU_START системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void statePsuOn( void ){

  if( mcuRun == SET ){
    // Направление - off->on
    switch( mcuRunWait ){
      case MSTATE_NON:
        if( i2cGetRelevant( RUN_NUM ) ){
          mcuRunWait = MSTATE_ON;
          timerModArg(&pwrOnToutTimer, I2C_READY_TOUT, mcuState );
        }
        break;
      case MSTATE_ON:
      {
        // Релевантность данных установилась

        if( (idx = i2cDevReady()) == CONFIG_I2C_NUM_MAX ){
  #if DEBUG_TRACE_RUN
          trace_puts(" SYS_Run: PSU_ON->I2C_ON");
  #endif
          // Устройства контроля на I2C готовы - можно реагировать на включение системы
          timerDel( &pwrOnToutTimer );
          mcuState = MCUSTATE_I2C_ON;
          mcuRunWait = MSTATE_NON;
        }
        break;
      default:
        break;
      }
    }
  }
  else{
    // Выключаем систему
#if DEBUG_TRACE_RUN
  trace_puts("SYS_START->SYS_OFF");
#endif
    timerDel( &pwrOnToutTimer );
    mcuRunWait = MSTATE_NON;
    mcuState--;
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_I2C_ON системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateI2cOn( void ){
  //    case MCUSTATE_I2C_ON:  // Система контроля за питанием на I2C готова
  if( mcuRun == SET ){
    // Включаем систему
    switch( mcuRunWait ){
      case MSTATE_NON:
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: Fan test");
#endif
        mcuRunWait = SET;
        fanStartTest = SET;
        fanFullEnable();
        mcuRunWait = MSTATE_OFF;
        break;
      case MSTATE_OFF:
  #if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: I2C_ON->FAN_ON");
  #endif
        mcuRunWait = MSTATE_NON;
        mcuState = MCUSTATE_FAN_ON;
        break;
      default:
        break;
    }
  }
  else {
    // Направление off->on : просто продолжаем выключать систему
#if DEBUG_TRACE_RUN
  trace_puts(" SYS_Run: I2C_ON->PS_ON");
#endif
    fanFullDisable();
    mcuRunWait = MSTATE_NON;
    mcuState--;
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_FAN_ON системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateFanOn( void ){
  //    case MCUSTATE_FAN_ON
  if( mcuRun == SET ){
    // Включаем систему

    switch( mcuRunWait ){
      case MSTATE_NON:
        gpioPinSetNow( &gpioPinMzuProg );
        // Включаем первое питание ПЛИС FPGA
        pwrEnOn(RUN_1, RESET);
        timerModArg(&pwrOnToutTimer, TOUT_1000, mcuState );
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: RUN_1 ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( pgRun1Test() ){
          timerDel( &pwrOnToutTimer );
          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT1 );
          mcuRunWait = MSTATE_ON_OK;
        }
        break;
      default:
        break;
    }
  }
  else {
    // Выключаем систему
    if( mcuRunWait == MSTATE_NON ){
      // FPGA_EN1 Выключаем
      timerDel( &pwrOnToutTimer );
      pgRun1Reset();
      pwrEnOff( RUN_1 );
      timerDel( &pwrOffUpdateTimer );
      timerModArg( &pwrOffToutTimer, 30 * TOUT_1000, mcuState );
  #if DEBUG_TRACE_RUN
      trace_puts(" SYS_Run: RUN_1_OFF");
  #endif
      if( intel.runFlag == RESET ){
        // Выключаем +12V_INTEL
        pwrEnOff( RUN_I12V );
        mcuRunWait = MSTATE_OFF_OK;
      }
      else {
        mcuRunWait = MSTATE_NON_2;
      }
    }
    else if( mcuRunWait == MSTATE_NON_2 ){
      if( (gpioPinReadNow( &(gpioPinSlpS[SLP_S3]) ) == Bit_RESET) ||     // SLP_S3 - INTEL выключается
          (intel.iErr != I_ERR_OK ) ) {              // Превышение таймаута выключения INTEL
        timerModArg( &pwrOffToutTimer, 10 * TOUT_1000, mcuState );
        mcuRunWait = MSTATE_OFF;
      }
    }
    else if( mcuRunWait == MSTATE_OFF ){
      // Ждем остановки INTEL
      if( (gpioPinReadNow( &(gpioPinSlpS[SLP_S5]) ) == Bit_RESET ) ||    // SLP_S5 - INTEL выключился
          (intel.iErr != I_ERR_OK ) )              // Превышение таймаута выключения INTEL
      {
        timerDel( &pwrOffToutTimer );
        intel.runFlag = RESET;
        // Выключаем +12V_INTEL
        pwrEnOff( RUN_I12V );
        timerMod( &pwrOffUpdateTimer, TOUT_200 );
        mcuRunWait = MSTATE_OFF_OK;
      }

    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_RUN_1 системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateRun1( void ){
// case MCUSTATE_RUN_1:   // Первое питание ПЛИС FPGA включено
  if( mcuRun == SET ){
    // Включаем систему
    switch( mcuRunWait ){
      case MSTATE_NON:
        // Включаем первое питание ПЛИС FPGA
        pwrEnOn(RUN_2, RESET);
        timerModArg(&pwrOnToutTimer, TOUT_1000, mcuState );
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: RUN_2 ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( pgRun2Test() ){
          timerDel( &pwrOnToutTimer );
          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT1 );
          mcuRunWait = MSTATE_ON_OK;
          extiPgRun2On();
        }
        break;
      default:
        break;
    }
  }
  else {
    if( gpioPinRun[RUN_2].state != Bit_RESET ){
      // Выключаем второе питание ПЛИС FPGA_EN2
      timerDel( &pwrOnToutTimer );
      pgRun2Reset();
      pwrEnOff( RUN_2 );
#if DEBUG_TRACE_RUN
      trace_puts("RUN2_OFF");
#endif

    }
    else if(mcuRunWait == MSTATE_NON){
      mcuState--;
    }
  }
}

/**
 * @brief Функции обработки состояния MCUSTATE_RUN_2 системы
  *
  * @param[in]  none
  *
  * @retval none
  */
void stateRun2( void ){
// case MCUSTATE_RUN_2:   // Второе питание ПЛИС FPGA включено
  if( mcuRun == SET ){
    // Включаем систему
    switch( mcuRunWait ){
      case MSTATE_NON:
        // Включаем первое питание ПЛИС FPGA
        extiPgRun3On();
        pwrEnOn(RUN_3, RESET);
        timerModArg(&pwrOnToutTimer, TOUT_1000, mcuState );
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: RUN_3 ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( pgRun3Test() ){
          timerDel( &pwrOnToutTimer );
          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT1 );
          mcuRunWait = MSTATE_ON_OK;
        }
        break;
      default:
        break;
    }
  }
  else {
    if( gpioPinRun[RUN_3].state != Bit_RESET ){
      timerDel( &pwrOnToutTimer );
      // Выключаем третье питание ПЛИС FPGA_EN3
      extiPgRun3Off();
      pwrEnOff( RUN_3 );
#if DEBUG_TRACE_RUN
      trace_puts("RUN3_OFF");
#endif

    }
    else if(mcuRunWait == MSTATE_NON ){
      mcuState--;
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_RUN_3 системы
 *
 * @param[in]  none
 *
 * @retval none
 */
void stateRun3( void ){
//  case MCUSTATE_RUN_3:  // Третье питание ZYNQ_PL включено
  if( mcuRun == SET ){
    switch( mcuRunWait ){
      case MSTATE_NON:
          mcuRunWait = MSTATE_ON;
          // Запускаем PLL
          pllStart( &pll );
          tmptick = 0;
          break;
      case MSTATE_ON:
        if( tmptick == 0 ){
          if( pll.pllLock != RESET ){   // Ждем установки частоты PLL
            // Запустилась PLL
            ledOn( LED_G, 1000);
            // Переходим к следующему этапу
            // Пауза до следующего этапа = 500мс
            tmptick = mTick + 500;
          }
          else if( pll.clkErr != PLL_CLK_ERR_OK ){
            mcuRunWait = MSTATE_NON;
  // Ошибка запуска PLL - выключаем
  //            fpgaOffQuery = SET;
            mcuState = MCUSTATE_PLL_LOCK;
          }
        }
        else if( tmptick < mTick ){
          // Сброс ложных ошибок PLL_LOCK
          pll.clkErr = RESET;
          pll.statusClkLimErr = RESET;

          mcuRunWait = MSTATE_NON;
          mcuState = MCUSTATE_PLL_LOCK;
        }
        break;
      default:
        break;
    }
  }
  else {
    pllDisable();
    mcuState--;
#if DEBUG_TRACE_RUN
      trace_puts("MZU_EN3->EN2");
#endif
  }

}


/**
 * @brief Функции обработки состояния MCUSTATE_PLL_LOCK системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
inline void statePllLock( void ){
//case MCUSTATE_PLL_LOCK:  // PLL Запущен

  if( mcuRun == SET ){
    // Включаем систему
    if( mcuRunWait == MSTATE_NON){
/*
      // Multiboot_Select
      if(sysFlags.mbAddrSetUpdate == RESET ){
        sysFlags.mbootAddrSet = (gpioPinReadNow( &gpioPinMzuMbSel ) != Bit_RESET);
      }
      else {
        sysFlags.mbAddrSetUpdate = RESET;
      }
      gpioPinCmdNow( &gpioPinMzuMb, sysFlags.mbootAddrSet );
*/

      // Снимаем MZU_PROG
      gpioPinResetNow( &gpioPinMzuProg );
#if JTAG_ENABLE
      fpgaDevUpdate( MZU_CHAIN, mcuRun );   // Начинаем работу JTAG_FPGA
#endif // JTAG_ENABLE
      // Таймер ожидания FPGA_DONE
      timerModArg( &pwrOnToutTimer, PWR_EN_STATE_TOUT10 * 5, mcuState );
//      tmptick = mTick + PWR_EN_STATE_TOUT10 * 5;
      mcuRunWait = MSTATE_ON;
    }
    else if( (gpioPinRead(&gpioPinMzuDone) == Bit_SET)
        || (gpioPinReadNow(&gpioPinDbgJmp2) == Bit_RESET) ){
      // Запуск  ZYNQ_PL окончен удачно
      timerDel( &pwrOnToutTimer );
      mcuState = MCUSTATE_MZU_DONE;
      mcuRunWait = MSTATE_NON;
    }
  }
  else {
    // Сейчас просто переходим к выключению питания MZU
#if JTAG_ENABLE
    fpgaDevUpdate( MZU_CHAIN, mcuRun );   // Начинаем работу JTAG_FPGA
#endif // JTAG_ENABLE
    gpioPinSetNow( &gpioPinMzuProg );
    timerDel( &pwrOnToutTimer );
    mcuState--;
  }

}


/**
 * @brief Функции обработки состояния MCUSTATE_MZU_DONE системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */

void stateMzuDone( void ){
//case MCUSTATE_MZU_DONE:  // ПЛИС MZU работает
  if( mcuRun == SET ){
    // Включаем систему
    switch( mcuRunWait ){
      case MSTATE_NON:
        // ----------------  Тестируем связи GPIO MCU <--> ZYNQ_PL ---------------------------
#if FPGA_GPIO_TEST
        fpgaGpioTest();
#endif // FPGA_GPIO_TEST

#if MZU_UART_ENABLE && !FPGA_GPIO_TEST
        uartEnable( mzuHnd.rxh, mzuHnd.txh );    // Запускаем работу UART_MZU
#endif  // MZU_UART_ENABLE
#if DEBUG_TRACE_RUN
        trace_puts("MZU_DONE");
#endif
        // Обнуляем локальные экстремумы
        timerMod( &peakResetTimer, TOUT_100 );

        // Включаем 12В INTEL
        pwrEnOn( RUN_I12V, RESET );
        tmptick = mTick;
        timerModArg(&pwrOnToutTimer, TOUT_1000, mcuState );
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        if( pgCoreTest() ){
          extiPgCoreOn();
          timerDel( &pwrOnToutTimer );
          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT1 );
          mcuRunWait = MSTATE_ON_OK;
        }
        break;
      default:
        break;
    }



  }
  else if( mcuRunWait == MSTATE_NON ){
//    ledOff( LED_PWR, 0);
    mcuRunWait = MSTATE_OFF;

    timerDel( &pwrOnUpdateTimer );

#if MZU_UART_ENABLE && !FPGA_GPIO_TEST
    uartDisable( mzuHnd.rxh, mzuHnd.txh );    // Останавливаем работу UART_MZU
#endif  // MZU_UART_ENABLE
#if DEBUG_TRACE_RUN
    trace_puts("DONE->RUN_3");
#endif
    timerMod(&pwrOffUpdateTimer, PWR_EN_STATE_TOUT10 );
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_IPWR_ON системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
void stateIStart( void ){
//case MCUSTATE_IPWR_ON:  //  Запущен старт INTEL
  if( mcuRun == SET ){
    switch( mcuRunWait ){
      case MSTATE_NON:
        if( gpioPinDsw.state == Bit_SET ){    // Опрашиваем вывод в gpioClock -> gpioPgTest
          // +3.3V_DSW включилось
          // Включаем "PWRBtn_OUT" и ждем 100мс
          sysFlags.iPwrOutSet = SET;
          timerModArg(&pwrOnToutTimer, INTEL_WKUP_TOUT, mcuState );
          mcuRunWait = MSTATE_ON;
        }
        break;
      case MSTATE_ON:
        if( (gpioPinSlpS[SLP_S3].gpio->IDR & gpioPinSlpS[SLP_S3].pin) != RESET ){      // SLP_S3
          // INTEL проснулся и работает
          timerDel(&pwrOnToutTimer);

#if INTEL_UART_ENABLE
          uartEnable( intelHnd.rxh, intelHnd.txh );
#endif // INTEL_UART_ENABLE

          mcuRunWait = MSTATE_NON;
          intel.runFlag = SET;

          mcuState = MCUSTATE_SYS_ON;
#if DEBUG_TRACE_RUN
          trace_puts("SYS_Run: -> SYS_ON");
#endif
        break;
      default:
        break;
      }
    }
  }
  else {
    switch( mcuRunWait ){
      case MSTATE_NON:
        timerDel( &pwrOnToutTimer );

        // Отслеживаем состояние INTEL - SlpS3
        if( gpioPinRead( &gpioPinSlpS[SLP_S3] ) == Bit_RESET ){
          // INTEL уже начал засыпать или выключаться - выключаем систему
          gpioPinResetNow( &gpioPinPwrOut );
          mcuRunWait = MSTATE_OFF;
        }
        else {
          // Выключаем INTEL коротким нажатием кнопки
          intel.commFlag = RESET;
          // Включаем "PWRBtn_OUT" и ждем 100мс
          sysFlags.iPwrOutSet = SET;
          mcuRunWait = MSTATE_OFF;
        }
        break;
      case MSTATE_OFF:
        if( gpioPinPwrOut.state == Bit_RESET ){
          // Имитация нажатия кнопки INTEL_PWR закончилось

          extiPgCoreOff();

#if INTEL_UART_ENABLE
          uartDisable( intelHnd.rxh, intelHnd.txh );
#endif // INTEL_UART_ENABLE

          // Также запускаем таймаут выключения INTEL - Slp_S5 (~30 сек)
          timerModArg(&pwrOffToutTimer, INTEL_OFF_TOUT, mcuState );
          mcuRunWait = MSTATE_NON;
          mcuState--;
        }
        break;
      default:
        break;
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_SYS_ON системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
void stateSysOn( void ){
  if( mcuRun == SET ){
    switch( mcuRunWait ) {
      case MSTATE_NON:
        mcuRunWait = MSTATE_ON;
#if DEBUG_TRACE_RUN
        trace_puts("SYS_ON");
#endif
        // Выключаем Желтый светодиод
        ledOff( LED_PWR_Y, 0 );
        // Включаем моргание зеленого светодиода
        ledToggleSet( LED_G, LED_BLINK_ON_TOUT, LED_BLINK_OFF_TOUT, 0 );
        // Выключаем светодиод POWER
        ledOn( LED_PWR_G, 0 );
        break;
      case MSTATE_ON:
        if( fpgaOnNeed ){
          fpgaOnCan = RESET;
          fpgaOnNeed = RESET;
          mcuRunWait = MSTATE_ON_OK;
          fpgaRun = SET;
          i2cDevUpdate( fpgaRun, RUN_FPGA_1);
          i2cDevUpdate( fpgaRun, RUN_FPGA_2);
          i2cDevUpdate( fpgaRun, RUN_FPGA_3);
          ledOff( LED_BM_ERR, 0 );
//          timerMod(&pwrOnUpdateTimer, PWR_EN_STATE_TOUT5 );
        }
        break;
      case MSTATE_ON_OK:
        if( (idx = i2cDevReady()) == CONFIG_I2C_NUM_MAX ){
          mcuState++;
          mcuRunWait = MSTATE_NON;
        }
        break;
      default:
        break;
    }
  }
  else {
    if( mcuRunWait == MSTATE_NON ){
      mcuRunWait = MSTATE_OFF_OK;
    }
    else {
      // Выключение системы
      ledOff( LED_PWR_G, 0 );
      ledToggleSet( LED_PWR_Y, LED_TOGGLE_TOUT, LED_TOGGLE_TOUT, 0 );
      mcuRunWait = MSTATE_NON;
      mcuState--;
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_START системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateFpgaStart( void ){
// case MCUSTATE_FPGA_START:   // Запуск FPGA_VP12
  if( fpgaRun == SET ){
    // Направление - off->on
    switch ( mcuRunWait ){
      case MSTATE_NON:
        // Устанвливаем FPGA_PROG
        gpioPinSetNow( &gpioPinFpgaProg );

        // Включаем первое питание ПЛИС FPGA
        extiPgFpgaRun1On();
        pwrEnOn( RUN_FPGA_1, RESET );
//        timerMod(&pwrOffUpdateTimer, TOUT_500 );
        tmptick = mTick;
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: FPGA_EN1_ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        timerModArg( &pwrOnToutTimer, TOUT_1000, mcuState );
        mcuRunWait = MSTATE_ON_OK;
        break;
      case MSTATE_ON_OK:
        if( pgFpgaRun1Test() ){
          timerDel( &pwrOnToutTimer );
          mcuRunWait = MSTATE_NON;
          mcuState++;
        }
        break;
      default:
        break;
    }
  }
  else {
    switch( mcuRunWait ){
      case MSTATE_NON:
        timerDel( &pwrOnToutTimer );
        pwrEnOff( RUN_FPGA_1 );
    #if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: FPGA_EN1_OFF");
    #endif
        mcuRunWait = MSTATE_OFF;
        break;
      case MSTATE_OFF:
      {
        uint32_t tout;
        if( alertFlag ){
      // Возникла авария - Выдержим большую паузу для возможности включения
          tout = TOUT_2000 * 2;
        }
        else {
          tout = TOUT_200;
        }
        timerMod( &fpgaOnCanTimer, tout );
        // Сбрасываем пиковые значения для VP1 и VP2
        timerMod( &vpPeakResetTimer, TOUT_100 );
        mcuRunWait = MSTATE_OFF_OK;
        break;
      }
//      case MSTATE_OFF_OK:
//        mcuRunWait = MSTATE_NON;
//        mcuState--;
//#if DEBUG_TRACE_RUN
//        trace_puts("FPGA_START->SYS_ON");
//#endif
//        break;
      default:
        break;
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_1 системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
inline void stateFpga1( void ){
// case MCUSTATE_FPGA_1:   // Первое питание ПЛИС FPGA включено
  if( fpgaRun == SET ){
    // Включаем систему
    switch ( mcuRunWait ){
      case MSTATE_NON:
        // Включаем второе питание ПЛИС FPGA
        pwrEnOn( RUN_FPGA_2, RESET );
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: FPGA_EN2_ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        timerModArg( &pwrOnToutTimer, TOUT_1000, mcuState );
        mcuRunWait = MSTATE_ON_OK;
        break;
      case MSTATE_ON_OK:
        if( pgFpgaRun2Test() ){
          timerDel( &pwrOnToutTimer );
          mcuRunWait = MSTATE_NON;
          mcuState++;
        }
        break;
      default:
        break;
    }
  }
  else {
    if( mcuRunWait == MSTATE_NON ){
//      assert_param( gpioPinRunFpga2.state != Bit_RESET );
      timerDel( &pwrOnToutTimer );
      // Выключаем второе питание ПЛИС FPGA_EN2
      pwrEnOff( RUN_FPGA_2 );
#if DEBUG_TRACE_RUN
      trace_puts(" SYS_Run: FPGA_EN2_OFF");
#endif
    }
  }
}



/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_2 системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateFpga2( void ){
// case MCUSTATE_FPGA_2:   // Второе питание ПЛИС FPGA включено
  if( fpgaRun == SET ){
    // Включаем систему
    switch ( mcuRunWait ){
      case MSTATE_NON:
        // Включаем второе питание ПЛИС FPGA
        extiPgFpgaRun3On();
        pwrEnOn( RUN_FPGA_3, RESET );
        timerDel( &pwrOnUpdateTimer );
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: FPGA_EN3_ON");
#endif
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
        timerModArg( &pwrOnToutTimer, TOUT_1000, mcuState );
        mcuRunWait = MSTATE_ON_OK;
        break;
      case MSTATE_ON_OK:
        if( pgFpgaRun3Test() ){
          timerDel( &pwrOnToutTimer );
          // Сбрасываем пиковые значения для VP1 и VP2
          timerMod( &vpPeakResetTimer, TOUT_100 );
          mcuRunWait = MSTATE_NON;
          mcuState++;
        }
        break;
      default:
        break;
    }
  }
  else {
    if( mcuRunWait == MSTATE_NON ){
//      assert_param( gpioPinRunFpga2.state != Bit_RESET );
      timerDel( &pwrOnToutTimer );
      // Выключаем третье питание ПЛИС FPGA_EN3
      pgFpgaRun3Reset();
      pgFpgaRun2Reset();
      pgFpgaRun1Reset();
      pwrEnOff( RUN_FPGA_3 );
#if DEBUG_TRACE_RUN
      trace_puts(" SYS_Run: FPGA_EN3_OFF");
#endif
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_3 системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
void stateFpga3( void ){
  //  case MCUSTATE_FPGA_3:  // Третье питание ПЛИС FPGA включено
  if( fpgaRun == SET ){
    // Включаем систему
    switch( mcuRunWait ){
      case MSTATE_NON:
#if DEBUG_TRACE_RUN
        trace_puts(" SYS_Run: PLL_LOCK->FPGA_DONE");
#endif
        gpioPinResetNow( &gpioPinFpgaProg );
#if JTAG_ENABLE
        fpgaDevUpdate( FPGA_CHAIN, fpgaRun );   // Начинаем работу JTAG_FPGA
#endif // JTAG_ENABLE
        timerModArg( &pwrOnToutTimer, FPGA_DONE_TOUT, mcuState);
        mcuRunWait = MSTATE_ON;
        break;
      case MSTATE_ON:
//        if( (gpioPinReadNow(&gpioPinFpgaDone) == Bit_SET)
//            || gpioPinReadNow(&gpioPinDbgJmp2) == Bit_RESET){
          // Запуск  ПЛИС FPGA окончен удачно
          // Установлен JMP2 - игнорир. FPGA_DONE
          gpioPinSetNow(  &gpioPinMzuRsrv2 );
          timerDel( &pwrOnToutTimer );
          mcuState = MCUSTATE_FPGA_DONE;
          mcuRunWait = MSTATE_NON;
//        }
        break;
      default:
        break;
    }
  }
  else {
    // Если выключаем FPGA
    if( mcuRunWait == MSTATE_NON ){
      timerDel( &pwrOnToutTimer );
      // Сброс FPGA
      gpioPinResetNow(  &gpioPinMzuRsrv2 );
      gpioPinSetNow( &gpioPinFpgaProg );

#if JTAG_ENABLE
      fpgaDevUpdate( FPGA_CHAIN, fpgaRun );   // Останавливаем работу JTAG_FPGA
#endif // JTAG_ENABLE
      timerMod(&pwrOffUpdateTimer, PWR_EN_STATE_TOUT10 );
      mcuRunWait = MSTATE_OFF;
#if DEBUG_TRACE_RUN
      trace_puts(" SYS_Run: FPGA_2->FPGA_1");
#endif
    }
  }
}


/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_DONE системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
void stateFpgaDone( void ){
//case MCUSTATE_FPGA_DONE:  // ПЛИС FPGA работает
  if( fpgaRun == SET ){
    // Включаем систему
#if FPGA_GPIO_TEST
      fpgaGpioTest();
#endif // FPGA_GPIO_TEST

#if FPGA_UART_ENABLE
      uartEnable( fpgaHnd.rxh, fpgaHnd.txh );
#endif  // FPGA_UART_ENABLE
      mcuState = MCUSTATE_FPGA_ON;
  }
  else {
#if FPGA_UART_ENABLE
      uartDisable( fpgaHnd.rxh, fpgaHnd.txh );
#endif  // FPGA_UART_ENABLE
    mcuState--;
  }
}



/**
 * @brief Функции обработки состояния MCUSTATE_FPGA_ON системы
 *
 * @param[in]  self  дескриптор интерфейса
 *
 * @retval none
 */
void stateFpgaOn( void ){
  if( fpgaRun == SET ){
    if( mcuRunWait == MSTATE_NON ){
      mcuRunWait = MSTATE_ON;
#if DEBUG_TRACE_RUN
      trace_puts(" SYS_Run: FPGA_ON");
#endif

//      tmptick = mTick + 1000;
//    }
//    else if(tmptick < mTick ){
//      sysOff();
    }
  }
  else {
    mcuRunWait = MSTATE_NON;
    mcuState--;
#if DEBUG_TRACE_RUN
    trace_puts(" SYS_Run: FPGA_ON->FPGA_DONE");
#endif
  }
}


#endif /* STATEFUNC_C_ */
