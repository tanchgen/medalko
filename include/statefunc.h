/*
 * statefunc.h
 *
 *  Created on: 11 авг. 2020 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef STATEFUNC_H_
#define STATEFUNC_H_

#include "gpio_arch.h"

typedef enum {
  MSTATE_NON,
  MSTATE_NON_2,
  MSTATE_OFF_OK,
  MSTATE_OFF,
  MSTATE_ON,
  MSTATE_ON_OK,

} eMiniState;

extern eMiniState measurRunWait;

void stateOff( void );

/**
 * @brief Функции обработки состояния MCUSTATE_SYS_START системы
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void stateStart( void );


static inline void extiPgOn( ePgood pg ){
  pgOnFlag[pg] = SET;
}

static inline void extiPgOff( ePgood pg ){
  pgOnFlag[pg] = RESET;
}

/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_1
  *
  * @retval код результата теста
  */
inline FlagStatus pgRun1Test( void ){

  return (i2cDevInfo[I2C_LTM_1V_MZU].devState->pgood);
}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_1
  *
  * @retval код результата теста
  */
inline void pgRun1Reset( void ){

  i2cDevInfo[I2C_LTM_1V_MZU].devState->pgood = RESET;
}


/**
  * @brief  Включение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_2
  *
  * @retval код результата теста
  */
inline void extiPgRun2On( void ){
  pg4vOnFlag = SET;
}


/**
  * @brief  Выключение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_2
  *
  * @retval код результата теста
  */
inline void extiPgRun2Off( void ){
  pg4vOnFlag = RESET;
}


/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_2
  *
  * @retval код результата теста
  */
inline FlagStatus pgRun2Test( void ){
  return (i2cDevInfo[I2C_LTM_1V8_MZU].devState->pgood && pgFlags.pg4v0State);
}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_2
  *
  * @retval код результата теста
  */
inline void pgRun2Reset( void ){
  i2cDevInfo[I2C_LTM_1V8_MZU].devState->pgood = RESET;
  extiPgRun2Off();
}


/**
  * @brief  Включение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_3
  *
  * @retval код результата теста
  */
static inline void extiPgRun3On( void ){
  extiPgOn( PG_AVTT_MZU );
}


/**
  * @brief  Выключение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_3
  *
  * @retval код результата теста
  */
static inline void extiPgRun3Off( void ){
  extiPgOff( PG_AVTT_MZU );
}


/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_3
  *
  * @retval код результата теста
  */
static inline FlagStatus pgRun3Test( void ){
  return pgFlags.pgAvttMzuState;
}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_3
  *
  * @retval код результата теста
  */
static inline void pgRun3Reset( void ){
  extiPgRun3Off();
}


/**
  * @brief  Включение прерывания сигнала extiPinPg для напряжения I_CORE
  *
  * @retval код результата теста
  */
static inline void extiPgCoreOn( void ){
  pgCoreOnFlag = SET;
//  extiPgOn( PG_GT );
//  extiPgOn( PG_VCCSA );
}


/**
  * @brief  Выключение прерывания сигнала extiPinPg для напряжения I_CORE
  *
  * @retval код результата теста
  */
static inline void extiPgCoreOff( void ){
  pgCoreOnFlag = RESET;
//  extiPgOff( PG_GT );
//  extiPgOff( PG_VCCSA );
}

/**
  * @brief  Проверка сигнала POWERGOOD для напряжения I_CORE
  *
  * @retval код результата теста
  */
inline FlagStatus pgCoreTest( void ){
//  return (pgFlags.pgCoreState && pgFlags.pgGtState && pgFlags.pgVccsaState);
  return (pgFlags.pgCoreState);
}

/**
  * @brief  Включение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_FPGA_1
  *
  * @retval код результата теста
  */
static inline void extiPgFpgaRun1On( void ){
  extiPgOn( PG_0v95_VP1 );
  extiPgOn( PG_AVCC_VP1 );
  extiPgOn( PG_0v95_VP2 );
  extiPgOn( PG_AVCC_VP2 );
}


/**
  * @brief  Выключение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_FPGA_1
  *
  * @retval код результата теста
  */
static inline void extiPgFpgaRun1Off( void ){
  extiPgOff( PG_0v95_VP1 );
  extiPgOff( PG_AVCC_VP1 );
  extiPgOff( PG_0v95_VP2 );
  extiPgOff( PG_AVCC_VP2 );
}


/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_1
  *
  * @retval код результата теста
  */
static inline FlagStatus pgFpgaRun1Test( void ){
  return (pgFlags.pg0v95Vp1State && pgFlags.pg0v95Vp2State
          && pgFlags.pgAvccVp1State && pgFlags.pgAvccVp2State);

}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_1
  *
  * @retval код результата теста
  */
static inline void pgFpgaRun1Reset( void ){
  i2cDevInfo[I2C_LTM_0V95_VP10].devState->pgood = RESET;
  i2cDevInfo[I2C_LTM_0V95_VP11].devState->pgood = RESET;
  i2cDevInfo[I2C_LTM_0V95_VP20].devState->pgood = RESET;
  i2cDevInfo[I2C_LTM_0V95_VP21].devState->pgood = RESET;
  extiPgFpgaRun1Off();
}


/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_2
  *
  * @retval код результата теста
  */
static inline FlagStatus pgFpgaRun2Test( void ){
  return (i2cDevInfo[I2C_LTM_1V8_VP1].devState->pgood
          && i2cDevInfo[I2C_LTM_1V8_VP2].devState->pgood);
}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_2
  *
  * @retval код результата теста
  */
static inline void pgFpgaRun2Reset( void ){
  i2cDevInfo[I2C_LTM_1V8_VP1].devState->pgood = RESET;
  i2cDevInfo[I2C_LTM_1V8_VP2].devState->pgood = RESET;
}


/**
  * @brief  Включение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_FPGA_3
  *
  * @retval код результата теста
  */
static inline void extiPgFpgaRun3On( void ){
  extiPgOn( PG_AVTT_VP1 );
  extiPgOn( PG_AVTT_VP2 );
}


/**
  * @brief  Выключение прерывания сигнала extiPinPg для напряжений, включаемых по RUN_FPGA_3
  *
  * @retval код результата теста
  */
static inline void extiPgFpgaRun3Off( void ){
  extiPgOff( PG_AVTT_VP1 );
  extiPgOff( PG_AVTT_VP2 );
}

/**
  * @brief  Проверка сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_3
  *
  * @retval код результата теста
  */
static inline FlagStatus pgFpgaRun3Test( void ){
  return (pgFlags.pgAvttVp1State && pgFlags.pgAvttVp2State );
}


/**
  * @brief  Сброс сигнала POWERGOOD для напряжений, включаемых по RUN_FPGA_3
  *
  * @retval код результата теста
  */
static inline void pgFpgaRun3Reset( void ){
  i2cDevInfo[I2C_LTM_3V3_VP1].devState->pgood = RESET;
  i2cDevInfo[I2C_LTM_3V3_VP2].devState->pgood = RESET;
  extiPgFpgaRun3Off();
}


#endif /* STATEFUNC_H_ */
