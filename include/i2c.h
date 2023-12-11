/*
 * i2c.h
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f10x.h"

#define PRESS_I2C           I2C1
#define PRESS_I2C_SPEED     400000UL
#define PRESS_I2C_ADDR      (0x6D << 1)

#define I2C_WRITE_BIT       0x1
#define I2C_READ_BIT        0x0

typedef enum {
  I2C_STATE_IDLE,
  I2C_STATE_WRITE,
  I2C_STATE_READ,
  I2C_STATE_END
} eI2cState;

typedef enum _alcoreg {
  REG_PRESS_MSB,
  REG_PRESS_CSB,
  REG_PRESS_LSB,
  REG_T_MSB,
  REG_T_LSB,
  REG_CMD,
  REG_SYS_CFG,
  REG_P_CFG,
} eAlcoReg;

typedef enum _alcoProgr {
  ALCO_CFG_0,             // Конфигурация #0
  ALCO_CFG_1,             // Конфигурация #1
  ALCO_START,             // Запуск измерений
  ALCO_WAIT,              // Ожидание окончания замеров
  ALCO_T_L,               // Считывание TEMP_LSB
  ALCO_T_M,               // Считывание TEMP_MSB
  ALCO_P_L,               // Считывание PRESS_LSB
  ALCO_P_C,               // Считывание PRESS_CSB
  ALCO_P_M,               // Считывание PRESS_MSB
  ALCO_PROGR_NUM
} eAlcoProgr;


typedef struct _i2ctrans {
  eAlcoReg regTx;
  uint8_t txData;
  uint8_t txLen;
  eAlcoReg regRx;
  uint8_t rxData[3];
  uint8_t rxLen;
  eI2cState state;
} sI2cTrans;

typedef struct _alcoDev {
  sI2cTrans i2ctrans;

} sAlcoDev;


#endif /* I2C_H_ */
