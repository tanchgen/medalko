/*
 * i2c.h
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f10x.h"

#define PRESS_I2C          I2C1
#define PRESS_I2C_SPEED    400000UL

// ------------------ REG P_CONFIG ------------------------------
// Разрешение
#define GZP6859_PCFG_OSR        0x7     // MASK
#define GZP6859_PCFG_OSR_8      0x4     // 256
#define GZP6859_PCFG_OSR_9      0x5     // 512
#define GZP6859_PCFG_OSR_10     0x0     // 1024
#define GZP6859_PCFG_OSR_11     0x1     // 2048
#define GZP6859_PCFG_OSR_12     0x2     // 4096
#define GZP6859_PCFG_OSR_13     0x3     // 8192
#define GZP6859_PCFG_OSR_14     0x6     // 16384
#define GZP6859_PCFG_OSR_13     0x7     // 32768

// Усиление
#define GZP6859_PCFG_GAIN       (0x7<<3)     // MASK
#define GZP6859_PCFG_GAIN_1     (0x0<<3)     // X1
#define GZP6859_PCFG_GAIN_2     (0x1<<3)     // X2
#define GZP6859_PCFG_GAIN_4     (0x2<<3)     // X4
#define GZP6859_PCFG_GAIN_8     (0x3<<3)     // X8
#define GZP6859_PCFG_GAIN_16    (0x4<<3)     // X16
#define GZP6859_PCFG_GAIN_32    (0x5<<3)     // X32
#define GZP6859_PCFG_GAIN_64    (0x6<<3)     // X64
#define GZP6859_PCFG_GAIN_128   (0x7<<3)     // X128

#define INP_SWAP                (1<<6)       // Inversion input

// ------------------ REG SYS_CONFIG ------------------------------


typedef enum {
  I2C_STATE_IDLE,
  I2C_STATE_WRITE,
  I2C_STATE_READ,
  I2C_STATE_END
} eI2cState;

typedef enum _pressreg {
  REG_PRESS_MSB,
  REG_PRESS_CSB,
  REG_PRESS_LSB,
  REG_T_MSB,
  REG_T_LSB,
  REG_CMD,
  REG_SYS_CFG,
  REG_P_CFG,
  REG_NUM
} ePressReg;

typedef enum _pressProgr {
  PRESS_CFG_0,             // Конфигурация #0
  PRESS_CFG_1,             // Конфигурация #1
  PRESS_START,             // Запуск измерений
  PRESS_WAIT,              // Ожидание окончания замеров
  PRESS_T_L,               // Считывание TEMP_LSB
  PRESS_T_M,               // Считывание TEMP_MSB
  PRESS_P_L,               // Считывание PRESS_LSB
  PRESS_P_C,               // Считывание PRESS_CSB
  PRESS_P_M,               // Считывание PRESS_MSB
  PRESS_PROGR_NUM
} ePressProgr;


typedef struct _i2ctrans {
  ePressReg reg;
  uint8_t txData;
  volatile uint8_t txLen;
  ePressReg regRx;
  volatile uint8_t rxData[3];
  volatile uint8_t rxLen;
} sI2cTrans;

typedef struct _pressDev {
  sI2cTrans i2ctrans;

} sPressDev;


#endif /* I2C_H_ */
