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
#define I2C_FS_SPEED            400000UL
#define PRESS_I2C_ADDR      (0x6D << 1)

#define I2C_WRITE_BIT       0x1
#define I2C_READ_BIT        0x0

#define I2C_PACK_MAX      10

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
#define GZP6859_PCFG_OSR_15     0x7     // 32768

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
  I2C_STATE_START,
  I2C_STATE_WRITE,
  I2C_STATE_READ,
  I2C_STATE_END,
  I2C_STATE_ERR
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

// Список операций конфигурации и работы датчика
typedef enum _pressProgr {
  PRESS_CFG_P,             // Конфигурация #0
  PRESS_CFG_SYS_W,             // Конфигурация #1
  PRESS_START,             // Запуск измерений
  PRESS_WAIT,              // Ожидание окончания замеров
  PRESS_DATA_READ,         // Считывание результатов
  PRESS_PROGR_NUM
} ePressProgr;

// ------------------ REG CMD ------------------------------------------------------
typedef union _cmdreg {
  struct {
    uint8_t measCtrl: 3;      // Режим работы датчика
    uint8_t sco: 1;           // Запуск-останов сбора данных
    uint8_t sleep: 4;         // Время сна
  };
  uint8_t u8cmd;

} uCmdReg;

#define CMD_MEAS_CTRL       0x2
#define CMD_SCO             (0x1<<3)
#define CMD_SLEEP_62MS      (0x1<<4)    // 62.5ms
#define CMD_SLEEP_1S        (0xF<<4)    // 1s
// ---------------------------------------------------------------------------------
// ------------------ REG SYS_CFG --------------------------------------------------
typedef union _syscfg {
  struct {
    uint8_t diag: 1;          // Вкл/выкл диагностики
    uint8_t outCtrl: 1;       // Вывод данных с учетом калибровки/ вывод сырых данных
    uint8_t unipolar: 1;      // Однополярный / двуполярный сигнал на выходе
    uint8_t ldo: 1;           // 1.8V / 3.6V LDO
    uint8_t Aout: 4;          // Внутренняя конфигурация - не менять!!
  };
  uint8_t u8syscfg;
} uSysCfgReg;

#define SYSCFG_DIAG         0x1             // 0 - diag_off, 1 - diag_on
#define SYSCFG_OUT_CTRL     (0x0 << 1)      // Выходные данные с учетом калибровки
#define SYSCFG_UNIPOLAR     (0x0 << 2)      // Данные со знаком
#define SYSCFG_LDO          (0x0 << 3)      // LDO = 1.8V
#define SYSCFG_AOUT_MASK    (0xF << 4)      // Маска AOUT
// ---------------------------------------------------------------------------------
// ----------------- REG P_CFG -----------------------------------------------------
typedef union _pcfg {
  struct {
    uint8_t osr: 3;           // Разрядность
    uint8_t gain: 3;          // Усиление
    uint8_t inSwap: 1;        // Полярность сигнала
  };
  uint8_t u8pcfg;
} uPCfgReg;

#define PCFG_OSR            (0x3)           // Разрядность 13бит (0-8191)
#define PCFG_GAIN           (0x0<<3)        // Усиление x1
#define PCFG_INSWAP         (0x0<<6)        // Изменить полярность входного сигнала

// ---------------------------------------------------------------------------------

typedef struct _i2ctrans {
  ePressReg reg;
  uint8_t txData;
  volatile uint8_t txLen;
  volatile uint8_t rxData[3];
  volatile uint8_t rxLen;
  volatile uint8_t rxCount;
  FlagStatus rxDataFlag;  // RESET - оптравляем регистр, 1 - читаем данные из регистра
} sI2cTrans;

typedef struct _pressDev {
  sI2cTrans i2ctrans;

} sPressDev;


#endif /* I2C_H_ */
