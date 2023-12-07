/*
 * i2c.c
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#include "main.h"
#include "press_i2c.h"

sGpioPin pressPinScl = {GPIOB, 1, GPIO_Pin_6, 6, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin pressPinSda = {GPIOB, 1, GPIO_Pin_7, 7, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };

uint8_t  regAddr[REG_NUM] = {
  0x06,         // REG_PRESS_MSB,
  0x07,         // REG_PRESS_CSB,
  0x08,         // REG_PRESS_LSB,
  0x09,         // REG_T_MSB,
  0x0A,         // REG_T_LSB,
  0x30,         // REG_CMD,
  0xA5,         // REG_SYS_CFG,
  0xA6,         // REG_P_CFG,
};

// Значения регистров по умолчанию
const sPCfgReg pcfgReg = {PCFG_INSWAP | PCFG_GAIN | PCFG_OSR};
const sSysCfgReg syscfgReg = {SYSCFG_LDO | SYSCFG_UNIPOLAR | SYSCFG_OUT_CTRL | SYSCFG_DIAG};
const sCmdReg cmdReg = {CMD_SLEEP_62MS | CMD_SCO | CMD_MEAS_CTRL};

struct _i2cProgrData {
  uint8_t reg;
  uint8_t data;
} i2cProgrData = {
    {regAddr[REG_P_CFG], pcfgReg },
    {regAddr[REG_SYS_CFG], pcfgReg },
    {regAddr[REG_SYS_CFG], pcfgReg },
    {regAddr[REG_CMD], pcfgReg },
};

#define I2C_PACK_MAX      10
sI2cTrans i2cTrans[I2C_PACK_MAX];
volatile uint8_t transCount;
eI2cState i2cState;
ePressProgr pressProgr;
uint32_t pressI2cTout;

FlagStatus i2cRegWrite( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transCount < I2C_PACK_MAX );
  trans = &(i2cTrans[transCount++]);

  trans->reg = reg;
  trans->txData = data;
  trans->txLen = 2;

  return rc;
}

FlagStatus i2cRegRead( uint8_t reg, uint8_t len ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transCount < I2C_PACK_MAX );
  trans = &(i2cTrans[transCount++]);

  trans->reg = reg;
  trans->txLen = 1;
  trans->rxLen = len;

  return rc;
}


// Процес информационного обмена по I2C с PRESS
void pressI2cProc( void ){
  transCount = 0;
  switch( pressProgr ){
    case PRESS_CFG_P:
      // Записываем конфигурацию P_CFG, Считываем SYS_CFG
      i2cRegWrite( regAddr[REG_P_CFG], pcfgReg );
      i2cRegRead( regAddr[REG_SYS_CFG], 1 );
      break;
    case PRESS_CFG_SYS_W:
      // Запись в SYS_CFG
      uint8_t data;

      // Сохраняем AOUT
      data = (i2cTrans[transCount - 1].rxData & SYSCFG_AOUT_MASK) | syscfgReg;
      i2cRegWrite( regAddr[REG_SYS_CFG], data );
      // Конфиг CMD_REG и сразу запуск измерений
      i2cRegWrite( regAddr[REG_CMD], cmdReg );
      pressProgr = PRESS_START;
    case PRESS_START:
      // Запуск измерений - CMD.SCO
      break;
    case PRESS_WAIT:
      // Читаем статус - ждем окончания измерений
      break;
    case PRESS_T_L:
      // Считываем результаты
      break;
    default:
      break;
  }
  if( pressProgr == PRESS_CFG_0 ){
    // Записываем конфигурацию
    i2cRegWrite()
  }
  else if( )
}


void i2cClock( void ){

  switch(i2cState){
    case I2C_STATE_IDLE:
      if( pressI2cTout < mTick ){
        pressI2cProc();
      }
      break;
    case I2C_STATE_WRITE:
      break;
    case I2C_STATE_READ:
      break;
    case I2C_STATE_END:
      if( i2cTrans[transCount].txLen == 0 ){
        // Больше передавать нечего - переходим к следующему пункту программы обмена
        if( ++pressProgr == PRESS_PROGR_NUM ){
          // Дошли до конца списка
          pressI2cTout = mTick + 20;
        }
        else if( pressProgr == PRESS_T_L ){
          // Ожидание окончания измерения
          if( ((sCmdReg)(i2cTrans[transCount - 1].rxData[0])).sco != RESET ){
            // Данные НЕ готовы - проверяем опять
            pressProgr--;
            pressI2cTout = mTick;
          }
        }
        else {
          pressI2cTout = mTick;
        }
      }
      break;
    default:
      break;
  }
}

void i2cEnable( void ){
  PRESS_I2C->CR1 |= I2C_CR1_PE;
  PRESS_I2C->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}


void i2cInit( void ){
  uint8_t freq = (rccClocks.PCLK1_Frequency/1000);
  uint32_t tmp;

  // Data setup
  i2cState = I2C_STATE_IDLE;
  transCount = 0;

  // I2C GPIO setup
  gpioPinSetup( &pressPinScl );
  gpioPinSetup( &pressPinSda );

  // I2C periphery setup
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  PRESS_I2C->CR2 = (PRESS_I2C->CR2 & ~I2C_CR2_FREQ) | freq;
  // In FAST MODE I2C
  PRESS_I2C->TRISE = (freq * 300) / 1000 + 1;

  tmp = rccClocks.PCLK1_Frequency / (PRESS_I2C_SPEED * 3U);
  // For dutycicle = 2
  if( tmp & I2C_CCR_CCR == 0 ){
    tmp = 1;
  }
  PRESS_I2C->CCR = (PRESS_I2C->CCR & ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR)) | I2C_CCR_FS | tmp;

  NVIC_SetPriority(I2C1_EV_IRQn, 0);
  NVIC_SetPriority(I2C1_ER_IRQn, 0);

}

