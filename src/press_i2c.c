/*
 * i2c.c
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#include "main.h"
#include "measur.h"
#include "press_i2c.h"
#include "smbus.h"

#define GPZ6859_ADDR      (0x5A << 1) //(0x6D << 1)
#define GPZ6859_PEC       PEC_DISABLE

sGpioPin pressPinScl = {GPIOB, 1, GPIO_Pin_6, 6, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin pressPinSda = {GPIOB, 1, GPIO_Pin_7, 7, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin i2cPinSclPu = {GPIOB, 1, GPIO_Pin_0, 0, GPIO_MODE_IUD, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin i2cPinSdaPu = {GPIOB, 1, GPIO_Pin_1, 1, GPIO_MODE_IUD, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };

//uint8_t  regAddr[REG_NUM] = {
//  0x06,         // REG_PRESS_MSB,
//  0x07,         // REG_PRESS_CSB,
//  0x08,         // REG_PRESS_LSB,
//  0x09,         // REG_T_MSB,
//  0x0A,         // REG_T_LSB,
//  0x30,         // REG_CMD,
//  0xA5,         // REG_SYS_CFG,
//  0xA6,         // REG_P_CFG,
//};

uint8_t  regAddr[REG_NUM] = {
  0x04,           // REG_PRESS0_LSB,
  0x05,           // REG_PRESS0_MSB,
  0x06,           // REG_PRESS1_LSB,
  0x07,           // REG_PRESS1_MSB,
  0x08,           // REG_PRESS2_LSB,
  0x09,           // REG_PRESS2_MSB,
  0x5E,           // REG_SYS_CFG,
  0x5D,           // REG_SYS_CFG2,
  0x5C,           // REG_SYS_CFG3,
};

// Значения регистров по умолчанию
const uPCfgReg pcfgReg = {
  .osr = 0x3,       //PCFG_OSR
  .gain = 0x0,      // PCFG_GAIN
  .inSwap = 0x0    // PCFG_INSWAP
};

const uSysCfgReg syscfgReg = {
  .diag = 1,          // SYSCFG_DIAG
  .outCtrl = 0,       // SYSCFG_OUT_CTRL
  .unipolar = 0,      // SYSCFG_UNIPOLAR
  .ldo = 0,           // SYSCFG_LDO
  .Aout = 0xF,          // Внутренняя конфигурация - не менять!!
};

const uCmdReg cmdReg = {
    .measCtrl = 2,      // CMD_MEAS_CTRL
    .sco = 1,           // CMD_SCO
    .sleep = 1         // CMD_SLEEP_62MS
};

struct _i2cProgrData {
  uint8_t reg;
  uint8_t data;
} i2cProgrData;

sI2cTrans i2cTrans[I2C_PACK_MAX];
volatile uint8_t transCount;
uint8_t transLen;

eI2cState i2cState;
ePressProgr pressProgr;
uint32_t pressI2cTout = 0;
uint8_t sysCfgData;

//int32_t i2cPress;
//uint16_t pressCount;
//int32_t i2cTerm;

uint16_t cap[3];

// ----------------------------------------------------------------------
void _i2cInit( I2C_TypeDef * i2c ){
  uint8_t freq = (rccClocks.PCLK1_Frequency/1000000);
  uint32_t tmp;

  i2c->CR2 = (i2c->CR2 & ~I2C_CR2_FREQ) | freq;
#if I2C_FS_ENABLE
  // In FAST MODE I2C
  i2c->CCR &= ~I2C_CCR_FS;
  i2c->TRISE = (freq * 300) / 1000 + 1;
#else
  // In STANDART MODE I2C
  i2c->CCR &= ~I2C_CCR_FS;
  i2c->TRISE = freq + 1;
#endif // I2C_FS_ENABLE

  // For dutycicle = 2
  tmp = rccClocks.PCLK1_Frequency / (I2C_SM_SPEED * 2U);
  if( (tmp & I2C_CCR_CCR) == 0 ){
    tmp = 1;
  }
  i2c->CCR = (i2c->CCR & ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR)) | tmp;

  i2c->CR1 |= I2C_CR1_PE;
}

static inline void _i2cStart( void ){
  i2cState = i2cTrans[transCount].state;
  PRESS_I2C->CR1 |= I2C_CR1_START;
}


// Сброс шины и периферии I2C
void i2cReset( I2C_TypeDef * i2c ){
  while( i2c->CR1 & (I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_PEC) )
  {}

  for( uint8_t i = 0; i < 2; i++ ){
    i2c->CR1 |= I2C_CR1_START;
    uDelay( 50 );
    if( i2c->CR1 & I2C_CR1_START){
      i2c->CR1 &= ~I2C_CR1_START;
    }
    i2c->CR1 |= I2C_CR1_STOP;
    uDelay( 50 );
    if( i2c->CR1 & I2C_CR1_STOP){
      i2c->CR1 &= ~I2C_CR1_STOP;
    }
  }
  i2c->CR1 |= I2C_CR1_SWRST;
  uDelay( 10 );
  while( ((pressPinScl.gpio->IDR & pressPinScl.pin) == RESET) \
         || ((pressPinSda.gpio->IDR & pressPinSda.pin) == RESET) )
  {}
  i2c->CR1 &= ~I2C_CR1_SWRST;

  _i2cInit( i2c );
}


FlagStatus i2cRegWrite( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transLen < I2C_PACK_MAX );
  trans = &(i2cTrans[transLen++]);

  trans->state = I2C_STATE_WRITE;
  trans->reg.cmd = reg;
  trans->regLen = 1;
  trans->regCount = 0;
  trans->pec = GPZ6859_PEC;
  trans->len = 1;
  trans->count = 0;
  trans->u8data[0] = data;
  trans->tout = 0;
  trans->parsed = true;
  trans++;

  return rc;
}

FlagStatus i2cRegRead( uint8_t reg, uint8_t len ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transLen < I2C_PACK_MAX );
  trans = &(i2cTrans[transLen++]);

  trans->state = I2C_STATE_READ;
  trans->reg.cmd = reg;
  trans->regLen = 1;
  trans->regCount = 0;
  trans->pec = GPZ6859_PEC;
  trans->len = 1;
  trans->count = 0;
  trans->len = len;
  trans->count = 0;
  trans->tout = 0;
  trans->parsed = true;
  // Сначала пишем
  trans->rxDataFlag = RESET;

  return rc;
}


// Обработка прерываний при записи в регистр
void i2cWriteHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  uint32_t sr = i2c->SR1;

  assert_param( i2c == PRESS_I2C );
  assert_param( i2cState == I2C_STATE_WRITE );

  sr = (sr | i2c->SR2 << 16)  & 0x00FFFFFF;

  // ITEVTEN
  if( (sr & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT ){
    // Старт-условие
    assert_param( i2cState == I2C_STATE_WRITE );
    i2c->DR = GPZ6859_ADDR | I2C_WRITE_BIT;
  }
  if( (sr & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ){
    // Адрес отправлен
    i2c->DR = trans->reg.cmd;
    trans->regCount++;
    if( (trans->regLen > 1) || (trans->len) ){
      i2c->CR2 |= I2C_CR2_ITBUFEN;
    }
  }
  if( (sr & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED ){
    assert_param( (trans->regCount == trans->regLen) && (trans->count == trans->len) );
    i2c->CR1 |= I2C_CR1_STOP;
    while( i2c->CR1 & I2C_CR1_STOP )
    {}
    assert_param( trans->regLen == trans->regCount );
    i2cState = I2C_STATE_END;
  }
  // ITBUFEN
  else if( (sr & I2C_EVENT_MASTER_BYTE_TRANSMITTING) == I2C_EVENT_MASTER_BYTE_TRANSMITTING ){
    if( trans->regCount < trans->regLen ){
      i2c->DR = trans->reg.u8reg[trans->regCount++];
    }
    else if( trans->count < trans->len ){
      i2c->DR = trans->u8data[trans->count++];
      if( trans->count == trans->len ){
        i2c->CR2 &= ~I2C_CR2_ITBUFEN;
      }
    }
    else {
      while(1)
      {}
    }
  }
}


// Обработка прерываний при чтении в регистр
void i2cReadHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  uint32_t sr1 = i2c->SR1;

  assert_param( i2c == PRESS_I2C );
  assert_param( i2cState == I2C_STATE_READ );

  if( sr1 & I2C_SR1_SB ){
    // Старт-условие
    if (trans->rxDataFlag){
      i2c->DR = GPZ6859_ADDR | I2C_READ_BIT;
    }
    else {
      i2c->DR = GPZ6859_ADDR;
    }
//      i2c->DR = BQ30Z55_ADDR | ((trans->rxDataFlag)? I2C_READ_BIT : I2C_WRITE_BIT);
  }
  if( sr1 & I2C_SR1_ADDR ){
    // Адрес отправлен
    // Стираем флаг ADDR;
    (void)i2c->SR2;
    if(trans->rxDataFlag){
      if( trans->len == 1){
        i2c->CR1 = (i2c->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
      }
      else {
        i2c->CR1 |= I2C_CR1_ACK;
      }
      trace_printf( "CR1:0x%x\n", i2c->CR1 );
      i2c->CR2 |= I2C_CR2_ITBUFEN;
    }
    else {
      // Пишем номер регистра
      i2c->DR = (uint8_t)(trans->reg.u8reg[trans->regCount++]);
      if( trans->regLen > 1){
        i2c->CR2 |= I2C_CR2_ITBUFEN;
      }
      else {
        i2c->CR1 |= I2C_CR1_START;
        while((i2c->SR1 & I2C_SR1_SB) == 0)
        {}
        trans->rxDataFlag = SET;
      }
    }
  }
  else if( (i2c->CR2 & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_TXE) ){
    assert_param( trans->regCount < trans->regLen );
    i2c->DR = (uint8_t)(trans->reg.u8reg[trans->regCount++]);
    if( trans->regCount == trans->regLen ){
      i2c->CR2 &= ~I2C_CR2_ITBUFEN;
      i2c->CR1 |= I2C_CR1_START;
      trans->rxDataFlag = SET;
    }
  }
  else if( (i2c->CR2 & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_RXNE) ){
    assert_param( trans->len > 0 );
    trans->u8data[trans->count++] = i2c->DR;
    if( (trans->len - trans->count) == 1 ){
      i2c->CR1 = (i2c->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
    }
    else if( trans->len == trans->count ){
      i2c->CR2 &= ~I2C_CR2_ITBUFEN;
      while( i2c->CR1 & I2C_CR1_STOP )
      {}
      i2cState = I2C_STATE_END;
    }
  }
}


// Оработка ошибки на I2C
void I2C1_ER_IRQHandler( void ){
  uint32_t sr1 = PRESS_I2C->SR1;

  if( sr1 & (I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_ARLO | I2C_SR1_AF) ){
    i2cReset( PRESS_I2C );
    i2cState = I2C_STATE_ERR;
  }
}


void I2C1_EV_IRQHandler( void ){
  if( i2cState == I2C_STATE_WRITE ){
    i2cWriteHandle( PRESS_I2C, &i2cTrans[transCount] );
  }
  else {
    assert_param( i2cState == I2C_STATE_READ );
    i2cReadHandle( PRESS_I2C, &i2cTrans[transCount] );
  }
}


// Процес информационного обмена по I2C с PRESS
void pressI2cProc( void ){
  transCount = 0;
  // Обмен по I2C производиться в обратном порядке
  switch( pressProgr ){
    case PRESS_CFG_P:
//      // 2. Считываем SYS_CFG
//      i2cRegWrite( regAddr[REG_P_CFG], pcfgReg.u8pcfg );
//      i2cRegRead( regAddr[REG_SYS_CFG], 1 );
//      // Запускаем 1-й пакет в работу
//      _i2cStart();
      break;
      // 1. Записываем конфигурацию P_CFG,
    case PRESS_CFG_SYS_W:
//      // Запись в SYS_CFG
//
//      // В обратном порядке
//      // 2. Конфиг CMD_REG и сразу запуск измерений
//      i2cRegWrite( regAddr[REG_CMD], cmdReg.u8cmd );
//      // 1. Сохраняем AOUT
//      assert_param( sysCfgData != 0 );
//      i2cRegWrite( regAddr[REG_SYS_CFG], sysCfgData );
//      // Запускаем 1-й пакет в работу
//      _i2cStart();
      break;
    case PRESS_START:
      // Запуск измерений - CMD.SCO
//      assert_param( sysCfgData != 0 );
//      i2cRegWrite( regAddr[REG_SYS_CFG], sysCfgData | CMD_SCO );
      i2cRegRead( regAddr[REG_SYS_CFG2], 1 );
      i2cRegWrite( regAddr[REG_SYS_CFG], 0x3c);
      i2cRegWrite( regAddr[REG_SYS_CFG2], 0x51);
      i2cRegWrite( regAddr[REG_SYS_CFG3], 0x2B);
      // Запускаем 1-й пакет в работу
      _i2cStart();
      break;
    case PRESS_WAIT:
      // Читаем статус - ждем окончания измерений
      i2cRegRead( regAddr[REG_SYS_CFG], 1 );
      i2cRegRead( regAddr[REG_SYS_CFG2], 1 );
      i2cRegRead( regAddr[REG_SYS_CFG3], 1 );
      // Запускаем 1-й пакет в работу
      _i2cStart();
      break;
    case PRESS_DATA_READ:
      // Считываем результаты
      i2cRegRead( regAddr[REG_PRESS0_LSB], 6 );
//      i2cRegRead( regAddr[REG_PRESS0_MSB], 1 );
//      i2cRegRead( regAddr[REG_PRESS1_LSB], 2 );
//      i2cRegRead( regAddr[REG_PRESS1_MSB], 1 );
//      i2cRegRead( regAddr[REG_PRESS2_LSB], 2 );
//      i2cRegRead( regAddr[REG_PRESS2_MSB], 1 );
      // Запускаем 1-й пакет в работу
      _i2cStart();
      break;
    default:
      break;
  }
}


void pressI2cClock( void ){

  switch(i2cState){
    case I2C_STATE_IDLE:
      if( pressI2cTout < mTick ){
        pressI2cProc();
      }
      break;
    case I2C_STATE_END:       // Нормальное окончание программы
      if( ++transCount < transLen ){
        // Запускаем 1-й пакет в работу
        _i2cStart();
      }
      else {
        // Обработка и сохранение полученных данных
        assert_param( transCount == transLen );
        transCount = 0;
        transLen = 0;

        // Больше передавать нечего - переходим к следующему пункту программы обмена
        if( ++pressProgr == PRESS_PROGR_NUM ){
          // Дошли до конца списка
          //Сохраняем значения
//          i2cPress = (i2cTrans[0].data << 2) | (i2cTrans[1].data << 1) | i2cTrans[2].data;
//#if !PRESS_ADC
//          pressProc( i2cPress, &pressCount );
//#endif // !PRESS_ADC
//          i2cTerm = (i2cTrans[4].data << 1) | i2cTrans[3].data;
          cap[0] = i2cTrans[0].u16data[0];
          cap[1] = i2cTrans[0].u16data[1];
          cap[2] = i2cTrans[0].u16data[2];
          pressI2cTout = mTick + 20;
          pressProgr = PRESS_DATA_READ;
        }
//        else if( pressProgr == PRESS_DATA_READ ){
//          // Ожидание окончания измерения
//          if( ((uCmdReg)(i2cTrans[transCount - 1].data[0])).sco != RESET ){
//            // Данные НЕ готовы - проверяем опять
//            pressProgr--;
//            pressI2cTout = mTick + 5;
//          }
//        }
//        else {
//          if( pressProgr == PRESS_CFG_P ){
//            // Получили конфигурацию
//            sysCfgData = (i2cTrans[transCount - 1].data[0] & SYSCFG_AOUT_MASK) | syscfgReg.u8syscfg;
//          }
//          pressI2cTout = mTick;
//        }
        i2cState = I2C_STATE_IDLE;
      }

      break;
    case I2C_STATE_ERR:
      pressProgr--;
      pressI2cTout = mTick + 20;    // 20мс для восстановления шины
      i2cState = I2C_STATE_IDLE;
      break;
    case I2C_STATE_WRITE:
    case I2C_STATE_READ:
    default:
      break;
  }
}

void pressI2cEnable( void ){
  PRESS_I2C->CR1 |= I2C_CR1_PE;
  PRESS_I2C->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}


void pressI2cInit( void ){
  // Data setup
  i2cState = I2C_STATE_IDLE;
  transCount = 0;
  transLen = 0;
  pressProgr = PRESS_START;

  // I2C GPIO setup
  gpioPinSetup( &pressPinScl );
  gpioPinSetup( &pressPinSda );
  gpioPinSetup( &i2cPinSclPu );
  gpioPinSetup( &i2cPinSdaPu );

  // I2C periphery setup
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  PRESS_I2C->CR1 |= I2C_CR1_PE;
  i2cReset( PRESS_I2C );

  NVIC_SetPriority(I2C1_EV_IRQn, 0);
  NVIC_SetPriority(I2C1_ER_IRQn, 0);

}

