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

int32_t i2cPress;
uint16_t pressCount;
int32_t i2cTerm;

// ----------------------------------------------------------------------
// Инициализация шины
void i2cInit( I2C_TypeDef * i2c ){
  uint8_t freq = (rccClocks.PCLK1_Frequency/1000000);
  uint32_t tmp;

  i2c->CR2 = (i2c->CR2 & ~I2C_CR2_FREQ) | freq;
  // In FAST MODE I2C
  i2c->TRISE = (freq * 300) / 1000 + 1;

  // For dutycicle = 2
  tmp = rccClocks.PCLK1_Frequency / (I2C_FS_SPEED * 3U);
  if( (tmp & I2C_CCR_CCR) == 0 ){
    tmp = 1;
  }
  i2c->CCR = (i2c->CCR & ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR)) | I2C_CCR_FS | tmp;

  i2c->CR1 |= I2C_CR1_PE;
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

  i2cInit( i2c );
}


FlagStatus i2cRegWrite( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transLen < I2C_PACK_MAX );
  trans = &(i2cTrans[transLen++]);

  trans->state = I2C_STATE_WRITE;
  trans->reg.cmd = reg;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 1;
  trans->data[0] = data;
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
  trans->len = len;
  trans->tout = 0;
  trans->parsed = true;
  // Сначала пишем
  trans->rxDataFlag = RESET;

  return rc;
}


// Обработка прерываний при записи в регистр
void i2cWriteHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  uint32_t sr1 = i2c->SR1;

  assert_param( i2c == PRESS_I2C );
  // ITEVTEN
  if( sr1 & I2C_SR1_SB ){
    // Старт-условие
    assert_param( i2cState == I2C_STATE_WRITE );
    i2c->DR = PRESS_I2C_ADDR | I2C_WRITE_BIT;
  }
  if( sr1 & I2C_SR1_ADDR ){
    // Адрес отправлен
    // Стираем флаг ADDR
    (void)i2c->SR2;
    i2c->DR = trans->reg.cmd;
    while( (i2c->SR1 & I2C_SR1_TXE) == 0 )
    {}
    i2c->DR = trans->data[0];
    i2c->CR2 &= ~I2C_CR2_ITBUFEN;

    i2c->CR1 |= I2C_CR1_STOP;
    while( i2c->SR1 & I2C_SR1_TXE )
    {}
    i2cState = I2C_STATE_END;

  }
  if( i2c->SR1 & I2C_SR1_BTF ){
    i2c->CR1 |= I2C_CR1_STOP;
  }
//  // ITBUFEN
//  if( i2c->SR1 & I2C_SR1_TXE ){
//    // При записи отправляем только один байт данных
//    assert_param( trans->txLen == 1 );
//    trans->txLen = 0;
//    i2c->DR = trans->txData;
//  }
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
//      i2c->DR = GPZ6859_ADDR | ((trans->rxDataFlag)? I2C_READ_BIT : I2C_WRITE_BIT);
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
    trans->data[trans->count++] = i2c->DR;
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
      // 2. Считываем SYS_CFG
      i2cRegWrite( regAddr[REG_P_CFG], pcfgReg.u8pcfg );
      i2cRegRead( regAddr[REG_SYS_CFG], 1 );
      // Запускаем 1-й пакет в работу
      PRESS_I2C->CR1 |= I2C_CR1_START;
      break;
      // 1. Записываем конфигурацию P_CFG,
    case PRESS_CFG_SYS_W:
      // Запись в SYS_CFG

      // В обратном порядке
      // 2. Конфиг CMD_REG и сразу запуск измерений
      i2cRegWrite( regAddr[REG_CMD], cmdReg.u8cmd );
      // 1. Сохраняем AOUT
      assert_param( sysCfgData != 0 );
      i2cRegWrite( regAddr[REG_SYS_CFG], sysCfgData );
      // Запускаем 1-й пакет в работу
      PRESS_I2C->CR1 |= I2C_CR1_START;
      break;
    case PRESS_START:
      // Запуск измерений - CMD.SCO
      assert_param( sysCfgData != 0 );
      i2cRegWrite( regAddr[REG_SYS_CFG], sysCfgData | CMD_SCO );
      // Запускаем 1-й пакет в работу
      PRESS_I2C->CR1 |= I2C_CR1_START;
      break;
    case PRESS_WAIT:
      // Читаем статус - ждем окончания измерений
      i2cRegRead( regAddr[REG_SYS_CFG], 1 );
      // Запускаем 1-й пакет в работу
      PRESS_I2C->CR1 |= I2C_CR1_START;
      break;
    case PRESS_DATA_READ:
      // Считываем результаты
      i2cRegRead( regAddr[REG_PRESS_MSB], 1 );
      i2cRegRead( regAddr[REG_PRESS_CSB], 1 );
      i2cRegRead( regAddr[REG_PRESS_LSB], 1 );
      i2cRegRead( regAddr[REG_T_MSB], 1 );
      i2cRegRead( regAddr[REG_T_LSB], 1 );
      // Запускаем 1-й пакет в работу
      PRESS_I2C->CR1 |= I2C_CR1_START;
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
        PRESS_I2C->CR1 |= I2C_CR1_START;
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
          i2cPress = (i2cTrans[0].data << 2) | (i2cTrans[1].data << 1) | i2cTrans[2].data;
  #if !PRESS_ADC
          pressProc( i2cPress, &pressCount );
  #endif // !PRESS_ADC
          i2cTerm = (i2cTrans[4].data << 1) | i2cTrans[3].data;
          pressI2cTout = mTick + 20;
          pressProgr = PRESS_START;
        }
        else if( pressProgr == PRESS_DATA_READ ){
          // Ожидание окончания измерения
          if( ((uCmdReg)(i2cTrans[transCount - 1].data[0])).sco != RESET ){
            // Данные НЕ готовы - проверяем опять
            pressProgr--;
            pressI2cTout = mTick + 5;
          }
        }
        else {
          if( pressProgr == PRESS_CFG_P ){
            // Получили конфигурацию
            sysCfgData = (i2cTrans[transCount - 1].data[0] & SYSCFG_AOUT_MASK) | syscfgReg.u8syscfg;
          }
          pressI2cTout = mTick;
        }
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
  pressProgr = PRESS_DATA_READ;

  // I2C GPIO setup
  gpioPinSetup( &pressPinScl );
  gpioPinSetup( &pressPinSda );

  // I2C periphery setup
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  PRESS_I2C->CR1 |= I2C_CR1_PE;
  i2cReset( PRESS_I2C );

  NVIC_SetPriority(I2C1_EV_IRQn, 0);
  NVIC_SetPriority(I2C1_ER_IRQn, 0);

}

