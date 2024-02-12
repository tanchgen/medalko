/*
 * i2c.c
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#include "i2c.h"
#include "main.h"
#include "gzp6859.h"

#ifndef FLAG_Mask
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)
#endif // FLAG_Mask

sGpioPin i2cPinScl = {GPIOB, 1, GPIO_Pin_6, 6, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin i2cPinSda = {GPIOB, 1, GPIO_Pin_7, 7, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin i2cPinSclPu = {GPIOB, 1, GPIO_Pin_0, 0, GPIO_MODE_IUD, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin i2cPinSdaPu = {GPIOB, 1, GPIO_Pin_1, 1, GPIO_MODE_IUD, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };


struct _i2cData {
  uint8_t reg[4];
  uint8_t * txData;
  uint16_t txCount;
  uint8_t * rxData;
  uint16_t rxCount;
} i2cData;

sI2cTrans i2cTrans[I2C_PACK_MAX];
uint8_t transLen;
uint8_t transCount;
eI2cState i2cState;
ePressProgr pressProgr;
uint32_t pressI2cTout = 0;

struct timer_list i2cBusyTimer;
int32_t i2cPress;
uint16_t pressCount;
int32_t i2cTerm;

// --------------------- FUNCTION PROTOTYPE -----------------------------
FlagStatus i2cRegWrite( void );
FlagStatus i2cRegRead( void );
// ----------------------------------------------------------------------
// Инициализация шины
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
  while( ((i2cPinScl.gpio->IDR & i2cPinScl.pin) == RESET) \
         || ((i2cPinSda.gpio->IDR & i2cPinSda.pin) == RESET) )
  {}
  i2c->CR1 &= ~I2C_CR1_SWRST;

  _i2cInit( i2c );
}


/**
  * @brief  Проверка занятости цепочки устройств I2C.
  *
  * @param[in]  chain дескриптор цепочки устройств I2C
  *
  * @retval true/false
  */
FlagStatus i2cIsBusy( void ){
  if ( (I2Cx->SR2 & I2C_SR2_BUSY) == RESET) {
    // I2C не занято
    timerDel(&i2cBusyTimer);
    return false;
  }

  /* Запускаем таймер ожидания готовности шины I2C. */
  if (!timerPending(&i2cBusyTimer)){
    timerMod(&i2cBusyTimer, I2C_BUSY_TOUT);
  }

  return true;
}


/**
  * @brief  Формирование последовательности регистров обработки данных устройства I2C.
  *
  * @param[in]  chain дескриптор цепочки устройств I2C
  *
  * @retval none
  */
size_t getI2cSequence( void ){
  size_t len;
  sI2cTrans * r0;
//  eI2cRst rst;

  assert_param( transCount == 0 );

  r0 = i2cTrans;
  // Копируем регистры сброса данных.
  if ( gzp6859Dev.cfgFlag == false ) {
    r0 = gzp6859Dev.cfgSequence( r0 );
  }

  /* Копируем регистры чтения данных. */
  r0 = gzp6859Dev.inSequence( r0 );

  len = r0 - i2cTrans;

  transLen = len;

  assert_param( len <= ARRAY_SIZE( i2cTrans ) );
  len *= sizeof( *i2cTrans );

  assert_param( ARRAY_SIZE( i2cTrans ) >= transLen );

  return len;
}


/**
  * @brief  Генерация состояния START.
  *
  * @param[in]  chain дескриптор цепочки устройств I2C
  * @param[in]  device  дескриптор устройства I2C
  * @param[in]  reg   данные регистра для приема или передачи
  *
  * @retval none
  */
void i2cReadWriteStart( void ){

  /* Выставим начальное состояние транзакции обработки данных. */
  i2cState = i2cTrans[transCount].state;

  if( i2cState == I2C_STATE_WRITE ){
    i2cRegWrite();
  }
  else {
    assert_param( i2cState == I2C_STATE_READ );
    i2cRegRead();
  }
}


FlagStatus i2cRegWrite( void ){
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transCount < I2C_PACK_MAX );
  trans = &(i2cTrans[transCount]);

  trans->regCount = 0;
  trans->count = 0;
  // Только пишем
  trans->rxDataFlag = RESET;
  // Запускаем 1-й пакет в работу
  I2Cx->CR1 |= I2C_CR1_START;

  return rc;
}


FlagStatus i2cRegRead( void ) {
  FlagStatus rc = RESET;
  sI2cTrans * trans;

  assert_param( transCount < I2C_PACK_MAX );
  trans = &(i2cTrans[transCount]);

  trans->regCount = 0;
  trans->count = 0;
  // Сначала пишем
  trans->rxDataFlag = RESET;
  // Запускаем 1-й пакет в работу
  I2Cx->CR1 |= I2C_CR1_START;

  return rc;
}


// Обработка прерываний при записи в регистр
void i2cWriteHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  uint32_t sr = i2c->SR1;

  assert_param( i2c == I2Cx );
  assert_param( i2cState == I2C_STATE_WRITE );

  sr = (sr | i2c->SR2 << 16)  & FLAG_Mask;

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
    while( i2c->SR1 & I2C_SR1_TXE )
    {}
    assert_param( trans->regLen == trans->regCount );
    i2cState = I2C_STATE_END;
  }
  // ITBUFEN
  else if( (sr & I2C_EVENT_MASTER_BYTE_TRANSMITTING) == I2C_EVENT_MASTER_BYTE_TRANSMITTING ){
    if( trans->regCount < trans->regLen ){
      i2c->DR = *(trans->reg.u8reg + trans->regCount++);
    }
    else if( trans->count < trans->len ){
      i2c->DR = *(trans->data + trans->count++);
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

  assert_param( i2c == I2Cx );
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
  uint32_t sr1 = I2Cx->SR1;

  if( sr1 & (I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_ARLO | I2C_SR1_AF) ){
    i2cReset( I2Cx );
    i2cState = I2C_STATE_ERR;
  }
}


void I2C1_EV_IRQHandler( void ){
  if( i2cState == I2C_STATE_WRITE ){
    i2cWriteHandle( I2Cx, &i2cTrans[transCount] );
  }
  else {
    assert_param( i2cState == I2C_STATE_READ );
    i2cReadHandle( I2Cx, &i2cTrans[transCount] );
  }
}


void i2cClock( void ){
  uint8_t seqlen;

  switch( i2cState ) {
    case I2C_STATE_IDLE:
      if (!i2cIsBusy()) {
        if( transCount ){
          i2cReadWriteStart();
        }
        else if( (seqlen = getI2cSequence()) ){
          assert_param(transCount == 0);
          i2cReadWriteStart();
        }
      }
      break;
    case I2C_STATE_END:
      if( ++transCount == transLen ){
        // Обработка и сохранение полученных данных
        transCount = 0;
      }
      break;

    case I2C_STATE_ERR:
      i2cReset( I2Cx );
      break;

    default:
      break;
  }
}


void i2cEnable( void ){
  I2Cx->CR1 |= I2C_CR1_PE;
  I2Cx->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}


void i2cInit( void ){
  // Data setup
  i2cState = I2C_STATE_IDLE;
  transCount = 0;

  // I2C GPIO setup
  gpioPinSetup( &i2cPinScl );
  gpioPinSetup( &i2cPinSda );
  gpioPinSetup( &i2cPinSclPu );
  gpioPinSetup( &i2cPinSdaPu );

  // I2C periphery setup
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  I2Cx->CR1 |= I2C_CR1_PE;
  i2cReset( I2Cx );

  NVIC_SetPriority(I2C1_EV_IRQn, 0);
  NVIC_SetPriority(I2C1_ER_IRQn, 0);

}

