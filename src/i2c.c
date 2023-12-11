/*
 * i2c.c
 *
 *  Created on: 3 дек. 2023 г.
 *      Author: jet
 */

#include "main.h"
#include "i2c.h"

sGpioPin alcoPinScl = {GPIOB, 1, GPIO_Pin_6, 6, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };
sGpioPin alcoPinSda = {GPIOB, 1, GPIO_Pin_7, 7, GPIO_MODE_AFOD_10, GPIO_PULLUP, Bit_SET, Bit_SET, RESET };

sI2cTrans i2cTrans[TRANS_NUM];

FlagStatus i2cRegWrite( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;

  // ..........
  // Запускаем
  PRESS_I2C->CR1 |= I2C_CR1_START;
  return rc;
}

FlagStatus i2cRegRead( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;


  return rc;
}


void i2cWriteHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  // 1. Отправляем адрес
  // 2. Отправляем вдрес регистра
  //   Сост. запись
  // 3. Пока есть еще что отправлять - отправляем
  // 4. Есть чт
  // 4. Если нечего читать - останавливаемся ( IDLE )
  // 5. Рестарт
  // 6. -> п. 1
  //

  asser_param( i2c = PRESS_I2C );
  if( i2c->SR1 & I2C_SR1_SB ){
    // Старт-условие
    assert_param( trans->state == I2C_STATE_WRITE );
    i2c->DR = PRESS_I2C_ADDR | I2C_WRITE_BIT;
  }
  if( i2c->SR1 & I2C_SR1_ADDR ){
    // Адрес отправлен
    i2c->DR = trans->regTx;
    i2c->DR = trans->txData;
  }
  if( i2c->SR1 & I2C_SR1_TXE ){
    // При записи отправляем только один байт данных
    assert_param( trans->txLen == 1 );
    trans->txLen = 0;
    i2c->DR = trans->txData;
    i2c->CR2 &= ~I2C_CR2_ITBUFEN;
  }
  if( i2c->SR1 & I2C_SR1_BTF ){
    i2c->CR1 |= I2C_CR1_STOP;
  }


    else {
      assert_pararm( trans->state == I2C_STATE_READ );
      // При чтении отправляем только один байт - арес регистра
      assert_param( trans->txLen == 0 );
      assert_param( (i2c->CR2 & I2C_CR2_ITBUFEN) == RESET );
      i2c->DR = trans->regRx;
    }
  }
  if( i2c->SR1 & I2C_SR1_TXE ){
    // При записи отправляем только один байт данных
    assert_param( trans->txLen == 1 );
    trans->txLen = 0;
    i2c->DR = trans->txData;
    i2c->CR2 &= ~I2C_CR2_ITBUFEN;
  }


}


void i2cReadHandle( I2C_TypeDef * i2c, sI2cTrans * trans ){
  // 1. Отправляем адрес
  // 2. Отправляем вдрес регистра
  //   Сост. запись
  // 3. Пока есть еще что отправлять - отправляем
  // 4. Есть чт
  // 4. Если нечего читать - останавливаемся ( IDLE )
  // 5. Рестарт
  // 6. -> п. 1
  //

  asser_param( i2c = PRESS_I2C );
  if( i2c->SR1 & I2C_SR1_SB ){
    // Старт-условие
    if( trans->state == I2C_STATE_WRITE ){
      i2c->DR = trans->regTx;
    }
    else {
      assert_pararm( trans->state == I2C_STATE_READ );
      i2c->DR = trans->regRx;
    }
  }
  if( i2c->SR1 & I2C_SR1_ADDR ){
    // Адрес отправлен
    if( trans->state == I2C_STATE_WRITE ){
      i2c->DR = trans->txData;
      i2c->CR1 |= I2C_CR1_STOP;
    }
    else {
      assert_pararm( trans->state == I2C_STATE_READ );
      // При чтении отправляем только один байт - арес регистра
      assert_param( trans->txLen == 0 );
      assert_param( (i2c->CR2 & I2C_CR2_ITBUFEN) == RESET );
      i2c->DR = trans->regRx;
    }
  }
  if( i2c->SR1 & I2C_SR1_TXE ){
    // При записи отправляем только один байт данных
    assert_param( trans->txLen == 1 );
    trans->txLen = 0;
    i2c->DR = trans->txData;
    i2c->CR2 &= ~I2C_CR2_ITBUFEN;
  }


}

void i2c1IrqHandle( void ){
  if( i2cTrans.state == I2C_STATE_WRITE ){
    i2cWriteHandle( PRESS_I2C, &i2cTrans );
  }
  else {
    assert_param( i2cTrans.state == I2C_STATE_READ );
    i2cReadHandle( PRESS_I2C, &i2cTrans );
  }
}


void i2cEnable( void ){
  ALCO_I2C->CR1 |= I2C_CR1_PE;
  ALCO_I2C->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}


void i2cInit( void ){
  uint8_t freq = (rccClocks.PCLK1_Frequency/1000);
  uint32_t tmp;

  gpioPinSetup( &alcoPinScl );
  gpioPinSetup( &alcoPinSda );

  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  ALCO_I2C->CR2 = (ALCO_I2C->CR2 & ~I2C_CR2_FREQ) | freq;
  // In FAST MODE I2C
  ALCO_I2C->TRISE = (freq * 300) / 1000 + 1;

  tmp = rccClocks.PCLK1_Frequency / (ALCO_I2C_SPEED * 3U);
  // For dutycicle = 2
  if( tmp & I2C_CCR_CCR == 0 ){
    tmp = 1;
  }
  ALCO_I2C->CCR = (ALCO_I2C->CCR & ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR)) | I2C_CCR_FS | tmp;

  NVIC_SetPriority(I2C1_EV_IRQn, 0);
  NVIC_SetPriority(I2C1_ER_IRQn, 0);
}

