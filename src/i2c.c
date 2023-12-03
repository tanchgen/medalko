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

FlagStatus i2cRegWrite( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;


  return rc;
}

FlagStatus i2cRegRead( uint8_t reg, uint8_t data ){
  FlagStatus rc = RESET;


  return rc;
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

