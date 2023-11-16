#include <stddef.h>

#include "main.h"

void gpioPinResetNowTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;

  gpioPinResetNow( pin );
}


void gpioPinSetup(sGpioPin *pin) {
  assert_param( pin->gpioNum <= 3);

  if( pin->gpio == GPIOA ){
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  }
  else if( pin->gpio == GPIOB ){
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  }
  else if( pin->gpio == GPIOC ){
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  }
  else if( pin->gpio == GPIOD ){
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
  }

  /* Установим начальное состояние вывода GPIO для режимов Open-Drain и Push-Pull. */
  if( (pin->mode & 0x3) && ((pin->mode &0x8) == 0) ) {
    // GPIO pin mode - output
    gpioPinCmd( pin, pin->state == Bit_SET );
  }
  else if( pin->mode == GPIO_MODE_IUD ){
    gpioPinCmd( pin, (FlagStatus)pin->pupd );
  }

  // MODE
  if( pin->pinNum < 8){
    pin->gpio->CRL = (pin->gpio->CRL & ~(0xF << (pin->pinNum*4))) | pin->mode  << (pin->pinNum*4);
  }
  else {
    pin->gpio->CRH = (pin->gpio->CRH & ~(0xF  << ((pin->pinNum & 0x7)*4))) | pin->mode  << ((pin->pinNum & 0x7)*4);
  }
}

//void extiPinSetup(sExtiPin *pin ){
//  GPIO_InitTypeDef sGpioInit;
//  EXTI_InitTypeDef sExtiInit;
//  uint8_t portNum;
//  uint8_t pinNum;
//  IRQn_Type irqNum;
//
//  /* Установим начальное состояние вывода GPIO. */
//  GPIO_StructInit(&sGpioInit);
//
//  sGpioInit.GPIO_Pin  = pin->pin;
//  sGpioInit.GPIO_PuPd = pin->pupd;
//  GPIO_Init(pin->gpio, &sGpioInit);
//
//  portNum = gpioPortNum( pin->gpio );
//  pinNum = gpioPinNum( pin->pin );
//  SYSCFG_EXTILineConfig( portNum, pinNum );
//
//  // Установки EXTI
//  sExtiInit.EXTI_Mode = EXTI_Mode_Interrupt;
//  sExtiInit.EXTI_LineCmd = ENABLE;
//  sExtiInit.EXTI_Line = pin->pin;
//  sExtiInit.EXTI_Trigger = pin->trigger;
//  EXTI_Init( &sExtiInit );
//  EXTI_ClearITPendingBit( pin->pin );
//
//  // Установим соответствующее входу прерывание
//  if( pinNum < 5 ){
//    irqNum = EXTI0_IRQn + pinNum;
//  }
//  else if( pinNum < 10 ){
//    irqNum = EXTI9_5_IRQn;
//  }
//  else {
//    irqNum = EXTI15_10_IRQn;
//  }
//  NVIC_EnableIRQ( irqNum );
//  NVIC_SetPriority( irqNum, KEY_IRQ_PRIORITY );
//}


bool changePinState(sGpioPin *pin ){

  if (pin->state != pin->newstate) {
    if( (pin->mode & 0x3) && ((pin->mode &0x8) == 0) ) {
      // GPIO pin mode - output
      gpioPinCmd( pin, (FlagStatus)pin->state);
    }
    pin->state = pin->newstate;
    pin->change = SET;

    return true;
  }

  return false;
}

//bool changeExtiPinState( sExtiPin *pin ){
//
//  if (pin->state != pin->newstate) {
//    pin->state = pin->newstate;
//    pin->change = SET;
//    return true;
//  }
//  return false;
//}

void changePinStateTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;
  changePinState( pin );
}

void gpioPinCmd( sGpioPin *pin, FlagStatus act ){
  if( act == RESET ){
    gpioPinReset( pin );
  }
  else {
    gpioPinSet( pin );
  }
}

void gpioPinCmdNow( sGpioPin *pin, FlagStatus act ){
  if( act == RESET ){
    gpioPinResetNow( pin );
  }
  else {
    gpioPinSetNow( pin );
  }
}


void gpioPinSet( sGpioPin *pin ){
  assert_param( (pin->mode & 0x3) && ((pin->mode & 0x8) == 0) );
  pin->gpio->BSRR = pin->pin;
  pin->newstate = Bit_SET;
}

void gpioPinSetNow( sGpioPin *pin ){
  assert_param( (pin->mode & 0x3) && ((pin->mode & 0x8) == 0) );
  pin->gpio->BSRR = pin->pin;
  if(pin->state == Bit_RESET ){
    pin->change = SET;
  }
  pin->state = pin->newstate = Bit_SET;
}


void gpioPinReset( sGpioPin * pin ){
  assert_param( (pin->mode & 0x3) && ((pin->mode & 0x8) == 0) );
  pin->gpio->BRR = pin->pin;
  pin->newstate = Bit_RESET;
}

void gpioPinResetNow( sGpioPin * pin ){
  assert_param( (pin->mode & 0x3) && ((pin->mode & 0x8) == 0) );
  pin->gpio->BRR = pin->pin;
  if(pin->state != Bit_RESET ){
    pin->change = SET;
  }
  pin->state = pin->newstate = Bit_RESET;
}
