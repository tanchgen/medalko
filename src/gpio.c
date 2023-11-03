#include <stddef.h>

#include "main.h"

void gpioPinSet( sGpioPin *pin ){
  pin->gpio->BSRR = pin->pin;
  pin->newstate = SET;
}

void gpioPinSetNow( sGpioPin *pin ){
  pin->gpio->BSRR = pin->pin;
  if(pin->state == Bit_RESET ){
    pin->change = SET;
  }
  pin->state = pin->newstate = Bit_SET;
}


void gpioPinReset( sGpioPin * pin ){
  pin->gpio->BRR = pin->pin;
  pin->newstate = RESET;
}

void gpioPinResetNow( sGpioPin * pin ){
  pin->gpio->BRR = pin->pin;
  if(pin->state != Bit_RESET ){
    pin->change = SET;
  }
  pin->state = pin->newstate = Bit_RESET;
}

void gpioPinResetNowTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;

  gpioPinResetNow( pin );
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


void gpioPinSetup(sGpioPin *pin) {
  GPIO_InitTypeDef  gpio_init_struct;

  /* Установим начальное состояние вывода GPIO для режимов Open-Drain и Push-Pull. */
  if( (pin->mode & 0x3) && ((pin->mode &0x8) == 0) ) {
    // GPIO pin mode - output
    gpioPinCmd( pin->gpio->ODR , pin->pin, pin->state);
  }
  else if( pin->mode == GPIO_MODE_IUD ){
    gpioPinCmd( pin->gpio->ODR , pin->pin, pin->pupd );
  }

  // MODE
  *(&(pin->gpio->CRL) + (pin->pinNum & 0x8)) = pin->mode;
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
      gpioPinCmd( pin->gpio->ODR , pin->pin, pin->state);
    }
    pin->state = pin->newstate;
    pin->change = SET;

    return true;
  }

  return false;
}

bool changeExtiPinState( sExtiPin *pin ){

  if (pin->state != pin->newstate) {
    pin->state = pin->newstate;
    pin->change = SET;
    return true;
  }
  return false;
}

void changePinStateTout( uintptr_t arg ){
  sGpioPin *pin = (sGpioPin *)arg;
  changePinState( pin );
}

