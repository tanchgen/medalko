#include <stddef.h>

#include "main.h"

void setModeGpioPin(sGpioPin *pin, GPIOMode_TypeDef mode, GPIOOType_TypeDef otype, GPIOPuPd_TypeDef pupd){
  GPIO_InitTypeDef  gpio_init_struct;

  GPIO_StructInit(&gpio_init_struct);

  gpio_init_struct.GPIO_Pin  = pin->pin;
  gpio_init_struct.GPIO_Mode = pin->mode = mode;
  gpio_init_struct.GPIO_OType = pin->otype = otype;
  gpio_init_struct.GPIO_PuPd = pin->pupd = pupd;

  GPIO_Init(pin->gpio, &gpio_init_struct);
}

void gpioPinSet( sGpioPin *pin ){
  GPIO_SetBits( pin->gpio, pin->pin );
  pin->newstate = SET;
}

void gpioPinSetNow( sGpioPin *pin ){
  GPIO_SetBits( pin->gpio, pin->pin );
  if(pin->state == Bit_RESET ){
    pin->change = SET;
  }
  pin->state = pin->newstate = Bit_SET;
}


void gpioPinReset( sGpioPin * pin ){
  GPIO_ResetBits( pin->gpio, pin->pin );
  pin->newstate = RESET;
}

void gpioPinResetNow( sGpioPin * pin ){
  GPIO_ResetBits( pin->gpio, pin->pin );
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


void gpioPinSetup(sGpioPin *pin)
{
  GPIO_InitTypeDef  gpio_init_struct;

  /* Установим начальное состояние вывода GPIO для режимов Open-Drain и Push-Pull. */
  if (pin->mode == GPIO_Mode_OUT) {
    GPIO_WriteBit(pin->gpio, pin->pin, pin->state);
  }

  GPIO_StructInit(&gpio_init_struct);

  gpio_init_struct.GPIO_Pin  = pin->pin;
  gpio_init_struct.GPIO_Mode = pin->mode;
  gpio_init_struct.GPIO_OType = pin->otype;
  gpio_init_struct.GPIO_PuPd = pin->pupd;

  GPIO_Init(pin->gpio, &gpio_init_struct);

}

void extiPinSetup(sExtiPin *pin ){
  GPIO_InitTypeDef sGpioInit;
  EXTI_InitTypeDef sExtiInit;
  uint8_t portNum;
  uint8_t pinNum;
  IRQn_Type irqNum;

  /* Установим начальное состояние вывода GPIO. */
  GPIO_StructInit(&sGpioInit);

  sGpioInit.GPIO_Pin  = pin->pin;
  sGpioInit.GPIO_PuPd = pin->pupd;
  GPIO_Init(pin->gpio, &sGpioInit);

  portNum = gpioPortNum( pin->gpio );
  pinNum = gpioPinNum( pin->pin );
  SYSCFG_EXTILineConfig( portNum, pinNum );

  // Установки EXTI
  sExtiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  sExtiInit.EXTI_LineCmd = ENABLE;
  sExtiInit.EXTI_Line = pin->pin;
  sExtiInit.EXTI_Trigger = pin->trigger;
  EXTI_Init( &sExtiInit );
  EXTI_ClearITPendingBit( pin->pin );

  // Установим соответствующее входу прерывание
  if( pinNum < 5 ){
    irqNum = EXTI0_IRQn + pinNum;
  }
  else if( pinNum < 10 ){
    irqNum = EXTI9_5_IRQn;
  }
  else {
    irqNum = EXTI15_10_IRQn;
  }
  NVIC_EnableIRQ( irqNum );
  NVIC_SetPriority( irqNum, KEY_IRQ_PRIORITY );
}


bool changePinState(sGpioPin *pin ){

  if (pin->state != pin->newstate) {
    if ( pin->mode == GPIO_Mode_OUT ) {
      GPIO_WriteBit(pin->gpio, pin->pin, pin->newstate);
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


void init_gpio(void)
{
  /* Конфигурация вывода GPIO MCU_RUN. */
//  gpioPinSetup(&gpio_pin_mcu_run, gpio_pin_mcu_run.state);
}

