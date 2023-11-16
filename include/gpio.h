#ifndef _GPIO_H
#define _GPIO_H

#include <stdbool.h>

#ifdef STM32F4XX
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#endif // STM32F4XX

#include "times.h"
//#include "gpio_arch.h"

typedef enum {
  GPIO_MODE_AIN = 0b0000,
  GPIO_MODE_IFL = 0b0100,
  GPIO_MODE_IUD = 0b1000,
  GPIO_MODE_OPP_10 = 0b0001,
  GPIO_MODE_OPP_2 = 0b0010,
  GPIO_MODE_OPP_50 = 0b0011,
  GPIO_MODE_OOD_10 = 0b0101,
  GPIO_MODE_OOD_2 = 0b0110,
  GPIO_MODE_OOD_50 = 0b0111,
  GPIO_MODE_AFPP_10 = 0b1001,
  GPIO_MODE_AFPP_2 = 0b1010,
  GPIO_MODE_AFPP_50 = 0b1011,
  GPIO_MODE_AFOD_10 = 0b1101,
  GPIO_MODE_AFOD_2 = 0b1110,
  GPIO_MODE_AFOD_50 = 0b1111,
} eGpioMode;

typedef enum {
  GPIO_PULLDOWN = 0,
  GPIO_PULLUP = 1,
  GPIO_NOPULL = 2,
} eGpioPuPd;

//typedef enum
//{
//  EXTI_Trigger_Rising = 0x01,
//  EXTI_Trigger_Falling = 0x02,
//  EXTI_Trigger_Rising_Falling = 0x3
//}eExtiTrigger;

/** Структура дескриптора вывода GPIO. */
typedef struct {
  GPIO_TypeDef  *gpio;      /** Дескриптор GPIO. */
  uint8_t gpioNum;
  uint16_t  pin;            /** Номер вывода GPIO. */
  uint8_t pinNum;
  eGpioMode  mode;          /** Режим работы вывода GPIO. */
  eGpioPuPd pupd;
  BitAction  state, newstate;     /** Текущее и запрашиваемое состояние вывода GPIO. */
  FlagStatus change;              // Флаг изменения состояниявывода
} sGpioPin;

///** Структура дескриптора входа EXTI. */
//typedef struct {
//  GPIO_TypeDef  *gpio;      /** Дескриптор GPIO. */
//  uint8_t gpioNum;
//  uint16_t  pin;            /** Номер вывода GPIO. */
//  uint8_t pinNum;
//  eGpioPuPd pupd;
//  eExtiTrigger trigger;       /** Признак срабатывания прерывания по фронту */
//  BitAction  state, newstate;     /** Текущее и запрашиваемое состояние вывода GPIO. */
//  FlagStatus change;              // Флаг изменения состояниявывода
//
//} sExtiPin;

///**
//  * @brief  Установка режима работы вывода GPIO.
//  *
//  * @param[in]  pin дескриптор вывода GPIO
//  * @param[in]  mode  режим работы вывода GPIO - in/out
//  * @param[in]  otype режим работы вывода GPIO - OD/PP
//  * @param[in]  pupd  режим работы вывода GPIO - pullup/pulldown
//  *
//  * @retval none
//  */
//void setModeGpioPin(sGpioPin *pin, GPIOMode_TypeDef mode, GPIOOType_TypeDef otype, GPIOPuPd_TypeDef pupd);

void gpioPinSet( sGpioPin * pin );
void gpioPinReset( sGpioPin * pin );

void gpioPinCmd( sGpioPin *pin, FlagStatus act );
void gpioPinCmdNow( sGpioPin * pin, FlagStatus cmd );

//BitAction  gpioPinRead( sGpioPin * pin );
//BitAction  gpioPinReadNow( sGpioPin * pin );
//
//BitAction  extiPinRead( sExtiPin * pin );
//BitAction  extiPinReadNow( sExtiPin * pin );

void gpioInit( void);
void gpioClock( void);

/**
  * @brief  Безотлогательная установка вывода GPIO, вызываемая из таймера.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinResetNowTout( uintptr_t arg );

/**
  * @brief  Инициализация вывода GPIO.
  *
  * @param[in]  pin дескриптор вывода GPIO
  * @param[in]  mode  режим работы вывода GPIO
  *
  * @retval none
  */
void gpioPinSetup(sGpioPin *pin);

///**
//  * @brief  Инициализация входа EXTI.
//  *
//  * @param[in]  pin дескриптор вывода GPIO
//  *
//  * @retval none
//  */
//void extiPinSetup(sExtiPin *pin);

bool changePinState(sGpioPin *pin );

/**
  * @brief  Проверка и изменение состояния вывода GPIO по таймауту таймера.
  *
  * @param[in]  agr  значение указателя на дескриптор вывода GPIO
  *
  * @retval none
  */
void changePinStateTout( uintptr_t arg );

/**
  * @brief  Проверка и изменение состояния вывода GPIO.
  *
  * @param[in]  pin   дескриптор вывода GPIO
  *
  * @retval true    состояние изменилось
  * @retval false   состояние не изменилось
  */
//bool changePinState( sGpioPin *pin );

///**
//  * @brief  Проверка и изменение состояния входа EXTI.
//  *
//  * @param[in]  pin   дескриптор входа EXTI
//  *
//  * @retval true    состояние изменилось
//  * @retval false   состояние не изменилось
//  */
//bool changeExtiPinState( sExtiPin *pin );

/**
  * @brief  Инициализация подсистемы GPIO.
  *
  * @param  none
  *
  * @retval none
  */
void init_gpio(void);

/**
  * @brief  Обработчик таймаута положительного импульса
  *
  * @param[in]  arg  указатель на структуру GPIO-вывода
  *
  * @retval none
  */
void gpioPosPulseTout(uintptr_t arg);

/**
  * @brief  Обработчик таймаута отрицательного импульса
  *
  * @param[in]  arg  указатель на структуру GPIO-вывода
  *
  * @retval none
  */
void gpioNegPulseTout(uintptr_t arg);

void togglePinStateTout( uintptr_t arg );

static inline FlagStatus gpioPinState( sGpioPin *pin ){
  return (pin->gpio->IDR & pin->pin)? SET: RESET;
}

static inline uint8_t gpioPortNum( GPIO_TypeDef *port){
  assert_param(IS_GPIO_ALL_PERIPH(port));
  uint8_t pn;

  pn = ((uint32_t)port - GPIOA_BASE)/(GPIOB_BASE - GPIOA_BASE);

  return pn;
}

static inline uint8_t gpioPinNum( uint32_t pin ){
  uint8_t i;

  assert_param(IS_GET_GPIO_PIN(pin));

  for( i = 0; (pin & 0x1) == 0; i++ ){
    pin >>= 1;
  }
  return i;
}

static inline BitAction  gpioPinRead( sGpioPin * pin ){
  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;

  pin->newstate = bit;
  return bit;
}

static inline BitAction  gpioPinReadNow( sGpioPin * pin ){
  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;
  if( bit != pin->newstate ){
    pin->state = pin->newstate = bit;
    pin->change = SET;
  }
  else {
    pin->state = bit;
  }
  return bit;
}

/**
  * @brief  Безотлогательная установка вывода GPIO.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinSetNow( sGpioPin *pin );

/**
  * @brief  Безотлогательный сброс вывода GPIO.
  *
  * @param[in]  *pin указатель на дескриптор вывода GPIO
  *
  * @retval none
  */
void gpioPinResetNow( sGpioPin * pin );

//static inline BitAction  extiPinRead( sExtiPin * pin ){
//  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;
//  if( bit != pin->newstate ){
//    pin->newstate = bit;
//    pin->change = SET;
//  }
//  return bit;
//}
//
//static inline BitAction  extiPinReadNow( sExtiPin * pin ){
//  BitAction bit = (pin->gpio->IDR & pin->pin)? Bit_SET : Bit_RESET;
//  if( bit != pin->newstate ){
//    pin->state = pin->newstate = bit;
//    pin->change = SET;
//  }
//  else {
//    pin->state = bit;
//  }
//  return bit;
//}

#endif /* _GPIO_H */

