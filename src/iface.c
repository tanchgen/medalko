#include <stddef.h>

#include "main.h"

void gpioInit( void );
void timersInit( void );
void adcMainInit( void );
void usbInit( void );
void measInit( void );
//void pressI2cInit( void );

void gpioEnable( void );
void adcMainEnable( void );
//void pressI2cEnable( void );

void timersClock( void );
void gpioClock( void );
void adcProcess( void );
void usbClock();
void measClock( void );
//void pressI2cClock( void );

void (*funcClock[])( void ) = {
   timersClock,
   gpioClock,
//   pressI2cClock,
   adcProcess,
   measClock,
   usbClock,
};

void ifaceEnable( void ){
  gpioEnable();
  adcMainEnable();
//  pressI2cEnable();
}

void ifaceInit( void ){
  timersInit();
  measInit();
  gpioInit();
//  pressI2cInit();
  adcMainInit();
  usbInit();
}

void ifaceClock( void ){
  for( uint8_t i = 0; i < ARRAY_SIZE(funcClock); i++ ){
    funcClock[i]();
  }
}
