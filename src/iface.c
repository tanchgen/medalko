#include <stddef.h>

#include "main.h"

void gpioInit( void );
void adcMainInit( void );
void usbInit( void );

void gpioEnable( void );
void adcMainEnable( void );

void timersClock( void );
void gpioClock( void );
void adcProcess( void );
void usbClock( void );

void (*funcClock[])( void ) = {
   timersClock,
   gpioClock,
   adcProcess,
   usbClock
};

void ifaceEnable( void ){
  gpioEnable();
  adcMainEnable();
}

void ifaceInit( void ){
  gpioInit();
  adcMainInit();
  usbInit();
}

void ifaceClock( void ){
  for( uint8_t i = 0; i < ARRAY_SIZE(funcClock); i++ ){
    funcClock[i]();
  }
}
