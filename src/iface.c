#include <stddef.h>

#include "main.h"
#include "usart_arch.h"
#include "i2c_arch.h"
#include "fpga_arch.h"

void gpioInit( void );
void adcMainInit( void );
void i2c1Init( void );
void psInit( void );
void hatInit( void );
#if JTAG_ENABLE
void fpgaInit( void );
#endif // JTAG_ENABLE
void fansInit( void );
void tmInit( void );
void xferInit( void );
void timersInit( void );
void pllInit();
//void logInit();

void gpioEnable( void );
void adcMainEnable( void );
void i2c1Enable( void );
void psEnable( void );
void hatEnable( void );
//#ifndef HATISA
void pllEnable( void );
//#endif // HATISA
void tmEnable( void );
void logEnable( void );

void timersClock( void );
void gpioClock( void );
void adcProcess( void );
void tmClock( void );
void psClock( void );
void i2c1Clock( void );
#if JTAG_ENABLE
void fpgaClock( void );
#endif // JTAG_ENABLE
void fansClock( void );
void xferTxFrameProcess( void );
void hatClock( void );
void pllClock( void );
void logClock( void );

void (*funcClock[])( void ) = {
   timersClock,
   gpioClock,
   adcProcess,
   i2c1Clock,
//   i2c2Clock,
#if JTAG_ENABLE
   fpgaClock,
#endif // JTAG_ENABLE
   psClock,
   fansClock,
   tmClock,
   xferTxFrameProcess,
   hatClock,
   pllClock,
   logClock,
};

void ifaceEnable( void ){
  gpioEnable();
  adcMainEnable();
  i2c1Enable();
  psEnable();
  hatEnable();
  tmEnable();
  logEnable();
  pllEnable();
}

void ifaceInit( void ){
  gpioInit();
  adcMainInit();
  i2c1Init();
  psInit();
#if JTAG_ENABLE
  fpgaInit();
#endif // JTAG_ENABLE
  hatInit();
  pllInit();
  tmInit();
  timersInit();
  xferInit();
  fansInit();
//  logInit();
}

void ifaceClock( void ){
  for( uint8_t i = 0; i < ARRAY_SIZE(funcClock); i++ ){
    funcClock[i]();
    if( psuAlrtOffQuery ){
      psuAlertOff();
    }
    else if( sysAlrtOffQuery ){
      sysAlertOff();
    }
    else if( fpgaAlrtOffQuery ){
      fpgaAlertOff();
    }
    if( sysFailOffQuery ){
      sysFailOff();
    }
    else if( sysOffQuery ){
      sysOff();
    }
    else if( fpgaOffQuery ){
      fpgaOff();
    }
  }
}
