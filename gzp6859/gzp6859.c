/*
 * gzp6859.c
 *
 *  Created on: 25 дек. 2023 г.
 *      Author: jet
 */

#include "main.h"
#include "gzp6859.h"


const uint8_t bq30zIdKey[16] = {
    0x66, 0x99, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

sBq30z55Dev gzp6859Dev;

// --------------------- ACCESS FUNCTION ----------------------------------
void gpz6859Clock( void ){
  if( i2cState == I2C_STATE_END ){
    // Окончание опроса датчика

  }

  if( ++transCount == transLen ){
    // Обработка и сохранение полученных данных
    transCount = 0;
  }
  i2cState = I2C_STATE_IDLE;

}


void gzp6859Init( void ){
  gzp6859Dev.cfgSequence = mpr121CfgSeq; // bq30zCfgSeq;
  gzp6859Dev.inSequence = mpr121InSeq; //bq30zInSeq;
  gzp6859Dev.cfgFlag = RESET;
}

