/*
 * xfer.c
 *
 *  Created on: 4 нояб. 2023 г.
 *      Author: jet
 */
#include <stdio.h>
#include <string.h>

#include "measur.h"


size_t sendTmPrep( uint8_t * buf ){
  size_t sz;

  sz = sprintf( (char*)buf, "\"alcoData\":{\"startTime\":%ld.%ld,\"stopTime\":%ld.%ld,\"measData\":{", \
                measDev.secsStart, measDev.msecStart, measDev.secsStop, measDev.msecStop );
  buf += sz;
  for( uint16_t i = 0; i < measDev.dataNum; i++ ){
    // Давление
    sz += sprintf( (char*)buf, "{\"press\":%ld,\"alco\":%ld, \"temp\":%ld},", \
                    measDev.alcoData[i].press,
                    measDev.alcoData[i].alco,
                    measDev.alcoData[i].temp );
    buf += sz;
  }
  // Удалим конечную запятую
  buf--;
  *buf++ = '}';
  *buf++ = '}';
  sz++;     // sz = sz - 1 + 2

  return sz;
}

