/*
 * measur.c
 *
 *  Created on: 3 нояб. 2023 г.
 *      Author: jet
 */

#include "measur.h"

eMeasState measState = MEASST_OFF;
eMeasState measOnNeed = RESET;
FlagStatus measRun = RESET;

const float measPressLimMin = 15.0;
const float measPressLimMax = 34.0;
const float measAlkoLimMin = 15.0;

sMeasur measDev;


void measInit( void ){

}
