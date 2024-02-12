/*
 * ltmreg_read.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTMREG_READ_H_
#define LTMREG_READ_H_

#include "i2c.h"
#include "gzp6859.h"
#include "gzp6859_reg.h"

// ==============================================================================================
static inline sI2cTrans * mpr121afe1( sI2cTrans * trans, uint8_t * pdata ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = 0x5c;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 1;
  trans->data = pdata;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

static inline sI2cTrans * mpr121afe2( sI2cTrans * trans, uint8_t * pdata ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = 0x5d;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 1;
  trans->data = pdata;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// ================================== INPUT_SEQUENCE ============================================
// BatteryMode
static inline sI2cTrans * bq30zBattMode( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = BATT_MODE;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Temperature
static inline sI2cTrans * bq30zT( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = TEMPERATURE;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Voltage
static inline sI2cTrans * bq30zV( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = VOLT;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Current
static inline sI2cTrans * bq30zI( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CURR;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// AverageCurrent
static inline sI2cTrans * bq30zAvgI( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = AVG_CURR;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// RemainingCapacity
static inline sI2cTrans * bq30zRemainCap( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = REMAIN_CAP;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// FullChargeCapacity
static inline sI2cTrans * bq30zFullChCap( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = FULL_CHARGE_CAP;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// RunTimeEmpty
static inline sI2cTrans * bq30zRunTimeEmpty( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = RUN_TIME_EMPTY;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// AverageRunTimeEmpty
static inline sI2cTrans * bq30zAvgRunTimeEmpty( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = AVG_TIME_EMPTY;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// ChargingCurrent
static inline sI2cTrans * bq30zChI( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CHARGE_CURR;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// ChargingVoltage
static inline sI2cTrans * bq30zChV( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CHARGE_VOLT;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// BatteryStatus
static inline sI2cTrans * bq30zBattStatus( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = BATT_STATUS;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// CycleCount
static inline sI2cTrans * bq30zCycleCount( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CYCLE_COUNT;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// DesignCapacity
static inline sI2cTrans * bq30zDesCap( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = DESIGN_CAP;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// DesignVoltage
static inline sI2cTrans * bq30zDesV( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = DESIGN_VOLT;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// CellVoltage_3
static inline sI2cTrans * bq30zCellV3( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CELL_VOLT_3;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// CellVoltage_2
static inline sI2cTrans * bq30zCellV2( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CELL_VOLT_2;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// CellVoltage_1
static inline sI2cTrans * bq30zCellV1( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CELL_VOLT_1;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// CellVoltage_0
static inline sI2cTrans * bq30zCellV0( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CELL_VOLT_0;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Voltage_0
static inline sI2cTrans * bq30zV0( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = VOLTAGE_FULL;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 12;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Temperature_0
static inline sI2cTrans * bq30zT0( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = TEMPERATURE_FULL;
  trans->regLen = 1;
  trans->pec = GPZ6859_PEC;
  trans->len = 14;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}


#endif /* LTMREG_READ_H_ */
