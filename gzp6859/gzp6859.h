/*
 * bq30z50.h
 *
 *  Created on: 25 дек. 2023 г.
 *      Author: jet
 */

#ifndef GPZ6859_H_
#define GPZ6859_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f10x.h"

#include "gzp6859_reg.h"
#include "gzp6859_smbus.h"

#define GPZ6859_CHIP_TYPE   0x010550
#define MEAS_NUM_MAX        1024


typedef struct {
   uint16_t inChargeCtrl: 1;
   uint16_t primBattSupp: 1;
   uint16_t reserv2: 1;
   uint16_t reserv3: 1;
   uint16_t reserv4: 1;
   uint16_t reserv5: 1;
   uint16_t reserv6: 1;
   uint16_t condFlag: 1;          // Condition Flag
   uint16_t chargeCtrlEn: 1;      // Battery pack's internal charge controller state
   uint16_t primBatt: 1;          // Primary battery
   uint16_t reserv10: 1;
   uint16_t reserv11: 1;
   uint16_t reserv12: 1;
   uint16_t alrmMode: 1;
   uint16_t capMode: 1;           // Capacity mode: 0 - Report in mAh", 1 - "Report in 10mWh
 } sBattStatusFlag;

 typedef enum {
   BATT_ERR_OK,
   BATT_ERR_BUSY,
   BATT_ERR_RESERV_CMD,
   BATT_ERR_UNSUPP_CMD,
   BATT_ERR_ACCESS,
   BATT_ERR_OF_UF,            // Over/Underflow
   BATT_ERR_SIZE,             // Bad Size
   BATT_ERR_UNKNOWN           // Unknown Error
 } eBattErr;

 typedef enum {
   BATT_INIT_CORRECT,
   BATT_INIT_RECALIBR
 } eBattInited;

 // SBS_BATTERY_STATUS_INFO
 typedef struct {
   eBattErr errCode: 4;
   uint16_t fullDischarged: 1;
   uint16_t fullCharged: 1;
   uint16_t Discharge;
   uint16_t battInited: 1;
   uint16_t remainTimeAlrm: 1;         // Remaining time to depletion alarm tripped.
   uint16_t remainCapAlrm: 1;          // Remaining capacity alarm tripped.
   uint16_t reserv10: 1;
   uint16_t treminDischargeAlrm: 1;
   uint16_t otAlrm: 1;
   uint16_t reserv13: 1;
   uint16_t terminChargeAlrm: 1;
   uint16_t overChargeAlrm: 1;

 } sBattStatus;

 // SBS_SPECIFICATION_INFO
 typedef enum {
   SPEC_REV_V0,
   SPEC_REV_V1_0,
   SPEC_REV_V1_1,
   SPEC_REV_V1_1PEC,
   SPEC_REV_VF4,
   SPEC_REV_VF5,
   SPEC_REV_VF6,
   SPEC_REV_VF7,
 } eSpecInfoVers;

typedef struct {
  uint16_t revision: 4;
  eSpecInfoVers version: 4;
  uint16_t vScale: 4;         // Voltage scaling exponent. Multiplies voltages by 10 ^ VScale.
  uint16_t ipScale: 4;        // Current/capacity scaling exp. Multiplies currents and capacities by 10 ^ IPScale.
} sSbsSpecInfo;

typedef struct {
  uint16_t icc: 1;            // 0 - Internal Charge Controller (R) 0 = Function not supported
  uint16_t pbc: 1;            // 1 - Primary Battery Support (R) 0 = Primary Battery Support, 1 = Primary or Secondary Battery Support
  uint16_t reserv2: 1;        // 2 - Reserve 2
  uint16_t reserv3: 1;        // 3 - Reserve 3
  uint16_t reserv4: 1;        // 4 - Reserve 4
  uint16_t reserv5: 1;        // 5 - Reserve 5
  uint16_t reserv6: 1;        // 6 - Reserve 6
  uint16_t cf: 1;             // 7 - Condition Flag 0 = Battery OK 1 = Conditioning cycle requested
  uint16_t cce: 1;            // 8 - Charge Controller Enabled (R/W) 0 = Internal charge controller disabled
  uint16_t pb: 1;             // 9 - Primary Battery (R/W) 0 = Battery operating in its secondary role (default) 1 = Battery operating in its primary role
  uint16_t reserv10: 1;       // 13 - Reserve 10
  uint16_t reserv11: 1;       // 11 - Reserve 11
  uint16_t reserv12: 1;       // 12 - Reserve 12
  uint16_t am: 1;             // 13 - Alarm Mode (R/W) 0 = Enable Alarm Warning broadcasts to host and smart battery charger 1 = Disable Alarm Warning broadcasts to host and smart battery charger
  uint16_t chgm: 1;           // 14 - Charger Mode (R/W) 0 = Enable ChargingVoltage() and ChargingCurrent() broadcasts to host and smart battery charger 1 = Disable ChargingVoltage() and ChargingCurrent() broadcasts to host and smart battery charger
  uint16_t reserv15: 1;       // 15 - CAPM—Capacity Mode (R/W): 0 = mA or mAh (default) 1 = 10 mW or 10 mWh
} sBq30zStatus;


typedef struct {
  sI2cTrans * (* cfgSequence)( sI2cTrans * trans );
  sI2cTrans * (* inSequence)( sI2cTrans * trans );
  FlagStatus cfgFlag;

  sBq30zStatus status;
  sBattStatus  battStatus;

  // Значения параметров БАТАРЕИ
  uint16_t t;                 // Температура Батареи  0.1 гр.Ц
  uint16_t v;                 // Напряжение           mV
  int16_t i;                  // Ток                  mA
  int16_t avgI;               // Средний Ток          mA
  uint16_t remainCap;         // Ожидаемая Емкость        mAh/10 mWh
  uint16_t fullChCap;         // Полная емкость заряда    mAh/10 mWh
  uint16_t timeEmpty;         // RunTimeEmpty         min
  uint16_t avgTimeEmpty;      // AverageRunTimeEmpty  min
  int16_t chI;                // ChargingCurrent      mA
  uint16_t chV;               // ChargingVoltage      mV
  uint16_t cycleCount;        // Счетчик циклов       cycle
  uint16_t desCap;            // DesignCapacity       mAh/10 mWh
  uint16_t desV;              // DesignVoltage        mV
  uint16_t cellV3;            // CellVoltage_3        mV
  uint16_t cellV2;            // CellVoltage_2        mV
  uint16_t cellV1;            // CellVoltage_1        mV
  uint16_t cellV0;            // CellVoltage_0        mV
  uint16_t v0;                // Voltage_0            mV
  uint16_t t0;                // Temperature_0        0.1 гр.Ц
} sBq30z55Dev;

extern sBq30z55Dev gzp6859Dev;
extern const uint8_t bq30zIdKey[16];
extern FlagStatus cfgFlag;

#endif /* GPZ6859_H_ */
