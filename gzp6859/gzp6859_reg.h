/*
 * gzp6859_reg.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

// "Texas Instruments BQ30z55 chip"

#ifndef GPZ6859_REG_H_
#define GPZ6859_REG_H_

/**
  * @defgroup Регистры GPZ6859.
  * @{
  */

/*
 * Smart Battery Data Specification 1.1 commands list
 * This is a list taken directly from specification.
 */

// GPZ6859 SBS_CMD

typedef enum {
  MFR_ACCESS  =  0x00,              // ManufacturerAccess
  REMAIN_CAP_ALRM  =  0x01,         // RemainingCapacityAlarm
  REMAIN_TIME_ALRM  =  0x02,        // RemainingTimeAlarm
  BATT_MODE  =  0x03,               // BatteryMode
  AT_RATE  =  0x04,                 // AtRate
  AT_RATE_TO_FILL  =  0x05,         // AtRateToFull
  AT_RATE_TO_EMPTY  =  0x06,        // AtRateToEmpty
  AT_RATE_OK  =  0x07,              // AtRateOK
  TEMPERATURE  =  0x08,             // Temperature
  VOLT  =  0x09,                    // Voltage
  CURR  =  0x0a,                    // Current
  AVG_CURR  =  0x0b,                // AverageCurrent
  MAX_ERR  =  0x0c,                 // MaxError
  RELAT_STATE_OF_CHARGE  =  0x0d,   // RelativeStateOfCharge
  ABS_STATE_OF_CHARGE  =  0x0e,     // AbsoluteStateOfCharge
  REMAIN_CAP  =  0x0f,              // RemainingCapacity
  FULL_CHARGE_CAP  =  0x10,         // FullChargeCapacity
  RUN_TIME_EMPTY  =  0x11,          // RunTimeToEmpty
  AVG_TIME_EMPTY  =  0x12,          // AverageTimeToEmpty
  AVG_TIME_FULL  =  0x13,           // AverageTimeToFull
  CHARGE_CURR  =  0x14,             // ChargingCurrent
  CHARGE_VOLT  =  0x15,              // ChargingVoltage
  BATT_STATUS  =  0x16,             // BatteryStatus
  CYCLE_COUNT  =  0x17,             // CycleCount
  DESIGN_CAP  =  0x18,              // DesignCapacity
  DESIGN_VOLT  =  0x19,             // DesignVoltage
  SPEC_INFO  =  0x1a,               // SpecificationInfo
  MFR_DATE  =  0x1b,                // ManufactureDate
  SERIAL_NUM  =  0x1c,              // SerialNumber
  MFR_NAME  =  0x20,                // ManufacturerName
  DEVICE_NAME  =  0x21,             // DeviceName
  DEVICE_CHEMISTRY  =  0x22,        // DeviceChemistry
  MFR_DATA  =  0x23,                // ManufacturerData
  OPT_MFR_FUNC5  =  0x2f,           // OptionalMfgFunction5
  CELL_VOLT_3 = 0x3c,               // Cell Voltag
  CELL_VOLT_2 = 0x3d,               // OptionalMfgFunction3
  CELL_VOLT_1 = 0x3e,               // OptionalMfgFunction2
  CELL_VOLT_0 = 0x3f,               // OptionalMfgFunction1
  MFR_INFO = 0x70,                  // ManufacturerInfo
  VOLTAGE_FULL = 0x71,              // FullVoltage
  TEMPERATURE_FULL = 0x72,          // FullTemperature
} eSbsCmd;

typedef enum {
  MFR_ACC_DATA = 0x0000,            // ManufacturerData
  DEV_TYPE = 0x0001,                // DeviceType
  FW_VER = 0x0002,                  // FirmwareVersion
  HW_VER = 0x0003,                  // HardwareVersion
} eMfrAccSubCmd;

// Address spaces used in BQ family SBS chips
typedef enum {
  DATA_FLASH    = 0,
  INSTR_FLASH   = 1,
} eRawAddrSpace;


// Flags used in BatteryMode command
typedef enum {
  INT_CHARGE_CTRL = 0,        // INTERNAL_CHARGE_CONTROLLER  = 0
  PRIM_BATT_SUPP = 1,         // PRIMARY_BATTERY_SUPPORT   = 1
  RESERVED2 = 2,              // RESERVED2 = 2
  RESERVED3 = 3,              // RESERVED3 = 3
  RESERVED4 = 4,              // RESERVED4 = 4
  RESERVED5 = 5,              // RESERVED5 = 5
  RESERVED6 = 6,              // RESERVED6 = 6
  OCND_FLAG = 7,              // CONDITION_FLAG = 7
  CHARGE_CTRL_EN = 8,         // CHARGE_CONTROLLER_ENABLED = 8
  PRIM_BATT = 9,              // PRIMARY_BATTERY       = 9
  SBS_RSRV10 = 10,            // RESERVED10 = 10
  SBS_RSRV11 = 11,            // RESERVED11 = 11
  RESERVED12 = 12,            // RESERVED12 = 12
  ALRM_MODE = 13,             // ALARM_MODE = 13
  CHARGER_MODE = 14,          // CHARGER_MODE = 14
  CAPACITY_MODE = 15
} eSbsFlagBatt;

// Flags used in BatteryStatus command
typedef enum {
  ERROR_CODE = 0,
  FULLY_DISCHARGED = 4,
  FULLY_CHARGED = 5,
  DISCHARGING = 6,
  INITIALIZED = 7,
  REMAINING_TIME_ALRM = 8,
  REMAINING_CAPACITY_ALRM = 9,
  SBS_BATT_RSRV10 = 10,
  TERMINATE_DISCHARGE_ALRM = 11,
  OT_ALRM = 12,
  RESERVED13 = 13,
  TERMINATE_CHARGE_ALRM = 14,
  OVER_CHARGED_ALARM = 15,
} eSbsBattStatus;

// Flags used in SpecificationInfo command
typedef enum {
  REVISION = 0,
  VERSION = 4,
  VSCALE = 8,
  IPSCALE = 12
} eSbsSpecInfo;

// List of groups of commands/offsets.
typedef enum {
  DEV_INFO = 0x00,
  USE_INFO = 0x01,
  COMPUTED_INFO = 0x02,
  STATUSBITS = 0x03,
  AT_RATES = 0x04,
  BQ_STATUS_BIT = 0x06,
  BQ_STATUS_BIT_MA = 0x07,
  BQ_CELL_VOLT = 0x08,
  BQ_LIFE_TIME_DATA = 0x09,
  BQ_LIFE_TIME_DATA_MA = 0x0a,
  IMPEDANCE_TRACK = 0x0b,
  IMPEDANCE_TRACK_MA = 0x0c,
  BQ_TURBO_MODE = 0x0f,
} eMonGroup;


typedef struct {
  uint8_t i2cAddr;
  uint8_t page;
  uint16_t vout;
  float ioutMin;
  float ioutMax;
} sBq30SmbusReg;

extern const sBq30SmbusReg bq30PmbusReg;

#endif /* GPZ6859_REG_H_ */
