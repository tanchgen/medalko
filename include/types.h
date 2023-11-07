/*
 * types.h
 *
 *  Created on: 19 мая 2021 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef TYPES_H_
#define TYPES_H_


typedef enum {
  ACT_NO_ACT,
  ACT_HW_LOG,
  ACT_FPGA_OFF,
  ACT_SYS_OFF,
  ACT_SYS_FAIL_OFF,
  ACT_SYS_ALRT_OFF,
  ACT_PSU_ALRT_OFF,
  ACT_FPGA_ALRT_OFF,
} eStatusAct;

typedef enum {
  MCUSTATE_BAT_OFF,     // 0
  MCUSTATE_SYS_OFF,     // 1
  MCUSTATE_SYS_START,   // 2
  MCUSTATE_PSU_ON,      // 3
  MCUSTATE_I2C_ON,      // 4
  MCUSTATE_FAN_ON,      // 5
  MCUSTATE_RUN_1,       // 6
  MCUSTATE_RUN_2,       // 7
  MCUSTATE_RUN_3,       // 8
  MCUSTATE_PLL_LOCK,    // 9
  MCUSTATE_MZU_DONE,    // 10
  MCUSTATE_ISTART,      // 11
  MCUSTATE_SYS_ON,      // 12
  MCUSTATE_FPGA_START,  // 13
  MCUSTATE_FPGA_1,      // 14
  MCUSTATE_FPGA_2,      // 15
  MCUSTATE_FPGA_3,      // 16
  MCUSTATE_FPGA_DONE,   // 17
  MCUSTATE_FPGA_ON,     // 18
} eMcuState;

typedef enum {
  INT,
  FLOAT
} ePtype;



#endif /* TYPES_H_ */
