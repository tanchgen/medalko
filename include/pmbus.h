#ifndef _PMBUS_H
#define _PMBUS_H

#include <stddef.h>
#include <stdint.h>


/**
  * @defgroup	Регистры PMBus (The Power Management Bus).
  * @{
  */

#define PMBUS_REG_PAGE			(0x00)
#define PMBUS_REG_OPERATION		(0x01)
#define PMBUS_REG_ON_OFF_CONFIG		(0x02)
#define PMBUS_REG_CLEAR_FAULTS		(0x03)
#define PMBUS_REG_PHASE			(0x04)

#define PMBUS_REG_WRITE_PROTECT		(0x10)
#define PMBUS_REG_STORE_DEFAULT_ALL	(0x11)
#define PMBUS_REG_RESTORE_DEFAULT_ALL	(0x12)
#define PMBUS_REG_STORE_USER_ALL	(0x15)
#define PMBUS_REG_RESTORE_USER_ALL	(0x16)
#define PMBUS_REG_CAPABILITY		(0x19)
#define PMBUS_REG_QUERY			(0x1A)
 
#define PMBUS_REG_VOUT_MODE		(0x20)
#define PMBUS_REG_VOUT_COMMAND		(0x21)
#define PMBUS_REG_VOUT_TRIM		(0x22)
#define PMBUS_REG_VOUT_CAL_OFFSET	(0x23)
#define PMBUS_REG_VOUT_MAX		(0x24)
#define PMBUS_REG_VOUT_MARGIN_HIGH	(0x25)
#define PMBUS_REG_VOUT_MARGIN_LOW	(0x26)
#define PMBUS_REG_VOUT_TRANSITION_RATE	(0x27)
#define PMBUS_REG_VOUT_DROOP		(0x28)
#define PMBUS_REG_VOUT_SCALE_LOOP	(0x29)
#define PMBUS_REG_VOUT_SCALE_MONITOR	(0x2A)
 
#define PMBUS_REG_COEFFICIENTS		(0x30)
#define PMBUS_REG_POUT_MAX		(0x31)
#define PMBUS_REG_MAX_DUTY		(0x32)
#define PMBUS_REG_FREQUENCY_SWITCH	(0x33)
#define PMBUS_REG_VIN_ON		(0x35)
#define PMBUS_REG_VIN_OFF		(0x36)
#define PMBUS_REG_INTERLEAVE		(0x37)
#define PMBUS_REG_IOUT_CAL_GAIN		(0x38)
#define PMBUS_REG_IOUT_CAL_OFFSET	(0x39)
 
#define PMBUS_REG_VOUT_OV_FAULT_LIMIT		(0x40)
#define PMBUS_REG_VOUT_OV_FAULT_RESPONSE	(0x41)
#define PMBUS_REG_VOUT_OV_WARN_LIMIT		(0x42)
#define PMBUS_REG_VOUT_UV_WARN_LIMIT		(0x43)
#define PMBUS_REG_VOUT_UV_FAULT_LIMIT		(0x44)
#define PMBUS_REG_VOUT_UV_FAULT_RESPONSE	(0x45)
#define PMBUS_REG_IOUT_OC_FAULT_LIMIT		(0x46)
#define PMBUS_REG_IOUT_OC_FAULT_RESPONSE	(0x47)
#define PMBUS_REG_IOUT_OC_LV_FAULT_LIMIT	(0x48)
#define PMBUS_REG_IOUT_OC_LV_FAULT_RESPONSE	(0x49)
#define PMBUS_REG_IOUT_OC_WARN_LIMIT		(0x4A)
#define PMBUS_REG_IOUT_UC_FAULT_LIMIT		(0x4B)
#define PMBUS_REG_IOUT_UC_FAULT_RESPONSE	(0x4C)
#define PMBUS_REG_OT_FAULT_LIMIT		(0x4F)

#define PMBUS_REG_OT_FAULT_RESPONSE	(0x50)
#define PMBUS_REG_OT_WARN_LIMIT		(0x51)
#define PMBUS_REG_UT_WARN_LIMIT		(0x52)
#define PMBUS_REG_UT_FAULT_LIMIT	(0x53)
#define PMBUS_REG_UT_FAULT_RESPONSE	(0x54)
#define PMBUS_REG_VIN_OV_FAULT_LIMIT	(0x55)
#define PMBUS_REG_VIN_OV_FAULT_RESPONSE	(0x56)
#define PMBUS_REG_VIN_OV_WARN_LIMIT	(0x57)
#define PMBUS_REG_VIN_UV_WARN_LIMIT	(0x58)
#define PMBUS_REG_VIN_UV_FAULT_LIMIT	(0x59)
#define PMBUS_REG_VIN_UV_FAULT_RESPONSE	(0x5A)
#define PMBUS_REG_IIN_OC_FAULT_LIMIT	(0x5B)
#define PMBUS_REG_IIN_OC_WARN_LIMIT	(0x5D)
#define PMBUS_REG_POWER_GOOD_ON		(0x5E)
#define PMBUS_REG_POWER_GOOD_OFF	(0x5F)
 
#define PMBUS_REG_TON_DELAY			          (0x60)
#define PMBUS_REG_TON_RISE			      (0x61)
#define PMBUS_REG_TON_MAX_FAULT_RESPONSE	(0x63)
#define PMBUS_REG_TOFF_DELAY			(0x64)
#define PMBUS_REG_TOFF_FALL			(0x65)
#define PMBUS_REG_POUT_OP_FAULT_LIMIT		(0x68)
#define PMBUS_REG_POUT_OP_WARN_LIMIT		(0x6A)
#define PMBUS_REG_PIN_OP_WARN_LIMIT		(0x6B)
 
#define PMBUS_REG_STATUS_BYTE		(0x78)
#define PMBUS_REG_STATUS_WORD		(0x79)
#define PMBUS_REG_STATUS_VOUT		(0x7A)
#define PMBUS_REG_STATUS_IOUT		(0x7B)
#define PMBUS_REG_STATUS_INPUT		(0x7C)
#define PMBUS_REG_STATUS_TEMPERATURE	(0x7D)
#define PMBUS_REG_STATUS_CML		(0x7E)
#define PMBUS_REG_STATUS_OTHER		(0x7F)

#define PMBUS_REG_STATUS_MFR_SPECIFIC	(0x80)
#define PMBUS_REG_STATUS_FAN_1_2	(0x81)
#define PMBUS_REG_STATUS_FAN_3_4	(0x82)
#define PMBUS_REG_READ_VIN		(0x88)
#define PMBUS_REG_READ_IIN		(0x89)
#define PMBUS_REG_READ_VCAP		(0x8A)
#define PMBUS_REG_READ_VOUT		(0x8B)
#define PMBUS_REG_READ_IOUT		(0x8C)
#define PMBUS_REG_READ_TEMPERATURE_1	(0x8D)
#define PMBUS_REG_READ_TEMPERATURE_2	(0x8E)
#define PMBUS_REG_READ_TEMPERATURE_3	(0x8F)

#define PMBUS_REG_READ_FAN_SPEED_1	(0x90)
#define PMBUS_REG_READ_FAN_SPEED_2	(0x91)
#define PMBUS_REG_READ_FAN_SPEED_3	(0x92)
#define PMBUS_REG_READ_FAN_SPEED_4	(0x93)
#define PMBUS_REG_READ_DUTY_CYCLE	(0x94)
#define PMBUS_REG_READ_FREQUENCY	(0x95)
#define PMBUS_REG_READ_POUT		(0x96)
#define PMBUS_REG_READ_PIN		(0x97)
#define PMBUS_REG_REVISION		(0x98)
#define PMBUS_REG_MFR_ID		(0x99)
#define PMBUS_REG_MFR_MODEL		(0x9A)
#define PMBUS_REG_MFR_REVISION		(0x9B)
#define PMBUS_REG_MFR_LOCATION		(0x9C)
#define PMBUS_REG_MFR_DATE		(0x9D)
#define PMBUS_REG_MFR_SERIAL		(0x9E)
 
/**
  * @}
  */

/**
  * @brief	Преобразование данных из формата Linear-11 в действительное значение величины.
  *
  * @param[in]	data	преобразуемые данные
  *
  * @retval	действительное значение величины
  */
float pmbus_data_linear_11_to_regular(uint16_t data);

/**
  * @brief	Преобразование данных из действительного значения в формат Linear-11.
  *
  * @param[in]	data	преобразуемые данные
  *
  * @retval	данные в формате Linear-11
  */
uint16_t pmbus_data_regular_to_linear_11(float data);

/**
  * @brief  Преобразование данных в Linear_11
  *
  * @param[in] data  Преобразовываемые данные (ед.изм. = 1/k от основной ед.)
  *
  * @retval Данные в формате linear_11
  */
uint16_t toLinear11( int32_t data, uint16_t k );

/**
  * @brief  Преобразование данных из Linear_11
  *
  * @param[in] data  Данные в формате linear_11
  *
  * @retval Преобразованные данные (ед.изм. = 1/4096 от основной ед.)
  */
int32_t fromLinear11( int32_t data, uint16_t k  );

#endif /* _PMBUS_H */

