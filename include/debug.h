/*
 * error.h
 *
 *  Created on: 26 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>
#include <stddef.h>

typedef enum {
  ERR_OK,         /** < Отсутсвие ошибок */
  ERR_I2C1_LINK,  /** < Ошибка связи с устройством I2C1 */
  ERR_I2C2_LINK,  /** < Ошибка связи с устройством I2C2 */
  ERR_I2C1_DEV,   /** < Ошибка устройства I2C1 */
  ERR_I2C2_DEV,   /** < Ошибка устройства I2C2 */
  ERR_I2C_BUF,    /** < Ошибка обращения к буферу данных I2C2 */
  ERR_I2C_CRC,      /** < Ошибка CRC полученых от устройства I2C данных */

  ERR_CAN_BUF,      /** < Ошибка записи в буфер отправки CAN */
  ERR_CAN_SRCID,    /** < Неправильный SrcID */
  ERR_CAN_DEVID,    /** < Неправильный DevID */
  ERR_CAN_PWRLINE,  /** < Неправильный PwrLine */
  ERR_CAN_PRMID,    /** < Неправильный PrmID */
  ERR_CAN_PRMTYPE,  /** < Неправильный PrmType */
  ERR_CAN_PRM,      /** < Неправильное значения Параметра/MAX/MIN/LIMITS */

  // ---------------- ETH ERROR ---------------------
  ERR_TCP_CONN_MAX,   /** < Превышение максимального количества TCP-клиентов */

  ERR_SYS_ON_TOUT,  /** < Ошибка Таймаут включения системы */
  ERR_SYS_OFF_TOUT, /** < Ошибка Таймаут выключения системы */

  ERR_PWR_ALRT,
  ERR_PWR_ALRM,
  ERR_ADC,        /** < Ошибка АЦП   */
  ERR_BAT_DOWN,   /** < Ошибка: не падает напржение Bat_CMOS */
  ERR_BAT_UP,     /** < Ошибка: не растет напржение Bat_CMOS */
  ERR_ADC_HCUR,   /** < Ошибка АЦП Hcur (Ток нагревателя) */
  ERR_FAN,        /** < Ошибка Вентилятора */
  ERR_DSW,        /** < Ошибка: +3.3В DSW */
  ERR_START_I,    /** < Ошибка: превышен таймаут запуска INTEL */
  ERR_STOP_I,     /** < Ошибка: превышен таймаут остановки INTEL */
  ERR_CATERR_I,   /** < Ошибка: CatError INTEL */
  ERR_DATA_NRDY,  /** < Ошибка: Общая ошибка - данные не готовы */
  ERR_REG_CODE, /**< Неизвестный код регистра SLAVE-модуля */
  ERR_PLL,
  ERR_USB,

  ERR_UNKNOWN,    /** <  Неизвестная ошибка */
  ERR_NUM
} eErr;

extern const char * alrmMsg[];
extern const char * pwrStateMsg[];

void dbgSend( const char * buf, uint8_t len);
int dbgPrintf(const char* format, ...);
void dbgPuts(const char *s);
void errHandler( eErr err );

void * my_malloc( size_t size );
void my_free( void * p );

#endif /* ERROR_H_ */
