/*
 * adc.h
 *
 *  Created on: 26 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ADC_H_
#define ADC_H_

#ifdef STM32F4XX
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#endif // STM32F4XX

#include "gpio.h"

#define ADC_CH_NUM    2
#define VBAT_CH      8

#define ADC_RESULT_PERIOD   10  // периодичность сохранения намеряных результатов ~10мс

#define ADC_TIM                 TIM8
#define ADC_TIM_CLK_EN          RCC_APB2ENR_TIM8EN
#define ADC_TIM_IRQn            TIM8_IRQn
#define ADC_TIM_IRQ_PRIORITY    (3)
#define ADC_TIM_FREQ            10000

/* Temperature sensor calibration value address */


#define ADC_MAX_TR  3350  /** < Значение АЦП THRESHOLD - Верхнее граничное значение батареи CMOS (2.7В) */
#define ADC_MIN_TR  620   /** < Значение АЦП THRESHOLD - Верхнее граничное значение батареи CMOS (0.5В) */

#define VBAT_ON_VOLT  2750
#define VBAT_OFF_VOLT  100

#define ADC_MAX_TR  3350  /** < Значение АЦП THRESHOLD - Верхнее граничное значение батареи CMOS (2.7В) */
#define ADC_MIN_TR  620   /** < Значение АЦП THRESHOLD - Верхнее граничное значение батареи CMOS (0.5В) */

#define HEAT_TIM      TIM14
#define HEATER_R      500       /** < Сопротивление нагревателя, с которого снимается контрольное напряжение, (1мОм) */
#define HCUR_K        100     /** < Коэффициент пересчета тока нагревателя в (1/1000)*/
#define HCUR_MAX      600   /** < Максимально допустимый ток нагревателя */
#define HCUR(x)        (( (x) * HCUR_K) / HEATER_R)

#define BAT_OFF_TOUT      5000  /** < Таймаут на выключение батареи CMOS (1мс) */
#define BAT_ON_TOUT_TOUT  5000  /** < Таймаут на задержку включения батареи CMOS (1мс) */
#define BAT_ON_TOUT       1000   /** < Таймаут на включение батареи CMOS  (1мс) */




#define ADC_AVRG_IDX      5

typedef enum {
  ADC_PARAM_VBAT,
  ADC_PARAM_NUM
} eAdcParam;

typedef struct __attribute__((__packed__)){
// ----------- ADC_PARAM_TM ---------------------
  union {
    struct {
      uint16_t paramNok:  1;          /** < Флаг признака отсутствия готовности данных 3.3V OPT */
      uint16_t paramPeakNok:  1;      /** < Флаг признака отсутствия готовности пиковых данных 3.3V OPT */
      uint16_t paramLowLim: 1;        /**< Нижнее пороговое значение напряжения 3.3V OPT. */
      uint16_t paramHiLim: 1;       /**< Верхнее пороговое значение напряжения 3.3V OPT. */
    };
    uint16_t u16adcSt;
  };
  uint16_t  param;         /**< Мгновенное значение напряжения 3.3V OPT. */
  uint16_t  paramPeakMin;         /**< Нижнее пиковое значение напряжения 3.3V OPT. */
  uint16_t  paramPeakMax;         /**< Верхнее пиковое значение напряжения 3.3V OPT. */
  uint16_t  paramLowAlrmLimit;         /**< Нижнее пороговое значение напряжения 3.3V OPT. */
  uint16_t  paramHiAlrmLimit;         /**< Верхнее пороговое значение напряжения 3.3V OPT. */
} sAdcParamTm;

// Структура данных АЦП
typedef struct __attribute__((__packed__)) {
  union {
    struct {
      uint16_t paramAdcNok:   1;         /** < Флаг признака отсутствия готовности данных 3.3V OPT */
      uint16_t paramAdcPeakNok:  1;      /** < Флаг признака отсутствия готовности пиковых данных 3.3V OPT */
      uint16_t paramAdcLowLim: 1;        /**< Флаг признака пересечения нижнего пороговое значение напряжения 3.3V OPT. */
      uint16_t paramAdcHiLim: 1;       /**< Флаг признака пересечения верхнего пороговое значение напряжения 3.3V OPT. */
    };
    uint16_t u16status;
  };

  uint16_t  paramAdc;                /**< Мгновенное значение напряжения 3.3V OPT (1мВ)*/
  uint16_t  paramAdcPeakMin;          /**< Нижнее пиковое значение тока 3.3V OPT. */
  uint16_t  paramAdcPeakMax;          /**< Нижнее пиковое значение тока 3.3V OPT. */
  uint16_t  paramAdcLowAlrmLimit;         /**< Нижнее пороговое значение напряжения 3.3V OPT. (1мВ)*/
  uint16_t  paramAdcHiAlrmLimit;         /**< Верхнее пороговое значение напряжения 3.3V OPT. (1мВ)*/
} sAdcData;

typedef struct {
  struct {
    uint8_t batOff:1 ;   /** < Флаг признака выключенной батареи */
    uint8_t batOffOk: 1; /** < Флаг признака что напряжение батареи снизилось до нужного уровня (~ < 0.5В) */
    uint8_t vbatOk: 1;   /** < Флаг признака что напряжение батареи поднялось до нужного уровня (~ > 2.7В) */
    uint8_t adcOn: 1;    /** < Флаг признака включенного АЦП */
    uint8_t adcOk: 1;    /** < Флаг признака полученных значений АЦП->DMA */
    uint8_t clrBatErr: 1;  /** < Флаг ошибки стирания CMOS */
  };
  struct {
    uint16_t adcVparam[ADC_PARAM_NUM];      /** < Напряжение OPT1 - значение АЦП */
    uint16_t adcVref;   /** < Значение АЦП - опорное напряжение АЦП */
  };
  uint16_t vdd;       /** < Напряжение питания MCU - 1 мВ */
  volatile FlagStatus * clrBatFlag;

  struct {
      uint16_t paramNok:  1;          /** < Флаг признака отсутствия готовности данных 3.3V OPT */
      uint16_t paramPeakNok:  1;      /** < Флаг признака отсутствия готовности пиковых данных 3.3V OPT */
  } paramStatus[ADC_PARAM_NUM];
  sParamData adcData[ADC_PARAM_NUM];

} sAdcHandle;

// Структура флагов изменения значения параметра, пиков, порогов
typedef union {
  FlagStatus vinDevi[ADC_PARAM_NUM];      // Флаг изменения параметра и/или min-max
  uint32_t u32Devi;
} uAdcDeviation;


/** Структура дескриптора модуля АЦП: Бат. CMOS (Bat_CMOS, vbat) */
extern sAdcHandle adcHandle;

void batOffStart( sGpioPin * pinBatOff, sGpioPin * pinBatDisch );

/**
  * @brief  Обновление данных телеметрии АЦП.
  *
  * @param[out] to    буфер данных телеметрии АЦП
  *
  * @retval none
  */
void refreshAdcParamTm(eAdcParam adcopt, sAdcParamTm *to);
size_t refreshAdcTm( sAdcParamTm * padctm );

/**
  * @brief  Обновление данных АЦП на полученные по UART
  *
  * @param[in]  from_offset смещение регистра от начала поля данных
  * @param[in]  from    указатель на значение регистра
  * @param[in]  from_size длина значения регистра
  *
  * @retval   false Успешное обновление данных
  *           true  Ошибка при обновлении данных
  */
bool arch_updateAdcTm( size_t regaddr, uint16_t regdata );

/**
  * @brief  Сброс пиковых значений данных АЦП.
  *
  * @param none
  *
  * @retval none
  */
static inline void adcPeakReset( sAdcHandle * adc, eAdcParam num){
  adc->paramStatus[num].paramPeakNok = SET;
  paramPeakReset( &(adc->adcData[num]) );
}

#endif /* ADC_H_ */
