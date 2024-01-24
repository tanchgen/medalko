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
#include "tm.h"
#include "measur.h"

#define VREFINT_VOL     1200

//#define LEARN_COUNT_MIN   100
#define LEARN_COUNT_MAX   200

#define ADC_RESULT_PERIOD   10  // периодичность сохранения намеряных результатов ~10мс

#define ADC_TIM                 TIM3
#define ADC_TIM_CLK_EN          RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define ADC_TIM_IRQn            TIM3_IRQn
#define ADC_TIM_IRQ_PRIORITY    (3)
#define ADC_TIM_FREQ            10000   // T=100mks

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

#ifndef PRESS_ADC
#error "PRESS_ADC not defined"
#endif // PRESS_ADC

#define VDD_AVG           1
#define TERM_AVG          0
#define PRESS_AVG         0
#define ALCO_AVG          0

#if PRESS_ADC
#if PRESS_AVG
#define PRESS_AVRG_IDX      10
#endif //PRESS_AVG
#endif //PRESS_ADC

#if VDD_AVG
#define VDD_AVRG_IDX      10
#endif //VDD_AVG

#if ALCO_AVG
#define ALCO_AVRG_IDX      10
#endif //ALCO_AVG

#if TERM_AVG
#define TERM_AVRG_IDX      10
#endif //TERM_AVG

typedef enum {
  ADC_PRM_VDD,
  ADC_PRM_TERM,
#if PRESS_ADC
  ADC_PRM_PRESS,
#endif // PRESS_ADC
  ADC_PRM_ALCO,
  ADC_PRM_VAD,
  ADC_PRM_NUM
} eAdcPrm;

typedef struct __attribute__((__packed__)){
// ----------- ADC_PARAM_TM ---------------------
  union {
    struct {
      uint16_t paramNok:  1;
      uint16_t paramPeakNok:  1;
      uint16_t paramLowLim: 1;
      uint16_t paramHiLim: 1;
    };
    uint16_t u16adcSt;
  };
  uint16_t  param;
  uint16_t  paramPeakMin;
  uint16_t  paramPeakMax;
  uint16_t  paramLowAlrmLimit;
  uint16_t  paramHiAlrmLimit;
} sAdcParamTm;

// Структура данных АЦП
typedef struct __attribute__((__packed__)) {
  union {
    struct {
      uint16_t prmNok:   1;
      uint16_t prmPeakNok:  1;
      uint16_t prmLowLim: 1;
      uint16_t prmHiLim: 1;
    };
    uint16_t u16status;
  };

  int32_t  prm;
  int32_t  prmPeakMin;
  int32_t  prmPeakMax;
  int32_t  prmLowAlrmLimit;
  int32_t  prmHiAlrmLimit;
} sAdcData;

typedef struct {
  struct {
    uint16_t adcVprm[ADC_PRM_NUM];
  };
  uint16_t avgVdd;

  struct {
      uint16_t prmNok:  1;
      uint16_t prmPeakNok:  1;
  } paramStatus[ADC_PRM_NUM];
  sAdcData adcData[ADC_PRM_NUM];

  FlagStatus adcOn;
  FlagStatus adcOk;

  // PRESS
  uint16_t pressCount;
  FlagStatus learnFlag;
  int32_t pressAvg;
} sAdcHandle;

// Структура флагов изменения значения параметра, пиков, порогов
typedef union {
  FlagStatus vinDevi[ADC_PRM_NUM];      // Флаг изменения параметра и/или min-max
  uint32_t u32Devi;
} uAdcDeviation;


/** Структура дескриптора модуля АЦП: Бат. CMOS (Bat_CMOS, vbat) */
extern sAdcHandle adcHandle;

/**
  * @brief  Сброс пиковых значений данных АЦП.
  *
  * @param none
  *
  * @retval none
  */
static inline void adcPeakReset( sAdcHandle * adc){
  for( eAdcPrm i = 0; i < ADC_PRM_NUM; i++ ){
    adc->paramStatus[i].prmPeakNok = SET;
    adc->adcData[i].prmPeakMax = INT32_MIN;
    adc->adcData[i].prmPeakMax = INT32_MAX;
  }
}

#endif /* ADC_H_ */
