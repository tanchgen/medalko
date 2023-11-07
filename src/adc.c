/*
 * adc.c
 *
 *  Created on: 27 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */
#include <math.h>

#include "main.h"
#include "adc.h"
#include "gpio_arch.h"
#include "measur.h"

#define R1          10000L       // Сопротивление резистора R1
#define LN_RT25     9.21034     // ln(Rt) при 25гр.Ц
#define B25_50      3900
#define B25_80      3930
#define B25_85      3934
#define B25_100     3944
#define _1_298K     0.0033557   // Обратная величина от (25 + 273)гр.Ц

#define PRESS_NUL   2500        // Напряжение нулевого давления

/** Структура дескриптора модуля АЦП: Бат. CMOS (Bat_CMOS, vbat) */
sAdcHandle adcHandle = {
  .adcOn = RESET,
  .adcOk = RESET,
};

sGpioPin gpioPinAdcT = {GPIOA, 0, GPIO_Pin_0, 0, GPIO_MODE_AIN, GPIO_PULLDOWN, Bit_RESET, Bit_RESET, RESET };
sGpioPin gpioPinAdcPress = {GPIOA, 0, GPIO_Pin_1, 1, GPIO_MODE_AIN, GPIO_PULLDOWN, Bit_RESET, Bit_RESET, RESET };
sGpioPin gpioPinAdcAlco = {GPIOA, 0, GPIO_Pin_2, 2, GPIO_MODE_AIN, GPIO_PULLDOWN, Bit_RESET, Bit_RESET, RESET };


#define ADC_KPARAM_0    (4096L)   // Делитель для VBAT: 10/20

const uint16_t adcKprm[ADC_PRM_NUM] = {
    ADC_KPARAM_0,
    ADC_KPARAM_0,
    ADC_KPARAM_0,
    ADC_KPARAM_0
};


void adcStart( void );

//// Расчет скользящего среднего
//static inline void fMovAvg( float *avg, float pt ){
//  const float a = 2 / (1+10);
//
//  *avg = pt * a + (*avg * (1 - a));
//}
//


// Расчет скользящего среднего беззнакового
inline void movAvgU( uint16_t *avg, uint32_t pt ){
  const uint32_t a = 2000 / (1+ ADC_AVRG_IDX);
  uint32_t tmp = *avg;
  *avg = (uint16_t)((pt * a + (tmp * (1000 - a)) + 500)/1000);
}

// Расчет скользящего среднего беззнакового
inline void movAvgS( int16_t *avg, int32_t pt ){
  const int32_t a = 2000 / (1+ ADC_AVRG_IDX);
  int32_t tmp = *avg;
  *avg = (int16_t)((pt * a + (tmp * (1000 - a)) + 500)/1000);
}


//inline void adcDataReset( sAdcHandle * adc, eAdcPrm num ){
//  timerModArg( &adcPeakTimer, TOUT_100, num );
//  adc->adcOk = RESET;
//  adc->adcData[num].u16status = 0;
//}
//
//void adcPeakTout( uintptr_t arg ){
//  adcPeakReset( &(adcHandle), (eAdcPrm)arg  );
//}


/**
  * @brief  This function configures DMA for transfer of data from ADC
  * @param  None
  * @retval None
  */
void adcDmaInit(void) {
  // Для синхронной работы требуется четное число каналов
  MAYBE_BUILD_BUG_ON( ADC_PRM_NUM != ((ADC_PRM_NUM/2) * 2) );

  /*## Configuration of DMA ##################################################*/
  /* Enable the peripheral clock of DMA */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  /* Configure the DMA transfer */
  DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR = (uint32_t)adcHandle.adcVprm;
  DMA1_Channel1->CCR = DMA_MemoryDataSize_Word | DMA_PeripheralDataSize_Word
      | DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_TCIE | DMA_CCR1_TEIE | DMA_CCR1_PL_1;
  /* Set DMA transfer addresses of source and destination */
  DMA1_Channel1->CNDTR = ADC_PRM_NUM/2;
  NVIC_SetPriority( DMA1_Channel1_IRQn, 2 );
}


// Batary volt sensor init function
void adcInit(void){
// --------- ADC ------------------------------------

  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;// | RCC_APB2ENR_SYSCFGEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;// | RCC_APB2ENR_SYSCFGEN;

  ADC1->CR2 &= ~ADC_CR2_ADON;
  ADC2->CR2 &= ~ADC_CR2_ADON;

  // Режим сканирования
  ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2;
  ADC2->CR1 = ADC_CR1_SCAN;
  // Режим перебора каналов и DMA Выбор триггера для преобразования TIM3_TRGO
  ADC1->CR2 = ADC_CR2_TSVREFE | ADC_CR2_DMA | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2;
  ADC2->CR2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2;

  // Меряем 2 канала: канал 0 (ADC_TEMP) и Vref(Ch17)
  ADC1->SQR1 = ((ADC_PRM_NUM / 2) - 1) << 20;
  ADC1->SQR3 = (VDD_CH << 0) | (TEMP_CH << 5);
  // Меряем 2 канала: канал 2 (ADC_PRESS) и 3 (ADC_ALCO)
  ADC2->SQR1 = ((ADC_PRM_NUM / 2) - 1) << 20;
  ADC2->SQR3 = (PRESS_CH << 0) | (ALCO_CH << 5);

  // Длительность сэмпла = 13.5 ADCCLK
  ADC1->SMPR1 = ADC_SMPR1_SMP17_1;
  ADC1->SMPR2 = ADC_SMPR2_SMP0_1;
  ADC2->SMPR2 = ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP3_1;

  ADC1->CR2 |= ADC_CR2_ADON;
  ADC2->CR2 |= ADC_CR2_ADON;
  // Wait for ADC ON >= (2 ADC_CLK)
  for( uint32_t wait = (32 >> 1); wait; wait-- )
  {}
  ADC1->CR2 |= ADC_CR2_CAL;
  ADC2->CR2 |= ADC_CR2_CAL;
  // Wait ADC1 CAL and ADC2 CAL
  while( (ADC1->CR2 |= ADC_CR2_CAL) || (ADC2->CR2 |= ADC_CR2_CAL) )
  {}

}


/**
  * @brief  Настройка вывода GPIO для АЦП 3.3V_OPT1-5
  * @param  None
  *
  * @retval None
  */
static inline void adcGpioInit( void ){

  gpioPinSetup( &gpioPinAdcT );
  gpioPinSetup( &gpioPinAdcPress );
  gpioPinSetup( &gpioPinAdcAlco );
}


/**
  * @brief  Настройка таймера тригера АЦП
  * @param  None
  *
  * @retval None
  */
inline void adcTrigTimInit( void ){
  uint16_t psc;
  uint8_t rc;

  ADC_TIM_CLK_EN;

//  TODO: Сделать расчет psc
  rc = timPscSet( ADC_TIM, ADC_TIM_FREQ, &psc );
#ifdef  USE_FULL_ASSERT
  assert_param( rc == RESET );
#else
  (void)rc;
#endif /* USE_FULL_ASSERT */

  ADC_TIM->PSC = psc;

  // Период таймера 10 мс.
  ADC_TIM->ARR = ( (ADC_TIM_FREQ * ADC_RESULT_PERIOD) / 1000 ) -1;
  ADC_TIM->CR1 |= TIM_CR1_URS | TIM_CR1_ARPE;
  ADC_TIM->CR2 |= TIM_CR2_MMS_1;    // Update -> TRGO
  ADC_TIM->EGR |= TIM_EGR_UG;
  ADC_TIM->DIER = 0;
}


void adcPrmInit( sAdcData * prm ){
  prm->u16status = 0;

  prm->prm = 0;

  prm->prmPeakMax = (int32_t)(~(0UL >> 1));
  prm->prmPeakMin = (int32_t)(0UL >> 1);
}


void adcStart( void ){
  // Вкл тактирование АЦП

  // Опять включаем АЦП после калибровки
//  ADC1->CR2 |= ADC_CR2_ADON;

  adcHandle.adcOn = SET;
  // Включаем прерывание от DMA
  /* Configure NVIC to enable DMA interruptions */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
  DMA1_Channel1->CCR |= DMA_CCR1_EN;

  ADC_TIM->CR1 |= TIM_CR1_CEN;
//  ADC1->CR2 |= ADC_CR2_SWSTART;
}


/**
  * @brief  Остановка преобразований АЦП.
  * @param  None
  *
  * @retval RESET - Остановка прошла успешно
  *         SET - АЦП не остановлен: Работает "Выключение Bat_CMOS" или "Нагреватель"
  */
FlagStatus adcStop( void ){
  FlagStatus rc = SET;
  if( adcHandle.adcOn ){
    ADC_TIM->CR1 &= ~TIM_CR1_CEN;
    // ADC запущен - принудительно останавливавем
    ADC1->CR2 &= ~ADC_CR2_ADON;
    // Выключаем DMA
    DMA1_Channel1->CCR &= ~DMA_CCR1_EN;

    // Выключаем прерывание от DMA
    NVIC_DisableIRQ( DMA1_Channel1_IRQn );

    adcHandle.adcOn = RESET;
    adcHandle.adcOk = RESET;
    rc = RESET;
  }
  return rc;
}

/**
  * @brief  This function handles ADC interrupt request.
  *         It manages the ADC in case of overrun
  *         the ADC is stopped but not disabled,
  * @param  None
  * @retval None
  */
void ADC1_IRQHandler( void ){ while(1){} }

/**
  * @brief  Обработка прерывания DMA АЦП.
  * @param  None
  *
  * @retval none
  */
void DMA1_Channel1_IRQHandler( void ){
  if ((DMA1->ISR & DMA_ISR_TEIF1) != RESET){
    // Ошибка работы DMA ADC
    DMA1->IFCR |= DMA_IFCR_CTEIF4;
    // Снимаем флаг готовности данных
    adcHandle.adcOk = RESET;
  }
  if ((DMA1->ISR & DMA_ISR_TCIF1) != RESET){
    // Выставляем флаг готовности данных
    adcHandle.adcOk = SET;
    DMA2->IFCR |= DMA_IFCR_CTCIF1;
  }
  else {
    // Снимаем флаг готовности данных
    adcHandle.adcOk = RESET;
    while(1)
    {}
  }
}


void adcProcess( uintptr_t arg ){
  (void)arg;

  if( (adcHandle.adcOn == RESET) || (adcHandle.adcOk == RESET) ){
    // АЦП выключено или данные АЦП не готовы
    return;
  }

  adcHandle.adcOk = RESET;

  for( eAdcPrm i = 0; i < ADC_PRM_NUM; i++ ){
    sAdcData * pData = &(adcHandle.adcData[i]);

    switch ( i ){
      case ADC_PRM_VDD:
        pData->prm = (uint16_t)(4096000/(((uint32_t)adcHandle.adcVprm[ADC_PRM_VDD]*1000)/VREFINT_VOL));
        break;
      case ADC_PRM_PRESS:     // Давление Pa = mV
        // Вычисляем напряжение
        pData->prm = ((adcHandle.adcData[ADC_PRM_VDD].prm * adcHandle.adcVprm[i]) / adcKprm[i]) - PRESS_NUL;
        pressProc( pData->prm );
        break;
      case ADC_PRM_TERM: {
        // Вычисляем напряжение
        uint16_t rt = ((R1 * ADC_KPARAM_0) / adcHandle.adcVprm[i]) - R1;
        // Расчет температуры T1 = 1 / ((ln(R1) – ln(R2)) / B + 1 / T2), где T1 в 0.001 гр.К, T2 - в гр.К
        adcHandle.adcData[i].prm = (int32_t)(1000.0 / (((log(rt) - LN_RT25) / B25_100) + _1_298K)) - 273000;
        termProc( adcHandle.adcData[i].prm );
        break;
      }
      case ADC_PRM_ALCO:      // Просто напряжение
        // Вычисляем напряжение
        adcHandle.adcData[i].prm = ((adcHandle.adcData[ADC_PRM_VDD].prm * adcHandle.adcVprm[i]) / adcKprm[i]);
        break;
      default:
        break;
    }

    if( measDev.status.measStart ){
      pData->prmPeakMin = min( pData->prmPeakMin, pData->prm );
      pData->prmPeakMax = max( pData->prmPeakMax, pData->prm );
    }
  }

  totalProc();
}

///**
//  * @brief  Периодически процесс модуля АЦП Батареей BAT-CMOS и Heater Current.
//  * @param  iface  указатель на структуру iface (не используется)
//  *
//  * @retval None
//  */
//void adcOptClock( const struct iface * iface ){
//  (void)iface;
//}

/**
  * @brief  Запуск АЦП Батарии BAT-CMOS.
  * @param  iface  указатель на структуру iface (не используется)
  *
  * @retval None
  */
void adcMainEnable( void ){
  // Надо запустить АЦП
  adcStart();
}

/**
  * @brief  Инициализация модуля работы с АЦП Батареей BAT-CMOS и Heater Current.
  * @param  iface  указатель на структуру iface (не используется)
  *
  * @retval None
  */
void adcMainInit( void ){
  adcGpioInit();
  adcInit();
  adcDmaInit();
  adcTrigTimInit();

  for( eAdcPrm i = 0; i < ADC_PRM_NUM; i++ ){
    adcPrmInit( &(adcHandle.adcData[i]) );
  }

}


// --------------------------------------------------------------------------
