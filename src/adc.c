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

/** Структура дескриптора модуля АЦП: Бат. CMOS (Bat_CMOS, vbat) */
sAdcHandle adcHandle = {
  .adcOn = RESET,
  .adcOk = RESET,
  .clrBatFlag = (volatile FlagStatus *) BKPSRAM_BASE,
};


/** Структура дескриптора таймера включения-выключения Bat_CMOS. */
struct timer_list  batClrTimer;
/** Структура дескриптора таймера включения-выключения Bat_CMOS. */
struct timer_list  adcTimer;
/** Структура дескриптора таймера таймаута сброса мин/макс АЦП */
struct timer_list  adcPeakTimer;

/** Указатель на обработчик таймера включения-выключения батареи Bat_CMOS. */
void (*batClrHandler)(sGpioPin *);


#define ADC_KPARAM_0    (4096L)   // Делитель для VBAT: 10/20

const uint16_t adcKparam[ADC_PARAM_NUM] = {
    ADC_KPARAM_0,
};


const eParamId adcDeviPrmid[ADC_PARAM_NUM] = {
    PRM_VIN,                // Флаг изменения параметра и/или min-max
};

const ePrmType adcDeviPrmtype[ADC_PARAM_NUM] = {
    PRM_TYPE_PEAK,                // Флаг изменения параметра и/или min-max
};

const ePwrLine adcPwrLine[ADC_PARAM_NUM] = {
    PWR_LINE_VBAT,                // Флаг изменения параметра и/или min-max
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


inline void adcDataReset( sAdcHandle * adc, eAdcParam num ){
  timerModArg( &adcPeakTimer, TOUT_100, num );
  adc->adcOk = RESET;
  adc->adcData[num].status.u8stat = 0;
}

void adcPeakTout( uintptr_t arg ){
  adcPeakReset( &(adcHandle), (eAdcParam)arg  );
}

/**
  * @brief  Обновление данных телеметрии АЦП.
  *
  * @param[out] to    буфер данных телеметрии АЦП
  *
  * @retval none
  */
void refreshAdcParamTm(eAdcParam adcopt, sAdcParamTm *to){
  sParamData  *adcdata = &(adcHandle.adcData[adcopt]);


// XXX: Привести в соответствие с форматом регистров телеметрии
// ----------- 3.3V_OPT ------------------------
  to->paramNok = adcHandle.paramStatus[adcopt].paramNok;
  to->paramPeakNok = adcHandle.paramStatus[adcopt].paramNok;
  to->paramLowLim = adcdata->status.onLowAlrm;
  to->paramHiLim = adcdata->status.onHiAlrm;

  if ( to->paramNok == RESET ) {
    to->param  = pmbus_data_regular_to_linear_11( adcdata->now.param );
    to->paramPeakMin  = pmbus_data_regular_to_linear_11( adcdata->now.peakMin );
    to->paramPeakMax = pmbus_data_regular_to_linear_11( adcdata->now.peakMax );
  }
  else{
    to->param  = 0x7bff;
    to->paramPeakMin = 0x7bff;
    to->paramPeakMax = 0x7bff;
  }
  to->paramLowAlrmLimit  = pmbus_data_regular_to_linear_11( adcdata->now.limitOnLow );
  to->paramHiAlrmLimit  = pmbus_data_regular_to_linear_11( adcdata->now.limitOnHi );

#if DEBUG_DATA
    trace_printf("ADC param %d\n", adcdata->paramAdc);
#endif
}

size_t refreshAdcTm( sAdcParamTm * padctm ){
  (void) padctm;
  size_t sz = 0;


  for( eAdcParam i = 0; i < ADC_PARAM_NUM; i++, padctm++ ){
    refreshAdcParamTm( i, padctm );
    sz += sizeof( sAdcParamTm );
  }

  return sz;
}


// Batary volt sensor init function
void adcInit(void){
// --------- ADC ------------------------------------

  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;// | RCC_APB2ENR_SYSCFGEN;

  ADC1->CR2 &= ~ADC_CR2_ADON;

  // Режим сканирования
  ADC1->CR1 = ADC_CR1_SCAN;
  // Режим перебора каналов и DMA
  ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;
  // Выбор триггера для преобразования
  // TIM8_TRGO
  ADC1->CR2 |= ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1;
  // Rising edge
  ADC1->CR2 |= ADC_CR2_EXTEN_0;
  // Конфигурация тактирования АЦП - PCLK/4 (15MHz)
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_TSVREFE;

  // Меряем 1 каналов: канал 0 (ADC_VIN) и Vref(Ch17)
  ADC1->SQR1 = (ADC_CH_NUM - 1) << 20;
  ADC1->SQR3 = (VBAT_CH << 0) | (17 << 5);

  // Длительность сэмпла = 84 ADCCLK
  ADC1->SMPR1 = ADC_SMPR1_SMP17;
  ADC1->SMPR2 = ADC_SMPR2_SMP0;

// ---------- DMA ADC -------------------------------
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

//  DMA_StructInit(&dma_init_struct);
//
//  dma_init_struct.DMA_Channel = DMA_Channel_0;
//  dma_init_struct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
//  dma_init_struct.DMA_Memory0BaseAddr     = (uint32_t)&adcHandle.adcVrtcbat;
//  dma_init_struct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
//  dma_init_struct.DMA_BufferSize         = 2;
//  dma_init_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//  dma_init_struct.DMA_Mode               = DMA_Mode_Circular;
//  dma_init_struct.DMA_Priority           = DMA_Priority_Low;
//
//  DMA_Init(DMA2_Stream4, &dma_init_struct);
//

  DMA2_Stream4->PAR = (uint32_t) (&(ADC1->DR));
  // Mem_inc, Mem 16bit, Periph 16bit, Circ mode, TC interrupt
  DMA2_Stream4->CR = DMA_Channel_0 | DMA_MemoryDataSize_HalfWord | DMA_PeripheralDataSize_HalfWord
                      | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  DMA_ClearITPendingBit( DMA2_Stream4, DMA_IT_TCIF0 );

  NVIC_SetPriority( DMA2_Stream4_IRQn, 2 );

}

/**
  * @brief  Настройка вывода GPIO для АЦП 3.3V_OPT1-5
  * @param  None
  *
  * @retval None
  */
inline void adcGpioInit( void ){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  GPIOB->MODER |= GPIO_MODER_MODER0;      // VBat
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

  RCC->APB2ENR |= ADC_TIM_CLK_EN;

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


void adcStart( void ){
  // Вкл тактирование АЦП
//  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//  ADC->CCR |= ADC_CCR_VBATEN;

  // Опять включаем АЦП после калибровки
  ADC1->CR2 |= ADC_CR2_ADON;

  DMA2_Stream4->M0AR = (uint32_t)(&(adcHandle.adcVparam[0]));
  DMA2_Stream4->NDTR = ADC_CH_NUM ;

  adcHandle.adcOn = SET;
  // Включаем прерывание от DMA
  NVIC_EnableIRQ( DMA2_Stream4_IRQn );
  DMA2_Stream4->CR |= DMA_SxCR_EN;
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
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;

    // Выключаем прерывание от DMA
    NVIC_DisableIRQ( DMA2_Stream4_IRQn );

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
void ADC1_IRQHandler( void ){
  if ((ADC1->SR & ADC_SR_OVR) != 0) {
    ADC1->SR &= ~ADC_SR_OVR; /* Clears the pending bit */
  }
}

/**
  * @brief  Обработка прерывания DMA АЦП.
  * @param  None
  *
  * @retval none
  */
void DMA2_Stream4_IRQHandler( void ){
  if ((DMA2->HISR & DMA_HISR_TEIF4) != RESET){
    // Ошибка работы DMA ADC
    DMA2->HIFCR |= DMA_HIFCR_CTEIF4;
    // Снимаем флаг готовности данных
    adcHandle.adcOk = RESET;
  }
  if ((DMA2->HISR & DMA_HISR_TCIF4) != RESET){
    // Выставляем флаг готовности данных
    adcHandle.adcOk = SET;
    DMA2->HIFCR |= DMA_HIFCR_CTCIF4;
  }
  else {
    // Снимаем флаг готовности данных
    adcHandle.adcOk = RESET;
    errHandler( ERR_UNKNOWN );
    assert_param( 0 );
  }
}

bool updateAdcOptTm( eAdcParam adcparam, size_t regaddr, uint16_t regdata ){
  sParamData * adcdata = &(adcHandle.adcData[adcparam]);

  switch (regaddr) {
    case 0:
      adcdata->status.onLowAlrm = RESET;
      adcdata->status.onHiAlrm = RESET;
      break;
    case offsetof(sAdcParamTm, paramPeakMin):
       adcdata->now.peakMin = NAN;
      break;
    case offsetof(sAdcParamTm, paramPeakMax):
           adcdata->now.peakMax = NAN;
      break;
    case offsetof(sAdcParamTm, paramLowAlrmLimit):
      adcdata->now.limitOnLow = fromLinear11( (uint32_t)regdata, 1000 );
      break;
    case offsetof(sAdcParamTm, paramHiAlrmLimit):
    adcdata->now.limitOnHi = fromLinear11( (uint32_t)regdata, 1000 );
      break;
    default:
      break;
  }

  return 0;
}

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
bool arch_updateAdcTm( size_t regaddr, uint16_t regdata ){
  eAdcParam i;

  for( i = 0; regaddr >= sizeof(sAdcParamTm); i++){
    regaddr -= sizeof(sAdcParamTm);
  }

  return updateAdcOptTm( i, regaddr, regdata );
}


void batClrTout( uintptr_t arg ){
  void (**handler)(sGpioPin *) = (void (**)(sGpioPin *))(arg);

  (*handler)( &gpioPinBatOff );
}

/**
  * @brief  Обработка таймаута сброса напряжения Bat_CMOS при выключении ее
  *         или раста напряжения при вклюбчении ее обратно.
  *
  * @param  arg тип: uintptr_t  указатель на структуру gpio_pin
  *
  * @retval None
  */
void batOffTout( sGpioPin * pin ){
  (void)pin;

  // Батарею выключили - напряжение не падает
//  errHandler( ERR_BAT_DOWN );
  adcHandle.batOffOk = RESET;
  adcHandle.batOff = RESET;
}

/**
  * @brief  Обработка таймаута роста напряжения Bat_CMOS при включении после
  *         сброса напряжения при выключении батареи
  *
  * @param  pin указатель на структуру gpio_pin
  *
  * @retval None
  */
void batOnTout( sGpioPin * pin ){
  (void)pin;

  // Батарею выключили - напряжение не падает
//  errHandler( ERR_BAT_UP );
  adcHandle.vbatOk = RESET;
  adcHandle.batOff = RESET;
}

/**
  * @brief  Обработка таймаута отсрочки включения батареи Bat_CMOS
  *
  * @param  pin указатель на структуру gpio_pin выключателя батареи CMOS
  *
  * @retval None
  */
void batOnToutTout( sGpioPin * pin ){

  // Включаем батарею Bat_CMOS
  gpioPinResetNow( pin );
  gpioPinResetNow( &gpioPinBatDisch );
  if( adcHandle.adcOn == RESET ){
    // Надо запустить АЦП
    adcStart();
  }

  batClrHandler = batOnTout;
  timerMod( &batClrTimer, BAT_ON_TOUT);
}

/**
  * @brief  Старт выключения батареи CMOS
  *
  * @param  pin Указатель на структуру gpio_pin выключателя батареи CMOS
  *
  * @retval None
  */
void batOffStart( sGpioPin * pinBatOff, sGpioPin * pinBatDisch ){
  // Выключаем батарею
  gpioPinSetNow( pinBatOff );
  // Включаем разрядник
  gpioPinSetNow( pinBatDisch );
  adcHandle.batOffOk = RESET;
  adcHandle.vbatOk = RESET;
  adcHandle.batOff = SET;

  // Включаем таймер таймаута батареи
  batClrHandler = batOffTout;
  timerMod( &batClrTimer, BAT_OFF_TOUT );
  if( adcHandle.adcOn == RESET ){
    // Надо запустить АЦП
    adcStart();
  }
}

void batOffProcess( eAdcParam idx ){
  uint16_t vbat = adcHandle.adcData[idx].now.param * 1000;

  // Идет процесс сброса батареи Bat_CMOS

  adcHandle.paramStatus[idx].paramNok = SET;
  adcHandle.paramStatus[idx].paramPeakNok = SET;

  if( (gpioPinBatOff.state == Bit_RESET) && (vbat > VBAT_ON_VOLT) ){
    /* Идет процесс набора напряжения после сброса батареи Bat_CMOS
     * и Напряжение достигло рабочего значения
     */
    adcHandle.vbatOk = SET;
    adcHandle.batOff = RESET;
    // Останавливаем таймер таймаута
    timerDel(&batClrTimer);
  }
  else if( (adcHandle.batOffOk == RESET)
            && (gpioPinBatOff.state == Bit_SET)
            && (vbat < VBAT_OFF_VOLT) )
  {
    /* Идет процесс сброса батареи Bat_CMOS
     * и Напряжение достигло значения выключеного - запускаем таймер на отсрочку включения батареи обратно
     */
    adcHandle.batOffOk = SET;
//      // АЦП не нужен - останавливаем
    ledWink( LED_TOGGLE_TOUT );
    batClrHandler = batOnToutTout;
    timerMod(&batClrTimer, BAT_ON_TOUT_TOUT);
  }
  else {
    adcHandle.vbatOk = RESET;
  }
}

void adcProcess( uintptr_t arg ){
  (void)arg;

  if( (adcHandle.adcOn == RESET) || (adcHandle.adcOk == RESET) ){
    // АЦП выключено или данные АЦП не готовы
//    for( eAdcParam i = ADC_PARAM_1; i < ADC_PARAM_NUM; i++ ){
//      adcHandle.adcData[i].paramAdcNok = RESET;
//    }
    return;
  }

  adcHandle.adcOk = RESET;

  adcHandle.vdd = (uint16_t)(4096000/(((uint32_t)adcHandle.adcVref*1000)/1210));

  for( eAdcParam i = 0; i < ADC_PARAM_NUM; i++ ){

    // Вычисляем напряжение
    adcHandle.adcData[i].now.param = (float)((adcHandle.vdd * adcHandle.adcVparam[i]) / adcKparam[i]) / 1000.0;

    if( adcHandle.batOff && (i == ADC_PARAM_VBAT) ){
      // Процесс сброса CMOS
      batOffProcess( ADC_PARAM_VBAT );
    }
    else {
      // НЕ процесс сброса батареи
      tmParamProcF( &adcHandle.adcData[i], RESET, RESET );
      adcHandle.paramStatus[i].paramNok = RESET;
      adcHandle.paramStatus[i].paramPeakNok = RESET;

      if( (adcHandle.adcData[i].now.param > adcHandle.adcData[i].now.limitOnHi)
          && (adcHandle.adcData[i].status.onHiAlrm == RESET) ){
        uint16_t data;

        adcHandle.adcData[i].status.onHiAlrm = SET;

        // Логируем ошибку ADC
        data =  pmbus_data_regular_to_linear_11( adcHandle.adcData[i].now.param );
        logger( DEVID_ADC, i, LOG_ERR_PRM_ON_HI, PRM_VIN, data );

      }
      else if( (adcHandle.adcData[i].now.param < adcHandle.adcData[i].now.limitOnLow)
               && (adcHandle.adcData[i].status.onLowAlrm == RESET) ){
        uint16_t data;

        adcHandle.adcData[i].status.onLowAlrm = SET;

        // Логируем ошибку ADC
        data =  pmbus_data_regular_to_linear_11( adcHandle.adcData[i].now.param );
        logger( DEVID_ADC, i, LOG_ERR_PRM_ON_LOW, PRM_VIN, data );
      }
    }
  }

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
  adcTrigTimInit();

  adcHandle.adcData[ADC_PARAM_VBAT].now.limitOnLow = 2.7;
  adcHandle.adcData[ADC_PARAM_VBAT].now.limitOnHi = 3.2;
  for( eAdcParam i = 0; i < ADC_PARAM_NUM; i++ ){
    tmPrmInit( &(adcHandle.adcData[i]), RESET, SET );
  }

  timerSetup(&batClrTimer, batClrTout, (uintptr_t)&batClrHandler);
  timerSetup( &adcPeakTimer, adcPeakTout, (uintptr_t)ADC_PARAM_VBAT);

}


// --------------------------------------------------------------------------
