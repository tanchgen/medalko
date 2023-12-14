#ifndef _GPIO_ARCH_H
#define _GPIO_ARCH_H

#include <stddef.h>

#include "main.h"

/** Тайм-аут переходов между состояниями выводов GPIO FPGA_EN_x или FPGA_EN_x. */
#define PWR_EN_STATE_TOUT       TOUT_100
#define PWR_EN_STATE_TOUT05     TOUT_50
#define PWR_EN_STATE_TOUT1      TOUT_100
#define PWR_EN_STATE_TOUT2      TOUT_200
#define PWR_EN_STATE_TOUT3      TOUT_300
#define PWR_EN_STATE_TOUT5      TOUT_500
#define PWR_EN_STATE_TOUT10     TOUT_1000
#define PWR_EN_STATE_TOUT15     TOUT_1500

/** Тайм-аут переходного процесса при изменении состояния вывода GPIO. */
#define _GPIO_PIN_STATE_TRANSIENT_TOUT  (CONFIG_HZ)

#define PIN_TEST_EN		0

/** Тайм-аут процедуры антидребезга входов EXTI. */
#define KEY_DEBOUNCE_TOUT  50

#define ZOOM_TIM          TIM1
#define ZOOM_TIM_CLK_EN   (RCC->APB2ENR |= RCC_APB2ENR_TIM1EN)
#define ZOOM_TIM_IRQn     TIM1_UP_IRQn
#define ZOOM_TIM_IRQH     TIM1_UP_IRQHandler

#define KEY_TIM          TIM5
#define KEY_TIM_CLK_EN   RCC_APB1ENR_TIM5EN
#define KEY_TIM_IRQn     TIM5_IRQn
#define KEY_TIM_IRQH     TIM5_IRQHandler
#define KEY_AF_TIM       GPIO_AF_TIM5

//#define PWR_KEY_TIM          TIM10
//#define PWR_KEY_TIM_CLK_EN   RCC_APB2ENR_TIM10EN
//#define PWR_KEY_TIM_IRQn     TIM1_UP_TIM10_IRQn
//#define PWR_KEY_TIM_IRQH     TIM1_UP_TIM10_IRQHandler
//#define PWR_KEY_AF_TIM       GPIO_AF_TIM10
//
//#define RST_KEY_TIM          TIM11
//#define RST_KEY_TIM_CLK_EN   RCC_APB2ENR_TIM11EN
//#define RST_KEY_TIM_IRQn     TIM1_TRG_COM_TIM11_IRQn
//#define RST_KEY_TIM_IRQH     TIM1_TRG_COM_TIM11_IRQHandler
//#define RST_KEY_AF_TIM       GPIO_AF_TIM11

#define CLR_KEY_TIM          TIM9
#define CLR_KEY_TIM_CLK_EN   RCC_APB2ENR_TIM9EN
#define CLR_KEY_TIM_IRQn     TIM1_BRK_TIM9_IRQn
#define CLR_KEY_TIM_IRQH     TIM1_BRK_TIM9_IRQHandler
#define CLR_KEY_AF_TIM       GPIO_AF_TIM9

//#define FRST_KEY_TIM          TIM13
//#define FRST_KEY_TIM_CLK_EN   RCC_APB1ENR_TIM13EN
//#define FRST_KEY_TIM_IRQn     TIM8_UP_TIM13_IRQn
//#define FRST_KEY_TIM_IRQH     TIM8_UP_TIM13_IRQHandler

#define REL_PULSE_TIM            TIM2
#define REL_PULSE_TIM_CLK_EN     RCC_APB1ENR_TIM2EN
#define REL_PULSE_TIM_IRQn       TIM2_IRQn
#define REL_PULSE_TIM_IRQH       TIM2_IRQHandler

/* Перечеслитель состояний ВЫКЛ/НЕОПРЕДЕЛЕННОЕ/ВЫКЛ */
typedef enum {
  PWR_STATE_OFF,
  PWR_STATE_UNDEF,
  PWR_STATE_ON,
  PWR_STATE_ON_2,
} ePwrState;


// Состояние ВКЛЮЧЕНИЯ питания ПЛИС: состояния ПЕРЕД включением 1-го, 2-го или 3-го питания
#define IS_PWR_EN_ON_MCU_STATE(x) (x < MCUSTATE_SYS_ON)
// Состояние ВЫКЛЮЧЕНИЯ питания ПЛИС: состояния включенного 3-го, 2-го или 1-го питания
#define IS_PWR_EN_OFF_MCU_STATE(x) (x > MCUSTATE_SYS_OFF)

// Линии питания - связанные I2C-устройства
typedef enum {
  RUN_1,
  RUN_2,
  RUN_3,
  RUN_FPGA_1,
  RUN_FPGA_2,
  RUN_FPGA_3,
  RUN_I12V,
  RUN_PSU,
  RUN_NUM
} eRun;


typedef enum{
  PG_AVTT_MZU,
  PG_0v95_VP1,
  PG_AVCC_VP1,
  PG_AVTT_VP1,
  PG_0v95_VP2,
  PG_AVCC_VP2,
  PG_AVTT_VP2,
  PG_GT,
  PG_VCCSA,
  PG_PSU,
  PG_NUM
} ePgood;


typedef enum{
  PWR_EN_0,
  PWR_EN_1,
  PWR_EN_2,
  PWR_EN_3,
} ePwrEnState;

typedef enum{
  FPGA_D1,
  FPGA_D2,
  FPGA_D3,
  FPGA_D4,
  FPGA_D5,
  FPGA_D6,
  FPGA_D7,
  FPGA_D8,
  FPGA_D_NUM,
} eFpgaData;

typedef enum {
  I2C_RST_1,
  I2C_RST_2,
  I2C_RST_NUM
} eI2cRst;

typedef enum {
  SLP_S0,
  SLP_S3,
  SLP_S4,
  SLP_S5,
  SLP_A,
  SLP_SUS,
  SLP_NUM
} eIntelSlp;

typedef struct {
  sGpioPin gpioOptRst;
  FlagStatus rstState;      // Текущее состояния
  FlagStatus rstEvent;       // Индикация события
  struct timer_list rstTimer;     // Таймер сброса линии RST
  FlagStatus rstl;
} sOptRst;


#define PG_STATE_MASK   0x0000003FL
#define PG_FLAG_MASK    0x00000FC0L

typedef struct __aligned(4) {
  uint32_t pg4v0State: 1;
  uint32_t pgAvttMzuState: 1;
  uint32_t pg0v95Vp1State: 1;
  uint32_t pgAvttVp1State: 1;
  uint32_t pgAvccVp1State: 1;
  uint32_t pg0v95Vp2State: 1;
  uint32_t pgAvttVp2State: 1;
  uint32_t pgAvccVp2State: 1;
  uint32_t pgCoreState: 1;
  uint32_t pgGtState: 1;
  uint32_t pgVccsaState: 1;
  uint32_t pgPsuState: 1;
  uint32_t pg4v0Flag: 1;
  uint32_t pgAvttMzuFLag: 1;
  uint32_t pg0v95Vp1FLag: 1;
  uint32_t pgAvttVp1FLag: 1;
  uint32_t pgAvccVp1FLag: 1;
  uint32_t pg0v95Vp2FLag: 1;
  uint32_t pgAvttVp2FLag: 1;
  uint32_t pgAvccVp2FLag: 1;
  uint32_t pgCoreFlag: 1;
  uint32_t pgGtFlag: 1;
  uint32_t pgVccsaFlag: 1;
  uint32_t pgPsuFlag: 1;
} sPgFlags;


typedef struct {
  uint32_t sysOffSet: 1;
  uint32_t sysOnSet: 1;
  uint32_t fpgaOffSet: 1;
  uint32_t fpgaOnSet: 1;
  uint32_t mzuRstSet: 1;
  uint32_t mzuRstEvnt: 1;
  uint32_t mzuProgSet: 1;
  uint32_t mzuProgEvnt: 1;
  uint32_t mzuRsrv1Evnt: 1;
  uint32_t fpgaProgSet: 1;
  uint32_t fpgaProgEvnt: 1;
  uint32_t intelRstSet: 1;
  uint32_t intelRstEvnt: 1;
  uint32_t iPwrOutSet: 1;
  uint32_t mbootAddrSet: 1;
  uint32_t mbAddrSetUpdate: 1;
  uint32_t mbErr:1;
  uint32_t clrBatEvnt: 1;
  uint32_t caterr: 1;
  uint32_t logTm: 1;
} sSysFlags;


typedef union {
  struct {
#define DEVI_PROG_POS    0  // Позиция флага DONE_1
    uint32_t fpgaprog:  1;      // Флаг изменения состояния вывода
    uint32_t done:  1;       // Флаг изменения состояния вывода
    uint32_t mbid:  1;         // Флаг изменения состояния вывода
    uint32_t mbfin:  1;        // Флаг изменения состояния вывода
    uint32_t work:  1;         // Флаг изменения состояния вывода
    uint32_t rsrv1:  1;        // Флаг изменения состояния вывода
    uint32_t fpgarst:  1;       // Флаг изменения состояния вывода
    uint32_t plllock:  1;      // Флаг изменения состояния вывода
    uint32_t rsrv2:  1;        // Флаг изменения состояния вывода
    uint32_t mbaddr:  1;       // Флаг изменения состояния вывода
    uint32_t batoff:  1;      // Флаг изменения состояния вывода
    uint32_t ipwrout:  1;     // Флаг изменения состояния вывода
    uint32_t dsw:  1;         // Флаг изменения состояния вывода
    uint32_t irst:  1;        // Флаг изменения состояния вывода
    uint32_t slp0:  1;        // Флаг изменения состояния вывода
    uint32_t slp3:  1;        // Флаг изменения состояния вывода
    uint32_t slp4:  1;        // Флаг изменения состояния вывода
    uint32_t slp5:  1;        // Флаг изменения состояния вывода
    uint32_t slpsus:  1;      // Флаг изменения состояния вывода
    uint32_t caterr:  1;      // Флаг изменения состояния вывода
    uint32_t oil1:  1;        // Флаг изменения состояния вывода
    uint32_t oil2:  1;        // Флаг изменения состояния вывода
    uint32_t oil3:  1;        // Флаг изменения состояния вывода
    uint32_t rel:  1;         // Флаг изменения состояния вывода
    uint32_t rel380:  1;      // Флаг изменения состояния вывода
    uint32_t pump:  1;        // Флаг изменения состояния вывода
    uint32_t pwren1:  1;      // Флаг изменения состояния вывода
    uint32_t pwren2:  1;      // Флаг изменения состояния вывода
    uint32_t jmp1:  1;         // Флаг изменения параметра и/или порогов
    uint32_t rstkey:  1;       // Флаг изменения параметра и/или min-max
    uint32_t pwrkey:  1;       // Флаг изменения параметра и/или min-max
    uint32_t jtagsens:  1;      // Флаг изменения состояния вывода
  };
  uint32_t u32Devi;
} uGpioDeviation;


extern FlagStatus measurRun;
/** Состояние уровня запуска системы */
extern eMcuState measurState;

extern sGpioPin gpioPinAlcoRst;
extern sGpioPin gpioPinRelEn;
extern sGpioPin gpioPinRelOn;
extern sGpioPin gpioPinZoom;
extern sGpioPin gpioPinTest;

// ------------------- LEDS -----------------------
// Определено в led.c

// ----------- TIMERS ---------------------------
///** Структура дескриптора таймера таймаута изменения состояния системы при включении */
//extern struct timer_list  pwrOnToutTimer;
///** Структура дескриптора таймера таймаута изменения состояния системы при выключении */
//extern struct timer_list  pwrOffToutTimer;
///** Структура дескриптора таймера изменения состояния системы при включении*/
//extern struct timer_list  pwrOnUpdateTimer;
///** Структура дескриптора таймера изменения состояния системы при выключении */
//extern struct timer_list  pwrOffUpdateTimer;
//
//extern struct timer_list  psuOffTimer;
//
///** Структура дескриптора таймера таймаута восстановления после ошибки */
//extern struct timer_list  pwrOnCanTimer;
///** Структура дескриптора таймера таймаута восстановления после ошибки FPGA*/
//extern struct timer_list  fpgaOnCanTimer;
//
///** Структура дескриптора таймера таймаута ожидания отклика на FPGA_RST*/
//extern struct timer_list  mzuRstTestTimer;

extern struct timer_list  alcoOffTimer;
extern struct timer_list  measOnCanTimer;

// ===========================================================================================================
void zoomOn( void );
void zoomOff( void );

void gpioIrqHandler5_9( uint32_t pin );
void gpioIrqHandler10_15( uint32_t pin );
/**
  * @brief  Обработка данных интерфейса GPIO.
  *
  * @param[in]  self  дескриптор интерфейса
  *
  * @retval none
  */
void gpioClock( void );

/**
  * @brief  Запуск пульса SRST / POR.
  *
  * @param[in]  none
  *
  * @retval none
  */
void gpioRelPulse( void );

#endif /* _GPIO_ARCH_H */
