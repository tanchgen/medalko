/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include "stm32f10x_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void){
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void){
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void){
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void){
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void){
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void){
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void){
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void){
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void WWDG_IRQHandler(void){ while(1){} };
void PVD_IRQHandler(void){ while(1){} };
void TAMPER_IRQHandler(void){ while(1){} };
void RTC_IRQHandler(void){ while(1){} };
void FLASH_IRQHandler(void){ while(1){} };
void RCC_IRQHandler(void){ while(1){} };
void EXTI0_IRQHandler(void){ while(1){} };
void EXTI1_IRQHandler(void){ while(1){} };
void EXTI2_IRQHandler(void){ while(1){} };
void EXTI3_IRQHandler(void){ while(1){} };
void EXTI4_IRQHandler(void){ while(1){} };
//void DMA1_Channel1_IRQHandler(void){ while(1){} };
void DMA1_Channel2_IRQHandler(void){ while(1){} };
void DMA1_Channel3_IRQHandler(void){ while(1){} };
void DMA1_Channel4_IRQHandler(void){ while(1){} };
void DMA1_Channel5_IRQHandler(void){ while(1){} };
void DMA1_Channel6_IRQHandler(void){ while(1){} };
void DMA1_Channel7_IRQHandler(void){ while(1){} };
void ADC1_2_IRQHandler(void){ while(1){} };
void USB_HP_CAN1_TX_IRQHandler(void){ while(1){} };
//void USB_LP_CAN1_RX0_IRQHandler(void){ while(1){} };
void CAN1_RX1_IRQHandler(void){ while(1){} };
void CAN1_SCE_IRQHandler(void){ while(1){} };
void EXTI9_5_IRQHandler(void){ while(1){} };

void TIM1_BRK_IRQHandler(void){ while(1){} };
void TIM1_UP_IRQHandler(void){ while(1){} };
void TIM1_TRG_COM_IRQHandler(void){ while(1){} };

void TIM1_CC_IRQHandler(void){ while(1){} };
//void TIM2_IRQHandler(void){ while(1){} };
//void TIM3_IRQHandler(void){ while(1){} };
void TIM4_IRQHandler(void){ while(1){} };
//void I2C1_EV_IRQHandler(void){ while(1){} };
//void I2C1_ER_IRQHandler(void){ while(1){} };
void I2C2_EV_IRQHandler(void){ while(1){} };
void I2C2_ER_IRQHandler(void){ while(1){} };
void SPI1_IRQHandler(void){ while(1){} };
void SPI2_IRQHandler(void){ while(1){} };
void USART1_IRQHandler(void){ while(1){} };
void USART2_IRQHandler(void){ while(1){} };
void USART3_IRQHandler(void){ while(1){} };
void EXTI15_10_IRQHandler(void){ while(1){} };
void RTCAlarm_IRQHandler(void){ while(1){} };
void USBWakeUp_IRQHandler(void){ while(1){} };

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
