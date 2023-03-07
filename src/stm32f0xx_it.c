/**
  ******************************************************************************
  * @file    Valkiria/stm32f0xx_it.c
  * @author  Ing. Edwin A. Agudelo G.
  * @version V1.0.0
  * @date    23-Junio-2018
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles the RX USART1.
  * @param  None
  * @retval None
  */
/*void USART1_IRQHandler(void){
	char ctmp;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET){ // Se recibe un caracter
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		if(recibidos == 100){ // Pruebo si llegue al limite, para reiniciar el contador
			recibidos = 0;
		}
		if(recibidos == 0){
			GPIOC->BSRR = LED_VERDE;
		}
		ctmp = USART_ReceiveData(USART1) & 0xff;
		if(ctmp == 0xa){ // Si llega el caracte 0xa, es el fin del mensaje
			// Ahgora debo validar si los 2 ultimos caracteres son OK
			if(recibidos > 1){
				if(modob == 0){
					if(strMensaje[recibidos - 2] == 'K' && strMensaje[recibidos - 1] == '\r'){
						cflags |= F_RECI;
						strMensaje[recibidos] = '\0';
						recibidos = 0;
						GPIOC->BRR = LED_VERDE;
					}
					else{
						strMensaje[recibidos] = ctmp;
						recibidos++;
					}
				}
				else{
					if(strMensaje[recibidos - 1] == '\r'){
						cflags |= F_RECI;
						strMensaje[recibidos] = '\0';
						recibidos = 0;
						GPIOC->BRR = LED_VERDE;
					}
					else{
						strMensaje[recibidos] = ctmp;
						recibidos++;
					}
				}
			}
			cflags |= F_RECI;
			strMensaje[recibidos] = '\0';
			recibidos = 0;
			GPIOC->BRR = LED_VERDE;
		}
		else{
			strMensaje[recibidos] = ctmp;
			recibidos++;
		}
	}
}*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
