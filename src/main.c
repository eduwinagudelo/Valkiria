/**
  ******************************************************************************
  * @file    Valkiria/main.c
  * @author  Ing. Edwin Alberto Agudelo Garcia
  * @version V1.0.0
  * @date    24-June-2018
  * @brief   Main program body
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
 /**
  * Proyecto Valkiria
  *
  * Objetivo: Desarrollar el software necesario para hacer mover un dron casero.
  *
  **/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_it.h"
#include "mpu6050.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BSRR_VAL        0x0300
#define F_RECI			0X01
#define LED_AZUL		0x0100;
#define LED_VERDE		0x0200;
#define KEY				0x0080;
#define PWRP			0x0040;
#define ADDRGIR			0x68

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;
USART_InitTypeDef		USART_InitStruct;
NVIC_InitTypeDef 		NVIC_InitStruct;
EXTI_InitTypeDef		EXTI_InitStruct;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
I2C_InitTypeDef			I2C_InitStruct;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void delay (int a);
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

/**
 * Variables globales
 */
int recibidos;
char cflags;
char tflags;
char strMensaje[100];

char strPruebaAT[] = "AT+NAME?";
char strPruebaATV[] = "AT+VERSION?";
char strUart[] = "AT+UART?";
char strAdress[] = "AT+ADDR?";
char strMode[] = "AT+CMODE?";
char strState[] = "AT+STATE?";
int envio;
int espera;
int paso;
int modob;

/**
 * Rutina para la atencion de la interrupcion por puerto serie
 * */
void USART1_IRQHandler(void){
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
		}
		else{
			strMensaje[recibidos] = ctmp;
			recibidos++;
		}
	}
}

/**
 * Rutina para atender la interrupcion externa del pin 0
 * */
void EXTI0_1_IRQHandler(void){
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Do your stuff when PD0 is changed */
		tflags = 1;
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/*
 * Rutina para atender la interrupcion por el TIM_2
 * */
void TIM2_IRQHandler(void){
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		paso++;
	}
}

void enviarChar(char val){
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,val);
}

void enviarMensaje(char *strmen){
	int i = 0;
	while(strmen[i] != '\0'){
		enviarChar('a');
		enviarChar(strmen[i]);
		enviarChar('a');
		i++;
		if(i == 100)
			break;
	}
}

void enviarCharComm(char val){
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,val);
}

void enviarMensajeComm(char *strmen){
	int i = 0;
	while(strmen[i] != '\0'){
		enviarCharComm(strmen[i]);
		i++;
		if(i == 100)
			break;
	}
	enviarCharComm('\r');
	enviarCharComm('\n');
}


int main(void){
  // Declaro las variables a usar
  int16_t x,y,z;
  char strLec[32];
  MPU6050_errorstatus errorstatus;

  // Inicializacion del hardware
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Pines del puerto de comunicacion
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // Pines para el led y para la habilitacion del modulo
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // Pines para la comunicacion I2C
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Usart para recibir mensaje
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Usart para enviar a la LCD
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Para el TIM
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); // Comunicacion con el Giroscopio

  // Configuro los LED (Facil)
  /* Configure PC8 and PC9 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //  Ahora realizo la configuración de las funciones alternas
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_1);

  //  Ahora realizo la configuraci�n de las funciones alternas
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Inicializo el pin del push button
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Sigo con la inicializacion del I2C
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);

  // Inicializo el puerto B
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configuro el primer puerto serie (from Bluetooth)
  //USART_InitStruct.USART_BaudRate = 115200; // Con esto configuro la raspberry
  USART_InitStruct.USART_BaudRate = 38400;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStruct);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_ITConfig(USART1, USART_IT_WU, DISABLE);
  USART_ITConfig(USART1, USART_IT_CM, DISABLE);
  USART_ITConfig(USART1, USART_IT_EOB, DISABLE);
  USART_ITConfig(USART1, USART_IT_RTO, DISABLE);
  USART_ITConfig(USART1, USART_IT_CTS, DISABLE);
  USART_ITConfig(USART1, USART_IT_LBD, DISABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  USART_ITConfig(USART1, USART_IT_ERR, DISABLE);
  USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
  USART_ITConfig(USART1, USART_IT_PE, DISABLE);
  USART_Cmd(USART1, ENABLE);

  // Configuro el segundo puerto serie (To LCD)
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStruct);
  USART_Cmd(USART2, ENABLE);

  // Configuro el I2C
  //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2 , ENABLE);
  //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2 , DISABLE);
  I2C_InitStruct.I2C_Timing = 0x0010010B;
  I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStruct.I2C_DigitalFilter = 0x0;
  I2C_InitStruct.I2C_OwnAddress1 = 0x0;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C2, &I2C_InitStruct);
  I2C_Cmd(I2C2, ENABLE);

  // Configuro el TIM_2 para
  TIM_TimeBaseStructure.TIM_Prescaler = 47999;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // Habilito la interrupcion por fuente externa
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitStruct.EXTI_Line = EXTI_Line0;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&  EXTI_InitStruct);

  // Configuro la interrupcion del puerto serie
  NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  // Configuro la interrupcion del pshbtn
  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  // Configuro la interrupcion del TIM2
  NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  // Inicializo variables
  cflags = 0x0;
  recibidos = 0;
  strMensaje[recibidos] = '\0';
  envio = 0;
  tflags = 0;
  paso = 0;
  modob = 0;
  // Primer Bucle, es para conocer la configuracion del modulo
  GPIOC->ODR = 0x0;
  delay(5000);
  GPIOC->BSRR = KEY;
  delay(5000);
  GPIOC->BSRR |= PWRP;
  delay(200000);
  enviarChar('b');
  enviarChar('b');
  enviarChar('b');
  enviarMensaje("Hola Mundo");
  //delay(5000);
  delay(3000);
  //enviarMensajeComm(strState);
  //TIM_Cmd(TIM2,ENABLE);
  do{
	  switch(paso){
	  case 1:
		  enviarMensajeComm(strPruebaAT);
		  break;
		  case 2:
			  enviarMensajeComm(strPruebaATV);
			  break;
		  case 3:
			  enviarMensajeComm(strUart);
			  break;
		  case 4:
			  enviarMensajeComm(strAdress);
			  break;
		  case 5:
	  		 enviarMensajeComm(strMode);
	  		 break;
	  	  case 6:
	  		 enviarMensajeComm(strState);
	  		 break;
	  	  default:
	  		 enviarMensajeComm(strState);
	  		 break;
	  }
	  delay(3000000);
	  if(cflags != 0){
		  cflags = 0;
		  GPIOC->BSRR |= LED_AZUL;
		  // Borro la LCD
		  enviarChar('b');
		  enviarChar('b');
		  enviarChar('b');
		  enviarMensaje(strMensaje);
		  GPIOC->BRR |= LED_AZUL;
		  paso++;
	  }
  }while(paso < 6);
  // Ahora lo entro en modo de operacion normal
  GPIOC->BRR |= PWRP;
  delay(5000);
  GPIOC->BRR = KEY;
  delay(5000);
  enviarChar('b');
  enviarChar('b');
  enviarChar('b');
  enviarMensaje("Inic Normal Mode");

  // Inicio el pruerto
  USART_DeInit(USART1);
  modob = 1;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Usart para recibir mensaje
  // Configuro el primer puerto serie (from Bluetooth)
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStruct);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_Cmd(USART1, ENABLE);

  GPIOC->BSRR |= PWRP;
  delay(200000);

  // Ahora voy a leer los registros
  unsigned char ctemp;
  errorstatus = MPU6050_Initialization();
  if(errorstatus != 0){
	  enviarMensaje("Error Iniciando el modulo");
  }
  errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_CONFIG, &ctemp, 1);
  if(errorstatus != 0){
	  enviarMensaje("Nos fuimos de error");
	  //return errorstatus;
  }
  enviarChar('b');
  enviarChar('b');
  enviarChar('b');
  enviarChar('a');
  enviarChar('-');
  enviarChar('a');
  enviarChar('a');
  enviarChar(ctemp);
  enviarChar('a');


  while (1) {
	  // Valido si se recibio algo
	  if ( cflags != 0) {
		  cflags = 0;
		  //GPIOC->BSRR = BSRR_VAL;
		  GPIOC->BSRR = LED_AZUL;
		  // Borro la LCD
		  enviarChar('b');
		  enviarChar('b');
		  enviarChar('b');
		  enviarMensaje(strMensaje);
		  GPIOC->BRR = LED_AZUL;
	  }
	  if(tflags != 0){
		  tflags = 0;
		  errorstatus = MPU6050_Get_Accel_Data_Raw(&x,&y,&z);
		  if(errorstatus == MPU6050_NO_ERROR){
			  ctemp = (x & 0xff00)>>8;
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[0] = ctemp;
			  ctemp = (x & 0xff00)>>8;
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[1] = ctemp;
			  ctemp = (x & 0x00ff);
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[2] = ctemp;
			  ctemp = (x & 0x00ff);
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[3] = ctemp;
			  ctemp = (y & 0xff00)>>8;
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[4] = ctemp;
			  ctemp = (y & 0xff00)>>8;
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[5] = ctemp;
			  ctemp = (y & 0x00ff);
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[6] = ctemp;
			  ctemp = (y & 0x00ff);
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[7] = ctemp;
			  ctemp = (z & 0xff00)>>8;
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[8] = ctemp;
			  ctemp = (z & 0xff00)>>8;
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[9] = ctemp;
			  ctemp = (z & 0x00ff);
			  ctemp = ctemp & 0xf0;
			  ctemp = ctemp >> 4;
			  ctemp += 48;
			  strLec[10] = ctemp;
			  ctemp = (z & 0x00ff);
			  ctemp = ctemp & 0x0f;
			  ctemp += 48;
			  strLec[11] = ctemp;
			  strLec[12] = '\0';
		  }
		  else{
			  strLec[0] = 'e';
			  strLec[1] = 'r';
			  strLec[2] = 'r';
			  strLec[3] = '\0';
		  }
		  //sprintf(strLec,"x:%d;y:%d;z:%d",x,y,z);
		  enviarChar('b');
		  enviarChar('b');
		  enviarChar('b');
		  enviarMensaje(strLec);
	  }
	  delay(5);
  }

}

void delay (int a)
{
	volatile int i,j;

	for (i=0 ; i < a ; i++)
	{
		j++;
	}

	return;
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

