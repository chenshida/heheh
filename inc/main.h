/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NB_TXD_Pin GPIO_PIN_0
#define NB_TXD_GPIO_Port GPIOC
#define NB_RXD_Pin GPIO_PIN_1
#define NB_RXD_GPIO_Port GPIOC
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOA
#define ADC1_VREF_Pin GPIO_PIN_1
#define ADC1_VREF_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_4
#define DIO3_GPIO_Port GPIOC
#define DIO2_Pin GPIO_PIN_5
#define DIO2_GPIO_Port GPIOC
#define DIO2_EXTI_IRQn EXTI4_15_IRQn
#define DIO1_Pin GPIO_PIN_0
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI0_1_IRQn
#define DIO0_Pin GPIO_PIN_1
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI0_1_IRQn
#define LORA_RST_Pin GPIO_PIN_2
#define LORA_RST_GPIO_Port GPIOB
#define POWER_MOD_Pin GPIO_PIN_7
#define POWER_MOD_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define D18b20_Pin GPIO_PIN_12
#define D18b20_GPIO_Port GPIOA
#define NB_RESET_Pin GPIO_PIN_5
#define NB_RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define REPORT_PERIOD_SAVE_ADDR ADDR_FLASH_PAGE_800
#define MASTER_ADDRESS_SAVE_ADDR ADDR_FLASH_PAGE_801
#define DEFAULT_MASTER_ADDRESS 0x80000001
#define DEFAULT_REPORT_PERIOD 60
#define SLAVER_ADDRESS_SAVE_ADDR ADDR_FLASH_PAGE_802

typedef struct
{
	char net_time[30];
	char net_csq[5];
	char net_cfun[5];
	char net_ip[20];
	char net_cscon[5];
	char net_cereg[5];
	char net_cgatt[5];
	char net_npsmr[5];
	char net_nsocr[10];
	char net_nsocl[10];
	char net_nsost[30];
	char net_nsonmi[10];
	char net_nsorf[30];
//	char net_nmgr[30];
	char net_nmgr[100];
	char net_Signal_power[10];
	char net_Total_power[10];
	char net_TX_power[10];
	char net_TX_time[10];
	char net_RX_time[10];
	char net_SCell_ID[10];
	char net_ECL[10];
	char net_SNR[10];
	char net_EARFCN[10];
	char net_PCI[10];
	char net_RSRQ[10];
}msg_sys_type;

extern msg_sys_type bc95_net_data;

typedef struct
{
	char Latitude[8];
	char Longitude[9];	
}msg_for_GPS;
extern msg_for_GPS GPS_send;

typedef struct {
	uint8_t report_stat;
	uint32_t report_count;
	uint32_t report_count_max;
} report_config;
extern report_config report_cfg;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
