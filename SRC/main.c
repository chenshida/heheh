/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
	
	//master
	//if communication with l4 , should change Fre,protocol_length
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neul_bc95.h"
#include "los_dev_st_uart.h"
#include "string.h"
#include <stdio.h>
#include <array_operation.h>
#include "protocol.h"
// #include "xieyi.h"
#include "decode.h"
#include "nb_send_machine.h"
#include "nb_init.h"

#include "ds18b20.h"
#include "math.h"
#include "SX127X_Driver.h"
#include "common_define.h"
// #include "common_function.h"
#include "lora_master.h"
#include "flash_save_data.h"
#include "uart_change_addr.h"
#define SIGLEADC 0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
msg_sys_type bc95_net_data;
msg_for_GPS GPS_send;
report_config report_cfg;
float Longitude;
float Latitude;
char nbiot_uart[500];

int socket_udp;
char pull_data[500];
int ret;

uint8_t uart1_recv_buf[20] = {0};
uint8_t uart1_recv_max_length = 14; //"S\r\nA\r\n00000001"
uint16_t adcvalue[2];


float a = 0;
//1- CT, 0-baterry
int8_t powermode=1;
//static int decode_ret;


//===================================????===================================================
enum DemoInternalStates
{
    APP_RNG = 0, // nothing to do (or wait a radio interrupt)
    RX_DONE,
    TX_DONE,
    TX_ING,
    APP_IDLE,
};
//long SysTick = 0;
uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
uint8_t WakeAddr[8] = {5, 6, 7, 8, 9, 10, 11, 7};
uint8_t RXbuffer[10];
#if l4
uint32_t Fre[6] = {471300000, 494600000, 510000000, 868000000, 915000000,0};
#endif
uint32_t Fre[6] = {433000000, 438000000, 443000000, 448000000, 453000000, 458000000};
uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
uint16_t E_Cnt = 0;
uint8_t SF;

int16_t SNR = 0;
int16_t RSSI = 0;

uint8_t communication_states;
void SystemClock_Config(void);
void Error_Handler(void);

//[����][ÿ��һ����ĳ���]
uint32_t slaveTable[MAX_SLAVE_NUM];
uint8_t Rxbuf[Max_Length];

uint8_t  Flag_timer_exceed=0;
uint8_t respon2authFrame_handle=0,respon2ask4data_handle=0;
uint8_t hand_change_fre_handle=0,platform_change_fre_handle=0,hand_add_del_handle=0,
platform_add_del_handle=0,hand_self_test_handle=0,platform_self_test_handle=0,
selftest_slaver_respone_handle=0;
uint8_t need_reauth=0;
uint8_t Flag_timer_2s_exceed=0,Flag_timer_5min_exceed=0;
//uint32_t default_MasterNum=0x00000000;
uint32_t default_MasterNum=0x80000001;
uint8_t slave_num = 1;
uint32_t MasterNum = 0x80000001;
uint8_t lora_blink_time=0,count_1s=0;
float temp_18b20=0;
void __start_timer(uint32_t second)
{
    //HAL_TIM_Base_Stop_IT(&htim3);
    MX_TIM3_Init_Ms(second*1000);
	  // __HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_UPDATE);
  
    HAL_TIM_Base_Start_IT(&htim3);//��ʱ2s����`
}
void __stop_timer()
{
	  __HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(&htim3);
		Flag_timer_exceed = 0; 
}

void NBLEDStateControl()
{
	if(nb_work_state_led.led_now_state == 0)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		nb_work_state_led.led_now_state = 1;
//		printf("NB is connected, LED switch on\n");
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		nb_work_state_led.led_now_state = 0;
//		printf("NB is connected, LED switch off\n");
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 //if(htim->Instance==TIM3)
     Flag_timer_exceed = 1;
     if (htim->Instance==TIM2 && report_cfg.report_stat == 0)
     {
         report_cfg.report_count += 1;
         if (report_cfg.report_count >= report_cfg.report_count_max)
         {
             report_cfg.report_stat = 1;
             report_cfg.report_count = 0;
			 HAL_TIM_Base_Stop_IT(&htim2);
         }
     }
	 if (htim->Instance==TIM6)
	 {
		 if(nb_work_state_led.connect_state == 0)
		 {
			 printf("NB is not connected\n");
			 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //�?直亮�?
		 }
		 else
		 {
			 nb_work_state_led.count_now += 1;
			 if(nb_work_state_led.count_now >= nb_work_state_led.count_max)
			 {
				 NBLEDStateControl();
				 nb_work_state_led.count_now = 0;
			 }
		 }

     //add ezio 190403 
     count_1s++;
     if(lora_blink_time!=0 && count_1s == lora_blink_time)
     {
        count_1s=0;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (1-HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)));
      }
	 }
}

uint8_t validMasterAddr(uint32_t master_address)
{
    if(master_address < 0x80000000 || master_address > 0x80ffffff)
    {
        return 0;
    }
    return 1;
}

void changeDeviceAddr()
{
    if(uart_change.dev_type == 'S')
    {
        if(uart_change.dev_opt == 'A')
        {
            if(addItem(uart_change.dev_addr, slaveTable, MAX_SLAVE_NUM) == 0)
            {
                printf("could not add an exit addr!\n");
            }
            else
            {
                printf("add successful!\n");
                writeFlashWordArray(SLAVER_ADDRESS_SAVE_ADDR, slaveTable, MAX_SLAVE_NUM);
            }


        }
        else if(uart_change.dev_opt == 'D')
        {
            if(delItem(uart_change.dev_addr, slaveTable, MAX_SLAVE_NUM) == 0)
            {
                printf("could not del a null addr!\n");
            }
            else
            {
                printf("del successful!\n");
                writeFlashWordArray(SLAVER_ADDRESS_SAVE_ADDR, slaveTable, MAX_SLAVE_NUM);
            }

        }
        else if(uart_change.dev_opt == 'S')
        {
            //change slaver addr
            //save to flash
        }
        else
        {
            printf("Error Operation, It Must A(add slaver addr) or D(delete slaver addr)\n");
        }
    }
    else if(uart_change.dev_type == 'M')
    {
        if(validMasterAddr(uart_change.dev_addr))
        {
            MasterNum = uart_change.dev_addr;
            printf("change master addr successful!\n");
            writeFlashWord(MASTER_ADDRESS_SAVE_ADDR, MasterNum);
        }
        else
        {
            printf("invalid master address, it must 0x80000000-0x80ffffff\n");
        }
    }
    else
    {
        printf("Command Error!\n");
        printf("Add Slaver Addr: S\r\nA\r\n00000001\n");
        printf("Del Slaver Addr: S\r\nD\r\n00000001\n");
        printf("Change Master Addr: M\r\nM\r\n80000001\n");
    }
    memset(uart1_recv_buf, 0, 20);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
//		while(HAL_UART_Receive_IT(&huart1, &receive_char, 1) != HAL_OK);
        while(HAL_UART_Receive_IT(&huart1, uart1_recv_buf, uart1_recv_max_length) != HAL_OK);
        uartChangeAddrDecode(uart1_recv_buf);
        changeDeviceAddr();
    }

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void data_report_task()
//{
//	Latitude = 100.12345;
//    Longitude = 100.12345;
//	memset(GPS_send.Latitude, 0, 8);
//	memset(GPS_send.Longitude, 0, 9);
//	sprintf(GPS_send.Latitude, "%.5f", Latitude);
//	sprintf(GPS_send.Longitude, "%.5f", Longitude);
//	neulSendAndRecv(socket_udp, (const char*)(&GPS_send), sizeof(GPS_send), bc95_net_data.net_nmgr, 5, 0, 3, 3);
//}

	float temper=0 ;
	int count=0;
	
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim==(&htim2))
//    {
//        count++;       //LED1??
//    }
//}



//HAL_GPIO_EXTI_Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1)
			powermode = 1;
		else 
			powermode = 0;

    if(DIO0_GetState() == GPIO_PIN_SET)
    {

        uint8_t flag;
        SX127X_Read(REG_LR_IRQFLAGS, &flag);
        SX127X_Write(REG_LR_IRQFLAGS, 0xff); //clear flags
        if(flag & RFLR_IRQFLAGS_TXDONE)
        {
            communication_states = TX_DONE;
            T_Cnt++;
								//OLED_ShowString(0,60,"dio0:Tx",12);
                //OLED_ShowNum(48,0,auth_Success,1,12);
                //OLED_Refresh_Gram();
						DIO0_EnableInterrupt();
				    SX127X_StartRx();
        }
        else if(flag & RFLR_IRQFLAGS_RXDONE)
        {
            //communication_states = RX_DONE;
					DIO0_DisableInterrupt();
					SX127X_RxPacket(Rxbuf);
					printf("receive data from slaver: %s\n", Rxbuf);
				//	check_receive_data(Rxbuf);
                    // OLED_ShowString(24,0"receive:",12);
					R_Cnt++;
					//update 2019-0319
					//DIO0_EnableInterrupt();
					//SX127X_StartRx();
								//OLED_ShowString(0,60,"dio0:Rx",12);
                //OLED_ShowNum(48,0,auth_Success,1,12);
                //OLED_Refresh_Gram();
        }
        else
        {
            DIO0_EnableInterrupt();
            SX127X_StartRx();
								//OLED_ShowString(0,60,"dio0:else",12);
                //OLED_ShowNum(48,0,auth_Success,1,12);
                //OLED_Refresh_Gram();
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t plat_ip = 0x00000000;
		int8_t i=0,auth_Success=0,receive4data=0;
	int ret=0;
	uint8_t need_reauth=0;
	uint32_t adc_ave=0;
//	uint8_t i=0;
 //__read_Storage_SlaveNum(flash_addr,slaveTable);
// for (i=0;i<4;i++)
//     slaveTable[0][i]=0;

//MasterNum = default_MasterNum;
	uint32_t SlaveNum = 0;

	#if 0
	uint32_t master_ip = 0x80000001;
    
    uint32_t slaver_ip[20] = {0x00000001, 0x00000002, 0x00000003, 0x00000004, 0x00000005,
                              0x00000006, 0x00000007, 0x00000008, 0x00000009, 0x00000010,
                              0x00000011, 0x00000012, 0x00000013, 0x00000014, 0x00000015,
                              0x00000016, 0x00000017, 0x00000018, 0x00000019, 0x00000020};
    uint16_t temp[20] = {3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900};
    uint16_t voltage[20] = {3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900};
    uint16_t tempM = 1000;
    uint16_t volM = 1000;
    uint8_t power_mode = 0;
    uint8_t singnal_strength = 15;
    uint8_t slave_num = 20;
    uint8_t data_frame[256] = {0};
    int data_length = 0;
//    data_length = PacketFrame(master_ip, plat_ip, slaver_ip, temp, voltage, tempM, volM, power_mode, singnal_strength, slave_num, data_frame);
#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
    report_cfg.report_stat = 0;
    report_cfg.report_count = 0;
    for (i = 0; i < 3; i++)
    {
        report_cfg.report_count_max = readFlashWord(REPORT_PERIOD_SAVE_ADDR);
        HAL_Delay(100);
    }
    if(report_cfg.report_count_max > 0x000005A0)
        report_cfg.report_count_max = DEFAULT_REPORT_PERIOD;
//	for(i = 0; i < 33; i++)
//	{
//		slaveTable[i] = i+1;
//	}
//	writeFlashWordArray(SLAVER_ADDRESS_SAVE_ADDR, slaveTable, 70);
	for(i = 0; i < 2; i++)
	{
		readFlashWordArray(SLAVER_ADDRESS_SAVE_ADDR, slaveTable , MAX_SLAVE_NUM);
		MasterNum = readFlashWord(MASTER_ADDRESS_SAVE_ADDR);
		if ( !validMasterAddr(MasterNum))
        {
		    MasterNum = DEFAULT_MASTER_ADDRESS;
        }
	}
	#if 1
    HAL_UART_Receive_IT(&huart1, uart1_recv_buf, uart1_recv_max_length);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);//关灯
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      //  HAL_GPIO_WritePin(NB_RESET_GPIO_Port, NB_RESET_Pin, GPIO_PIN_SET);
      //  HAL_GPIO_WritePin(NB_RESET_GPIO_Port, NB_RESET_Pin, GPIO_PIN_RESET);
	printf("init NB\n");
      int nb_init_flag = los_dev_uart_init(LOS_STM32L431_UART3, 9600, nbiot_uart, 500);
      socket_udp = neulBC95Start("119.29.112.88", 8086, 1, 8888);
	#endif
	
	//Tag :ezio
//	#if USEDMA

//	   HAL_ADC_Start_DMA(&hadc,(uint32_t*) adcvalue,2);
//	#endif
	//	HAL_ADCEx_Calibration_Start(&hadc,ADC_SINGLE_ENDED);
	
	


        //���ø�������
    G_LoRaConfig.LoRa_Freq = Fre[0];      //����Ƶ��470MHz
    G_LoRaConfig.BandWidth = BW125KHZ;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = SF09;  //SF = 9
    G_LoRaConfig.CodingRate = CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 15;          //19��1dBm
    G_LoRaConfig.MaxPowerOn = true;       //����ʿ���?
    G_LoRaConfig.CRCON = true;            //CRCУ�鿪��
    G_LoRaConfig.ExplicitHeaderOn = true; //Header����
    
    G_LoRaConfig.PayloadLength = Length;      //���ݰ�����
        
    if(SX127X_Lora_init() != NORMAL)     //����ģ���ʼ��?
            return -1;
		
    DIO0_EnableInterrupt();
    SX127X_StartRx();

		// __start_timer(2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 #if D18b20
  	SET_DS18B20();         //ʹPA2����ߵ��?

  	DS18B20_Rst();
  	
  	//d18b20_in_Init();
  	ret=DS18B20_Check();

  	/*
  	while(DS18B20_Init())
  	{
  		HAL_Delay(500);
  		
  	}
  	*/
	#endif
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim6);
	MX_IWDG_Init();
  //before establish network ,turn lora light on 
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); 
 while (1)
  {
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		

  #if D18b20

  	
  	//d18b20_in_Init();

    DS18B20_Init()
    temp_18b20 = DS18B20_GetTemp_SkipRom();
  	HAL_Delay(500);
  
	#endif
		
		

		
    //readFlashWordArray(Page_storage_slavers,slaveTable,MAX_SLAVE_NUM);
				{
			
            {
							//ret =
							__auth_network(MasterNum,slaveTable,slave_num);
                if(ret == 0)
                {
                    auth_Success++;                  
                    //50s 
                    lora_blink_time = 50;
                }
                if(ret >0)
                    lora_blink_time = 5;

            }
			//err and don't know how to fix
    	//ret = auth_network(MasterNum,slaveTable,slave_num);
			#if 1
    	do {
			//err and don't know how to fix
    	//ret = ask4data(MasterNum,slaveTable,slave_num);
				
                {
					ret = __ask4data(MasterNum,slaveTable,slave_num);
								printf("Aquire data \n");

									
                    if(ret == 0)
                    {
                      receive4data++;
                      // lora_normal_led();
                      lora_blink_time = 50;
                    }
                    if(ret >0)
                      lora_blink_time = 5;
				

										
                    //or set a timer?
                    // Flag_timer_5min_exceed = 0;
                    // if(1 == Flag_timer_5min_exceed);
                    // if(20 == receive4data)
                    {
                        if (report_cfg.report_stat == 1)
                        {
                            send_data2_platform_and_receive(MasterNum,plat_ip,slaveTable,slave_num);
                            report_cfg.report_stat = 0;
							report_cfg.report_count = 0;
							HAL_TIM_Base_Start_IT(&htim2);
							for(i = 0; i < 6; i++)
							{
//								printf("report success, led blink %d\n", i);
								NBLEDStateControl();
								HAL_Delay(500);
							}
                        }
                    }
                    //Tag :5mins
//                    HAL_Delay(3000);
					HAL_IWDG_Refresh(&hiwdg);
//					printf("Refreshes the IWDG\n");
                    #if 0
                    //Another state
                    if(hand_self_test_handle||platform_self_test_handle)
                        {
                           handle_self_test_master();

                         
                        }

                    if(hand_add_del_handle||platform_add_del_handle)
                    {
                      handle_add_del_master();
                        //0xAA add, 0x55 del


                    }
                    if(hand_change_fre_handle||platform_change_fre_handle)
                    {//same as slave
                    // handle_change_Fre_master();
                    ret=handle_change_Fre_maseter(MasterNum,SlaveNum,Rxbuf);
                    }
                    //Fre : need shouchi- master - slaver?
										#endif
                                
            	}
            }while(need_reauth==0);
			#endif
		
  }
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
