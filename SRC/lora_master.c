#include "lora_master.h"
#include "math.h"
//#include "common_define.h"
#include "decode.h"
#include "common_function.h"
#include "protocol.h"
#include "math.h"
#include "neul_bc95.h"
#include "nb_init.h"
#include "nb_send_machine.h"
#include "main.h"
#include "flash_save_data.h"
#include "array_operation.h"
#include "helper.h"

const float Rp=10000.0;

const float T2 =(273.15+25.0);
const float Bx = 3950.0;
const float Ka = 273.15;

extern uint8_t Rxbuf[Max_Length];
extern uint32_t slaveTable[MAX_SLAVE_NUM];
extern uint8_t hand_change_fre_handle,platform_change_fre_handle,hand_add_del_handle,
platform_add_del_handle,hand_self_test_handle,platform_self_test_handle
,selftest_slaver_respone_handle ,respon2ask4data_handle,respon2authFrame_handle;
extern recv_add_del_data recv_add_del;
extern recv_change_freq_data recv_change_freq;
extern uint8_t Flag_timer_2s_exceed,Flag_timer_5min_exceed;
extern uint32_t MasterNum;
extern uint8_t slave_num;
extern uint16_t adcvalue[2];
uint16_t temper=0,voltage=0;

extern int socket_udp;
extern char pull_data[500];
extern int ret;
extern report_config report_cfg;
//190401
extern uint8_t  Flag_timer_exceed;
//struct 
data2platform __data2platform ;
uint8_t i=0;
int handle_change_Fre_maseter(uint32_t MasterNum,uint32_t SlaverNum,uint8_t *Rxbuf)
{  	
  uint8_t i=0;
  uint8_t DataFrame[Length];
  if (hand_change_fre_handle)
	  {
      //reserved
    }
  if (platform_change_fre_handle)
  {
    uint8_t flag = 0x00;
  //     typedef struct _recv_change_freq_data{
  //     uint16_t recv_length;
  //     uint8_t frequency;
  // } recv_change_freq_data;

    // change all the master and slaver fre
    for(i=0;i<slave_num;i++)
    {
       SetFreFrame(MasterNum,slaveTable[i],recv_change_freq.frequency,flag,DataFrame);
       send(DataFrame);
        //wait for 2s
        Flag_timer_2s_exceed = 0;
        do {

        }while(0 == Flag_timer_2s_exceed);
    }
    //master 
    change_Fre(recv_change_freq.frequency);
  }

  return 0;

}

int del_slave(uint32_t slave_num,uint32_t del_addr)
{
  int i=0,slaveTable_tmp[MAX_SLAVE_NUM]={0};
  uint8_t del_index=0;
  for(i=0; i<slave_num; i++)
    slaveTable_tmp[i] = slaveTable[i];
  for(i=0; i<slave_num; i++)
    if(slaveTable[i]==del_addr)
    {
      del_index = i;
      continue;
    }
  for(i=del_index;i<slave_num-1;i++)
    slaveTable[i] = slaveTable_tmp[i+1];

  return 0;
}
int handle_add_del_master()
{
    uint8_t i=0;
  	if (hand_add_del_handle)
	  {
        hand_add_del_handle--;
        
        if(0xAA == Rxbuf[12])
        {
              i=0;
              uint32_t add_Addr[MAX_SLAVE_NUM] = {0};
              uint8_t num = Rxbuf[13]*pow(2,4) + Rxbuf[14];
              
              do{
                slave_num++;
                // for(i=0;i<num;i++)
                add_Addr[i] = 0;//Rxbuf[5]*pow(2,24*4)+Rxbuf[6]*pow(2,16*4)+Rxbuf[7]*pow(2,2^8*4)+Rxbuf[8];
                slaveTable[slave_num]=add_Addr[i];
              }while(slave_num != MAX_SLAVE_NUM && (++i < num));

        }
        else if(0x55 == Rxbuf[12])
        {
          uint8_t num = Rxbuf[13]*pow(2,4) + Rxbuf[14];
          i=num;
          uint32_t  del_Addr[MAX_SLAVE_NUM] = {0};
          //tranform
          // del_Addr=Rxbuf[5]*pow(2,24*4)+Rxbuf[6]*pow(2,16*4)+Rxbuf[7]*pow(2,2^8*4)+Rxbuf[8];
          do{
            slave_num--;
            del_slave(slave_num,del_Addr[i]);
						//wirteFlashWordArray(Page_storage_slavers,slaveTable,MAX_SLAVE_NUM);
          }while(slave_num >0 && (--i >0));
        }
    }
    if (platform_add_del_handle)
	  {
      // recv_add_del_data recv_add_del;

      //       typedef struct _recv_add_del_data{
      //     uint16_t recv_length;
      //     uint8_t operation;
      //     uint16_t dev_num;
      //     uint32_t device_addr[MAX_DEVICE_NUM];
      // } recv_add_del_data;
        platform_add_del_handle--;
        if(0xAA == recv_add_del.operation)
        { 
              i=0;

              uint8_t num = recv_add_del.dev_num;
              
              do{
                slave_num++;
                // for(i=0;i<num;i++)
                
                slaveTable[slave_num] = recv_add_del.device_addr[i];
              }while(slave_num != MAX_DEVICE_NUM && (++i < num));
        }
        else if(0x55 == recv_add_del.operation)
        {
              
   
              uint8_t num = recv_add_del.dev_num;
              i=num;

              do{
                slave_num--;
                del_slave(slave_num,recv_add_del.device_addr[i]);
              
              }while(slave_num >0 && (--i >0));
        }
    }
    return 0;
}
int handle_self_test_master()
{
  uint8_t DataFrame[Length]={0};

	if (hand_self_test_handle)
	{
	  hand_self_test_handle--;
		uint32_t FailAddr[MAX_SLAVE_NUM];
		uint8_t FailNum =0;
		uint32_t SlaverNum=0;
	  for(i=0;i<slave_num;i++)
                           {
                            CheckSelfFrame( MasterNum, slaveTable[i], DataFrame);
                            send(DataFrame);
                            //wait for 2s
                            Flag_timer_2s_exceed = 0;
                            do {
	                            if(selftest_slaver_respone_handle)
	                            {
	                                if(Rxbuf[12] == 0x00)
	                                {
	                                    
	                                    FailAddr[FailNum]=Rxbuf[5]*pow(2,24*4)+Rxbuf[6]*pow(2,16*4)+Rxbuf[7]*pow(2,2^8*4)+Rxbuf[8];
	                                    FailNum++;
	                                }

	                            }
                            }while(0 == Flag_timer_2s_exceed);

                          }
                            Hand_CheckSelfRspFrame(MasterNum, SlaverNum, FailNum, FailAddr, DataFrame);
                            send(DataFrame);
    }
  if (platform_self_test_handle)
	{
	  platform_self_test_handle--;
		    __data2platform.FailNum=0;
	  for(i=0;i<slave_num;i++)
                           {
                            CheckSelfFrame( MasterNum, slaveTable[i], DataFrame);
                            send(DataFrame);
                            //wait for 2s
                            Flag_timer_2s_exceed = 0;
                            do {
                            if(selftest_slaver_respone_handle)
	                            {
	                                if(Rxbuf[12] == 0x00)
	                                {
	                                    
	                                    __data2platform.FailAddr[__data2platform.FailNum]=Rxbuf[5]*pow(2,24*4)+Rxbuf[6]*pow(2,16*4)+Rxbuf[7]*pow(2,2^8*4)+Rxbuf[8];
	                                    __data2platform.FailNum++;
	                                }
	                            }

                            }while(0 == Flag_timer_2s_exceed);


                          }

    // typedef struct _recv_self_check_data {
    // uint16_t recv_length;
    // uint8_t effective_data;
    // } recv_self_check_data;
    // recv_self_check_data recv_self_check;
    //?
     //send_to_platform(FailNum);
     //	neulSendAndRecv(socket_udp, (const char*)(&GPS_send), sizeof(GPS_send), bc95_net_data.net_nmgr, 5, 0, 3, 3);
    }
    return 0;
}

//Tag : check if slaverNum is matched ?
int  check_response(uint32_t slaveNum,uint8_t *Rxbuf)
{
	
	#if 0
    int8_t i=0;
    for(i=0;i<4;i++)
        if(slaveNum[i] != Rxbuf[5+i])
            return -1;
		#endif
	 return 0;
   
}

// 0 is normal , >0 means how many slave deive dont response
int __auth_network(uint32_t MasterNum,uint32_t *slaveTable,uint8_t slave_num)
{
	int8_t i=0;
	int ret=0;
	uint8_t dataFrame[Length]={0};
  for(i=0;i<slave_num;i++)
	{

		authFrame(MasterNum,slaveTable[i],dataFrame);
		send(dataFrame);
		//communication_states=TX_DONE;
		//while(communication_states!=TX_DONE);

    HAL_Delay(2000);



		{
			if(respon2authFrame_handle)
			{
				respon2authFrame_handle--;
				ret = check_response(slaveTable[i],Rxbuf);
				// if(ret>=0)
				// {
				// 	// continue;
				// 	__stop_timer();
				// 	return ret;
				// }
			}
      else 
        ret++;
		}

		
		#if 0
		//authFrame(MasterNum,slaveTable,dataFrame);
		send(dataFrame);
		//while(communication_states!=TX_DONE);
		/*
		OLED_ShowNum(85,0,Flag_timer_exceed,1,12);
		OLED_Refresh_Gram();
		*/
		__start_timer(3);
	//	OLED_ShowNum(95,0,Flag_timer_exceed,1,12);
	//	OLED_Refresh_Gram();
		//Flag_timer_2s_exceed,Flag_auth_response
		do
		{
			if(respon2authFrame_handle)
			{
				respon2authFrame_handle--;
				ret = check_response(slaveTable,Rxbuf);
				if(ret>=0)
				{
					// continue;
					__stop_timer();
					return ret;
				}
			}
		}
		while(!Flag_timer_exceed);

								//OLED_ShowString(0,0,"authFrame",12);            
               // OLED_Refresh_Gram();
		__stop_timer();

		#endif
		
	}
		return ret ;
}	
void save_temper_voltage(uint8_t i)
{
    //   DataFrame[12]=(temper>>8)&0xff;//�¶�����
    // DataFrame[13]=temper&0xff;//�¶�С��
    // DataFrame[14]=(voltage>>8)&0xff;//��ѹ����
    // DataFrame[15]=voltage&0xff;//��ѹС��
	printf("Rxbuf_temp is %d %d\n",Rxbuf[12],Rxbuf[13]);
	printf("vol_temp is %d %d\n",Rxbuf[14],Rxbuf[15]);
	__data2platform.temp_slaver[i] = ((Rxbuf[12] | 0x0000) <<8)+Rxbuf[13] ;
  __data2platform.vol_slaver[i] = ((Rxbuf[14] | 0x0000)<<8) +Rxbuf[15] ;

}
int __ask4data(uint32_t MasterNum,uint32_t *slaveTable,uint8_t slave_num)
{
		//Master temper report
    int8_t i=0,time_s =2;
		int ret=0;
    uint8_t dataFrame[Length]={0};
    for(i=0;i<slave_num;i++)
    {

        AcquireFrame(MasterNum,slaveTable[i],dataFrame);
        // AcquireFrame(uint32_t masterNum,uint32_t SlaverNum,uint8_t *DataFrame)
        send(dataFrame);
				//while(communication_states!=TX_DONE);
    #if 1
            // __start_timer(time_s);
            //Flag_timer_2s_exceed,Flag_auth_response
            HAL_Delay(2000);
            // do
            {
                if(respon2ask4data_handle)
    						{
    								respon2ask4data_handle--;
                    ret = check_response(slaveTable[i],Rxbuf);
                                    // if(ret>=0)
                                    // {
                                    //     // continue;
                    					           // __stop_timer();
                                    //     return ret;
                                    // }
    						}
                //if(exit_network_handle)
                            //[9]=0x06

            }
            // while(!Flag_timer_exceed);
            // __stop_timer();
            // time_s++;
    #endif
					save_temper_voltage(i);
        }
        
     
        
    
		return 0;

}
void get_temp_voltage()
{
    float temper_tmp=0.0f,voltage_tmp=0.0f;
    float temp=0.0f;
    float Rt=0.0f;
    float volta=0.0f;


    uint8_t temper_array[2]={0},voltage_array[2]={0};
    // for(int i=0;i<5;i++)
    {
        configChanggel1();
        HAL_ADC_Start(&hadc);
        //HAL_Delay(500);
        //190329 03:28 stall
        //while
        if(HAL_OK==HAL_ADC_PollForConversion(&hadc, 1000))
        {

            adcvalue[0] =HAL_ADC_GetValue(&hadc);
            // HAL_Delay(500);

        }
        HAL_ADC_Stop(&hadc);

        configChanggel2();
        HAL_ADC_Start(&hadc);
        //HAL_Delay(500);
        //while
        if(HAL_OK==HAL_ADC_PollForConversion(&hadc, 1000))
        {

            adcvalue[1] =HAL_ADC_GetValue(&hadc);
            // HAL_Delay(500);

        }
        HAL_ADC_Stop(&hadc);


        //HAL_Delay(500);//500ms
    }
    //ntc

    //printf("adcvalue0 is %.4X \n",adcvalue[0]);
    //printf("adcvalue1 is %.4X \n",adcvalue[1]);
    temper_tmp = NTCConvert(adcvalue[0]);
    voltage_tmp= getVoltage(adcvalue[1]);
    temper = (int)(temper_tmp*100);
    voltage = (int )(voltage_tmp*2*1000);
    //float2uint16_t

    

}
int send_data2_platform_and_receive(uint32_t MasterNum,uint32_t plat_ip,int32_t *slaveTable,int8_t slave_num)
{

	//package

      uint8_t power_mode=0;
       uint16_t temp_master=0,vol_master=0;
	int index = 0;
      //check_powermode(power_mode);
			get_temp_voltage();

     contruct_send_mesage(MasterNum,plat_ip,slaveTable, __data2platform.temp_slaver, __data2platform.vol_slaver ,temp_master,vol_master,power_mode,slave_num);
      //send to platform
     //extern socket_udp
     //#include "math.h"
     
     ret = neulSendAndRecv(socket_udp, bc95_net_data.net_nmgr, 100, 0, 3, 3);
      /**
       *
       * @param recv NB ????
      * @return 解码结果  1: 增加/删除设备
      *                  2: 更改频点
      *                  3: 自检请求
      *                  4: 平台数据上报回复
      *                  5: 平台注册回复
      *                   6 : check deivce 
      *                  -1: 功能码未知
       */

      // int decode_recv(char *recv)
			
     // ret=decode_recv(pull_data);
     // if (1==ret)
      //    platform_add_del_handle++;
     // if (2==ret)
     //     platform_change_fre_handle++;
     // if (3==ret)
     //     platform_self_test_handle++;
          /*create data package */


      /*check report result*/
//      ret = decode_recv(bc95_net_data.net_nmgr);
    /*读取数据下行 */
//      ret = neul_bc95_udp_read(socket_udp, pull_data, 500, 0);
	HAL_Delay(10);
	  while(neul_bc95_udp_read(socket_udp, pull_data, 500, 0) > 0)
	  {
		  printf("commond: %s\n", pull_data);
    /*有数据下行,解析;无数据下行,跳过*/
		  
		  ret = decode_recv(pull_data);
		   if (1==ret)
		   {
			   platform_add_del_handle++;
			   if(recv_add_del.operation == 0x55) //删减设备
			   {
				   for(index = 0; index < recv_add_del.dev_num; index ++)
				   {
					   delItem(recv_add_del.device_addr[index], slaveTable, MAX_SLAVE_NUM);
				   }
			   }
			   else
			   {
				   for(index = 0; index < recv_add_del.dev_num; index ++)
				   {
					   addItem(recv_add_del.device_addr[index], slaveTable, MAX_SLAVE_NUM);
				   }
			   }
			   writeFlashWordArray(SLAVER_ADDRESS_SAVE_ADDR, slaveTable, MAX_SLAVE_NUM);
			   //recv_add_del 数据在这里,定义在decode.c里面
			   //需要把新的slaver地址保存到flash
//			   writeFlashWordArray(uint32_t addr, uint32_t *inputArray, uint32_t arrayLength);
			   //初始化LORA网络时，slaver地址从flash读取
		   }
		   if (2==ret)
			   platform_change_fre_handle++;
		   if (3==ret)
			   platform_self_test_handle++;
		  if(ret == 7)
		  {
			  report_cfg.report_count_max = (uint32_t) (recv_change_report_period.report_period_minite * 60);
			  if(report_cfg.report_count_max >= 0xffffffff)
			  {
				  report_cfg.report_count_max = DEFAULT_REPORT_PERIOD;
			  }
			  writeFlashWord(REPORT_PERIOD_SAVE_ADDR, report_cfg.report_count_max);
		  }
	}
    //if good.													
    return 0;													
}