#include <stdio.h>
#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "common_define.h"
typedef struct data2platfrom
{
  /* data */
  //for self-test
  uint8_t FailNum;
  uint32_t *FailAddr;
  //for temper & voltage
  uint16_t temp_slaver[MAX_SLAVE_NUM];
  uint16_t vol_slaver[MAX_SLAVE_NUM];
}data2platform;



int handle_change_Fre_maseter(uint32_t MasterNum,uint32_t SlaverNum,uint8_t *Rxbuf);
int del_slave(uint32_t slave_num,uint32_t del_addr);
int handle_add_del_master();
int handle_self_test_master();
int  check_response(uint32_t slaveNum,uint8_t *Rxbuf);
int __auth_network(uint32_t MasterNum,uint32_t *slaveTable,uint8_t slave_num);
void save_temper_voltage(uint8_t i);//, struct data2platform __data2platform);
int __ask4data(uint32_t MasterNum,uint32_t *slaveTable,uint8_t slave_num);
int send_data2_platform_and_receive(uint32_t MasterNum,uint32_t plat_ip,int32_t *slaveTable,int8_t slave_num);
//void get_temp_voltage(uint16_t temper,uint16_t voltage);
void get_temp_voltage();