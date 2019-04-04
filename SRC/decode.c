//
// Created by pi on 19-3-18.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "decode.h"
#include "helper.h"

common_recv_data common_recv;
recv_add_del_data recv_add_del;
recv_change_freq_data recv_change_freq;
recv_self_check_data recv_self_check;
recv_report_response_data recv_report_response;
recv_login_response_data recv_login_response;
recv_check_device_data recv_check_device;
recv_change_report_period_data recv_change_report_period;

void clear_data()
{
    memset(&common_recv, 0, sizeof(common_recv));
    memset(&recv_add_del, 0, sizeof(recv_add_del));
    memset(&recv_change_freq, 0, sizeof(recv_change_freq));
    memset(&recv_self_check, 0, sizeof(recv_self_check));
    memset(&recv_report_response, 0, sizeof(recv_report_response));
    memset(&recv_login_response, 0, sizeof(recv_login_response));
	memset(&recv_check_device, 0, sizeof(recv_check_device));
	memset(&recv_change_report_period, 0, sizeof(recv_change_report_period));
}


/**
 *
 * @param recv NB 下行数据
 * @return 解码结果  1: 增加/删除设备
 *                  2: 更改频点
 *                  3: 自检请求
 *                  4: 平台数据上报回复
 *                  5: 平台注册回复
 *                  -1: 功能码未知
 */

int decode_recv(char *recv)
{
    int index;
//    char socket[1];
//    char ip[16];
//    char port[5];
//    char length[4];
//    char data[256];
//    char res[4];
    char frame_begin[2];
    char master_addr[8];
    char hand_dev_addr[8];
    char func_code[2];

    char real_length[4];
    char operation[2];
    char dev_num[4];
    char slaver_addr_item[8];
    char freq_code[2];
    char report_rsp[2];
    char login_response[2];
    char period_str[4];
//    sscanf(recv, "%[^,],%[^,],%[^,],%[^,],%[^,],%s", socket, ip, port, length, data, res);
    sscanf(recv, "%2s%8s%8s%2s", frame_begin, master_addr, hand_dev_addr, func_code);
    common_recv.master_addr = (uint32_t) htoi(master_addr);
    common_recv.hand_dev_addr = (uint32_t) htoi(hand_dev_addr);
    common_recv.func_code = (uint8_t) htoi(func_code);
//    printf("master_addr: %08x\n", common_recv.master_addr);
//    printf("hand_dev_addr: %08x\n", common_recv.hand_dev_addr);
//    printf("func_code: %02x\n", common_recv.func_code);

//	clear_data();

    if(common_recv.func_code == ADDDELDEV_REQ)
    {

        real_length[0] = recv[20];
        real_length[1] = recv[21];
        real_length[2] = recv[22];
        real_length[3] = recv[23];
        operation[0] = recv[24];
        operation[1] = recv[25];
        dev_num[0] = recv[26];
        dev_num[1] = recv[27];
        dev_num[2] = recv[28];
        dev_num[3] = recv[29];
        recv_add_del.recv_length = (uint16_t) htoi(real_length);
        recv_add_del.operation = (uint8_t) htoi(operation);
        recv_add_del.dev_num = (uint16_t) htoi(dev_num);
		if(recv_add_del.dev_num > 20)
		{
			recv_add_del.dev_num = 20;
		}
        for(index = 0; index < recv_add_del.dev_num; index++)
        {
            memset(slaver_addr_item, 0, 8);
            slaver_addr_item[0] = recv[30 + index * 8 + 0];
            slaver_addr_item[1] = recv[30 + index * 8 + 1];
            slaver_addr_item[2] = recv[30 + index * 8 + 2];
            slaver_addr_item[3] = recv[30 + index * 8 + 3];
            slaver_addr_item[4] = recv[30 + index * 8 + 4];
            slaver_addr_item[5] = recv[30 + index * 8 + 5];
            slaver_addr_item[6] = recv[30 + index * 8 + 6];
            slaver_addr_item[7] = recv[30 + index * 8 + 7];
            recv_add_del.device_addr[index] = (uint32_t) htoi(slaver_addr_item);
        }
        return 1;
    }
    else if (common_recv.func_code == FREQCHANGE_REQ)
    {
        recv_change_freq.recv_length = 0x01;
        freq_code[0] = recv[22];
        freq_code[1] = recv[23];
        recv_change_freq.frequency = (uint8_t) htoi(freq_code);
        return 2;
    }
    else if (common_recv.func_code == SELFCHECK_REQ)
    {
        recv_self_check.recv_length = 0x01;
        recv_self_check.effective_data = 0xff;
        return 3;
    }
    else if (common_recv.func_code == PLATFORMRESPONSE)
    {
        recv_report_response.recv_length = 0x01;
        report_rsp[0] = recv[22];
        report_rsp[1] = recv[23];
        recv_report_response.ret_response = (uint8_t) htoi(report_rsp);
        return 4;
    }
    else if(common_recv.func_code == PLATFORMLOGIN_RES)
    {
        recv_login_response.recv_length = 0x01;
        login_response[0] = recv[22];
        login_response[1] = recv[23];
        recv_login_response.login_response = (uint8_t) htoi(login_response);
        return 5;
    }
    else if(common_recv.func_code == HANDDEVCHECK_REQ)
    {
        recv_check_device.recv_length = 0x01;
        recv_check_device.effective_data = 0xff;
        return 6;
    }
    else if(common_recv.func_code == CHANGEREPORTPERIOD_REQ)
    {
        recv_change_report_period.recv_length = 0x02;
        period_str[0] = recv[22];
        period_str[1] = recv[23];
		period_str[2] = recv[24];
		period_str[3] = recv[25];
        recv_change_report_period.report_period_minite = (uint8_t) htoi(period_str);
        return 7;
    }
    else
    {
        return -1;
    }
    return 0;
}