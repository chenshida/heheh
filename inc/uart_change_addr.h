#ifndef TEMPERATUREMONITOR_UART_CHANGE_ADDR_H
#define TEMPERATUREMONITOR_UART_CHANGE_ADDR_H

#include "stdint.h"

typedef struct uart_change_addr_t{
    uint8_t dev_type;//master:M, slaver:S
    uint8_t dev_opt;//add:A, del:D
    uint32_t dev_addr; //address of slaver or master
} uart_change_addr;

extern uart_change_addr uart_change;

void uartChangeAddrDecode(uint8_t *recv_buf);

#endif //TEMPERATUREMONITOR_UART_CHANGE_ADDR_H
