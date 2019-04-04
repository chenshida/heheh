#include <stdlib.h>
#include "uart_change_addr.h"
#include "stdint.h"
#include "stdio.h"
#include "helper.h"
#include "string.h"

uart_change_addr uart_change;

void uartChangeAddrDecode(uint8_t *recv_buf)
{
    uint8_t dev_type[2] = {0};
    uint8_t dev_opt[2] = {0};
    uint8_t dev_addr[10] = {0};
    sscanf(recv_buf, "%s\r\n%s\r\n%s", dev_type, dev_opt, dev_addr);
    uart_change.dev_type = dev_type[0];
    uart_change.dev_opt = dev_opt[0];
    uart_change.dev_addr = (uint32_t) atoi(dev_addr);
}

