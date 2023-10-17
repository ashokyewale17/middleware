#include "driver_UART.h"
#include "device.h"
static unsigned char devsn_string[10] = {0x20, 0,0,0,0, 0,0,0,0, 0x2F};
static unsigned char stat_str[] = {0x10,0x02,0x1F};


void DEVSN_send(void)
{
    uint32_t mydevsnL;
    uint32_t mydevsnH;
    mydevsnL    = (uint32_t)DEVSN0;
    mydevsnH    = (uint32_t)DEVSN1;
 __builtin_memcpy((uint8_t *)(devsn_string + 1), &mydevsnH, 4);
 __builtin_memcpy((uint8_t *)(devsn_string + 5), &mydevsnL, 4);
 send_data_uart(devsn_string, 10);
}

void device_state_send(void)
{
  
 send_data_uart(stat_str,3);
}


 //*/