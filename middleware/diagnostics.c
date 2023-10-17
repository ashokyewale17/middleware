#include "driver_UART.h"
#include "stdio.h"
#include "string.h"
#include "diagnostics.h"

static uint8_t diag_data[26] = {0x80, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0x8F}; 
static int OKdata = 0;

void fill_diag_data(int data_state, float fd_Vdc, float fd_Idc, float fd_rpm, float fd_Id, float fd_Tesc, float fd_Tmot) // Add Tcnt later
{
       
    float tempf;
    
    //if((data_state == 3) | (data_state == 2) )  
    OKdata = 1;
    //else OKdata = 0;
    
    tempf = fd_Vdc;
    __builtin_memcpy((uint8_t *)(diag_data + 1), &tempf, 4);

    tempf = fd_Idc;
    __builtin_memcpy((uint8_t *)(diag_data + 5), &tempf, 4);

    tempf = fd_rpm;
    __builtin_memcpy((uint8_t *)(diag_data + 9), &tempf, 4);

    tempf = fd_Id;
    __builtin_memcpy((uint8_t *)(diag_data + 13), &tempf, 4);

    tempf = fd_Tesc;
    __builtin_memcpy((uint8_t *)(diag_data + 17), &tempf, 4);

    tempf = fd_Tmot;
    __builtin_memcpy((uint8_t *)(diag_data + 21), &tempf, 4);
}

void send_diag_data(void)
{
    if(OKdata == 1)
        send_data_uart(diag_data, 26);
}
