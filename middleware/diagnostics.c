#include "driver_UART.h"
#include "stdio.h"
#include "string.h"
#include "diagnostics.h"
#include "driver_CANbus.h"

static uint8_t diag_data[26] = {0x80, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0x8F}; 

static uint8_t c1data[8] = {0,0,0,0, 0,0,0,0};
static uint8_t c2data[8] = {0,0,0,0, 0,0,0,0};
static uint8_t c3data[8] = {0,0,0,0, 0,0,0,0};
static uint8_t c4data[8] = {0,0,0,0, 0,0,0,0};
static uint8_t c5data[8] = {0,0,0,0, 0,0,0,0};
static uint8_t c6data[8] = {0,0,0,0, 0,0,0,0};

void fill_diag_data(int data_state, float fd_Vdc, float fd_Idc, float fd_rpm, float fd_Id, float fd_Tesc, float fd_Tmot) // Add Tcnt later
{
       
    float tempf;
    
    
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
        send_data_uart(diag_data, 26);
}


void fill_TU_data1(float V0, float V1, float V2, float V3, float V4, float V5)
{
    __builtin_memcpy((uint32_t *)(c1data+0), (uint32_t *)&V0, 4);
    __builtin_memcpy((uint32_t *)(c1data+4), (uint32_t *)&V1, 4);
    
    __builtin_memcpy((uint32_t *)(c2data+0), (uint32_t *)&V2, 4);
    __builtin_memcpy((uint32_t *)(c2data+4), (uint32_t *)&V3, 4);
   
    __builtin_memcpy((uint32_t *)(c3data+0), (uint32_t *)&V4, 4);
    __builtin_memcpy((uint32_t *)(c3data+4), (uint32_t *)&V5, 4);
}

void fill_TU_data2(float V6, float V7, float V8, float V9, float V10, float V11)
{
    __builtin_memcpy((uint32_t *)(c4data+0), (uint32_t *)&V6, 4);
    __builtin_memcpy((uint32_t *)(c4data+4), (uint32_t *)&V7, 4);
    
    __builtin_memcpy((uint32_t *)(c5data+0), (uint32_t *)&V8, 4);
    __builtin_memcpy((uint32_t *)(c5data+4), (uint32_t *)&V9, 4);
    
    __builtin_memcpy((uint32_t *)(c6data+0), (uint32_t *)&V10, 4);
    __builtin_memcpy((uint32_t *)(c6data+4), (uint32_t *)&V11, 4);    
}


void C0_send_TU_data_CAN(void)
{
    CANbus_write(0xC1, c1data, 8);
    CANbus_write(0xC2, c2data, 8);
    CANbus_write(0xC3, c3data, 8);
    CANbus_write(0xC4, c4data, 8);
    CANbus_write(0xC5, c5data, 8);
    CANbus_write(0xC6, c6data, 8);
}
