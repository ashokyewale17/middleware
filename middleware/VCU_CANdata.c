#include "VCU_CANdata.h"
#include "device.h"
#include "math.h"
#include "driver_CANbus.h"
#include "stdlib.h"

static float speed = -2000, Ibat = -0x65, Iph = -365, Tmcu = -25, Tmot = -25, Vbat = 48;
static int fault = 0;
static volatile uint8_t speed_limit = 0, control_type = 0, Iregen_limit = 0;
static int ind_rev = 0, ind_flt = 0, ind_ss = 0, ind_brk = 0, ind_L = 0, ind_H = 0, ind_Park = 1;

void reverse_indicator(int stat)
{
    ind_rev = stat;
}

void fault_indicator(int stat)
{
    if(stat == 0) ind_flt = 0;
    else ind_flt = 1;
}

void side_stand_indicator(int stat)
{
    ind_ss = stat;
}

void brake_indicator(int stat)
{
    ind_brk = stat;
}

void Lmode_indicator(int stat)
{
    ind_L = stat;
}

void Hmode_indicator(int stat)
{
    ind_H = stat;
}

void Parkmode_indicator(int stat)
{
    ind_Park = stat;
}


void fill_data_for_VCU(float rpm_to_speed, float rpm, float Idc, float Id, float Tesc, float Tmotor, float Vdc, int faultid)
{
    static uint16_t broadcast_time = 0;
    speed = rpm * rpm_to_speed;
    Ibat = -Idc; 
    Iph = Id;
    Tmcu = Tesc;
    fault = faultid;
    Tmot = Tmotor;
    Vbat = Vdc;        
    
    broadcast_time++;
    
    if(broadcast_time == 1250)
    {
        send_A2data();
    }
    
    if(broadcast_time > 2500)
    {
        broadcast_time = 0;
        send_A4data();
    }
}

void A1_action(uint8_t* VCUmsg, uint8_t msg_len)
{
    send_A2data();
}

void A3_action(uint8_t* VCUmsg, uint8_t msg_len)
{
    send_A4data();
}

void send_A2data(void)
{   
    uint8_t A2data[8] = {0,0,0,0,0,0,0,0};
    uint16_t Iphase = 0;
    
    A2data[0] = (uint8_t)(fabs(speed));
    A2data[1] = (int)(Ibat);
    Iphase = (uint16_t)(fabs(Iph));
    
    A2data[3] = Iphase;
    A2data[2] = Iphase >> 8;
    A2data[4] = (uint8_t)(fabs(Tmcu));
    A2data[5] = (uint8_t)(fabs(Tmot));
    A2data[6] = (uint8_t)(fabs(Vbat));
    
    if(ind_rev == 1) A2data[7] = A2data[7] | 0b00000001; 
    else A2data[7] = A2data[7] & 0b11111110; 
    
    if(ind_flt == 1) A2data[7] = A2data[7] | 0b00000010; 
    else A2data[7] = A2data[7] & 0b11111101;
    
    if(ind_ss == 1) A2data[7] = A2data[7] | 0b00000100; 
    else A2data[7] = A2data[7] & 0b11111011;
    
    if(ind_brk == 1) A2data[7] = A2data[7] | 0b00001000; 
    else A2data[7] = A2data[7] & 0b11110111;
    
    if(ind_L == 1) A2data[7] = A2data[7] | 0b00010000; 
    else A2data[7] = A2data[7] & 0b11101111;
    
    if(ind_H == 1) A2data[7] = A2data[7] | 0b00100000; 
    else A2data[7] = A2data[7] & 0b11011111;
    
    if(ind_Park == 1) A2data[7] = A2data[7] | 0b01000000; 
    else A2data[7] = A2data[7] & 0b10111111;
    
    CANbus_write(0xA2, A2data, 8);
}

void send_A4data(void)
{   
    #define KMPH_SCALE 50

    uint8_t A4data[2] = {0,0};
    
    switch(fault)
    {
        case 0: //no fault
            A4data[0] = 0b00000000;
        break;
            
        case 1: //overvoltage
            A4data[0] = 0b00101000;
        break;
        
        case 2: //phase current
            A4data[0] = 0b10000000;
        break;
        
        case 3: //input current
            A4data[0] = 0b10000000;
        break;
        
        case 4: //temperature fault
            A4data[0] = 0b01000000;
        break;
        
        case 5: //undervoltage
            A4data[0] = 0b00011000;
        break;
        
        case 6: //motor temperature
            A4data[0] = 0b00000100;
        break;
        
        case 7: //sensor
            A4data[0] = 0b00000100;
        break;
        
        case 9: //throttle
            A4data[0] = 0b00000001;
        break;
                
        default:
            A4data[0] = 0b00001000;
        break;
    }
    
    CANbus_write(0xA4, A4data, 2);
}


