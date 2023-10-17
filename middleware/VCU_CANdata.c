#include "VCU_CANdata.h"
#include "device.h"
#include "math.h"
#include "driver_CANbus.h"
#include "stdlib.h"

static float speed = -2000, Ibat = -0x65, Iph = -365, Tc = -25;
static int fault = 0;
static volatile uint8_t speed_limit = 0, control_type = 0, Iregen_limit = 0;
static int speed_limit_EN = 0, regen_EN = 0, immob_EN = 0, reverse_EN = 0;
static float VM_rpm_limit_mem = 0;

void fill_data_for_VCU(float rpm_to_speed, float rpm, float Idc, float Id, float Tesc, int faultid)
{
    static uint16_t broadcast_time = 0;
    speed = rpm * rpm_to_speed;
    Ibat = -Idc; 
    Iph = Id;
    Tc = Tesc;
    fault = faultid;
    
    broadcast_time++;
    
    if(broadcast_time > 2500)
    {
        broadcast_time = 0;
        send_A2data();
        send_A4data();
    }
}

void A1_action(uint8_t* VCUmsg, uint8_t msg_len)
{
    speed_limit = *VCUmsg;
    control_type = *(VCUmsg+1);
    Iregen_limit = *(VCUmsg+2);
    
    if((control_type & 0b00000001) > 0) speed_limit_EN = 1;
    else {if(speed_limit_EN==1) speed_limit_EN = 2;}
        
    if((control_type & 0b00000010) > 0) regen_EN = 1;
    else {if(regen_EN == 1) regen_EN = 2;}
    
    if((control_type & 0b00000100) > 0) immob_EN = 1;
    else {if(immob_EN == 1) immob_EN = 2;}
    
    if((control_type & 0b00001000) > 0) reverse_EN = 1;
    else {if(reverse_EN == 1) reverse_EN = 2;}
    
    Nop();
    send_A2data();
}

void A3_action(uint8_t* VCUmsg, uint8_t msg_len)
{
    send_A4data();
}

void send_A2data(void)
{   
    #define KMPH_SCALE 50

    uint8_t A2data[5] = {0,0,0,0,0};
    uint16_t Iphase = 0;
    
    A2data[0] = (uint8_t)(fabs(speed));
    A2data[1] = (int)(Ibat);
    Iphase = (uint16_t)(fabs(Iph));
    
    A2data[3] = Iphase;
    A2data[2] = Iphase >> 8;
    A2data[4] = (uint8_t)(fabs(Tc));
    
    
    CANbus_write(0xA2, A2data, 5);
    
    Nop();
    Nop();
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
            A4data[0] = 0b00100000;
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
            A4data[0] = 0b00010000;
        break;
        
        case 6: //sensor
            A4data[0] = 0b00000100;
        break;
        
        case 7: //sensor
            A4data[0] = 0b00000100;
        break;
        
        case 9:
            A4data[0] = 0b00000001;
        break;
                
        default:
            A4data[0] = 0b00001000;
        break;
    }
    
    CANbus_write(0xA4, A4data, 2);
}


void CAN_init(float rpm_max)
{
    VM_rpm_limit_mem = rpm_max;
}

//called only in drive reset mode (vehicle is stationary)   
void CAN_reverse(float Iph_max, float rpm_max, float reverse_rpm, float *VM_rpm_limit_adr) 
{
    if(reverse_EN == 1)
    {
                         
        if(*VM_rpm_limit_adr >= 1) VM_rpm_limit_mem = *VM_rpm_limit_adr; 
        *VM_rpm_limit_adr = -reverse_rpm * rpm_max/100;        
    }
    else
    {
        if(reverse_EN == 2)
        {
            *VM_rpm_limit_adr = VM_rpm_limit_mem;
            reverse_EN = 0;
        }
    }
}

//called only in drive reset mode (vehicle is stationary)
void CAN_immob_EN(float Iph_max, float *VM_rpm_limit_adr)
{       
    if(immob_EN == 1)
    {
        
        if(*VM_rpm_limit_adr >= 1) VM_rpm_limit_mem = *VM_rpm_limit_adr; 
        *VM_rpm_limit_adr = 0;
    }
    else
    {
        if(immob_EN == 2)
        {
            *VM_rpm_limit_adr = VM_rpm_limit_mem;    //here throttle will be zero = safe start
            immob_EN = 0;
        }
    }
}

void CAN_immob_DIS(float *VM_rpm_limit_adr)
{
    if(immob_EN == 2)
    {
        *VM_rpm_limit_adr = 0.1;  //release immob to reset the drive safely (as throttle could be high).
    }
}


void CAN_Vmode(float rpm_max, float rpm_to_speed, float *VM_rpm_limit_adr, float *VM_Iregen_adr)
{
    if(speed_limit_EN == 1)
    {
        if(rpm_to_speed > 0.001)  //zero protection
        *VM_rpm_limit_adr = fabs(((float)speed_limit)/rpm_to_speed); 
        if(*VM_rpm_limit_adr < 1) *VM_rpm_limit_adr = 1;
            
    }
    else if(speed_limit_EN == 2) //limit release algorithm
    {
        speed_limit_EN = 0;
        *VM_rpm_limit_adr = rpm_max; 
    }    
    
    if(regen_EN == 1)
    {
        *VM_Iregen_adr = fabs((float)Iregen_limit);                
    }
    else if(regen_EN == 2)
    {
        regen_EN = 0;
        *VM_Iregen_adr = 0;
    }
}