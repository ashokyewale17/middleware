#include "stdlib.h"
#include "tuning.h"
#include "math.h"
#include "device.h"
#include "driver_UART.h"
#include "default_ESC_config.h"

static uint8_t read_tuning_data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t send_tuning_data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static int ESCid = 0;
static float T_Iph_max, T_rpm_max;
static int T_poles;                                         //FF01
static float T_Rph, T_Ld, T_Lq, T_PHIph;                    //FF02
static float T_zero_angle, T_zero_angle_measured;           //FF04
static int T_sensor_direction, T_sensor_direction_measured; //FF04

static int T_rotation_direction;                                    //FF06
static float T_ang_margine, T_rpm_to_kmph, T_rpm_fault;             //FF06
static float T_motor_derateC, T_motor_TfaultC, T_ESC_derateC, T_ESC_TfaultC; //FF07
static float T_Vbat_max, T_Vbat_min, T_OV_fault, T_UV_fault;        //FF08
static float T_Ibat_max, T_Ibat_regen, T_Iph_fault, T_Ibat_fault;    //FF08

static int T_driving_mode;     //FF0B
static float T_Leco_rpm, T_eco_rpm;            //FF0B
static float T_throttle_zero, T_throttle_max;       //FF0C
static float T_Vbrake_derate, T_immob_rpm, T_reverse_rpm;                               //FF0C
static float T_kp_rpm, T_ki_rpm, T_kp_torque, T_ki_torque; //FF0D

void send_measured_angle(float theta, float theta0, float theta1, int sensor_direction_estimate)
{
    T_zero_angle_measured = theta;                                      //update the zero angle 
    T_sensor_direction_measured = sensor_direction_estimate;            //update correct direction of sensor
    send_currently_used_values(0x04);  
}

void tune_state_variables(float * Rph_adr, float * Ld_adr, float * Lq_adr, float * PHIph_adr, float * zero_angle_adr, int * sensor_direction_adr, float * ang_margine_adr, int * rotation_direction_adr)
{
    volatile uint32_t temp32;
    volatile float tempf;
    volatile bool Eread_stat = 0;
    
    Eread_stat = Eread_stat;
    
    //==========================================================================
    //first update with currently used values
    
    T_Rph = *Rph_adr;
    T_Ld = *Ld_adr;
    T_Lq = *Lq_adr;
    T_PHIph = *PHIph_adr;
        
    T_zero_angle = *zero_angle_adr;   
    T_sensor_direction = *sensor_direction_adr;
    
    T_rotation_direction = *rotation_direction_adr;
    T_ang_margine = *ang_margine_adr;            
    
    
    //====================================================================================
    //update values from EEPROM if ESCid> 0
    
    Eread_stat = EEPROM_WordRead(0x10, (uint32_t *)&temp32);
    __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
    
    ESCid = tempf;
    
    if(ESCid > 0)
    {
        //2==========================================================================
        Eread_stat = EEPROM_WordRead(0x20, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Rph = fabs(tempf)/1000.0;
        
        Eread_stat = EEPROM_WordRead(0x24, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Ld = fabs(tempf)/1000.0;
                
        Eread_stat = EEPROM_WordRead(0x28, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Lq = fabs(tempf)/1000.0;
        
        Eread_stat = EEPROM_WordRead(0x2C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_PHIph = fabs(tempf)/1000.0;
        
        //4==========================================================================
        Eread_stat = EEPROM_WordRead(0x40, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_zero_angle = tempf;     
        
        Eread_stat = EEPROM_WordRead(0x44, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        if(tempf > 0) T_sensor_direction = 1;
        else T_sensor_direction = 0;        
      
        //6==========================================================================
        Eread_stat = EEPROM_WordRead(0x60, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        if(tempf > 0) T_rotation_direction = 1;
        else T_rotation_direction = 0;
        
        Eread_stat = EEPROM_WordRead(0x64, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_ang_margine = fabs(tempf);
        if(T_ang_margine < 5) T_ang_margine = 5;
        if(T_ang_margine > 50) T_ang_margine = 50;
    }
    
    //===============================================================================
    //Return tuning if ESCid > 0 from EEPROM otherwise as it is (default values)
    
    *Rph_adr = T_Rph;
    *Ld_adr = T_Ld;
    *Lq_adr = T_Lq;
    *PHIph_adr = T_PHIph;
    
    *ang_margine_adr = T_ang_margine;
    *zero_angle_adr = T_zero_angle;
    
    *sensor_direction_adr = T_sensor_direction;
    *rotation_direction_adr = T_rotation_direction;
}

void tune_faults(float *rpm_fault_adr, float * OV_fault_adr, float * UV_fault_adr, float * Iph_fault_adr, float * Ibat_fault_adr, float * ESC_TfaultC_adr, float * motor_TfaultC_adr)
{
    volatile uint32_t temp32;
    volatile float tempf;
    volatile bool Eread_stat = 0;
    
    Eread_stat = Eread_stat;
    
    
    T_rpm_fault = *rpm_fault_adr;
    T_OV_fault = *OV_fault_adr;
    T_UV_fault = *UV_fault_adr;
    T_Iph_fault = *Iph_fault_adr;
    T_Ibat_fault = *Ibat_fault_adr;
    T_ESC_TfaultC = *ESC_TfaultC_adr;
    T_motor_TfaultC = *motor_TfaultC_adr;
    
    if(ESCid > 0)
    {
        //6==========================================================================
        Eread_stat = EEPROM_WordRead(0x6C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_rpm_fault = fabs(tempf);    

        //7==========================================================================
        Eread_stat = EEPROM_WordRead(0x74, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_motor_TfaultC = fabs(tempf);

        Eread_stat = EEPROM_WordRead(0x7C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_ESC_TfaultC = fabs(tempf);
        if(T_ESC_TfaultC > ESC_TFLTC_MAX) T_ESC_TfaultC = ESC_TFLTC_MAX;

        //8==========================================================================
        Eread_stat = EEPROM_WordRead(0x88, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_OV_fault = fabs(tempf);
        if(T_OV_fault > OVFLT_MAX) T_OV_fault = OVFLT_MAX;

        Eread_stat = EEPROM_WordRead(0x8C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_UV_fault = fabs(tempf);
        if(T_UV_fault > T_OV_fault) T_UV_fault = T_OV_fault - 10;
         
        //9==========================================================================
        Eread_stat = EEPROM_WordRead(0x98, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Iph_fault = fabs(tempf);
        if(T_Iph_fault > IPH_FLT_MAX) T_Iph_fault = IPH_FLT_MAX; 
        T_Iph_fault = T_Iph_fault*1.414214;

        Eread_stat = EEPROM_WordRead(0x9C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Ibat_fault = fabs(tempf);
        if(T_Ibat_fault > IBAT_FLT_MAX) T_Ibat_fault = IBAT_FLT_MAX;
    }
    
    *rpm_fault_adr = T_rpm_fault;
    *OV_fault_adr = T_OV_fault;
    *UV_fault_adr = T_UV_fault;
    *Iph_fault_adr = T_Iph_fault;
    *Ibat_fault_adr = T_Ibat_fault;
    *ESC_TfaultC_adr = T_ESC_TfaultC;
    *motor_TfaultC_adr = T_motor_TfaultC; 
}

void tune_powertrain_variables(int *poles_adr, float * Iph_max_adr, float * rpm_max_adr, float *rpm_to_kmph_adr,  float * motor_derateC_adr, float * ESC_derateC_adr, float * Vbat_max_adr, float * Vbat_min_adr, float * Ibat_max_adr, float * Ibat_regen_adr)
{
    volatile uint32_t temp32;
    volatile float tempf;
    volatile bool Eread_stat = 0;
    
    Eread_stat = Eread_stat;
    
    T_poles = *poles_adr; 
    T_Iph_max = *Iph_max_adr;
    T_rpm_max = *rpm_max_adr;
    T_rpm_to_kmph = *rpm_to_kmph_adr;
    T_motor_derateC = *motor_derateC_adr;
    T_ESC_derateC = *ESC_derateC_adr;
    T_Vbat_max = *Vbat_max_adr;
    T_Vbat_min = *Vbat_min_adr;
    T_Ibat_max = *Ibat_max_adr;
    T_Ibat_regen = *Ibat_regen_adr;
    
        
    if(ESCid > 0)
    {
        //1==========================================================================
        Eread_stat = EEPROM_WordRead(0x14, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Iph_max = fabs(tempf);
        if(T_Iph_max > IPH_MAX_HLIMIT) T_Iph_max = IPH_MAX_HLIMIT;
        
        Eread_stat = EEPROM_WordRead(0x18, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_rpm_max = fabs(tempf);
        
        Eread_stat = EEPROM_WordRead(0x1C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_poles = (int)(fabs(tempf));
        
        //6==========================================================================
        Eread_stat = EEPROM_WordRead(0x68, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_rpm_to_kmph = fabs(tempf);
        
        //7==========================================================================
        Eread_stat = EEPROM_WordRead(0x70, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_motor_derateC = fabs(tempf);        
        
        Eread_stat = EEPROM_WordRead(0x78, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_ESC_derateC = fabs(tempf);
        
        //8==========================================================================
        Eread_stat = EEPROM_WordRead(0x80, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Vbat_max = fabs(tempf);        
        
        Eread_stat = EEPROM_WordRead(0x84, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Vbat_min = fabs(tempf);
        
        //9==========================================================================
        Eread_stat = EEPROM_WordRead(0x90, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Ibat_max = fabs(tempf);        
        
        Eread_stat = EEPROM_WordRead(0x94, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Ibat_regen = fabs(tempf);
               
    }    
    
    *poles_adr = T_poles; 
    *Iph_max_adr = T_Iph_max;
    *rpm_max_adr = T_rpm_max;
    *rpm_to_kmph_adr = T_rpm_to_kmph;
    *motor_derateC_adr = T_motor_derateC;
    *ESC_derateC_adr = T_ESC_derateC;
    *Vbat_max_adr = T_Vbat_max;
    *Vbat_min_adr = T_Vbat_min;
    *Ibat_max_adr = T_Ibat_max;
    *Ibat_regen_adr = T_Ibat_regen;
}

void tune_vehicle_veriables(int * driving_mode_adr, float * immob_rpm_adr, float * Leco_rpm_adr, float * eco_rpm_adr, float * throttle_zero_adr, float * throttle_max_adr, float * Vbrake_derate_adr, float * reverse_rpm_adr, float * kp_rpm_adr, float * ki_rpm_adr, float * kp_torque_adr, float * ki_torque_adr)
{
    volatile uint32_t temp32;
    volatile float tempf;
    volatile bool Eread_stat = 0;
    
    Eread_stat = Eread_stat;
    
    T_driving_mode = *driving_mode_adr; 
    T_immob_rpm = *immob_rpm_adr;
    T_Leco_rpm = *Leco_rpm_adr;
    T_eco_rpm = *eco_rpm_adr;
    
    T_throttle_zero = *throttle_zero_adr;
    T_throttle_max = *throttle_max_adr;
    
    T_Vbrake_derate = *Vbrake_derate_adr;
    T_reverse_rpm = *reverse_rpm_adr;
    
    T_kp_rpm = *kp_rpm_adr;
    T_ki_rpm = *ki_rpm_adr;
    T_kp_torque = *kp_torque_adr;
    T_ki_torque = *ki_torque_adr;
    
       
    if(ESCid > 0)
    {
        //00==========================================================================
        Eread_stat = EEPROM_WordRead(0x00, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_driving_mode = (int)tempf;
        if(ESCid == 1000) T_driving_mode = 4; 
               
        Eread_stat = EEPROM_WordRead(0x04, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_immob_rpm = fabs(tempf);
        if(T_immob_rpm > 100) T_immob_rpm = 100;
        
        Eread_stat = EEPROM_WordRead(0x08, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Leco_rpm = fabs(tempf);
        if(T_Leco_rpm > 100) T_Leco_rpm = 100;
        
        Eread_stat = EEPROM_WordRead(0x0C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_eco_rpm = fabs(tempf);
        if(T_eco_rpm > 100) T_eco_rpm = 100;
        
        //03==========================================================================
        
        Eread_stat = EEPROM_WordRead(0x30, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_throttle_zero = fabs(tempf);
        
        if(T_throttle_zero > THROTTLE_ZERO_HL) T_throttle_zero = THROTTLE_ZERO_HL;
        if(T_throttle_zero < THROTTLE_ZERO_LL) T_throttle_zero = THROTTLE_ZERO_LL;
        
        Eread_stat = EEPROM_WordRead(0x34, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_throttle_max = fabs(tempf); 
        
        if(T_throttle_max > THROTTLE_MAX_HL) T_throttle_max = THROTTLE_MAX_HL;
        if(T_throttle_max < THROTTLE_MAX_LL) T_throttle_max = THROTTLE_MAX_LL;
            
        Eread_stat = EEPROM_WordRead(0x38, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_Vbrake_derate = fabs(tempf);
        if(T_Vbrake_derate > 80) T_Vbrake_derate = 80;
        
        Eread_stat = EEPROM_WordRead(0x3C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_reverse_rpm = fabs(tempf);
        if(T_reverse_rpm > 100) T_reverse_rpm = 100;
        
        
        //05==========================================================================
        Eread_stat = EEPROM_WordRead(0x50, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_kp_rpm = tempf;   
        
        if(T_kp_rpm < KP_RPM_MIN) T_kp_rpm = KP_RPM_MIN;
        if(T_kp_rpm > KP_RPM_MAX) T_kp_rpm = KP_RPM_MAX;
        
        Eread_stat = EEPROM_WordRead(0x54, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_ki_rpm = tempf;
                   
        if(T_ki_rpm < KI_RPM_MIN) T_ki_rpm = KI_RPM_MIN;
        if(T_ki_rpm > KI_RPM_MAX) T_ki_rpm = KI_RPM_MAX;
    
        
        /*Eread_stat = EEPROM_WordRead(0x58, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_kp_torque = tempf;        
        
        Eread_stat = EEPROM_WordRead(0x5C, (uint32_t *)&temp32);
        __builtin_memcpy((uint32_t *)&tempf, (uint32_t *)&temp32, 4);
        T_ki_torque = tempf;    */            
    }
    
    
    //===============================================================================
    //Return tuning if ESCid > 0 from EEPROM otherwise as it is (default values)
        
    *driving_mode_adr = T_driving_mode;
    *immob_rpm_adr = T_immob_rpm;
    
    *Leco_rpm_adr = T_Leco_rpm;
    *eco_rpm_adr = T_eco_rpm;
    
    *throttle_zero_adr = T_throttle_zero;
    *throttle_max_adr = T_throttle_max;
    
    *Vbrake_derate_adr = T_Vbrake_derate;
    *reverse_rpm_adr = T_reverse_rpm;
    
    *kp_rpm_adr = T_kp_rpm;
    *ki_rpm_adr = T_ki_rpm;
    *kp_torque_adr = T_kp_torque;
    *ki_torque_adr = T_ki_torque;   
}

void send_currently_used_values(uint8_t adr_HB)
{
    float tempf;
    
    //write address field
    
    send_tuning_data[0] = 0xFF;    
    send_tuning_data[1] = adr_HB;
    
    send_tuning_data[18] = 0xFF;
    send_tuning_data[19] = adr_HB;
    
    //write data based on address
    
    if(adr_HB == 0x01)
    {
        tempf = ESCid;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_Iph_max;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_rpm_max;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_poles;        
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);        
    }
    
    else if(adr_HB == 0x02)
    {
        tempf = T_Rph * 1000.0;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_Ld * 1000.0;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_Lq * 1000.0;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_PHIph * 1000.0;        
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }    

    
    /*else if(adr_HB == 03)
    {
        tempf = T_desired_drive_mode;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_dir_rot;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_zero_angle;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_dirMR;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }*/ 
    
    else if(adr_HB == 0x04)
    {
        tempf = T_zero_angle;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_sensor_direction;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_zero_angle_measured;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_sensor_direction_measured;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }
    
    else if(adr_HB == 0x06)
    {
        tempf = T_rotation_direction;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_ang_margine;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_rpm_to_kmph;
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_rpm_fault;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }    
    
    else if(adr_HB == 0x07)
    {
        tempf = T_motor_derateC;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_motor_TfaultC;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_ESC_derateC;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_ESC_TfaultC;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    } 
    
    else if(adr_HB == 0x08)
    {
        tempf = T_Vbat_max;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_Vbat_min;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_OV_fault;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_UV_fault;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }
    
    else if(adr_HB == 0x09)
    {
        tempf = T_Ibat_max;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_Ibat_regen;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_Iph_fault/1.414214;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_Ibat_fault;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }
    
    else if(adr_HB == 0x00)
    {
        tempf = T_driving_mode;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_immob_rpm;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_Leco_rpm;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_eco_rpm;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }  
    
    else if(adr_HB == 0x03)
    {
        tempf = T_throttle_zero;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_throttle_max;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = T_Vbrake_derate;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = T_reverse_rpm;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }
    
    else if(adr_HB == 0x05)
    {
        tempf = T_kp_rpm;    
        __builtin_memcpy((uint8_t *)(send_tuning_data + 2), &tempf, 4);

        tempf = T_ki_rpm;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 6), &tempf, 4);

        tempf = 0;//T_BrakeEN;   
        __builtin_memcpy((uint8_t *)(send_tuning_data + 10), &tempf, 4);

        tempf = 0;//T_;      
        __builtin_memcpy((uint8_t *)(send_tuning_data + 14), &tempf, 4);
    }     
    
    else
    {
        send_tuning_data[18] = 0xF1;
    }
    
    send_data_uart(&send_tuning_data[0], 20);
}

void update_EEPROM_values(void)                 //Just store desired values in memory
{
    volatile uint32_t v1, v2, v3, v4;
    volatile uint32_t EEadr = 0;
    volatile bool Ewrite_stat = 0;
    
    Ewrite_stat = Ewrite_stat;
    
    EEadr = read_tuning_data[1];
    EEadr = EEadr << 4;                 //address is high nibble as each adress should have 16 bytes for 4 floats

    __builtin_memcpy((uint8_t *)&v1, (uint8_t *)(read_tuning_data + 2), 4);
    Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
    EEadr = EEadr + 0x04;
    
    EECONCLR = _EECON_WREN_MASK; 
    
    __builtin_memcpy((uint8_t *)&v2, (uint8_t *)(read_tuning_data + 6), 4);
    Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
    EEadr = EEadr + 0x04;
    
    EECONCLR = _EECON_WREN_MASK; 
    
    __builtin_memcpy((uint8_t *)&v3, (uint8_t *)(read_tuning_data + 10), 4);
    Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
    EEadr = EEadr + 0x04;
    
    EECONCLR = _EECON_WREN_MASK; 
    
    __builtin_memcpy((uint8_t *)&v4, (uint8_t *)(read_tuning_data + 14), 4);
    Ewrite_stat = EEPROM_WordWrite(EEadr, v4);

    EECONCLR = _EECON_WREN_MASK;                        // Turn off writes or read won't work
}

void tune_command(uint8_t * command_adr)                //decipher the tuning command
{
    __builtin_memcpy((uint8_t *)&read_tuning_data, (uint8_t *)command_adr, 20); //copy command
    if(*(read_tuning_data+18) == 0xff)
    {
        if(*(read_tuning_data+1) == *(read_tuning_data+19))
        update_EEPROM_values();     
        send_data_uart(&read_tuning_data[0], 20);            //acknowledgement
    }
    else
    {
        send_currently_used_values(*(read_tuning_data + 1));                    //currently used values
    }    
}

void fill_empty_EEPROM(void)
{
    volatile uint32_t v1, v2, v3, v4;
    volatile uint32_t EEadr = 0, temp32;
    bool Ewrite_stat = 0, Eread_stat = 0;
    
    float tempf;
    Eread_stat = Eread_stat;
    Eread_stat = EEPROM_WordRead(0x10, (uint32_t *)&temp32);
    
    if(temp32 == 0xFFFFFFFF)
    {
        Ewrite_stat = Ewrite_stat;

        //1======================================================================
        EEadr = 1;
        EEadr = EEadr << 4;                 //address is high nibble as each address should have 16 bytes for 4 floats

        tempf = -51;    //Default ESCid can relate to default config                     
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = IPH_MAX;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = RPM_MAX;    
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = POLES;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; 
        
        //2======================================================================
        EEadr = 2;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = RPH * 1000.0;                          
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = LD * 1000.0;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = LQ * 1000.0;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = PHIPH * 1000.0;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK;  
        
        //3======================================================================
        /*EEadr = 3;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = 1;    //calib                     
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = 0;     //calib
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = 310;     //calib
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = 0;    //calib
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; */
        
        //4======================================================================
        EEadr = 4;
        EEadr = EEadr << 4;                 //address is high byte as each address should have 16 bytes for 4 floats

        tempf = ZERO_ANGLE;                     
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = SENSOR_DIRECTION;    
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        /*tempf = 310;     //zero_angle
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = 0;     //sensor orientation
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; */
        
        
        //6======================================================================
        EEadr = 6;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = ROTATION_DIRECTION;                      
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = ANG_MARGINE;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = RPM_TO_KMPH;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = RPM_FLT;    
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; 
        
                
        //7======================================================================
        EEadr = 7;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = MOTOR_DERATEC;                        
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = MOTOR_TFLTC;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = ESC_DERATEC;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = ESC_TFLTC;    
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; 
        
        //8======================================================================
        EEadr = 8;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = VBAT_MAX;                       
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = VBAT_MIN;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = OVFLT;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = UVFLT;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK;  
        
        
        //9======================================================================
        EEadr = 9;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = IBAT_MAX;                       
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = IBAT_REGEN;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = IPHFLT;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = IBATFLT;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; 
        
        //B======================================================================
        EEadr = 0x00;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = DRIVING_MODE;                       
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = IMMOB_RPM;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = LECO_RPM;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = ECO_RPM;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; 
        
        //C======================================================================
        EEadr = 0x03;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = THROTTLE_ZERO;                       
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = THROTTLE_MAX;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = VBRAKE_DERATE;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = REVERSE_RPM;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK;
        
        //D======================================================================
        EEadr = 0x05;
        EEadr = EEadr << 4;                 //address is high byte as each adress should have 16 bytes for 4 floats

        tempf = KP_RPM;                       
        __builtin_memcpy((uint8_t *)&v1, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v1);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        tempf = KI_RPM;     
        __builtin_memcpy((uint8_t *)&v2, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v2);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 

        /*tempf = 0.1;     
        __builtin_memcpy((uint8_t *)&v3, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v3);
        EEadr = EEadr + 0x04;
        EECONCLR = _EECON_WREN_MASK; 
        
        tempf = 0.2;     
        __builtin_memcpy((uint8_t *)&v4, &tempf, 4);
        Ewrite_stat = EEPROM_WordWrite(EEadr, v4);
        EECONCLR = _EECON_WREN_MASK; */
    }
            
}