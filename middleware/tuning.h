
#ifndef _TUNING_H    /* Guard against multiple inclusion */
#define _TUNING_H

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


#ifdef __cplusplus
extern "C" {
#endif
    
    
    void update_EEPROM_values(void);        //updates specific locations of EEPROM
                                            //location address and data is available in read_tuning data array
        
    void send_currently_used_values(uint8_t);   //prepares send_tuning_data array to be sent over UART
    //unit8t EEPROM_Address_corresponding_to_values_to_be_seen
    
    void tune_command(uint8_t *);           //decode the command received
    //address of command array coming from UART driver
    
    void tune_state_variables(float *, float *, float *, float *, float *, int*, int *);
    //&Rph, &Ld, &Lq, &PHIph, &zero_angle, &sensor_direction, &rotation_direction
    
    void tune_faults(float *, float * , float * , float * , float * , float * , float * );
    //&rpm_fault, &OV_fault, &UV_fault, &Iph_fault, &Ibat_fault, &ESC_TfaultC, &motor_TfaultC
    
    void tune_powertrain_variables(int *, float *, float *, float *, float *, float *, float *, float *, float *, float *, int *);
    //&poles, &Iph_max, &rpm_max, &rpm_to_kmph, &motor_derateC, &ESC_derateC, &Vbat_max, &Vbat_min, &Ibat_max, &Ibat_regen, &CAN_baud

    void tune_vehicle_veriables(int *, float*, float *, float *, float *, float *, float *, float *, float *, float *, float *, float *, float *, float *, float *, float * );
    //&driving_mode, &reverse_rpm, &L_rpm, &M_rpm, &throttle_zero, &throttle_max, &Vbrake_derate, &k_auto_brake, &kp_rpm, &ki_rpm, &L_acceleration, &M_acceleration, &L_Ibat, &M_Ibat, &L_Iph, &M_Iph
    
    void send_measured_angle(float, float, float, int); //special function to send detected angles after rotor position caliberation
    //Theta at 45, theta at 135, corrected theta at 45, corrected MR orientation
    
    void fill_empty_EEPROM(void);
    

#ifdef __cplusplus
}
#endif

#endif /* _TUNING_H */

