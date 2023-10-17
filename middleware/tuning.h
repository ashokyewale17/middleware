
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
    
    void tune_state_variables(float *, float *, float *, float *, float *, int*,  float *, int *);
    //void tune_state_variables(float * Rph_adr, float * Ld_adr, float * Lq_adr, float * PHIph_adr, float * zero_angle_adr, int * sensor_direction_adr, float * ang_margine_adr, int * rotation_direction_adr)
    
    void tune_faults(float *, float * , float * , float * , float * , float * , float * );
    //void tune_faults(float * speed_fault_adr, float * OV_fault_adr, float * UV_fault_adr, float * Iph_fault_adr, float * Ibat_fault_adr, float * ESC_TfaultC_adr, float * motor_TfaultC_adr)
    
    void tune_powertrain_variables(int *, float *, float *, float *, float *, float *, float *, float *, float *, float *);
    //void tune_powertrain_variables(int *poles_adr, float * Iph_max_adr, float * speed_max_adr, float * rpm_to_kmph_adr, float motor_derateC_adr, float ESC_derateC_adr, float Vbat_max_adr, float Vbat_min_adr, float Ibat_max_adr, float Ibat_regen_adr)

    void tune_vehicle_veriables(int *, float *, float*, float *, float *, float *, float *, float *, float *, float *, float *, float *);
    //void tune_vehicle_veriables(int * driving_mode_adr, float * immob_speed_adr, float * eco_current_adr, float * eco_speed_adr, float * throttle_zero_adr, float * throttle_max_adr, float brake_current_adr, float reverse_speed_adr, float * kp_speed_adr, float * ki_speed_adr, float * kp_torque_adr, float * ki_torque_adr )
    
    void send_measured_angle(float, float, float, int); //special function to send detected angles after rotor position caliberation
    //Theta at 45, theta at 135, corrected theta at 45, corrected MR orientation
    
    void fill_empty_EEPROM(void);
    

#ifdef __cplusplus
}
#endif

#endif /* _TUNING_H */

