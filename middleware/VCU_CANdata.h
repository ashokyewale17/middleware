#include "device.h"

#ifndef _VCU_CANDATA_H    /* Guard against multiple inclusion */
#define _VCU_CANDATA_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void fill_data_for_VCU(float, float, float, float, float, int);
    void A1_action(uint8_t *, uint8_t);
    void A3_action(uint8_t *, uint8_t);
    void send_A2data(void);
    void send_A4data(void);
    void CAN_Vmode(float, float, float *, float *);
    //void CAN_Vmode(float speed_max, float rpm_to_speed, float *VM_speed_limit_adr, float *Id_limit_N_adr, float *VM_Iregen_adr)
   
    void CAN_init(float);
    
    void CAN_reverse(float , float , float , float *);
    //Iph_max, rpm_max, reverse_rpm, *VM_rpm_limit_adr, * Id_limit_N_adr) 
    
    void CAN_immob_EN(float, float *);
    //void GPIO_immob_EN(float Iph_max, float *VM_rpm_limit_adr, float * Id_limit_N_adr)
    
    void CAN_immob_DIS(float *);
    //void GPIO_immob_DIS(float *VM_rpm_limit_adr)
    
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
