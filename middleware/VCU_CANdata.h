#include "device.h"

#ifndef _VCU_CANDATA_H    /* Guard against multiple inclusion */
#define _VCU_CANDATA_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void fill_data_for_VCU(float, float, float, float, float, float, float, int);
    void A1_action(uint8_t *, uint8_t);
    void A3_action(uint8_t *, uint8_t);
    void send_A2data(void);
    void send_A4data(void);
    
    void reverse_indicator(int);
    void fault_indicator(int);
    void side_stand_indicator(int);
    void brake_indicator(int);
    void Lmode_indicator(int);
    void Hmode_indicator(int);
    void Parkmode_indicator(int);
    

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
