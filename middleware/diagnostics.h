
#ifndef DIAGNOSTICS_H
#define	DIAGNOSTICS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "stdint.h"
void fill_diag_data(int, float fd_Vdc, float fd_Idc, float fd_rpm, float fd_Id, float fd_Tesc, float fd_Tmot);
void send_diag_data(void);
void fill_TU_data1(float, float, float, float, float, float);
void fill_TU_data2(float, float, float, float, float, float);
void C0_send_TU_data_CAN(void);


#ifdef	__cplusplus
}
#endif

#endif
