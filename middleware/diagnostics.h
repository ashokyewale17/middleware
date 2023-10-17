
#ifndef DIAGNOSTICS_H
#define	DIAGNOSTICS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "stdint.h"
void fill_diag_data(int, float fd_Vdc, float fd_Idc, float fd_rpm, float fd_Id, float fd_Tesc, float fd_Tmot);
void send_diag_data(void);



#ifdef	__cplusplus
}
#endif

#endif
