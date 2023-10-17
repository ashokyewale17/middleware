#include "math.h"
#include "stdlib.h"


float find_ang(float cosx, float sinx)
{
    float x = 0;
    float x2,x3,x5,x7,x9,x11,x13;
    float ang_offset;
    
    if(fabs(cosx) < fabs(sinx)) 
    {
        x = fabs(cosx); 
        ang_offset = 1.570796327;
    }
    else
    {
        x = -fabs(sinx);
        ang_offset = 0;
    }

    x2 = x * x;
    x3 = x * x2;
    x5 = x3 * x2;
    x7 = x5 * x2;
    x9 = x7 * x2;
    x11 = x9 * x2;
    x13 = x11 * x2;
    
    x = x + x3 * 0.166666667 + x5 * 0.075000000 + x7 * 0.044642857 + x9 * 0.030381944 + x11 * 0.022372159 + x13 * 0.017352764;
    x = ang_offset - x;
    
    if((cosx >= 0) & (sinx >= 0)) x = x;
    else if((cosx < 0) & (sinx >= 0)) x = 3.141592654 - x;

    else if ((cosx < 0) & (sinx < 0)) x = 3.141592654 + x;
    else x = 6.283185307 - x;
    
    x = x*57.29577951; //degree conv.
    
    return x;
}