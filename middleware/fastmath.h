

#ifndef _FASTMATH_H    /* Guard against multiple inclusion */
#define _FASTMATH_H


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    //returns theta in degrees (0 to 360)
    float find_ang(float, float); //cos, sin (unit magnitude)
    
#ifdef __cplusplus
}
#endif

#endif /* _FASTMATH_H */

