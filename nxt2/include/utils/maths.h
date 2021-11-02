#ifndef __MATHS_H__
#define __MATHS_H__

#include "systypes.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

inline uint32 calcAvg(sint32* ary, uint16 size)
{
    uint32 idx = 0;
    sint32 sum = 0;

    for(; idx < size; ++idx)
    {
        sum += ary[idx];
    }

    return sum / size;  
}

#endif /* __MATHS_H__ */
