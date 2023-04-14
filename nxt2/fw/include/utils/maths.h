/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __MATHS_H__
#define __MATHS_H__

#include "platform/systypes.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

inline uint32 calcAvg(sint32* ary, uint16 size)
{
    uint32 idx = 0;
    sint32 sum = 0;

    for (; idx < size; ++idx)
    {
        sum += ary[idx];
    }

    return sum / size;
}

sint32 sign(sint32 x)
{
    return (x > 0) - (x < 0);
}

#endif /* __MATHS_H__ */
