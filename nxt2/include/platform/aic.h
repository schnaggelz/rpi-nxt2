/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Driver for the AT91SAM7's Advanced Interrupt Controller (AIC).
*
* The AIC is responsible for queuing interrupts from other
* peripherals on the board. It then hands them one by one to the ARM
* CPU core for handling, according to each peripheral's configured
* priority.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __AIC_H__
#define __AIC_H__

#include "systypes.h"

void aic_init(void);
void aic_set_vector(uint32 vector, uint32 mode, uint32 isr);
void aic_mask_on(uint32 vector);
void aic_mask_off(uint32 vector);
void aic_clear(uint32 mask);
void aic_set(uint32 mask);

/* Priority levels for interrupt lines. */
#define AIC_INT_LEVEL_LOWEST 1
#define AIC_INT_LEVEL_LOW    2
#define AIC_INT_LEVEL_NORMAL 4
#define AIC_INT_LEVEL_ABOVE_NORMAL 5
#define AIC_INT_LEVEL_HIGH   7

#endif
