/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Provides a 1000Hz tick for the system.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef  __SYSTICK_H__
#define __SYSTICK_H__

#include "systypes.h"

#define CLOCK_FREQUENCY 48054850

#ifdef __cplusplus
extern "C" {
#endif

void systick_init(void);
void systick_wait_ms(uint32 ms);
void systick_wait_ns(uint32 ns);
void systick_test(void);
void systick_suspend(void);
void systick_resume(void);

uint32 systick_get_ms(void);
uint64 systick_get_ns(void);

#ifdef __cplusplus
}
#endif

#endif
