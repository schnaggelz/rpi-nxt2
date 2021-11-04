/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __OS_H__
#define __OS_H__

#include "scheduler.h"
#include "tasks.h"

#include "platform/systypes.h"
#include "platform/systick.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/* OS control functions. */
void os_init(void);
void os_start(void);
void os_tick(void);

uint8 os_running(void);

#endif /*__OS_H__*/