/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "tasks.h"

#include "platform/systypes.h"

#define OS_MAX_TASKS 9
#define OS_TICK_RATE 1    /* 1 ms */

#ifdef __cplusplus
extern "C" {
#endif

/* OS task setup using unique priorities (=indexes) */
void os_add_task(os_callback_t cbk, uint8 idx, uint32 rate);

/* OS task counter */
uint32 os_get_task_count(uint8 idx);

#ifdef __cplusplus
}
#endif

/* Scheduling function */
void os_schedule();

/* Status information */
sint32 os_scheduler_overruns();

#endif /*__SCHEDULER_H__*/
