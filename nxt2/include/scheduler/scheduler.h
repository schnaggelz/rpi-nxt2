#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "systypes.h"
#include "tasks.h"

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
