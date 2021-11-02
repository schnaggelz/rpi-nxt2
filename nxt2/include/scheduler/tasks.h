#ifndef __TASKS_H__
#define __TASKS_H__

#include "systypes.h"

#define OS_IDLE_TASK_ID 255

typedef void (*os_callback_t)(void);

typedef enum
{
    OS_TASK_STATE_IDLE = 0,
    OS_TASK_STATE_RUNNING = 1
}
os_task_state_e;

/* Task configuration structure. */
typedef struct task
{
   sint32 task_state; /* current task state (idle|run)   */
   uint32 task_count; /* number of (periodic) task ticks */
   uint32 tick_rate;  /* time period (tick rate) in ms   */
   uint32 task_time;  /* time since task's previous tick */
   uint32 exec_time;  /* time used by callback function  */  
   os_callback_t callback;    /* tick call-back function */
} 
os_task_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Application background task. */
extern void app_bg_task();

#ifdef __cplusplus
}
#endif

#endif /*__TASKS_H__*/
