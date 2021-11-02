/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "os/scheduler.h"

#include "platform/systypes.h"

/* Index of the highest priority task. */
uint8 current_task = 0;

/* Busy state */
uint8 scheduler_busy = 0;

/* Running tasks, [0] always idle task. */
uint8 running_tasks[OS_MAX_TASKS] = { OS_IDLE_TASK_ID };

/* Scheduler configuration structure */
typedef struct os_scheduler 
{
    uint8 num_tasks; /* number of tasks in array */
    os_task_t tasks[OS_MAX_TASKS];    /* tasks to schedule */
} 
os_scheduler_t;

/* Singleton scheduler. */
os_scheduler_t scheduler;

void os_schedule()
{
    uint8 idx = 0;

    for (idx = 0; idx < scheduler.num_tasks; ++idx) 
    {
        /* If task has higher priority, is ready and not running */
        if ((running_tasks[current_task] > idx) && 
            (scheduler.tasks[idx].task_time >= scheduler.tasks[idx].tick_rate) &&
            (scheduler.tasks[idx].task_state != OS_TASK_STATE_RUNNING))
        {
            /* Start administrative work, disable interrupts. */
            //enter_critical_section();

            /* Reset time since last tick. */
            scheduler.tasks[idx].task_time = 0;

            /* Mark task as running. */
            scheduler.tasks[idx].task_state = OS_TASK_STATE_RUNNING;

            /* Select this task. */
            current_task += 1;

            /* Add it to the running tasks. */
            running_tasks[current_task] = idx;

            /* End administrative work, enable interrupts. */
            //leave_critical_section();

            /* Execute task tick function. */
            scheduler.tasks[idx].callback();  

            /* Start administrative work, disable interrupts. */
            //enter_critical_section();

            /* Mark task as not running. */
            scheduler.tasks[idx].task_state = OS_TASK_STATE_IDLE;

            /* Remember the numer of ticks for debugging purposes. */
            scheduler.tasks[idx].task_count ++;

            /* Remove it from the running tasks. */
            running_tasks[current_task] = OS_IDLE_TASK_ID;

            /* Un-select this task. */
            current_task -= 1;

            /* End administrative work, enable interrupts. */
            //leave_critical_section();
        }
        scheduler.tasks[idx].task_time += OS_TICK_RATE;
    }
}

/* Task setup. */
void os_add_task(os_callback_t cbk, uint8 idx, uint32 rate)
{
    if (idx >= OS_MAX_TASKS)
        return;
    
    scheduler.tasks[idx].tick_rate = rate;
    scheduler.tasks[idx].task_time = rate;
    scheduler.tasks[idx].task_state = OS_TASK_STATE_IDLE;
    scheduler.tasks[idx].callback = cbk;

    scheduler.num_tasks ++;
}

uint32 os_get_task_count(uint8 idx)
{
    if (idx >= OS_MAX_TASKS)
        return 0;

    return scheduler.tasks[idx].task_count;
}
