#include "../../include/scheduler/tasks.h"

extern void nxt_bg_task(void);

/* Background processing called by the 1kHz 
   timer interrupt hander. */
void bg_task()
{
    /* System background process. */
    nxt_bg_task();

    /* Application background process. */
    app_bg_task();
}
