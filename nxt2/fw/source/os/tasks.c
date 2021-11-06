/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

extern void nxt_bg_task(void);
extern void app_bg_task(void);

#ifdef NXT_DUMMY_APP_BG_TASK
void app_bg_task(void) {};
#endif

/* Background processing called by the 1kHz
   timer interrupt handler. */
void bg_task()
{
    /* System background process. */
    nxt_bg_task();

    /* Application background process. */
    app_bg_task();
}
