/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

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
