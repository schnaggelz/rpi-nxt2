/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

extern void nxt_devices_background(void);
extern void os_app_background(void);

#ifdef NXT_DUMMY_APP_BG_TASK
void os_app_background(void) {};
#endif

/* Background processing called by the 1kHz
   timer interrupt handler. */
void os_background()
{
    /* System background process. */
    nxt_devices_background();

    /* Application background process. */
    os_app_background();
}
