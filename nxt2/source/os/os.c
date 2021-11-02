/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Simple run-to-completion tasker. Not really an OS, but we name it like that.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "os/os.h"

#include "platform/systick.h"

/* Flag to notify OS has been started. */
static volatile uint8 os_started;

void os_init(void)
{
    os_started = 0;
}

void os_start(void)
{
    /* Application startup. */
    os_app_init();

    /* OS now running. */
    os_started = 1;

    while (1)
    {
        /* Idle loop. */
        systick_wait_ms(100);
    }
}

void os_tick(void)
{
    os_schedule();
}

uint8 os_running(void)
{
    return os_started;
}
