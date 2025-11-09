/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Status display application. Just for debugging.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "os/os.h"

//
// Runnable scheduling
//

void taskCbk10ms() {}

void taskCbk20ms() {}

void taskCbk50ms() {}

static void addTasks()
{
    os_add_task(&taskCbk10ms, 0, 10);
    os_add_task(&taskCbk20ms, 1, 20);
    os_add_task(&taskCbk50ms, 2, 50);
}

extern "C" {

//
// Background processing
//

void os_app_background()
{
}

//
// Runnable initialization
//

void os_app_init()
{
    // Setup OS tasks
    addTasks();

    // Set up our application display
    monitor.setTitle("TEMPLATE");
}
}
