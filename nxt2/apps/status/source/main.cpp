/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "os/os.h"

#include "statmon/status_monitor.hpp"

nxt::libs::StatusMonitor status_monitor;

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

void app_bg_task(){};
void os_app_init()
{
    // Setup OS tasks
    addTasks();

    // Set up our application display
    status_monitor.setTitle("GENERIC TESTER 3");
};
}
