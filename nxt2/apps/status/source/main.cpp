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

void app_bg_task()
{
    static int counter = 0;

    if (counter % 100 == 0)
    {
        status_monitor.update();
    }
};
void os_app_init()
{
    // Setup OS tasks
    addTasks();

    // Set up our application display
    status_monitor.setTitle("GENERIC TESTER 3");

    status_monitor.setLineName(0, "0:");
    status_monitor.setLineName(1, "1:");
    status_monitor.setLineName(2, "2:");
    status_monitor.setLineName(3, "3:");
    status_monitor.setLineName(4, "4:");
    status_monitor.setLineName(5, "5:");
    status_monitor.setLineName(6, "6:");

    // Set up status display
    status_monitor.init();

    // Update the display once
    status_monitor.update();
};
}
