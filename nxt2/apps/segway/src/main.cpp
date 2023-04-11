/*******************************************************************************
 * Copyright (C) 2023 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Status display application. Just for debugging.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "os/os.h"

#include "utils/status_monitor.hpp"

#include "segway.hpp"

nxt::app_utils::StatusMonitor monitor;

nxt::segway::Segway segway(10);

//
// Runnable scheduling
//

void taskCbk10ms() 
{
    segway.step();
}

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
    static int counter = 0;

    if (counter % 100 == 0)
    {
        monitor.update();
    }

    counter++;
}

//
// Runnable initialization
//

void os_app_init()
{
    // Setup OS tasks
    addTasks();

    // Set up our application display
    monitor.setTitle("SEGWAY V1");

    monitor.setLineName(0, "M1:");
    monitor.setLineName(1, "M2:");
    monitor.setLineName(2, "M3:");
    monitor.setLineName(3, "S1:");
    monitor.setLineName(4, "S2:");
    monitor.setLineName(5, "S3:");
    monitor.setLineName(6, "S4:");

    // Set up status display
    monitor.init();

    // Update the display once
    monitor.update();

    // Start application
    segway.init();
}
}
