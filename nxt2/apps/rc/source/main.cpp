/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * Remote control application. Brick is controlled by a Raspberry Pi.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "os/os.h"

#include "wrappers/monitor.hpp"

#include "remote.hpp"

nxt::wrappers::Monitor monitor;
nxt::apps::Remote remote(monitor);

//
// Runnable scheduling
//

void taskCbk10ms()
{
    // TODO
}

void taskCbk20ms()
{
    // TODO
}

void taskCbk50ms()
{
    remote.run();
}

static void addTasks()
{
    os_add_task(&taskCbk10ms, 0, 10);
    os_add_task(&taskCbk20ms, 1, 20);
    os_add_task(&taskCbk50ms, 2, 50);
}

void setupMonitor()
{
    // Set up our application display
    monitor.setTitle("REMOTE CONTROL 1");

    monitor.setLineName(0, "M1:");
    monitor.setLineName(1, "M2:");
    monitor.setLineName(2, "M3:");
    monitor.setLineName(3, "S1:");
    monitor.setLineName(4, "S2:");
    monitor.setLineName(5, "S3:");
    monitor.setLineName(6, "S4:");

    monitor.setLineValue(0, 0);
    monitor.setLineValue(1, 0);
    monitor.setLineValue(2, 0);
    monitor.setLineValue(3, 0);
    monitor.setLineValue(4, 0);
    monitor.setLineValue(5, 0);
    monitor.setLineValue(6, 0);

    // Set up status display
    monitor.init();

    // Update the display once
    monitor.update();
}

extern "C" {

//
// Background processing
//

void app_bg_task()
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

    // Setup display
    setupMonitor();

    // Init RC
    remote.init();
}
}

