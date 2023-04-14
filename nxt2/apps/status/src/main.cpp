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

#include "utils/status_monitor.hpp"

#include "nxt/nxt_ht_gyro_sensor.hpp"
#include "nxt/nxt_touch_sensor.hpp"

nxt::app_utils::StatusMonitor monitor;

struct Sensors
{
    Sensors()
        : gyro_sensor(nxt::fw::Sensor::Port::PORT_1)
    {
        gyro_sensor.init();
    }

    void read()
    {
        gyro_sensor.read();
    }

    nxt::fw::ht::GyroSensor gyro_sensor;
};

Sensors sensors;

//
// Runnable scheduling
//

void taskCbk10ms() 
{
    sensors.read();
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

    if (counter % 10 == 0)
    {
        monitor.setLineValue(0, 0, sensors.gyro_sensor.getAnglarVelocity());
        monitor.setLineValue(1, 0, 0);
        monitor.setLineValue(2, 0, 0);
        monitor.setLineValue(3, 0, 0);
        monitor.setLineValue(4, 0, 0);
        monitor.setLineValue(5, 0, 0);
        monitor.setLineValue(6, 0, 0);

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
    monitor.setTitle("GENERIC STATUS 4");

    monitor.setLineName(0, "0:");
    monitor.setLineName(1, "1:");
    monitor.setLineName(2, "2:");
    monitor.setLineName(3, "3:");
    monitor.setLineName(4, "4:");
    monitor.setLineName(5, "5:");
    monitor.setLineName(6, "6:");

    // Set up status display
    monitor.init();

    // Update the display once
    monitor.update();
}
}
