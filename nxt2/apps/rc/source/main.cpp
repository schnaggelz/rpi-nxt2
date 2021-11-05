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

#include "monitor/monitor.hpp"

#include "remote.hpp"

nxt::libs::Monitor monitor;
nxt::apps::Remote remote;

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

void app_bg_task()
{
   static int counter = 0;

   if (counter % 100 == 0)
   {
       monitor.setLineValue(0, 0);
       monitor.setLineValue(1, 0);
       monitor.setLineValue(2, 0);
       monitor.setLineValue(3, 0);
       monitor.setLineValue(4, 0);
       monitor.setLineValue(5, 0);
       monitor.setLineValue(6, 0);

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
   monitor.setTitle("REMOTE CONTROL 1");

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
