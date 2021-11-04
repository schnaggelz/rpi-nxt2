/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Initialisation for ARM7
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "os/os.h"

#include "platform/platform.h"

/* External init functions. */
extern void nxt_devices_init();

static void cpp_init()
{
    /* Start and end points of the constructor list,
       defined by the linker script. */
    extern void (*__init_array_start__)();
    extern void (*__init_array_end__)();

    /* Call each function in the list. We have to take the
       address of the symbols, as __init_array_start__ *is*
       the first function pointer, not the address of it. */
    for (void (**p)() = &__init_array_start__;
            p < &__init_array_end__; ++p)
    {
        (*p)();
    }
}

int main(void)
{
    /* C++ static constructor support. */
    cpp_init();

    /* Setup the OS. */
    os_init();

    /* Initialize NXT devices. */
    nxt_devices_init();

    /* Start the OS. */
    os_start();

    /* Never reach here. */
    return 0;
}
