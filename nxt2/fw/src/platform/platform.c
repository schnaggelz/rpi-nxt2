/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/platform.h"

#include "platform/aic.h"
#include "platform/i2c.h"
#include "platform/irqs.h"
#include "platform/systick.h"
#include "platform/twi.h"

void at91_platform_init()
{
    /* Init interrupt controller. */
    aic_init();

    /* Enable all interrupts. */
    irqs_enable();

    /* Initialize SPI. */
    twi_init();

    /* Initialize timer. */
    systick_init();

    /* Initialize I2C. */
    i2c_init();
}
