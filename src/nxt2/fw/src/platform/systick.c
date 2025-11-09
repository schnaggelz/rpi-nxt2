/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
 *
 * Provides a 1000Hz tick for the system.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "platform/systick.h"

#include "platform/aic.h"
#include "platform/irqs.h"

#include "platform/at91/at91sam7s256.h"

#define CLK_FREQ 48054841
#define PIT_FREQ 1000 /* Hz */
#define LOW_PRIORITY_IRQ 10

extern void os_tick(void);
extern void os_background(void);

static volatile uint32 systick_ms = 0;

void systick_isr_handler(void)
{
    uint32 status;

    /* Read status to confirm interrupt. */
    status = *AT91C_PITC_PIVR;

    /* Update with number of ticks since last time. */
    systick_ms += (status & AT91C_PITC_PICNT) >> 20;

    /* Trigger low priority task. */
    aic_set(LOW_PRIORITY_IRQ);
}

void systick_lp_isr_handler(void)
{
    /* Clear low priority task interrupt. */
    aic_clear(LOW_PRIORITY_IRQ);

    /* Allow nesting. */
    enable_all_interrupts();

    /* Call background task. */
    os_background();

    /* Call OS base tick function. */
    os_tick();

    /* Disallow nesting. */
    disable_all_interrupts();
}

uint32 systick_get_ms(void)
{
    /* We're using a 32-bitter and can assume that we
       don't need to do any locking here. */
    return systick_ms;
}

uint64 systick_get_ns(void)
{
    uint32 ms;
    uint32 piir;
    uint32 ns;

    do
    {
        ms = systick_ms;
        piir = *AT91C_PITC_PIIR;
    } while (systick_ms != ms);

    /* Add in any missed ms. */
    ms += (piir >> 20);

    /* Get nanoseconds. */
    ns = ((piir & AT91C_PITC_CPIV) * PIT_FREQ) / (CLK_FREQ / 16 / 1000000);
    return (uint64)ms * 1000000 + ns;
}

void systick_wait_ms(uint32 ms)
{
    volatile uint32 final = ms + systick_ms;
    while (systick_ms < final)
    {
    }
}

void systick_wait_ns(uint32 ns)
{
    volatile uint32 x = (ns >> 7) + 1;
    while (x)
    {
        x--;
    }
}

void systick_init(void)
{
    int i_state = irqs_get_and_disable();

    /* Install low priority handler. */
    aic_mask_off(LOW_PRIORITY_IRQ);
    aic_set_vector(LOW_PRIORITY_IRQ,
                   AT91C_AIC_SRCTYPE_INT_EDGE_TRIGGERED | AIC_INT_LEVEL_LOW,
                   (uint32)systick_lp_isr_handler);
    aic_mask_on(LOW_PRIORITY_IRQ);

    /* Set up periodic interval timer. */
    *AT91C_PITC_PIMR = AT91C_PITC_PITIEN |               /* interrupt enable */
                       AT91C_PITC_PITEN |                /* timer enable */
                       ((CLK_FREQ / PIT_FREQ / 16) - 1); /* interval value */

    /* Install main timer handler. */
    aic_mask_off(AT91C_ID_SYS);
    aic_set_vector(AT91C_ID_SYS,
                   AT91C_AIC_SRCTYPE_INT_EDGE_TRIGGERED | AIC_INT_LEVEL_NORMAL,
                   (uint32)systick_isr_handler);
    aic_mask_on(AT91C_ID_SYS);

    if (i_state)
        irqs_enable();
}

void systick_suspend(void)
{
    aic_mask_off(LOW_PRIORITY_IRQ);
}

void systick_resume(void)
{
    aic_mask_on(LOW_PRIORITY_IRQ);
}
