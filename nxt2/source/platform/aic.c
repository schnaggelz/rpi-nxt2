/* Driver for the AT91SAM7's Advanced Interrupt Controller (AIC).
 *
 * The AIC is responsible for queuing interrupts from other
 * peripherals on the board. It then hands them one by one to the ARM
 * CPU core for handling, according to each peripheral's configured
 * priority. */

#include "platform/aic.h"

#include "platform/irqs.h"

#include "platform/at91/at91sam7.h"

/* Default handler installed for all vectors by default. */
static void aic_default_handler(void)
{
    while(1) {}; /* should never stay here */
}

/* Initialise the Advanced Interrupt Controller.
 *
 * Note that this function leaves interrupts disabled in the ARM core
 * when it returns, so that other board drivers may register interrupt
 * handlers safely.
 */
void aic_init(void)
{
    int i;

    /* Prevent the ARM core from being interrupted while we set up the
     * AIC. */
    irqs_get_and_disable();

    /* If we're coming from a warm boot, the AIC may be in a weird
     * state. Do some cleaning up to bring the AIC back into a known
     * state:
     *  - All interrupt lines disabled,
     *  - No interrupt lines handled by the FIQ handler,
     *  - No pending interrupts,
     *  - AIC idle, not handling an interrupt. */
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_FFDR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_ICCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_EOICR = 1;

    /* Enable debug protection. This is necessary for JTAG debugging, so
     * that the hardware debugger can read AIC registers without
     * triggering side-effects. */
    AT91C_BASE_AIC->AIC_DCR = 1;

    /* Set up the default AIC interrupts handler vectors. */
    for (i = 0; i < 32; i++)
    {
        AT91C_BASE_AIC->AIC_SMR[i] = 0;
        AT91C_BASE_AIC->AIC_SVR[i] = (uint32)aic_default_handler;
    }
    AT91C_BASE_AIC->AIC_SVR[AT91C_ID_FIQ] = (uint32)aic_default_handler;
    AT91C_BASE_AIC->AIC_SPU = (uint32)aic_default_handler;
}

/* Register an interrupt service routine for an interrupt line.
 *
 * Note that while this function registers the routine in the AIC, it
 * does not enable or disable the interrupt line for that vector. Use
 * aic_mask_on and aic_mask_off to control actual activation of the
 * interrupt line. */
void aic_set_vector(uint32 vector, uint32 prio, uint32 isr)
{
    if (vector < 32)
    {
        int i_state = irqs_get_and_disable();
        AT91C_BASE_AIC->AIC_SMR[vector] = prio;
        AT91C_BASE_AIC->AIC_SVR[vector] = isr;
        if (i_state)
            irqs_enable();
    }
}

/* Enable handling of an interrupt line in the AIC. */
void aic_mask_on(uint32 vector)
{
    int i_state = irqs_get_and_disable();
    AT91C_BASE_AIC->AIC_IECR = (1 << vector);
    if (i_state)
        irqs_enable();
}

/* Disable handling of an interrupt line in the AIC. */
void aic_mask_off(uint32 vector)
{
    int i_state = irqs_get_and_disable();
    AT91C_BASE_AIC->AIC_IDCR = (1 << vector);
    if (i_state)
        irqs_enable();
}

/* Clear an interrupt line in the AIC. */
void aic_clear(uint32 vector)
{
    int i_state = irqs_get_and_disable();
    AT91C_BASE_AIC->AIC_ICCR = (1 << vector);
    if (i_state)
        irqs_enable();
}

/* Set interrupt command register in the AIC. */
void aic_set(uint32 vector)
{
    int i_state = irqs_get_and_disable();
    AT91C_BASE_AIC->AIC_ISCR = (1 << vector);
    if (i_state)
        irqs_enable();
}
