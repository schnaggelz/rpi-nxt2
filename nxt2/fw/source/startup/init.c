/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* Initialisation for ARM7
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/at91/at91sam7s256.h"

/* Shorthand for addressing the System Controller structure defined in
 * the AT91 platform package, where the AIC's registers are defined.
 */

int _low_level_init(void)
{
    /* Set flash wait state.
       Single cycle access at up to 30 MHz or 40 MHz
       If MCK = 48054841 we have 50 cycles for 1 us (field MC_FMR->FMCN)
       Result: AT91C_MC_FMR = 0x00320100 (MC Flash Mode Register) */
    AT91C_BASE_MC->MC_FMR = ((AT91C_MC_FMCN)&(50 <<16)) | AT91C_MC_FWS_1FWS;

    /* Disable watchdog.
       Result: AT91C_WDTC_WDMR = 0x00008000 (Watchdog Mode Register) */
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Enable the main oscillator and wait for startup.
       Set MCK at 48054841, SCK = 1/32768 = 30.51 us
       Start-up time = 8 * 6 / SCK = 56 * 30.51 = 1,46484375 ms
       Result: AT91C_CKGR_MOR = 0x00000601 (Main Oscillator Register) */
    AT91C_BASE_PMC->PMC_MOR = (((AT91C_CKGR_OSCOUNT & (0x06 <<8)) | AT91C_CKGR_MOSCEN));
    while(!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    /* Set up PMC clock generator PLL register and wait for startup (until PMC
       status register LOCK bit is set).
       The following settings are used:  DIV = 14, MUL = 72, PLLCOUNT = 10
       Main clock (MAINCK from crystal oscillator) = 18432000 Hz
       MAINCK / DIV = 18432000/14 = 1316571 Hz
       PLLCK = 1316571 * (MUL + 1) = 1316571 * (72 + 1) = 1316571 * 73 = 96109683 Hz
       PLLCOUNT = number of slow clock cycles before the LOCK bit is set
                  in PMC_SR after CKGR_PLLR is written.
       PLLCOUNT = 10
       OUT = 0 (not used)
       Result: AT91C_CKGR_PLLR = 0x00000000480A0E (PLL Register) */
    AT91C_BASE_PMC->PMC_PLLR = ((AT91C_CKGR_DIV & 14) |
                       (AT91C_CKGR_PLLCOUNT & (10<<8)) |
                       (AT91C_CKGR_MUL & (72<<16)));
    while(!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK));

    /* Set up PMC master clock (MCK) register and wait for startup.
       CSS = 3 (PLLCK clock selected)
	   PRES = 1 (MCK = PLLCK / 2) = 96109683/2 = 48054841 Hz
       Note: Master Clock MCK = 48054841 Hz (this is the CPU clock speed)
       Result: AT91C_PMC_MCKR = 0x00000007 (Master Clock Register) */
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_CSS_PLL_CLK | AT91C_PMC_PRES_CLK_2;
    while(!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    return 1;
}
