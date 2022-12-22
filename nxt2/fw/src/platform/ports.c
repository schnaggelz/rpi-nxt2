/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/ports.h"

#include "drivers/nxt_avr.h" // TODO

#include "platform/at91/at91sam7.h"

/* Sensor port digital pins */
const port_pins sensor_pins[4] =
{
    {{AT91C_PIO_PA23, AT91C_PIO_PA18}, AT91C_ADC_CH1, AT91C_ADC_CDR1},
    {{AT91C_PIO_PA28, AT91C_PIO_PA19}, AT91C_ADC_CH2, AT91C_ADC_CDR2},
    {{AT91C_PIO_PA29, AT91C_PIO_PA20}, AT91C_ADC_CH3, AT91C_ADC_CDR3},
    {{AT91C_PIO_PA30, AT91C_PIO_PA2},  AT91C_ADC_CH7, AT91C_ADC_CDR7}
};

/* Masks and bits used for sensor events. */
#define GT_EVENTS 0xf
#define LT_EVENTS 0xf0
#define GT_SHIFT 0
#define LT_SHIFT 4
#define GET_TARGET(f) (((f) >> 8) & 0x3ff)
#define GET_TOLERANCE(f) (((f) >> 18) & 0xff)

void sp_reset(uint8 port)
{
    /* Reset the port to be normal digital I/O. */
    sp_set_mode(port, SP_DIGI0, SP_MODE_OUTPUT);
    sp_set_mode(port, SP_DIGI1, SP_MODE_OUTPUT);

    /* Set the output to be zero. */
    sp_set(port, SP_DIGI0, 0);
    sp_set(port, SP_DIGI1, 0);

    /* If this is port with RS485 on it, reset those pins as well. */
    if (port == RS485_PORT)
    {
        *AT91C_PIOA_PER = AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
        *AT91C_PIOA_PPUDR = AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
        *AT91C_PIOA_OER = AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
        *AT91C_PIOA_CODR = AT91C_PIO_PA5 | AT91C_PIO_PA6 | AT91C_PIO_PA7;
    }

    /* Reset the power being supplied to the port. */
    sp_set_power(port, 0);
}

void sp_init(void)
{
    uint8 port;

    for (port = 0; port < NUM_SENSOR_PORTS; port++)
    {
        sp_reset(port);
    }
}

sint32 sp_read(uint8 port, uint8 pin)
{
    if (pin == SP_ANA)
    {
        return nxt_avr_get_sensor_adc(port);
    }
    else
    {
        return *sensor_pins[port].adc_data;
    }
}

sint32 sp_check_event(sint32 filter)
{
    uint8 port;

    int bit = (1 << GT_SHIFT) | (1 << LT_SHIFT);
    int changed = 0;
    int target = GET_TARGET(filter);
    int tolerance = GET_TOLERANCE(filter);

    for(port = 0; port < NUM_SENSOR_PORTS; port++, bit <<= 1)
    {
        if (filter & bit)
        {
            uint16 val = nxt_avr_get_sensor_adc(port);
            if ((filter & bit & GT_EVENTS) && (val > (target + tolerance)))
                changed |= (bit & GT_EVENTS);
            if ((filter & bit & LT_EVENTS) && (val < (target - tolerance)))
                changed |= (bit & LT_EVENTS);
        }
    }
    return changed;
}

void sp_set_mode(uint8 port, uint8 pin, sint32 mode)
{
    int at_pin = sensor_pins[port].pins[pin];
    *AT91C_PIOA_PPUDR = at_pin;

    switch (mode)
    {
    case SP_MODE_OFF:
        *AT91C_PIOA_ODR = at_pin;
        *AT91C_PIOA_PDR = at_pin;
        break;
    case SP_MODE_INPUT:
        *AT91C_PIOA_PER = at_pin;
        *AT91C_PIOA_ODR = at_pin;
        break;
    case SP_MODE_OUTPUT:
        *AT91C_PIOA_PER = at_pin;
        *AT91C_PIOA_OER = at_pin;
        break;
    case SP_MODE_ADC:
        *AT91C_PIOA_ODR = at_pin;
        *AT91C_PIOA_PER = at_pin;
        break;
    }
    if (pin == SP_DIGI1)
    {
        if (mode == SP_MODE_ADC)
            *AT91C_ADC_CHER = sensor_pins[port].adc_channel;
        else
            *AT91C_ADC_CHDR = sensor_pins[port].adc_channel;
    }
}

sint32 sp_get(uint8 port, uint8 pin)
{
    return (sensor_pins[port].pins[pin] & *AT91C_PIOA_PDSR) != 0;
}

void sp_set(uint8 port, uint8 pin, sint32 val)
{
    int at_pin = sensor_pins[port].pins[pin];
    if (val)
        *AT91C_PIOA_SODR = at_pin;
    else
        *AT91C_PIOA_CODR = at_pin;
}

void sp_set_power(uint8 port, uint32 val)
{
    nxt_avr_set_input_power(port, val);
}
