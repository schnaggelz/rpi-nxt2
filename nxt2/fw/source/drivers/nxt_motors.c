/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "drivers/nxt_motors.h"

#include "drivers/nxt_avr.h"

#include "platform/aic.h"
#include "platform/at91/at91sam7.h"
#include "platform/irqs.h"

#include <stdlib.h>
#include <limits.h>

#define MA0 15
#define MA1 1
#define MB0 26
#define MB1 9
#define MC0 0
#define MC1 8

#define MOTOR_PIN_MASK                                                         \
    ((1 << MA0) | (1 << MA1) | (1 << MB0) | (1 << MB1) | (1 << MC0) |          \
     (1 << MC1))
#define MOTOR_INTERRUPT_PINS ((1 << MA0) | (1 << MB0) | (1 << MC0))

typedef struct
{
    sint32 tolerance;
    sint32 current_count;
    sint32 target_count;
    sint32 speed_percent;
    uint32 last_edge;
} nxt_motor_port;

static nxt_motor_port motor_ports[NXT_NUM_MOTOR_PORTS];

static uint32 nxt_motors_initialised;
static uint32 interrupts_this_period;

sint32 nxt_motor_get_speed(uint8 port)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        return motor_ports[port].speed_percent;
    }

    return 0;
}

sint32 nxt_motor_get_current_count(uint8 port)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        return motor_ports[port].current_count;
    }

    return 0;
}

sint32 nxt_motor_get_target_count(uint8 port)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        return motor_ports[port].target_count;
    }

    return 0;
}

void nxt_motor_set_speed(uint8 port, sint32 speed_percent, sint32 brake)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        if (speed_percent > 100)
            speed_percent = 100;
        if (speed_percent < -100)
            speed_percent = -100;

        motor_ports[port].speed_percent = speed_percent;
        nxt_avr_set_motor(port, speed_percent, brake);
    }
}

void nxt_motor_set_current_count(uint8 port, sint32 count)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        motor_ports[port].current_count = count;
    }
}

static void nxt_motor_check_target(uint8 port)
{
    if (port >= NXT_NUM_MOTOR_PORTS)
    {
        return;
    }

    sint32 tolerance = motor_ports[port].tolerance;
    sint32 speed = motor_ports[port].speed_percent;
    sint32 current_count = motor_ports[port].current_count;
    sint32 target_count = motor_ports[port].target_count;

    if (target_count != UINT_MAX && speed != 0)
    {
        if (target_count < current_count - tolerance)
        {
            sint32 expected_speed = -abs(speed);

            if (speed != expected_speed)
            {
                nxt_motor_set_speed(port, expected_speed, 1);
            }
        }
        else if (target_count > current_count + tolerance)
        {
            sint32 expected_speed = abs(speed);

            if (speed != expected_speed)
            {
                nxt_motor_set_speed(port, expected_speed, 1);
            }
        }
        else
        {
            nxt_motor_set_speed(port, 0, 1);

            nxt_motor_set_target_count(port, UINT_MAX, 0);
        }
    }
}

void nxt_motor_set_target_count(uint8 port, sint32 count, sint32 tolerance)
{
    if (port < NXT_NUM_MOTOR_PORTS)
    {
        motor_ports[port].target_count = count;
        motor_ports[port].tolerance = tolerance;
    }
}

void nxt_motor_1kHz_process(void)
{
    if (!nxt_motors_initialised)
    {
        return;
    }

    for (unsigned port = 0; port < NXT_NUM_MOTOR_PORTS; port++)
    {
        nxt_motor_check_target(port);
    }

    interrupts_this_period = 0;

    *AT91C_PIOA_IER = MOTOR_INTERRUPT_PINS;
}

void nxt_motor_quad_decode(nxt_motor_port* port, uint32 value)
{
    uint32 dir = value & 2;
    uint32 edge = value & 1;

    if (edge != port->last_edge)
    {
        if (edge && !dir)
        {
            port->current_count++;
        }
        else if (edge && dir)
        {
            port->current_count--;
        }
        else if (!edge && dir)
        {
            port->current_count++;
        }
        else if (!edge && !dir)
        {
            port->current_count--;
        }

        port->last_edge = edge;
    }
}

void nxt_motor_isr_handler(void)
{
    uint32 i_state = irqs_get_and_disable();

    uint32 pin_changes = *AT91C_PIOA_ISR;   /* Acknowledge change. */
    uint32 current_pins = *AT91C_PIOA_PDSR; /* Read pins. */
    uint32 pins;

    interrupts_this_period++;
    if (interrupts_this_period > 4)
    {
        *AT91C_PIOA_IDR = MOTOR_INTERRUPT_PINS;
    }

    /* Motor A */
    pins = ((current_pins >> MA0) & 1) | ((current_pins >> (MA1 - 1)) & 2);
    nxt_motor_quad_decode(&motor_ports[0], pins);

    /* Motor B */
    pins = ((current_pins >> MB0) & 1) | ((current_pins >> (MB1 - 1)) & 2);
    nxt_motor_quad_decode(&motor_ports[1], pins);

    /* Motor C */
    pins = ((current_pins >> MC0) & 1) | ((current_pins >> (MC1 - 1)) & 2);
    nxt_motor_quad_decode(&motor_ports[2], pins);

    if (i_state)
        irqs_enable();
}

void nxt_motor_init(void)
{
    *AT91C_PMC_PCER = (1 << AT91C_PERIPHERAL_ID_PIOA); /* Power to the pins! */
    *AT91C_PIOA_IDR = ~0;
    *AT91C_PIOA_IFER = MOTOR_PIN_MASK;
    *AT91C_PIOA_PPUDR = MOTOR_PIN_MASK;
    *AT91C_PIOA_PER = MOTOR_PIN_MASK;
    *AT91C_PIOA_ODR = MOTOR_PIN_MASK;

    /* Enable ISR. */
    aic_mask_off(AT91C_PERIPHERAL_ID_PIOA);

    aic_set_vector(AT91C_PERIPHERAL_ID_PIOA, AIC_INT_LEVEL_NORMAL,
                   (uint32)nxt_motor_isr_handler);

    aic_mask_on(AT91C_PERIPHERAL_ID_PIOA);

    *AT91C_PIOA_IER = MOTOR_INTERRUPT_PINS;

    for (unsigned port = 0; port < NXT_NUM_MOTOR_PORTS; port++)
    {
        motor_ports[port].tolerance = 0;
        motor_ports[port].speed_percent = 0;
        motor_ports[port].target_count = UINT_MAX;
        motor_ports[port].current_count = 0;
        motor_ports[port].last_edge = 0;
    }

    nxt_motors_initialised = 1;
}
