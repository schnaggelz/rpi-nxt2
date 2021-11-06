/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_ht_ir_seeker.h"

#include "drivers/nxt_sensors.h"

#include "platform/i2c.h"

static uint8 sensor_data_state[12];

void nxt_ht_ir_seeker_init(uint8 port)
{
    nxt_sensor_init_i2c(port, NXT_LOWSPEED_PORT);
}

void nxt_ht_ir_seeker_get_data(uint8 port, sint8 buf[12])
{
    uint32 i;

    /* Support for multiple IR Seeker in a NXT. */
    static uint8 data[4][12] = {{0}};

    for (i = 0; i < 12; i++)
    {
        buf[i] = (sint8)data[port][i];
        sensor_data_state[i] = buf[i];
    }
    /* Data representation from the sensor, per Hitechnic's documentation:
       data[0]:  Direction  DC 8 bits
       data[1]:  Intensity1 DC 8 bits
       data[2]:  Intensity2 DC 8 bits
       data[3]:  Intensity3 DC 8 bits
       data[4]:  Intensity4 DC 8 bits
       data[5]:  Intensity5 DC 8 bits
       data[6]:  Direction  AC 8 bits
       data[7]:  Intensity1 AC 8 bits
       data[8]:  Intensity2 AC 8 bits
       data[9]:  Intensity3 AC 8 bits
       data[10]: Intensity4 AC 8 bits
       data[11]: Intensity5 AC 8 bits */

    if (i2c_status(port) == 0)
    {
        /* i2c_start just triggers an I2C transaction, the actual
         * data transaction between ARM7 and the IR sensor is done
         * by an ISR after this, so there is one raster delay for
         * consistent data acquisition. */
        i2c_start(port, 0x8, 0x42, &data[port][0], 12, 0);
    }
}

void nxt_ht_ir_seeker_term(uint8 port)
{
    i2c_disable(port);
}
