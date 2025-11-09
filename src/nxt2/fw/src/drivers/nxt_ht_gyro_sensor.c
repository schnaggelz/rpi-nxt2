/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_ht_gyro_sensor.h"

#include "drivers/nxt_sensors.h"
#include "drivers/nxt_avr.h"

#include "platform/i2c.h"

void nxt_ht_gyro_sensor_init(uint8 port)
{
    nxt_sensor_init_i2c(port, NXT_LOWSPEED_PORT);
}

uint16 nxt_ht_gyro_sensor_get_angular_velocity_preview(uint8 port)
{
    return nxt_avr_get_sensor_adc(port);
}

uint16 nxt_ht_gyro_sensor_get_angular_velocity(uint8 port)
{
    uint16 res = 0;
    uint8 cycle = 0;

    /* Support for multiple compass sensors in a NXT. */
    static uint8 data[2] = {0};

    if (i2c_status(port) == 0)
    {
        /* i2c_start just triggers an I2C transaction, the actual
         * data transaction between ARM7 and the compass sensor
         * is done by an ISR after this, so there is one cycle
         * delay for consistent data acquisition. We need to
         * alternate between high and low byte. */
        if (cycle % 2 == 0)
        {
            i2c_start(port, 1, 0x42, &data[0], 1, 0);
        }
        else
        {
            i2c_start(port, 1, 0x43, &data[1], 1, 0);
        }
    }

    res = ((uint16)data[0] << 8) | (uint8)data[1];


    return res;
}

void nxt_ht_gyro_sensor_term(uint8 port)
{
    i2c_disable(port);
}
