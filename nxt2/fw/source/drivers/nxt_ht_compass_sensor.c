/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* NXT C driver code.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "drivers/nxt_ht_compass_sensor.h"

#include "drivers/nxt_sensors.h"

#include "platform/i2c.h"

void nxt_ht_compass_sensor_init(uint8 port)
{
    nxt_sensor_init_i2c(port, NXT_LOWSPEED_PORT);
}

uint8 nxt_ht_compass_sensor_calibrate(uint8 port)
{
    uint8 cmd;

    if (i2c_wait_ready(port, 50) == 0) return 1;

    /* Start calibration command. */
    cmd = 0x43;
    i2c_start(port, 1, 0x41, &cmd, 1, 1);
    if (i2c_wait_ready(port, 50) == 0) return 1;

    /* End calibration command. */
    cmd = 0x00;
    i2c_start(port, 1, 0x41, &cmd, 1, 1);
    if (i2c_wait_ready(port, 50) == 0) return 1;

    /* Now read back the byte, if it's 0 then calibration succeeded,
       otherwise it'll be a 2 and it failed. */
    i2c_start(port, 1, 0x41, &cmd, 1, 0);
    if (i2c_wait_ready(port, 50) == 0) return 1;

    return cmd;
}

sint16 nxt_ht_compass_sensor_get_direction(uint8 port)
{
    /* Support for multiple compass sensors in a NXT. */
    static uint8 data[4][5] = {{0}};

    /* Data representation from the sensor, per Hi-Technic's
     * documentation:
     *    data[0]: mode control
     *    data[1]: two degree heading
     *    data[2]: one degree heading
     *    data[3]: heading low bytes
     *    data[4]: heading high bytes */
    sint16 buf = ((sint16)data[port][3] & 0xff) | (
                 ((sint16)data[port][4] << 8) & 0xff00);

    if (i2c_status(port) == 0)
    {
        /* i2c_start just triggers an I2C transaction, the actual
         * data transaction between ARM7 and the compass sensor
         * is done by an ISR after this, so there is one cycle
         * delay for consistent data acquisition. */
        i2c_start(port, 1, 0x41, &data[port][0], 5, 0);
    }

    return buf;
}

void nxt_ht_compass_sensor_term(uint8 port)
{
    i2c_disable(port);
}
