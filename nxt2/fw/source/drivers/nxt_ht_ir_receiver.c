#include "drivers/nxt_ht_ir_receiver.h"

#include "platform/i2c.h"

void nxt_ht_ir_receiver_init(uint8 port)
{
    i2c_init(port, LOWSPEED_PORT);
}

sint8 nxt_ht_ir_receiver_get_motor_control(uint8 port, uint8 channel_id, uint8 motor_id)
{
    /* Support for multiple IR receivers in a NXT. */
    static sint8 data[4][4][2] = {{{0}}};

    /* Data representation from the sensor, per Hi-Technic's documentation:
       data[0]: motor 1A control
       data[1]: motor 1B control
       data[2]: motor 2A control
       data[3]: motor 2B control
       data[4]: motor 3A control
       data[5]: motor 3B control
       data[6]: motor 4A control
       data[7]: motor 4B control */
    sint8 control_value = -1;

    if (channel_id > 3 || motor_id > 1)
    	return control_value; /* Invalid arguments. */

    control_value = data[port][channel_id][motor_id];

    if (i2c_status(port) == 0)
    {
        /* i2c_start just triggers an I2C transaction, the actual
         * data transaction between ARM7 and the IR receiver is
         * done by an ISR after this, so there is one raster delay
         * for consistent data acquisition. */
        i2c_start(port, 1, 0x42, &data[port][0], 8, 0);
    }

    return control_value;
}

void nxt_ht_ir_receiver_term(uint8 port)
{
    i2c_disable(port);
}
