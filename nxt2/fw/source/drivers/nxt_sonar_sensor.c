#include "drivers/nxt_sonar_sensor.h"

#include "drivers/nxt_sensors.h"

#include "platform/i2c.h"

static sint32 distance_state[4] = { -1, -1, -1, -1 }; /* -1: sensor is not connected */

sint32 nxt_sonar_sensor_get_distance(uint8 port)
{
    sint32 distance = -1;
    static uint8 data[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

    if (i2c_status(port) == 0)
    {
        /* i2c_start just triggers an I2C transaction, the actual
         * data transaction between ARM7 and the ultrasonic sensor
         * is done by an ISR after this, so there is one cycle
         * delay for consistent data acquisition. */
        distance = data[port];
        i2c_start(port, 1, 0x42, &data[port], 1, 0);
    }

    /* Update state for offline queries. */
    distance_state[port] = data[port];

    return distance;
}

void nxt_sonar_sensor_init(uint8 port)
{
    nxt_sensor_init_i2c(port, NXT_LOWSPEED_PORT_9V);
}

void nxt_sonar_sensor_term(uint8 port)
{
    i2c_disable(port);
}
