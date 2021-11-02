#ifndef __NXT_SENSORS_H__
#define __NXT_SENSORS_H__

#include "platform/systypes.h"

#define NXT_LOWSPEED_PORT_9V 1
#define NXT_LOWSPEED_PORT    2

#define NXT_NUM_SENSOR_PORTS 4

typedef enum
{
    NXT_PORT_S1,
    NXT_PORT_S2,
    NXT_PORT_S3,
    NXT_PORT_S4
} NXT_SENSOR_PORTS;

void nxt_sensors_init(void);
void nxt_sensors_poll(void);

void nxt_sensor_init_i2c(uint8 port, uint8 type);
void nxt_sensor_set_digi0(uint8 port);
void nxt_sensor_set_digi1(uint8 port);
void nxt_sensor_unset_digi0(uint8 port);
void nxt_sensor_unset_digi1(uint8 port);

#endif /* __NXT_SENSORS_H__ */
