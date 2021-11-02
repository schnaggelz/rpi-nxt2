#ifndef __NXT_MOTORS_H__
#define __NXT_MOTORS_H__

#include "platform/systypes.h"

#define NXT_NUM_MOTOR_PORTS  3

typedef enum
{
    NXT_PORT_A,
    NXT_PORT_B,
    NXT_PORT_C
} NXT_MOTOR_PORTS;

#ifdef __cplusplus
extern "C" {
#endif

sint32 nxt_motor_get_count(uint8 port);
void nxt_motor_set_count(uint8 port, sint32 count);
void nxt_motor_set_speed(uint8 port, sint32 speed_percent, sint32 brake);
void nxt_motor_command(uint8 port, sint32 cmd, sint32 target_count, sint32 speed_percent);
void nxt_motor_init(void);

#ifdef __cplusplus
}
#endif

/* ISR entry point used by motor processing. */
void nxt_motor_1kHz_process(void);

#endif
