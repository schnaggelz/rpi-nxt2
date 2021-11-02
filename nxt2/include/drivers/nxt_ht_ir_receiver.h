#ifndef __NXT_HT_IR_RECEIVER_H__
#define __NXT_HT_IR_RECEIVER_H__

#include "nxt_devices.h"

void nxt_ht_ir_receiver_init(uint8 port);
sint8 nxt_ht_ir_receiver_get_motor_control(uint8 port, uint8 motor_id, uint8 channel_id);
void nxt_ht_ir_receiver_term(uint8 port);

#endif /* __NXT_HT_IR_RECEIVER_H__ */
