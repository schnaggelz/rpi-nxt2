#include "../../include/drivers/nxt_buttons.h"

#include "../../include/drivers/nxt_avr.h"

uint8 nxt_avr_check_buttons_event(uint8 filter)
{
    return nxt_avr_get_button_state() & filter;
}

uint8 nxt_enter_button_is_pressed(void)
{
    return nxt_avr_check_buttons_event(NXT_BUTTON_ENTER);
}

uint8 nxt_escape_button_is_pressed(void)
{
    return nxt_avr_check_buttons_event(NXT_BUTTON_ESCAPE);
}

uint8 nxt_right_button_is_pressed(void)
{
    return nxt_avr_check_buttons_event(NXT_BUTTON_RIGHT);
}

uint8 nxt_left_button_is_pressed(void)
{
    return nxt_avr_check_buttons_event(NXT_BUTTON_LEFT);
}
