#include "nxt_devices.h"

#include "irqs.h"
#include "aic.h"
#include "uart.h"
#include "twi.h"
#include "i2c.h"
#include "bt.h"
#include "udp.h"
#include "systick.h"
#include "ports.h"

#include "nxt_spi.h"
#include "nxt_avr.h"
#include "nxt_lcd.h"
#include "nxt_motors.h"
#include "nxt_sensors.h"
#include "nxt_usb.h"
#include "nxt_display.h"
#include "nxt_sound.h"
#include "nxt_color_sensor.h"

void nxt_devices_init(void)
{
    /* Initialize interrupt controller. */
    aic_init();

    /* Reset all sensor ports. */
    sp_init();

    /* Enable interrupts from here onwards. */
    irqs_enable();

    /* Initialize communication with AVR. */
    nxt_avr_init();

    /* Initialize system clock. */
    systick_init();

    /* Initialize TWI. */
    i2c_init();

    /* Initialize AVR controlled motor ports. */
    nxt_motor_init();

    /* Initialize Bluetooth device. */
    bt_init();

    /* Initialize LCD display buffer. */
    nxt_display_init();

    /* Initialize the LCD with the buffer. */
    nxt_lcd_init(nxt_display_get_buffer());

    /* Initialize SPI for LCD. */
    nxt_spi_init();

    /* Power to the LCD. */
    nxt_lcd_power_up();

    /* Initialize digital sensors. */
    nxt_sensors_init();
}

void nxt_bg_task()
{
    /* Do AVR main processing. */
    nxt_avr_1kHz_update();

    /* Raise motor interrupt. */
    nxt_motor_1kHz_process();

    /* Do USB I/O processing. */
    nxt_usb_1kHz_process();
}
