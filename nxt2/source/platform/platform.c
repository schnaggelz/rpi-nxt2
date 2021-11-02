#include "platform/platform.h"
#include "platform/irqs.h"
#include "platform/aic.h"
#include "platform/twi.h"
#include "platform/systick.h"
#include "platform/i2c.h"

void at91_platform_init()
{
    /* Init interrupt controller. */
    aic_init();

    /* Enable all interrupts. */
    irqs_enable();

    /* Initialize SPI. */
    twi_init();

    /* Initialize timer. */
    systick_init();

    /* Initialize I2C. */
    i2c_init();
}
