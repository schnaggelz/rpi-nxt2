#include "drivers/nxt_lcd.h"
#include "drivers/nxt_spi.h"

#include "platform/systick.h"

static uint8 *display = (uint8 *) 0;

void nxt_lcd_command(uint8 cmd)
{
    uint8 tmp = cmd;
    nxt_spi_write(0, &tmp, 1);
}

void nxt_lcd_set_col(uint32 coladdr)
{
    nxt_lcd_command(0x00 | (coladdr & 0xF));
    nxt_lcd_command(0x10 | ((coladdr >> 4) & 0xF));
}

void nxt_lcd_set_multiplex_rate(uint32 mr)
{
    nxt_lcd_command(0x20 | (mr & 3));
}

void nxt_lcd_set_temp_comp(uint32 tc)
{
    nxt_lcd_command(0x24 | (tc & 3));
}

void nxt_lcd_set_panel_loading(uint32 hi)
{
    nxt_lcd_command(0x28 | ((hi) ? 1 : 0));
}

void nxt_lcd_set_pump_control(uint32 pc)
{
    nxt_lcd_command(0x2c | (pc & 3));
}

void nxt_lcd_set_scroll_line(uint32 sl)
{
    nxt_lcd_command(0x40 | (sl & 0x3f));
}

void nxt_lcd_set_page_address(uint32 pa)
{
    nxt_lcd_command(0xB0 | (pa & 0xf));
}

void nxt_lcd_set_pot(uint32 pot)
{
    nxt_lcd_command(0x81);
    nxt_lcd_command(pot & 0xff);
}

void nxt_lcd_set_ram_address_control(uint32 ac)
{
    nxt_lcd_command(0x88 | (ac & 7));
}

void nxt_lcd_set_frame_rate(uint32 fr)
{
    nxt_lcd_command(0xA0 | (fr & 1));
}

void nxt_lcd_set_all_pixels_on(uint32 on)
{
    nxt_lcd_command(0xA4 | ((on) ? 1 : 0));
}

void nxt_lcd_inverse_display(uint32 on)
{
    nxt_lcd_command(0xA6 | ((on) ? 1 : 0));
}

void nxt_lcd_enable(uint32 on)
{
    nxt_lcd_command(0xAE | ((on) ? 1 : 0));
}

void nxt_lcd_set_map_control(uint32 map_control)
{
    nxt_lcd_command(0xC0 | ((map_control & 3) << 1));
}

void nxt_lcd_reset(void)
{
    nxt_lcd_command(0xE2);
}

void nxt_lcd_set_bias_ratio(uint32 ratio)
{
    nxt_lcd_command(0xE8 | (ratio & 3));
}

void nxt_lcd_set_cursor_update(uint32 on)
{
    nxt_lcd_command(0xEE | ((on) ? 1 : 0));
}

void nxt_lcd_force_update()
{
    /* Update the screen the slow way.
       Works with interrupts disabled. */
    int i;
    uint8 *disp = display;

    for (i = 0; i < NXT_LCD_DEPTH; i++)
    {
        nxt_lcd_set_col(0);
        nxt_lcd_set_page_address(i);

        nxt_spi_write(1, disp, NXT_LCD_WIDTH);
        disp += NXT_LCD_WIDTH;
    }
}

void nxt_lcd_update()
{
#define DMA_REFRESH
#ifdef DMA_REFRESH
    nxt_spi_refresh();
#else
    nxt_lcd_force_update();
#endif
}

void nxt_lcd_power_up(void)
{
    systick_wait_ms(20);
    nxt_lcd_reset();
    systick_wait_ms(20);

    nxt_lcd_set_multiplex_rate(3);      /* 1/65 */
    nxt_lcd_set_bias_ratio(3);          /* 1/9 */
    nxt_lcd_set_pot(0x60);              /* ?? 9V?? */
    nxt_lcd_set_ram_address_control(1); /* Auto wrap */
    nxt_lcd_set_map_control(0x02);      /* Mirror in y */
    nxt_spi_set_display(display);
    nxt_lcd_enable(1);
}

void nxt_lcd_power_down(void)
{
    nxt_lcd_reset();
}

void nxt_lcd_init(const uint8* disp)
{
    display = (uint8*)disp;
}
