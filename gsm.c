#include<avr/io.h>
#include<util/delay.h>
#include"uart.c"
#include"uart.h"
#include<compat/deprecated.h>

void main()
{
uart_init(UART_BAUD_SELECT(9600,F_CPU));
uart_puts("AT");
uart_putc(13);
_delay_ms(2000);
uart_puts("ATD9782747899");
uart_putc(13);
_delay_ms(2000);
}