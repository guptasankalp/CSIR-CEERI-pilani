#include<avr/io.h>
#include<compat/deprecated.h>
#include<util/delay.h>
#include"lcd.h"
#include"lcd.c"
#include"adc.c"

void main()
{
int x;
char a[100];

lcd_init(LCD_DISP_ON);

	lcd_clrscr();
	
	while(1)
	{
	
	lcd_clrscr();
	x=readADC(0);
	itoa(x,a,10);
	
	lcd_gotoxy(3,0);
	lcd_puts(a);
	_delay_ms(200);	
	if(x==4)
	{
	
	lcd_gotoxy(3,1);
	lcd_puts("green");
	_delay_ms(200);
	}
	if(x==3)
	{
	
	lcd_gotoxy(3,1);
	lcd_puts("red");
	_delay_ms(200);
	}
	
	if(x==5)
	{
	
	lcd_gotoxy(3,1);
	lcd_puts("yellow");
	_delay_ms(200);
	}
	
	
	}
	
}