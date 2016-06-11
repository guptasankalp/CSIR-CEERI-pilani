#include<avr/io.h>
#include<util/delay.h>
void left();
void right();
void forw();
void back();
void sl();
void sr();





#include<avr/io.h>
#include<compat/deprecated.h>
//#include"adc.c"
#define b bit_is_clear 
void timer()
{

	TCCR1A|= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B|= (1<<WGM12)|(1<<WGM13)|(1<<CS10);
	ICR1 = 50000;
	
}


void forw()
{	
		OCR1A=50000;
		OCR1B=30000;
		cbi(PORTD,2);
		sbi(PORTD,3);
		cbi(PORTD,6);
		sbi(PORTD,7);
	
		
	}
	
	void back()
	{
	OCR1A=50000;
	OCR1B=30000;
	cbi(PORTD,3);
	sbi(PORTD,2);
	
	cbi(PORTD,7);
	sbi(PORTD,6);
	
	}
	
	void right()
	{
	OCR1A=50000;
	OCR1B=30000;
	
	cbi(PORTD,3);
	sbi(PORTD,2);
	
	cbi(PORTD,6);
	sbi(PORTD,7);
	
}

	
	void left()
	{
	OCR1A=50000;
	OCR1B=30000;
	cbi(PORTD,2);
	sbi(PORTD,3);
	
	cbi(PORTD,7);
	sbi(PORTD,6);

	}
	
	void sl()
	{
	OCR1A=30000;
	OCR1B=10000;
	cbi(PORTD,2);
	sbi(PORTD,3);
	
	
		}
		
	void sr()
	{
	OCR1A=30000;
	OCR1B=10000;
	cbi(PORTD,6);
	
	sbi(PORTD,7);
	
		}	
	
	
	void main()
{   timer();
  DDRD=0xFF;
 /* check while(1)
  {
  if(bit_is_clear(PINA,1))
  {back();
  _delay_ms(1);
  
  }
  
  if(!bit_is_clear(PINA,1))
  
  {forw();
  _delay_ms(1);
  }}
  
  
 */
   while(1)
   { 
		if((!bit_is_clear(PINA,2))&&(bit_is_clear(PINA,0))&&(bit_is_clear(PINA,4)))
		{	   
			forw();

		}

        if((!bit_is_clear(PINA,0))&&(bit_is_clear(PINA,4)))

        { 
		    left();
        }


        if((!bit_is_clear(PINA,4))&& (bit_is_clear(PINA,0)))
	    { 
             right();
	    }	

	    if((!bit_is_clear(PINA,1))&& (bit_is_clear(PINA,3)))
	    {  
		     left();
	
        }
        if((!bit_is_clear(PINA,3))&& (bit_is_clear(PINA,1)))
        {
              right();
        }
        
        
	/*	if(!bit_is_clear(PINA,2)&&!bit_is_clear(PINA,0)&&!bit_is_clear(PINA,4)&&!bit_is_clear(PINA,1)&&!bit_is_clear(PINA,3))
		{	   
			back();

		}
		*/
		
		   
		   
		   
		   
		   
		   
		   
		   
		   
		
    }
}
