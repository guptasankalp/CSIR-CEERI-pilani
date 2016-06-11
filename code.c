#include<avr/io.h>
#include<util/delay.h>
#include"lcd.c"
#include"lcd.h"
#include<compat/deprecated.h>


// BRAM Debugging Mode, 0-Debug Off, 1-Debug On
#define BRAM_DEBUG 0
#define BAUD_RATE 19200
#define MAX_TRIES 50
#define MCP23008_ID    0x40  // MCP23008 Device Identifier
#define MCP23008_ADDR  0x0E  // MCP23008 Device Address
#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPPU  0x06           // MCP23008 I/O Pull-Up Resistor Register
#define GPIO  0x09           // MCP23008 General Purpose I/O Register
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define ACK 1
#define NACK 0
// Define BRAM Steering
#define MOVE_FORWARD  0
#define TURN_LEFT     1
#define TURN_RIGHT    2
#define ROTATE_LEFT   3
#define ROTATE_RIGHT  4
#define MOVE_BACKWARD 5
#define FULL_STOP     6
// Define BRAM Sensor
#define MAX_MAP 24
#define TARGET_VAL 60
#define MAX_SENSOR_MAP 120
// Define BRAM Variables
const unsigned int sensor_map[] PROGMEM = {
  0b00000,0,
  0b00001,10,
  0b00011,20,
  0b00010,30,
  0b00111,40,
  0b00110,50,
  0b00100,60,
  0b01100,70,
  0b11100,80,
  0b01000,90,
  0b11000,100,
  0b10000,110
};
unsigned char MaxSpeed;                         // Hold Motor Maximum Speed
unsigned int Kp,Ki,Kd;                          // PID Parameters
int prev_res=0, prev_err_1=0, prev_err_2=0;     // PID Control Variables
void uart_init(void)
{
  UBRR0H = (((F_CPU/BAUD_RATE)/16)-1)>>8;	// set baud rate
  UBRR0L = (((F_CPU/BAUD_RATE)/16)-1);
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); 		// enable Rx & Tx
  UCSR0C=  (1<<UCSZ01)|(1<<UCSZ00);  	        // config USART; 8N1
}
void uart_flush(void)
{
  unsigned char dummy;
  while (UCSR0A & (1<<RXC0)) dummy = UDR0;
}
int uart_putch(char ch,FILE *stream)
{
   if (ch == '\n')
    uart_putch('\r', stream);
   while (!(UCSR0A & (1<<UDRE0)));
   UDR0=ch;
   return 0;
}
int uart_getch(FILE *stream)
{
   unsigned char ch;
   while (!(UCSR0A & (1<<RXC0)));
   ch=UDR0;  
   /* Echo the Output Back to terminal */
   uart_putch(ch,stream);       
   return ch;
}
void ansi_cl(void)
{
  // ANSI clear screen: cl=\E[H\E[J
  putchar(27);
  putchar('[');
  putchar('H');
  putchar(27);
  putchar('[');
  putchar('J');
}
void ansi_me(void)
{
  // ANSI turn off all attribute: me=\E[0m
  putchar(27);
  putchar('[');
  putchar('0');
  putchar('m');
}
void ansi_cm(unsigned char row,unsigned char col)
{
  // ANSI cursor movement: cl=\E%row;%colH
  putchar(27);
  putchar('[');
  printf("%d",row);
  putchar(';');
  printf("%d",col);
  putchar('H');
}
// BRAM Steering Function
// PD2 - Input 1 (Left Motor), PD3 - Input 2 (Left Motor)
// PD4 - Input 3 (Right Motor), PD7 - Input 4 (Right Motor)
void BRAM_Steer(unsigned char steer)
{
  switch(steer) {
    case MOVE_FORWARD:
	  PORTD &= ~(1 << PD4); PORTD |= (1 << PD7);  // Right Motor On Forward
	  PORTD &= ~(1 << PD2); PORTD |= (1 << PD3);  // Left Motor On Forward
      break;
    case TURN_LEFT:
  	  PORTD &= ~(1 << PD4); PORTD |= (1 << PD7);  // Right Motor On Forward
	  PORTD &= ~(1 << PD2); PORTD &= ~(1 << PD3); // Left Motor Off
      break;
    case TURN_RIGHT:
	  PORTD &= ~(1 << PD4); PORTD &= ~(1 << PD7); // Right Motor Off
	  PORTD &= ~(1 << PD2); PORTD |= (1 << PD3);  // Left Motor On Forward
      break;
    case ROTATE_LEFT:
 	  PORTD &= ~(1 << PD4); PORTD |= (1 << PD7);  // Right Motor On Forward
	  PORTD |= (1 << PD2); PORTD &= ~(1 << PD3);  // Left Motor On Reverse
          break;
    case ROTATE_RIGHT:
	  PORTD |= (1 << PD4); PORTD &= ~(1 << PD7);  // Right Motor On Reverse
	  PORTD &= ~(1 << PD2); PORTD |= (1 << PD3);  // Left Motor On Forward
          break;
    case MOVE_BACKWARD:
 	  PORTD |= (1 << PD4); PORTD &= ~(1 << PD7);  // Right Motor On Reverse
	  PORTD |= (1 << PD2); PORTD &= ~(1 << PD3);  // Left Motor On Reverse
          break;
    case FULL_STOP:
	  PORTD &= ~(1 << PD4); PORTD &= ~(1 << PD7);  // Right Motor Off
	  PORTD &= ~(1 << PD2); PORTD &= ~(1 << PD3);  // Left Motor Off
          break;
  }
}
// BRAM Motor Speed Control
// PD5 - OC0B -> ENB1 (Left Motor)
// PD6 - OC0A -> ENB2 (Right Motor)
void BRAM_DriveMotor(int left_speed, int right_speed)
{
  unsigned char left_pwm,right_pwm;
  if (left_speed >= 0 && right_speed >= 0) {
    // Move Forward
    BRAM_Steer(MOVE_FORWARD);
    left_pwm=left_speed;
    right_pwm=right_speed;
  } else if (left_speed < 0 && right_speed < 0) {
    // Move Backward
    BRAM_Steer(MOVE_BACKWARD);
    left_pwm=left_speed * -1;
    right_pwm=right_speed * -1;
  } else if (left_speed < 0 && right_speed >= 0) {
    // Rotate Left
    BRAM_Steer(ROTATE_LEFT);
    left_pwm=left_speed * -1;
    right_pwm=right_speed;
  } else if (left_speed >= 0 && right_speed < 0) {
    // Rotate Right
    BRAM_Steer(ROTATE_RIGHT);
    left_pwm=left_speed;
    right_pwm=right_speed * -1;
  } else {
    // Full Stop
    BRAM_Steer(FULL_STOP);
    left_pwm=0;
    right_pwm=0;
  }
  // Assigned the value to the PWM Output Compare Registers A and B
  OCR0A=right_pwm;
  OCR0B=left_pwm;
}
/* START I2C Routine */
unsigned char i2c_transmit(unsigned char type) {
  switch(type) {
     case I2C_START:    // Send Start Condition
       TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
       break;
     case I2C_DATA:     // Send Data with No-Acknowledge
       TWCR = (1 << TWINT) | (1 << TWEN);
       break;
     case I2C_DATA_ACK: // Send Data with Acknowledge
       TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
       break;
     case I2C_STOP:     // Send Stop Condition
       TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
       return 0;
  }
  // Wait for TWINT flag set on Register TWCR
  while (!(TWCR & (1 << TWINT)));
  // Return TWI Status Register, mask the prescaller bits (TWPS1,TWPS0)
  return (TWSR & 0xF8);
}
char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type)
{
  unsigned char n = 0;
  unsigned char twi_status;
  char r_val = -1;
i2c_retry:
  if (n++ >= MAX_TRIES) return r_val;
  // Transmit Start Condition
  twi_status=i2c_transmit(I2C_START);
  // Check the TWI Status
  if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
  if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
  // Send slave address (SLA_W)
  TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | rw_type;
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);
  // Check the TWSR status
  if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
  if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
  r_val=0;
i2c_quit:
  return r_val;
}
void i2c_stop(void)
{
  unsigned char twi_status;
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_STOP);
}
char i2c_write(char data)
{
  unsigned char twi_status;
  char r_val = -1;
  // Send the Data to I2C Bus
  TWDR = data;
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);
  // Check the TWSR status
  if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;
  r_val=0;
i2c_quit:
  return r_val;
}
char i2c_read(char *data,char ack_type)
{
  unsigned char twi_status;
  char r_val = -1;               
  if (ack_type) {
    // Read I2C Data and Send Acknowledge
    twi_status=i2c_transmit(I2C_DATA_ACK);
    if (twi_status != TW_MR_DATA_ACK) goto i2c_quit;
  } else {
    // Read I2C Data and Send No Acknowledge
    twi_status=i2c_transmit(I2C_DATA);
    if (twi_status != TW_MR_DATA_NACK) goto i2c_quit;
  }
  // Get the Data
  *data=TWDR;
  r_val=0;
i2c_quit:
  return r_val;
}
void Write_MCP23008(unsigned char reg_addr,unsigned char data)
{
   // Start the I2C Write Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
   // Sending the Register Address
   i2c_write(reg_addr);
   // Write data to MCP23008 Register
   i2c_write(data);
   // Stop I2C Transmission
   i2c_stop();
}
unsigned char Read_MCP23008(unsigned char reg_addr)
{
   char data;
   // Start the I2C Write Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
   // Read data from MCP23008 Register Address
   i2c_write(reg_addr);
   // Stop I2C Transmission
   i2c_stop();
   // Re-Start the I2C Read Transmission
   i2c_start(MCP23008_ID,MCP23008_ADDR,TW_READ);
   i2c_read(&data,NACK);
   // Stop I2C Transmission
   i2c_stop();
   return data;
}
unsigned int BRAM_IRSensor()
{
   static unsigned int old_val = TARGET_VAL;
   unsigned int map_val;
   unsigned char sensor_val,ptr;
   // Turn On the Sensor IR LED
   Write_MCP23008(GPIO,0b00011111);    
   // Read sensor
   sensor_val = Read_MCP23008(GPIO) & 0x1F;
   // Turn Off the Sensor IR LED
   Write_MCP23008(GPIO,0b00111111);
   // Return Value from Sensor Map
   map_val=TARGET_VAL;        // Default always on center
   if (sensor_val > 0) {
     for(ptr = 0; ptr < MAX_MAP; ptr++) {
       // Now get the sensor map value
       if (pgm_read_word(sensor_map + ptr) == sensor_val) {
	 map_val=pgm_read_word(sensor_map + ptr + 1);
       }
     }
     old_val=map_val;   // Save the Current IR Array Value
   } else {
     map_val=0;
     // Adjust for zero result if previous value greater than 5
     if (old_val > TARGET_VAL) map_val=MAX_SENSOR_MAP;
   }
   // Display IR Sensor Value in Debugging Mode
#if BRAM_DEBUG
   ansi_cm(3,1);
   printf("IR Sensor: %02x, Map Value: %03d",sensor_val,map_val);
#endif
   return map_val;
}
void BRAM_PIDControl(unsigned int sensor_val)
{
   int motor_res,err_func;
   float KP,KI,KD,cont_res;
   // Get the Error Function
   err_func=sensor_val - TARGET_VAL;
   // Divide all the PID parameters for decimal value
   KP=Kp * 0.1;
   KI=Ki * 0.01;
   KD=Kd * 0.01;
   // Calculate the Motor Response using PID Control Equation
   cont_res=(float)(prev_res + KP * (err_func - prev_err_1) + KI * (err_func + prev_err_1)/2.0
               + KD * (err_func - 2.0 * prev_err_1 + prev_err_2));        
   // Now we have to Limit the control response to the Maximum of our motor PWM Motor Value
   motor_res=(int)cont_res;
   if (motor_res > MaxSpeed)
     motor_res = MaxSpeed;
   if (motor_res < -MaxSpeed)
     motor_res = -MaxSpeed;  
   // Save the Motor Response and Error Function Result
   prev_res=motor_res;
   prev_err_2=prev_err_1;
   prev_err_1=err_func;
   // Display Motor Response Value in Debugging Mode
#if BRAM_DEBUG
   ansi_cm(4,1);
   printf("Motor Response Factor: %d   ",motor_res);
#endif
   // Negative result mean BRAM is on the right, so we need to adjust to the left
   // Positive result mean BRAM is on the left, so we need to adjust to the right
   if (motor_res < 0)
     BRAM_DriveMotor(MaxSpeed + motor_res,MaxSpeed); // Left less speed, Right full speed
   else
     BRAM_DriveMotor(MaxSpeed,MaxSpeed - motor_res); // Left full speed, Right less speed
}
unsigned int getnumber(unsigned int min, unsigned int max)
{
  int inumber;
  scanf("%d",&inumber);
  if (inumber < min || inumber > max) {
    printf("\n\nInvalid [%d to %d]!",min,max);
    _delay_ms(500);
    return -1;
  }
  return inumber;
}
void Read_Parameter(void)
{
  // Read the Kp,Ki and Kd From EEPROM at Address: 0x00,0x02,0x04
  Kp=eeprom_read_word((unsigned int*) 0x0000);
  Ki=eeprom_read_word((unsigned int*) 0x0002);
  Kd=eeprom_read_word((unsigned int*) 0x0004);
}
// Assign I/O stream to UART
FILE uart_str = FDEV_SETUP_STREAM(uart_putch, uart_getch, _FDEV_SETUP_RW);
int main()
{
  unsigned char mode,press_tm;
  unsigned int ir_sensor;
  int ichoice;
  // Initial PORT Used
  DDRB = 0b11111110;     // Set PORTB: PB0=Input, Others as Output
  PORTB = 0;
  DDRC = 0b00000000;     // Set PORTC: Input
  PORTC = 0xFF;          // Pull-Up Input
  DDRD = 0b11111111;     // Set PORTD: Output
  PORTD = 0;
  // Define Output/Input Stream
  stdout = stdin = &uart_str;
  // Initial UART Peripheral
  uart_init();
  // Initial BRAM Terminal Screen
  ansi_me();
  ansi_cl();                            // Clear Screen
#if BRAM_DEBUG
  ansi_cm(1,1);
  printf("Welcome to AVRJazz Mega168 BRAM Debugging Mode");
#endif
  // Initial The 8-bit PWM 0
  // Fast PWM Frequency = fclk / (N * 256), Where N is the prescale
  // f_PWM = 11059200 / (8 * 256) = 5400 Hz
  TCCR0A = 0b10100011; // Fast PWM 8 Bit, Clear OCA0/OCB0 on Compare Match, Set on TOP
  TCCR0B = 0b00000010; // Used 8 Prescaler
  TCNT0 = 0;           // Reset TCNT0
  OCR0A = 0;           // Initial the Output Compare register A & B
  OCR0B = 0;
  // Initial ATMega168 TWI/I2C Peripheral
  TWSR = 0x00;         // Select Prescale of 1
  // SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
  TWBR = 0x30;        // 48 Decimal
  // Initial the MCP23008 Devices GP0 to GP4 Input, GP5 to GP7 Output
  Write_MCP23008(IODIR,0b00011111);
  Write_MCP23008(GPPU,0b00011111);    // Enable Pull-Up on Input
  Write_MCP23008(GPIO,0b00111111);    // Reset all the Output Port, Make GP5 High
  // Set ADCSRA Register on ATMega168
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
  // Set ADMUX Register on ATMega168
  ADMUX = (1<<ADLAR); // Use Right Justified, Select Channel 0
  // Initial the Motor
  BRAM_DriveMotor(0,0);
  mode=0;              // Default BRAM Off
  press_tm=0;
  // Initial PID Control Variables
  prev_res=0;
  prev_err_1=0;
  prev_err_2=0;
  MaxSpeed = 150;	  // Initial Maximum Speed
   // Read Default BRAM Parameter from EEPROM
  Read_Parameter();
  for(;;) {            // Loop Forever
    // Check if Button is pressed than enter to the Setup Mode
    if (bit_is_clear(PINB, PB0)) {          // if button is pressed
      _delay_us(100);                       // Wait for debouching
      if (bit_is_clear(PINB, PB0)) {
        press_tm++;
        if (press_tm > 100) {
          press_tm=0;
          mode++;
          if (mode > 2)
    	    mode = 0;
         }
       }
     }   			
     // Start conversion by setting ADSC on ADCSRA Register
     ADCSRA |= (1<<ADSC);
     // wait until convertion  complete ADSC=0 -> Complete
     while (ADCSRA & (1<<ADSC));
     // Get ADC the Result
     MaxSpeed = ADCH;
     if (mode == 0) {
       // Initial PID Control Variables
       prev_res=0;
       prev_err_1=0;
       prev_err_2=0;
     }
    if (mode == 1) {
      // Read the IR Sensor
      ir_sensor=BRAM_IRSensor();
      // Give some delay Before PID Calculation
      _delay_us(50);
      // Execute the BRAM LFR PID Controller
      BRAM_PIDControl(ir_sensor);
    }
    // Entering BRAM PID Setup Mode
    if (mode == 2) {
      // Stop BRAM Motors
      BRAM_DriveMotor(0,0);
      // Initial BRAM Terminal Screen
      uart_flush();                         // Flush UART
      ansi_me();
      ansi_cl();                            // Clear Screen
      ansi_cm(1,1);
      printf("Welcome to AVRJazz Mega168 BRAM Setup");
      ansi_cm(3,1);
      printf("BRAM Maximum Speed Value: %d",MaxSpeed);
      ansi_cm(5,1);
      printf("1. Kp - Proportional Parameter Factor (%d)\n",Kp);
      printf("2. Ki - Integral Parameter Factor (%d)\n",Ki);
      printf("3. Kd - Derivative Parameter Factor (%d)\n",Kd);
      printf("4. Save Parameters to the EEPROM\n");
      printf("5. Refresh Setup\n");
      printf("6. Exit\n");
      printf("\nEnter Choice: ");
      if ((ichoice=getnumber(1,6)) < 0) continue;	
      switch (ichoice) {
	    case 1:  // Kp Parameter
		  printf("\n\nKp Parameter [0-1000]: ");
		  if ((Kp=getnumber(0,1000)) < 0) continue;
		  break;
            case 2:  // Ki Parameter
		  printf("\n\nKi Parameter [0-1000]: ");
		  if ((Ki=getnumber(0,1000)) < 0) continue;
		  break;
            case 3:  // Kd Parameter
		  printf("\n\nKd Parameter [0-1000]: ");
		  if ((Kd=getnumber(0,1000)) < 0) continue;
		  break;
            case 4:  // Save to EEPROM
		  // Write the Kp,Ki and Kd to EEPROM Address: 0x0000,0x0002,0x0004
		  eeprom_write_word((unsigned int*) 0x0000,Kp);
		  eeprom_write_word((unsigned int*) 0x0002,Ki);
		  eeprom_write_word((unsigned int*) 0x0004,Kd);
		  // Read BRAM Parameter from EEPROM
                  Read_Parameter();
		  break;
	    case 5:	 // Refresh Setup
  	  	  // Start conversion by setting ADSC on ADCSRA Register
	          ADCSRA |= (1<<ADSC);
	          // wait until convertion  complete ADSC=0 -> Complete
                  while (ADCSRA & (1<<ADSC));
	          // Get ADC the Result
	          MaxSpeed = ADCH;
		  // Read BRAM Parameter from EEPROM
                  Read_Parameter();
		  break;
	    case 6:  // Exit Setup
		  mode = 0;
		  ansi_cl();
		  break;
      }
    }
  }
  return 0;	           // Standard Return Code
}
/* EOF: bramflr.c */
