/**************************************************************************************************
		Platform: Fire Bird V ATMEGA 2560
		IO Operations Buzzer Beep
		Written by: Bharat Jain(09305029) and Dinesh Rajpoot (09305043) 
		Last Modification: 2010-11-07
		This header file contains useful functions.
		Compiled with: AVR Studio 4.17
**************************************************************************************************/

/********************************************************************************

   Copyright (c) 2010, ERTS Lab, IIT Bombay
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <util/delay.h>

#include <math.h>


int data,data3;
unsigned long int ShaftCountLeft = 0; 	//to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; 	//to keep track of right position encoder
unsigned int Degrees; 					//to accept angle in degrees for turning
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
float BATT_Voltage, BATT_V;
//int data;
//int data1;



/**
 * Function to Initialize PORTS 
**/
void init_ports(void)
{
 PORTA = 0xFF;
 DDRA  = 0xFF;
 PORTB = 0xF0;
 DDRB  = 0xF0;
 PORTC = 0x00; 
 DDRC  = 0xFF;
 PORTD = 0x00;
 DDRD  = 0x00;
 PORTE = 0x8F;
 DDRE  = 0x8F;
 PORTF = 0xFF;
 DDRF  = 0x00;
 PORTG = 0x00;
 DDRG  = 0x00;
 PORTH = 0x00;
 DDRH  = 0x0C;
 PORTJ = 0xAA;
 DDRJ  = 0xFF;
 PORTK = 0x00;
 DDRK  = 0x00;
 PORTL = 0x38;
 DDRL  = 0x38;
}

/**
* Function To Initializa TIMER1 
* prescale:256
* WGM: 7) PWM 10bit fast, TOP=0x03FF
* desired value: 40Hz
* actual value: 42.187Hz (5.2%)
**/ 

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter High Value To Which OCR1xH Value Is Compared With
 TCNT1L = 0x01;	//Counter High Value To Which OCR1xL Value Is Compared With
 OCR1AH = 0x03;	//Output Compare Register High Value For S1
 OCR1AL = 0xFF;	//Output Compare Register Low Value For S1
 OCR1BH = 0x03;	//Output Compare Register High Value For S2
 OCR1BL = 0xFF;	//Output Compare Register Low Value For S2
 OCR1CH = 0x00;	//Output Compare Register High Value For S3
 OCR1CL = 0x00;	//Output Compare Register Low Value For S3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAF; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionalit to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/**
* Function to Initialize TIMER5
* Prescale:64
* PWM 8bit fast, TOP=0x00FF
* Timer Frequency:674.988Hz
**/
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter Higher 8-bit value To Which OCR5xH Value is Compared With
	TCNT5L = 0x01;	//Counter Lower 8-bit value To Which OCR5xL Value is Compared With
	OCR5AH = 0x00;	//Output Compare Register High Value For Left Motor
	OCR5AL = 0xFF;	//Output Compare Register Low Value For Left Motor
	OCR5BH = 0x00;	//Output Compare Register High Value For Right Motor
	OCR5BL = 0xFF;	//Output Compare Register Low Value For Right Motor
	OCR5CH = 0x00;	//Output Compare Register High Value For Motor C1
	OCR5CL = 0xFF;	//Output Compare Register Low Value For Motor C1
	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionalit to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM52=1; CS52=0, CS51=1, CS50=1 (Prescaler=64)
}


/** 
* Function to Initialize ADC 
**/
void init_adc()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}








/**
* Function To Initialize UART0 ***
* desired baud rate:9600
* actual baud rate:9600 (0.0%)
* char size: 8 bit
**/ parity: Disabled

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

/**
* Function To Initialize UART1 
* desired baud rate:9600
* actual baud rate:9600 (0.0%)
* char size: 8 bit
* parity: Disabled
**/
/*void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x47; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
}*/


void position_encoder_interrupt_init(void)
{
cli();
EICRB = EICRB | 0x0A;
EIMSK = EIMSK | 0x30;
sei();
}




/**
* Function for Speed Control 
**/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	if(left_motor>254)
	left_motor = 255;	//limiting the max velocity

	if(right_motor>254)
	right_motor = 255;	//limiting the max velocity

	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


void LEFT_SPEED(int val)
{

	if(val>254)
		val = 255;	//limiting the max velocity

	OCR5AL = (unsigned char)val;


}


void RIGHT_SPEED(int val)
{

	if(val>254)
		val = 255;	//limiting the max velocity

	OCR5BL = (unsigned char)val;


}



int send (data3)
	{
		UDR0=data3;
		return(UDR0);
	}









/**
*Function used for setting motor's direction
**/




void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel for the protection
 PortARestore = PORTA; 			// reading the PORTA original status
 PortARestore &= 0xF0; 			// making lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 			// executing the command
}

void forward(void) //both wheels forward
{

  motion_set(0x06);
}

void back(void) //both wheels backward
{

  motion_set(0x09);
}

void soft_left(void) //Left wheel backward, Right wheel forward
{

  motion_set(0x05);
}

void soft_right(void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void move_left(void) //Left wheel stationary, Right wheel forward
{

 motion_set(0x04);
 velocity(0,110);
}

void move_right(void) //Left wheel forward, Right wheel is stationary
{
velocity(110,0);
 motion_set(0x02);
}

void inplace_left(void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void inplace_right(void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop(void)
{
  motion_set(0x00);
}

/**
*  This Function accepts the Channel Number and returns the corresponding Analog Value 
**/
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10;
	ADCSRB = 0x00;
	return a;
}



/**
 * This Function prints the Analog Value Of Corresponding Channel No. at required Row
 * and Coloumn Location. 
**/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/**
 * Converting Received ADC Data From Slave in Decimal Value and displaying on LCD 
**/


/**
 * This Function calculates the actual distance in millimeters(mm) from the input
 * analog value of Sharp Sensor. 
**/
unsigned int Sharp_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/**
* Function To Switch Buzzer ON 
**/
void BUZZER_ON(void)
{
 unsigned char portc_restore = 0;
 
 portc_restore = PORTC; // reading the PORTC original status
 portc_restore |= 0x08; // setting the bit to turn on the buzzer
 PORTC = portc_restore; // executing the command

_delay_ms(3000);

}

/**
* Function To Switch Buzzer OFF 
**/
void BUZZER_OFF (void)
{
 unsigned char portc_restore = 0;
 
 portc_restore = PORTC; // reading the PORTC original status
 portc_restore &= 0xF7; // resetting the bit to turn off the buzzer
 PORTC = portc_restore; // executing the command
}

/**
* Function To Switch Front Sharp And White Line Sensors ON 
**/
void sharp_and_white_line_sensor_on(void)  //sharp and white line sensor  ON
{  
unsigned char portg_restore = 0;
 
 portg_restore = PORTG; // reading the PORTG original status
 portg_restore |= 0x04; // setting bit to turn on sensors
 PORTG = portg_restore; // executing the command
}

/**
* Function To Switch Front Sharp And White Line Sensors OFF 
**/
void sharp_and_white_line_sensor_off(void)  //sharp and white line sensor OFF
{
unsigned char portg_restore = 0;
 
 portg_restore = PORTG; // reading the PORTG original status
 portg_restore &= 0xFB; // resetting the bit to turn off  sensors
 PORTG = portg_restore; // executing the command
}


/**
*  Function To Switch  Analog IR- Proximity Sensors ON 
**/
void analog_IR_proximity_sensors_on (void)
{
unsigned char porth_restore = 0;
 
 porth_restore = PORTH; // reading the PORTH original status
 porth_restore |= 0x04; // setting the bit to turn on tcrt sensors
 PORTH = porth_restore; // executing the command
}

/**
* Function To Switch  Analog IR- Proximity Sensors OFF 
**/
void analog_IR_proximity_sensors_off (void)
{
unsigned char porth_restore = 0;
 
 porth_restore = PORTH; // reading the PORTH original status
 porth_restore |= 0xFB; // resetting the bit to turn off  tcrt sensors
 PORTH = porth_restore; // executing the command
}

/**
* Function To Switch  Side Sharp Sensors ON 
**/
void side_sharp_sensors_on (void)
{
unsigned char porth_restore = 0;
 
 porth_restore = PORTH; // reading the PORTH original status
 porth_restore |= 0x08; // setting the bit to turn on side sharp sensors
 PORTH = porth_restore; // executing the command
}

/**
* Function To Switch  Side Sharp Sensors OFF 
**/
void side_sharp_sensors_off (void)
{
unsigned char porth_restore = 0;
 
 porth_restore = PORTH; // reading the PORTH original status
 porth_restore |= 0xF7; // resetting the bit to turn off  side sharp sensors
 PORTH = porth_restore; // executing the command
}


/**
* Motors Delay Function 
**/
void motors_delay()
{
	unsigned int i;
	for(i=0; i<5;i++)
	{
		_delay_ms(100);
	}
}

/**
* Buzzer Delay Function 
**/
void buzzer_delay()
{
	unsigned int i;
	for(i=0; i<5;i++)
	{
		_delay_ms(100);
	}
}


/**
* ISR for right position encoder
**/
ISR(INT5_vect)  
{
	ShaftCountRight++;  //increment right shaft position count
}


/**
* ISR for left position encoder
**/
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}



/**
*  Function To Initialize all the Peripherals 
**/
void init_devices(void)
{
 
 cli(); //disable all interrupts
 
 init_ports();
 port_init();
 timer1_init();
 timer5_init();
 init_adc();
// spi_init();
 //uart0_init();
 uart1_init();
// uart2_init();
 lcd_set_4bit();
 lcd_init();

 EICRA = 0x00; //Falling edge of signal generate INT0 & INT1
 EICRB = 0x0A; // used to select trigger interrupt for INT7-INT4
 EIMSK = 0x30; // Enable Interrupt INT4, INT5- for position encoders

 sei(); //re-enable interrupts
 //all peripherals are now initialized
}



//  lcd functions



#define FCPU 11059200ul
#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC

#define sbit(reg,bit)	reg |= (1<<bit)
#define cbit(reg,bit)	reg &= ~(1<<bit)

/*void init_ports();
void lcd_reset_4bit();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_home();
void lcd_cursor(char, char);
void lcd_print(char, char, unsigned int, int);
void lcd_string(char*);
*/


unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;

int i;


/**
* Function to Reset LCD
**/
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 to initialise LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	
}

/**
* Function to Initialize LCD 
**/
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28);			//LCD 4-bit mode and 2 lines.
	lcd_wr_command(0x01);
	lcd_wr_command(0x06);
	lcd_wr_command(0x0E);
	lcd_wr_command(0x80);
		
}

	 
/**
* Function to Write Command on LCD
**/
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

/**
* Function to Write Data on LCD
**/
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}





void lcd_home()
{
	lcd_wr_command(0x80);
}

/**
* Function to Print String on LCD
**/
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

/**
* Position the LCD cursor at "row", "column". 
**/

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

/**
*  Function To Print Any input value upto the desired digit on LCD 
**/
void lcd_print (char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
	
}




//  SIGNALS


//#define MOTOR_SHAFT_L shaft_count_left()
//#define MOTOR_SHAFT_R shaft_count_right()

#define MOTOR_SHAFT_L ShaftCountLeft
#define MOTOR_SHAFT_R ShaftCountRight
#define SHAFT_RESER_BOTH reset_both()




void reset_both()
{

	ShaftCountLeft=0;
	ShaftCountRight=0;
}

/*
#define LEFT_WHITELINE_VALUE ADC_Conversion(3)
#define MIDDLE_WHITELINE_VALUE ADC_Conversion(2)
#define RIGHT_WHITELINE_VALUE ADC_Conversion(1)
*/




#define WL_LEFT ADC_Conversion(3)
#define WL_MIDDLE ADC_Conversion(2)
#define WL_RIGHT ADC_Conversion(1)


/*#define FRONT_IR_VALUE front_dist_mm()
#define EXTREME_LEFT_IR_VALUE extreme_left_distance_mm()
#define LEFT_IR_VALUE left_distance_mm()
#define RIGHT_IR_VALUE right_distance_mm()
#define EXTREME_RIGHT_IR_VALUE extreme_right_distance_mm()
*/



int extreme_right_distance_mm(void)
{

	int extreme_right_distance_mm;
      unsigned char analog_val;
  	analog_val = ADC_Conversion(13);
 	extreme_right_distance_mm=Sharp_estimation(analog_val);
      return extreme_right_distance_mm; 	


}

int extreme_left_distance_mm(void)
{

	int extreme_left_distance_mm;
      unsigned char analog_val;
  	analog_val = ADC_Conversion(9);
 	extreme_left_distance_mm=Sharp_estimation(analog_val);
      return extreme_left_distance_mm; 	

}


int right_distance_mm(void)
{

	int right_distance_mm;
      unsigned char analog_val;
  	analog_val = ADC_Conversion(12);
 	right_distance_mm=Sharp_estimation(analog_val);
      return right_distance_mm; 	


}


int left_distance_mm(void)
{

	int left_distance_mm;
      unsigned char analog_val;
  	analog_val = ADC_Conversion(10);
 	left_distance_mm=Sharp_estimation(analog_val);
      return left_distance_mm; 	
}


int front_dist_mm(void) 
{
  int front_distance_mm;
  unsigned char analog_val;
  analog_val = ADC_Conversion(11);
  front_distance_mm=Sharp_estimation(analog_val);
  return front_distance_mm; 	
}



/*#define PROX_1 proximity_1()
#define PROX_2 proximity_2()
#define PROX_3 proximity_3()
#define PROX_4 proximity_4()
#define PROX_5 proximity_5()
*/


int proximity_1()
{
	int distance;
    unsigned char analog_val;
  	analog_val = ADC_Conversion(4);
 	//distance=Sharp_estimation(analog_val);
    return analog_val; 

}


int proximity_2()
{
	int distance;
    unsigned char analog_val;
  	analog_val = ADC_Conversion(5);
 	//distance=Sharp_estimation(analog_val);
    return analog_val; 

}


int proximity_3()
{
	int distance;
    unsigned char analog_val;
  	analog_val = ADC_Conversion(6);
 	//distance=Sharp_estimation(analog_val);
    return analog_val; 

}


int proximity_4()
{
	int distance;
    unsigned char analog_val;
  	analog_val = ADC_Conversion(7);
 	//distance=Sharp_estimation(analog_val);
    return analog_val; 

}


int proximity_5()
{
	int distance;
    unsigned char analog_val;
  	analog_val = ADC_Conversion(8);
 	//distance=Sharp_estimation(analog_val);
    return analog_val; 

}

void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
}

void servo_1(unsigned char degrees)
{
 float PositionPanServo = 0;
 PositionPanServo = ((float)degrees / 2.25) + 21.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
void servo_3(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionTiltServo;
}

/**
* servo_free functions unlocks the servo motors from the any angle
* and make them free by giving 100% duty cycle at the PWM. This function can be used to
* reduce the power consumption of the motor if it is holding load against the gravity.
**/
void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03;
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
}



		
/**
* Servo function to pick objects
**/
void pick()
{

servo_2(100);
_delay_ms(1000);
servo_1(100);
_delay_ms(1000);

for(int k =100; k> 65;k--)

{
servo_2(k);
_delay_ms(100);


 }

 //servo_1(135);
 for( int j = 100; j < 135;j++)
 {
 servo_1(j);
 _delay_ms(300);
}

for(int i = 65; i < 90; i++)
{
servo_2(i);
_delay_ms(100);
}

}
/**
* Servo function to place objects
**/
void place()
{

for(int i = 100; i > 65; i--)
{
servo_2(i);
_delay_ms(100);
}

servo_1(100);
_delay_ms(1000);
for(int i = 65; i < 100; i++)
{
servo_2(i);
_delay_ms(100);
}
}




/**
* Function to rotate firebird by given degree in left direction
**/
void angle_rotate_left(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	    unsigned long int ReqdShaftCountInt = 0;
		    ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
			ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
			ShaftCountRight = 0; 
			ShaftCountLeft = 0; 
			//	lcd_print(1, 1, ShaftCountRight, 3);      
				_delay_ms(10);		 
			
			     while(ShaftCountRight <= ReqdShaftCountInt)
				 { 
				
			
						lcd_print(1, 1, ShaftCountRight, 3);
						lcd_print(1, 9, ReqdShaftCountInt, 3);
						lcd_print(2, 9, Degrees, 3);
						move_left();
						velocity(150,150);
					}
					 
						stop();
			 
   			


}
/**
* Function to rotate firebird by given degree in right direction
**/
void angle_rotate_right(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	    unsigned long int ReqdShaftCountInt = 0;
		    ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
			ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
			ShaftCountRight = 0; 
			ShaftCountLeft = 0; 
		//	lcd_print(1, 5, ShaftCountLeft, 3);  
			_delay_ms(10);
			
			     while((ShaftCountLeft <= ReqdShaftCountInt))
				 { 
					    
						 
			
						lcd_print(1, 5, ShaftCountLeft, 3);
						move_right();
						velocity(150,150);
					}
					 
						stop();
			 
   			


}
/**
* Function to move firebird in forward by given distance in mm
**/
void linear_distance_mm(unsigned int DistanceInMM)
{ 	
	float ReqdShaftCount = 0;
		 unsigned long int ReqdShaftCountInt = 0;
		  	ReqdShaftCount = DistanceInMM / 5.338; 	// division by resolution to get shaft count
				 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
				  	ShaftCountLeft = 0; 	
					while(1) 	
					{
					  		
							if(ShaftCountLeft> ReqdShaftCountInt)
							 		 {
									   		break;
									}
									else
									{
										forward();
										velocity(150,150);
									}	
					  } 
					  stop(); //Stop action
}

/**
* ISR for receive complete interrupt
**/
SIGNAL(SIG_USART0_RECV) // 
{
	data = UDR0; //making copy of data from UDR0 in 'data' variable 



}





