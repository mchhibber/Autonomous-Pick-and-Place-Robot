/*
* Team Id		 : <#NS 4716>
* Author List	 : <Manav Chhibber>
* Filename		 : <main.h>
* Theme			 : <Nutty Squirel>
* Functions		 : <port_init(), timer5_init(), velocity(unsigned char,unsigned char), adc_pin_config(), adc_init(), motion_pin_config(),
					ADC_Conversion(unsigned char Ch), print_sensor(char, char,unsigned char), forward(), back(), left(), right(), soft_left(),
					soft_right(), stop(), adjust_pick(), adjust_place(); buzzer_config(), buzzer_on(), buzzer_off(), init_devices()>
* GlobalVariables: <ADC_Value>
* File for main pin config and initialization
*/

/*
Connections:
*Motor	:	PA0-> L1	PA1-> L2
			PA2-> R1	PA3-> R2
			PL3(0C5A)-> RIGHT	PL4(OC5B)-> LEFT
*WHITE LINE SENSOR:	PF1-PF3
*SHAR IR:	PK1
*/

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>




void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;






//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}



// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//Initialize ADC registers 
void adc_init()
{
	//ADC Control and Status Register A 
	ADCSRA = 0x00;	
	//ADC Control and Status Register B  
	ADCSRB = 0x00;		//MUX5 = 0
	//ADC Multiplexer Selection Register
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	//Analog Comparator Control and Status Register 
	ACSR = 0x80;		//Analog Comparator Disable
	
	ADCSRA = 0x86;		//ADEN=1 --- ADSP1=1 --- ADPS2=1
}

/*
* Function Name	: <ADC_Conversion(unsigned char Ch)>
* Input			: <Ch-> representing the channel no. for which ADC conversion has to be done>
* Output		: <a value between 0-255 representing the equivalent voltage at the channel>
* Logic			: <ADC starts reading the analog signal immediately from the input channel (specified in ADMUX as explained above) 
				   and converts it to a number b/w 0-255. After finishing the conversion,the ADC module writes the output of the conversion 
				   to register ADCH from which it is taken and returned>
* Example Call	: <ADC_Conversion(1)>
*/
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
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/*
* Function Name	: <print_sensor>
* Input			: <row-> specifying row of lcd, column-> specifying column of lcd, channel-> channel of which ADC value has to be displayed>
* Output		: <None>
* Logic			: <Finds ADC converted value and displays on lcd >
* Example Call	: <print_sensor(1,1,1)>
*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
* Function Name	: <velocity>
* Input			: <left_motor-> Velocity of left wheel, right_motor-> Velocity of right wheel>
* Output		: <None>
* Logic			: <Modify PWM duty cycle to change velocity of wheels >
* Example Call	: <velocity(180,180)>
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

//Move both wheels in forward direction
void forward (void) 
{
	
  motion_set(0x05);
}

//Move both wheels in backward direction
void back (void)
{
	motion_set(0x0A);
}

//Moves robot in right direction (left wheel forwards, right wheel backwards)
void right (void)
{
	motion_set(0x09);
}

//Moves robot in left direction (right wheel forward, left wheel backward)
void left (void)
{
	motion_set(0x06);
}

//Left wheel stopped, Right wheel forwards


//Right wheel stopped, Left wheel forwards


//Stops both wheels
void stop (void)
{
   motion_set(0x00);
}

/*
* Function Name	: <adjust_pick>
* Input			: <None>
* Output		: <None>
* Logic			: <Moves robot back to adjust to pick nuts >
* Example Call	: <adjust_pick()>
*/
void adjust_pick()
{
	back();
	_delay_ms(565);
	stop();
	servo_2(93);		//open gripper
	_delay_ms(1000);
	servo_1(13);		//lower the arm
	_delay_ms(3000);
	forward();
	_delay_ms(300);
	stop();
}

/*
* Function Name	: <adjust_place>
* Input			: <None>
* Output		: <None>
* Logic			: <Moves robot back to adjust to place nuts >
* Example Call	: <adjust_place()>
*/
void adjust_place()
{
	back();
	_delay_ms(575);
	stop();
}

//Buzzer Pin Configuration
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//To Configure Buzzer On
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

//To configure Buzzer Off
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
//Function to Initialize PORTS
void port_init()
{
	buzzer_pin_config();
	servo_config();//Servo configuration included in arm.h
	lcd_port_config();// lcd configuration include in lcd.h
	adc_pin_config();// adc configuration
	motion_pin_config();//motor pin configuration
	color_sensor_init();// color sensor configuration included in color.h
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();//initialize adc registers
	timer1_init();//initialize Timer 1
	timer5_init();//initialize Timer 2
	sei();   //Enables the global interrupts
}


