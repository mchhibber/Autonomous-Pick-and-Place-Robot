/*
* Team Id		 : <#NS 4716>
* Author List	 : <Manav Chhibber>
* Filename		 : <arm.h>
* Theme			 : <Nutty Squirel>
* Functions		 : <servo_config(), timer1_init(), servo_1(unsigned char degrees), servo_2(unsigned char degrees), servo_1_free(), servo_2_free()>
* GlobalVariables: <None>
* File for arm mechanism
*/

/*
Connections:
*Servo 1: PB5
*Servo 2: PB6
*/

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function

void servo_config();
void timer1_init();
void servo_1(unsigned char degrees);
void servo_2(unsigned char degrees);


void servo_config()
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
	
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//TIMER1 initialization in 10 bit fast PWM mode  
//Prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
* Function Name	: <servo_1>
* Input			: <degrees-> Angle in degrees by which the servo should rotate>
* Output		: <None>
* Logic			: <Width of positive pulse is used to control servo angle, here servo is rotated in angles multiple of 1.86 degrees>
* Example Call	: <servo_1(93)>
*/
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


/*
* Function Name	: <servo_2>
* Input			: <degrees-> Angle in degrees by which the servo should rotate>
* Output		: <None>
* Logic			: <Width of positive pulse is used to control servo angle, here servo is rotated in angles multiple of 1.86 degrees>
* Example Call	: <servo_2(93)>
*/
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}


/*
* Function Name	: <servo_1_free>
* Input			: <None>
* Output		: <None>
* Logic			: <This function is used to unlock servo motor from any angle. Used to lower arm for picking nuts>
* Example Call	: <servo_1_free()>
*/
void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

/*
* Function Name	: <servo_2_free>
* Input			: <None>
* Output		: <None>
* Logic			: <This function is used to unlock servo motor from any angle. Used to release nuts >
* Example Call	: <servo_2_free()>
*/
void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

/*
* Function Name	: <pick>
* Input			: <None>
* Output		: <None>
* Logic			: <This function is used to pick nuts using arm. Servos are rotated to appropiate angles to pick nut. >
* Example Call	: <pick()>
*/
void pick()
{
	
	
	servo_2(25);		//grab nut
	_delay_ms(2000);	
	servo_1(153);		//elevate the arm
	_delay_ms(3000);
	rgb_off();
	servo_1_free();
	servo_2_free();
	
}

/*
* Function Name	: <place>
* Input			: <None>
* Output		: <None>
* Logic			: <This function is used to place nuts using arm. Servos are rotated to appropiate angles to place nut. >
* Example Call	: <place()>
*/
void place()
{
	servo_2(43);
	_delay_ms(100);
	servo_1(13);		//lower the arm
	_delay_ms(3000);
	servo_2(93);		//release the nut
	_delay_ms(2000);
	
	servo_1(153);		//elevate the arm
	_delay_ms(3000);
	servo_1_free();
	servo_2_free();
	
}

