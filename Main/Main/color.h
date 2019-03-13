/*
* Team Id		 :	<#NS 4716>
* Author List	 :	<Manav Chhibber>
* Filename		 :	<color-sharp.h>
* Theme			 :	<Nutty Squirel>
* Functions		 :	<void rgb_port_config (void), void glow(), void glow_blue(), void glow_green(), void glow_red(), void color_sensor_pin_config(), 
					void color_sensor_pin_interrupt_init(), void color_sensor_init(), void filter_red(void), void filter_blue(void), void filter_green(void),
					void filter_clear(void), void green_read(void), void blue_read(void), void red_read(void)>
* GlobalVariables:	<volatile unsigned int pulse=0, red, blue, green >
* File for Color Sensor control
*/

/*
Connections:
* RGB LED		:	R,G,B --> PA4,PA5,PA6
* COLOR SENSOR	:	S0->PG5		S1->PE3		S2->PH3		S3->PH4		OUT->PD0
*/
#define F_CPU 16000000;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void rgb_port_config (void);
int glow();
void color_sensor_pin_config();
void color_sensor_pin_interrupt_init();
void color_sensor_init();

volatile unsigned int pulse=0,red,blue,green;


//Function for rgb led port configuration
void rgb_port_config (void)
{
	DDRA |= 0x70;
	PORTA |= 0x70;

}

void rgb_off(void)
{
	PORTA |= 0x70;
}
//function to glow Red LED
void glow_red(void)
{
	
	PORTA |= 0x70;
	PORTA&=0x6F;
}

//function to glow Blue LED
void glow_blue(void)
{
	PORTA |= 0x70;
	PORTA &=0x3F;
}

//function to glow Green LED
void glow_green(void)
{
	PORTA |= 0x70;
	PORTA&=0x5F;
}

//Define DDR and PORT values for the port on which Color sensor is connected
void color_sensor_pin_config(void)
{
	// Color Sensor Scaling 
	// 20% Scaling
	DDRE|=0x08;
	PORTE&=0xF7;//S1 LOW
	DDRG|=0x20;
	PORTG|=0x20;//S0 HIGH
	
	//Configuring Ports for S3 and S2 pins
	DDRH|=0x18;
	PORTH&=0xEF;//S3 LOW
	PORTH|=0x08;//S2 HIGH
	
}

//Interrupt 0 enable
void color_sensor_pin_interrupt_init(void) 
{
	cli(); //Clears the global interrupt
	EICRA = 0x02; // INT0 is set to trigger with falling edge
	EIMSK = 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

//ISR for color sensor
ISR(INT0_vect) 
{
	//increment on receiving pulse from the color sensor
	pulse++;
}



//Filter Selection
void filter_red(void) //Filter Select - RED filter
{	
	PORTH&=0xF7;// S2 LOW
	PORTH&=0xEF;// S3 LOW
}

void filter_green(void)	//Filter Select - GREEN filter
{
	PORTH|=0x08;// S2 HIGH
	PORTH|=0x10;// S3 HIGH
}

void filter_blue(void)	//Filter Select - BLUE filter
{
	PORTH&=0xF7;// S2 LOW
	PORTH|=0x10;// S3 HIGH
}

void filter_clear(void)	//Filter Select - no filter
{
	PORTH&=0xEF;// S3 LOW
	PORTH|=0x08;// S2 HIGH
}


/*
* Function Name	: <red_read>
* Input			: <None>
* Output		: <None>
* Logic			: <Select red filter, count number of pulses in 100ms and store the count in variable red >
* Example Call	: <red_read()>
*/
void red_read(void) 
{
	//Red
	filter_red();//select red filter
	pulse = 0;//reset the count to 0
	_delay_ms(100);//capture the pulses for 100 ms or 0.1 second
	red = pulse;//store the count in variable called red

}

/*
* Function Name	: <green_read>
* Input			: <None>
* Output		: <None>
* Logic			: <Select green filter, count number of pulses in 100ms and store the count in variable green >
* Example Call	: <green_read()>
*/
void green_read(void) 
{
	//Green
	filter_green();//select green filter
	pulse = 0;//reset the count to 0
	_delay_ms(100);//capture the pulses for 100 ms or 0.1 second
	green = pulse;//store the count in variable called green

}

/*
* Function Name	: <blue_read>
* Input			: <None>
* Output		: <None>
* Logic			: <Select blue filter, count number of pulses in 100ms and store the count in variable blue >
* Example Call	: <blue_read()>
*/
void blue_read(void) 
{
	//Blue
	filter_blue(); //select blue filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	blue = pulse;  //store the count in variable called blue
	
}

/*
* Function Name	: <glow>
* Input			: <None>
* Output		: <Outputs the integer according to which color nut is detected. Red->1		Green->2	Blue->3
					e.g. if Red nut is detected it returns 1>
* Logic			: <Glows appropiate LED color based on the input from color sensor in accordance with the color of block infront>
* Example Call	: <glow()>
*/
int glow()
{
	red_read();
	blue_read();
	green_read();
	if(red>blue && red>green)
	{
		glow_red();
		return 1;
	}	
	else if(blue>red && blue>green)
	{
		glow_blue();
		return 3;
	}	
	else if(green>red && green>blue)
	{
		glow_green();
		return 2;
	}	
}

/*
* Function Name	: <color_sensor_init>
* Input			: <None>
* Output		: <None>
* Logic			: <Initialise PORTS for Color Sensor and RGB conrol>
* Example Call	: <color_sensor_init()>
*/
void color_sensor_init()
{
	rgb_port_config();
	color_sensor_pin_config();
	color_sensor_pin_interrupt_init();
}