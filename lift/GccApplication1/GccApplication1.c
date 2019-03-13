/*
 * GccApplication1.c
 *
 * Created: 2/3/2019 7:44:34 PM
 *  Author: Lenovo
 */ 

#define F_CPU 16000000//define F_CPU value
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
int flag=0;
int dir=1;
unsigned int motor_flag=1;

void motor_port_config()
{
	DDRB|=0x03;
	PORTB&=0xFC;//off motor
}
void interrupt_config(void) //Interrupt 0 enable
{
	PORTD=0x0C;//pull-up
	EICRA = 0x00; // INT0 and INT1 is set to trigger with low
	EIMSK = 0x03; // Enable Interrupt INT0 for limit and INT1 for motor/IR
	
}
ISR(INT0_vect)
{
	PORTB&=0xFC;//stops motor
	dir=dir*(-1);
	_delay_ms(5000);
	flag=1;
}
ISR(INT1_vect)
{
	
		if(dir==1)
			PORTB=0x02;//starts motor upwards
		else
			PORTB=0x01;//starts motor downwards
	flag=1;
}
void init_devices()
{
	motor_port_config();
	interrupt_config();
}
int main(void)
{
    init_devices();
	while(1)
	{
		if(flag)
		{
			cli();
			_delay_ms(100);
			flag=0;
		}
		else
			sei();
	}
}