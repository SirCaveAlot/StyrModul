/*
 * Styrmodul_SirCave.c
 *
 * Created: 4/3/2017 1:51:03 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include "PWM_grip_arm.h"
#include "PWM_SirCave.h"
#include "UART.h"

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

char data;
char buffer;

int main(void)
{
	DDRD = 0xFE;
	DDRA = 0xFF;
	
	Timer1_init();
	Timer2_init();
	USART_Init(UBBR);
	Interrupt_Init();
	Open_grip_arm();

	while(1)
	{
		while(data == 'f')
		{
			Drive_forward(0.9, 0.9);
		}
		while(data == 'b')
		{
			Drive_backwards(0.9, 0.9);
		}
		while(data == 'l')
		{
			Rotate_counter_clockwise(0.9, 0.9);
		}
		while(data == 'r')
		{
			Rotate_clockwise(0.9, 0.9);
		}
		while(data == 's')
		{
			Drive_forward(0, 0);
		}
		while(data == 'o')
		{
			Open_grip_arm();
		}
		while(data == 'c')
		{
			Close_grip_arm();
		}
		while(data == 'm')
		{
			Center_grip_arm();
		}
	}
}