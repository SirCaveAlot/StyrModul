/*
 * Kommunikationsmodul.c
 *
 * Created: 4/13/2017 1:27:36 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "UART.h"

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

bool mode_changed = true; 

int main(void)
{
	USART_Init(UBBR);
	Interrupt_Init();
	DDRD |= (1 << PORTD1);
    
	while(1)
    {
		if(mode_changed == true)
		{
			Data_transmission();
		}
    }
}