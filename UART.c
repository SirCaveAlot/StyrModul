/*
 * UART.c
 *
 * Created: 3/31/2017 2:22:01 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include "PWM_SirCave.h"
#include "PWM_grip_arm.h"

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

char data;
char buffer;
void USART_Transmit(char data);

ISR(USART0_RX_vect)
{
	data = UDR0;
	/*
	if(data == 'f')
	{
		Drive_forward(0.5, 0.5);
	}
	if(data == 'b')
	{
		Drive_backwards(0.5, 0.5);
	}
	if(data == 'l')
	{
		Rotate_counter_clockwise(0.5, 0.5);
	}
	if(data == 'r')
	{
		Rotate_clockwise(0.5, 0.5);
	}
	if(data == 's')
	{
		Drive_forward(0, 0);
	}*/

	USART_Transmit('[');
	USART_Transmit(data);
	USART_Transmit(']');
	USART_Transmit('\n');
}

void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRR0H = 0;
	UBRR0L = 7;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = 0b00000110;
}

void USART_Transmit(char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

char USART_Receive( void )
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

void Interrupt_Init()
{
	UCSR0B |= (1<<RXCIE0);
	sei();
}

int main(void)
{
	DDRD = 0xFE;
	DDRA = 0xFF;
	
	Timer1_init();
	Timer2_init();
	USART_Init(UBBR);
	Interrupt_Init();
	//Drive_forward(0.5, 0.5);
	while(1)
	{
		/*
        Drive_forward(0.5, 0.5);
        _delay_ms(3000);
        Drive_backwards(0.9, 0.9);
        _delay_ms(3000);
        Rotate_clockwise(0.5, 0.5);
        _delay_ms(3000);
        Rotate_counter_clockwise(0.8, 0.8);
        _delay_ms(3000);*/
		while(data == 'f')
		{
			Drive_forward(0.5, 0.5);
		}
		while(data == 'b')
		{
			Drive_backwards(0.5, 0.5);
		}
		while(data == 'l')
		{
			Rotate_counter_clockwise(0.5, 0.5);
		}
		while(data == 'r')
		{
			Rotate_clockwise(0.5, 0.5);
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
