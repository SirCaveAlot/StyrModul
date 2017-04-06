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
#include "UART.h"


//----------------Global variables-----------------------

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

// Received data
char mode = 'S';
uint8_t receiving_counter = 0;
uint16_t data_buffer = 0;

// Data for transmission
char mode_complete = 'D';
uint8_t transmission_counter = 0;


//-------------------Interrupts--------------------------

ISR(USART0_RX_vect)
{
	uint8_t data = UDR0;
	if(receiving_counter == 0)
	{
		mode = data; // Store the first byte of transmission in UDR0 in mode.
		receiving_counter = receiving_counter + 1;	
	}
	else if(data == 0x00)
	{
		receiving_counter = 0;
	}
	else if(receiving_counter == 1)
	{
		data_buffer = data;
		receiving_counter = receiving_counter + 1;
	}
	else if(receiving_counter == 2)
	{
		data_buffer = (data_buffer << 8);
		data_buffer = data_buffer | data;
		receiving_counter = receiving_counter + 1;
	}
}


//-------------------Initializations-------------------

void USART_Init(unsigned int baud)
{
	/* Set baud rate */
	UBRR0H = 0;
	UBRR0L = 7;
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // Receiver and transmitter enabled.
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = 0b00000110; // frame 
}

void Interrupt_Init()
{
	UCSR0B |= (1<<RXCIE0); // Enables receive interrupt
	sei(); // Remove later
}


//----------------------Functions-----------------------

void Data_transmission()
{
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
	UDR0 = mode_complete; // Puts the transmission data on the transmission register.
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
	UDR0 = 0x00; // Sends stopbyte
}
