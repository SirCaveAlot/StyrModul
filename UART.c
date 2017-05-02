// ﻿/*
//  * UART.c
//  *
//  * Created: 3/31/2017 2:22:01 PM
//  *  Author: gusst967
//  */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include "PWM_SirCave.h"
#include "UART.h"
#include "Sensor_values.h"


//----------------Global variables-----------------------

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

// Received data
uint8_t mode;
volatile uint8_t receiving_counter = 0;
uint16_t data_buffer = 0;
bool mode_changed;

// Data for transmission
char mode_complete = 'd';
uint8_t transmission_counter = 0;

/* Queue structure */
#define UART_QUEUE_ELEMENTS 25
#define UART_QUEUE_SIZE (UART_QUEUE_ELEMENTS + 1) // maximum of element is QUEUE_ELEMENTS
volatile uint8_t UART_queue[UART_QUEUE_SIZE];
uint8_t UART_queue_in, UART_queue_out;
uint8_t UART_queue_length;

//-------------------Interrupts--------------------------

ISR(USART0_RX_vect)
{
	//PORTA |= (1 << PORTA5);
 	volatile uint8_t data = UDR0;
	 
	UART_queue_put(data);
// 	if(receiving_counter == 0)
// 	{
// 		if(data == 'A')
// 		{
// 			autonomous = !autonomous;
// 			mode = 's';
// 		}
// 		else
// 		{
// 			mode = data; // Store the first byte of transmission in UDR0 in mode.	
// 		}
// 		receiving_counter = receiving_counter + 1;
// 	}
// 	else if(data == 0x00)
// 	{
// 		receiving_counter = 0;
// 		//mode_changed = true;		
// 	}
// 	else if(receiving_counter == 1)
// 	{
// 		data_buffer = data;
// 		receiving_counter = receiving_counter + 1;
// 	}
// 	else if(receiving_counter == 2)
// 	{
// 		data_buffer = (data_buffer << 8);
// 		data_buffer = data_buffer | data;
// 		receiving_counter = receiving_counter + 1;
// 	}
	//PORTA &= ~(1 << PORTA5);
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
	UCSR0B |= (1<<RXCIE0); // Enables receive interrupt
	UART_queue_init();
}


//----------------------Functions-----------------------

void Data_transmission(char data)
{
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
	UDR0 = data; // Puts the transmission data on the transmission register.
// 	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
// 	UDR0 = 0x00; // Sends stopbyte
}


//------------------------Queue-------------------------

void UART_queue_init(void)
{
    UART_queue_in = UART_queue_out = 0;
}

void UART_queue_put(uint8_t new)
{
    if(UART_queue_in == ((UART_queue_out + UART_QUEUE_ELEMENTS) % UART_QUEUE_SIZE))
    {
        return; /* Queue Full*/
    }

    UART_queue[UART_queue_in] = new;

    UART_queue_in = (UART_queue_in + 1) % UART_QUEUE_SIZE;

   // return 0; // No errors
}

void UART_queue_get(uint8_t *old)
{
    if(UART_queue_in == UART_queue_out)
    {
        return; /* Queue Empty - nothing to get*/
    }

    *old = UART_queue[UART_queue_out];
	
	UART_queue[UART_queue_out] = 0;

	UART_queue_out = (UART_queue_out + 1) % UART_QUEUE_SIZE;

    //return 0; // No errors
}

uint8_t UART_queue_peek(uint8_t queue_index)
{
	return UART_queue[queue_index];
}

void UART_queue_remove()
{
	if(UART_queue_in == UART_queue_out)
	{
		return; // Queue Empty - nothing to remove
	}
	UART_queue[UART_queue_out] = 0;
	UART_queue_out = (UART_queue_out + 1) % UART_QUEUE_SIZE;
	UART_queue_length--;
}

// uint8_t UART_queue_length()
// {
// 	if(UART_queue_in == ((UART_queue_out + QUEUE_ELEMENTS) % QUEUE_SIZE))
// 	{
// 		return QUEUE_ELEMENTS;
// 	}
// 	else if(UART_queue_in == UART_queue_out)
// 	{
// 		return 0;
// 	}
// 	else if(UART_queue_out > UART_queue_in)
// 	{
// 		return QUEUE_SIZE - (UART_queue_out - UART_queue_in); 
// 	}
// 	else
// 	{
// 		return UART_queue_in - UART_queue_out;
// 	}
// }

void Dequeue_UART_queue()
{
	uint8_t first_byte = UART_queue_peek(UART_queue_out);
	//uint8_t second_byte = UART_queue_peek(UART_queue_out + 1);
	
	if(UART_queue_length < 2)
	{
		return;
	}
	
	if(first_byte == 'A')
	{
		autonomous = !autonomous;
		UART_queue_remove();
		UART_queue_remove();
	}
	else
	{
		UART_queue_get(&mode);
		UART_queue_remove();
	}
}

// void Start_dequeuing();
// {
// 
// 	
// 	if(second_byte == 0x00)
// 	{
// 		dequeue = true;
// 	}
// 	else
// 	{
// 		dequeue = false;
// 	}
// }