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
#include "Modes.h"


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
	UART_queue_length = 0;
}

void UART_queue_put(uint8_t new)
{
    if(UART_queue_in == ((UART_queue_out + UART_QUEUE_ELEMENTS) % UART_QUEUE_SIZE))
    {
        return; /* Queue Full*/
    }

    UART_queue[UART_queue_in] = new;
    UART_queue_in = (UART_queue_in + 1) % UART_QUEUE_SIZE;
	UART_queue_length++;
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
	UART_queue_length--;
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

void Dequeue_UART_queue()
{
	if(UART_queue_length < 3)
	{
		return;
	}
	
	uint8_t first_byte;
	UART_queue_get(&first_byte);
	
	if(first_byte == 0x00)
	{
		uint8_t second_byte = 0;
		uint8_t data = 0;
		UART_queue_get(&second_byte);
		UART_queue_get(&data);
		
		if(second_byte == 'A')
		{
			autonomous = !autonomous;
			return;
		}
		else if((second_byte == 'f') | (second_byte == 'b'))
		{
			Set_distance_until_stop(data);		
		}
		else if((second_byte == 'l') | (second_byte == 'r'))
		{
			Set_angle_to_rotate(data);
		}
		
		mode = second_byte; // more modes, create switch-case.
	}
}

void Test_UART_queue()
{
	UART_queue_put(0x00);
	UART_queue_put('A');
	UART_queue_put(0x00);
	
	UART_queue_put(0x00);
	UART_queue_put('f');
	UART_queue_put(3);
	
	UART_queue_put(0x00);
	UART_queue_put('r');
	UART_queue_put(90);
	
	UART_queue_put(0x00);
	UART_queue_put('l');
	UART_queue_put(90);
	
	UART_queue_put(0x00);
	UART_queue_put('L');
	UART_queue_put(0x00);
	
	UART_queue_put(0x00);
	UART_queue_put('b');
	UART_queue_put(7);
	
	UART_queue_put(0x00);
	UART_queue_put('A');
	UART_queue_put(0x00);
}