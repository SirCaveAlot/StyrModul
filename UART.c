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

// Data for transmission

/* Queue structure */
#define UART_QUEUE_ELEMENTS 25
#define UART_QUEUE_SIZE (UART_QUEUE_ELEMENTS + 1) // maximum of element is QUEUE_ELEMENTS
volatile uint8_t UART_queue[UART_QUEUE_SIZE];
uint8_t UART_queue_in, UART_queue_out;
uint8_t UART_queue_length;
uint8_t UART_counter;

//-------------------Interrupts--------------------------

ISR(USART0_RX_vect)
{
 	volatile uint8_t data = UDR0;
	if(data == 'B')
	{
		distance_until_stop = 1300;
		stop_distance = 550;
		wheel_sensor_counter = 0;
		mode_complete = false;
		mode = 'b';
	}
	else if(data == 'F')
	{
		Set_distance_until_stop(15);
		wheel_sensor_counter = 0;
		mode_complete = false;
		mode = 'f';
	}
	else
	{
		UART_queue_put(data);
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
	UCSR0B |= (1<<RXCIE0); // Enables receive interrupt
	UART_queue_init();
}


//----------------------Functions-----------------------

void UART_transmission(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
	UDR0 = data; // Puts the transmission data on the transmission register.
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

void Clear_UART_queue()
{
	while(UART_queue_in != UART_queue_out)
	{
		return;
	}
	
}

void Dequeue_UART_queue()
{
	if(UART_queue_length < 3)
	{
		return;
	}
	
	if(autonomous)
	{
		uint8_t first_byte;
		UART_queue_get(&first_byte);
		
		if(first_byte == 0)
		{
			uint8_t second_byte = UART_queue_peek(UART_queue_out);
			
			if(second_byte == 0)
			{
				return;
			}
			
			UART_queue_get(&second_byte);
			
			if(second_byte == 'A')
			{
				autonomous = false;
				mode_complete = true;
				mode = 's';
				wheel_sensor_counter = 0;
				standing_still_counter = 0;
				distance_until_stop = 0;
				travel_distance = 0;
				angle = 0;
				angle_to_rotate = 0;
				UART_queue_remove();
				return;
			}
			
			if(second_byte == 'C')
			{
				Rotate_LIDAR(0.2);
				if(competition_mode == 0)
				{
					competition_mode = 1;
				}
				else if(competition_mode == 1)
				{
					while(UART_queue_in != UART_queue_out)
					{
						UART_queue_remove();
					}
					competition_mode = 0;
				}
				
				mode_complete = true;
				UART_queue_remove();
				return;
			}
			
			if(mode_complete)
			{
				uint8_t data;
				UART_queue_get(&data);
				if(data == 180)
				{
					data = 230;
					turn_around = true;
				}
				else
				{
					turn_around = false;
				}
				
				if((second_byte == 'f') || (second_byte == 'b'))
				{
					mode_complete = false;
					Set_distance_until_stop(data);
				}
				else if((second_byte == 'l') || (second_byte == 'r'))
				{
					mode_complete = false;
					angle = 0;
					Set_angle_to_rotate(data);
					Set_rotation_distance(data);
				}
				
				mode = second_byte;
			}
		}
	}
	else // Manual mode
	{
		uint8_t first_byte;
		UART_queue_get(&first_byte);
		
		if(first_byte == 0)
		{
			uint8_t second_byte = UART_queue_peek(UART_queue_out);
			
			if(second_byte == 0)
			{
				return;
			}
			
			UART_queue_get(&second_byte);
			
			UART_queue_remove();
			
			if(second_byte == 'A')
			{
				autonomous = true;
				return;
			}
			
			mode = second_byte;
		}
	}
}
	
	
	
	
	
// 	uint8_t first_byte;
// 	UART_queue_get(&first_byte);
// 	
// 	if(first_byte == 0x00)
// 	{	
// 		uint8_t next_data;
// 		next_data = UART_queue_peek(UART_queue_out);
// 			
// 		if(next_data == 0x00)
// 		{
// 			return;
// 		}
// 		
// 		uint8_t second_byte;
// 		UART_queue_get(&second_byte);
// 		//PORTA = (second_byte | 0x0F);
// 		
// 		if(second_byte == 'A')
// 		{
// 			autonomous = !autonomous;
// 			if(autonomous)
// 			{
// 				PORTA |= (1 << PORTA0);
// 			}
// 			else
// 			{
// 				PORTA &= ~(1 << PORTA0);
// 			}
// 			mode = 's';
// 			UART_queue_remove();
// 			return;
// 		}
// 		
// 		if(!autonomous)
// 		{
// 			mode = second_byte;
// 			UART_queue_remove();
// 			return;
// 		}
// 		
// 		if(!mode_complete)
// 		{
// 			UART_queue_remove();
// 			return;
// 		}
// 		/*mode = second_byte;*/
// 		
// 		uint8_t data;
// 		UART_queue_get(&data);
// 		
// 		switch(second_byte)
// 		{
// 			case 'f':
// 			mode = 'f';
// 			mode_complete = false;
// 			Set_distance_until_stop(data);
// 			break;
// 			
// 			case 'b':
// 			mode = 'b';
// 			mode_complete = false;
// 			Set_distance_until_stop(data);
// 			break;
// 			
// 			case 'r':
// 			mode = 'r';
// 			mode_complete = false;
// 			Set_rotation_distance(data);
// 			break; 
// 			
// 			case 'l':
// 			mode = 'l';
// 			mode_complete = false;
// 			Set_rotation_distance(data);
// 			break;
// 		}
// 	}
// }

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