// ﻿/*
//  * UART.c
//  *
//  * Created: 3/31/2017 2:22:01 PM
//  * Author: Gustav Strandberg, gusst967
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


//----------------------------Global variables---------------------------------

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

//-----------------------------Queue structure---------------------------------

/* Very simple queue
 * These are FIFO queues which discard the new data when full.
 *
 * Queue is empty when in == out.
 * If in != out, then 
 *  - items are placed into in before incrementing in
 *  - items are removed from out before incrementing out
 * Queue is full when in == (out-1 + QUEUE_SIZE) % QUEUE_SIZE;
 *
 * The queue will hold QUEUE_ELEMENTS number of items before the
 * calls to QueuePut fail.
 */

#define UART_QUEUE_ELEMENTS 25
#define UART_QUEUE_SIZE (UART_QUEUE_ELEMENTS + 1)
volatile uint8_t UART_queue[UART_QUEUE_SIZE];
volatile uint8_t UART_queue_in, UART_queue_out;
volatile uint8_t UART_queue_length;


//-----------------------------------Interrupt---------------------------------

ISR(USART0_RX_vect)
{
 	volatile uint8_t data = UDR0;
	 
	if(data == 'B')
	{
		// 'B' is sent from the communications module if the line of the 
		// distressed is detected and the system is in the mapping stage.
		if(competition_mode)
		{
			distance_until_stop = 1000;
			stop_distance = 550;
			wheel_sensor_counter = 0;
			mode_complete = false;
			mode = 'b';
		}
	}
	else if(data == 'F')
	{
		// 'F' is sent from the communications module if the line of the
		// startposition is detected and the system is in the mapping stage.
		if(competition_mode)
		{
			Set_distance_until_stop(15);
			wheel_sensor_counter = 0;
			mode_complete = false;
			mode = 'f';
		}
	}
	else
	{
		UART_queue_put(data);
	}
}


//--------------------------------Initializations------------------------------

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


//-----------------------------------Functions---------------------------------

void UART_transmission(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmission register.
	UDR0 = data; // Puts the transmission data on the transmission register.
}

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

// Dequeues all UART values. Starts with a startbyte (0x00) and then all
// the UART data will come in the same order every time.
void Dequeue_UART_queue()
{
	// Dequeues if there is more than 2 bytes in the queue.
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
			
			// If the second byte is 'A', autonomous is toggled and parameters
			// are reseted. The third value will be removed.
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
			
			// If the second byte is 'C', competition_mode is toggled.
			// The third value will be removed.
			if(second_byte == 'C')
			{
				if(!competition_mode)
				{
					Rotate_LIDAR(0.2);
				}
				else if(competition_mode)
				{
					Stop_LIDAR();
					while(UART_queue_in != UART_queue_out)
					{
						UART_queue_remove();
					}
				}
				
				competition_mode = !competition_mode;
				mode_complete = true;
				UART_queue_remove();
				return;
			}
			
			// If autonomous, all modes will be the second_byte and the
			// data will be the third byte.
			if(mode_complete)
			{
				uint8_t data;
				UART_queue_get(&data);
				PORTA |= (data << 4);
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