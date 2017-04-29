/*
 * SPI.c
 *
 * Created: 4/3/2017 2:10:56 PM
 *  Author: gusst967
 */ 

// 
// void SPI_slave_init()
// {
// 	DDRB |= (1 << 6); // MISO as output, slave configuration.
// 	SPCR |= (1 << SPE) | (1 << SPIE); // SPI and SPI interrupt enabled.
// 	SPSR |= (0 << SPI2X);
// 	SPDR = 0x00; // Clear SPI interrupt flag by reading SPSR and SPDR.
// }
// 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "SPI.h"
#include "Sensor_values.h"


/* Queue structure */
#define SPI_QUEUE_ELEMENTS 40
#define SPI_QUEUE_SIZE (SPI_QUEUE_ELEMENTS + 1)
volatile uint8_t SPI_queue[SPI_QUEUE_SIZE];
uint8_t SPI_queue_in, SPI_queue_out;
uint8_t SPI_queue_length;
bool dequeue;

volatile uint8_t SPI_receiving_counter = 0;

//---------------------Interrupts--------------

ISR(SPI_STC_vect)
{
	volatile uint8_t data = SPDR;
	
	SPI_queue_put(data);
	
// 	if(data == 0x00 && SPI_receiving_counter == 0)
// 	{
// 		SPI_receiving_counter = 1;
// 	}
// 	else if(SPI_receiving_counter == 1)
// 	{
// 		IR_conversion(true, data);
// 		//Store_in_IR_array(true, data);
// 		SPI_receiving_counter = 2;
// 	}
// 	else if(SPI_receiving_counter == 2)
// 	{
// 		IR_conversion(false, data);
// 		//Store_in_IR_array(false, data);
// 		SPI_receiving_counter = 3;
// 	}
// 	else if(SPI_receiving_counter == 3)
// 	{
// 		// right tape sensor
// 		SPI_receiving_counter = 4;
// 	}
// 	else if(SPI_receiving_counter == 4)
// 	{
// 		// left tape sensor
// 		SPI_receiving_counter = 5;
// 	}
// 	else if(SPI_receiving_counter == 5)
// 	{
// 		// right wheel sensor
// 		SPI_receiving_counter = 6;
// 	}
// 	else if(SPI_receiving_counter == 6)
// 	{
// 		// left wheel sensor
// 		SPI_receiving_counter = 7;
// 	}
// 	else if(SPI_receiving_counter == 7)
// 	{
// 		gyro_rotation_speed = data;
// 		SPI_receiving_counter = 8;
// 	}
// 	else if(SPI_receiving_counter == 8)
// 	{
// 		// High byte LIDAR
// 		SPI_receiving_counter = 9;
// 	}
// 	else if(SPI_receiving_counter == 9)
// 	{
// 		// Low byte LIDAR
// 		SPI_receiving_counter = 10;
// 	}
// 	else if(SPI_receiving_counter == 10 && data == 0xFF)
// 	{
// 		SPI_receiving_counter = 0;
// 	}
	
// 	
// 	if(SPI_receiving_counter == 0)
// 	{
// 		IR_conversion_right(data);0
// 	}
// 	else if(SPI_receiving_counter == 1)
// 	{
// 		IR_conversion_left(data);
// 	}
// 	else if(data == 0x00)
// 	{
// 		SPI_receiving_counter = 0;
// 	}
// 	SPI_receiving_counter = SPI_receiving_counter + 1;
// 	if(left_right)
// 	{
// 		right_distance = SPDR;
// 		SPDR = 0xFF;
// 	}
// 	else
// 	{
// 		left_distance = SPDR;
// 		SPDR = 0x00;
// 	}
	
	//left_right = !left_right; 	
}



void Spi_init()
{
	DDRB=(1<<DDB6);               //MISO as OUTPUT
	SPCR=(1<<SPIE)|(1<<SPE);      //Enable SPI && interrupt enable bit
	SPDR=0;
	SPI_queue_init();
}


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


void SPI_queue_init()
{
	SPI_queue_in = SPI_queue_out = 0;
	SPI_queue_length = 0;
	dequeue = false;
}

void SPI_queue_put(uint8_t new)
{
	if(SPI_queue_in == ((SPI_queue_out + SPI_QUEUE_ELEMENTS) % SPI_QUEUE_SIZE))
	{
		return; /* Queue Full*/
	}

	SPI_queue[SPI_queue_in] = new;
	SPI_queue_in = (SPI_queue_in + 1) % SPI_QUEUE_SIZE;	
	SPI_queue_length++;
}

void SPI_queue_get(uint8_t *old)
{
	if(SPI_queue_in == SPI_queue_out)
	{
		return; /* Queue Empty - nothing to get*/
	}

	*old = SPI_queue[SPI_queue_out];	
	SPI_queue[SPI_queue_out] = 0;
	SPI_queue_out = (SPI_queue_out + 1) % SPI_QUEUE_SIZE;	
	SPI_queue_length--;
}

uint8_t SPI_queue_peek(uint8_t queue_place)
{
	return SPI_queue[queue_place];
}

void SPI_queue_remove()
{
	if(SPI_queue_in == SPI_queue_out)
	{
		return; // Queue Empty - nothing to remove
	}
	SPI_queue[SPI_queue_out] = 0;
	SPI_queue_out = (SPI_queue_out + 1) % SPI_QUEUE_SIZE;
	SPI_queue_length--;
}

// uint8_t SPI_queue_length()
// {
// 	if(SPI_queue_in == ((SPI_queue_out + SPI_QUEUE_ELEMENTS) % SPI_QUEUE_SIZE))
// 	{
// 		return SPI_QUEUE_ELEMENTS;
// 	}
// 	else if(SPI_queue_in == SPI_queue_out)
// 	{
// 		return 0;
// 	}
// 	else if(SPI_queue_out > SPI_queue_in)
// 	{
// 		return SPI_QUEUE_SIZE - (SPI_queue_out - SPI_queue_in);
// 	}
// 	else
// 	{
// 		return SPI_queue_in - SPI_queue_out;
// 	}
// }

void Dequeue_SPI_queue()
{	
	Start_dequeuing();
		
	if(SPI_queue_length < 11)
	{
		dequeue = false;
		return;
	}
	
	if(dequeue)
	{
		PORTA = 0b00010000;
		uint8_t IR_value;
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_get(&IR_value);
		IR_conversion(true, IR_value);
		SPI_queue_get(&IR_value);
		IR_conversion(false, IR_value);
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_remove();
		SPI_queue_remove();
		
		dequeue = false;
		PORTA = 0b00000000;
	}
	else
	{
		SPI_queue_remove();
	}
}

void Start_dequeuing()
{
	uint8_t first_value = SPI_queue_peek(SPI_queue_out);
	uint8_t second_value = SPI_queue_peek(SPI_queue_out + 1);
	
	if(first_value == 0xFF && second_value == 0xFF)
	{
		dequeue = true;
	}
	else
	{
		dequeue = false;
	}
}

void Test_SPI_queue()
{
// 	uint8_t data;
// 	SPI_queue_put(0xFF);
// 	SPI_queue_put(0xFF);
// 	
// 	for(uint8_t i = 1; i < 11; i++)
// 	{
// 		data = rand() / 200;
// 		SPI_queue_put(data);
// 	}

 	SPI_queue_put(0xFF);
 	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(30);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
}