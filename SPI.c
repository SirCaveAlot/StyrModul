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
#include "Control.h"


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
	
	SPI_queue_put(data); // Puts the received data on the queue.	
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

uint8_t SPI_queue_peek(uint8_t queue_index)
{
	return SPI_queue[queue_index];
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
	if(SPI_queue_length < 12)
	{
		dequeue = false;
		return;
	}
	
	cli();
	Start_dequeuing();
	if(dequeue)
	{
		uint8_t IR_value;
		uint8_t left_wheel;
		uint8_t right_wheel;
		uint8_t average_wheel;
		uint8_t gyro_high;
		uint8_t gyro_low;
		uint16_t gyro_data;
		
		SPI_queue_remove(); // Startbytes
		SPI_queue_remove();
		
		SPI_queue_get(&IR_value); // IR right
		IR_conversion(true, IR_value);
		SPI_queue_get(&IR_value); // IR left
		IR_conversion(false, IR_value);
		
		SPI_queue_remove(); // Right tape
		SPI_queue_remove(); // Left tape
		
		SPI_queue_get(&right_wheel); // Right wheel
		SPI_queue_get(&left_wheel); // Left wheel
		average_wheel = (left_wheel + right_wheel) * 10 * 0.5; // * 10 to avoid float
		Calculate_wheel_sensor_counter(average_wheel);
		
		SPI_queue_get(&gyro_high); // Gyro high
		gyro_data = gyro_high; 
		SPI_queue_get(&gyro_low); // Gyro low
		gyro_data = (gyro_data << 8) | gyro_low;
		Gyro_calculation(gyro_data);
		
		SPI_queue_remove(); // LIDAR high
		SPI_queue_remove(); // LIDAR low
		dequeue = false;
		update_control = true;
	}
	else
	{
		SPI_queue_remove();
	}
	sei();
}

void Start_dequeuing()
{
	uint8_t first_value = SPI_queue_peek(SPI_queue_out);
	uint8_t second_value = SPI_queue_peek(SPI_queue_out + 1);
	
	if(first_value == 0xFF && second_value == 0xFF)
	{
		uint8_t ninth_value = SPI_queue_peek(SPI_queue_out + 8);
		uint8_t tenth_value = SPI_queue_peek(SPI_queue_out + 9);
		
		if(ninth_value == 0xFF && tenth_value == 0xFF)
		{
			dequeue = false;
		}
		else
		{
			dequeue = true;
		}
	}
	else
	{
		dequeue = false;
	}
}

void Test_SPI_queue()
{
 	SPI_queue_put(0xFF);
 	SPI_queue_put(0xFF);
	SPI_queue_put(100);
	SPI_queue_put(80);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
	SPI_queue_put(40);
	SPI_queue_put(100);
}