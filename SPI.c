/*
 * SPI.c
 *
 * Created: 4/3/2017
 * Author: Gustav Strandberg, gusst967
 */  

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "SPI.h"
#include "Sensor_values.h"
#include "Control.h"
#include "Modes.h"

//---------------------------Queue structure-----------------------------------


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

#define SPI_QUEUE_ELEMENTS 40
#define SPI_QUEUE_SIZE (SPI_QUEUE_ELEMENTS + 1)
volatile uint8_t SPI_queue[SPI_QUEUE_SIZE];
uint8_t SPI_queue_in, SPI_queue_out;
uint8_t SPI_queue_length;
bool dequeue;


//------------------------------Interrupt--------------------------------------

ISR(SPI_STC_vect)
{
	if(autonomous)
	{
		volatile uint8_t data = SPDR;
		SPI_queue_put(data); // Puts the received data on the queue.	
	}
}

//------------------------------Functions--------------------------------------

void Spi_init()
{
	DDRB=(1<<DDB6); //MISO as OUTPUT
	SPCR=(1<<SPIE)|(1<<SPE); //Enable SPI && interrupt enable bit
	SPDR=0;
	SPI_queue_init();
}

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

// Dequeues all SPI values. Starts with two startbytes (0xFF) and then all
// the sensor data will come in the same order every time.
void Dequeue_SPI_queue()
{		
	// Start dequeuing if there is more than 14 bytes in the queue.
	if(SPI_queue_length < 15)
	{
		dequeue = false;
		return;
	}
	
	Start_dequeuing();
	if(dequeue)
	{
		uint8_t IR_value;
		uint8_t tape_data;
		uint8_t left_wheel;
		uint8_t right_wheel;
		uint8_t average_wheel;
		uint8_t gyro_high;
		uint8_t gyro_low;
		uint16_t gyro_data;
		
		SPI_queue_remove(); // Startbytes
		SPI_queue_remove();
		
		SPI_queue_get(&IR_value); // IR right front
		IR_conversion('r', IR_value);
		SPI_queue_get(&IR_value); // IR right back
		IR_conversion('b', IR_value);
		SPI_queue_get(&IR_value); // IR left front
		IR_conversion('l', IR_value);
		SPI_queue_get(&IR_value); // IR left back
		IR_conversion('w', IR_value);
		SPI_queue_get(&IR_value); // IR front
		IR_conversion('f', IR_value);
		
		SPI_queue_remove(); // Right tape
		SPI_queue_get(&tape_data); // Left tape
		Line_detection(tape_data);
		
		SPI_queue_get(&right_wheel); // Right wheel
		SPI_queue_get(&left_wheel); // Left wheel
		average_wheel = (left_wheel + right_wheel) * 10 * 0.5; // * 10 to avoid float
		Calculate_wheel_sensor_counter(average_wheel);
		
		SPI_queue_get(&gyro_high); // Gyro high
		gyro_data = gyro_high; 
		SPI_queue_get(&gyro_low); // Gyro low
		gyro_data = (gyro_data << 8) | gyro_low;
		Gyro_calculation(gyro_data);
		Angle_calculation();
		
		SPI_queue_remove(); // LIDAR high
		SPI_queue_remove(); // LIDAR low
		dequeue = false;
		update_control = true;
	}
	else
	{
		SPI_queue_remove();
	}
}

// Checks if the SPI data is in the correct order in the queue. If it is, the
// function returns true.
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