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
#include "SPI.h"
#include "Sensor_values.h"


/* Queue structure */
#define QUEUE_ELEMENTS 25
#define QUEUE_SIZE (QUEUE_ELEMENTS + 1)
uint8_t Queue[QUEUE_SIZE];
int QueueIn, QueueOut;

bool left_right;

uint8_t SPI_receiving_counter = 0;

//---------------------Interrupts--------------

ISR(SPI_STC_vect)
{
	uint8_t data = SPDR;
	
	if(data == 0x00 && SPI_receiving_counter == 0)
	{
		SPI_receiving_counter = 1;
	}
	else if(SPI_receiving_counter == 1)
	{
		IR_conversion(true, data);
		//Store_in_IR_array(true, data);
		SPI_receiving_counter = 2;
	}
	else if(SPI_receiving_counter == 2)
	{
		IR_conversion(false, data);
		//Store_in_IR_array(false, data);
		SPI_receiving_counter = 3;
	}
	else if(SPI_receiving_counter == 3)
	{
		// right tape sensor
		SPI_receiving_counter = 4;
	}
	else if(SPI_receiving_counter == 4)
	{
		// left tape sensor
		SPI_receiving_counter = 5;
	}
	else if(SPI_receiving_counter == 5)
	{
		// right wheel sensor
		SPI_receiving_counter = 6;
	}
	else if(SPI_receiving_counter == 6)
	{
		// left wheel sensor
		SPI_receiving_counter = 7;
	}
	else if(SPI_receiving_counter == 7)
	{
		gyro_rotation_speed = data;
		SPI_receiving_counter = 8;
	}
	else if(SPI_receiving_counter == 8)
	{
		// High byte LIDAR
//		front_distance = data;
		SPI_receiving_counter = 9;
	}
	else if(SPI_receiving_counter == 9)
	{
		// Low byte LIDAR
// 		front_distance = front_distance << 8; //left shift 8 bits
// 		front_distance = front_distance + data; //add data
		SPI_receiving_counter = 10;
	}
	else if(SPI_receiving_counter == 10 && data == 0xFF)
	{
		SPI_receiving_counter = 0;
	}
	
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


void QueueInit(void)
{
    QueueIn = QueueOut = 0;
}

void QueuePut(uint8_t new)
{
    if(QueueIn == QueueOut && Queue[0] != 0)
    {
        return; /* Queue Full*/
    }

    Queue[QueueIn] = new;

    QueueIn = (QueueIn + 1) % QUEUE_SIZE;

   // return 0; // No errors
}

void QueueGet(uint8_t *old)
{
    if(QueueIn == QueueOut && Queue[0] == 0)
    {
        return; /* Queue Empty - nothing to get*/
    }

    *old = Queue[QueueOut];
	
	Queue[QueueOut] = 0;

	//int statement = QueueIn - QueueOut;
/*
    for(int i = 0; i < statement; i++)
	{
		Queue[QueueOut + i] = Queue[QueueOut + i + 1];
	}*/

	QueueOut = (QueueOut + 1) % QUEUE_SIZE;

    //return 0; // No errors
}
