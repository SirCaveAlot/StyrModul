/*
 * Styrmodul_SirCave.c
 *
 * Created: 4/3/2017 1:51:03 PM
 *  Author: gusst967
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#include "PWM_SirCave.h"
#include "UART.h"
#include "SPI.h"
#include "Control.h"
#include "Sensor_values.h"

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

int main(void)
{	
	Timer1_init();
	Timer2_init();
	USART_Init(UBBR);
	Spi_init();
	DDRD = 0xFA;
	DDRA = 0xFF;
	DDRC |= 0b00000011;
	Center_grip_arm();
	UART_queue_put('A');
	UART_queue_put(0x00);
	UART_queue_put('f');
	UART_queue_put(0x00);
	_delay_ms(1000);
	sei();
	
	while(1)
	{ 		
// 		if(UART_queue_length() >= 2)
// 		{
// 			uint8_t data1;
// 			UART_queue_get(&data1);
// 			uint8_t data2;
// 			UART_queue_get(&data2);			
// 		}
		//Test_SPI_queue();
		
// 		if(UART_queue_peek(UART_queue_out) == 'A')
// 		{
// 			UART_queue_remove(); // remove current element.
// 			autonomous = !autonomous;
// 		}
// 		else
// 		{
// 			UART_queue_get(&mode); // Store in mode.
// 		} 
// 		if(Left_side_detectable())
// 		{
// 			PORTA |= (1 << 4);
// 		}
// 		else
// 		{
// 			PORTA &=
// 		}

		while(UART_queue_length > 1) // Dequeue_UART_queue not working, but this is....
		{
			uint8_t first_byte = UART_queue_peek(UART_queue_out);

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
		//Dequeue_UART_queue();
		Dequeue_SPI_queue(); // Load Sensor values from queue.
				
		if(autonomous) // Autonomous mode
		{
			if(mode == 'f')
			{
				Hallway_control(true);
			}
			else if(mode == 's')
			{
				Drive_forward(0, 0);
			}
		}
		else // Manual mode
		{
			if(mode == 'f')
			{
				Drive_forward(0.9, 0.9);
			}
			else if(mode == 'b')
			{
				Drive_backwards(0.9, 0.9);
			}
			else if(mode == 'l')
			{
				Rotate_counter_clockwise(0.9, 0.9);
			}
			else if(mode == 'r')
			{
				Rotate_clockwise(0.9, 0.9);
			}
			else if(mode == 's')
			{
				Drive_forward(0, 0);
			}
			else if(mode == 'o')
			{
				Open_grip_arm();
			}
			else if(mode == 'c')
			{
				Close_grip_arm();
			}
			else if(mode == 'm')
			{
				Center_grip_arm();
			}
			else if(mode == 't')
			{
				Rotate_LIDAR(0.5);
			}
			else if(mode == 'n')
			{
				Stop_LIDAR();
			}
		}
	}
}