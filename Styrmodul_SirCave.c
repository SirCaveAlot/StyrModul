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
#include "Modes.h"

#define clkspd 14745600
#define BAUD 115200
#define UBBR clkspd/16/BAUD-1

uint8_t last_movement;

void Test_set_speed()
{
	if(!update_control)
	{
		return;
	}
	float speed;
	speed = Set_speed();
	Drive_forward(speed, speed);
	update_control = false; 
}


void Course_simulation()
{
	if(mode == 's')
	{
		switch(last_movement)
		{
			case 'f':
			UART_queue_put(0);
			UART_queue_put('r');
			UART_queue_put(90);
			last_movement = 'r';
			break;
			
			case 'r':
			UART_queue_put(0);
			UART_queue_put('f');
			UART_queue_put(2);
			last_movement = 'f';
			break;
		}
	}
}


int main(void)
{	
	Timer1_init();
	Timer2_init();
	USART_Init(UBBR);
	Spi_init();
	Center_grip_arm();
	DDRD = 0xFA;
	DDRA = 0xFF;
	update_control = false;
	autonomous = false;
	mode_complete = true; 
	line_detected = false;
//	last_movement = 'r';
	mode = 's';
	competition_mode = 0;
	distance_until_stop = 0;
	travel_distance = 0;
	first_detection = false;
	after_right_turn = false;
	turn_around = false;
	

// 	UART_queue_put(0);
//  	UART_queue_put('A');
//  	UART_queue_put(0);
// 	UART_queue_put(0);
// 	UART_queue_put('l');
// 	UART_queue_put(90);
//  	Dequeue_UART_queue();
	
	//Test_UART_queue();
	sei();
	
	while(1)
	{
// 		if(mode == 's')
// 		{
// 			UART_queue_put(0);
// 			UART_queue_put('l');
// 			UART_queue_put(90);
// 		}
		
		Dequeue_UART_queue(); // Load UART data from communication module.
		Dequeue_SPI_queue(); // Load Sensor values from queue.
		Mode_loop();
		if(competition_mode == 1)
		{
			PORTA |= (1 << 5);
		}
		else if(competition_mode == 2)
		{
			PORTA |= (1 << 6);
		}
		
		if(left_side_detected)
		{
			PORTA |= 0b10000000;
		}
		else
		{
			PORTA &= 0b01101111;
		}
		if(right_side_detected)
		{
			//PORTA |= 0b10000000;
		}
		else
		{
			PORTA &= 0b11101111;
		}
	}
}