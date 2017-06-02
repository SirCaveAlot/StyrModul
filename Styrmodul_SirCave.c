/*
 * Styrmodul_SirCave.c
 *
 * Created: 4/3/2017
 * Author: Gustav Strandberg, gusst967
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

//-----------------------------Definitions-------------------------------------
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

// A small simulation where the robot drives in a squared course of
// 3x3 modules.
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

// The main function of the steering module.
int main(void)
{	
	// All initializations.	
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
	mode = 's';
	competition_mode = false;
	distance_until_stop = 0;
	travel_distance = 0;
	first_detection = false;
	after_right_turn = false;
	turn_around = false;
	sei();
	
	while(1)
	{
		Dequeue_UART_queue(); // Load UART data from communication module.
		Dequeue_SPI_queue(); // Load sensor values from SPI queue.
		Mode_loop();
	}
}