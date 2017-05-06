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

int main(void)
{	
	Timer1_init();
	Timer2_init();
	USART_Init(UBBR);
	Spi_init();
	DDRD = 0xFA;
	DDRA = 0xFF;
	Center_grip_arm();
	update_control = false;
	autonomous = false;
	mode = 's';
	
	UART_queue_put(0);
 	UART_queue_put('A');
 	UART_queue_put(0);
	UART_queue_put(0);
 	UART_queue_put('r');
 	UART_queue_put(90);
	//Test_UART_queue();
	_delay_ms(1000);
	sei();
	
	while(1)
	{
		Dequeue_UART_queue(); // Load UART data from communication module.
		Dequeue_SPI_queue(); // Load Sensor values from queue.
		
		//Test_set_speed();
		Mode_loop();
		
// 		if(Standing_still())
// 		{
// 			UART_queue_put(0);
// 			UART_queue_put('l');
// 			UART_queue_put(90);
// 		}
	}
}