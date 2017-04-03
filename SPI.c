/*
 * SPI.c
 *
 * Created: 4/3/2017 2:10:56 PM
 *  Author: gusst967
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

void SPI_slave_init()
{
	DDRB |= (1 << 6); // MISO as output, slave configuration.
	SPCR |= (1 << SPE) | (1 << SPIE); // SPI and SPI interrupt enabled.
	SPSR |= (0 << SPI2X);
	SPDR = 0x00; // Clear SPI interrupt flag by reading SPSR and SPDR.
}

