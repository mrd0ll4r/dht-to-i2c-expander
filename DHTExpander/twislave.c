/*
* twislave.c
*
* Created: 22-Jul-21 5:35:30 PM
* Author: Leo
* Source: https://rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc
*
* We support all three types of I2C transactions:
* - Pure write: One byte can be written to address 0x0.
*	This is the status register. Consult main.c for an explanation.
* - Pure read: 97 bytes can be read, starting from address 0x0.
*    The data consists of one status byte (at 0x0) followed by 16 groups of 6 bytes each.
*    The groups describe one DHT result each, consisting of 40 bits = 5 bytes read from the sensor plus 1 byte indicating success (0) or an error during readout.
* - Write+read: This is actually just a read from some given address (the one byte written).
*/

#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "twislave.h"

// The currently selected register address (within i2cdata) to be read from/written to.
volatile uint8_t buffer_addr;

void init_twi_slave(uint8_t addr)
{
	// I2C addresses are 7 bits! We have to shift.
	TWAR=(addr<<1);
	TWCR &= ~((1<<TWSTA)|(1<<TWSTO));
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE);
	buffer_addr=0xFF;
}

// Macros for TWI bitmasks

// ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

// NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

// Switch to the non-addressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);

// TWI interrupt service routine
ISR (TWI_vect)
{
	uint8_t data=0;

	// Check TWI status register
	switch (TW_STATUS)
	{
		// Slave Receiver

		// 0x60 Slave Receiver, Slave was addressed
		case TW_SR_SLA_ACK:
		// Receive next byte of data, send ACK afterwards
		TWCR_ACK;
		// Set "register address" to undefined
		buffer_addr=0xFF;
		break;
		
		// 0x80 Slave Receiver, a byte of data has been received
		case TW_SR_DATA_ACK:
		// Read received data
		data=TWDR;
		
		// First access of this transaction, set register address
		if (buffer_addr == 0xFF) {
			// First byte of this transaction
			// This specifies the register address, usually.
			if (data < i2c_buffer_size) {
				buffer_addr = data;
			}

			// Receive next byte, ACK afterwards to request next byte
			TWCR_ACK;

		} else {
			// Subsequent byte(s) of this transaction
			// We can now receive data and use it.
			// We only allow writing the status register 0x00.
			if(buffer_addr == 0x0)
			{
				// And we only allow 0x00 (no DHT power + no readout), 0x02 (no readout + DHT power), and 0x03 (DHT power + readout).
				if (data == 0x00 || data == 0x02 || data == 0x03) {
					// Write request to status register
					i2cdata[buffer_addr]=data;
				}
			}

			// Increase address for subsequent writes (which are not supported)
			buffer_addr++;

			// Receive next byte, ACK afterwards to request next byte
			TWCR_ACK;
		}
		break;


		//Slave transmitter

		//0xA8 Slave wurde im Lesemodus adressiert und hat ein ACK zurückgegeben.
		case TW_ST_SLA_ACK:
		// fallthrough

		// 0xB8 Slave Transmitter, data requested
		case TW_ST_DATA_ACK:

		if (buffer_addr == 0xFF)
		{
			// This is either a pure read transaction (no Write+Read) or something else went wrong.
			// We'll just assume they want to read everything...
			buffer_addr=0x00;
		}
		
		if(buffer_addr<i2c_buffer_size+1)
		{
			// Send one byte of data
			TWDR = i2cdata[buffer_addr];
			// Auto increment address
			buffer_addr++;
		} else {
			// Invalid address/read too many bytes/... -- we send 0xFE as an error
			TWDR=0xFE;
		}
		TWCR_ACK;
		break;

		case TW_SR_STOP:
		TWCR_ACK;
		break;
		case TW_ST_DATA_NACK: // 0xC0 Keine Daten mehr gefordert
		case TW_SR_DATA_NACK: // 0x88
		case TW_ST_LAST_DATA: // 0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
		default:
		TWCR_RESET;
		break;
		
	}
}