/*
* DHTExpander.c
*
* Created: 20-Jul-21 2:39:59 PM
* Author : Leo
*/

/*
Translates from I2C to 16 DHT22/AM2302 sensors.
AM2301 sensors also kinda work, but they send additional zeroes.

General operation:
0. Blink LED 1 every 100 ms (shows operation)
1. Wait for input signal from I2C.
	Setting bit 0x01 via I2C in "register" 0x00 triggers a readout.
	Setting bit 0x02 via I2C in "register" 0x00 sets power to the DHTs on or off.
2. "Register" 0x00 is evaluated every ~100 ms.

Readout operation:
1. Read all DHTs (takes < 500 ms).
2. Write DHT result data (see below) to "registers" 0x01 - 0x61 (96 bytes)
3. Clear readout bit 0x01 from "register" 0x00, set readout finished bit 0x04.
4. Subsequent I2C read of registers 0x00-0x61 will return that data.

In general you probably want to write 0x03 to the status register to trigger a readout.
You'll then read 0x06 after the readout is finished.

To turn off the sensors, write 0x00, to turn them back on write 0x02.

LED 1 stops blinking during readout.

Data format:
16 groups of 6 bytes each = 96 bytes.
Each group consists of 5 bytes DHT data plus one byte for status code.
The status code is one of:
- 0x00 (success): DHT data is valid.
- 0x01 (E_DHT_NO_START_CONDITION): DHT did not respond with a start signal, DHT data is invalid.
- 0x02 (E_DHT_TIMEOUT): readout failed, got partial data, DHT data is invalid.

I2C behavior:
Register address auto-increment is implemented.
If data beyond the register file is requested in one transaction, 0xFE is returned.
If an address beyond the register file is requested, the address is set to 0x00 instead.

DHT code is mostly taken from https://github.com/fengcda/DHT_Sensor_AVR_Library.
Some changes were made to the timings, and pins are no longer hardcoded.

I2C implementation is mostly taken from https://rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc.

This is intended to run on some 8MHz ATmega8.
Running on 5V is recommended for DHTs with long cables.

*/

#ifndef F_CPU
# define F_CPU 8000000UL // Change timeout below!
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <string.h> // For memset.
#include "twislave.h"

// I2C slave address.
#define I2C_SLAVE_ADRESS 0x3D

// DHT readout error codes.
#define E_DHT_NO_START_CONDITION 1
#define E_DHT_TIMEOUT 2

// Status register flags.
#define BIT_READOUT 0x01
#define BIT_DHT_POWER 0x02
#define BIT_READOUT_FINISHED 0x04

// Timeout. Must be (F_CPU / 5000)
#define DHT_TIMEOUT 1600

// Buffer for DHT values.
static uint8_t dht_values[(5+1)*16];

// Resets the pin after reading from it by setting it to an input and enabling pull-up.
// This is for DHTs connected to port B.
void dht_finalize_pin_portb(uint8_t pin) {
	// Enable pull-up
	PORTB |= (1<<pin);
	// Set input
	DDRB &= ~(1<<pin);
}

// Resets the pin after reading from it by setting it to an input and enabling pull-up.
// This is for DHTs connected to port D.
void dht_finalize_pin_portd(uint8_t pin) {
	// Enable pull-up
	PORTD |= (1<<pin);
	// Set input
	DDRD &= ~(1<<pin);
}

// Sends the start signal and reads a response from a DHT.
// This is for DHTs connected to port B.
int8_t dht_read_portb(uint8_t pin, uint8_t dht_data[5]) {
	uint8_t bits[5]; // we always read 5 bytes from DHT
	uint8_t i,j = 0;
	
	// Calculate these because we need them all the time.
	// If we don't do this they'll be recalculated all the time and mess up our timings.
	uint8_t pin_mask = (1<<pin);
	uint8_t pin_mask_inverted = ~(1<<pin);

	//begin send request
	PORTB &= pin_mask_inverted; //low
	DDRD |= pin_mask; // output
	_delay_ms(5);
	PORTB |= pin_mask; //high
	DDRB &= pin_mask_inverted; //input
	_delay_us(40);

	//check first start condition
	if((PINB & pin_mask)) {
		return E_DHT_NO_START_CONDITION;
	}
	_delay_us(80);
	
	//check second start condition
	if(!(PINB & pin_mask)) {
		return E_DHT_NO_START_CONDITION;
	}
	_delay_us(80);

	//read-in data
	uint16_t timeoutcounter = 0;
	for (j=0; j<sizeof(bits); j++) { //for each byte (5 total)
		uint8_t result = 0;
		for(i=0; i<8; i++) {//for each bit in each byte (8 total)
			timeoutcounter = 0;
			while(!(PINB & pin_mask)) { //wait for an high input (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return E_DHT_TIMEOUT;
				}
			}
			_delay_us(30);
			if(PINB & pin_mask){
				result |= (1<<(7-i));
			}
			timeoutcounter = 0;
			while(PINB & pin_mask) {
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return E_DHT_TIMEOUT;
				}
			}
		}
		bits[j] = result;
	}

	for(i=0;i<sizeof(bits);i++) {
		dht_data[i] = bits[i];
	}

	return 0;
}

// Sends the start signal and reads a response from a DHT.
// This is for DHTs connected to port D.
int8_t dht_read_portd(uint8_t pin, uint8_t dht_data[5]) {
	uint8_t bits[5];	// we always read 5 bytes from DHT
	uint8_t i,j = 0;
	
	// Calculate these because we need them all the time.
	// If we don't do this they'll be recalculated all the time and mess up our timings.
	uint8_t pin_mask = (1<<pin);
	uint8_t pin_mask_inverted = ~(1<<pin);

	//begin send request
	PORTD &= pin_mask_inverted; //low
	DDRD |= pin_mask; // output
	_delay_ms(5);
	PORTD |= pin_mask; //high
	DDRD &= pin_mask_inverted; //input
	_delay_us(40);

	//check first start condition
	if((PIND & pin_mask)) {
		return E_DHT_NO_START_CONDITION;
	}
	_delay_us(80);
	
	//check second start condition
	if(!(PIND & pin_mask)) {
		return E_DHT_NO_START_CONDITION;
	}
	_delay_us(80);

	//read-in data
	uint16_t timeoutcounter = 0;
	for (j=0; j<sizeof(bits); j++) { //for each byte (5 total)
		uint8_t result = 0;
		for(i=0; i<8; i++) {//for each bit in each byte (8 total)
			timeoutcounter = 0;
			while(!(PIND & pin_mask)) { //wait for an high input (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return E_DHT_TIMEOUT;
				}
			}
			_delay_us(30);
			if(PIND & pin_mask){
				result |= (1<<(7-i));
			}
			timeoutcounter = 0;
			while(PIND & pin_mask) {
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return E_DHT_TIMEOUT;
				}
			}
		}
		bits[j] = result;
	}

	for(i=0;i<sizeof(bits);i++) {
		dht_data[i] = bits[i];
	}

	return 0;
}

// Initializes outputs.
static void io_init(){
	// Outputs (DHT power + 3 status LEDs)
	DDRC |= (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3);

	// Enable DHT power by default.
	PORTC |= (1<<PC0);

	// Make DHT pins inputs and enable pull-ups.
	// DHT line has to be high in idle mode.
	PORTB |= 0xFF;
	PORTD |= 0xFF;
	DDRB |= 0xFF;
	DDRD |= 0xFF;
}

// Helper for >255 ms delay.
static void long_delay(uint16_t ms)
{
	for(; ms>0; ms--) _delay_ms(1);
}

// Translates a DHT number (0-15) to a pin number on a port.
// If true is returned, the sensor is connected to port D.
// If false is returned, the sensor is connected to port B.
uint8_t bankd_and_pin_no_for_dht(uint8_t *pin, uint8_t dht_no) {
	*pin = dht_no % 8;
	if (dht_no < 8) {
		return 1;
	} else {
		return 0;
	}
}

// Reads all 16 DHTs.
// This reads out the sensors and then sets all pins to input+pull-up again.
// Interrupts are disabled during readout.
void read_all_dhts() {
	// Pin number of the current DHT
	uint8_t pin;
	// Whether the current DHT is on port D
	uint8_t port_d;
	// Number of the current DHT (0-15)
	uint8_t dht_no;
	// Some return value
	int8_t retval = 0;
	// Whether there were any errors (for any DHT).
	int8_t any_error = 0;

	// Indicate reading in progress.
	PORTC |= (1<<PC2);

	// Reset DHT values.
	memset(dht_values,0,sizeof(dht_values));

	// Disable interrupts (we need tight timings, but no clue if this is necessary).
	cli();

	// Read all sensors, one after another.
	// TODO if we want really high performance we could move the 1-10ms LOW out of this function into another preparation function and do that in parallel for all DHTs.
	for (dht_no=0; dht_no < 16; dht_no++) {
		port_d = bankd_and_pin_no_for_dht(&pin,dht_no);

		// Read sensor.
		// If this succeeds, the result is written to dht_values.
		if (port_d) {
			retval = dht_read_portd(pin, dht_values + dht_no*6);
		} else {
			retval = dht_read_portb(pin, dht_values + dht_no*6);
		}

		// In case of failure: write status code for this DHT.
		if (retval != 0) {
			dht_values[dht_no*6 + 5] = retval;
		}
		
		// Track whether anything went wrong, ever.
		any_error |= retval;
	}

	// Enable interrupts.
	sei();

	// Reset all DHT pins to inputs with pull-up.
	for (dht_no=0; dht_no < 16; dht_no++) {
		port_d = bankd_and_pin_no_for_dht(&pin,dht_no);

		if (port_d) {
			dht_finalize_pin_portd(pin);
			} else {
			dht_finalize_pin_portb(pin);
		}
	}

	// Indicate readout finished.
	PORTC &= ~(1<<PC2);

	// Indicate whether the readout was (partially) not successful.
	if (any_error) {
		PORTC |= (1<<PC3);
	}
}

int main(void)
{
	// Set inputs/outputs.
	io_init();
	// Clear DHT values.
	memset(dht_values, 0, sizeof(dht_values));

	// Clear I2C register buffer.
	for (int i=0; i<i2c_buffer_size; i++) {
		i2cdata[i] = 0;
	}
	// Set status register to have DHT power enabled.
	i2cdata[0] |= BIT_DHT_POWER;

	// Enable I2C.
	init_twi_slave(I2C_SLAVE_ADRESS);
	
	// Enable watchdog to restart if we didn't reset it for 2 seconds.
	wdt_enable(WDTO_2S);

	// Enable interrupts.
	sei();

	// Loop forever
	while (1)
	{
		// Wait.
		long_delay(100);
		// Blink LED 1.
		PORTC ^= (1<<PC1);

		// Set DHT power according to whatever is written to the status register.
		if ((i2cdata[0] & BIT_DHT_POWER)) {
			PORTC |= (1<<PC0);
		} else {
			PORTC &= ~(1<<PC0);
		}

		// If there is a request to read DHTs...
		if ((i2cdata[0] & BIT_READOUT)) {
			// Sleep a little bit in case I2C stuff is still happening (do we need this?)
			_delay_ms(10);

			// Reset debugging LEDs.
			PORTC &= ~((1<<PC2) | (1<<PC3));

			// Read sensors.
			read_all_dhts();

			// Copy results to I2C buffer for readout.
			for (int i=0; i<i2c_buffer_size-1;i++) {
				i2cdata[i+1] = dht_values[i];
			}

			// Set status byte accordingly
			i2cdata[0] &= ~BIT_READOUT;
			i2cdata[0] |= BIT_READOUT_FINISHED;
		}

		// Reset watchdog timer.
		wdt_reset();
	}
}

