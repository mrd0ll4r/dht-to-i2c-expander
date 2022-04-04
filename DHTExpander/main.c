/*
* DHTExpander.c
*
* Created: 20-Jul-21 2:39:59 PM
* Author : Leo & Falk
*/

/*
Translates from I2C to 16 DHT22/AM2302 sensors.
AM2301 sensors also kinda work, but they send additional zeros.

General operation:
0. Blink LED 1 (PC3) every 100 ms (shows operation)
1. Wait for input signal from I2C.
Setting bit 0 via I2C in "register" 0x00 triggers a readout.
Once readout is complete, bit reads as 0 again.

Bit 1 signals if no reset by the watchdog has occurred.
The initial state is high, meaning no watchdog reset.
If it gets cleared by the system it should be reset by the user
to detect the next watchdog reset which is unintended behavior.

Bits 2 and 3 correspond to output pins PC0 and PC1.
Setting or clearing these bits manipulates pin output.

Bits [4..7] describe how many sensors were read successfully last time.
Writing to these bits or any register other than 0x00 has no effect.
2. "Register" 0x00 is evaluated every ~100 ms.

Readout operation:
1. Read all DHTs (takes < 500 ms).
2. Write DHT result data (see below) to "registers" 0x01 - 0x61 (96 bytes)
3. Clear readout bit 0x01 from "register" 0x00.
4. Subsequent I2C read of registers 0x01-0x61 will return that data.

LED 2 lights up while readout is in progress.

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

Pin assignments
PORTB: DHTs on PCB terminals
PORTD: DHTs on pin headers
PC0, PC1: output pin headers (e.g. DHT power)
PC2: LED 2
PC3: LED 1

*/

#ifndef F_CPU
# define F_CPU 8000000UL
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
#define I2C_BIT_READOUT 0x01
#define I2C_BIT_WDT_RESET 0x02
#define I2C_BIT_PC0 0x04
#define I2C_BIT_PC1 0x08

// Timeout for DHT readout. Must be (F_CPU / 5000).
// Will be evaluated at compile time. Trust me.
#define DHT_TIMEOUT (F_CPU / 5000)

// Buffer for DHT values.
static uint8_t dht_values[(5+1)*16];


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
	DDRB |= pin_mask; // output
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
	// Output pins and LEDs. Values are set in main loop so no initialization necessary.
	DDRC |= (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3);

	// Make DHT pins inputs and enable pull-ups (not necessary due to external pull-ups).
	// DHT line has to be high in idle mode.
	DDRB = 0x00;
	DDRD = 0x00;
	PORTB = 0xFF;
	PORTD = 0xFF;
}


// Reads all 16 DHTs.
// This reads out the sensors and then sets all pins to input+pull-up again.
// Interrupts are disabled during readout.
// Returns the number of sensors that could be read successfully.
uint8_t read_all_dhts() {
	// Number of successful readouts.
	int8_t success_count = 0;
	// Status code of last DHT readout.
	int8_t code = 0;
	// Global sensor numbers €[0..15]
	uint8_t dht_no;
	// Pin number in bank €[0..7]
	uint8_t pin;

	// Reset DHT values.
	memset(dht_values,0,sizeof(dht_values));

	// Disable interrupts for tight timings.
	cli();

	// Read all sensors, one after another.
	// TODO if we want really high performance we could move the 1-10ms LOW out of this function into another preparation function and do that in parallel for all DHTs.
	for (dht_no=0; dht_no < 16; dht_no++) {
		pin = dht_no % 8;

		// Read sensor.
		// If this succeeds, the result is written to dht_values.
		if (dht_no < 8) {
			// Read sensors [0..7] from PORTD.
			code = dht_read_portd(pin, dht_values + dht_no*6);
			} else {
			// Read sensors [8..15] from PORTB.
			code = dht_read_portb(pin, dht_values + dht_no*6);
		}

		// In case of failure: write status code for this DHT.
		if (code != 0) {
			dht_values[dht_no*6 + 5] = code;
			}else{
			++success_count;
		}
	}
	
	// Enable interrupts.
	sei();

	return success_count;
}

// Set output pins according to status register.
static void synchronize_output_pins(void){
	if ((i2cdata[0] & I2C_BIT_PC0)) {
		PORTC |= (1<<PC0);
		} else {
		PORTC &= ~(1<<PC0);
	}

	if ((i2cdata[0] & I2C_BIT_PC1)) {
		PORTC |= (1<<PC1);
		} else {
		PORTC &= ~(1<<PC1);
	}
}

int main(void){

	// Set inputs/outputs.
	io_init();

	// Clear DHT values.
	memset(dht_values, 0, sizeof(dht_values));

	// Clear I2C register buffer.
	for (int i=0; i<i2c_buffer_size; i++) {
		i2cdata[i] = 0;
	}
	// 1 means no WDT reset occurred.
	i2cdata[0] |= I2C_BIT_WDT_RESET;
	
	// Enable watchdog to restart if we didn't reset it for 2 seconds.
	wdt_enable(WDTO_2S);
	// Enable I2C.
	init_twi_slave(I2C_SLAVE_ADRESS);
	// Enable Interrupts.
	sei();

	while (1)
	{
		_delay_ms(100);
		PORTC ^= (1<<PC3); // Blink LED 1.

		wdt_reset(); // Reset watchdog timer.
		if(MCUCSR & (1 << WDRF)){
			// A reset by the watchdog has occurred.
			// Signal this by clearing bit 2 in status byte.
			i2cdata[0] &= ~(I2C_BIT_WDT_RESET);
			// Clear flag for next time.
			MCUCSR &= ~(1<<WDRF);
		}

		// Set output pins according to status register.
		synchronize_output_pins();

		// If there is a request to read DHTs...
		if ((i2cdata[0] & I2C_BIT_READOUT)) {
			// Signal readout begins.
			PORTC |= (1<<PC2);
			
			// Read sensors and store number of successful readouts.
			i2cdata[0] &= 0x0F; // Clear relevant bits.
			i2cdata[0] |= (read_all_dhts() << 4);

			// Copy results to I2C buffer for readout.
			for (int i=0; i<i2c_buffer_size-1;i++) {
				i2cdata[i+1] = dht_values[i];
			}

			// Clear bit one in status byte
			i2cdata[0] &= ~I2C_BIT_READOUT;
			// Signal readout over.
			PORTC &= ~(1<<PC2);
		}
	}
}